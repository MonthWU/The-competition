#!/usr/bin/env python3
"""Interactive OpenCV illumination-compensation and threshold tuner."""

from __future__ import annotations

import base64
import json
import os
import time
from typing import Any

import cv2
import numpy as np
from flask import Flask, jsonify, render_template_string, request


app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 15 * 1024 * 1024

PARAM_SPECS: dict[str, tuple[float, float, float, type]] = {
    "max_width": (320, 1600, 960, int),
    "roi_x": (0, 99, 0, int),
    "roi_y": (0, 99, 0, int),
    "roi_width": (1, 100, 100, int),
    "roi_height": (1, 100, 100, int),
    "denoise_kernel": (1, 15, 3, int),
    "illumination_kernel": (3, 201, 81, int),
    "compensation_strength": (0, 100, 100, int),
    "gamma": (0.2, 3.0, 1.0, float),
    "threshold": (0, 255, 150, int),
    "morph_kernel": (1, 15, 3, int),
    "morph_iterations": (0, 5, 1, int),
    "h_min": (0, 179, 35, int),
    "h_max": (0, 179, 95, int),
    "s_min": (0, 255, 50, int),
    "s_max": (0, 255, 255, int),
    "v_min": (0, 255, 30, int),
    "v_max": (0, 255, 255, int),
    "canny_low": (0, 255, 50, int),
    "canny_high": (0, 255, 150, int),
    "canny_aperture": (3, 7, 3, int),
    "contour_min_area": (0, 200000, 100, int),
    "contour_max_area": (100, 1000000, 200000, int),
    "contour_epsilon": (0, 10, 1.0, float),
    "contour_thickness": (1, 10, 2, int),
    "max_objects": (1, 50, 20, int),
    "center_marker_size": (3, 31, 11, int),
    "center_marker_thickness": (1, 5, 2, int),
    "circularity_threshold": (0.5, 1.0, 0.82, float),
    "measurement_decimals": (0, 3, 1, int),
    "measurement_font_scale": (0.3, 1.5, 0.55, float),
}
ODD_PARAMETERS = {
    "denoise_kernel",
    "illumination_kernel",
    "morph_kernel",
    "canny_aperture",
    "center_marker_size",
}


def _clamp_parameters(raw: dict[str, Any]) -> dict[str, int | float | bool | str]:
    parsed: dict[str, int | float | bool | str] = {}
    for name, (minimum, maximum, default, caster) in PARAM_SPECS.items():
        try:
            value = caster(raw.get(name, default))
        except (TypeError, ValueError):
            value = caster(default)
        value = max(caster(minimum), min(caster(maximum), value))
        if name in ODD_PARAMETERS and int(value) % 2 == 0:
            value = int(value) + 1 if value < maximum else int(value) - 1
        parsed[name] = value
    invert = raw.get("invert", False)
    parsed["invert"] = invert is True or str(invert).lower() in {"1", "true", "yes", "on"}
    contour_source = str(raw.get("contour_source", "hsv"))
    parsed["contour_source"] = contour_source if contour_source in {"binary", "hsv", "canny"} else "hsv"
    center_method = str(raw.get("center_method", "pixel"))
    parsed["center_method"] = center_method if center_method in {"pixel", "minrect", "bbox", "circle"} else "pixel"
    measurement_mode = str(raw.get("measurement_mode", "auto"))
    parsed["measurement_mode"] = (
        measurement_mode if measurement_mode in {"auto", "rectangle", "circle", "polygon"} else "auto"
    )
    return parsed


def _make_default_image(width: int = 960, height: int = 600) -> np.ndarray:
    image = np.full((height, width, 3), (235, 235, 235), dtype=np.uint8)
    cv2.rectangle(image, (105, 115), (310, 300), (45, 85, 210), -1)
    cv2.circle(image, (485, 205), 100, (45, 175, 85), -1)
    triangle = np.array([[690, 305], [825, 105], [915, 315]], dtype=np.int32)
    cv2.fillPoly(image, [triangle], (190, 75, 55))
    cv2.ellipse(image, (285, 455), (145, 70), 0, 0, 360, (60, 60, 60), -1)
    cv2.line(image, (530, 410), (865, 510), (35, 35, 35), 22, cv2.LINE_AA)

    yy, xx = np.mgrid[0:height, 0:width]
    horizontal = 1.04 - 0.42 * (xx / max(width - 1, 1))
    shadow = 1.0 - 0.34 * np.exp(
        -(((xx - width * 0.62) / (width * 0.23)) ** 2 + ((yy - height * 0.58) / (height * 0.28)) ** 2)
    )
    lighting = np.clip(horizontal * shadow, 0.42, 1.08)[..., None]
    return np.clip(image.astype(np.float32) * lighting, 0, 255).astype(np.uint8)


def _decode_upload() -> np.ndarray:
    uploaded = request.files.get("image")
    if uploaded is None or not uploaded.filename:
        return _make_default_image()
    data = uploaded.read()
    if not data:
        raise ValueError("上传文件为空")
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("无法解码图片，请上传 JPG、PNG、BMP 或 WebP")
    return image


def _resize_for_preview(image: np.ndarray, max_width: int) -> np.ndarray:
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    scale = max_width / width
    return cv2.resize(image, (max_width, max(1, round(height * scale))), interpolation=cv2.INTER_AREA)


def _contour_center(contour: np.ndarray, method: str) -> tuple[float, float]:
    if method == "minrect":
        center, _size, _angle = cv2.minAreaRect(contour)
        return float(center[0]), float(center[1])
    if method == "bbox":
        x, y, width, height = cv2.boundingRect(contour)
        return x + (width - 1) / 2.0, y + (height - 1) / 2.0
    if method == "circle":
        center, _radius = cv2.minEnclosingCircle(contour)
        return float(center[0]), float(center[1])

    x, y, width, height = cv2.boundingRect(contour)
    local_mask = np.zeros((height, width), dtype=np.uint8)
    shifted = contour - np.array([[[x, y]]], dtype=contour.dtype)
    cv2.drawContours(local_mask, [shifted], -1, 255, cv2.FILLED)
    pixels_y, pixels_x = np.nonzero(local_mask)
    if pixels_x.size == 0:
        return x + (width - 1) / 2.0, y + (height - 1) / 2.0
    return x + float(np.mean(pixels_x)), y + float(np.mean(pixels_y))


def _measure_contour(
    contour: np.ndarray,
    mode: str,
    epsilon_percent: float,
    circularity_threshold: float,
    decimals: int,
) -> dict[str, Any]:
    perimeter = float(cv2.arcLength(contour, True))
    area = float(cv2.contourArea(contour))
    epsilon = max(0.0001, epsilon_percent) * perimeter
    polygon = cv2.approxPolyDP(contour, epsilon, True)
    circularity = 4.0 * float(np.pi) * area / (perimeter * perimeter) if perimeter > 0 else 0.0

    selected_mode = mode
    if mode == "auto":
        if circularity >= circularity_threshold:
            selected_mode = "circle"
        elif len(polygon) == 4:
            selected_mode = "rectangle"
        else:
            selected_mode = "polygon"

    if selected_mode == "circle":
        _center, radius = cv2.minEnclosingCircle(contour)
        diameter = round(float(radius) * 2.0, decimals)
        return {
            "shape": "圆形",
            "label": f"D={diameter}px",
            "measurement": f"直径 {diameter} px",
            "circularity": round(circularity, 3),
        }

    if selected_mode == "rectangle":
        _center, (side_a, side_b), _angle = cv2.minAreaRect(contour)
        short_side, long_side = sorted((float(side_a), float(side_b)))
        short_side, long_side = round(short_side, decimals), round(long_side, decimals)
        return {
            "shape": "矩形",
            "label": f"L={long_side} S={short_side}px",
            "measurement": f"长边 {long_side} px；短边 {short_side} px",
            "circularity": round(circularity, 3),
        }

    points = polygon.reshape(-1, 2).astype(np.float32)
    if len(points) >= 2:
        edge_lengths = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
        rounded_edges = [round(float(length), decimals) for length in edge_lengths]
    else:
        rounded_edges = []
    preview_edges = ",".join(str(length) for length in rounded_edges[:4])
    if len(rounded_edges) > 4:
        preview_edges += ",..."
    all_edges = "、".join(f"{length} px" for length in rounded_edges) or "无法计算"
    return {
        "shape": f"{len(points)}边形",
        "label": f"E={preview_edges}",
        "measurement": f"边长 {all_edges}",
        "circularity": round(circularity, 3),
    }


def process_image(
    image: np.ndarray, params: dict[str, int | float | bool | str]
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    source_height, source_width = image.shape[:2]
    image = _resize_for_preview(image, int(params["max_width"]))
    image_height, image_width = image.shape[:2]
    source_scale_x = source_width / image_width
    source_scale_y = source_height / image_height
    x1 = min(image_width - 1, round(image_width * int(params["roi_x"]) / 100))
    y1 = min(image_height - 1, round(image_height * int(params["roi_y"]) / 100))
    requested_x2 = round(image_width * (int(params["roi_x"]) + int(params["roi_width"])) / 100)
    requested_y2 = round(image_height * (int(params["roi_y"]) + int(params["roi_height"])) / 100)
    x2 = max(x1 + 1, min(image_width, requested_x2))
    y2 = max(y1 + 1, min(image_height, requested_y2))

    original_with_roi = image.copy()
    cv2.rectangle(original_with_roi, (x1, y1), (x2 - 1, y2 - 1), (0, 255, 255), 3)
    roi = image[y1:y2, x1:x2].copy()
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    denoise_kernel = int(params["denoise_kernel"])
    working = gray if denoise_kernel == 1 else cv2.GaussianBlur(gray, (denoise_kernel, denoise_kernel), 0)
    illumination_kernel = int(params["illumination_kernel"])
    background = cv2.GaussianBlur(working, (illumination_kernel, illumination_kernel), 0)
    safe_background = np.maximum(background, 1)
    scale = float(np.mean(background))
    corrected = cv2.divide(working, safe_background, scale=scale)

    strength = float(params["compensation_strength"]) / 100.0
    compensated = cv2.addWeighted(working, 1.0 - strength, corrected, strength, 0)
    gamma = float(params["gamma"])
    if abs(gamma - 1.0) > 1e-6:
        lookup = np.array([((index / 255.0) ** gamma) * 255 for index in range(256)], dtype=np.uint8)
        compensated = cv2.LUT(compensated, lookup)

    threshold_type = cv2.THRESH_BINARY_INV if bool(params["invert"]) else cv2.THRESH_BINARY
    _, binary = cv2.threshold(compensated, int(params["threshold"]), 255, threshold_type)
    morph_iterations = int(params["morph_iterations"])
    if morph_iterations > 0:
        morph_kernel = int(params["morph_kernel"])
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=morph_iterations)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    s_min, s_max = sorted((int(params["s_min"]), int(params["s_max"])))
    v_min, v_max = sorted((int(params["v_min"]), int(params["v_max"])))
    lower_sv = (s_min, v_min)
    upper_sv = (s_max, v_max)
    h_min, h_max = int(params["h_min"]), int(params["h_max"])
    if h_min <= h_max:
        hsv_mask = cv2.inRange(hsv, (h_min, *lower_sv), (h_max, *upper_sv))
    else:
        mask_high = cv2.inRange(hsv, (h_min, *lower_sv), (179, *upper_sv))
        mask_low = cv2.inRange(hsv, (0, *lower_sv), (h_max, *upper_sv))
        hsv_mask = cv2.bitwise_or(mask_high, mask_low)
    if morph_iterations > 0:
        hsv_mask = cv2.morphologyEx(hsv_mask, cv2.MORPH_CLOSE, kernel, iterations=morph_iterations)

    canny_low, canny_high = sorted((int(params["canny_low"]), int(params["canny_high"])))
    canny = cv2.Canny(
        compensated,
        canny_low,
        canny_high,
        apertureSize=int(params["canny_aperture"]),
        L2gradient=True,
    )

    masks = {"binary": binary, "hsv": hsv_mask, "canny": canny}
    contour_mask = masks[str(params["contour_source"])]
    found_contours, _ = cv2.findContours(contour_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area, max_area = sorted((float(params["contour_min_area"]), float(params["contour_max_area"])))
    filtered_contours = [contour for contour in found_contours if min_area <= cv2.contourArea(contour) <= max_area]
    filtered_contours.sort(key=cv2.contourArea, reverse=True)
    displayed_contours = filtered_contours[: int(params["max_objects"])]
    contour_preview = roi.copy()
    epsilon_percent = float(params["contour_epsilon"]) / 100.0
    drawable_contours = []
    for contour in displayed_contours:
        if epsilon_percent > 0:
            epsilon = epsilon_percent * cv2.arcLength(contour, True)
            contour = cv2.approxPolyDP(contour, epsilon, True)
        drawable_contours.append(contour)
    cv2.drawContours(contour_preview, drawable_contours, -1, (0, 255, 0), int(params["contour_thickness"]))

    center_preview = roi.copy()
    measurement_preview = roi.copy()
    cv2.drawContours(center_preview, drawable_contours, -1, (0, 255, 0), int(params["contour_thickness"]))
    cv2.drawContours(measurement_preview, drawable_contours, -1, (0, 255, 0), int(params["contour_thickness"]))
    objects: list[dict[str, Any]] = []
    marker_size = int(params["center_marker_size"])
    marker_thickness = int(params["center_marker_thickness"])
    font_scale = float(params["measurement_font_scale"])
    decimals = int(params["measurement_decimals"])
    for index, contour in enumerate(displayed_contours, start=1):
        center_x, center_y = _contour_center(contour, str(params["center_method"]))
        center_point = (round(center_x), round(center_y))
        cv2.drawMarker(
            center_preview,
            center_point,
            (0, 0, 255),
            cv2.MARKER_CROSS,
            marker_size,
            marker_thickness,
            cv2.LINE_AA,
        )
        cv2.circle(center_preview, center_point, max(2, marker_size // 3), (255, 0, 255), marker_thickness)
        cv2.putText(
            center_preview,
            f"#{index} ({center_point[0]},{center_point[1]})",
            (max(0, center_point[0] + 6), max(14, center_point[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

        measurement = _measure_contour(
            contour,
            str(params["measurement_mode"]),
            epsilon_percent,
            float(params["circularity_threshold"]),
            decimals,
        )
        bounds_x, bounds_y, _bounds_width, _bounds_height = cv2.boundingRect(contour)
        label_y = bounds_y - 8 if bounds_y >= 18 else bounds_y + 18
        cv2.putText(
            measurement_preview,
            f"#{index} {measurement['label']}",
            (max(0, bounds_x), label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 180, 255),
            max(1, int(params["contour_thickness"])),
            cv2.LINE_AA,
        )
        objects.append(
            {
                "index": index,
                "area": round(float(cv2.contourArea(contour)), decimals),
                "center_roi": [round(center_x, decimals), round(center_y, decimals)],
                "center_image": [
                    round((center_x + x1) * source_scale_x, decimals),
                    round((center_y + y1) * source_scale_y, decimals),
                ],
                **measurement,
            }
        )

    white_ratio = float(np.count_nonzero(binary)) / float(binary.size) * 100.0
    hsv_ratio = float(np.count_nonzero(hsv_mask)) / float(hsv_mask.size) * 100.0
    edge_ratio = float(np.count_nonzero(canny)) / float(canny.size) * 100.0
    return (
        {
            "original": original_with_roi,
            "roi": roi,
            "gray": gray,
            "compensated": compensated,
            "binary": binary,
            "hsv": hsv_mask,
            "canny": canny,
            "contours": contour_preview,
            "centers": center_preview,
            "measurements": measurement_preview,
        },
        {
            "width": image_width,
            "height": image_height,
            "source_width": source_width,
            "source_height": source_height,
            "roi_width": roi.shape[1],
            "roi_height": roi.shape[0],
            "white_ratio": round(white_ratio, 2),
            "hsv_ratio": round(hsv_ratio, 2),
            "edge_ratio": round(edge_ratio, 2),
            "contour_count": len(filtered_contours),
            "displayed_count": len(displayed_contours),
            "objects": objects,
        },
    )


def _png_data_url(image: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("图片编码失败")
    return "data:image/png;base64," + base64.b64encode(encoded).decode("ascii")


@app.get("/")
def index() -> str:
    return render_template_string(PAGE)


@app.post("/api/process")
def process_api():
    started = time.perf_counter()
    try:
        raw_params = json.loads(request.form.get("params", "{}"))
        if not isinstance(raw_params, dict):
            raise ValueError("参数格式错误")
        params = _clamp_parameters(raw_params)
        stages, stats = process_image(_decode_upload(), params)
        stats["processing_ms"] = round((time.perf_counter() - started) * 1000, 1)
        return jsonify(
            ok=True,
            images={name: _png_data_url(image) for name, image in stages.items()},
            params=params,
            stats=stats,
        )
    except (ValueError, json.JSONDecodeError) as exc:
        return jsonify(ok=False, error=str(exc)), 400
    except Exception:
        app.logger.exception("Image processing failed")
        return jsonify(ok=False, error="处理失败，请检查图片和参数"), 500


@app.errorhandler(413)
def too_large(_error):
    return jsonify(ok=False, error="图片超过 15 MB 限制"), 413


PAGE = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OpenCV 综合视觉调参器</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f3f6f8; --panel: #ffffff; --text: #17202a; --muted: #62707d;
      --border: #d9e1e7; --accent: #1677ff; --accent-soft: #eaf3ff; --danger: #b42318;
      --shadow: 0 10px 30px rgba(25, 42, 62, .08);
    }
    * { box-sizing: border-box; }
    body { margin: 0; background: var(--bg); color: var(--text); font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }
    button, input { font: inherit; }
    .shell { max-width: 1500px; margin: auto; padding: 22px; }
    header { display: flex; gap: 16px; justify-content: space-between; align-items: flex-end; flex-wrap: wrap; margin-bottom: 16px; }
    h1 { margin: 0; font-size: clamp(22px, 3vw, 32px); font-weight: 650; }
    .subtitle { margin: 4px 0 0; color: var(--muted); }
    .toolbar { display: flex; gap: 9px; flex-wrap: wrap; }
    .btn, .upload-label { border: 1px solid var(--border); background: var(--panel); color: var(--text); border-radius: 10px; padding: 9px 13px; cursor: pointer; }
    .btn:hover, .upload-label:hover { border-color: var(--accent); }
    .upload-label { background: var(--accent); border-color: var(--accent); color: white; }
    #imageInput { position: absolute; inline-size: 1px; block-size: 1px; opacity: 0; }
    .layout { display: grid; grid-template-columns: minmax(280px, 350px) minmax(0, 1fr); gap: 16px; align-items: start; }
    .panel { background: var(--panel); border: 1px solid var(--border); border-radius: 14px; box-shadow: var(--shadow); }
    aside { padding: 16px; }
    h2 { margin: 0 0 12px; font-size: 18px; }
    details { border-top: 1px solid var(--border); }
    details:first-of-type { border-top: 0; }
    summary { padding: 12px 0; cursor: pointer; font-weight: 650; }
    .group-body { padding: 0 0 10px; }
    .control { padding: 10px 0; border-top: 1px solid var(--border); }
    .control:first-of-type { border-top: 0; }
    .control-head { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 7px; }
    .control-head label { font-weight: 600; }
    .number { width: 88px; border: 1px solid var(--border); border-radius: 8px; padding: 6px 8px; color: var(--text); background: var(--panel); }
    .select { width: 100%; border: 1px solid var(--border); border-radius: 8px; padding: 8px; color: var(--text); background: var(--panel); }
    input[type="range"] { width: 100%; accent-color: var(--accent); }
    .hint { color: var(--muted); font-size: 12px; margin-top: 3px; }
    .switch { display: flex; gap: 10px; align-items: center; padding-top: 12px; border-top: 1px solid var(--border); }
    .switch input { width: 18px; height: 18px; accent-color: var(--accent); }
    .check-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .check-item { display: flex; align-items: center; gap: 7px; }
    .check-item input { width: 17px; height: 17px; accent-color: var(--accent); }
    .mini-actions { display: flex; gap: 8px; margin-top: 10px; }
    .mini-actions .btn { flex: 1; padding: 6px 8px; }
    main { min-width: 0; }
    .status { display: flex; gap: 8px 18px; flex-wrap: wrap; padding: 12px 14px; margin-bottom: 16px; color: var(--muted); }
    .status strong { color: var(--text); font-weight: 650; }
    #message.error { color: var(--danger); }
    .previews { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; }
    figure { margin: 0; overflow: hidden; }
    figcaption { padding: 11px 13px; display: flex; justify-content: space-between; gap: 8px; border-bottom: 1px solid var(--border); font-weight: 650; }
    .stage { color: var(--accent); }
    .image-wrap { min-height: 220px; padding: 12px; display: grid; place-items: center; background: repeating-conic-gradient(#e7ebef 0 25%, #f6f8fa 0 50%) 0/18px 18px; }
    figure img { display: block; max-width: 100%; max-height: 520px; object-fit: contain; }
    .busy figure img { opacity: .55; }
    .is-hidden { display: none !important; }
    .results { margin-top: 16px; padding: 14px; }
    .table-wrap { overflow-x: auto; }
    table { width: 100%; border-collapse: collapse; }
    th, td { padding: 8px 10px; border-bottom: 1px solid var(--border); text-align: left; white-space: nowrap; }
    th { color: var(--muted); font-weight: 650; }
    footer { color: var(--muted); margin: 14px 2px 0; font-size: 13px; }
    @media (max-width: 900px) { .layout { grid-template-columns: 1fr; } }
    @media (max-width: 640px) { .shell { padding: 12px; } .previews { grid-template-columns: 1fr; } .image-wrap { min-height: 160px; } }
  </style>
</head>
<body>
<div class="shell">
  <header>
    <div><h1>OpenCV 综合视觉调参器 ——By MonthWU</h1><p class="subtitle">ROI → HSV / Canny → 轮廓 → 中心点与像素尺寸</p></div>
    <div class="toolbar">
      <label class="upload-label" for="imageInput">上传图片</label>
      <input id="imageInput" type="file" accept="image/png,image/jpeg,image/webp,image/bmp">
      <button class="btn" id="exampleButton" type="button">恢复示例图</button>
      <button class="btn" id="resetButton" type="button">重置参数</button>
      <button class="btn" id="saveButton" type="button">保存参数 JSON</button>
    </div>
  </header>

  <div class="layout">
    <aside class="panel">
      <h2>实时参数</h2>
      <details open><summary>基础与光照补偿</summary><div class="group-body">
        <div class="control"><div class="control-head"><label for="max_width_num">处理宽度</label><input class="number" id="max_width_num" type="number" min="320" max="1600" step="80" value="960"></div><input id="max_width_range" type="range" min="320" max="1600" step="80" value="960"><div class="hint">大图等比例缩放，降低网页延迟</div></div>
        <div class="control"><div class="control-head"><label for="denoise_kernel_num">降噪核</label><input class="number" id="denoise_kernel_num" type="number" min="1" max="15" step="2" value="3"></div><input id="denoise_kernel_range" type="range" min="1" max="15" step="2" value="3"></div>
        <div class="control"><div class="control-head"><label for="illumination_kernel_num">光照估计核</label><input class="number" id="illumination_kernel_num" type="number" min="3" max="201" step="2" value="81"></div><input id="illumination_kernel_range" type="range" min="3" max="201" step="2" value="81"></div>
        <div class="control"><div class="control-head"><label for="compensation_strength_num">补偿强度 %</label><input class="number" id="compensation_strength_num" type="number" min="0" max="100" step="1" value="100"></div><input id="compensation_strength_range" type="range" min="0" max="100" step="1" value="100"></div>
        <div class="control"><div class="control-head"><label for="gamma_num">Gamma</label><input class="number" id="gamma_num" type="number" min="0.2" max="3" step="0.05" value="1"></div><input id="gamma_range" type="range" min="0.2" max="3" step="0.05" value="1"></div>
        <div class="control"><div class="control-head"><label for="threshold_num">二值阈值</label><input class="number" id="threshold_num" type="number" min="0" max="255" step="1" value="150"></div><input id="threshold_range" type="range" min="0" max="255" step="1" value="150"></div>
        <div class="control"><div class="control-head"><label for="morph_kernel_num">形态学核</label><input class="number" id="morph_kernel_num" type="number" min="1" max="15" step="2" value="3"></div><input id="morph_kernel_range" type="range" min="1" max="15" step="2" value="3"></div>
        <div class="control"><div class="control-head"><label for="morph_iterations_num">闭运算次数</label><input class="number" id="morph_iterations_num" type="number" min="0" max="5" step="1" value="1"></div><input id="morph_iterations_range" type="range" min="0" max="5" step="1" value="1"></div>
      </div></details>
      <details><summary>ROI 百分比区域</summary><div class="group-body">
        <div class="control"><div class="control-head"><label for="roi_x_num">左边界 X %</label><input class="number" id="roi_x_num" type="number" min="0" max="99" step="1" value="0"></div><input id="roi_x_range" type="range" min="0" max="99" step="1" value="0"></div>
        <div class="control"><div class="control-head"><label for="roi_y_num">上边界 Y %</label><input class="number" id="roi_y_num" type="number" min="0" max="99" step="1" value="0"></div><input id="roi_y_range" type="range" min="0" max="99" step="1" value="0"></div>
        <div class="control"><div class="control-head"><label for="roi_width_num">宽度 %</label><input class="number" id="roi_width_num" type="number" min="1" max="100" step="1" value="100"></div><input id="roi_width_range" type="range" min="1" max="100" step="1" value="100"></div>
        <div class="control"><div class="control-head"><label for="roi_height_num">高度 %</label><input class="number" id="roi_height_num" type="number" min="1" max="100" step="1" value="100"></div><input id="roi_height_range" type="range" min="1" max="100" step="1" value="100"></div>
      </div></details>
      <details><summary>HSV 颜色范围</summary><div class="group-body">
        <div class="control"><div class="control-head"><label for="h_min_num">H 最小值</label><input class="number" id="h_min_num" type="number" min="0" max="179" step="1" value="35"></div><input id="h_min_range" type="range" min="0" max="179" step="1" value="35"></div>
        <div class="control"><div class="control-head"><label for="h_max_num">H 最大值</label><input class="number" id="h_max_num" type="number" min="0" max="179" step="1" value="95"></div><input id="h_max_range" type="range" min="0" max="179" step="1" value="95"><div class="hint">最小值大于最大值时自动跨越红色边界</div></div>
        <div class="control"><div class="control-head"><label for="s_min_num">S 最小值</label><input class="number" id="s_min_num" type="number" min="0" max="255" step="1" value="50"></div><input id="s_min_range" type="range" min="0" max="255" step="1" value="50"></div>
        <div class="control"><div class="control-head"><label for="s_max_num">S 最大值</label><input class="number" id="s_max_num" type="number" min="0" max="255" step="1" value="255"></div><input id="s_max_range" type="range" min="0" max="255" step="1" value="255"></div>
        <div class="control"><div class="control-head"><label for="v_min_num">V 最小值</label><input class="number" id="v_min_num" type="number" min="0" max="255" step="1" value="30"></div><input id="v_min_range" type="range" min="0" max="255" step="1" value="30"></div>
        <div class="control"><div class="control-head"><label for="v_max_num">V 最大值</label><input class="number" id="v_max_num" type="number" min="0" max="255" step="1" value="255"></div><input id="v_max_range" type="range" min="0" max="255" step="1" value="255"></div>
      </div></details>
      <details><summary>Canny 边缘</summary><div class="group-body">
        <div class="control"><div class="control-head"><label for="canny_low_num">低阈值</label><input class="number" id="canny_low_num" type="number" min="0" max="255" step="1" value="50"></div><input id="canny_low_range" type="range" min="0" max="255" step="1" value="50"></div>
        <div class="control"><div class="control-head"><label for="canny_high_num">高阈值</label><input class="number" id="canny_high_num" type="number" min="0" max="255" step="1" value="150"></div><input id="canny_high_range" type="range" min="0" max="255" step="1" value="150"></div>
        <div class="control"><div class="control-head"><label for="canny_aperture_num">Sobel 孔径</label><input class="number" id="canny_aperture_num" type="number" min="3" max="7" step="2" value="3"></div><input id="canny_aperture_range" type="range" min="3" max="7" step="2" value="3"></div>
      </div></details>
      <details><summary>轮廓筛选与绘制</summary><div class="group-body">
        <label for="contour_source">轮廓输入</label><select class="select" id="contour_source"><option value="hsv">HSV 掩膜</option><option value="binary">黑白图</option><option value="canny">Canny 边缘</option></select>
        <div class="control"><div class="control-head"><label for="contour_min_area_num">最小面积 px²</label><input class="number" id="contour_min_area_num" type="number" min="0" max="200000" step="100" value="100"></div><input id="contour_min_area_range" type="range" min="0" max="200000" step="100" value="100"></div>
        <div class="control"><div class="control-head"><label for="contour_max_area_num">最大面积 px²</label><input class="number" id="contour_max_area_num" type="number" min="100" max="1000000" step="100" value="200000"></div><input id="contour_max_area_range" type="range" min="100" max="1000000" step="100" value="200000"></div>
        <div class="control"><div class="control-head"><label for="contour_epsilon_num">多边形近似 %</label><input class="number" id="contour_epsilon_num" type="number" min="0" max="10" step="0.1" value="1"></div><input id="contour_epsilon_range" type="range" min="0" max="10" step="0.1" value="1"></div>
        <div class="control"><div class="control-head"><label for="contour_thickness_num">绘制线宽</label><input class="number" id="contour_thickness_num" type="number" min="1" max="10" step="1" value="2"></div><input id="contour_thickness_range" type="range" min="1" max="10" step="1" value="2"></div>
        <div class="control"><div class="control-head"><label for="max_objects_num">最多处理目标</label><input class="number" id="max_objects_num" type="number" min="1" max="50" step="1" value="20"></div><input id="max_objects_range" type="range" min="1" max="50" step="1" value="20"></div>
      </div></details>
      <details><summary>中心点识别</summary><div class="group-body">
        <label for="center_method">中心算法</label><select class="select" id="center_method"><option value="pixel">区域像素质心</option><option value="minrect">拟合旋转矩形中心</option><option value="bbox">水平外接框中心</option><option value="circle">最小外接圆中心</option></select>
        <div class="control"><div class="control-head"><label for="center_marker_size_num">中心标记尺寸</label><input class="number" id="center_marker_size_num" type="number" min="3" max="31" step="2" value="11"></div><input id="center_marker_size_range" type="range" min="3" max="31" step="2" value="11"></div>
        <div class="control"><div class="control-head"><label for="center_marker_thickness_num">中心标记线宽</label><input class="number" id="center_marker_thickness_num" type="number" min="1" max="5" step="1" value="2"></div><input id="center_marker_thickness_range" type="range" min="1" max="5" step="1" value="2"></div>
      </div></details>
      <details><summary>边长与直径测量</summary><div class="group-body">
        <label for="measurement_mode">测量方法</label><select class="select" id="measurement_mode"><option value="auto">自动分类</option><option value="rectangle">旋转矩形长短边</option><option value="circle">最小外接圆直径</option><option value="polygon">拟合多边形各边</option></select>
        <div class="control"><div class="control-head"><label for="circularity_threshold_num">圆度判定阈值</label><input class="number" id="circularity_threshold_num" type="number" min="0.5" max="1" step="0.01" value="0.82"></div><input id="circularity_threshold_range" type="range" min="0.5" max="1" step="0.01" value="0.82"></div>
        <div class="control"><div class="control-head"><label for="measurement_decimals_num">保留小数位</label><input class="number" id="measurement_decimals_num" type="number" min="0" max="3" step="1" value="1"></div><input id="measurement_decimals_range" type="range" min="0" max="3" step="1" value="1"></div>
        <div class="control"><div class="control-head"><label for="measurement_font_scale_num">标注字号</label><input class="number" id="measurement_font_scale_num" type="number" min="0.3" max="1.5" step="0.05" value="0.55"></div><input id="measurement_font_scale_range" type="range" min="0.3" max="1.5" step="0.05" value="0.55"></div>
        <div class="hint">当前输出为处理图上的像素尺寸；真实尺寸需要相机标定或比例尺。</div>
      </div></details>
      <details open><summary>预览图显示</summary><div class="group-body">
        <div class="check-grid">
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="original" checked>原图与ROI</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="roi" checked>ROI图像</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="gray" checked>灰度图</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="compensated" checked>光照补偿</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="binary" checked>黑白图</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="hsv" checked>HSV掩膜</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="canny" checked>Canny边缘</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="contours" checked>轮廓结果</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="centers" checked>中心点</label>
          <label class="check-item"><input class="preview-toggle" type="checkbox" data-preview="measurements" checked>尺寸测量</label>
        </div>
        <div class="mini-actions"><button class="btn" id="showAllPreviews" type="button">全部显示</button><button class="btn" id="hideAllPreviews" type="button">全部关闭</button></div>
      </div></details>
      <label class="switch" for="invert"><input id="invert" type="checkbox"><span>黑白反相</span></label>
    </aside>

    <main>
      <section class="panel status" aria-live="polite">
        <span>图片：<strong id="sourceName">内置阴影示例</strong></span>
        <span>原图尺寸：<strong id="sourceDimensions">—</strong></span>
        <span>处理尺寸：<strong id="dimensions">—</strong></span>
        <span>ROI：<strong id="roiDimensions">—</strong></span>
        <span>白色占比：<strong id="whiteRatio">—</strong></span>
        <span>HSV占比：<strong id="hsvRatio">—</strong></span>
        <span>边缘占比：<strong id="edgeRatio">—</strong></span>
        <span>轮廓数：<strong id="contourCount">—</strong></span>
        <span>已测目标：<strong id="objectCount">—</strong></span>
        <span>耗时：<strong id="processingTime">—</strong></span>
        <span id="message">正在初始化…</span>
      </section>
      <section class="previews" id="previewGrid">
        <figure class="panel" data-preview-card="original"><figcaption><span><span class="stage">1</span> 原图与ROI</span><span>黄色框</span></figcaption><div class="image-wrap"><img id="originalImage" alt="带ROI框的原始图片预览"></div></figure>
        <figure class="panel" data-preview-card="roi"><figcaption><span><span class="stage">2</span> ROI图像</span><span>处理输入</span></figcaption><div class="image-wrap"><img id="roiImage" alt="ROI图片预览"></div></figure>
        <figure class="panel" data-preview-card="gray"><figcaption><span><span class="stage">3</span> 灰度图</span><span>BGR → Gray</span></figcaption><div class="image-wrap"><img id="grayImage" alt="灰度图片预览"></div></figure>
        <figure class="panel" data-preview-card="compensated"><figcaption><span><span class="stage">4</span> 光照补偿</span><span>背景场除法</span></figcaption><div class="image-wrap"><img id="compensatedImage" alt="光照补偿图片预览"></div></figure>
        <figure class="panel" data-preview-card="binary"><figcaption><span><span class="stage">5</span> 黑白图</span><span>Threshold</span></figcaption><div class="image-wrap"><img id="binaryImage" alt="黑白二值图片预览"></div></figure>
        <figure class="panel" data-preview-card="hsv"><figcaption><span><span class="stage">6</span> HSV掩膜</span><span>inRange</span></figcaption><div class="image-wrap"><img id="hsvImage" alt="HSV颜色掩膜预览"></div></figure>
        <figure class="panel" data-preview-card="canny"><figcaption><span><span class="stage">7</span> Canny边缘</span><span>Canny</span></figcaption><div class="image-wrap"><img id="cannyImage" alt="Canny边缘预览"></div></figure>
        <figure class="panel" data-preview-card="contours"><figcaption><span><span class="stage">8</span> 轮廓结果</span><span>绿色线</span></figcaption><div class="image-wrap"><img id="contoursImage" alt="轮廓绘制结果预览"></div></figure>
        <figure class="panel" data-preview-card="centers"><figcaption><span><span class="stage">9</span> 中心点识别</span><span>十字标记</span></figcaption><div class="image-wrap"><img id="centersImage" alt="几何图形中心点预览"></div></figure>
        <figure class="panel" data-preview-card="measurements"><figcaption><span><span class="stage">10</span> 边长与直径</span><span>像素测量</span></figcaption><div class="image-wrap"><img id="measurementsImage" alt="几何图形尺寸测量预览"></div></figure>
      </section>
      <section class="panel results"><h2>目标检测结果</h2><div class="table-wrap"><table><thead><tr><th>#</th><th>类型</th><th>ROI中心</th><th>原图中心</th><th>面积</th><th>测量</th></tr></thead><tbody id="resultRows"><tr><td colspan="6">等待处理</td></tr></tbody></table></div></section>
      <footer>HSV支持色相跨0边界；轮廓输入可在黑白图、HSV掩膜和Canny边缘之间切换。</footer>
    </main>
  </div>
</div>
<script>
const defaults = {
  max_width:960,
  roi_x:0, roi_y:0, roi_width:100, roi_height:100,
  denoise_kernel:3, illumination_kernel:81, compensation_strength:100, gamma:1,
  threshold:150, morph_kernel:3, morph_iterations:1,
  h_min:35, h_max:95, s_min:50, s_max:255, v_min:30, v_max:255,
  canny_low:50, canny_high:150, canny_aperture:3,
  contour_min_area:100, contour_max_area:200000, contour_epsilon:1, contour_thickness:2,
  max_objects:20, center_marker_size:11, center_marker_thickness:2,
  circularity_threshold:0.82, measurement_decimals:1, measurement_font_scale:0.55,
  contour_source:'hsv', center_method:'pixel', measurement_mode:'auto', invert:false
};
const numericNames = Object.keys(defaults).filter(name => typeof defaults[name] === 'number');
const oddNames = new Set(['denoise_kernel', 'illumination_kernel', 'morph_kernel', 'canny_aperture']);
let selectedFile = null;
let debounceTimer = null;
let controller = null;
let requestSerial = 0;

function normalizedValue(name, value) {
  const input = document.getElementById(name + '_num');
  let number = Number(value);
  if (!Number.isFinite(number)) number = defaults[name];
  number = Math.max(Number(input.min), Math.min(Number(input.max), number));
  if (oddNames.has(name) && number % 2 === 0) number = Math.min(Number(input.max), number + 1);
  const decimals = (input.step.split('.')[1] || '').length;
  return Number(number.toFixed(decimals));
}

function bindPair(name) {
  const number = document.getElementById(name + '_num');
  const range = document.getElementById(name + '_range');
  const sync = event => {
    const value = normalizedValue(name, event.target.value);
    number.value = value;
    range.value = value;
    scheduleProcess();
  };
  number.addEventListener('input', sync);
  number.addEventListener('change', sync);
  range.addEventListener('input', sync);
}

function currentParams() {
  const params = {};
  numericNames.forEach(name => { params[name] = normalizedValue(name, document.getElementById(name + '_num').value); });
  params.invert = document.getElementById('invert').checked;
  params.contour_source = document.getElementById('contour_source').value;
  params.center_method = document.getElementById('center_method').value;
  params.measurement_mode = document.getElementById('measurement_mode').value;
  return params;
}

function setParams(params) {
  numericNames.forEach(name => {
    const value = normalizedValue(name, params[name] ?? defaults[name]);
    document.getElementById(name + '_num').value = value;
    document.getElementById(name + '_range').value = value;
  });
  document.getElementById('invert').checked = Boolean(params.invert);
  document.getElementById('contour_source').value = params.contour_source || defaults.contour_source;
  document.getElementById('center_method').value = params.center_method || defaults.center_method;
  document.getElementById('measurement_mode').value = params.measurement_mode || defaults.measurement_mode;
}

function renderResultRows(objects) {
  const body = document.getElementById('resultRows');
  body.replaceChildren();
  if (!objects.length) {
    const row = document.createElement('tr');
    const cell = document.createElement('td');
    cell.colSpan = 6; cell.textContent = '当前参数下未检测到符合条件的目标';
    row.appendChild(cell); body.appendChild(row); return;
  }
  objects.forEach(object => {
    const row = document.createElement('tr');
    const values = [
      object.index,
      object.shape,
      '(' + object.center_roi.join(', ') + ')',
      '(' + object.center_image.join(', ') + ')',
      object.area + ' px²',
      object.measurement
    ];
    values.forEach(value => { const cell = document.createElement('td'); cell.textContent = value; row.appendChild(cell); });
    body.appendChild(row);
  });
}

function applyPreviewVisibility(save = true) {
  const state = {};
  document.querySelectorAll('.preview-toggle').forEach(toggle => {
    state[toggle.dataset.preview] = toggle.checked;
    const card = document.querySelector('[data-preview-card="' + toggle.dataset.preview + '"]');
    if (card) card.classList.toggle('is-hidden', !toggle.checked);
  });
  if (save) { try { localStorage.setItem('opencv-preview-state', JSON.stringify(state)); } catch (_) {} }
}

function restorePreviewVisibility() {
  try {
    const state = JSON.parse(localStorage.getItem('opencv-preview-state') || '{}');
    document.querySelectorAll('.preview-toggle').forEach(toggle => {
      if (typeof state[toggle.dataset.preview] === 'boolean') toggle.checked = state[toggle.dataset.preview];
    });
  } catch (_) {}
  applyPreviewVisibility(false);
}

function scheduleProcess() {
  clearTimeout(debounceTimer);
  debounceTimer = setTimeout(runProcess, 120);
}

async function runProcess() {
  const serial = ++requestSerial;
  if (controller) controller.abort();
  controller = new AbortController();
  const message = document.getElementById('message');
  message.className = '';
  message.textContent = '处理中…';
  document.getElementById('previewGrid').classList.add('busy');
  const body = new FormData();
  body.append('params', JSON.stringify(currentParams()));
  if (selectedFile) body.append('image', selectedFile);
  try {
    const response = await fetch('/api/process', {method:'POST', body, signal:controller.signal});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || '处理失败');
    if (serial !== requestSerial) return;
    document.getElementById('originalImage').src = result.images.original;
    document.getElementById('roiImage').src = result.images.roi;
    document.getElementById('grayImage').src = result.images.gray;
    document.getElementById('compensatedImage').src = result.images.compensated;
    document.getElementById('binaryImage').src = result.images.binary;
    document.getElementById('hsvImage').src = result.images.hsv;
    document.getElementById('cannyImage').src = result.images.canny;
    document.getElementById('contoursImage').src = result.images.contours;
    document.getElementById('centersImage').src = result.images.centers;
    document.getElementById('measurementsImage').src = result.images.measurements;
    document.getElementById('dimensions').textContent = result.stats.width + ' × ' + result.stats.height;
    document.getElementById('sourceDimensions').textContent = result.stats.source_width + ' × ' + result.stats.source_height;
    document.getElementById('roiDimensions').textContent = result.stats.roi_width + ' × ' + result.stats.roi_height;
    document.getElementById('whiteRatio').textContent = result.stats.white_ratio.toFixed(2) + '%';
    document.getElementById('hsvRatio').textContent = result.stats.hsv_ratio.toFixed(2) + '%';
    document.getElementById('edgeRatio').textContent = result.stats.edge_ratio.toFixed(2) + '%';
    document.getElementById('contourCount').textContent = result.stats.contour_count;
    document.getElementById('objectCount').textContent = result.stats.displayed_count;
    document.getElementById('processingTime').textContent = result.stats.processing_ms.toFixed(1) + ' ms';
    renderResultRows(result.stats.objects || []);
    message.textContent = '实时预览已更新';
  } catch (error) {
    if (error.name !== 'AbortError') { message.className = 'error'; message.textContent = error.message; }
  } finally {
    if (serial === requestSerial) document.getElementById('previewGrid').classList.remove('busy');
  }
}

numericNames.forEach(bindPair);
document.getElementById('invert').addEventListener('change', scheduleProcess);
document.getElementById('contour_source').addEventListener('change', scheduleProcess);
document.getElementById('center_method').addEventListener('change', scheduleProcess);
document.getElementById('measurement_mode').addEventListener('change', scheduleProcess);
document.querySelectorAll('.preview-toggle').forEach(toggle => toggle.addEventListener('change', () => applyPreviewVisibility(true)));
document.getElementById('showAllPreviews').addEventListener('click', () => {
  document.querySelectorAll('.preview-toggle').forEach(toggle => { toggle.checked = true; }); applyPreviewVisibility(true);
});
document.getElementById('hideAllPreviews').addEventListener('click', () => {
  document.querySelectorAll('.preview-toggle').forEach(toggle => { toggle.checked = false; }); applyPreviewVisibility(true);
});
document.getElementById('imageInput').addEventListener('change', event => {
  selectedFile = event.target.files[0] || null;
  document.getElementById('sourceName').textContent = selectedFile ? selectedFile.name : '内置阴影示例';
  scheduleProcess();
});
document.getElementById('exampleButton').addEventListener('click', () => {
  selectedFile = null; document.getElementById('imageInput').value = '';
  document.getElementById('sourceName').textContent = '内置阴影示例'; scheduleProcess();
});
document.getElementById('resetButton').addEventListener('click', () => { setParams(defaults); scheduleProcess(); });
document.getElementById('saveButton').addEventListener('click', () => {
  const blob = new Blob([JSON.stringify(currentParams(), null, 2)], {type:'application/json'});
  const link = document.createElement('a'); link.href = URL.createObjectURL(blob); link.download = 'opencv_tuner_params.json'; link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 500);
});
setParams(defaults);
restorePreviewVisibility();
runProcess();
</script>
</body>
</html>'''


if __name__ == "__main__":
    app.run(
        host=os.environ.get("OPENCV_TUNER_HOST", "127.0.0.1"),
        port=int(os.environ.get("OPENCV_TUNER_PORT", "5000")),
        debug=False,
        threaded=True,
    )
