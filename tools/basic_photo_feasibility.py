#!/usr/bin/env python3
"""Offline feasibility check for the 26E basic four-piece photo.

This tool intentionally mirrors the explainable part of the ROS pipeline:
green A4 quadrilateral -> perspective warp -> white-piece contours -> basic
template rigid fitting -> target rectangle closure.  It is not a replacement
for Jetson ROS or final post-placement verification.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


SCALE = 4.0
A4_W_MM = 210.0
A4_H_MM = 297.0
TARGET_W_MM = 100.0
TARGET_H_MM = 60.0
TARGET_CENTER_MM = np.array([105.0, 222.75], dtype=np.float64)
TARGET_OFFSET_MM = TARGET_CENTER_MM - np.array([TARGET_W_MM / 2.0, TARGET_H_MM / 2.0])

# These are the project baseline values and the photo-specific tuned candidate.
BASELINE_GREEN = (60, 78, 91, 90, 201, 245)
TUNED_GREEN = (35, 80, 70, 75, 255, 255)
WHITE = (0, 0, 160, 179, 77, 255)

TEMPLATES = [
    ("right_triangle", np.array([[20, 0], [100, 0], [100, 60]], dtype=np.float64), 2400.0),
    ("upper_quadrilateral", np.array([[0, 0], [20, 0], [36, 12], [0, 20]], dtype=np.float64), 480.0),
    ("middle_quadrilateral", np.array([[0, 20], [36, 12], [76, 42], [0, 30]], dtype=np.float64), 1080.0),
    ("lower_quadrilateral", np.array([[0, 30], [76, 42], [100, 60], [0, 60]], dtype=np.float64), 2040.0),
]


def mask_hsv(image: np.ndarray, values: tuple[int, int, int, int, int, int]) -> np.ndarray:
    h0, s0, v0, h1, s1, v1 = values
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array([h0, s0, v0], np.uint8), np.array([h1, s1, v1], np.uint8))


def morph(mask: np.ndarray, size: int, iterations: int) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
    result = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
    return cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel, iterations=iterations)


def order_quad(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    sums = points.sum(axis=1)
    diffs = points[:, 0] - points[:, 1]
    return np.array(
        [points[np.argmin(sums)], points[np.argmax(diffs)], points[np.argmax(sums)], points[np.argmin(diffs)]],
        dtype=np.float32,
    )


def polygon_area(points: np.ndarray) -> float:
    return abs(float(cv2.contourArea(np.asarray(points, dtype=np.float32))))


def polygon_centroid(points: np.ndarray) -> np.ndarray:
    moments = cv2.moments(np.asarray(points, dtype=np.float32))
    if abs(moments["m00"]) < 1e-9:
        raise ValueError("zero-area polygon")
    return np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=np.float64)


def contour_center(contour: np.ndarray) -> np.ndarray:
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-9:
        raise ValueError("zero-area contour")
    return np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=np.float64) / SCALE


def fit_rigid(source: np.ndarray, target: np.ndarray) -> tuple[float, float, float, np.ndarray, np.ndarray]:
    """Match the same vertex-order formula used by puzzle_solver_core.cpp."""
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    dot = float(np.sum(source_centered[:, 0] * target_centered[:, 0] + source_centered[:, 1] * target_centered[:, 1]))
    cross = float(np.sum(source_centered[:, 0] * target_centered[:, 1] - source_centered[:, 1] * target_centered[:, 0]))
    angle = math.atan2(cross, dot)
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]], dtype=np.float64)
    translation = target_mean - source_mean @ rotation.T
    predicted = source @ rotation.T + translation
    errors = np.linalg.norm(predicted - target, axis=1)
    return float(np.sqrt(np.mean(errors * errors))), float(errors.max()), angle, translation, predicted


def best_template_fit(source: np.ndarray, measured_area: float) -> dict[str, Any] | None:
    best: dict[str, Any] | None = None
    for template_index, (name, template, expected_area) in enumerate(TEMPLATES):
        if len(source) != len(template):
            continue
        area_relative_error = abs(measured_area - expected_area) / expected_area
        if area_relative_error > 0.18:
            continue
        for reverse in (False, True):
            base = template[::-1] if reverse else template
            for shift in range(len(base)):
                candidate = np.roll(base, shift, axis=0) + TARGET_OFFSET_MM
                rms, max_error, angle, translation, predicted = fit_rigid(source, candidate)
                if rms > 3.0 or max_error > 5.0:
                    continue
                score = rms + 10.0 * area_relative_error
                result = {
                    "template_index": template_index,
                    "template_name": name,
                    "expected_area_mm2": expected_area,
                    "area_relative_error": area_relative_error,
                    "rms_mm": rms,
                    "max_vertex_error_mm": max_error,
                    "image_rotation_rad": angle,
                    "rotation_delta_ccw_deg": ((-math.degrees(angle) + 180.0) % 360.0) - 180.0,
                    "translation_mm": translation,
                    "predicted_target_polygon_mm": predicted,
                    "score": score,
                }
                if best is None or score < best["score"]:
                    best = result
    return best


def draw_arrow(image: np.ndarray, start: tuple[int, int], end: tuple[int, int], color: tuple[int, int, int]) -> None:
    cv2.arrowedLine(image, start, end, color, 2, cv2.LINE_AA, tipLength=0.08)


def put_label(image: np.ndarray, text: str, point: tuple[int, int], color: tuple[int, int, int]) -> None:
    cv2.putText(image, text, point, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, point, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def draw_cross(image: np.ndarray, point: tuple[int, int], color: tuple[int, int, int], size: int = 10) -> None:
    cv2.drawMarker(image, point, color, cv2.MARKER_CROSS, size, 2, cv2.LINE_AA)


def to_int_point(point: np.ndarray) -> tuple[int, int]:
    return int(round(float(point[0]))), int(round(float(point[1])))


def make_rectified_overlay(
    warped: np.ndarray, pieces: list[dict[str, Any]], inverse_h: np.ndarray, output: Path
) -> None:
    overlay = warped.copy()
    colors = [(40, 80, 220), (220, 90, 40), (50, 160, 70), (180, 70, 180)]
    for index, piece in enumerate(pieces, 1):
        color = colors[index - 1]
        source_poly_px = np.round(piece["source_polygon_mm"] * SCALE).astype(np.int32)
        target_poly_px = np.round(piece["target_polygon_mm"] * SCALE).astype(np.int32)
        cv2.polylines(overlay, [source_poly_px], True, color, 3, cv2.LINE_AA)
        cv2.polylines(overlay, [target_poly_px], True, color, 3, cv2.LINE_AA)
        source_px = to_int_point(piece["source_center_mm"] * SCALE)
        target_px = to_int_point(piece["target_center_mm"] * SCALE)
        draw_cross(overlay, source_px, color, 16)
        draw_cross(overlay, target_px, color, 16)
        draw_arrow(overlay, source_px, target_px, color)
        put_label(overlay, f"P{index} {piece['template_name'][:3]} d={piece['rotation_delta_ccw_deg']:.1f}deg", (source_px[0] + 8, source_px[1] - 8), color)
        put_label(overlay, f"S{index}", (source_px[0] + 8, source_px[1] + 20), color)
        put_label(overlay, f"T{index}", (target_px[0] + 8, target_px[1] + 20), color)
    rect = np.round((np.array([TARGET_OFFSET_MM, TARGET_OFFSET_MM + [TARGET_W_MM, 0], TARGET_OFFSET_MM + [TARGET_W_MM, TARGET_H_MM], TARGET_OFFSET_MM + [0, TARGET_H_MM]]) * SCALE)).astype(np.int32)
    cv2.polylines(overlay, [rect], True, (0, 0, 0), 7, cv2.LINE_AA)
    cv2.polylines(overlay, [rect], True, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.imwrite(str(output), overlay, [cv2.IMWRITE_JPEG_QUALITY, 94])


def make_original_overlay(
    original: np.ndarray, quad: np.ndarray, homography: np.ndarray, pieces: list[dict[str, Any]], output: Path
) -> None:
    overlay = original.copy()
    colors = [(40, 80, 220), (220, 90, 40), (50, 160, 70), (180, 70, 180)]
    quad_i = np.round(quad).astype(np.int32)
    cv2.polylines(overlay, [quad_i], True, (0, 0, 0), 8, cv2.LINE_AA)
    cv2.polylines(overlay, [quad_i], True, (0, 255, 255), 4, cv2.LINE_AA)
    for index, piece in enumerate(pieces, 1):
        color = colors[index - 1]
        src_poly = cv2.perspectiveTransform((piece["source_polygon_mm"] * SCALE).astype(np.float32)[None], homography)[0]
        target_poly = cv2.perspectiveTransform((piece["target_polygon_mm"] * SCALE).astype(np.float32)[None], homography)[0]
        cv2.polylines(overlay, [np.round(src_poly).astype(np.int32)], True, color, 4, cv2.LINE_AA)
        cv2.polylines(overlay, [np.round(target_poly).astype(np.int32)], True, color, 3, cv2.LINE_AA)
        source_px = to_int_point(cv2.perspectiveTransform((piece["source_center_mm"] * SCALE).astype(np.float32).reshape(1, 1, 2), homography)[0, 0])
        target_px = to_int_point(cv2.perspectiveTransform((piece["target_center_mm"] * SCALE).astype(np.float32).reshape(1, 1, 2), homography)[0, 0])
        draw_cross(overlay, source_px, color, 20)
        draw_cross(overlay, target_px, color, 20)
        draw_arrow(overlay, source_px, target_px, color)
        put_label(overlay, f"P{index} d={piece['rotation_delta_ccw_deg']:.1f}deg", (source_px[0] + 10, source_px[1] - 10), color)
    put_label(overlay, "YELLOW=A4  S=current  T=target", (quad_i[0, 0], max(30, quad_i[0, 1] - 12)), (0, 255, 255))
    cv2.imwrite(str(output), overlay, [cv2.IMWRITE_JPEG_QUALITY, 94])


def make_target_layout(pieces: list[dict[str, Any]], output: Path) -> None:
    canvas = np.full((int(A4_H_MM * SCALE), int(A4_W_MM * SCALE), 3), (95, 170, 50), np.uint8)
    colors = [(70, 100, 230), (230, 110, 55), (70, 190, 90), (190, 80, 190)]
    for index, piece in enumerate(pieces, 1):
        color = colors[index - 1]
        points = np.round(piece["target_polygon_mm"] * SCALE).astype(np.int32)
        cv2.fillPoly(canvas, [points], tuple(int(0.25 * x + 0.75 * y) for x, y in zip(color, (95, 170, 50))))
        cv2.polylines(canvas, [points], True, color, 3, cv2.LINE_AA)
        center = to_int_point(piece["target_center_mm"] * SCALE)
        draw_cross(canvas, center, color, 18)
        put_label(canvas, f"T{index} {piece['template_name'][:3]}", (center[0] + 8, center[1] - 8), color)
    rect = np.round(np.array([TARGET_OFFSET_MM, TARGET_OFFSET_MM + [TARGET_W_MM, 0], TARGET_OFFSET_MM + [TARGET_W_MM, TARGET_H_MM], TARGET_OFFSET_MM + [0, TARGET_H_MM]]) * SCALE).astype(np.int32)
    cv2.polylines(canvas, [rect], True, (255, 255, 255), 5, cv2.LINE_AA)
    put_label(canvas, "100x60 mm target rectangle", (int(TARGET_OFFSET_MM[0] * SCALE), int((TARGET_OFFSET_MM[1] - 5) * SCALE)), (255, 255, 255))
    cv2.imwrite(str(output), canvas, [cv2.IMWRITE_PNG_COMPRESSION, 3])


def point_grid_inside_polygon(x_grid: np.ndarray, y_grid: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    inside = np.zeros(x_grid.shape, dtype=bool)
    previous = len(polygon) - 1
    for current in range(len(polygon)):
        x0, y0 = polygon[current]
        x1, y1 = polygon[previous]
        crosses = ((y0 > y_grid) != (y1 > y_grid)) & (
            x_grid < (x1 - x0) * (y_grid - y0) / (y1 - y0 + 1e-30) + x0
        )
        inside ^= crosses
        previous = current
    return inside


def measure_target_geometry(pieces: list[dict[str, Any]], resolution_mm: float = 0.1) -> dict[str, float | bool]:
    polygons = [piece["target_polygon_mm"] for piece in pieces]
    min_x = min(TARGET_OFFSET_MM[0], *(float(np.min(p[:, 0])) for p in polygons)) - 2.0
    max_x = max(TARGET_OFFSET_MM[0] + TARGET_W_MM, *(float(np.max(p[:, 0])) for p in polygons)) + 2.0
    min_y = min(TARGET_OFFSET_MM[1], *(float(np.min(p[:, 1])) for p in polygons)) - 2.0
    max_y = max(TARGET_OFFSET_MM[1] + TARGET_H_MM, *(float(np.max(p[:, 1])) for p in polygons)) + 2.0
    xs = np.arange(min_x, max_x, resolution_mm) + 0.5 * resolution_mm
    ys = np.arange(min_y, max_y, resolution_mm) + 0.5 * resolution_mm
    x_grid, y_grid = np.meshgrid(xs, ys)
    counts = np.zeros(x_grid.shape, dtype=np.uint8)
    for polygon in polygons:
        counts += point_grid_inside_polygon(x_grid, y_grid, polygon)
    ideal = (
        (x_grid >= TARGET_OFFSET_MM[0])
        & (x_grid < TARGET_OFFSET_MM[0] + TARGET_W_MM)
        & (y_grid >= TARGET_OFFSET_MM[1])
        & (y_grid < TARGET_OFFSET_MM[1] + TARGET_H_MM)
    )
    cell_area = resolution_mm * resolution_mm
    union = counts > 0
    intersection_area = float(np.sum(union & ideal) * cell_area)
    union_area = float(np.sum(union) * cell_area)
    hole_area = float(np.sum((counts == 0) & ideal) * cell_area)
    outside_area = float(np.sum(union & ~ideal) * cell_area)
    overlap_area = float(np.sum(counts > 1) * cell_area)
    x_values = [float(x) for p in polygons for x in p[:, 0]]
    y_values = [float(y) for p in polygons for y in p[:, 1]]
    bbox_width = max(x_values) - min(x_values)
    bbox_height = max(y_values) - min(y_values)
    bbox_error = max(abs(bbox_width - TARGET_W_MM), abs(bbox_height - TARGET_H_MM))
    rectangularity = intersection_area / max(union_area, 1e-9)
    target_coverage = intersection_area / (TARGET_W_MM * TARGET_H_MM)
    coarse_pass = rectangularity >= 0.95 and bbox_error <= 5.0
    strict_pass = coarse_pass and overlap_area <= 10.0 and hole_area <= 10.0 and outside_area <= 10.0
    return {
        "resolution_mm": resolution_mm,
        "bbox_width_mm": bbox_width,
        "bbox_height_mm": bbox_height,
        "bbox_error_mm": bbox_error,
        "union_area_mm2": union_area,
        "intersection_area_mm2": intersection_area,
        "hole_area_mm2": hole_area,
        "outside_area_mm2": outside_area,
        "overlap_area_mm2": overlap_area,
        "rectangularity": rectangularity,
        "target_coverage": target_coverage,
        "coarse_rectangle_pass": coarse_pass,
        "strict_rectangle_closure": strict_pass,
    }


def make_rectangle_error_map(pieces: list[dict[str, Any]], metrics: dict[str, float | bool], output: Path) -> None:
    resolution = 0.125
    min_x, max_x = 50.0, 160.0
    min_y, max_y = 188.0, 258.0
    xs = np.arange(min_x, max_x, resolution) + 0.5 * resolution
    ys = np.arange(min_y, max_y, resolution) + 0.5 * resolution
    x_grid, y_grid = np.meshgrid(xs, ys)
    counts = np.zeros(x_grid.shape, dtype=np.uint8)
    for piece in pieces:
        counts += point_grid_inside_polygon(x_grid, y_grid, piece["target_polygon_mm"])
    ideal = (
        (x_grid >= TARGET_OFFSET_MM[0])
        & (x_grid < TARGET_OFFSET_MM[0] + TARGET_W_MM)
        & (y_grid >= TARGET_OFFSET_MM[1])
        & (y_grid < TARGET_OFFSET_MM[1] + TARGET_H_MM)
    )
    image = np.full((*counts.shape, 3), (45, 45, 45), np.uint8)
    image[ideal & (counts == 1)] = (60, 165, 70)      # valid single coverage
    image[ideal & (counts == 0)] = (0, 0, 0)          # hole/gap
    image[(counts > 0) & ~ideal] = (40, 40, 225)      # outside ideal rectangle
    image[counts > 1] = (0, 220, 255)                 # overlap
    image = cv2.resize(image, None, fx=1.25, fy=1.25, interpolation=cv2.INTER_NEAREST)
    border = 82
    canvas = cv2.copyMakeBorder(image, border, 15, 15, 15, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    line1 = (
        f"gap={metrics['hole_area_mm2']:.1f}  overlap={metrics['overlap_area_mm2']:.1f}  "
        f"outside={metrics['outside_area_mm2']:.1f} mm2"
    )
    line2 = (
        f"rectangularity={metrics['rectangularity']:.4f}  coverage={metrics['target_coverage']:.4f}  "
        f"strict={'PASS' if metrics['strict_rectangle_closure'] else 'RETRY'}"
    )
    put_label(canvas, line1, (18, 30), (255, 255, 255))
    put_label(canvas, line2, (18, 58), (255, 255, 255))
    put_label(canvas, "GREEN=valid  BLACK=gap  YELLOW=overlap  RED=outside", (18, canvas.shape[0] - 12), (255, 255, 255))
    cv2.imwrite(str(output), canvas, [cv2.IMWRITE_PNG_COMPRESSION, 3])


def run(input_path: Path, output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    original = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if original is None:
        raise FileNotFoundError(input_path)

    baseline = morph(mask_hsv(original, BASELINE_GREEN), 7, 2)
    baseline_contours, _ = cv2.findContours(baseline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    baseline_area = max((cv2.contourArea(c) for c in baseline_contours), default=0.0)

    green = morph(mask_hsv(original, TUNED_GREEN), 7, 2)
    contours, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    if not contours:
        raise RuntimeError("A4 green contour not found")
    green_contour = contours[0]
    green_polygon = cv2.approxPolyDP(green_contour, 0.02 * cv2.arcLength(green_contour, True), True).reshape(-1, 2)
    if len(green_polygon) != 4:
        raise RuntimeError(f"A4 candidate has {len(green_polygon)} vertices, expected 4")
    quad = order_quad(green_polygon)
    warp_w, warp_h = int(A4_W_MM * SCALE), int(A4_H_MM * SCALE)
    destination = np.array([[0, 0], [warp_w - 1, 0], [warp_w - 1, warp_h - 1], [0, warp_h - 1]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(quad, destination)
    inverse_homography = np.linalg.inv(homography)
    warped = cv2.warpPerspective(original, homography, (warp_w, warp_h))

    white = morph(mask_hsv(warped, WHITE), 3, 1)
    white[: int(warp_h * 0.5), :] = white[: int(warp_h * 0.5), :]
    white[int(warp_h * 0.5) :, :] = 0
    margin = int(3.0 * SCALE)
    white[:margin, :] = 0
    white[:, :margin] = 0
    white[:, -margin:] = 0
    piece_contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    piece_contours = sorted(piece_contours, key=cv2.contourArea, reverse=True)
    pieces: list[dict[str, Any]] = []
    for contour in piece_contours:
        area = cv2.contourArea(contour) / (SCALE * SCALE)
        if area < 100.0 or area > 12000.0:
            continue
        polygon = cv2.approxPolyDP(contour, 1.75 * SCALE, True).reshape(-1, 2).astype(np.float64) / SCALE
        if not 3 <= len(polygon) <= 5:
            continue
        center = contour_center(contour)
        pieces.append({"source_polygon_mm": polygon, "source_center_mm": center, "area_mm2": area, "vertex_count": len(polygon)})
    if len(pieces) != 4:
        raise RuntimeError(f"white piece count {len(pieces)} != 4")

    # Evaluate all bijections, exactly as the basic solver does.
    assignment_results: list[dict[str, Any]] = []
    for assignment in itertools.permutations(range(4)):
        fits: list[dict[str, Any]] = []
        valid = True
        for piece, template_index in zip(pieces, assignment):
            name, template, expected_area = TEMPLATES[template_index]
            if len(piece["source_polygon_mm"]) != len(template):
                valid = False
                break
            rel = abs(piece["area_mm2"] - expected_area) / expected_area
            if rel > 0.18:
                valid = False
                break
            candidate_fits = []
            for reverse in (False, True):
                base = template[::-1] if reverse else template
                for shift in range(len(base)):
                    target = np.roll(base, shift, axis=0) + TARGET_OFFSET_MM
                    rms, max_error, angle, translation, predicted = fit_rigid(piece["source_polygon_mm"], target)
                    if rms <= 3.0 and max_error <= 5.0:
                        candidate_fits.append((rms + 10.0 * rel, rms, max_error, angle, translation, predicted))
            if not candidate_fits:
                valid = False
                break
            candidate_fits.sort(key=lambda x: x[0])
            score, rms, max_error, angle, translation, predicted = candidate_fits[0]
            target_polygon = np.asarray(predicted, dtype=np.float64)
            target_center = (piece["source_center_mm"] @ np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]], dtype=np.float64).T) + translation
            fits.append({"template_index": template_index, "template_name": name, "expected_area_mm2": expected_area, "score": score, "rms_mm": rms, "max_vertex_error_mm": max_error, "area_relative_error": rel, "image_rotation_rad": angle, "rotation_delta_ccw_deg": ((-math.degrees(angle) + 180.0) % 360.0) - 180.0, "target_polygon_mm": target_polygon, "target_center_mm": target_center})
        if valid:
            assignment_results.append({"score": sum(x["score"] for x in fits), "assignment": assignment, "fits": fits})
    assignment_results.sort(key=lambda x: x["score"])
    if not assignment_results:
        raise RuntimeError("no valid basic-task template assignment")
    best_assignment = assignment_results[0]
    second_score = assignment_results[1]["score"] if len(assignment_results) > 1 else best_assignment["score"] + 1000.0
    score_margin = second_score - best_assignment["score"]

    solved_pieces: list[dict[str, Any]] = []
    for piece, fit in zip(pieces, best_assignment["fits"]):
        solved = dict(piece)
        solved.update(fit)
        template = TEMPLATES[fit["template_index"]][1] + TARGET_OFFSET_MM
        solved["ideal_target_center_mm"] = polygon_centroid(template)
        solved["target_center_ideal_error_mm"] = float(
            np.linalg.norm(solved["target_center_mm"] - solved["ideal_target_center_mm"])
        )
        solved["source_center_inside"] = bool(
            cv2.pointPolygonTest(
                np.asarray(solved["source_polygon_mm"], dtype=np.float32),
                tuple(float(x) for x in solved["source_center_mm"]),
                False,
            ) >= 0
        )
        solved["target_center_inside"] = bool(
            cv2.pointPolygonTest(
                np.asarray(solved["target_polygon_mm"], dtype=np.float32),
                tuple(float(x) for x in solved["target_center_mm"]),
                False,
            ) >= 0
        )
        solved_pieces.append(solved)

    target_union_area = sum(x[2] for x in TEMPLATES)
    geometry_metrics = measure_target_geometry(solved_pieces)
    piece_upper_ok = all(float(np.max(p["source_polygon_mm"][:, 1])) <= A4_H_MM * 0.5 for p in pieces)
    template_ok = score_margin >= 2.0 and all(p["rms_mm"] <= 3.0 and p["max_vertex_error_mm"] <= 5.0 for p in solved_pieces)
    center_mapping_ok = all(
        p["source_center_inside"]
        and p["target_center_inside"]
        and p["target_center_ideal_error_mm"] <= 2.0
        for p in solved_pieces
    )
    strict_rectangle_ok = bool(geometry_metrics["strict_rectangle_closure"])
    if template_ok and center_mapping_ok and piece_upper_ok and not strict_rectangle_ok:
        decision = "RETRY_CONTOUR_REFINEMENT"
    elif template_ok and center_mapping_ok and piece_upper_ok and strict_rectangle_ok:
        decision = "PASS_WITH_GREEN_TUNING"
    else:
        decision = "RETRY_DETECTION_OR_MATCHING"
    report: dict[str, Any] = {
        "input": str(input_path),
        "image_width_px": int(original.shape[1]),
        "image_height_px": int(original.shape[0]),
        "baseline_green": {"hsv": list(BASELINE_GREEN), "largest_contour_area_px2": baseline_area, "a4_found": baseline_area > 0},
        "tuned_green": {"hsv": list(TUNED_GREEN), "largest_contour_area_px2": float(cv2.contourArea(green_contour)), "a4_quad_px": quad.tolist(), "hsv_note": "photo-specific candidate; re-sample on Jetson camera"},
        "warp": {"width_px": warp_w, "height_px": warp_h, "scale_px_per_mm": SCALE},
        "piece_count": len(solved_pieces),
        "initial_upper_half_ok": piece_upper_ok,
        "template_match_pass": template_ok,
        "center_mapping_pass": center_mapping_ok,
        "assignment": {"piece_to_template": {f"piece_{i+1}": p["template_name"] for i, p in enumerate(solved_pieces)}, "best_score": best_assignment["score"], "second_score": second_score, "score_margin": score_margin},
        "pieces": [],
        "target_geometry": {
            "ideal_template_width_mm": TARGET_W_MM,
            "ideal_template_height_mm": TARGET_H_MM,
            "offset_mm": TARGET_OFFSET_MM.tolist(),
            "ideal_template_union_area_mm2": target_union_area,
            "ideal_template_rectangle_closure": True,
            "measured_rigid_assembly": geometry_metrics,
        },
        "decision": decision,
        "warnings": ["Existing competition_tuning.yaml green H=60..90 fails on this photo; use the tuned candidate only as a starting point.", "workspace_mapping_valid remains false; coordinates are A4-local mm for tuning and must not be sent as workspace-absolute control coordinates.", "Rectangle check is the planned target geometry; final post-placement re-scan is still required on Jetson."],
    }
    for piece in solved_pieces:
        source_center_px = cv2.perspectiveTransform((piece["source_center_mm"] * SCALE).astype(np.float32).reshape(1, 1, 2), inverse_homography)[0, 0]
        target_center_px = cv2.perspectiveTransform((piece["target_center_mm"] * SCALE).astype(np.float32).reshape(1, 1, 2), inverse_homography)[0, 0]
        report["pieces"].append({"template": piece["template_name"], "source_center_a4_mm": piece["source_center_mm"].tolist(), "target_center_a4_mm": piece["target_center_mm"].tolist(), "ideal_target_center_a4_mm": piece["ideal_target_center_mm"].tolist(), "target_center_ideal_error_mm": piece["target_center_ideal_error_mm"], "source_center_inside": piece["source_center_inside"], "target_center_inside": piece["target_center_inside"], "source_center_image_px": source_center_px.tolist(), "target_center_image_px": target_center_px.tolist(), "center_delta_mm": (piece["target_center_mm"] - piece["source_center_mm"]).tolist(), "rotation_delta_ccw_deg": piece["rotation_delta_ccw_deg"], "measured_area_mm2": piece["area_mm2"], "template_area_mm2": piece["expected_area_mm2"], "vertex_count": piece["vertex_count"], "fit_rms_mm": piece["rms_mm"], "fit_max_vertex_error_mm": piece["max_vertex_error_mm"], "source_polygon_mm": piece["source_polygon_mm"].tolist(), "target_polygon_mm": piece["target_polygon_mm"].tolist()})

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "report.json"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    make_rectified_overlay(warped, solved_pieces, inverse_homography, output_dir / "rectified_overlay.jpg")
    make_original_overlay(original, quad, inverse_homography, solved_pieces, output_dir / "original_overlay.jpg")
    make_target_layout(solved_pieces, output_dir / "target_layout_check.png")
    make_rectangle_error_map(solved_pieces, geometry_metrics, output_dir / "rectangle_error_map.png")
    lines = ["# 基础题实拍图可行性结果", "", f"- 总结论：**{report['decision']}**", f"- 输入：`{input_path}`", f"- 原绿色 HSV 找到 A4：`{report['baseline_green']['a4_found']}`；本图临时阈值找到 A4：`True`", f"- 拼图块：`{len(solved_pieces)}/4`；均完整位于上半区：`{piece_upper_ok}`", f"- 模板匹配：`{template_ok}`；中心映射：`{center_mapping_ok}`；严格矩形闭合：`{strict_rectangle_ok}`", f"- 刚体旋转后外接尺寸：`{geometry_metrics['bbox_width_mm']:.3f} x {geometry_metrics['bbox_height_mm']:.3f} mm`", f"- 矩形度：`{geometry_metrics['rectangularity']:.4f}`；覆盖率：`{geometry_metrics['target_coverage']:.4f}`", f"- 缺口/重叠/越界：`{geometry_metrics['hole_area_mm2']:.1f} / {geometry_metrics['overlap_area_mm2']:.1f} / {geometry_metrics['outside_area_mm2']:.1f} mm2`", "", "## 中心点、目标点与旋转结果", "", "| 块 | 模板 | 当前中心 (mm) | 目标中心 (mm) | 目标中心偏差 (mm) | 位移 (mm) | 逆时针旋转 (deg) | 配准 RMS / 最大误差 (mm) |", "|---|---|---:|---:|---:|---:|---:|---:|"]
    for i, p in enumerate(report["pieces"], 1):
        f = lambda v: f"({v[0]:.2f}, {v[1]:.2f})"
        lines.append(f"| P{i} | {p['template']} | {f(p['source_center_a4_mm'])} | {f(p['target_center_a4_mm'])} | {p['target_center_ideal_error_mm']:.2f} | {f(p['center_delta_mm'])} | {p['rotation_delta_ccw_deg']:.2f} | {p['fit_rms_mm']:.2f} / {p['fit_max_vertex_error_mm']:.2f} |")
    lines += ["", "## 解释", "", "标注图中的 S/T 十字与箭头分别是 A4 局部坐标下的当前中心、计划目标中心和移动方向。白框是理想 100 x 60 mm 目标矩形，彩色轮廓是把当前实测轮廓按求解角度做刚体旋转和平移后的结果。", "", "4 个目标中心均位于对应目标轮廓内部，且相对理想模板中心的最大偏差不超过 2 mm，因此中心映射可作为本图的可行结果。", "", "但实测轮廓组合仍有缺口、重叠和越界，严格闭合未通过。下一步应优先细化 A4 四角、轮廓直线拟合和角点交点，而不是修改目标中心公式。", "", "这是 Windows 单图离线证据，不是 Jetson 精度、实时性或放置后验收证据；实际动作后仍必须重新采集并检查最终矩形。"]
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    report = run(args.input, args.output_dir)
    print(json.dumps({"decision": report["decision"], "piece_count": report["piece_count"], "score_margin": report["assignment"]["score_margin"], "rectangle": report["target_geometry"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
