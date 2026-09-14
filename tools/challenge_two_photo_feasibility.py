#!/usr/bin/env python3
"""Evaluate one photo against 26E challenge-task-two gates.

The tool measures current polygon centres, official/current edge gates, and
interior pattern evidence.  It never uses the basic-task target templates.
With Jetson solver results supplied, it also renders the safe no-target result.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import cv2
import numpy as np


WARP_WIDTH = 840
WARP_HEIGHT = 1188
PX_PER_MM = 4.0
OFFICIAL_MIN_EDGE_MM = 20.0


def text(image: np.ndarray, value: str, point: tuple[int, int], color: tuple[int, int, int], scale: float = 0.55) -> None:
    cv2.putText(image, value, point, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, value, point, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def cross(image: np.ndarray, point: tuple[int, int], color: tuple[int, int, int], size: int = 18) -> None:
    cv2.drawMarker(image, point, color, cv2.MARKER_CROSS, size, 3, cv2.LINE_AA)


def edge_lengths(polygon: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.roll(polygon, -1, axis=0) - polygon, axis=1)


def project(points_mm: np.ndarray, a4_to_image: np.ndarray) -> np.ndarray:
    values = (points_mm * PX_PER_MM).astype(np.float32).reshape(1, -1, 2)
    return cv2.perspectiveTransform(values, a4_to_image)[0]


def read_bool_parameter(path: Path, name: str) -> bool:
    match = re.search(rf"^\s*{re.escape(name)}\s*:\s*(true|false)\s*$", path.read_text(encoding="utf-8"), re.MULTILINE | re.IGNORECASE)
    if not match:
        raise ValueError(f"parameter {name} not found in {path}")
    return match.group(1).lower() == "true"


def read_float_parameter(path: Path, name: str) -> float:
    match = re.search(rf"^\s*{re.escape(name)}\s*:\s*([-+0-9.eE]+)\s*$", path.read_text(encoding="utf-8"), re.MULTILINE)
    if not match:
        raise ValueError(f"parameter {name} not found in {path}")
    return float(match.group(1))


def measure_piece_texture(rectified: np.ndarray, polygon_mm: np.ndarray) -> dict[str, float | bool | int]:
    polygon_px = np.round(polygon_mm * PX_PER_MM).astype(np.int32)
    mask = np.zeros(rectified.shape[:2], np.uint8)
    cv2.fillPoly(mask, [polygon_px], 255)
    # Ignore an approximately 2 mm boundary band so silhouette edges do not
    # masquerade as playing-card face texture.
    mask = cv2.erode(mask, np.ones((17, 17), np.uint8), iterations=1)
    valid = mask > 0
    if not np.any(valid):
        return {"interior_pixel_count": 0, "gray_mean": 0.0, "gray_std": 0.0, "dark_pixel_ratio": 0.0, "saturated_pixel_ratio": 0.0, "strong_gradient_ratio": 0.0, "pattern_evidence_present": False}
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    gradient = cv2.magnitude(
        cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3),
        cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3),
    )
    dark_ratio = float(np.mean(gray[valid] < 160))
    saturated_ratio = float(np.mean(hsv[:, :, 1][valid] > 80))
    gradient_ratio = float(np.mean(gradient[valid] > 50.0))
    # Diagnostic only: task-3 production scoring still uses seam ZNCC/SSIM.
    pattern_present = dark_ratio >= 0.005 or saturated_ratio >= 0.005 or gradient_ratio >= 0.01
    return {
        "interior_pixel_count": int(np.sum(valid)),
        "gray_mean": float(np.mean(gray[valid])),
        "gray_std": float(np.std(gray[valid])),
        "dark_pixel_ratio": dark_ratio,
        "saturated_pixel_ratio": saturated_ratio,
        "strong_gradient_ratio": gradient_ratio,
        "pattern_evidence_present": pattern_present,
    }


def make_strict_overlay(
    original: np.ndarray,
    quad: np.ndarray,
    a4_to_image: np.ndarray,
    pieces: list[dict[str, Any]],
    output: Path,
) -> None:
    canvas = original.copy()
    colors = [(40, 80, 230), (230, 100, 40), (50, 175, 70), (190, 70, 190)]
    cv2.polylines(canvas, [np.round(quad).astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA)
    for index, piece in enumerate(pieces, 1):
        polygon = np.asarray(piece["source_polygon_mm"], dtype=np.float64)
        polygon_px = project(polygon, a4_to_image)
        center_px = project(np.asarray([piece["source_center_a4_mm"]]), a4_to_image)[0]
        color = colors[index - 1]
        cv2.polylines(canvas, [np.round(polygon_px).astype(np.int32)], True, color, 4, cv2.LINE_AA)
        center = tuple(np.round(center_px).astype(int))
        cross(canvas, center, color)
        texture = piece["texture"]
        official = "E20 PASS" if piece["official_edge_gate_pass"] else "E20 FAIL"
        pattern = "PAT PASS" if texture["pattern_evidence_present"] else "PAT ABSENT"
        text(canvas, f"P{index} {official} {pattern}", (center[0] + 10, center[1] - 12), color)
    text(canvas, "TASK3 STRICT: CALIBRATION_REQUIRED", (210, 275), (0, 0, 255), 0.75)
    text(canvas, "Pure-white pieces: no card-face texture evidence", (210, 305), (0, 255, 255), 0.58)
    cv2.imwrite(str(output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 94])


def make_texture_overlay(rectified: np.ndarray, pieces: list[dict[str, Any]], output: Path) -> None:
    canvas = rectified.copy()
    colors = [(40, 80, 230), (230, 100, 40), (50, 175, 70), (190, 70, 190)]
    for index, piece in enumerate(pieces, 1):
        polygon = np.round(np.asarray(piece["source_polygon_mm"]) * PX_PER_MM).astype(np.int32)
        center = tuple(np.round(np.asarray(piece["source_center_a4_mm"]) * PX_PER_MM).astype(int))
        cv2.polylines(canvas, [polygon], True, colors[index - 1], 4, cv2.LINE_AA)
        metrics = piece["texture"]
        text(canvas, f"P{index} gray_std={metrics['gray_std']:.2f}", (center[0] + 8, center[1] - 12), colors[index - 1])
        text(canvas, f"dark={100*metrics['dark_pixel_ratio']:.3f}% grad={100*metrics['strong_gradient_ratio']:.3f}%", (center[0] + 8, center[1] + 16), colors[index - 1], 0.48)
    text(canvas, "Interior texture diagnostic (2 mm boundary excluded)", (22, WARP_HEIGHT - 30), (255, 255, 255), 0.62)
    cv2.imwrite(str(output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 94])


def make_no_target_overlay(rectified: np.ndarray, solver: dict[str, Any], output: Path) -> None:
    canvas = rectified.copy()
    split = int(148.5 * PX_PER_MM)
    shade = canvas.copy()
    cv2.rectangle(shade, (0, split), (WARP_WIDTH - 1, WARP_HEIGHT - 1), (10, 10, 10), -1)
    canvas = cv2.addWeighted(shade, 0.70, canvas, 0.30, 0)
    cv2.line(canvas, (0, split), (WARP_WIDTH - 1, split), (0, 255, 255), 3, cv2.LINE_AA)
    strict = solver["official20_filtered"]
    current = solver["current5_all"]
    text(canvas, f"OFFICIAL20 {strict['input_piece_count']} pieces: {strict['status']}", (60, split + 65), (0, 0, 255), 0.68)
    text(canvas, f"CURRENT5 {current['input_piece_count']} pieces: {current['status']}", (60, split + 105), (0, 0, 255), 0.68)
    center = (WARP_WIDTH // 2, (split + WARP_HEIGHT) // 2)
    cv2.line(canvas, (center[0] - 95, center[1] - 95), (center[0] + 95, center[1] + 95), (0, 0, 255), 14, cv2.LINE_AA)
    cv2.line(canvas, (center[0] + 95, center[1] - 95), (center[0] - 95, center[1] + 95), (0, 0, 255), 14, cv2.LINE_AA)
    text(canvas, "NO TARGET CENTERS", (225, center[1] + 140), (255, 255, 255), 0.8)
    text(canvas, "NO VALID TEXTURED RECTANGLE", (145, center[1] + 185), (255, 255, 255), 0.76)
    cv2.imwrite(str(output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 94])


def run(args: argparse.Namespace) -> dict[str, Any]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    original = cv2.imread(str(args.input), cv2.IMREAD_COLOR)
    if original is None:
        raise FileNotFoundError(args.input)
    extraction = json.loads(args.extraction_report.read_text(encoding="utf-8"))
    quad = np.asarray(extraction["tuned_green"]["a4_quad_px"], dtype=np.float32)
    rectified_quad = np.asarray([[0, 0], [WARP_WIDTH - 1, 0], [WARP_WIDTH - 1, WARP_HEIGHT - 1], [0, WARP_HEIGHT - 1]], dtype=np.float32)
    image_to_a4 = cv2.getPerspectiveTransform(quad, rectified_quad)
    a4_to_image = cv2.getPerspectiveTransform(rectified_quad, quad)
    rectified = cv2.warpPerspective(original, image_to_a4, (WARP_WIDTH, WARP_HEIGHT))
    cv2.imwrite(str(args.output_dir / "rectified_source.jpg"), rectified, [cv2.IMWRITE_JPEG_QUALITY, 96])

    implementation_min_edge = read_float_parameter(args.competition_yaml, "challenge_two_min_piece_edge_mm")
    calibration_valid = read_bool_parameter(args.vision_system_yaml, "camera_calibration_valid")
    pieces = []
    for index, source in enumerate(extraction["pieces"], 1):
        polygon = np.asarray(source["source_polygon_mm"], dtype=np.float64)
        lengths = edge_lengths(polygon)
        center = np.asarray(source["source_center_a4_mm"], dtype=np.float64)
        clearance = float(cv2.pointPolygonTest(polygon.astype(np.float32), tuple(float(x) for x in center), True))
        texture = measure_piece_texture(rectified, polygon)
        pieces.append(
            {
                "piece_id": index,
                "source_center_a4_mm": source["source_center_a4_mm"],
                "source_polygon_mm": source["source_polygon_mm"],
                "center_inside": clearance >= 0.0,
                "center_boundary_clearance_mm": clearance,
                "edge_lengths_mm": lengths.tolist(),
                "minimum_edge_mm": float(np.min(lengths)),
                "official_edge_gate_pass": bool(np.min(lengths) >= OFFICIAL_MIN_EDGE_MM),
                "implementation_edge_gate_pass": bool(np.min(lengths) >= implementation_min_edge),
                "texture": texture,
            }
        )

    solver = None
    if args.solver_results and args.solver_results.exists():
        solver = json.loads(args.solver_results.read_text(encoding="utf-8"))
    report = {
        "input": str(args.input),
        "task": "challenge_two_patterned_geometry",
        "basic_template_used": False,
        "camera_calibration_valid": calibration_valid,
        "strict_entry_status": "CONTINUE" if calibration_valid else "CALIBRATION_REQUIRED",
        "official_minimum_edge_mm": OFFICIAL_MIN_EDGE_MM,
        "implementation_minimum_edge_mm": implementation_min_edge,
        "edge_gate_configuration_conflict": implementation_min_edge < OFFICIAL_MIN_EDGE_MM,
        "green_baseline_found": extraction["baseline_green"]["a4_found"],
        "green_photo_tuned_hsv": extraction["tuned_green"]["hsv"],
        "current_centers_valid": all(piece["center_inside"] for piece in pieces),
        "official_valid_piece_count": sum(piece["official_edge_gate_pass"] for piece in pieces),
        "implementation_valid_piece_count": sum(piece["implementation_edge_gate_pass"] for piece in pieces),
        "pattern_evidence_present": any(piece["texture"]["pattern_evidence_present"] for piece in pieces),
        "pieces": pieces,
        "jetson_solver": solver,
        "target_centers_available": False if not solver else any(solver[key]["placements"] for key in ("official20_filtered", "current5_all")),
        "rotation_commands_available": False if not solver else any(solver[key]["placements"] for key in ("official20_filtered", "current5_all")),
        "rectangle_reconstructed": False if not solver else any(solver[key]["solved"] for key in ("official20_filtered", "current5_all")),
        "control_output_allowed": False,
        "decision": "FAIL_CALIBRATION_PATTERN_AND_GEOMETRY_GATES",
    }
    (args.output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    make_strict_overlay(original, quad, a4_to_image, pieces, args.output_dir / "strict_gate_overlay.jpg")
    make_texture_overlay(rectified, pieces, args.output_dir / "texture_diagnostic.jpg")
    if solver:
        make_no_target_overlay(rectified, solver, args.output_dir / "no_valid_rectangle_overlay.jpg")

    lines = [
        "# 发挥题二实拍图可行性结果",
        "",
        f"- 总结论：**{report['decision']}**",
        "- 基础题模板：`未使用`",
        f"- 严格入口：`{report['strict_entry_status']}`",
        f"- 当前中心均在轮廓内部：`{report['current_centers_valid']}`",
        f"- 牌面纹理证据：`{report['pattern_evidence_present']}`",
        f"- 题面 20 mm 门控有效块数：`{report['official_valid_piece_count']}/4`",
        f"- 当前实现 {implementation_min_edge:g} mm 门控有效块数：`{report['implementation_valid_piece_count']}/4`",
        f"- 边长配置与题面冲突：`{report['edge_gate_configuration_conflict']}`",
    ]
    if solver:
        lines += [
            f"- Jetson 题面20 mm过滤结果：`{solver['official20_filtered']['status']}`",
            f"- Jetson 当前5 mm配置结果：`{solver['current5_all']['status']}`",
            "- 目标中心/旋转指令：`未生成`",
        ]
    lines += [
        "",
        "## 当前中心、边长与纹理",
        "",
        "| 块 | 当前中心 A4 mm | 中心余量 mm | 最短边 mm | 20 mm | 5 mm | 灰度标准差 | 暗纹比例 | 强梯度比例 | 牌面证据 |",
        "|---|---:|---:|---:|---|---|---:|---:|---:|---|",
    ]
    for piece in pieces:
        center = piece["source_center_a4_mm"]
        texture = piece["texture"]
        lines.append(
            f"| P{piece['piece_id']} | ({center[0]:.2f}, {center[1]:.2f}) | {piece['center_boundary_clearance_mm']:.2f} | "
            f"{piece['minimum_edge_mm']:.2f} | {'PASS' if piece['official_edge_gate_pass'] else 'FAIL'} | "
            f"{'PASS' if piece['implementation_edge_gate_pass'] else 'FAIL'} | {texture['gray_std']:.2f} | "
            f"{100*texture['dark_pixel_ratio']:.3f}% | {100*texture['strong_gradient_ratio']:.3f}% | "
            f"{'PASS' if texture['pattern_evidence_present'] else 'ABSENT'} |"
        )
    lines += [
        "",
        "## 判断",
        "",
        "当前中心点均可从照片轮廓稳定计算，但任务3在任何目标求解前就会因 `camera_calibration_valid=false` 返回 `CALIBRATION_REQUIRED`。此外，本图碎片内部接近均匀白色，没有可用于扑克牌接缝 ZNCC/SSIM 的牌面花纹。",
        "",
        "任务3当前实现使用 5 mm 轮廓短边阈值，本图四块都会进入；题面现场碎片下限仍是 20 mm，按题面只有三块有效。不能通过保留 5 mm 参数把本基础题照片声明为发挥题二样本。",
    ]
    if solver:
        lines += [
            "",
            "在 Jetson 上旁路标定和感知入口后，分别注入题面20 mm过滤轮廓和当前5 mm配置的全部轮廓，并携带真实矫正图。两次任务3通用求解均未产生 placements，因此没有目标中心、旋转角或可验证矩形。",
        ]
    lines += [
        "",
        "下一次必须换用真实扑克牌牌面碎片、有效实测标定和满足20 mm边长的照片，再检查接缝质量、唯一性分差以及最终矩形闭合。",
    ]
    (args.output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--extraction-report", type=Path, required=True)
    parser.add_argument("--vision-system-yaml", type=Path, required=True)
    parser.add_argument("--competition-yaml", type=Path, required=True)
    parser.add_argument("--solver-results", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    report = run(args)
    print(json.dumps({"decision": report["decision"], "strict_entry_status": report["strict_entry_status"], "pattern_evidence_present": report["pattern_evidence_present"], "target_centers_available": report["target_centers_available"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
