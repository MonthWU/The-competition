#!/usr/bin/env python3
"""Build a user-facing challenge-task-one report from one real photo.

The script only reuses the source polygons from the earlier extraction report.
It deliberately ignores all basic-template target fields and combines the
strict 20 mm input gate with the actual Jetson generic-solver results.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


MIN_EDGE_MM = 20.0
WARP_WIDTH = 840
WARP_HEIGHT = 1188
PX_PER_MM = 4.0


def label(image: np.ndarray, text: str, point: tuple[int, int], color: tuple[int, int, int], scale: float = 0.55) -> None:
    cv2.putText(image, text, point, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, point, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def marker(image: np.ndarray, point: tuple[int, int], color: tuple[int, int, int], size: int = 18) -> None:
    cv2.drawMarker(image, point, color, cv2.MARKER_CROSS, size, 3, cv2.LINE_AA)


def project(points_mm: np.ndarray, a4_to_image: np.ndarray) -> np.ndarray:
    points_px = (points_mm * PX_PER_MM).astype(np.float32).reshape(1, -1, 2)
    return cv2.perspectiveTransform(points_px, a4_to_image)[0]


def edge_lengths(polygon: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.roll(polygon, -1, axis=0) - polygon, axis=1)


def make_original_gate_overlay(
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
        lengths = edge_lengths(polygon)
        polygon_px = project(polygon, a4_to_image)
        color = colors[index - 1]
        cv2.polylines(canvas, [np.round(polygon_px).astype(np.int32)], True, color, 4, cv2.LINE_AA)
        shortest = int(np.argmin(lengths))
        first = tuple(np.round(polygon_px[shortest]).astype(int))
        second = tuple(np.round(polygon_px[(shortest + 1) % len(polygon_px)]).astype(int))
        if lengths[shortest] < MIN_EDGE_MM:
            cv2.line(canvas, first, second, (0, 0, 255), 9, cv2.LINE_AA)
        for edge_index, length in enumerate(lengths):
            midpoint = 0.5 * (polygon_px[edge_index] + polygon_px[(edge_index + 1) % len(polygon_px)])
            label(canvas, f"{length:.1f}", tuple(np.round(midpoint).astype(int)), (255, 255, 255), 0.48)
        center_px = project(np.asarray([piece["source_center_a4_mm"]], dtype=np.float64), a4_to_image)[0]
        center = tuple(np.round(center_px).astype(int))
        marker(canvas, center, color)
        gate = "PASS" if float(np.min(lengths)) >= MIN_EDGE_MM else "FAIL<20mm"
        label(canvas, f"P{index} min={np.min(lengths):.2f} {gate}", (center[0] + 10, center[1] - 12), color)
    label(canvas, "Challenge-1 strict input gate; red edge is invalid", (210, 292), (0, 255, 255), 0.62)
    cv2.imwrite(str(output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 94])


def make_no_target_overlay(
    original: np.ndarray,
    image_to_a4: np.ndarray,
    pieces: list[dict[str, Any]],
    strict: dict[str, Any],
    bypass: dict[str, Any],
    output: Path,
) -> None:
    rectified = cv2.warpPerspective(original, image_to_a4, (WARP_WIDTH, WARP_HEIGHT))
    colors = [(40, 80, 230), (230, 100, 40), (50, 175, 70), (190, 70, 190)]
    for index, piece in enumerate(pieces, 1):
        polygon_px = np.round(np.asarray(piece["source_polygon_mm"]) * PX_PER_MM).astype(np.int32)
        center = tuple(np.round(np.asarray(piece["source_center_a4_mm"]) * PX_PER_MM).astype(int))
        cv2.polylines(rectified, [polygon_px], True, colors[index - 1], 4, cv2.LINE_AA)
        marker(rectified, center, colors[index - 1])
        label(rectified, f"P{index} CURRENT", (center[0] + 10, center[1] - 10), colors[index - 1])

    split_y = int(148.5 * PX_PER_MM)
    cv2.line(rectified, (0, split_y), (WARP_WIDTH - 1, split_y), (0, 255, 255), 3, cv2.LINE_AA)
    shade = rectified.copy()
    cv2.rectangle(shade, (0, split_y), (WARP_WIDTH - 1, WARP_HEIGHT - 1), (15, 15, 15), -1)
    rectified = cv2.addWeighted(shade, 0.68, rectified, 0.32, 0)
    center = (WARP_WIDTH // 2, (split_y + WARP_HEIGHT) // 2)
    cv2.line(rectified, (center[0] - 90, center[1] - 90), (center[0] + 90, center[1] + 90), (0, 0, 255), 14, cv2.LINE_AA)
    cv2.line(rectified, (center[0] + 90, center[1] - 90), (center[0] - 90, center[1] + 90), (0, 0, 255), 14, cv2.LINE_AA)
    label(rectified, f"STRICT {strict['input_piece_count']} pieces: {strict['status']}", (70, split_y + 70), (0, 0, 255), 0.72)
    label(rectified, f"BYPASS 4 pieces: {bypass['status']}", (70, split_y + 110), (0, 0, 255), 0.72)
    label(rectified, "NO TARGET CENTERS", (230, center[1] + 135), (255, 255, 255), 0.8)
    label(rectified, "NO ROTATION COMMANDS", (190, center[1] + 180), (255, 255, 255), 0.8)
    cv2.imwrite(str(output), rectified, [cv2.IMWRITE_JPEG_QUALITY, 94])


def run(input_path: Path, extraction_path: Path, solver_path: Path, output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    original = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if original is None:
        raise FileNotFoundError(input_path)
    extraction = json.loads(extraction_path.read_text(encoding="utf-8"))
    solver = json.loads(solver_path.read_text(encoding="utf-8"))
    quad = np.asarray(extraction["tuned_green"]["a4_quad_px"], dtype=np.float32)
    rectified_quad = np.asarray(
        [[0, 0], [WARP_WIDTH - 1, 0], [WARP_WIDTH - 1, WARP_HEIGHT - 1], [0, WARP_HEIGHT - 1]],
        dtype=np.float32,
    )
    image_to_a4 = cv2.getPerspectiveTransform(quad, rectified_quad)
    a4_to_image = cv2.getPerspectiveTransform(rectified_quad, quad)

    pieces: list[dict[str, Any]] = []
    invalid_ids: list[int] = []
    for index, source in enumerate(extraction["pieces"], 1):
        polygon = np.asarray(source["source_polygon_mm"], dtype=np.float64)
        lengths = edge_lengths(polygon)
        center = np.asarray(source["source_center_a4_mm"], dtype=np.float64)
        center_clearance = float(
            cv2.pointPolygonTest(
                polygon.astype(np.float32),
                tuple(float(value) for value in center),
                True,
            )
        )
        valid = bool(np.all(lengths >= MIN_EDGE_MM))
        if not valid:
            invalid_ids.append(index)
        pieces.append(
            {
                "piece_id": index,
                "source_center_a4_mm": source["source_center_a4_mm"],
                "source_polygon_mm": source["source_polygon_mm"],
                "edge_lengths_mm": lengths.tolist(),
                "minimum_edge_mm": float(np.min(lengths)),
                "edge_gate_pass": valid,
                "current_center_inside": center_clearance >= 0.0,
                "current_center_boundary_clearance_mm": center_clearance,
            }
        )

    strict = solver["strict_filtered"]
    bypass = solver["bypass_all"]
    report = {
        "input": str(input_path),
        "task": "challenge_one_unknown_geometry",
        "basic_template_used": False,
        "green_baseline_found": extraction["baseline_green"]["a4_found"],
        "green_photo_tuned_hsv": extraction["tuned_green"]["hsv"],
        "current_piece_count": len(pieces),
        "minimum_edge_required_mm": MIN_EDGE_MM,
        "strict_edge_gate_pass": len(invalid_ids) == 0,
        "invalid_piece_ids": invalid_ids,
        "strict_valid_piece_count": sum(piece["edge_gate_pass"] for piece in pieces),
        "current_centers_valid": all(piece["current_center_inside"] for piece in pieces),
        "pieces": pieces,
        "jetson_solver": solver,
        "target_centers_available": False,
        "rotation_commands_available": False,
        "rectangle_reconstructed": False,
        "decision": "FAIL_INPUT_CONSTRAINT_AND_NO_VALID_RECTANGLE",
        "control_output_allowed": False,
        "next_tuning": [
            "Use a true challenge-one sample whose every physical edge is at least 20 mm.",
            "Re-sample the green A4 HSV/Lab values on the final Jetson camera mode.",
            "Refine A4 corners and polygon vertices before re-running the generic solver.",
            "Do not lower the official 20 mm gate merely to make this basic-task photo pass.",
        ],
    }
    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    make_original_gate_overlay(original, quad, a4_to_image, pieces, output_dir / "strict_edge_gate_overlay.jpg")
    make_no_target_overlay(original, image_to_a4, pieces, strict, bypass, output_dir / "no_valid_rectangle_overlay.jpg")

    lines = [
        "# 发挥题一实拍图可行性结果",
        "",
        f"- 总结论：**{report['decision']}**",
        "- 基础题模板：`未使用`",
        f"- 当前检测轮廓：`{len(pieces)}` 块；严格 20 mm 门控后：`{report['strict_valid_piece_count']}` 块",
        f"- 正式门控求解：`{strict['status']}`",
        f"- 绕过短边门控、注入全部 4 块：`{bypass['status']}`",
        f"- 当前中心均位于各自轮廓内部：`{report['current_centers_valid']}`",
        "- 目标中心：`未生成`；旋转指令：`未生成`；控制输出：`禁止`",
        "",
        "## 当前中心与边长门控",
        "",
        "| 块 | 当前中心 A4 mm | 中心边界余量 mm | 各边长度 mm | 最短边 mm | 20 mm 门控 |",
        "|---|---:|---:|---|---:|---|",
    ]
    for piece in pieces:
        center = piece["source_center_a4_mm"]
        lengths = ", ".join(f"{value:.2f}" for value in piece["edge_lengths_mm"])
        lines.append(
            f"| P{piece['piece_id']} | ({center[0]:.2f}, {center[1]:.2f}) | "
            f"{piece['current_center_boundary_clearance_mm']:.2f} | {lengths} | "
            f"{piece['minimum_edge_mm']:.2f} | {'PASS' if piece['edge_gate_pass'] else 'FAIL'} |"
        )
    lines += [
        "",
        "## 中心点、目标点和矩形判断",
        "",
        "当前中心来自照片轮廓矩，均可在图中标出；但 P3 最短边仅约 6.73 mm。发挥题一要求每条边至少 20 mm，因此正式感知链路必须删除该候选，不能把 4 块作为完整场景下发。",
        "",
        "严格过滤后的 3 块和绕过门控后的全部 4 块都在 Jetson aarch64 上调用了现有通用几何求解器，两次均返回 `NO_VALID_RECTANGLE`，placements 为空。因此不存在可核验的目标中心或旋转角，不能绘制或发送猜测目标点。",
        "",
        "这张照片对应基础题几何，其中物理短边本来就小于发挥题一的 20 mm 下限。继续降低短边阈值会改变题面约束，不是允许的调参方向。下一轮应更换为真正满足发挥题一尺寸限制的碎片照片，再沿相同报告流程验证。",
        "",
        "## 候选路径结论",
        "",
        "- 复用基础题四模板：能够人为得到目标矩形，但泄漏已知模板，不属于发挥题一，淘汰。",
        "- 通用边匹配、部分边匹配和矩形硬约束搜索：符合发挥题一现有实现，本次采用；对本图返回拒绝。",
        "- 放宽 20 mm 硬门控：可用于隔离诊断，但不能形成正式控制输出。",
    ]
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--extraction-report", type=Path, required=True)
    parser.add_argument("--solver-results", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    report = run(args.input, args.extraction_report, args.solver_results, args.output_dir)
    print(json.dumps({"decision": report["decision"], "invalid_piece_ids": report["invalid_piece_ids"], "strict_valid_piece_count": report["strict_valid_piece_count"], "target_centers_available": report["target_centers_available"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
