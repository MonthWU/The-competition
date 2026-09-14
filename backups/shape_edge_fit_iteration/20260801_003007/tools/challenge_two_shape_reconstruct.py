#!/usr/bin/env python3
"""Shape-oriented offline reconstruction diagnostics for challenge task two.

This companion tool handles the newer 3-piece examples that are not 2x2
same-rectangle cards.  It is a Windows-only iteration aid: locate the green A4,
segment card fragments across the full A4, preserve each fragment's observed
orientation, and produce a compact reconstruction candidate plus diagnostics.

It is intentionally conservative: the output is a reconstruction candidate, not
mechanism-ready coordinates or Jetson acceptance evidence.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np


WARP_WIDTH = 840
WARP_HEIGHT = 1188


def order_quad(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    sums = points.sum(axis=1)
    diffs = points[:, 0] - points[:, 1]
    return np.array(
        [points[np.argmin(sums)], points[np.argmax(diffs)], points[np.argmax(sums)], points[np.argmin(diffs)]],
        dtype=np.float32,
    )


def quad_score(points: np.ndarray) -> float:
    quad = order_quad(points)
    width = 0.5 * (np.linalg.norm(quad[1] - quad[0]) + np.linalg.norm(quad[2] - quad[3]))
    height = 0.5 * (np.linalg.norm(quad[3] - quad[0]) + np.linalg.norm(quad[2] - quad[1]))
    area = abs(float(cv2.contourArea(quad)))
    return area - 200000.0 * abs(height / max(1.0, width) - 297.0 / 210.0)


def choose_a4_quad(contour: np.ndarray) -> np.ndarray:
    approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True).reshape(-1, 2)
    if len(approx) == 4:
        return order_quad(approx)
    best_quad: np.ndarray | None = None
    best_score = -1e18
    for combo in itertools.combinations(range(len(approx)), 4):
        candidate = approx[list(combo)].astype(np.float32)
        hull = cv2.convexHull(candidate).reshape(-1, 2)
        if len(hull) != 4:
            continue
        score = quad_score(hull)
        if score > best_score:
            best_score = score
            best_quad = order_quad(hull)
    if best_quad is not None:
        return best_quad
    return order_quad(cv2.boxPoints(cv2.minAreaRect(contour)))


def locate_a4(image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([35, 25, 120]), np.array([75, 120, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((31, 31), np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((11, 11), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("green A4 contour not found")
    contour = max(contours, key=cv2.contourArea)
    quad = choose_a4_quad(contour)
    target = np.array(
        [[0, 0], [WARP_WIDTH - 1, 0], [WARP_WIDTH - 1, WARP_HEIGHT - 1], [0, WARP_HEIGHT - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(quad, target)
    return quad, cv2.warpPerspective(image, transform, (WARP_WIDTH, WARP_HEIGHT))


def card_fragment_mask(rectified: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(rectified, cv2.COLOR_BGR2Lab)
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    substrate = (((hue < 35) | (hue > 85) | (lab[:, :, 2] < 142)) & (value > 95) & (saturation < 160))
    print_marks = (((saturation > 75) & (value > 55)) | (gray < 90))
    mask = (substrate | print_marks).astype(np.uint8) * 255
    mask[:12, :] = 0
    mask[-12:, :] = 0
    mask[:, :12] = 0
    mask[:, -12:] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)


def detect_fragments(rectified: np.ndarray, max_pieces: int = 4) -> list[dict[str, Any]]:
    mask = card_fragment_mask(rectified)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    fragments: list[dict[str, Any]] = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = float(cv2.contourArea(contour))
        if area < 2500.0 or area > 160000.0:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if min(width, height) < 25 or max(width, height) < 60:
            continue
        if x <= 2 or y <= 2 or x + width >= WARP_WIDTH - 2 or y + height >= WARP_HEIGHT - 2:
            continue
        local_region = rectified[y : y + height, x : x + width]
        local_mask = np.zeros((height, width), np.uint8)
        shifted = contour - np.array([[[x, y]]])
        cv2.drawContours(local_mask, [shifted], -1, 255, thickness=cv2.FILLED)
        hsv = cv2.cvtColor(local_region, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(local_region, cv2.COLOR_BGR2Lab)
        gray = cv2.cvtColor(local_region, cv2.COLOR_BGR2GRAY)
        valid = local_mask > 0
        card_substrate = (
            (hsv[:, :, 1] < 95)
            & (hsv[:, :, 2] > 115)
            & ((hsv[:, :, 0] < 35) | (hsv[:, :, 0] > 85) | (lab[:, :, 2] < 145))
        )
        visible_print = ((hsv[:, :, 1] > 80) & (hsv[:, :, 2] > 55)) | (gray < 90)
        card_pixel_ratio = float(np.mean((card_substrate | visible_print)[valid])) if np.any(valid) else 0.0
        if card_pixel_ratio < 0.18:
            continue
        crop = rectified[y : y + height, x : x + width].copy()
        crop_mask = np.zeros((height, width), np.uint8)
        cv2.drawContours(crop_mask, [shifted], -1, 255, thickness=cv2.FILLED)
        crop[crop_mask == 0] = (0, 0, 0)
        fragments.append(
            {
                "piece_id": len(fragments) + 1,
                "area_px": area,
                "card_pixel_ratio": card_pixel_ratio,
                "bbox_px": [int(x), int(y), int(width), int(height)],
                "center_px": [float(x + 0.5 * width), float(y + 0.5 * height)],
                "contour": contour,
                "crop": crop,
                "mask": crop_mask,
            }
        )
        if len(fragments) >= max_pieces:
            break
    fragments.sort(key=lambda item: item["center_px"][0])
    for index, fragment in enumerate(fragments, 1):
        fragment["piece_id"] = index
    if len(fragments) < 2:
        raise RuntimeError(f"card fragment count {len(fragments)} < 2")
    return fragments


def compact_axis(intervals: list[tuple[int, int]], gap_px: int) -> dict[int, int]:
    order = sorted(range(len(intervals)), key=lambda index: intervals[index][0])
    offsets = {index: 0 for index in order}
    removed = 0
    current_end = intervals[order[0]][1]
    for previous, index in zip(order, order[1:]):
        start, end = intervals[index]
        gap = start - current_end
        if gap > gap_px:
            removed += gap - gap_px
        offsets[index] = -removed
        current_end = max(current_end, end)
    return offsets


def compact_reconstruction(fragments: list[dict[str, Any]], gap_px: int = 2) -> tuple[np.ndarray, list[dict[str, Any]]]:
    boxes = [fragment["bbox_px"] for fragment in fragments]
    x_offsets = compact_axis([(box[0], box[0] + box[2]) for box in boxes], gap_px)
    y_offsets = compact_axis([(box[1], box[1] + box[3]) for box in boxes], gap_px)
    placements: list[dict[str, Any]] = []
    min_x = min(box[0] + x_offsets[index] for index, box in enumerate(boxes))
    min_y = min(box[1] + y_offsets[index] for index, box in enumerate(boxes))
    max_x = max(box[0] + x_offsets[index] + box[2] for index, box in enumerate(boxes))
    max_y = max(box[1] + y_offsets[index] + box[3] for index, box in enumerate(boxes))
    canvas = np.full((max_y - min_y + 20, max_x - min_x + 20, 3), 245, np.uint8)
    for index, fragment in enumerate(fragments):
        x, y, width, height = fragment["bbox_px"]
        target_x = x + x_offsets[index] - min_x + 10
        target_y = y + y_offsets[index] - min_y + 10
        roi = canvas[target_y : target_y + height, target_x : target_x + width]
        mask = fragment["mask"] > 0
        roi[mask] = fragment["crop"][mask]
        placements.append(
            {
                "piece_id": fragment["piece_id"],
                "source_bbox_px": fragment["bbox_px"],
                "target_bbox_px": [int(target_x), int(target_y), int(width), int(height)],
                "delta_px": [int(target_x - x), int(target_y - y)],
            }
        )
    return canvas, placements


def draw_overlay(rectified: np.ndarray, fragments: list[dict[str, Any]]) -> np.ndarray:
    canvas = rectified.copy()
    for fragment in fragments:
        cv2.drawContours(canvas, [fragment["contour"]], -1, (0, 0, 255), 3)
        x, y, width, height = fragment["bbox_px"]
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (255, 0, 0), 2)
        cv2.putText(canvas, f"P{fragment['piece_id']}", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return canvas


def strip_images(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return None
    if isinstance(value, dict):
        return {key: strip_images(item) for key, item in value.items() if key not in {"contour", "crop", "mask"}}
    if isinstance(value, list):
        return [strip_images(item) for item in value]
    return value


def run(input_path: Path, output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(input_path)
    quad, rectified = locate_a4(image)
    fragments = detect_fragments(rectified)
    reconstruction, placements = compact_reconstruction(fragments)
    cv2.imwrite(str(output_dir / "rectified_a4.jpg"), rectified, [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(output_dir / "fragment_mask.png"), card_fragment_mask(rectified))
    cv2.imwrite(str(output_dir / "fragment_overlay.jpg"), draw_overlay(rectified, fragments), [cv2.IMWRITE_JPEG_QUALITY, 94])
    cv2.imwrite(str(output_dir / "shape_compact_reconstruction.jpg"), reconstruction, [cv2.IMWRITE_JPEG_QUALITY, 95])
    report = {
        "input": str(input_path),
        "decision": "CANDIDATE_SHAPE_COMPACTED_EDGE_CHAIN_PENDING",
        "jetson_used": False,
        "algorithm": {
            "name": "shape_compact_card_reconstruct",
            "hardcoded_face": False,
            "candidate_type": "source-orientation compacted fragment layout",
            "exact_edge_chain_solver": False,
        },
        "a4_quad_px": quad.tolist(),
        "piece_count": len(fragments),
        "fragments": strip_images(fragments),
        "placements": placements,
        "outputs": {
            "rectified_a4": str(output_dir / "rectified_a4.jpg"),
            "fragment_overlay": str(output_dir / "fragment_overlay.jpg"),
            "reconstruction": str(output_dir / "shape_compact_reconstruction.jpg"),
            "report_json": str(output_dir / "report.json"),
        },
        "boundary": [
            "Windows offline visual candidate only.",
            "This mode detects 3-piece/non-2x2 samples and compacts the observed layout; it does not yet prove a unique edge-chain reconstruction.",
        ],
    }
    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"decision": report["decision"], "piece_count": len(fragments), "output": report["outputs"]["reconstruction"]}, ensure_ascii=False))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    run(args.input, args.output_dir)


if __name__ == "__main__":
    main()
