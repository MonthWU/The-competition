#!/usr/bin/env python3
"""Reconstruct the challenge-two spade-2 example from one Windows photo.

This is an offline evidence tool.  It does not touch Jetson hardware or ROS:

green A4 plane -> perspective warp -> white card-quarter contours -> 2x2
layout enumeration -> black-ink seam/corner scoring -> visual report.
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


A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0
SCALE = 4.0
WARP_WIDTH = int(A4_WIDTH_MM * SCALE)
WARP_HEIGHT = int(A4_HEIGHT_MM * SCALE)
CELL_WIDTH = 160
CELL_HEIGHT = 220


def order_quad(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    sums = points.sum(axis=1)
    diffs = points[:, 0] - points[:, 1]
    return np.array(
        [
            points[np.argmin(sums)],
            points[np.argmax(diffs)],
            points[np.argmax(sums)],
            points[np.argmin(diffs)],
        ],
        dtype=np.float32,
    )


def write_jpeg(path: Path, image: np.ndarray, quality: int = 94) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, quality]):
        raise RuntimeError(f"failed to write {path}")


def locate_green_a4(image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([35, 35, 35]), np.array([95, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("green A4 contour not found")
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 0.10 * image.shape[0] * image.shape[1]:
        raise RuntimeError("green A4 contour is too small")
    rect = cv2.minAreaRect(contour)
    quad = order_quad(cv2.boxPoints(rect))
    destination = np.array(
        [[0, 0], [WARP_WIDTH - 1, 0], [WARP_WIDTH - 1, WARP_HEIGHT - 1], [0, WARP_HEIGHT - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(quad, destination)
    warped = cv2.warpPerspective(image, transform, (WARP_WIDTH, WARP_HEIGHT))
    return quad, warped


def white_piece_mask(rectified: np.ndarray) -> np.ndarray:
    upper = rectified[: WARP_HEIGHT // 2]
    hsv = cv2.cvtColor(upper, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(upper, cv2.COLOR_BGR2Lab)
    white = ((hsv[:, :, 1] < 85) & (hsv[:, :, 2] > 75) & (lab[:, :, 0] > 80)).astype(np.uint8) * 255
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)
    return white


def extract_piece_patch(rectified: np.ndarray, box: np.ndarray) -> np.ndarray:
    source = order_quad(box)
    destination = np.array(
        [[0, 0], [CELL_WIDTH - 1, 0], [CELL_WIDTH - 1, CELL_HEIGHT - 1], [0, CELL_HEIGHT - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(rectified, transform, (CELL_WIDTH, CELL_HEIGHT))


def detect_pieces(rectified: np.ndarray) -> list[dict[str, Any]]:
    mask = white_piece_mask(rectified)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[dict[str, Any]] = []
    for contour in contours:
        area_px = float(cv2.contourArea(contour))
        if area_px < 3000.0 or area_px > 80000.0:
            continue
        rect = cv2.minAreaRect(contour)
        (center_x, center_y), (width, height), angle = rect
        if min(width, height) < 40.0 or max(width, height) < 80.0:
            continue
        box = cv2.boxPoints(rect).astype(np.float32)
        patch = extract_piece_patch(rectified, box)
        candidates.append(
            {
                "source_index": len(candidates) + 1,
                "center_px": [float(center_x), float(center_y)],
                "size_px": [float(width), float(height)],
                "angle_deg": float(angle),
                "area_px": area_px,
                "box_px": box.tolist(),
                "patch": patch,
            }
        )
    candidates.sort(key=lambda item: item["center_px"][0])
    for index, candidate in enumerate(candidates, 1):
        candidate["piece_id"] = index
    if len(candidates) != 4:
        raise RuntimeError(f"white card piece count {len(candidates)} != 4")
    return candidates


def dark_ink_mask(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = ((gray < 105) & (hsv[:, :, 1] > 25)).astype(np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8), iterations=1)


def edge_profile(mask: np.ndarray, side: str) -> np.ndarray:
    band = 12
    if side == "left":
        roi = mask[:, :band]
        return roi.mean(axis=1)
    if side == "right":
        roi = mask[:, -band:]
        return roi.mean(axis=1)
    if side == "top":
        roi = mask[:band, :]
        return roi.mean(axis=0)
    if side == "bottom":
        roi = mask[-band:, :]
        return roi.mean(axis=0)
    raise ValueError(side)


def shifted_profile_match(first: np.ndarray, second: np.ndarray) -> float:
    best = 0.0
    for shift in range(-8, 9):
        if shift < 0:
            lhs = first[-shift:]
            rhs = second[: len(lhs)]
        elif shift > 0:
            lhs = first[:-shift]
            rhs = second[shift:]
        else:
            lhs = first
            rhs = second
        if len(lhs) == 0:
            continue
        overlap = float(np.mean(np.minimum(lhs, rhs)))
        presence = float(np.mean(np.maximum(lhs, rhs)))
        value = 2.5 * overlap - 0.7 * max(0.0, presence - overlap)
        best = max(best, value)
    return best


def corner_ink(canvas: np.ndarray) -> dict[str, float]:
    mask = dark_ink_mask(canvas)
    corner_h = CELL_HEIGHT // 2
    corner_w = CELL_WIDTH // 2
    return {
        "top_left": float(mask[:corner_h, :corner_w].mean()),
        "top_right": float(mask[:corner_h, -corner_w:].mean()),
        "bottom_left": float(mask[-corner_h:, :corner_w].mean()),
        "bottom_right": float(mask[-corner_h:, -corner_w:].mean()),
    }


def central_pip_score(canvas: np.ndarray) -> float:
    mask = dark_ink_mask(canvas)
    seam_band = mask[:, CELL_WIDTH - 22 : CELL_WIDTH + 22]
    upper = float(seam_band[60:190].mean())
    lower = float(seam_band[250:380].mean())
    return upper + lower


def make_canvas(cells: list[np.ndarray]) -> np.ndarray:
    return np.vstack([np.hstack([cells[0], cells[1]]), np.hstack([cells[2], cells[3]])])


def score_layout(pieces: list[dict[str, Any]], permutation: tuple[int, ...], rotation_bits: int) -> dict[str, Any]:
    cells: list[np.ndarray] = []
    masks: list[np.ndarray] = []
    for slot, piece_index in enumerate(permutation):
        patch = pieces[piece_index]["patch"]
        if (rotation_bits >> slot) & 1:
            patch = cv2.rotate(patch, cv2.ROTATE_180)
        cells.append(patch)
        masks.append(dark_ink_mask(patch))

    top_vertical = shifted_profile_match(edge_profile(masks[0], "right"), edge_profile(masks[1], "left"))
    bottom_vertical = shifted_profile_match(edge_profile(masks[2], "right"), edge_profile(masks[3], "left"))
    left_horizontal = shifted_profile_match(edge_profile(masks[0], "bottom"), edge_profile(masks[2], "top"))
    right_horizontal = shifted_profile_match(edge_profile(masks[1], "bottom"), edge_profile(masks[3], "top"))
    seam_score = top_vertical + bottom_vertical + 0.2 * (left_horizontal + right_horizontal)

    canvas = make_canvas(cells)
    corners = corner_ink(canvas)
    diagonal_top_left = (
        corners["top_left"]
        + corners["bottom_right"]
        - 0.6 * (corners["top_right"] + corners["bottom_left"])
    )
    diagonal_top_right = (
        corners["top_right"]
        + corners["bottom_left"]
        - 0.6 * (corners["top_left"] + corners["bottom_right"])
    )
    corner_score = max(diagonal_top_left, 0.8 * diagonal_top_right)
    pip_score = central_pip_score(canvas)
    total_score = 5.0 * seam_score + 3.0 * corner_score + 2.5 * pip_score
    return {
        "score": total_score,
        "seam_score": seam_score,
        "top_vertical_seam_score": top_vertical,
        "bottom_vertical_seam_score": bottom_vertical,
        "corner_score": corner_score,
        "pip_score": pip_score,
        "corner_ink": corners,
        "permutation": [pieces[index]["piece_id"] for index in permutation],
        "rotated_180_slots": [slot for slot in range(4) if (rotation_bits >> slot) & 1],
        "canvas": canvas,
    }


def enumerate_layouts(pieces: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for permutation in itertools.permutations(range(len(pieces))):
        for rotation_bits in range(1 << len(pieces)):
            candidates.append(score_layout(pieces, permutation, rotation_bits))
    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates


def draw_piece_overlay(rectified: np.ndarray, pieces: list[dict[str, Any]]) -> np.ndarray:
    canvas = rectified.copy()
    for piece in pieces:
        box = np.asarray(piece["box_px"], dtype=np.int32)
        center = tuple(int(round(value)) for value in piece["center_px"])
        cv2.polylines(canvas, [box], True, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(canvas, f"P{piece['piece_id']}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return canvas


def draw_layout(canvas: np.ndarray, label: str | None = None) -> np.ndarray:
    output = canvas.copy()
    cv2.line(output, (CELL_WIDTH, 0), (CELL_WIDTH, 2 * CELL_HEIGHT - 1), (0, 255, 255), 2)
    cv2.line(output, (0, CELL_HEIGHT), (2 * CELL_WIDTH - 1, CELL_HEIGHT), (0, 255, 255), 2)
    if label:
        cv2.putText(output, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return output


def save_piece_sheet(pieces: list[dict[str, Any]], path: Path) -> None:
    sheet = np.full((260, len(pieces) * 190, 3), 255, np.uint8)
    for index, piece in enumerate(pieces):
        x = index * 190 + 15
        sheet[30:250, x : x + CELL_WIDTH] = piece["patch"]
        cv2.putText(sheet, f"P{piece['piece_id']}", (x + 45, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    write_jpeg(path, sheet)


def strip_images(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return None
    if isinstance(value, dict):
        return {key: strip_images(item) for key, item in value.items() if key != "patch" and key != "canvas"}
    if isinstance(value, list):
        return [strip_images(item) for item in value]
    return value


def run(input_path: Path, output_dir: Path, candidate_count: int) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(input_path)

    quad, rectified = locate_green_a4(image)
    pieces = detect_pieces(rectified)
    candidates = enumerate_layouts(pieces)
    best = candidates[0]
    second = candidates[1] if len(candidates) > 1 else None
    score_margin = best["score"] - second["score"] if second else math.inf

    write_jpeg(output_dir / "rectified_a4.jpg", rectified, 96)
    cv2.imwrite(str(output_dir / "white_piece_mask.png"), white_piece_mask(rectified))
    write_jpeg(output_dir / "piece_overlay.jpg", draw_piece_overlay(rectified, pieces), 94)
    save_piece_sheet(pieces, output_dir / "piece_sheet.jpg")
    for piece in pieces:
        write_jpeg(output_dir / f"piece_{piece['piece_id']}.jpg", piece["patch"], 94)

    clean_layout = best["canvas"]
    write_jpeg(output_dir / "spade2_reconstructed_clean.jpg", clean_layout, 95)
    write_jpeg(
        output_dir / "spade2_reconstructed_overlay.jpg",
        draw_layout(clean_layout, f"score {best['score']:.3f}"),
        95,
    )
    for index, candidate in enumerate(candidates[:candidate_count], 1):
        write_jpeg(
            output_dir / f"candidate_{index:02d}.jpg",
            draw_layout(candidate["canvas"], f"rank {index} score {candidate['score']:.3f}"),
            95,
        )

    exact_layout_unique = score_margin >= 0.10
    reconstructed = (
        best["top_vertical_seam_score"] >= 0.25
        and best["bottom_vertical_seam_score"] >= 0.25
        and best["corner_score"] >= 0.12
        and best["pip_score"] >= 0.45
    )
    if reconstructed and exact_layout_unique:
        decision = "PASS_SPADE2_RECONSTRUCTED"
    elif reconstructed:
        decision = "PASS_SPADE2_RECONSTRUCTED_WITH_INTERCHANGEABLE_ORDER"
    else:
        decision = "RETRY_DETECTION_OR_LAYOUT_SCORING"

    report: dict[str, Any] = {
        "input": str(input_path),
        "decision": decision,
        "jetson_used": False,
        "a4_quad_px": quad.tolist(),
        "piece_count": len(pieces),
        "layout_slots": ["top_left", "top_right", "bottom_left", "bottom_right"],
        "best_layout": strip_images(best),
        "second_layout": strip_images(second) if second else None,
        "score_margin": score_margin,
        "exact_layout_unique": exact_layout_unique,
        "spade2_evidence": {
            "top_vertical_seam_pass": best["top_vertical_seam_score"] >= 0.25,
            "bottom_vertical_seam_pass": best["bottom_vertical_seam_score"] >= 0.25,
            "opposite_corner_indices_pass": best["corner_score"] >= 0.12,
            "two_central_pips_pass": best["pip_score"] >= 0.45,
        },
        "pieces": strip_images(pieces),
        "outputs": {
            "rectified_a4": str(output_dir / "rectified_a4.jpg"),
            "piece_sheet": str(output_dir / "piece_sheet.jpg"),
            "reconstructed_clean": str(output_dir / "spade2_reconstructed_clean.jpg"),
            "reconstructed_overlay": str(output_dir / "spade2_reconstructed_overlay.jpg"),
            "report_json": str(output_dir / "report.json"),
            "report_md": str(output_dir / "report.md"),
        },
        "notes": [
            "Windows offline evidence only; no Jetson, ROS, HMI, MCU, or mechanism validation.",
            "When the score margin is small, visually equivalent blank/low-texture halves may be interchangeable even though the spade-2 face is reconstructed.",
        ],
    }
    candidate_report = [strip_images(candidate) for candidate in candidates[:candidate_count]]
    report["top_candidates"] = candidate_report
    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# Challenge Two Spade-2 Offline Reconstruction",
        "",
        f"- Decision: `{decision}`",
        f"- Jetson used: `{report['jetson_used']}`",
        f"- Piece count: `{report['piece_count']}`",
        f"- Best layout slots `[top_left, top_right, bottom_left, bottom_right]`: `{best['permutation']}`",
        f"- Best 180-degree rotated slots: `{best['rotated_180_slots']}`",
        f"- Score / margin: `{best['score']:.6f}` / `{score_margin:.6f}`",
        f"- Top/bottom vertical seam scores: `{best['top_vertical_seam_score']:.6f}` / `{best['bottom_vertical_seam_score']:.6f}`",
        f"- Opposite-corner score: `{best['corner_score']:.6f}`",
        f"- Central-pip score: `{best['pip_score']:.6f}`",
        "",
        "## Outputs",
        "",
        f"- Rectified A4: `{report['outputs']['rectified_a4']}`",
        f"- Piece sheet: `{report['outputs']['piece_sheet']}`",
        f"- Reconstructed clean: `{report['outputs']['reconstructed_clean']}`",
        f"- Reconstructed overlay: `{report['outputs']['reconstructed_overlay']}`",
        "",
        "## Boundary",
        "",
        "This proves only that the captured Windows-side still image can be segmented and reassembled into a black spade-2 face. It is not Jetson runtime, ROS, serial, HMI, MCU, or mechanism acceptance evidence.",
    ]
    if reconstructed and not exact_layout_unique:
        lines.extend(
            [
                "",
                "The best two candidates are close because one pair carries very weak distinguishing texture. The reconstructed card face is still a black spade 2; exact physical ordering of the low-texture halves should be rechecked with a sharper capture before control output.",
            ]
        )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"decision": decision, "best_layout": best["permutation"], "score": best["score"], "score_margin": score_margin}, ensure_ascii=False))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--candidate-count", type=int, default=8)
    args = parser.parse_args()
    run(args.input, args.output_dir, max(1, args.candidate_count))


if __name__ == "__main__":
    main()
