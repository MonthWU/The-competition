#!/usr/bin/env python3
"""Generic offline card-piece reconstruction for 26E challenge task two.

The tool is intentionally Windows/offline friendly.  It does not use Jetson,
ROS, HMI, MCU, or mechanism state.  It reconstructs a four-piece rectangular
playing-card example by:

1. locating the green A4 plane,
2. extracting white rectangular card pieces from the upper half,
3. enumerating all 2x2 layouts and 0/180 degree orientations,
4. ranking candidates with generic card evidence:
   internal seam continuity, outer white-border quality, opposite-corner ink,
   and score margin.

It does not hardcode spade-2 semantics; the previous spade-2 photo is only a
regression sample for this generic reconstruction score.
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


def write_image(path: Path, image: np.ndarray, quality: int = 94) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()
    params = [cv2.IMWRITE_JPEG_QUALITY, quality] if suffix in {".jpg", ".jpeg"} else []
    if not cv2.imwrite(str(path), image, params):
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

    quad = order_quad(cv2.boxPoints(cv2.minAreaRect(contour)))
    destination = np.array(
        [[0, 0], [WARP_WIDTH - 1, 0], [WARP_WIDTH - 1, WARP_HEIGHT - 1], [0, WARP_HEIGHT - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(quad, destination)
    return quad, cv2.warpPerspective(image, transform, (WARP_WIDTH, WARP_HEIGHT))


def white_piece_mask(rectified: np.ndarray) -> np.ndarray:
    upper = rectified[: WARP_HEIGHT // 2]
    hsv = cv2.cvtColor(upper, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(upper, cv2.COLOR_BGR2Lab)
    mask = ((hsv[:, :, 1] < 85) & (hsv[:, :, 2] > 75) & (lab[:, :, 0] > 80)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=1)


def extract_patch(rectified: np.ndarray, box: np.ndarray) -> np.ndarray:
    source = order_quad(box)
    destination = np.array(
        [[0, 0], [CELL_WIDTH - 1, 0], [CELL_WIDTH - 1, CELL_HEIGHT - 1], [0, CELL_HEIGHT - 1]],
        dtype=np.float32,
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(rectified, transform, (CELL_WIDTH, CELL_HEIGHT))


def detect_pieces(rectified: np.ndarray) -> list[dict[str, Any]]:
    contours, _ = cv2.findContours(white_piece_mask(rectified), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pieces: list[dict[str, Any]] = []
    for contour in contours:
        area_px = float(cv2.contourArea(contour))
        if area_px < 3000.0 or area_px > 80000.0:
            continue
        rect = cv2.minAreaRect(contour)
        (center_x, center_y), (width, height), angle = rect
        if min(width, height) < 40.0 or max(width, height) < 80.0:
            continue
        box = cv2.boxPoints(rect).astype(np.float32)
        pieces.append(
            {
                "source_index": len(pieces) + 1,
                "center_px": [float(center_x), float(center_y)],
                "size_px": [float(width), float(height)],
                "angle_deg": float(angle),
                "area_px": area_px,
                "box_px": box.tolist(),
                "patch": extract_patch(rectified, box),
            }
        )
    pieces.sort(key=lambda item: item["center_px"][0])
    for index, piece in enumerate(pieces, 1):
        piece["piece_id"] = index
    if len(pieces) != 4:
        raise RuntimeError(f"white card piece count {len(pieces)} != 4")
    return pieces


def ink_mask(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = ((gray < 105) & (hsv[:, :, 1] > 25)).astype(np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8), iterations=1)


def gradient_image(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, 3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, 3)
    grad = cv2.magnitude(gx, gy)
    return np.clip(grad / 255.0, 0.0, 1.0)


def band_profile(values: np.ndarray, side: str, band: int) -> np.ndarray:
    if side == "left":
        return values[:, :band].mean(axis=1)
    if side == "right":
        return values[:, -band:].mean(axis=1)
    if side == "top":
        return values[:band, :].mean(axis=0)
    if side == "bottom":
        return values[-band:, :].mean(axis=0)
    raise ValueError(side)


def zncc(first: np.ndarray, second: np.ndarray) -> float:
    first_centered = first.astype(np.float32) - float(np.mean(first))
    second_centered = second.astype(np.float32) - float(np.mean(second))
    denominator = float(np.linalg.norm(first_centered) * np.linalg.norm(second_centered))
    if denominator < 1e-6:
        return 1.0 - min(1.0, abs(float(np.mean(first)) - float(np.mean(second))))
    return float(np.clip(np.dot(first_centered, second_centered) / denominator, -1.0, 1.0))


def aligned_profile_score(first: dict[str, np.ndarray], second: dict[str, np.ndarray]) -> dict[str, float]:
    best: dict[str, float] | None = None
    for shift in range(-8, 9):
        if shift < 0:
            lhs = {key: value[-shift:] for key, value in first.items()}
            length = len(next(iter(lhs.values())))
            rhs = {key: value[:length] for key, value in second.items()}
        elif shift > 0:
            lhs = {key: value[:-shift] for key, value in first.items()}
            rhs = {key: value[shift:] for key, value in second.items()}
        else:
            lhs = first
            rhs = second
        if len(next(iter(lhs.values()))) == 0:
            continue
        ink_overlap = float(np.mean(np.minimum(lhs["ink"], rhs["ink"])))
        ink_unmatched = float(np.mean(np.abs(lhs["ink"] - rhs["ink"])))
        gradient_similarity = 0.5 * (zncc(lhs["gradient"], rhs["gradient"]) + 1.0)
        luma_similarity = 1.0 - float(np.mean(np.abs(lhs["luma"] - rhs["luma"])))
        evidence = max(
            float(np.mean(lhs["ink"])),
            float(np.mean(rhs["ink"])),
            float(np.mean(lhs["gradient"])),
            float(np.mean(rhs["gradient"])),
        )
        raw_match = (
            0.35 * ink_overlap
            + 0.25 * (1.0 - ink_unmatched)
            + 0.25 * gradient_similarity
            + 0.15 * luma_similarity
        )
        # Blank seams are acceptable but should not outrank seams with real
        # pattern evidence.  The evidence multiplier keeps both behaviors.
        score = raw_match * (0.35 + min(0.65, 3.0 * evidence))
        candidate = {
            "score": float(np.clip(score, 0.0, 1.0)),
            "raw_match": float(np.clip(raw_match, 0.0, 1.0)),
            "evidence": float(np.clip(evidence, 0.0, 1.0)),
            "ink_overlap": ink_overlap,
            "ink_unmatched": ink_unmatched,
            "gradient_similarity": gradient_similarity,
            "luma_similarity": luma_similarity,
            "shift_px": float(shift),
        }
        if best is None or candidate["score"] > best["score"]:
            best = candidate
    if best is None:
        raise RuntimeError("empty seam profile")
    return best


def side_descriptor(image: np.ndarray, side: str) -> dict[str, np.ndarray]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    return {
        "ink": band_profile(ink_mask(image).astype(np.float32), side, 12),
        "gradient": band_profile(gradient_image(image), side, 12),
        "luma": band_profile(gray, side, 12),
    }


def seam_score(left: np.ndarray, left_side: str, right: np.ndarray, right_side: str) -> dict[str, float]:
    return aligned_profile_score(side_descriptor(left, left_side), side_descriptor(right, right_side))


def outer_border_score(canvas: np.ndarray) -> dict[str, Any]:
    hsv = cv2.cvtColor(canvas, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(canvas, cv2.COLOR_BGR2Lab)
    ink = ink_mask(canvas)
    white_like = ((hsv[:, :, 1] < 90) & (hsv[:, :, 2] > 70) & (lab[:, :, 0] > 78) & (ink == 0)).astype(np.float32)
    band = 8
    sides = {
        "top": float(white_like[:band, :].mean()),
        "bottom": float(white_like[-band:, :].mean()),
        "left": float(white_like[:, :band].mean()),
        "right": float(white_like[:, -band:].mean()),
    }
    return {"score": float(np.mean(list(sides.values()))), "sides": sides}


def corner_pair_score(canvas: np.ndarray) -> dict[str, Any]:
    ink = ink_mask(canvas)
    corner_h = CELL_HEIGHT // 2
    corner_w = CELL_WIDTH // 2
    corners = {
        "top_left": float(ink[:corner_h, :corner_w].mean()),
        "top_right": float(ink[:corner_h, -corner_w:].mean()),
        "bottom_left": float(ink[-corner_h:, :corner_w].mean()),
        "bottom_right": float(ink[-corner_h:, -corner_w:].mean()),
    }
    diag_a = corners["top_left"] + corners["bottom_right"]
    diag_b = corners["top_right"] + corners["bottom_left"]
    off_a = corners["top_right"] + corners["bottom_left"]
    off_b = corners["top_left"] + corners["bottom_right"]
    score_a = diag_a - 0.6 * off_a
    score_b = diag_b - 0.6 * off_b
    return {
        "score": float(max(score_a, score_b)),
        "preferred_diagonal": "top_left_bottom_right" if score_a >= score_b else "top_right_bottom_left",
        "corners": corners,
        "corner_evidence": float(max(diag_a, diag_b)),
    }


def make_canvas(cells: list[np.ndarray]) -> np.ndarray:
    return np.vstack([np.hstack([cells[0], cells[1]]), np.hstack([cells[2], cells[3]])])


def score_layout(pieces: list[dict[str, Any]], permutation: tuple[int, ...], rotation_bits: int) -> dict[str, Any]:
    cells: list[np.ndarray] = []
    for slot, piece_index in enumerate(permutation):
        patch = pieces[piece_index]["patch"]
        if (rotation_bits >> slot) & 1:
            patch = cv2.rotate(patch, cv2.ROTATE_180)
        cells.append(patch)

    seams = {
        "top_vertical": seam_score(cells[0], "right", cells[1], "left"),
        "bottom_vertical": seam_score(cells[2], "right", cells[3], "left"),
        "left_horizontal": seam_score(cells[0], "bottom", cells[2], "top"),
        "right_horizontal": seam_score(cells[1], "bottom", cells[3], "top"),
    }
    seam_values = [item["score"] for item in seams.values()]
    seam_evidence_values = [item["evidence"] for item in seams.values()]
    seam_average = float(np.mean(seam_values))
    seam_evidence = float(np.mean(seam_evidence_values))
    seam_min = float(np.min(seam_values))

    canvas = make_canvas(cells)
    border = outer_border_score(canvas)
    corner = corner_pair_score(canvas)
    score = (
        4.0 * seam_average
        + 2.0 * border["score"]
        + 1.5 * max(0.0, corner["score"])
        + 1.0 * seam_evidence
        + 0.5 * seam_min
    )
    return {
        "score": float(score),
        "permutation": [pieces[index]["piece_id"] for index in permutation],
        "rotated_180_slots": [slot for slot in range(4) if (rotation_bits >> slot) & 1],
        "seams": seams,
        "seam_average": seam_average,
        "seam_min": seam_min,
        "seam_evidence": seam_evidence,
        "outer_border": border,
        "corner_pair": corner,
        "canvas": canvas,
    }


def enumerate_layouts(pieces: list[dict[str, Any]]) -> list[dict[str, Any]]:
    candidates = [
        score_layout(pieces, permutation, rotation_bits)
        for permutation in itertools.permutations(range(4))
        for rotation_bits in range(16)
    ]
    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates


def draw_layout(canvas: np.ndarray, label: str) -> np.ndarray:
    output = canvas.copy()
    cv2.line(output, (CELL_WIDTH, 0), (CELL_WIDTH, 2 * CELL_HEIGHT - 1), (0, 255, 255), 2)
    cv2.line(output, (0, CELL_HEIGHT), (2 * CELL_WIDTH - 1, CELL_HEIGHT), (0, 255, 255), 2)
    cv2.putText(output, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return output


def draw_piece_overlay(rectified: np.ndarray, pieces: list[dict[str, Any]]) -> np.ndarray:
    canvas = rectified.copy()
    for piece in pieces:
        box = np.asarray(piece["box_px"], dtype=np.int32)
        center = tuple(int(round(value)) for value in piece["center_px"])
        cv2.polylines(canvas, [box], True, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(canvas, f"P{piece['piece_id']}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return canvas


def save_piece_sheet(pieces: list[dict[str, Any]], path: Path) -> None:
    sheet = np.full((260, len(pieces) * 190, 3), 255, np.uint8)
    for index, piece in enumerate(pieces):
        x = index * 190 + 15
        sheet[30:250, x : x + CELL_WIDTH] = piece["patch"]
        cv2.putText(sheet, f"P{piece['piece_id']}", (x + 45, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    write_image(path, sheet)


def strip_images(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return None
    if isinstance(value, dict):
        return {key: strip_images(item) for key, item in value.items() if key not in {"patch", "canvas"}}
    if isinstance(value, list):
        return [strip_images(item) for item in value]
    return value


def decision_for(best: dict[str, Any], second: dict[str, Any] | None) -> tuple[str, dict[str, bool], float]:
    margin = best["score"] - second["score"] if second else math.inf
    gates = {
        "piece_count_pass": True,
        "outer_border_pass": best["outer_border"]["score"] >= 0.72,
        "seam_average_pass": best["seam_average"] >= 0.36,
        "seam_evidence_pass": best["seam_evidence"] >= 0.05,
        "corner_or_pattern_pass": best["corner_pair"]["corner_evidence"] >= 0.08 or best["seam_evidence"] >= 0.10,
        "unique_layout_pass": margin >= 0.10,
    }
    required = [
        gates["piece_count_pass"],
        gates["outer_border_pass"],
        gates["seam_average_pass"],
        gates["seam_evidence_pass"],
        gates["corner_or_pattern_pass"],
    ]
    if all(required) and gates["unique_layout_pass"]:
        return "PASS_RECONSTRUCTED_UNIQUE", gates, margin
    if all(required):
        return "PASS_RECONSTRUCTED_AMBIGUOUS", gates, margin
    return "RETRY_DETECTION_OR_LAYOUT_SCORING", gates, margin


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
    decision, gates, margin = decision_for(best, second)

    write_image(output_dir / "rectified_a4.jpg", rectified, 96)
    write_image(output_dir / "white_piece_mask.png", white_piece_mask(rectified))
    write_image(output_dir / "piece_overlay.jpg", draw_piece_overlay(rectified, pieces), 94)
    save_piece_sheet(pieces, output_dir / "piece_sheet.jpg")
    for piece in pieces:
        write_image(output_dir / f"piece_{piece['piece_id']}.jpg", piece["patch"], 94)
    write_image(output_dir / "reconstructed_clean.jpg", best["canvas"], 95)
    write_image(output_dir / "reconstructed_overlay.jpg", draw_layout(best["canvas"], f"score {best['score']:.3f}"), 95)
    for index, candidate in enumerate(candidates[:candidate_count], 1):
        write_image(
            output_dir / f"candidate_{index:02d}.jpg",
            draw_layout(candidate["canvas"], f"rank {index} score {candidate['score']:.3f}"),
            95,
        )

    report: dict[str, Any] = {
        "input": str(input_path),
        "decision": decision,
        "jetson_used": False,
        "algorithm": {
            "name": "generic_rectangular_card_grid_reconstruct",
            "hardcoded_face": False,
            "grid": "2x2",
            "candidate_count_total": len(candidates),
            "candidate_generation": "all piece permutations and 0/180-degree orientations",
            "ranking_features": [
                "internal seam ink/gradient/luma continuity",
                "outer white-border quality",
                "opposite-corner ink distribution",
                "score margin for ambiguity",
            ],
        },
        "a4_quad_px": quad.tolist(),
        "piece_count": len(pieces),
        "layout_slots": ["top_left", "top_right", "bottom_left", "bottom_right"],
        "best_layout": strip_images(best),
        "second_layout": strip_images(second) if second else None,
        "score_margin": margin,
        "gates": gates,
        "pieces": strip_images(pieces),
        "top_candidates": [strip_images(candidate) for candidate in candidates[:candidate_count]],
        "outputs": {
            "rectified_a4": str(output_dir / "rectified_a4.jpg"),
            "piece_sheet": str(output_dir / "piece_sheet.jpg"),
            "reconstructed_clean": str(output_dir / "reconstructed_clean.jpg"),
            "reconstructed_overlay": str(output_dir / "reconstructed_overlay.jpg"),
            "report_json": str(output_dir / "report.json"),
            "report_md": str(output_dir / "report.md"),
        },
        "boundary": [
            "Windows offline single-image evidence only.",
            "No Jetson runtime, ROS, serial, HMI, MCU, or mechanism validation was performed.",
            "A low score margin means the face is reconstructed but some low-texture physical piece order may remain ambiguous.",
        ],
    }
    (output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# Generic Challenge-Two Reconstruction Report",
        "",
        f"- Decision: `{decision}`",
        f"- Jetson used: `{report['jetson_used']}`",
        f"- Hardcoded face semantics: `{report['algorithm']['hardcoded_face']}`",
        f"- Piece count: `{len(pieces)}`",
        f"- Best layout `[top_left, top_right, bottom_left, bottom_right]`: `{best['permutation']}`",
        f"- Best 180-degree rotated slots: `{best['rotated_180_slots']}`",
        f"- Score / margin: `{best['score']:.6f}` / `{margin:.6f}`",
        f"- Seam average / min / evidence: `{best['seam_average']:.6f}` / `{best['seam_min']:.6f}` / `{best['seam_evidence']:.6f}`",
        f"- Outer border score: `{best['outer_border']['score']:.6f}`",
        f"- Corner-pair score: `{best['corner_pair']['score']:.6f}`",
        "",
        "## Gates",
        "",
    ]
    for name, value in gates.items():
        lines.append(f"- `{name}`: `{value}`")
    lines += [
        "",
        "## Outputs",
        "",
        f"- Reconstructed clean: `{report['outputs']['reconstructed_clean']}`",
        f"- Reconstructed overlay: `{report['outputs']['reconstructed_overlay']}`",
        f"- Piece sheet: `{report['outputs']['piece_sheet']}`",
        "",
        "## Boundary",
        "",
        "This is a generic card-piece reconstruction score, not a Jetson or mechanism acceptance result.",
    ]
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"decision": decision, "best_layout": best["permutation"], "score": best["score"], "score_margin": margin}, ensure_ascii=False))
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
