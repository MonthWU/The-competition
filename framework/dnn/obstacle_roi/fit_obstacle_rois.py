import argparse
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "source_annotations.json"
OUT = ROOT / "_fit_output"


def signed_area(points):
    p = np.asarray(points, dtype=np.float64)
    return 0.5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]))


def canonical_polygon(points):
    p = np.asarray(points, dtype=np.float64)
    hull = cv2.convexHull(p.astype(np.float32), clockwise=False, returnPoints=True).reshape(-1, 2).astype(np.float64)
    if len(hull) not in (3, 4):
        raise ValueError(f"Expected a convex triangle or quadrilateral, got {len(hull)} hull vertices: {points}")
    center = hull.mean(axis=0)
    order = np.argsort(np.arctan2(hull[:, 1] - center[1], hull[:, 0] - center[0]))
    q = hull[order]
    # In image coordinates (y grows downward), visually counter-clockwise is negative shoelace area.
    if signed_area(q) > 0:
        q = q[::-1]
    # Deterministic start: smallest y, then smallest x.
    start = min(range(len(q)), key=lambda i: (q[i, 1], q[i, 0]))
    return np.roll(q, -start, axis=0)


def best_align(reference, candidate):
    ref = canonical_polygon(reference)
    cand = canonical_polygon(candidate)
    if len(ref) != len(cand):
        triangle, quad = (ref, cand) if len(ref) == 3 else (cand, ref)
        edge_lengths = np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)
        shortest = int(np.argmin(edge_lengths))
        if edge_lengths[shortest] > 0.15 * float(edge_lengths.sum()):
            raise ValueError("Cannot pair a triangle with a quadrilateral without a short edge")
        midpoint = (quad[shortest] + quad[(shortest + 1) % 4]) / 2.0
        vertex = int(np.argmin(np.linalg.norm(triangle - midpoint, axis=1)))
        expanded = np.insert(triangle, vertex, triangle[vertex], axis=0)
        if len(ref) == 3:
            ref = expanded
        else:
            cand = expanded
    options = []
    for base in (cand, cand[::-1]):
        for shift in range(len(cand)):
            q = np.roll(base, shift, axis=0)
            rms = math.sqrt(float(np.mean(np.sum((ref - q) ** 2, axis=1))))
            options.append((rms, q))
    return ref, min(options, key=lambda item: item[0])


def polygon_area(points):
    return int(round(abs(signed_area(points))))


def center(points):
    return np.asarray(points, dtype=np.float64).mean(axis=0)


def intersection_area(a, b):
    qa = cv2.convexHull(np.asarray(a, dtype=np.float32)).reshape(-1, 2)
    qb = cv2.convexHull(np.asarray(b, dtype=np.float32)).reshape(-1, 2)
    area, _ = cv2.intersectConvexConvex(qa, qb)
    return float(area)


def list_overlaps(items, threshold=1.0):
    overlaps = []
    for x, y in itertools.combinations(items, 2):
        area = intersection_area(x["pts"], y["pts"])
        if area >= threshold:
            overlaps.append({
                "labels": [x["label"], y["label"]],
                "intersection_px2": round(area, 1),
                "fraction_of_smaller": round(area / min(x["w"], y["w"]), 4),
            })
    return overlaps


def clip_halfplane(points, midpoint, normal, margin_px=1.0):
    """Keep dot(point-midpoint, normal) <= -margin/2 using convex clipping."""
    poly = [np.asarray(p, dtype=np.float64) for p in points]
    unit = normal / np.linalg.norm(normal)

    def value(p):
        return float(np.dot(p - midpoint, unit) + margin_px / 2.0)

    out = []
    for start, end in zip(poly, poly[1:] + poly[:1]):
        vs, ve = value(start), value(end)
        inside_s, inside_e = vs <= 0.0, ve <= 0.0
        if inside_s and inside_e:
            out.append(end)
        elif inside_s and not inside_e:
            t = vs / (vs - ve)
            out.append(start + t * (end - start))
        elif not inside_s and inside_e:
            t = vs / (vs - ve)
            out.append(start + t * (end - start))
            out.append(end)
    return out


def reduce_convex_to_runtime_polygon(points):
    pts = [np.asarray(p, dtype=np.float64) for p in points]
    while len(pts) > 4:
        costs = []
        for i in range(len(pts)):
            prev = pts[(i - 1) % len(pts)]
            cur = pts[i]
            nxt = pts[(i + 1) % len(pts)]
            a = cur - prev
            b = nxt - cur
            costs.append(abs(float(a[0] * b[1] - a[1] * b[0])))
        del pts[int(np.argmin(costs))]
    if len(pts) not in (3, 4):
        raise ValueError(f"Topology clipping produced {len(pts)} vertices")
    return canonical_polygon(pts)


def cleanup_overlaps(items):
    """Split a narrow overlap along its own long axis, preserving both ROIs."""
    original_areas = {item["label"]: item["w"] for item in items}
    affected = set()
    for _ in range(8):
        overlaps = list_overlaps(items)
        if not overlaps:
            break
        by_label = {item["label"]: item for item in items}
        for overlap in overlaps:
            la, lb = overlap["labels"]
            a, b = by_label[la], by_label[lb]
            pa = np.asarray(a["pts"], dtype=np.float64)
            pb = np.asarray(b["pts"], dtype=np.float64)
            ca, cb = pa.mean(axis=0), pb.mean(axis=0)
            if overlap["fraction_of_smaller"] > 0.1:
                raise ValueError(f"ROI overlap too large to resolve automatically: {la}/{lb}")
            _, intersection = cv2.intersectConvexConvex(
                pa.astype(np.float32), pb.astype(np.float32)
            )
            boundary = intersection.reshape(-1, 2).astype(np.float64)
            midpoint = boundary.mean(axis=0)
            centered = boundary - midpoint
            eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered)
            axis = eigenvectors[:, int(np.argmax(eigenvalues))]
            normal = np.array([-axis[1], axis[0]], dtype=np.float64)
            if np.dot(cb - ca, normal) < 0:
                normal = -normal
            clipped_a = clip_halfplane(pa, midpoint, normal)
            clipped_b = clip_halfplane(pb, midpoint, -normal)
            a["pts"] = np.rint(reduce_convex_to_runtime_polygon(clipped_a)).astype(int).tolist()
            b["pts"] = np.rint(reduce_convex_to_runtime_polygon(clipped_b)).astype(int).tolist()
            a["w"] = polygon_area(a["pts"])
            b["w"] = polygon_area(b["pts"])
            affected.update((la, lb))
    return {
        label: round(next(item["w"] for item in items if item["label"] == label) / original_areas[label], 4)
        for label in sorted(affected, key=int)
    }


def fit_angle(angle, group_a, group_b):
    by_a = {str(item["label"]): item for item in group_a}
    by_b = {str(item["label"]): item for item in group_b}
    labels = sorted(set(by_a) | set(by_b), key=int)
    fitted = []
    diagnostics = []
    for label in labels:
        a = by_a.get(label)
        b = by_b.get(label)
        if a and b:
            topology_mismatch = len(canonical_polygon(a["pts"])) != len(canonical_polygon(b["pts"]))
            qa, (pair_rms, qb) = best_align(a["pts"], b["pts"])
            fused = canonical_polygon((qa + qb) / 2.0)
            center_shift = float(np.linalg.norm(center(qa) - center(qb)))
            source_count = 2
            confidence = "medium" if topology_mismatch or max(pair_rms, center_shift) > 80 else "high"
        else:
            only = a or b
            fused = canonical_polygon(only["pts"])
            pair_rms = None
            center_shift = None
            source_count = 1
            confidence = "low"
            topology_mismatch = False
        rounded = np.rint(fused).astype(int)
        fitted.append({
            "n": len(fitted) + 1,
            "label": label,
            "pts": rounded.tolist(),
            "w": polygon_area(rounded),
            "source_count": source_count,
            "confidence": confidence,
            "topology_mismatch": topology_mismatch,
        })
        diagnostics.append({
            "label": label,
            "source_count": source_count,
            "corner_pair_rms_px": None if pair_rms is None else round(pair_rms, 2),
            "center_shift_px": None if center_shift is None else round(center_shift, 2),
            "confidence": confidence,
            "topology_mismatch": topology_mismatch,
        })

    overlaps_before = list_overlaps(fitted)
    area_retention = cleanup_overlaps(fitted)
    overlaps_after = list_overlaps(fitted)

    payload = {
        "angle_deg": angle,
        "image_size": [1280, 720],
        "coordinate_space": "shared image pixels, x right, y down",
        "vertex_order": "visually counter-clockwise, deterministic top-most start",
        "method": "convex-hull normalization; minimum-RMS cyclic corner pairing; pointwise mean for two samples; minimum center contraction for conflicting ROIs",
        "labels": fitted,
        "diagnostics": diagnostics,
        "topology_cleanup": {
            "method": "intersection long-axis clipping with 1 px separation; reject overlaps above 10 percent",
            "area_retention": area_retention,
            "overlaps_before": overlaps_before,
            "overlaps_after": overlaps_after
        },
        "missing_labels": [str(i) for i in range(1, 14) if str(i) not in labels],
    }
    return payload


def draw_overlay(image_path, payload, output_path):
    img = cv2.imread(str(image_path))
    if img is None:
        raise FileNotFoundError(image_path)
    palette = [(0,255,255),(0,200,0),(255,100,0),(255,0,255),(0,128,255),(255,255,0),(80,180,255),(180,80,255),(255,180,80)]
    for idx, item in enumerate(payload["labels"]):
        pts = np.asarray(item["pts"], dtype=np.int32).reshape(-1, 1, 2)
        color = palette[idx % len(palette)]
        cv2.polylines(img, [pts], True, color, 3, cv2.LINE_AA)
        c = np.rint(np.asarray(item["pts"]).mean(axis=0)).astype(int)
        cv2.putText(img, item["label"], tuple(c), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
    cv2.imwrite(str(output_path), img)


def make_overview(output_dir, output_path):
    rows = []
    for angle, names in (
        (0, ("0deg_on_angle_pic1.jpg", "0deg_on_angle_pic1b.jpg")),
        (45, ("45deg_on_angle_pic2.jpg", "45deg_on_angle_pic2b.jpg")),
        (90, ("90deg_on_angle_pic3.jpg", "90deg_on_angle_pic3b.jpg")),
    ):
        cells = []
        for group, name in enumerate(names, start=1):
            img = cv2.imread(str(output_dir / name))
            if img is None:
                raise FileNotFoundError(output_dir / name)
            img = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
            cell = np.zeros((400, 640, 3), dtype=np.uint8)
            cell[40:, :] = img
            cv2.putText(
                cell,
                f"{angle} deg / group {group}",
                (18, 29),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cells.append(cell)
        rows.append(np.hstack(cells))
    overview = np.vstack(rows)
    cv2.imwrite(str(output_path), overview, [cv2.IMWRITE_JPEG_QUALITY, 94])


def validate_source_annotations(data, mappings):
    if data.get("image_size") != [1280, 720]:
        raise ValueError("Expected 1280x720 source annotations")
    confirmed = data.get("confirmed_triangles", {})
    for _, (group_a, group_b, _, _) in mappings.items():
        for group in (group_a, group_b):
            rows = data["sets"][group]
            seen = set()
            for number, row in enumerate(rows, start=1):
                label = str(row["label"])
                points = np.asarray(row["pts"], dtype=np.int32)
                if row["n"] != number or label in seen or not 1 <= int(label) <= 13:
                    raise ValueError(f"Invalid or repeated label in {group}: {label}")
                seen.add(label)
                if (points.ndim != 2 or points.shape[1] != 2
                        or len(points) not in (3, 4)
                        or np.any(points < 0)
                        or np.any(points[:, 0] >= 1280)
                        or np.any(points[:, 1] >= 720)):
                    raise ValueError(f"Invalid vertices in {group} label {label}")
                if abs(float(row["w"]) - cv2.contourArea(points)) > 0.51:
                    raise ValueError(f"Area mismatch in {group} label {label}")
                if not cv2.isContourConvex(points):
                    hull = cv2.convexHull(points)
                    if label not in confirmed.get(group, []) or len(hull) != 3:
                        raise ValueError(f"Unconfirmed non-convex ROI in {group} label {label}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Fit school obstacle ROIs from two annotation groups")
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--image-dir", type=Path, help="Optional directory with six angle_pic*.jpg photos")
    args = parser.parse_args(argv)
    data = json.loads(args.source.read_text(encoding="utf-8"))
    mappings = {
        0: ("g1_0", "g2_0", "angle_pic1.jpg", "angle_pic1b.jpg"),
        45: ("g1_45", "g2_45", "angle_pic2.jpg", "angle_pic2b.jpg"),
        90: ("g1_90", "g2_90", "angle_pic3.jpg", "angle_pic3b.jpg"),
    }
    validate_source_annotations(data, mappings)
    args.out.mkdir(parents=True, exist_ok=True)
    summary = {}
    for angle, (ka, kb, ia, ib) in mappings.items():
        payload = fit_angle(angle, data["sets"][ka], data["sets"][kb])
        (args.out / f"{angle}deg.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        runtime = [{key: item[key] for key in ("n", "label", "pts", "w")} for item in payload["labels"]]
        (args.out / f"{angle}deg_runtime.json").write_text(json.dumps(runtime, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if args.image_dir is not None:
            draw_overlay(args.image_dir / ia, payload, args.out / f"{angle}deg_on_{Path(ia).stem}.jpg")
            draw_overlay(args.image_dir / ib, payload, args.out / f"{angle}deg_on_{Path(ib).stem}.jpg")
        summary[str(angle)] = {
            "labels": [x["label"] for x in payload["labels"]],
            "missing_labels": payload["missing_labels"],
            "overlap_count_before": len(payload["topology_cleanup"]["overlaps_before"]),
            "overlap_count_after": len(payload["topology_cleanup"]["overlaps_after"]),
            "area_retention": payload["topology_cleanup"]["area_retention"],
            "diagnostics": payload["diagnostics"],
        }
    combined = {
        f"{angle}deg": json.loads((args.out / f"{angle}deg_runtime.json").read_text(encoding="utf-8"))
        for angle in mappings
    }
    (args.out / "all_angles_runtime.json").write_text(json.dumps(combined, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.image_dir is not None:
        make_overview(args.out, args.out / "fit_overview_3angles_2groups.jpg")
    (args.out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
