"""Tier A acceptance audit for registered real circuits.

Checks each circuit against the approval criteria in
docs/ADDING_REAL_CIRCUITS.md and reports pass/fail per item.
"""

from __future__ import annotations

import argparse
import json
from math import hypot
from pathlib import Path
import sys

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits
from models.schemas import Circuit


def _coord(point) -> tuple[float, float]:
    if isinstance(point, list):
        return float(point[0]), float(point[1])
    return float(point.x), float(point.y)


def _polyline_length(coords) -> float:
    return sum(
        hypot(*(a - b for a, b in zip(_coord(coords[i + 1]), _coord(coords[i]))))
        for i in range(len(coords) - 1)
    )


def _distance_to_polyline(point, coords) -> float:
    px, py = _coord(point)
    best = float("inf")
    for i in range(len(coords) - 1):
        x1, y1 = _coord(coords[i])
        x2, y2 = _coord(coords[i + 1])
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq <= 0:
            candidate = hypot(px - x1, py - y1)
        else:
            t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / length_sq))
            candidate = hypot(px - (x1 + dx * t), py - (y1 + dy * t))
        best = min(best, candidate)
    return best


def _point_and_tangent_at_progress(coords, progress: float):
    pts = [_coord(p) for p in coords]
    total = sum(hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    target = (progress % 1.0) * total
    walked = 0.0
    for i in range(len(pts) - 1):
        seg = hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if walked + seg >= target and seg > 0:
            t = (target - walked) / seg
            x = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t
            y = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t
            tangent = ((pts[i + 1][0] - pts[i][0]) / seg, (pts[i + 1][1] - pts[i][1]) / seg)
            return (x, y), tangent
        walked += seg
    seg = hypot(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1])
    return pts[-1], ((pts[-1][0] - pts[-2][0]) / seg, (pts[-1][1] - pts[-2][1]) / seg)


def _segment_at(circuit: Circuit, progress: float):
    progress %= 1.0
    for segment in circuit.segments:
        if segment.start <= progress < segment.end:
            return segment
    return circuit.segments[-1] if circuit.segments else None


def audit_circuit(circuit: Circuit) -> dict:
    checks: dict[str, object] = {}
    official = float(circuit.track_length_m)
    source = circuit.geo or circuit.metric

    # 1. Source metadata
    checks["geo_source_present"] = source is not None
    if source is not None:
        checks["source_license"] = source.license
        checks["source_id"] = source.source_id
        src_len = source.source_length_m
        comp_len = source.computed_length_m
        checks["source_length_m"] = src_len
        checks["computed_length_m"] = comp_len
        checks["source_length_within_1pct"] = (
            src_len is not None and abs(src_len - official) / official <= 0.01
        )
        checks["compiled_length_within_0p5m"] = (
            comp_len is not None and abs(comp_len - official) <= 0.5
        )

    # 2. Compiled geometry
    checks["centerline_points"] = len(circuit.track_coords)
    checks["centerline_points_ge_80"] = len(circuit.track_coords) >= 80
    checks["pit_points"] = len(circuit.pit_lane_coords)
    checks["pit_points_ge_8"] = len(circuit.pit_lane_coords) >= 8
    if circuit.track_coords:
        spacings = [
            hypot(
                _coord(circuit.track_coords[i + 1])[0] - _coord(circuit.track_coords[i])[0],
                _coord(circuit.track_coords[i + 1])[1] - _coord(circuit.track_coords[i])[1],
            )
            for i in range(len(circuit.track_coords) - 1)
        ]
        checks["max_point_spacing"] = round(max(spacings), 3)
        checks["max_point_spacing_lt_24"] = max(spacings) < 24.0

    # 3. Pit anchors
    pit = circuit.pit_lane
    if pit is None:
        checks["pit_lane_config"] = "MISSING"
    else:
        required = {
            "entry_progress": pit.entry_progress,
            "exit_progress": pit.exit_progress,
            "safety_car_line_2_progress": pit.safety_car_line_2_progress,
        }
        checks["pit_missing_fields"] = [k for k, v in required.items() if v is None]
        raw = None
        try:
            raw_all = json.loads((BACKEND_DIR / "data" / "circuits.json").read_text())
            raw = next((c for c in raw_all if c.get("id") == circuit.id), None)
        except Exception:
            pass
        if raw is not None:
            raw_pit = raw.get("pit_lane") or {}
            defaults_used = [
                key
                for key in (
                    "speed_limit_start",
                    "speed_limit_end",
                    "speed_limit_kph",
                    "box_progress",
                    "lane_width_m",
                    "side_entry_progress",
                    "side_rejoin_progress",
                )
                if key not in raw_pit
            ]
            checks["pit_fields_left_at_default"] = defaults_used

    if circuit.pit_lane_coords and len(circuit.pit_lane_coords) >= 2 and circuit.track_coords:
        first_err = _distance_to_polyline(circuit.pit_lane_coords[0], circuit.track_coords)
        last_err = _distance_to_polyline(circuit.pit_lane_coords[-1], circuit.track_coords)
        checks["pit_start_projection_err"] = round(first_err, 5)
        checks["pit_end_projection_err"] = round(last_err, 5)
        checks["pit_projection_ok"] = first_err < 0.01 and last_err < 0.01
        if pit is not None and pit.exit_progress is not None:
            prev = _coord(circuit.pit_lane_coords[-2])
            exit_pt = _coord(circuit.pit_lane_coords[-1])
            _, tangent = _point_and_tangent_at_progress(circuit.track_coords, pit.exit_progress)
            dot = (exit_pt[0] - prev[0]) * tangent[0] + (exit_pt[1] - prev[1]) * tangent[1]
            checks["pit_exit_follows_race_direction"] = dot > 0.0

    # 4. DRS zones inside straight segments
    drs_ok = True
    drs_detail = []
    for zone in circuit.drs_zones:
        start_seg = _segment_at(circuit, zone.start)
        end_seg = _segment_at(circuit, zone.end - 0.001)
        start_type = start_seg.type.value if start_seg else None
        end_type = end_seg.type.value if end_seg else None
        ok = start_type == "straight" and end_type == "straight"
        drs_ok = drs_ok and ok
        drs_detail.append({"name": zone.name, "start_seg": start_type, "end_seg": end_type, "ok": ok})
    checks["drs_zones_on_straights"] = drs_ok
    checks["drs_detail"] = drs_detail

    # 5. Landmarks
    checks["landmark_count"] = len(circuit.landmarks)
    checks["landmarks_indexed_in_range"] = all(
        lm.track_index < len(circuit.track_coords) for lm in circuit.landmarks
    )
    checks["has_pit_landmark"] = any(lm.type == "pit" for lm in circuit.landmarks)
    checks["has_start_finish_landmark"] = any(lm.type == "start_finish" for lm in circuit.landmarks)

    # 6. Segments cover 0..1 without gaps
    segments = sorted(circuit.segments, key=lambda s: s.start)
    gaps_ok = bool(segments) and abs(segments[0].start) < 1e-9 and abs(segments[-1].end - 1.0) < 1e-9
    for a, b in zip(segments, segments[1:]):
        if abs(a.end - b.start) > 1e-9:
            gaps_ok = False
    checks["segments_cover_lap"] = gaps_ok
    checks["segment_count"] = len(segments)

    # 7. Sectors
    checks["sector_count"] = len(circuit.sectors)
    checks["sectors_have_timing_ranges"] = all(
        s.start is not None and s.end is not None for s in circuit.sectors
    )

    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description="Tier A circuit audit")
    parser.add_argument("--circuit-id", default=None)
    args = parser.parse_args()

    circuits = load_circuits()
    if args.circuit_id is not None:
        circuits = [c for c in circuits if str(c.id) == args.circuit_id]

    report = {}
    for circuit in circuits:
        try:
            report[f"{circuit.id}:{circuit.name}"] = audit_circuit(circuit)
        except Exception as exc:
            report[f"{circuit.id}:{circuit.name}"] = {"error": f"{type(exc).__name__}: {exc}"}
    print(json.dumps(report, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
