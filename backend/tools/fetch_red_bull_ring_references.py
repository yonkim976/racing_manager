#!/usr/bin/env python3
"""Build compact Red Bull Ring width and 2025 qualifying references.

The command performs no project writes.  It prints a deterministic JSON
payload so maintainers can review and apply the generated profile explicitly.
"""

from __future__ import annotations

import csv
import io
import json
import statistics
import sys
from pathlib import Path
from urllib.parse import quote
from urllib.request import urlopen

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits  # noqa: E402
from simulation.track_geometry import normalize_points  # noqa: E402
from tools.calibrate_tracinginsights_lines import (  # noqa: E402
    _fit_similarity,
    _resample_closed,
)

WIDTH_SOURCE_URL = (
    "https://raw.githubusercontent.com/TUMFTM/racetrack-database/"
    "master/tracks/Spielberg.csv"
)
TELEMETRY_ROOT = (
    "https://raw.githubusercontent.com/TracingInsights-Archive/2025/main/"
    "Austrian%20Grand%20Prix/Qualifying"
)
REFERENCE_LAPS = (
    ("NOR", 17, "1:03.971"),
    ("LEC", 17, "1:04.492"),
    ("PIA", 16, "1:04.554"),
    ("HAM", 20, "1:04.582"),
    ("RUS", 17, "1:04.763"),
)


def _fetch_text(url: str) -> str:
    with urlopen(url, timeout=30) as response:  # noqa: S310 - fixed public URLs
        return response.read().decode("utf-8")


def _resample_width_source(
    rows: list[tuple[complex, float, float]],
    count: int,
) -> tuple[list[complex], list[float], list[float]]:
    points = [row[0] for row in rows]
    segment_lengths = [
        abs(points[(index + 1) % len(points)] - point)
        for index, point in enumerate(points)
    ]
    total = sum(segment_lengths)
    cumulative = [0.0]
    for length in segment_lengths:
        cumulative.append(cumulative[-1] + length)
    sampled_points: list[complex] = []
    sampled_right: list[float] = []
    sampled_left: list[float] = []
    segment = 0
    for sample_index in range(count):
        target = total * sample_index / count
        while segment + 1 < len(cumulative) and cumulative[segment + 1] < target:
            segment += 1
        following = (segment + 1) % len(rows)
        ratio = (target - cumulative[segment]) / max(1e-9, segment_lengths[segment])
        sampled_points.append(points[segment] + (points[following] - points[segment]) * ratio)
        sampled_right.append(rows[segment][1] + (rows[following][1] - rows[segment][1]) * ratio)
        sampled_left.append(rows[segment][2] + (rows[following][2] - rows[segment][2]) * ratio)
    return sampled_points, sampled_right, sampled_left


def _width_profile(circuit, count: int = 64) -> tuple[list[dict[str, float]], dict]:
    reader = csv.reader(
        line for line in _fetch_text(WIDTH_SOURCE_URL).splitlines() if not line.startswith("#")
    )
    rows = [
        (complex(float(x), float(y)), float(right), float(left))
        for x, y, right, left in reader
    ]
    source, right_widths, left_widths = _resample_width_source(rows, count)
    centerline = [complex(x, y) for x, y in normalize_points(circuit.track_coords)]
    centerline = _resample_closed(centerline, count)
    _, rms_units, mode, shift = _fit_similarity(source, centerline)
    reverse = mode.startswith("reverse")
    reflect = "+mirror" in mode
    swap_sides = reverse != reflect
    profile: list[dict[str, float] | None] = [None] * count
    for index in range(count):
        source_index = (-index) % count if reverse else index
        target_index = (index + shift) % count
        right = right_widths[source_index]
        left = left_widths[source_index]
        if swap_sides:
            left, right = right, left
        profile[target_index] = {
            "progress": round(target_index / count, 6),
            "left_width_m": round(left, 3),
            "right_width_m": round(right, 3),
        }
    completed = [item for item in profile if item is not None]
    total_widths = [item["left_width_m"] + item["right_width_m"] for item in completed]
    return completed, {
        "alignment": mode,
        "progress_shift": round(shift / count, 6),
        "fit_rms_normalized_units": round(rms_units, 6),
        "minimum_total_width_m": round(min(total_widths), 3),
        "maximum_total_width_m": round(max(total_widths), 3),
        "mean_total_width_m": round(statistics.fmean(total_widths), 3),
    }


def _channel_at(telemetry: dict, progress: float, channel: str) -> float:
    distances = telemetry["rel_distance"]
    values = telemetry[channel]
    target = progress % 1.0
    low = 0
    high = len(distances) - 1
    while low + 1 < high:
        middle = (low + high) // 2
        if float(distances[middle]) <= target:
            low = middle
        else:
            high = middle
    following = min(len(distances) - 1, low + 1)
    span = max(1e-9, float(distances[following]) - float(distances[low]))
    ratio = max(0.0, min(1.0, (target - float(distances[low])) / span))
    return float(values[low]) + (float(values[following]) - float(values[low])) * ratio


def _telemetry_reference(count: int = 128) -> list[dict[str, float]]:
    laps = []
    for driver, lap, _ in REFERENCE_LAPS:
        url = f"{TELEMETRY_ROOT}/{quote(driver)}/{lap}_tel.json"
        laps.append(json.loads(_fetch_text(url))["tel"])
    return [
        {
            "progress": round(index / count, 6),
            "speed_kph": round(
                statistics.median(
                    _channel_at(lap, index / count, "speed") for lap in laps
                ),
                3,
            ),
            "throttle_percent": round(
                statistics.median(
                    _channel_at(lap, index / count, "throttle") for lap in laps
                ),
                3,
            ),
            "braking_fraction": round(
                sum(
                    _channel_at(lap, index / count, "brake") >= 0.5
                    for lap in laps
                )
                / len(laps),
                3,
            ),
        }
        for index in range(count)
    ]


def main() -> int:
    circuit = next(item for item in load_circuits() if item.id == 4)
    widths, width_report = _width_profile(circuit)
    payload = {
        "source": "TUMFTM/racetrack-database Spielberg.csv",
        "source_url": WIDTH_SOURCE_URL,
        "license": "LGPL-3.0",
        "width_report": width_report,
        "track_width_calibration": {
            "method": "normalize_total_to_range_preserve_side_ratio",
            "minimum_total_width_m": 12.0,
            "maximum_total_width_m": 13.0,
            "source": "Official Austrian Grand Prix media kit",
            "source_url": (
                "https://www.fia.com/sites/default/files/media_kit_final_aut.pdf"
            ),
            "note": (
                "Normalize the satellite-derived total-width variation into "
                "the published 12-13 m F1 envelope while preserving the "
                "left/right centerline ratio."
            ),
        },
        "track_width_profile": widths,
        "physics_calibration": {
            "source": "TracingInsights-Archive/2025",
            "source_url": (
                "https://github.com/TracingInsights-Archive/2025/tree/main/"
                "Austrian%20Grand%20Prix/Qualifying"
            ),
            "reference_season": 2025,
            "reference_session": "Austrian Grand Prix Qualifying",
            "reference_laps": [
                f"{driver} {lap} {time}" for driver, lap, time in REFERENCE_LAPS
            ],
            "telemetry_speed_reference_weight": 1.0,
            "telemetry_progress_offset": 0.019,
            "telemetry_reference": _telemetry_reference(),
        },
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
