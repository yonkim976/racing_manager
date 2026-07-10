"""Build a Circuit.metric fragment from TUMFTM racetrack-database CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import urllib.request
from pathlib import Path
from typing import Any


def main() -> None:
    parser = argparse.ArgumentParser(description="Import a TUMFTM track CSV as Circuit.metric JSON")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="Existing TUMFTM tracks/*.csv file")
    source.add_argument("--url", help="Raw TUMFTM tracks/*.csv URL")
    parser.add_argument("--raceline-input", type=Path, help="Optional existing TUMFTM racelines/*.csv file")
    parser.add_argument("--raceline-url", help="Optional raw TUMFTM racelines/*.csv URL")
    parser.add_argument("--source-id", help="Source identifier to store in the fragment")
    parser.add_argument("--official-length-m", type=float, help="Official lap distance for scale correction")
    parser.add_argument("--start-index", type=int, default=0, help="Rotate loop so this point starts the lap")
    parser.add_argument("--reverse", action="store_true", help="Reverse the compiled centerline direction")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON")
    args = parser.parse_args()

    centerline_text = load_text(args.input, args.url)
    centerline = parse_track_csv(centerline_text)
    if len(centerline) < 4:
        raise SystemExit("Track CSV must contain at least four centerline rows")

    racing_line = []
    if args.raceline_input or args.raceline_url:
        racing_line = parse_raceline_csv(load_text(args.raceline_input, args.raceline_url))

    fragment: dict[str, Any] = {
        "source": "tumftm-racetrack-database",
        "sourceId": args.source_id or args.url or str(args.input),
        "license": "LGPL-3.0",
        "attribution": "TUMFTM racetrack-database",
        "coordinateSystem": "local_meters",
        "centerline": centerline,
        "racingLine": racing_line,
        "startFinishIndex": max(0, args.start_index),
        "reverse": args.reverse,
        "sourceLengthM": round(closed_length_m(centerline), 3),
        "scaleToOfficialLength": args.official_length_m is not None,
        "sampleSpacing": 16,
        "pitSampleSpacing": 14,
    }

    indent = 2 if args.pretty else None
    print(json.dumps(fragment, ensure_ascii=False, indent=indent))


def load_text(path: Path | None, url: str | None) -> str:
    if path:
        return path.read_text(encoding="utf-8")
    if not url:
        raise ValueError("Missing input path or URL")
    request = urllib.request.Request(
        url,
        headers={
            "Accept": "text/csv,text/plain,*/*",
            "User-Agent": "f1-race-manager-prototype/0.1",
        },
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return response.read().decode("utf-8")


def parse_track_csv(text: str) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for row in csv.reader(_numeric_lines(text)):
        if len(row) < 4:
            continue
        try:
            x_m, y_m, w_tr_right_m, w_tr_left_m = (float(value) for value in row[:4])
        except ValueError:
            continue
        points.append(
            {
                "x_m": round(x_m, 6),
                "y_m": round(y_m, 6),
                "w_tr_right_m": round(w_tr_right_m, 3),
                "w_tr_left_m": round(w_tr_left_m, 3),
            }
        )
    return points


def parse_raceline_csv(text: str) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for row in csv.reader(_numeric_lines(text)):
        if len(row) < 2:
            continue
        try:
            x_m, y_m = (float(value) for value in row[:2])
        except ValueError:
            continue
        points.append({"x_m": round(x_m, 6), "y_m": round(y_m, 6)})
    return points


def _numeric_lines(text: str) -> list[str]:
    return [
        line.strip()
        for line in text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def closed_length_m(points: list[dict[str, float]]) -> float:
    if len(points) < 2:
        return 0.0
    prepared = [(point["x_m"], point["y_m"]) for point in points]
    if prepared[0] != prepared[-1]:
        prepared.append(prepared[0])
    return sum(
        math.hypot(prepared[index + 1][0] - prepared[index][0], prepared[index + 1][1] - prepared[index][1])
        for index in range(len(prepared) - 1)
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
