"""Inspect OSM raceway ways inside a bbox to pick GP loop and pit-lane ways.

Helper for docs/ADDING_REAL_CIRCUITS.md: lists every ``highway=raceway`` way
with id, name, tags and length, then reports which closed cycles exist and
how far each cycle is from the official lap length.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from tools.import_osm_circuit import (
    closed_cycles,
    fetch_overpass_json,
    parse_raceway_ways,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="List OSM raceway ways and closed cycles")
    parser.add_argument("--bbox", required=True, help="minlat,minlon,maxlat,maxlon")
    parser.add_argument("--official-length-m", type=float, default=None)
    parser.add_argument("--allow-reverse-ways", action="store_true")
    parser.add_argument("--save-raw", type=Path, default=None, help="Save the Overpass response")
    args = parser.parse_args()

    data = fetch_overpass_json(args.bbox)
    if args.save_raw:
        args.save_raw.write_text(json.dumps(data), encoding="utf-8")

    tag_map = {
        element["id"]: element.get("tags", {})
        for element in data.get("elements", [])
        if element.get("type") == "way"
    }

    ways = parse_raceway_ways(data, include_karting=False)
    print(f"raceway ways: {len(ways)}")
    for way in sorted(ways, key=lambda w: -w.length_m):
        tags = tag_map.get(way.id, {})
        closed = "closed" if way.start_node == way.end_node else "open"
        interesting = {
            k: v
            for k, v in tags.items()
            if k in ("name", "ref", "sport", "service", "access", "oneway", "note", "description")
        }
        print(f"  way {way.id}: {way.length_m:8.1f}m {closed:6s} {interesting}")

    cycles = closed_cycles(ways, allow_reverse_ways=args.allow_reverse_ways)
    print(f"\nclosed cycles: {len(cycles)}")
    scored = []
    for cycle in cycles:
        length = sum(w.length_m for w in cycle)
        scored.append((length, [w.id for w in cycle]))
    scored.sort(key=lambda item: (
        abs(item[0] - args.official_length_m) if args.official_length_m else -item[0]
    ))
    for length, ids in scored[:10]:
        delta = f" (official delta {length - args.official_length_m:+.1f}m)" if args.official_length_m else ""
        print(f"  {length:9.1f}m{delta}: ways {ids}")


if __name__ == "__main__":
    main()
