"""Build a Circuit.geo fragment from OpenStreetMap raceway ways.

This is a lightweight importer for prototype-quality real circuit layouts. It
uses directed OSM ``highway=raceway`` ways, stitches the longest closed loop,
and can optionally include a named pit lane way.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
)
OVERPASS_URL = OVERPASS_URLS[0]


@dataclass(frozen=True)
class OsmWay:
    id: int
    name: str
    sport: str
    start_node: int
    end_node: int
    geometry: list[dict[str, float]]
    length_m: float


def main() -> None:
    parser = argparse.ArgumentParser(description="Import an OSM raceway loop as Circuit.geo JSON")
    parser.add_argument("--input", type=Path, help="Existing Overpass JSON response")
    parser.add_argument("--bbox", help="minlat,minlon,maxlat,maxlon for an Overpass query")
    parser.add_argument("--include-karting", action="store_true", help="Keep ways tagged sport=karting")
    parser.add_argument("--start-way-id", type=int, help="Rotate loop so this way starts the lap")
    parser.add_argument("--start-way-name", help="Rotate loop so a matching way name starts the lap")
    parser.add_argument("--start-lonlat", help="Rotate loop to the nearest point on track to lat,lon")
    parser.add_argument("--pit-way-id", type=int, help="Use this way as pit lane")
    parser.add_argument("--pit-way-ids", help="Comma-separated way ids chained entry->exit as pit lane")
    parser.add_argument("--pit-way-name", help="Use the first way with a matching name as pit lane")
    parser.add_argument(
        "--split-way",
        action="append",
        default=[],
        metavar="WAY_ID:NODE_ID[,NODE_ID...]",
        help="Split a way at internal node ids so alternate routings can be stitched",
    )
    parser.add_argument("--official-length-m", type=float, help="Official lap distance for scale correction")
    parser.add_argument("--allow-reverse-ways", action="store_true", help="Allow OSM ways to be stitched in reverse")
    parser.add_argument("--source-id", help="Source identifier to store in the fragment")
    parser.add_argument("--reverse", action="store_true", help="Reverse the compiled centerline direction")
    parser.add_argument("--output", type=Path, help="Write JSON to this file instead of stdout")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON")
    args = parser.parse_args()

    if not args.input and not args.bbox:
        parser.error("Provide --input or --bbox")

    data = load_overpass_json(args.input) if args.input else fetch_overpass_json(args.bbox)
    if args.split_way:
        apply_way_splits(data, [parse_split_spec(spec) for spec in args.split_way])
    ways = parse_raceway_ways(data, include_karting=args.include_karting)
    cycle = select_closed_cycle(
        ways,
        target_length_m=args.official_length_m,
        allow_reverse_ways=args.allow_reverse_ways,
    )
    if not cycle:
        raise SystemExit("No closed raceway loop found")

    cycle = rotate_cycle(cycle, way_id=args.start_way_id, way_name=args.start_way_name)
    centerline = geometry_from_cycle(cycle)
    if args.reverse:
        centerline = [centerline[0], *reversed(centerline[1:])]
    if args.start_lonlat:
        centerline = rotate_geometry_to_lonlat(centerline, parse_lonlat(args.start_lonlat))

    pit_lane = []
    if args.pit_way_ids:
        pit_lane = chain_pit_ways(
            ways,
            [int(part.strip()) for part in args.pit_way_ids.split(",") if part.strip()],
        )
    else:
        pit_way = find_way(ways, way_id=args.pit_way_id, way_name=args.pit_way_name)
        if pit_way:
            pit_lane = pit_way.geometry

    fragment: dict[str, Any] = {
        "source": "osm",
        "sourceId": args.source_id or "openstreetmap-overpass",
        "license": "ODbL",
        "attribution": "OpenStreetMap contributors",
        "centerlineLonLat": round_geometry(centerline),
        "pitLaneLonLat": round_geometry(pit_lane),
        "sourceLengthM": round(sum(way.length_m for way in cycle), 3),
        "scaleToOfficialLength": args.official_length_m is not None,
        "sampleSpacing": 16,
        "pitSampleSpacing": 14,
    }
    if args.official_length_m:
        fragment["officialLengthM"] = round(args.official_length_m, 3)

    indent = 2 if args.pretty else None
    output = json.dumps(fragment, ensure_ascii=False, indent=indent)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(f"{output}\n", encoding="utf-8")
    else:
        print(output)


def load_overpass_json(path: Path | None) -> dict[str, Any]:
    if path is None:
        raise ValueError("Missing input path")
    with path.open(encoding="utf-8") as file:
        return json.load(file)


def fetch_overpass_json(bbox: str | None) -> dict[str, Any]:
    if bbox is None:
        raise ValueError("Missing bbox")
    query = f"""
    [out:json][timeout:40];
    way["highway"="raceway"]({bbox});
    out geom;
    """
    body = urllib.parse.urlencode({"data": query}).encode()
    last_error: Exception | None = None
    for url in OVERPASS_URLS:
        request = urllib.request.Request(
            url,
            data=body,
            headers={
                "Accept": "application/json",
                "User-Agent": "f1-race-manager-prototype/0.1",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=50) as response:
                return json.loads(response.read().decode("utf-8"))
        except Exception as exc:  # noqa: BLE001 - fall through to the next mirror
            last_error = exc
            print(f"overpass endpoint failed ({url}): {exc}", file=sys.stderr)
    raise SystemExit(f"All Overpass endpoints failed: {last_error}")


def parse_split_spec(spec: str) -> tuple[int, list[int]]:
    try:
        way_part, node_part = spec.split(":", 1)
        way_id = int(way_part)
        node_ids = [int(part.strip()) for part in node_part.split(",") if part.strip()]
    except ValueError as exc:
        raise SystemExit(f"Invalid --split-way spec: {spec!r}") from exc
    if not node_ids:
        raise SystemExit(f"--split-way needs at least one node id: {spec!r}")
    return way_id, node_ids


def apply_way_splits(data: dict[str, Any], specs: list[tuple[int, list[int]]]) -> None:
    """Replace ways in the Overpass response with sub-ways cut at the given nodes.

    Sub-ways get synthetic ids (way_id * 100 + part index) so the cycle search can
    stitch alternate routings, e.g. a chicane bypass, through the cut points.
    """
    elements = data.get("elements", [])
    for way_id, node_ids in specs:
        element = next(
            (e for e in elements if e.get("type") == "way" and e.get("id") == way_id),
            None,
        )
        if element is None:
            raise SystemExit(f"--split-way: way {way_id} not found in input")
        nodes = element.get("nodes") or []
        geometry = element.get("geometry") or []
        cut_indices = sorted(
            {index for index, node in enumerate(nodes) if node in set(node_ids)}
        )
        missing = [node for node in node_ids if node not in nodes]
        if missing:
            raise SystemExit(f"--split-way: nodes {missing} not on way {way_id}")
        cut_indices = [index for index in cut_indices if 0 < index < len(nodes) - 1]
        if not cut_indices:
            raise SystemExit(f"--split-way: no internal cut points for way {way_id}")

        boundaries = [0, *cut_indices, len(nodes) - 1]
        parts = []
        for part_index, (start, end) in enumerate(zip(boundaries, boundaries[1:])):
            part = dict(element)
            part["id"] = way_id * 100 + part_index
            part["nodes"] = nodes[start : end + 1]
            part["geometry"] = geometry[start : end + 1]
            parts.append(part)
        position = elements.index(element)
        elements[position : position + 1] = parts


def chain_pit_ways(ways: list[OsmWay], way_ids: list[int]) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for way_id in way_ids:
        way = find_way(ways, way_id=way_id)
        if way is None:
            raise SystemExit(f"--pit-way-ids: way {way_id} not found")
        if points:
            if not same_lonlat(points[-1], way.geometry[0]):
                raise SystemExit(f"--pit-way-ids: way {way_id} does not continue the chain")
            points.extend(way.geometry[1:])
        else:
            points.extend(way.geometry)
    return points


def parse_raceway_ways(data: dict[str, Any], *, include_karting: bool = False) -> list[OsmWay]:
    ways: list[OsmWay] = []
    for element in data.get("elements", []):
        if element.get("type") != "way" or "geometry" not in element:
            continue
        nodes = element.get("nodes") or []
        geometry = element.get("geometry") or []
        if len(nodes) < 2 or len(geometry) < 2:
            continue

        tags = element.get("tags", {})
        sport = tags.get("sport", "")
        if sport == "karting" and not include_karting:
            continue

        ways.append(
            OsmWay(
                id=int(element["id"]),
                name=tags.get("name", ""),
                sport=sport,
                start_node=int(nodes[0]),
                end_node=int(nodes[-1]),
                geometry=[{"lat": float(point["lat"]), "lon": float(point["lon"])} for point in geometry],
                length_m=geometry_length_m(geometry),
            )
        )
    return ways


def longest_closed_cycle(ways: list[OsmWay]) -> list[OsmWay]:
    return select_closed_cycle(ways)


def select_closed_cycle(
    ways: list[OsmWay],
    *,
    target_length_m: float | None = None,
    allow_reverse_ways: bool = False,
) -> list[OsmWay]:
    cycles = closed_cycles(ways, allow_reverse_ways=allow_reverse_ways)
    if not cycles:
        return []
    if target_length_m:
        return min(cycles, key=lambda cycle: abs(_cycle_length(cycle) - target_length_m))
    return max(cycles, key=_cycle_length)


def closed_cycles(ways: list[OsmWay], *, allow_reverse_ways: bool = False) -> list[list[OsmWay]]:
    search_ways = _oriented_ways(ways) if allow_reverse_ways else ways
    by_start: dict[int, list[OsmWay]] = {}
    for way in search_ways:
        by_start.setdefault(way.start_node, []).append(way)

    cycles: list[list[OsmWay]] = []
    seen: set[tuple[int, ...]] = set()
    for start_way in search_ways:
        stack = [(
            start_way.end_node,
            start_way.start_node,
            [start_way],
            {start_way.id},
        )]
        while stack and len(cycles) < 50000:
            current_node, start_node, route, used_ids = stack.pop()
            if current_node == start_node:
                key = _cycle_key(route)
                if key not in seen:
                    seen.add(key)
                    cycles.append(route)
                continue
            if len(route) >= len(ways):
                continue

            candidates = [
                candidate
                for candidate in by_start.get(current_node, [])
                if candidate.id not in used_ids
            ]
            for candidate in sorted(candidates, key=lambda way: way.length_m):
                stack.append((
                    candidate.end_node,
                    start_node,
                    [*route, candidate],
                    {*used_ids, candidate.id},
                ))

    return cycles


def _oriented_ways(ways: list[OsmWay]) -> list[OsmWay]:
    oriented: list[OsmWay] = []
    for way in ways:
        oriented.append(way)
        oriented.append(
            OsmWay(
                id=way.id,
                name=way.name,
                sport=way.sport,
                start_node=way.end_node,
                end_node=way.start_node,
                geometry=list(reversed(way.geometry)),
                length_m=way.length_m,
            )
        )
    return oriented


def _cycle_key(cycle: list[OsmWay]) -> tuple[int, ...]:
    ids = [way.id for way in cycle]
    rotations = [tuple(ids[index:] + ids[:index]) for index in range(len(ids))]
    reversed_ids = list(reversed(ids))
    rotations.extend(tuple(reversed_ids[index:] + reversed_ids[:index]) for index in range(len(reversed_ids)))
    return min(rotations)


def _cycle_length(cycle: list[OsmWay]) -> float:
    return sum(way.length_m for way in cycle)


def rotate_cycle(cycle: list[OsmWay], *, way_id: int | None = None, way_name: str | None = None) -> list[OsmWay]:
    if not cycle:
        return cycle

    match_index = None
    if way_id is not None:
        match_index = next((index for index, way in enumerate(cycle) if way.id == way_id), None)
    if match_index is None and way_name:
        needle = way_name.lower()
        match_index = next((index for index, way in enumerate(cycle) if needle in way.name.lower()), None)
    if match_index is None:
        return cycle
    return [*cycle[match_index:], *cycle[:match_index]]


def find_way(ways: list[OsmWay], *, way_id: int | None = None, way_name: str | None = None) -> OsmWay | None:
    if way_id is not None:
        return next((way for way in ways if way.id == way_id), None)
    if way_name:
        needle = way_name.lower()
        return next((way for way in ways if needle in way.name.lower()), None)
    return None


def geometry_from_cycle(cycle: list[OsmWay]) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for way in cycle:
        next_points = way.geometry
        if points and same_lonlat(points[-1], next_points[0]):
            points.extend(next_points[1:])
        else:
            points.extend(next_points)

    if points and not same_lonlat(points[0], points[-1]):
        points.append(points[0])
    return points


def parse_lonlat(value: str) -> dict[str, float]:
    try:
        lat, lon = (float(part.strip()) for part in value.split(",", 1))
    except ValueError as exc:
        raise SystemExit("--start-lonlat must use lat,lon") from exc
    return {"lat": lat, "lon": lon}


def rotate_geometry_to_lonlat(
    geometry: list[dict[str, float]],
    target: dict[str, float],
) -> list[dict[str, float]]:
    if len(geometry) < 2:
        return geometry

    ring = geometry[:-1] if same_lonlat(geometry[0], geometry[-1]) else geometry[:]
    if len(ring) < 2:
        return geometry

    origin = _projection_origin([*ring, target])
    projected_ring = [_project_lonlat(point, origin) for point in ring]
    projected_target = _project_lonlat(target, origin)
    best_index = 0
    best_t = 0.0
    best_distance = float("inf")
    for index, point in enumerate(projected_ring):
        next_index = (index + 1) % len(projected_ring)
        candidate_t, candidate_distance = _projected_segment_position(
            projected_target,
            point,
            projected_ring[next_index],
        )
        if candidate_distance < best_distance:
            best_index = index
            best_t = candidate_t
            best_distance = candidate_distance

    next_index = (best_index + 1) % len(ring)
    inserted = _interpolate_lonlat(ring[best_index], ring[next_index], best_t)
    rotated = [inserted]
    for offset in range(len(ring)):
        point = ring[(next_index + offset) % len(ring)]
        if not same_lonlat(rotated[-1], point):
            rotated.append(point)
    if not same_lonlat(rotated[-1], inserted):
        rotated.append(inserted)
    return rotated


def _projection_origin(points: list[dict[str, float]]) -> dict[str, float]:
    return {
        "lat": sum(point["lat"] for point in points) / len(points),
        "lon": sum(point["lon"] for point in points) / len(points),
    }


def _project_lonlat(point: dict[str, float], origin: dict[str, float]) -> tuple[float, float]:
    mean_lat = math.radians(origin["lat"])
    x = (point["lon"] - origin["lon"]) * 111412.84 * math.cos(mean_lat)
    y = (origin["lat"] - point["lat"]) * 111132.92
    return x, y


def _projected_segment_position(
    target: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        return 0.0, math.hypot(target[0] - start[0], target[1] - start[1])
    t = max(0.0, min(1.0, ((target[0] - start[0]) * dx + (target[1] - start[1]) * dy) / length_squared))
    projected = (start[0] + dx * t, start[1] + dy * t)
    return t, math.hypot(target[0] - projected[0], target[1] - projected[1])


def _interpolate_lonlat(a: dict[str, float], b: dict[str, float], t: float) -> dict[str, float]:
    return {
        "lat": a["lat"] + (b["lat"] - a["lat"]) * t,
        "lon": a["lon"] + (b["lon"] - a["lon"]) * t,
    }


def round_geometry(geometry: list[dict[str, float]]) -> list[dict[str, float]]:
    return [
        {
            "lat": round(point["lat"], 7),
            "lon": round(point["lon"], 7),
        }
        for point in geometry
    ]


def geometry_length_m(geometry: list[dict[str, float]]) -> float:
    return sum(distance_m(geometry[index], geometry[index + 1]) for index in range(len(geometry) - 1))


def distance_m(a: dict[str, float], b: dict[str, float]) -> float:
    mean_lat = math.radians((float(a["lat"]) + float(b["lat"])) / 2)
    dx = (float(b["lon"]) - float(a["lon"])) * 111412.84 * math.cos(mean_lat)
    dy = (float(b["lat"]) - float(a["lat"])) * 111132.92
    return math.hypot(dx, dy)


def same_lonlat(a: dict[str, float], b: dict[str, float]) -> bool:
    return abs(a["lat"] - b["lat"]) < 1e-9 and abs(a["lon"] - b["lon"]) < 1e-9


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.exit(1)
