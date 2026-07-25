"""Validate imported circuits and their automatically generated racing lines."""

from __future__ import annotations

import argparse
import json
from math import hypot, isclose, isfinite
from pathlib import Path
import sys


BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits
from models.schemas import Circuit
from simulation.track_data_validation import (
    track_data_warnings,
    validate_track_data_v2,
)
from simulation.track_geometry import self_intersections, validate_circuit_geometry
from simulation.track_physics import (
    PHYSICAL_CAR_WIDTH_M,
    RACING_LINE_KERB_ALLOWANCE_M,
    TRACK_EDGE_MARGIN_M,
    build_track_physics_profile,
    clear_track_physics_caches,
    _closed_points,
    _path_physics,
)


DETERMINISM_ABS_TOLERANCE = 1e-7
DETERMINISM_REL_TOLERANCE = 1e-9


def _numerically_equal(first: float, second: float) -> bool:
    return isclose(
        first,
        second,
        abs_tol=DETERMINISM_ABS_TOLERANCE,
        rel_tol=DETERMINISM_REL_TOLERANCE,
    )


def _centerline_predicted_lap_time(circuit: Circuit) -> float:
    """Evaluate the centerline with the same path-speed model as the profile."""
    points = _closed_points(circuit)
    if len(points) < 3:
        return 0.0
    render_length = sum(
        hypot(
            points[(index + 1) % len(points)][0] - point[0],
            points[(index + 1) % len(points)][1] - point[1],
        )
        for index, point in enumerate(points)
    )
    _, _, lap_time = _path_physics(
        points,
        max(1.0, float(circuit.track_length_m)) / max(1e-9, render_length),
    )
    return lap_time


def validate_circuit(circuit: Circuit) -> list[str]:
    """Return all blocking errors for one imported circuit."""
    errors = validate_track_data_v2(circuit)
    errors.extend(validate_circuit_geometry(circuit))
    profile = build_track_physics_profile(circuit)
    if not profile.samples or not profile.racing_line_samples:
        return errors + ["racing line profile is empty"]

    line_coords = profile.racing_line_coords
    if len(line_coords) < 4 or line_coords[0] != line_coords[-1]:
        errors.append("racing line is not a closed path")
    elif self_intersections(line_coords) and not circuit.allows_self_intersection:
        errors.append("racing line has self-intersections")

    sample_count = len(profile.samples)
    for index, sample in enumerate(profile.samples):
        if not all(
            isfinite(value)
            for value in (
                sample.progress,
                sample.left_width_m,
                sample.right_width_m,
                sample.racing_line_offset_m,
            )
        ):
            errors.append(f"non-finite track profile value at sample {index}")
            break
        kerb_allowance = (
            RACING_LINE_KERB_ALLOWANCE_M
            if abs(sample.turn_signal) >= 0.08
            else 0.0
        )
        lower = (
            -sample.right_width_m
            - kerb_allowance
            + PHYSICAL_CAR_WIDTH_M / 2.0
            + TRACK_EDGE_MARGIN_M
        )
        upper = (
            sample.left_width_m
            + kerb_allowance
            - PHYSICAL_CAR_WIDTH_M / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        if not lower <= sample.racing_line_offset_m <= upper:
            errors.append(f"racing line exceeds body-safe boundary at sample {index}")
            break

        next_sample = profile.samples[(index + 1) % sample_count]
        segment_length_m = (
            (next_sample.progress - sample.progress) % 1.0
            * circuit.track_length_m
        )
        if segment_length_m <= 1e-9:
            errors.append(f"track profile has a duplicate/zero-length segment at sample {index}")
            break
        if (
            abs(next_sample.racing_line_offset_m - sample.racing_line_offset_m)
            / segment_length_m
            > 0.07
        ):
            errors.append(f"racing line lateral transition is too sharp at sample {index}")
            break

    centerline_lap_time = _centerline_predicted_lap_time(circuit)
    if (
        centerline_lap_time > 0.0
        and profile.predicted_racing_lap_time
        >= centerline_lap_time - DETERMINISM_ABS_TOLERANCE
    ):
        errors.append(
            "optimized racing line is not faster than the centerline in the shared path-speed model"
        )

    diagnostics = profile.optimization_diagnostics
    if diagnostics is None:
        errors.append("racing line optimization diagnostics are missing")
    elif diagnostics.final_objective_cost > diagnostics.initial_objective_cost + 1e-6:
        errors.append("optimized racing line is worse than its initial candidate")

    clear_track_physics_caches()
    repeat = build_track_physics_profile(circuit)
    repeat_is_independent = repeat is not profile
    diagnostics_mismatch = False
    if (profile.optimization_diagnostics is None) != (
        repeat.optimization_diagnostics is None
    ):
        diagnostics_mismatch = True
    elif profile.optimization_diagnostics is not None and repeat.optimization_diagnostics is not None:
        diagnostics_mismatch = any(
            getattr(profile.optimization_diagnostics, field)
            != getattr(repeat.optimization_diagnostics, field)
            for field in (
                "attempted_candidates",
                "evaluated_candidates",
                "accepted_updates",
            )
        ) or any(
            not _numerically_equal(
                getattr(profile.optimization_diagnostics, field),
                getattr(repeat.optimization_diagnostics, field),
            )
            for field in ("initial_objective_cost", "final_objective_cost")
        )

    coords_mismatch = (
        len(profile.racing_line_coords) != len(repeat.racing_line_coords)
        or any(
            len(first) != len(second)
            or any(
                not _numerically_equal(float(a), float(b))
                for a, b in zip(first, second)
            )
            for first, second in zip(
                profile.racing_line_coords,
                repeat.racing_line_coords,
            )
        )
    )
    offsets = [sample.lateral_offset_m for sample in profile.racing_line_samples]
    repeat_offsets = [sample.lateral_offset_m for sample in repeat.racing_line_samples]
    offsets_mismatch = (
        len(offsets) != len(repeat_offsets)
        or any(
            not _numerically_equal(first, second)
            for first, second in zip(offsets, repeat_offsets)
        )
    )
    if (
        not repeat_is_independent
        or coords_mismatch
        or offsets_mismatch
        or not _numerically_equal(
            profile.predicted_racing_lap_time,
            repeat.predicted_racing_lap_time,
        )
        or diagnostics_mismatch
    ):
        errors.append("racing line generation is not deterministic across independent builds")
    return errors


def validate_circuits(circuits: list[Circuit]) -> dict[str, list[str]]:
    result: dict[str, list[str]] = {}
    for circuit in circuits:
        try:
            result[circuit.name] = validate_circuit(circuit)
        except Exception as exc:
            result[circuit.name] = [
                f"validation failed without traceback: {type(exc).__name__}: {exc}"
            ]
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate circuit data and automatically generated racing lines."
    )
    parser.add_argument("--circuit-id", default=None, help="validate one circuit id")
    args = parser.parse_args(argv)

    circuits = load_circuits()
    if args.circuit_id is not None:
        circuits = [circuit for circuit in circuits if str(circuit.id) == args.circuit_id]
        if not circuits:
            parser.error(f"unknown circuit id: {args.circuit_id}")

    result = validate_circuits(circuits)
    warnings = {
        circuit.name: track_data_warnings(circuit)
        for circuit in circuits
        if track_data_warnings(circuit)
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    for circuit_name, circuit_warnings in warnings.items():
        for warning in circuit_warnings:
            print(f"WARNING [{circuit_name}] {warning}", file=sys.stderr)
    return 1 if any(result.values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
