"""Emit the reproducible Stage 3 single-vehicle kinematics diagnostic JSON."""

from __future__ import annotations

import json
import sys
from math import hypot, pi
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend"))

from data_loader import load_circuits, load_drivers, load_teams  # noqa: E402
from simulation.abstract import (  # noqa: E402
    AbstractSessionSnapshot,
    LateralTrajectory,
    SingleProbeKinematicCursor,
    SingleVehiclePoseSynthesizer,
    pose_sequence_hash,
)


BASELINE = {
    3: {
        "legacy_line_distance_error_at_progress_0_5_m": 2.1638865,
        "legacy_constant_progress_rate": 0.02,
        "legacy_max_speed_kph": 454.0601,
        "legacy_max_chord_acceleration_mps2": 257.3486,
        "legacy_min_chord_acceleration_mps2": -265.7765,
    },
    4: {
        "legacy_line_distance_error_at_progress_0_5_m": 3.1491842,
        "legacy_constant_progress_rate": 0.02,
        "legacy_max_speed_kph": 358.9163,
        "legacy_max_chord_acceleration_mps2": 165.2304,
        "legacy_min_chord_acceleration_mps2": -193.6523,
    },
}


def _angle_error(first: float, second: float) -> float:
    return abs((first - second + pi) % (2.0 * pi) - pi)


def diagnose(circuit_id: int, circuit, drivers, teams) -> dict:
    snapshot = AbstractSessionSnapshot.from_content(
        session_id=f"abstract-stage3-diagnostic-{circuit_id}",
        session_seed=42,
        circuit=circuit,
        drivers=drivers,
        teams=teams,
    )
    synthesizer = SingleVehiclePoseSynthesizer(snapshot)
    geometry = snapshot.track.display_geometry
    assert geometry is not None
    profile = geometry.compiled_profile

    # One selected probe only; this is deliberately not a retained race replay.
    probe_cursor = SingleProbeKinematicCursor(
        synthesizer,
        1,
        grid_position=1,
    )
    frames = [probe_cursor.current_pose()]
    accepted_steps = []
    for _ in range(round(180.0 / 0.10)):
        tick = probe_cursor.advance_one_tick()
        frames.append(tick.pose)
        accepted_steps.append(tick.accepted_step)
    deltas = [
        current.line_distance_m - previous.line_distance_m
        for previous, current in zip(frames, frames[1:])
    ]
    displacement_violations = sum(
        current_delta > previous.speed_mps * 0.10 + 0.5 + 1e-9
        for previous, current_delta in zip(frames, deltas)
    )
    world_chords = [
        hypot(
            current.world_x_m - previous.world_x_m,
            current.world_y_m - previous.world_y_m,
        )
        for previous, current in zip(frames, frames[1:])
    ]
    world_chord_violations = sum(
        chord > previous.speed_mps * 0.10 + 0.5 + 1e-9
        for previous, chord in zip(frames, world_chords)
    )
    interval_speeds = [delta / 0.10 for delta in deltas]
    derived_accelerations = [
        (next_speed - previous_speed) / 0.10
        for previous_speed, next_speed in zip(interval_speeds, interval_speeds[1:])
    ]
    integrator_errors = [
        abs(delta - (previous.speed_mps + current.speed_mps) * 0.05)
        for previous, current, delta in zip(frames, frames[1:], deltas)
    ]
    reported_acceleration_errors = [
        abs(current.longitudinal_acceleration_mps2 - (current.speed_mps - previous.speed_mps) / 0.10)
        for previous, current in zip(frames, frames[1:])
    ]
    world_chord_speeds = [chord / 0.10 for chord in world_chords]
    world_chord_speed_jumps = [
        abs(next_speed - previous_speed)
        for previous_speed, next_speed in zip(world_chord_speeds, world_chord_speeds[1:])
    ]
    accepted_step_distance_mismatch_count = sum(
        abs(current.line_distance_m - step.distance_m) > 1e-9
        for current, step in zip(frames[1:], accepted_steps)
    )
    accepted_step_speed_mismatch_count = sum(
        abs(current.speed_mps - step.speed_mps) > 1e-9
        for current, step in zip(frames[1:], accepted_steps)
    )
    accepted_step_acceleration_mismatch_count = sum(
        abs(current.longitudinal_acceleration_mps2 - step.longitudinal_acceleration_mps2) > 1e-9
        for current, step in zip(frames[1:], accepted_steps)
    )
    accepted_step_displacement_mismatch_count = sum(
        abs((current.line_distance_m - previous.line_distance_m) - step.displacement_m) > 1e-9
        for previous, current, step in zip(frames, frames[1:], accepted_steps)
    )
    accepted_step_adjustment_count = sum(step.integrator_adjusted for step in accepted_steps)
    curvature_values = [abs(sample.curvature_1pm) for sample in profile.racing_line_samples]
    curvature_cutoff = sorted(curvature_values)[int(len(curvature_values) * 0.75)]
    corner_chords = []
    for previous, current, chord in zip(frames, frames[1:], world_chords):
        index = int((current.progress % 1.0) * len(profile.racing_line_samples))
        index = min(len(profile.racing_line_samples) - 1, index)
        if abs(profile.racing_line_samples[index].curvature_1pm) >= curvature_cutoff:
            corner_chords.append(chord)

    lateral_limits = synthesizer.distance_contract.lateral_limits_at_progress(
        0.0,
        car_width_m=geometry.car_width_m,
    )
    lateral_trajectory = LateralTrajectory.create(
        start_offset_m=0.0,
        end_offset_m=min(0.5, lateral_limits[1]),
        start_time_s=0.0,
        duration_s=2.0,
        target_limits_m=lateral_limits,
    )
    lateral_frames = synthesizer.logical_sequence(
        1,
        start_logical_time_s=0.0,
        end_logical_time_s=5.0,
        lateral_trajectory=lateral_trajectory,
    )
    clearances = []
    boundary_violations = 0
    for frame in lateral_frames:
        minimum, maximum = synthesizer.distance_contract.lateral_limits_at_progress(
            frame.total_progress,
            car_width_m=geometry.car_width_m,
        )
        clearance = min(
            frame.lateral_offset_m - minimum,
            maximum - frame.lateral_offset_m,
        )
        clearances.append(clearance)
        boundary_violations += int(clearance < -1e-9)

    hashes = {
        f"{speed:g}x": pose_sequence_hash(
            synthesizer.logical_sequence(
                1,
                start_logical_time_s=0.0,
                end_logical_time_s=12.0,
                speed_multiplier=speed,
            )
        )
        for speed in (1.0, 2.0, 5.0)
    }
    grid_frames = synthesizer.grid_sequence(
        1,
        grid_position=1,
        logical_duration_s=3.0,
    )
    grid_sample_times = {0.0, 1.0, 2.0, 3.0}
    grid_samples = [
        {
            "time_s": frame.simulation_time_s,
            "line_distance_m": round(frame.line_distance_m, 6),
            "speed_mps": round(frame.speed_mps, 6),
            "lateral_offset_m": round(frame.lateral_offset_m, 6),
        }
        for frame in grid_frames
        if any(abs(frame.simulation_time_s - target) < 1e-9 for target in grid_sample_times)
    ]
    lap_wraps = [
        {
            "time_s": current.simulation_time_s,
            "from_lap": previous.lap_number,
            "to_lap": current.lap_number,
            "line_distance_m": round(current.line_distance_m, 6),
        }
        for previous, current in zip(frames, frames[1:])
        if current.lap_number > previous.lap_number
    ]
    derived_line_speed_min_mps = min(interval_speeds)
    derived_line_speed_max_mps = max(interval_speeds)
    derived_line_acceleration_min_mps2 = min(derived_accelerations)
    derived_line_acceleration_max_mps2 = max(derived_accelerations)
    integrator_identity_violation_count = sum(error > 1e-6 for error in integrator_errors)
    reported_acceleration_identity_violation_count = sum(
        error > 1e-6 for error in reported_acceleration_errors
    )
    post_integrator_distance_correction_count = accepted_step_distance_mismatch_count
    approval_passed = all(
        (
            derived_line_speed_min_mps >= -1e-9,
            derived_line_speed_max_mps * 3.6 <= 370.0 + 1e-6,
            derived_line_acceleration_max_mps2 <= 18.0 + 1e-6,
            derived_line_acceleration_min_mps2 >= -50.0 - 1e-6,
            integrator_identity_violation_count == 0,
            reported_acceleration_identity_violation_count == 0,
            post_integrator_distance_correction_count == 0,
            accepted_step_distance_mismatch_count == 0,
            accepted_step_speed_mismatch_count == 0,
            accepted_step_acceleration_mismatch_count == 0,
            accepted_step_displacement_mismatch_count == 0,
            accepted_step_adjustment_count == 0,
            displacement_violations == 0,
            world_chord_violations == 0,
            boundary_violations == 0,
            all(delta >= -1e-9 for delta in deltas),
            bool(lap_wraps),
        )
    )
    regression_progress_samples = {
        "0.42444": round(min(abs(frame.progress - 0.42444) for frame in frames), 9),
        "0.13158": round(min(abs(frame.progress - 0.13158) for frame in frames), 9),
    } if circuit_id == 3 else {
        "0.29789": round(min(abs(frame.progress - 0.29789) for frame in frames), 9),
        "0.29960": round(min(abs(frame.progress - 0.29960) for frame in frames), 9),
    }
    return {
        "circuit_id": circuit_id,
        "circuit_name": circuit.name,
        "coordinate_frame": {
            "origin_x_render": geometry.coordinate_frame.origin_x_render,
            "origin_y_render": geometry.coordinate_frame.origin_y_render,
            "meters_per_render_unit": geometry.coordinate_frame.meters_per_render_unit,
            "metric_track_length_m": geometry.coordinate_frame.track_length_m,
            "compiled_racing_line_length_m": synthesizer.line_length_m,
        },
        "baseline_reproduction": BASELINE[circuit_id],
        "max_speed_kph": round(max(frame.speed_mps for frame in frames) * 3.6, 6),
        "max_longitudinal_acceleration_mps2": round(
            max(frame.longitudinal_acceleration_mps2 for frame in frames),
            6,
        ),
        "max_braking_magnitude_mps2": round(
            max(0.0, -min(frame.longitudinal_acceleration_mps2 for frame in frames)),
            6,
        ),
        "max_lateral_speed_mps": round(
            max(abs(frame.lateral_velocity_mps) for frame in lateral_frames),
            6,
        ),
        "max_lateral_acceleration_mps2": round(
            max(abs(frame.lateral_acceleration_mps2) for frame in lateral_frames),
            6,
        ),
        "minimum_boundary_clearance_m": round(min(clearances), 6),
        "boundary_violation_count": boundary_violations,
        "maximum_tick_displacement_m": round(max(deltas), 6),
        "tick_displacement_violation_count": displacement_violations,
        "maximum_world_chord_m": round(max(world_chords), 6),
        "world_chord_violation_count": world_chord_violations,
        "derived_line_speed_min_mps": round(derived_line_speed_min_mps, 6),
        "derived_line_speed_max_mps": round(derived_line_speed_max_mps, 6),
        "derived_line_acceleration_min_mps2": round(derived_line_acceleration_min_mps2, 6),
        "derived_line_acceleration_max_mps2": round(derived_line_acceleration_max_mps2, 6),
        "reported_vs_derived_speed_error_max_mps": round(
            max(
                abs((previous.speed_mps + current.speed_mps) * 0.5 - interval_speed)
                for previous, current, interval_speed in zip(frames, frames[1:], interval_speeds)
            ),
            6,
        ),
        "integrator_distance_error_max_m": round(max(integrator_errors), 9),
        "integrator_identity_violation_count": integrator_identity_violation_count,
        "reported_acceleration_identity_violation_count": reported_acceleration_identity_violation_count,
        "post_integrator_distance_correction_count": post_integrator_distance_correction_count,
        "accepted_step_adjustment_count": accepted_step_adjustment_count,
        "accepted_step_pose_distance_mismatch_count": accepted_step_distance_mismatch_count,
        "accepted_step_pose_speed_mismatch_count": accepted_step_speed_mismatch_count,
        "accepted_step_pose_acceleration_mismatch_count": accepted_step_acceleration_mismatch_count,
        "accepted_step_displacement_mismatch_count": accepted_step_displacement_mismatch_count,
        "world_chord_speed_jump_max_mps": round(max(world_chord_speed_jumps, default=0.0), 6),
        "regression_progress_sample_distance": regression_progress_samples,
        "approval_passed": approval_passed,
        "corner_chord": {
            "curvature_cutoff_1pm": round(curvature_cutoff, 9),
            "maximum_world_chord_m": round(max(corner_chords), 6),
            "sample_count": len(corner_chords),
        },
        "lap_wrap": {
            "observed": bool(lap_wraps),
            "transition_count": len(lap_wraps),
            "first_transition": lap_wraps[0] if lap_wraps else None,
        },
        "pose_hashes": hashes,
        "grid_hold_and_launch_first_3s": grid_samples,
    }


def main() -> int:
    circuits = {circuit.id: circuit for circuit in load_circuits()}
    drivers = load_drivers()
    teams = load_teams()
    payload = {
        "diagnostic": "abstract-stage3-single-vehicle-kinematics",
        "version": "abstract-stage3-kinematics-v1",
        "logical_tick_seconds": 0.10,
        "circuits": [diagnose(circuit_id, circuits[circuit_id], drivers, teams) for circuit_id in (3, 4)],
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if all(item["approval_passed"] for item in payload["circuits"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
