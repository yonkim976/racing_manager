"""Tests for metric track width and generated racing lines."""

from __future__ import annotations

import hashlib
import json
import unittest
from copy import deepcopy
from math import cos, hypot, isfinite, pi, sin

from data_loader import load_circuits, load_teams
from simulation.car_performance import car_performance_factors
from simulation.track_geometry import self_intersections
from simulation.track_physics import (
    DRIVING_LINE_DEFENSIVE,
    DRIVING_LINE_INSIDE,
    DRIVING_LINE_OUTSIDE,
    DRIVING_LINE_RACING,
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
    RACING_LINE_KERB_ALLOWANCE_M,
    TRACK_EDGE_MARGIN_M,
    _TRACK_PHYSICS_CACHE,
    _VEHICLE_TRACK_PHYSICS_CACHE,
    LIVE_TRAJECTORY_CENTER_STRIDE,
    _path_physics,
    build_track_physics_profile,
    build_vehicle_track_physics_profile,
    clear_vehicle_track_physics_cache,
)
from simulation.track_surface import TrackSurfaceProfile
from simulation.tire_model import compute_tire_physics_factors
from simulation.trajectory_physics import TireTrajectorySpec, VehicleTrajectorySpec
from models.schemas import TireCompound


class TrackPhysicsProfileTests(unittest.TestCase):
    def test_session_cleanup_clears_only_vehicle_profile_cache(self) -> None:
        base_marker = object()
        vehicle_marker = object()
        _TRACK_PHYSICS_CACHE[("base",)] = base_marker
        _VEHICLE_TRACK_PHYSICS_CACHE[("vehicle",)] = vehicle_marker

        clear_vehicle_track_physics_cache()

        self.assertIs(_TRACK_PHYSICS_CACHE[("base",)], base_marker)
        self.assertEqual(_VEHICLE_TRACK_PHYSICS_CACHE, {})
        _TRACK_PHYSICS_CACHE.pop(("base",), None)

    def test_vehicle_lines_are_body_safe_and_continuous_on_all_circuits(self) -> None:
        team = next(team for team in load_teams() if team.id == 1)
        vehicle = VehicleTrajectorySpec.from_car_performance(
            car_performance_factors(team)
        )
        tire = TireTrajectorySpec.from_tire_physics(
            TireCompound.MEDIUM,
            compute_tire_physics_factors(TireCompound.MEDIUM, 0.0),
        )

        for circuit in load_circuits():
            profile = build_vehicle_track_physics_profile(circuit, vehicle, tire)
            surface = TrackSurfaceProfile.for_circuit(circuit, profile)
            for line_name, samples in profile.driving_line_samples.items():
                coords = profile.coords_for_line(line_name)
                if circuit.allows_self_intersection:
                    self.assertEqual(len(self_intersections(coords)), 1, circuit.name)
                else:
                    self.assertEqual(self_intersections(coords), [], circuit.name)
                line_length_m = profile.length_for_line(line_name)
                self.assertGreater(
                    line_length_m,
                    circuit.track_length_m * 0.9,
                    circuit.name,
                )
                self.assertLess(
                    line_length_m,
                    circuit.track_length_m * 1.1,
                    circuit.name,
                )
                self.assertTrue(
                    all(isfinite(sample.curvature_1pm) for sample in samples),
                    circuit.name,
                )

                assessment = surface.assess_trajectory(
                    [sample.center_progress for sample in samples],
                    [sample.lateral_offset_m for sample in samples],
                    track_length_m=circuit.track_length_m,
                    body_width_m=PHYSICAL_CAR_WIDTH_M,
                    body_length_m=PHYSICAL_CAR_LENGTH_M,
                    edge_margin_m=TRACK_EDGE_MARGIN_M,
                )
                self.assertEqual(
                    assessment.body_boundary_violations,
                    0,
                    f"{circuit.name}: {line_name}",
                )
                self.assertEqual(assessment.high_kerb_contacts, 0, circuit.name)
                self.assertEqual(assessment.runoff_contacts, 0, circuit.name)
                self.assertEqual(assessment.grass_contacts, 0, circuit.name)
                self.assertEqual(assessment.gravel_contacts, 0, circuit.name)

                for index, sample in enumerate(samples):
                    next_sample = samples[(index + 1) % len(samples)]
                    center_distance_m = (
                        (next_sample.center_progress - sample.center_progress) % 1.0
                        * circuit.track_length_m
                    )
                    curvature_change = abs(
                        next_sample.curvature_1pm - sample.curvature_1pm
                    )
                    self.assertLessEqual(
                        curvature_change,
                        0.065,
                        f"{circuit.name}: {line_name}",
                    )
                    self.assertLessEqual(
                        curvature_change / max(1.0, center_distance_m),
                        0.0025,
                        f"{circuit.name}: {line_name}",
                    )
                    self.assertLessEqual(
                        abs(
                            next_sample.lateral_offset_m
                            - sample.lateral_offset_m
                        )
                        / max(1.0, center_distance_m),
                        0.071,
                        f"{circuit.name}: {line_name}",
                    )

    def test_vehicle_trajectory_cache_is_reused_and_deterministic(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        team = next(team for team in load_teams() if team.id == 1)
        vehicle = VehicleTrajectorySpec.from_car_performance(
            car_performance_factors(team)
        )
        tire = TireTrajectorySpec.from_tire_physics(
            TireCompound.MEDIUM,
            compute_tire_physics_factors(TireCompound.MEDIUM, 0.0),
        )

        _VEHICLE_TRACK_PHYSICS_CACHE.clear()
        first = build_vehicle_track_physics_profile(circuit, vehicle, tire)
        cached = build_vehicle_track_physics_profile(circuit, vehicle, tire)
        self.assertIs(cached, first)
        first_signature = (
            round(first.predicted_racing_lap_time, 9),
            tuple(
                round(sample.lateral_offset_m, 9)
                for sample in first.racing_line_samples
            ),
        )

        _VEHICLE_TRACK_PHYSICS_CACHE.clear()
        rebuilt = build_vehicle_track_physics_profile(circuit, vehicle, tire)
        rebuilt_signature = (
            round(rebuilt.predicted_racing_lap_time, 9),
            tuple(
                round(sample.lateral_offset_m, 9)
                for sample in rebuilt.racing_line_samples
            ),
        )
        self.assertIsNot(rebuilt, first)
        self.assertEqual(rebuilt_signature, first_signature)

    def test_red_bull_ring_live_optimizer_samples_corner_entry_apex_and_exit(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        team = next(team for team in load_teams() if team.id == 1)
        vehicle = VehicleTrajectorySpec.from_car_performance(
            car_performance_factors(team)
        )
        tire = TireTrajectorySpec.from_tire_physics(
            TireCompound.SOFT,
            compute_tire_physics_factors(TireCompound.SOFT, 0.0),
        )

        _VEHICLE_TRACK_PHYSICS_CACHE.clear()
        profile = build_vehicle_track_physics_profile(circuit, vehicle, tire)
        diagnostics = profile.optimization_diagnostics
        self.assertIsNotNone(diagnostics)
        assert diagnostics is not None
        coarse_centers = (
            len(profile.racing_line_samples) + LIVE_TRAJECTORY_CENTER_STRIDE - 1
        ) // LIVE_TRAJECTORY_CENTER_STRIDE
        self.assertGreater(
            diagnostics.attempted_candidates,
            coarse_centers * 2 * 2,
        )

    def test_live_budget_preserves_vehicle_specific_performance_differences(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        teams = load_teams()
        tire = TireTrajectorySpec.from_tire_physics(
            TireCompound.MEDIUM,
            compute_tire_physics_factors(TireCompound.MEDIUM, 0.0),
        )
        first_vehicle = VehicleTrajectorySpec.from_car_performance(
            car_performance_factors(teams[0])
        )
        second_vehicle = VehicleTrajectorySpec.from_car_performance(
            car_performance_factors(teams[1])
        )

        _VEHICLE_TRACK_PHYSICS_CACHE.clear()
        first = build_vehicle_track_physics_profile(circuit, first_vehicle, tire)
        second = build_vehicle_track_physics_profile(circuit, second_vehicle, tire)

        self.assertIsNot(first, second)
        self.assertNotEqual(
            round(first.predicted_racing_lap_time, 6),
            round(second.predicted_racing_lap_time, 6),
        )
        self.assertNotEqual(
            {
                name: round(value, 6)
                for name, value in first.predicted_line_lap_times.items()
            },
            {
                name: round(value, 6)
                for name, value in second.predicted_line_lap_times.items()
            },
        )

    def test_optimizer_interface_preserves_frozen_circuit_outputs(self) -> None:
        expected_signatures = {
            3: "73b7e20d6763b8c67e5026aecc1044117cce112cd686a300c1989a62e41561ca",
            4: "6bd6563979f3a68822765b20c953630284b59d5f6318608966b769c9978a1169",
            5: "a0d3264ce93e1857d7487494dda657a8bd3cebe4c27ee89f58e096814a3ec6c5",
            6: "c896ed514c7aa4f826fd81c84d809aa51bb0313735f56dc86626630fb35d7b78",
            7: "45fb6d773f2bd58e2d4e3179622715192e2c55474dfb38f2c6b86c8f4c61d7f6",
            8: "ef40ca9734056e64b2e05c259ed0790acddf6cdb51b085360dfa7fb4620de2cf",
            9: "bdb6f9b1d7b42c52aa247865d9f203b8b33f8a2cab817be55aa1e019a7074957",
            10: "6ba33656c521c02cd21c8170c317d2607ae55c432fb8e46a2653a9957b1665aa",
            11: "304509d0e42bd22627a7c1a665fb867d3ead5148d63b71023cce2718d64481c0",
        }

        for circuit in load_circuits():
            _TRACK_PHYSICS_CACHE.clear()
            profile = build_track_physics_profile(circuit)
            frozen_output = {
                "time": round(profile.predicted_racing_lap_time, 9),
                "offsets": [
                    round(sample.racing_line_offset_m, 9)
                    for sample in profile.samples
                ],
            }
            signature = hashlib.sha256(
                json.dumps(frozen_output, separators=(",", ":")).encode()
            ).hexdigest()

            self.assertEqual(signature, expected_signatures[int(circuit.id)])
            diagnostics = profile.optimization_diagnostics
            self.assertIsNotNone(diagnostics)
            assert diagnostics is not None
            self.assertGreater(diagnostics.attempted_candidates, 0)
            self.assertGreater(diagnostics.evaluated_candidates, 0)
            self.assertGreaterEqual(
                diagnostics.initial_objective_cost,
                diagnostics.final_objective_cost,
            )
            self.assertGreaterEqual(diagnostics.duration_ms, 0.0)

    def test_2025_telemetry_references_drive_supported_circuit_lines(self) -> None:
        supported_ids = {3, 4, 5, 7}
        circuits = {
            int(circuit.id): circuit
            for circuit in load_circuits()
            if int(circuit.id) in supported_ids
        }

        self.assertEqual(set(circuits), supported_ids)
        for circuit in circuits.values():
            calibration = circuit.physics_calibration
            self.assertIsNotNone(calibration, circuit.name)
            assert calibration is not None
            self.assertEqual(calibration.source, "TracingInsights-Archive/2025")
            self.assertEqual(calibration.reference_season, 2025)
            self.assertEqual(len(calibration.reference_laps), 5)
            self.assertEqual(len(calibration.racing_line_reference), 64)

            measured_profile = build_track_physics_profile(circuit)
            synthetic_circuit = deepcopy(circuit)
            synthetic_circuit.physics_calibration = None
            synthetic_profile = build_track_physics_profile(synthetic_circuit)
            difference_rms = (
                sum(
                    (
                        measured.racing_line_offset_m
                        - synthetic.racing_line_offset_m
                    ) ** 2
                    for measured, synthetic in zip(
                        measured_profile.samples,
                        synthetic_profile.samples,
                    )
                )
                / len(measured_profile.samples)
            ) ** 0.5
            self.assertGreater(difference_rms, 0.25, circuit.name)

    def test_physical_car_dimensions_match_game_specification(self) -> None:
        self.assertEqual(PHYSICAL_CAR_WIDTH_M, 1.9)
        self.assertEqual(PHYSICAL_CAR_LENGTH_M, 5.0)

    def test_generated_racing_line_only_uses_small_kerb_allowance_in_corners(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        profile = build_track_physics_profile(circuit)

        self.assertGreater(len(profile.samples), 20)
        self.assertGreater(
            max(sample.racing_line_offset_m for sample in profile.samples),
            min(sample.racing_line_offset_m for sample in profile.samples),
        )
        for sample in profile.samples:
            kerb_allowance = (
                RACING_LINE_KERB_ALLOWANCE_M
                if abs(sample.turn_signal) >= 0.08
                else 0.0
            )
            self.assertLessEqual(
                sample.racing_line_offset_m,
                sample.left_width_m
                + kerb_allowance
                - PHYSICAL_CAR_WIDTH_M / 2
                - TRACK_EDGE_MARGIN_M,
            )
            self.assertGreaterEqual(
                sample.racing_line_offset_m,
                -sample.right_width_m
                - kerb_allowance
                + PHYSICAL_CAR_WIDTH_M / 2
                + TRACK_EDGE_MARGIN_M,
            )

    def test_profile_interpolates_width_and_line_continuously(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        profile = build_track_physics_profile(circuit)
        first = profile.at_progress(0.25)
        nearby = profile.at_progress(0.2501)

        self.assertLess(
            abs(first.racing_line_offset_m - nearby.racing_line_offset_m),
            0.2,
        )
        self.assertGreaterEqual(first.left_width_m, 4.0)
        self.assertGreaterEqual(first.right_width_m, 4.0)

    def test_generated_racing_line_has_physical_path_metrics(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        profile = build_track_physics_profile(circuit)

        self.assertEqual(len(profile.racing_line_samples), len(profile.samples))
        self.assertEqual(profile.racing_line_coords[0], profile.racing_line_coords[-1])
        self.assertGreater(profile.racing_line_length_m, circuit.track_length_m * 0.9)
        self.assertLess(profile.racing_line_length_m, circuit.track_length_m * 1.1)
        self.assertGreater(profile.predicted_racing_lap_time, 0.0)
        self.assertTrue(any(
            abs(sample.curvature_1pm) > 1e-5
            for sample in profile.racing_line_samples
        ))

        path_progress = [sample.path_progress for sample in profile.racing_line_samples]
        path_distance = [sample.path_distance_m for sample in profile.racing_line_samples]
        self.assertEqual(path_progress, sorted(path_progress))
        self.assertEqual(path_distance, sorted(path_distance))

    def test_generated_racing_line_has_driveable_lateral_transitions(self) -> None:
        for circuit in load_circuits():
            profile = build_track_physics_profile(circuit)
            samples = profile.racing_line_samples
            for index, sample in enumerate(samples):
                next_sample = samples[(index + 1) % len(samples)]
                center_distance_m = (
                    (next_sample.center_progress - sample.center_progress) % 1.0
                    * circuit.track_length_m
                )
                lateral_slope = abs(
                    next_sample.lateral_offset_m - sample.lateral_offset_m
                ) / max(1.0, center_distance_m)
                self.assertLessEqual(lateral_slope, 0.051, circuit.name)

    def test_generated_racing_line_is_faster_than_centerline_model(self) -> None:
        for circuit in load_circuits():
            profile = build_track_physics_profile(circuit)
            centerline = [
                (float(point[0]), float(point[1]))
                for point in circuit.track_coords[:-1]
            ]
            coordinate_length = sum(
                hypot(
                    centerline[(index + 1) % len(centerline)][0] - point[0],
                    centerline[(index + 1) % len(centerline)][1] - point[1],
                )
                for index, point in enumerate(centerline)
            )
            meters_per_unit = circuit.track_length_m / coordinate_length
            _, _, centerline_lap_time = _path_physics(centerline, meters_per_unit)
            self.assertLess(
                profile.predicted_racing_lap_time,
                centerline_lap_time,
                circuit.name,
            )

    def test_corner_line_set_has_distinct_bounded_physical_paths(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        profile = build_track_physics_profile(circuit)
        expected_names = {
            DRIVING_LINE_RACING,
            DRIVING_LINE_INSIDE,
            DRIVING_LINE_OUTSIDE,
            DRIVING_LINE_DEFENSIVE,
        }

        self.assertEqual(set(profile.driving_line_samples), expected_names)
        self.assertEqual(set(profile.predicted_line_lap_times), expected_names)
        self.assertEqual(set(profile.driving_line_lengths_m), expected_names)
        for name, samples in profile.driving_line_samples.items():
            self.assertEqual(len(samples), len(profile.samples), name)
            self.assertGreater(profile.driving_line_lengths_m[name], 0.0)
            self.assertEqual(
                profile.driving_line_coords[name][0],
                profile.driving_line_coords[name][-1],
            )
            self.assertEqual(self_intersections(profile.driving_line_coords[name]), [])
            for sample, track_sample in zip(samples, profile.samples):
                kerb_allowance = (
                    RACING_LINE_KERB_ALLOWANCE_M
                    if name == DRIVING_LINE_RACING
                    and abs(track_sample.turn_signal) >= 0.08
                    else 0.0
                )
                self.assertLessEqual(
                    sample.lateral_offset_m,
                    track_sample.left_width_m
                    + kerb_allowance
                    - PHYSICAL_CAR_WIDTH_M / 2
                    - TRACK_EDGE_MARGIN_M,
                )
                self.assertGreaterEqual(
                    sample.lateral_offset_m,
                    -track_sample.right_width_m
                    - kerb_allowance
                    + PHYSICAL_CAR_WIDTH_M / 2
                    + TRACK_EDGE_MARGIN_M,
                )
            for index, sample in enumerate(samples):
                next_sample = samples[(index + 1) % len(samples)]
                center_distance_m = (
                    (next_sample.center_progress - sample.center_progress) % 1.0
                    * circuit.track_length_m
                )
                self.assertLessEqual(
                    abs(next_sample.lateral_offset_m - sample.lateral_offset_m)
                    / max(1.0, center_distance_m),
                    0.071,
                    name,
                )

        strong_corners = [
            index for index, sample in enumerate(profile.samples)
            if abs(sample.turn_signal) >= 0.6
        ]
        inside_alignment = sum(
            profile.driving_line_samples[DRIVING_LINE_INSIDE][index].lateral_offset_m
            * profile.samples[index].turn_signal
            for index in strong_corners
        ) / len(strong_corners)
        outside_alignment = sum(
            profile.driving_line_samples[DRIVING_LINE_OUTSIDE][index].lateral_offset_m
            * profile.samples[index].turn_signal
            for index in strong_corners
        ) / len(strong_corners)
        defensive_alignment = sum(
            profile.driving_line_samples[DRIVING_LINE_DEFENSIVE][index].lateral_offset_m
            * profile.samples[index].turn_signal
            for index in strong_corners
        ) / len(strong_corners)
        self.assertGreater(inside_alignment, 0.0)
        self.assertLess(outside_alignment, 0.0)
        self.assertGreater(defensive_alignment, 0.0)
        self.assertGreater(
            profile.predicted_line_lap_times[DRIVING_LINE_INSIDE],
            profile.predicted_line_lap_times[DRIVING_LINE_RACING],
        )
        self.assertGreater(
            profile.predicted_line_lap_times[DRIVING_LINE_OUTSIDE],
            profile.predicted_line_lap_times[DRIVING_LINE_RACING],
        )
        self.assertGreater(
            profile.predicted_line_lap_times[DRIVING_LINE_DEFENSIVE],
            profile.predicted_line_lap_times[DRIVING_LINE_RACING],
        )

    def test_line_distance_and_center_progress_round_trip(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 6)
        profile = build_track_physics_profile(circuit)

        for line_name in profile.driving_line_samples:
            for total_progress in (-0.002, 0.0, 0.137, 0.731, 1.002, 2.91):
                line_distance = profile.line_distance_at_total_progress(
                    line_name,
                    total_progress,
                )
                restored = profile.total_progress_at_line_distance(
                    line_name,
                    line_distance,
                )
                self.assertAlmostEqual(restored, total_progress, places=8)

    def test_off_line_world_pose_is_continuous_across_path_sample_boundaries(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        profile = build_track_physics_profile(circuit)
        samples = profile.driving_line_samples[DRIVING_LINE_RACING]
        epsilon = 1e-6
        lateral_offset_m = 5.0

        for sample in samples:
            before = profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                sample.center_progress - epsilon,
            )
            after = profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                sample.center_progress + epsilon,
            )
            before_point = (
                before[0] - sin(before[2]) * lateral_offset_m,
                before[1] + cos(before[2]) * lateral_offset_m,
            )
            after_point = (
                after[0] - sin(after[2]) * lateral_offset_m,
                after[1] + cos(after[2]) * lateral_offset_m,
            )
            heading_delta = (after[2] - before[2] + pi) % (2.0 * pi) - pi
            self.assertLess(hypot(
                after_point[0] - before_point[0],
                after_point[1] - before_point[1],
            ), 0.10)
            self.assertLess(abs(heading_delta), 0.02)

    def test_spa_la_source_preserves_outside_apex_outside_line_shape(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 6)
        profile = build_track_physics_profile(circuit)

        entry = profile.line_offset_at_progress(DRIVING_LINE_RACING, 128.0 / 7004.0)
        apex = profile.line_offset_at_progress(DRIVING_LINE_RACING, 275.0 / 7004.0)
        exit_line = profile.line_offset_at_progress(DRIVING_LINE_RACING, 390.0 / 7004.0)

        self.assertLess(entry, -0.5)
        self.assertGreater(apex, 0.5)
        self.assertLess(exit_line, -0.5)


if __name__ == "__main__":
    unittest.main()
