"""Tests for vehicle- and tyre-aware whole-lap trajectory physics."""

from __future__ import annotations

import unittest
from dataclasses import replace
from math import cos, pi, sin

from data_loader import load_circuits, load_teams
from models.schemas import TireCompound
from engines.full.runtime.car_performance import car_performance_factors
from simulation.global_trajectory_optimizer import (
    GlobalTrajectoryOptimizationRequest,
    GlobalTrajectoryOptimizer,
    GlobalTrajectoryOptimizerConfig,
)
from engines.full.runtime.tire_model import compute_tire_physics_factors
from simulation.track_physics import (
    PhysicalGlobalTrajectoryCostModel,
    _closed_points,
    build_track_physics_profile,
)
from engines.full.runtime.track_surface import TrackSurfaceProfile
from engines.full.runtime.trajectory_physics import (
    TireTrajectorySpec,
    VehicleTrajectorySpec,
    build_trajectory_speed_profile,
)


class TrajectoryPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuit = next(c for c in load_circuits() if c.id == 4)
        track_profile = build_track_physics_profile(cls.circuit)
        samples = track_profile.racing_line_samples
        line_length = track_profile.racing_line_length_m
        cls.segment_lengths = [
            (
                samples[(index + 1) % len(samples)].path_distance_m
                - sample.path_distance_m
            )
            % line_length
            for index, sample in enumerate(samples)
        ]
        cls.curvatures = [sample.curvature_1pm for sample in samples]
        cls.vehicle = VehicleTrajectorySpec.from_car_performance(
            car_performance_factors(load_teams()[0])
        )
        cls.soft = TireTrajectorySpec.from_tire_physics(
            TireCompound.SOFT,
            compute_tire_physics_factors(TireCompound.SOFT, 0.0),
        )
        cls.hard = TireTrajectorySpec.from_tire_physics(
            TireCompound.HARD,
            compute_tire_physics_factors(TireCompound.HARD, 0.0),
        )
        cls.worn_soft = TireTrajectorySpec.from_tire_physics(
            TireCompound.SOFT,
            compute_tire_physics_factors(TireCompound.SOFT, 16.0),
        )

    def _profile(self, vehicle=None, tire=None):
        return build_trajectory_speed_profile(
            self.segment_lengths,
            self.curvatures,
            vehicle or self.vehicle,
            tire or self.soft,
        )

    def test_power_and_drag_change_straight_speed_and_lap_time(self) -> None:
        baseline = self._profile()
        lower_power = self._profile(
            replace(self.vehicle, engine_power_kw=680.0)
        )
        higher_drag = self._profile(
            replace(self.vehicle, straight_drag_area_m2=1.05)
        )

        self.assertGreater(lower_power.lap_time_seconds, baseline.lap_time_seconds)
        self.assertLess(
            max(lower_power.target_speeds_mps),
            max(baseline.target_speeds_mps),
        )
        self.assertGreater(higher_drag.lap_time_seconds, baseline.lap_time_seconds)
        self.assertLess(
            max(higher_drag.target_speeds_mps),
            max(baseline.target_speeds_mps),
        )

    def test_downforce_compound_and_wear_change_corner_profile(self) -> None:
        baseline = self._profile()
        higher_downforce = self._profile(
            replace(self.vehicle, corner_downforce_area_m2=6.2)
        )
        hard = self._profile(tire=self.hard)
        worn = self._profile(tire=self.worn_soft)

        self.assertLess(
            higher_downforce.lap_time_seconds,
            baseline.lap_time_seconds,
        )
        self.assertGreater(
            min(higher_downforce.target_speeds_mps),
            min(baseline.target_speeds_mps),
        )
        self.assertGreater(hard.lap_time_seconds, baseline.lap_time_seconds)
        self.assertGreater(worn.lap_time_seconds, hard.lap_time_seconds)

    def test_profile_contains_bounded_throttle_and_braking_commands(self) -> None:
        profile = self._profile()
        count = len(self.segment_lengths)

        self.assertEqual(len(profile.target_speeds_mps), count)
        self.assertEqual(len(profile.brake_utilization), count)
        self.assertEqual(len(profile.throttle_utilization), count)
        self.assertTrue(any(value > 0.05 for value in profile.brake_utilization))
        self.assertTrue(any(value > 0.05 for value in profile.throttle_utilization))
        self.assertTrue(all(0.0 <= value <= 1.0 for value in profile.brake_utilization))
        self.assertTrue(all(0.0 <= value <= 1.0 for value in profile.throttle_utilization))

    def test_physical_cost_model_plugs_into_deterministic_global_search(self) -> None:
        sample_count = 24
        radius_m = 100.0
        points = [
            (
                radius_m * cos(2.0 * pi * index / sample_count),
                radius_m * sin(2.0 * pi * index / sample_count),
            )
            for index in range(sample_count)
        ]
        widths = [(6.0, 6.0)] * sample_count
        model = PhysicalGlobalTrajectoryCostModel(
            points,
            widths,
            2.0 * pi * radius_m,
            0.0,
            self.vehicle,
            self.soft,
            speed_pass_count=2,
        )
        request = GlobalTrajectoryOptimizationRequest(
            initial_offsets_m=(0.0,) * sample_count,
            track_length_m=2.0 * pi * radius_m,
        )
        optimizer = GlobalTrajectoryOptimizer(
            GlobalTrajectoryOptimizerConfig(
                optimization_steps_m=(0.3,),
                transition_radius_m=50.0,
            )
        )

        first = optimizer.optimize(request, model)
        second = optimizer.optimize(request, model)

        self.assertEqual(first, second)
        self.assertEqual(
            len(first.evaluation.target_speeds_mps),
            sample_count,
        )
        self.assertGreater(first.evaluation.lap_time_seconds, 0.0)

    def test_surface_aware_cost_rejects_body_boundary_violation(self) -> None:
        track_profile = build_track_physics_profile(self.circuit)
        surface = TrackSurfaceProfile.for_circuit(self.circuit, track_profile)
        points = _closed_points(self.circuit)
        widths = [
            surface.trajectory_optimization_widths(sample.progress)
            for sample in track_profile.samples
        ]
        model = PhysicalGlobalTrajectoryCostModel(
            points,
            widths,
            self.circuit.track_length_m,
            0.0,
            self.vehicle,
            self.soft,
            speed_pass_count=2,
            surface_profile=surface,
            trajectory_progress=track_profile.progress,
        )

        clean = model.evaluate(
            [0.0] * len(points),
            [0.0] * len(points),
        )
        outside = model.evaluate(
            [20.0] * len(points),
            [0.0] * len(points),
        )

        self.assertIsNotNone(clean)
        self.assertIsNone(outside)


if __name__ == "__main__":
    unittest.main()
