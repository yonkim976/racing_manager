"""Tests for the optional fixed-step longitudinal vehicle physics."""

from __future__ import annotations

import unittest

from data_loader import load_circuits
from simulation.speed_profile import build_speed_profile
from simulation.track_physics import PHYSICAL_CAR_LENGTH_M
from simulation.vehicle_physics import (
    PHYSICS_STEP_SECONDS,
    LongitudinalVehiclePhysics,
    VehicleFollowingConstraint,
    VehiclePhysicsModifiers,
)


class LongitudinalVehiclePhysicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = load_circuits()[3]
        self.profile = build_speed_profile(
            self.circuit,
            acceleration_mps2=12.0,
            braking_mps2=34.0,
        )
        self.assertIsNotNone(self.profile)
        self.physics = LongitudinalVehiclePhysics(
            self.profile,
            self.circuit.track_length_m,
            self.circuit.base_lap_time,
        )
        self.modifiers = VehiclePhysicsModifiers()

    def test_physics_step_is_fifty_hz(self) -> None:
        self.assertAlmostEqual(PHYSICS_STEP_SECONDS, 0.02)

    def test_curvature_profile_directly_changes_physical_target_speed(self) -> None:
        slow_index = self.profile.raw_speeds_mps.index(min(self.profile.raw_speeds_mps))
        fast_index = self.profile.raw_speeds_mps.index(max(self.profile.raw_speeds_mps))
        slow_distance = self.profile.progress[slow_index] * self.circuit.track_length_m
        fast_distance = self.profile.progress[fast_index] * self.circuit.track_length_m

        slow_target = self.physics.target_speed_mps(slow_distance, self.modifiers)
        fast_target = self.physics.target_speed_mps(fast_distance, self.modifiers)

        self.assertGreater(fast_target, slow_target)

    def test_car_accelerates_or_brakes_toward_local_physical_limit(self) -> None:
        fast_index = self.profile.raw_speeds_mps.index(max(self.profile.raw_speeds_mps))
        fast_distance = self.profile.progress[fast_index] * self.circuit.track_length_m
        accelerating = self.physics.advance(
            distance_m=fast_distance,
            speed_mps=20.0,
            delta_seconds=0.1,
            modifiers=self.modifiers,
        )
        self.assertGreater(accelerating.speed_mps, 20.0)
        self.assertGreater(accelerating.throttle, 0.0)

        slow_index = self.profile.raw_speeds_mps.index(min(self.profile.raw_speeds_mps))
        slow_distance = self.profile.progress[slow_index] * self.circuit.track_length_m
        braking = self.physics.advance(
            distance_m=slow_distance,
            speed_mps=90.0,
            delta_seconds=0.1,
            modifiers=self.modifiers,
        )
        self.assertLess(braking.speed_mps, 90.0)
        self.assertGreater(braking.brake, 0.0)

    def test_fixed_substeps_are_partition_deterministic(self) -> None:
        one_call = self.physics.advance(
            distance_m=250.0,
            speed_mps=40.0,
            delta_seconds=0.1,
            modifiers=self.modifiers,
        )

        distance = 250.0
        speed = 40.0
        repeated = None
        for _ in range(5):
            repeated = self.physics.advance(
                distance_m=distance,
                speed_mps=speed,
                delta_seconds=0.02,
                modifiers=self.modifiers,
            )
            distance = repeated.distance_m
            speed = repeated.speed_mps

        self.assertIsNotNone(repeated)
        self.assertAlmostEqual(one_call.distance_m, repeated.distance_m, places=9)
        self.assertAlmostEqual(one_call.speed_mps, repeated.speed_mps, places=9)

    def test_same_line_follower_cannot_cross_minimum_gap(self) -> None:
        following = VehicleFollowingConstraint(
            leader_distance_m=100.0,
            leader_speed_mps=30.0,
            leader_end_distance_m=103.0,
            leader_end_speed_mps=30.0,
            desired_gap_m=18.0,
            minimum_gap_m=PHYSICAL_CAR_LENGTH_M,
        )
        result = self.physics.advance(
            distance_m=93.0,
            speed_mps=80.0,
            delta_seconds=0.1,
            modifiers=self.modifiers,
            following=following,
        )

        self.assertLessEqual(
            result.distance_m,
            following.leader_end_distance_m - PHYSICAL_CAR_LENGTH_M,
        )
        self.assertLessEqual(result.speed_mps, following.leader_end_speed_mps)
        self.assertGreater(result.brake, 0.0)

    def test_following_controller_slows_car_before_the_hard_limit(self) -> None:
        following = VehicleFollowingConstraint(
            leader_distance_m=120.0,
            leader_speed_mps=35.0,
            leader_end_distance_m=123.5,
            leader_end_speed_mps=35.0,
            desired_gap_m=22.0,
            minimum_gap_m=PHYSICAL_CAR_LENGTH_M,
        )
        free = self.physics.advance(
            distance_m=100.0,
            speed_mps=45.0,
            delta_seconds=0.1,
            modifiers=self.modifiers,
        )
        controlled = self.physics.advance(
            distance_m=100.0,
            speed_mps=45.0,
            delta_seconds=0.1,
            modifiers=self.modifiers,
            following=following,
        )

        self.assertLess(controlled.speed_mps, free.speed_mps)
        self.assertLess(controlled.distance_m, free.distance_m)

    def test_lateral_controller_moves_to_target_and_respects_track_edge(self) -> None:
        result = self.physics.advance(
            distance_m=100.0,
            speed_mps=45.0,
            delta_seconds=1.0,
            modifiers=self.modifiers,
            lateral_offset_m=0.0,
            lateral_speed_mps=0.0,
            target_lateral_offset_m=3.0,
            minimum_lateral_offset_m=-4.0,
            maximum_lateral_offset_m=4.0,
        )
        self.assertGreater(result.lateral_offset_m, 0.0)
        self.assertLessEqual(result.lateral_offset_m, 4.0)
        self.assertGreater(result.lateral_speed_mps, 0.0)

        clamped = self.physics.advance(
            distance_m=result.distance_m,
            speed_mps=result.speed_mps,
            delta_seconds=3.0,
            modifiers=self.modifiers,
            lateral_offset_m=result.lateral_offset_m,
            lateral_speed_mps=result.lateral_speed_mps,
            target_lateral_offset_m=10.0,
            minimum_lateral_offset_m=-4.0,
            maximum_lateral_offset_m=4.0,
        )
        self.assertLessEqual(clamped.lateral_offset_m, 4.0)


if __name__ == "__main__":
    unittest.main()
