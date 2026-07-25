"""Tests for the optional fixed-step longitudinal vehicle physics."""

from __future__ import annotations

import unittest

from data_loader import load_circuits
from simulation.speed_profile import SpeedProfile, build_speed_profile
from simulation.track_physics import PHYSICAL_CAR_LENGTH_M
from simulation.vehicle_physics import (
    PHYSICS_STEP_SECONDS,
    LongitudinalVehiclePhysics,
    VehicleFollowingConstraint,
    VehiclePhysicsModifiers,
)


class LongitudinalVehiclePhysicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
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

    def test_predictive_controller_does_not_consume_legacy_raw_speed_targets(self) -> None:
        altered_profile = SpeedProfile(
            progress=self.profile.progress,
            raw_speeds_mps=[18.0 for _ in self.profile.raw_speeds_mps],
            curvatures_1pm=self.profile.curvatures_1pm,
            signed_curvatures_1pm=self.profile.signed_curvatures_1pm,
        )
        altered_physics = LongitudinalVehiclePhysics(
            altered_profile,
            self.circuit.track_length_m,
            self.circuit.base_lap_time,
        )
        distance_m = 0.27 * self.circuit.track_length_m

        self.assertAlmostEqual(
            self.physics.target_speed_mps(distance_m, self.modifiers),
            altered_physics.target_speed_mps(distance_m, self.modifiers),
            places=9,
        )

    def test_measured_braking_calibration_creates_an_earlier_corner_approach(self) -> None:
        slow_index = self.profile.curvatures_1pm.index(
            max(self.profile.curvatures_1pm)
        )
        apex_distance_m = self.profile.progress[slow_index] * self.circuit.track_length_m
        approach_distance_m = (apex_distance_m - 100.0) % self.circuit.track_length_m
        calibrated = LongitudinalVehiclePhysics(
            self.profile,
            self.circuit.track_length_m,
            self.circuit.base_lap_time,
            planner_braking_utilization=0.55,
            controller_sample_distance_m=10.0,
            controller_speed_scale_floor=0.60,
        )

        self.assertLess(
            calibrated.target_speed_mps(approach_distance_m, self.modifiers),
            self.physics.target_speed_mps(approach_distance_m, self.modifiers),
        )
        self.assertEqual(calibrated.planner_braking_utilization, 0.55)
        self.assertEqual(calibrated.controller_sample_distance_m, 10.0)

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

    def test_effective_power_is_applied_as_a_real_drive_force(self) -> None:
        fast_index = self.profile.raw_speeds_mps.index(max(self.profile.raw_speeds_mps))
        fast_distance = self.profile.progress[fast_index] * self.circuit.track_length_m
        lower_power = self.physics.advance(
            distance_m=fast_distance,
            speed_mps=65.0,
            delta_seconds=1.0,
            modifiers=VehiclePhysicsModifiers(effective_power_kw=700.0),
        )
        higher_power = self.physics.advance(
            distance_m=fast_distance,
            speed_mps=65.0,
            delta_seconds=1.0,
            modifiers=VehiclePhysicsModifiers(effective_power_kw=780.0),
        )

        self.assertGreater(higher_power.speed_mps, lower_power.speed_mps)

    def test_straight_mode_drag_area_changes_high_speed_acceleration(self) -> None:
        fast_index = self.profile.raw_speeds_mps.index(max(self.profile.raw_speeds_mps))
        fast_distance = self.profile.progress[fast_index] * self.circuit.track_length_m
        high_drag = self.physics.advance(
            distance_m=fast_distance,
            speed_mps=80.0,
            delta_seconds=1.0,
            modifiers=VehiclePhysicsModifiers(straight_drag_area_m2=1.05),
        )
        low_drag = self.physics.advance(
            distance_m=fast_distance,
            speed_mps=80.0,
            delta_seconds=1.0,
            modifiers=VehiclePhysicsModifiers(straight_drag_area_m2=0.75),
        )

        self.assertGreater(low_drag.speed_mps, high_drag.speed_mps)

    def test_downforce_increases_the_physical_corner_speed_limit(self) -> None:
        corner_index = self.profile.curvatures_1pm.index(
            max(self.profile.curvatures_1pm)
        )
        corner_distance = (
            self.profile.progress[corner_index] * self.circuit.track_length_m
        )
        low_downforce_target = self.physics._lateral_speed_limit_mps(
            corner_distance,
            VehiclePhysicsModifiers(corner_downforce_area_m2=3.5),
        )
        high_downforce_target = self.physics._lateral_speed_limit_mps(
            corner_distance,
            VehiclePhysicsModifiers(corner_downforce_area_m2=6.2),
        )

        self.assertGreater(high_downforce_target, low_downforce_target)

    def test_friction_circle_reduces_acceleration_while_cornering(self) -> None:
        corner_index = self.profile.curvatures_1pm.index(
            max(self.profile.curvatures_1pm)
        )
        straight_index = self.profile.curvatures_1pm.index(
            min(self.profile.curvatures_1pm)
        )
        corner = self.physics.advance(
            distance_m=(
                self.profile.progress[corner_index] * self.circuit.track_length_m
            ),
            speed_mps=18.0,
            delta_seconds=0.5,
            modifiers=self.modifiers,
        )
        straight = self.physics.advance(
            distance_m=(
                self.profile.progress[straight_index] * self.circuit.track_length_m
            ),
            speed_mps=18.0,
            delta_seconds=0.5,
            modifiers=self.modifiers,
        )

        self.assertGreater(straight.speed_mps, corner.speed_mps)
        self.assertGreater(straight.acceleration_mps2, corner.acceleration_mps2)

    def test_tire_traction_factor_changes_low_speed_acceleration(self) -> None:
        straight_index = self.profile.curvatures_1pm.index(
            min(self.profile.curvatures_1pm)
        )
        distance_m = self.profile.progress[straight_index] * self.circuit.track_length_m
        worn = self.physics.advance(
            distance_m=distance_m,
            speed_mps=18.0,
            delta_seconds=0.5,
            modifiers=VehiclePhysicsModifiers(traction=0.78),
        )
        fresh = self.physics.advance(
            distance_m=distance_m,
            speed_mps=18.0,
            delta_seconds=0.5,
            modifiers=VehiclePhysicsModifiers(traction=1.02),
        )

        self.assertGreater(fresh.speed_mps, worn.speed_mps)

    def test_planar_vehicle_physics_has_no_grade_modifier(self) -> None:
        self.assertNotIn(
            "track_grade",
            VehiclePhysicsModifiers.__dataclass_fields__,
        )

    def test_tire_braking_factor_changes_corner_entry_deceleration(self) -> None:
        # Red Bull Ring T3 approach: low current curvature with a tight corner
        # inside the predictive controller's braking horizon.
        distance_m = 0.29 * self.circuit.track_length_m
        worn = self.physics.advance(
            distance_m=distance_m,
            speed_mps=80.0,
            delta_seconds=0.5,
            modifiers=VehiclePhysicsModifiers(braking=0.78),
        )
        fresh = self.physics.advance(
            distance_m=distance_m,
            speed_mps=80.0,
            delta_seconds=0.5,
            modifiers=VehiclePhysicsModifiers(braking=1.02),
        )

        self.assertLess(fresh.speed_mps, worn.speed_mps)

    def test_tire_lateral_grip_changes_corner_speed_limit(self) -> None:
        corner_index = self.profile.curvatures_1pm.index(
            max(self.profile.curvatures_1pm)
        )
        distance_m = self.profile.progress[corner_index] * self.circuit.track_length_m
        worn_limit = self.physics._lateral_speed_limit_mps(
            distance_m,
            VehiclePhysicsModifiers(grip=0.82),
        )
        fresh_limit = self.physics._lateral_speed_limit_mps(
            distance_m,
            VehiclePhysicsModifiers(grip=1.02),
        )

        self.assertGreater(fresh_limit, worn_limit)

    def test_excess_corner_speed_creates_understeer_and_outward_drift(self) -> None:
        corner_index = self.profile.curvatures_1pm.index(
            max(self.profile.curvatures_1pm)
        )
        corner_distance = (
            self.profile.progress[corner_index] * self.circuit.track_length_m
        )
        lateral_limit = self.physics._lateral_speed_limit_mps(
            corner_distance,
            self.modifiers,
        )
        result = self.physics.advance(
            distance_m=corner_distance,
            speed_mps=lateral_limit * 1.10,
            delta_seconds=0.1,
            modifiers=self.modifiers,
            lateral_offset_m=0.0,
            target_lateral_offset_m=0.0,
        )
        turn_direction = (
            1.0
            if self.profile.signed_curvatures_1pm[corner_index] > 0.0
            else -1.0
        )

        self.assertEqual(result.handling_state, "understeer")
        self.assertGreater(result.grip_utilization, 1.0)
        self.assertLess(result.lateral_offset_m * turn_direction, 0.0)
        self.assertNotEqual(result.slip_angle_rad, 0.0)

    def test_large_grip_excess_creates_run_wide_state(self) -> None:
        corner_index = self.profile.curvatures_1pm.index(
            max(self.profile.curvatures_1pm)
        )
        result = self.physics.advance(
            distance_m=(
                self.profile.progress[corner_index] * self.circuit.track_length_m
            ),
            speed_mps=30.0,
            delta_seconds=0.02,
            modifiers=self.modifiers,
        )

        self.assertEqual(result.handling_state, "run_wide")
        # The dynamic bicycle model reports the physically integrated body
        # heading after one 20 ms step; it no longer injects an arbitrary
        # visual slip-angle floor for a run-wide classification.
        self.assertGreater(abs(result.slip_angle_rad), 0.02)

    def test_throttle_overload_in_a_corner_creates_oversteer(self) -> None:
        corner_index = min(
            range(len(self.profile.curvatures_1pm)),
            key=lambda index: abs(self.profile.curvatures_1pm[index] - 0.029),
        )
        result = self.physics.advance(
            distance_m=(
                self.profile.progress[corner_index] * self.circuit.track_length_m
            ),
            speed_mps=25.0,
            delta_seconds=0.02,
            modifiers=VehiclePhysicsModifiers(
                speed_limit_factor=1.5,
                pace=1.05,
            ),
        )

        self.assertGreater(result.throttle, 0.45)
        self.assertEqual(result.handling_state, "oversteer")
        self.assertNotEqual(result.slip_angle_rad, 0.0)
        self.assertGreater(result.traction_slip_ratio, 0.05)
        self.assertGreater(result.tire_slide_energy_j, 0.0)

    def test_trail_braking_over_combined_grip_limit_creates_lockup(self) -> None:
        corner_index = min(
            range(len(self.profile.curvatures_1pm)),
            key=lambda index: abs(self.profile.curvatures_1pm[index] - 0.029),
        )
        result = self.physics.advance(
            distance_m=(
                self.profile.progress[corner_index] * self.circuit.track_length_m
            ),
            speed_mps=25.0,
            delta_seconds=PHYSICS_STEP_SECONDS,
            modifiers=VehiclePhysicsModifiers(
                speed_limit_factor=0.2,
                emergency_braking=True,
            ),
        )

        self.assertEqual(result.handling_state, "lockup")
        self.assertGreater(result.wheel_lock_ratio, 0.05)
        self.assertGreater(result.tire_slide_energy_j, 0.0)
        self.assertGreater(result.brake, 0.85)

    def test_brake_input_error_only_becomes_lockup_through_tire_force_limit(self) -> None:
        corner_index = min(
            range(len(self.profile.curvatures_1pm)),
            key=lambda index: abs(self.profile.curvatures_1pm[index] - 0.029),
        )
        result = self.physics.advance(
            distance_m=self.profile.progress[corner_index] * self.circuit.track_length_m,
            speed_mps=25.0,
            delta_seconds=PHYSICS_STEP_SECONDS,
            modifiers=VehiclePhysicsModifiers(brake_modulation_error=0.22),
        )

        self.assertGreater(result.brake, 0.85)
        self.assertGreater(result.wheel_lock_ratio, 0.05)
        self.assertEqual(result.handling_state, "lockup")

    def test_throttle_input_error_only_becomes_wheelspin_through_tire_force_limit(self) -> None:
        corner_index = min(
            range(len(self.profile.curvatures_1pm)),
            key=lambda index: abs(self.profile.curvatures_1pm[index] - 0.029),
        )
        result = self.physics.advance(
            distance_m=self.profile.progress[corner_index] * self.circuit.track_length_m,
            speed_mps=24.0,
            delta_seconds=PHYSICS_STEP_SECONDS,
            modifiers=VehiclePhysicsModifiers(
                speed_limit_factor=1.5,
                throttle_modulation_error=0.22,
            ),
        )

        self.assertGreater(result.throttle, 0.75)
        self.assertGreater(result.traction_slip_ratio, 0.05)
        self.assertIn(result.handling_state, {"oversteer", "wheelspin"})

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

    def test_impossible_following_gap_brakes_without_speed_or_position_snap(self) -> None:
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

        self.assertGreater(
            result.distance_m,
            following.leader_end_distance_m - PHYSICAL_CAR_LENGTH_M,
        )
        self.assertGreater(result.speed_mps, 74.0)
        self.assertLess(result.speed_mps, 80.0)
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

    def test_nominal_bounds_hold_stable_car_inside_the_safety_envelope(self) -> None:
        stable = self.physics.advance(
            distance_m=100.0,
            speed_mps=45.0,
            delta_seconds=2.0,
            modifiers=self.modifiers,
            lateral_offset_m=0.0,
            lateral_speed_mps=0.0,
            target_lateral_offset_m=5.0,
            target_lateral_speed_mps=4.0,
            minimum_lateral_offset_m=-6.0,
            maximum_lateral_offset_m=6.0,
            nominal_minimum_lateral_offset_m=-3.0,
            nominal_maximum_lateral_offset_m=3.0,
        )

        self.assertEqual(stable.handling_state, "stable")
        self.assertLessEqual(stable.lateral_offset_m, 3.0)
        self.assertGreater(stable.lateral_offset_m, 0.0)

    def test_car_beyond_nominal_bounds_recovers_without_position_snap(self) -> None:
        start_offset_m = -5.0
        recovering = self.physics.advance(
            distance_m=100.0,
            speed_mps=35.0,
            delta_seconds=PHYSICS_STEP_SECONDS,
            modifiers=self.modifiers,
            lateral_offset_m=start_offset_m,
            lateral_speed_mps=0.0,
            target_lateral_offset_m=0.0,
            minimum_lateral_offset_m=-6.0,
            maximum_lateral_offset_m=6.0,
            nominal_minimum_lateral_offset_m=-3.0,
            nominal_maximum_lateral_offset_m=3.0,
        )

        self.assertEqual(recovering.handling_state, "recovering")
        self.assertGreater(recovering.lateral_offset_m, start_offset_m)
        self.assertLess(recovering.lateral_offset_m - start_offset_m, 0.10)


if __name__ == "__main__":
    unittest.main()
