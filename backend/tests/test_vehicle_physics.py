"""Tests for the optional fixed-step longitudinal vehicle physics."""

from __future__ import annotations

import unittest

from data_loader import load_circuits
from simulation.speed_profile import SpeedProfile, build_speed_profile
from simulation.track_physics import PHYSICAL_CAR_LENGTH_M
from simulation.vehicle_physics import (
    PHYSICS_STEP_SECONDS,
    PREDICTIVE_SPEED_CACHE_MAX_ENTRIES,
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

    def test_controller_scale_cache_key_distinguishes_profile_shape(self) -> None:
        reordered_profile = SpeedProfile(
            progress=self.profile.progress,
            raw_speeds_mps=(
                self.profile.raw_speeds_mps[1:]
                + self.profile.raw_speeds_mps[:1]
            ),
            curvatures_1pm=self.profile.curvatures_1pm,
            signed_curvatures_1pm=self.profile.signed_curvatures_1pm,
            braking_fractions=self.profile.braking_fractions,
        )
        reordered = LongitudinalVehiclePhysics(
            reordered_profile,
            self.circuit.track_length_m,
            self.circuit.base_lap_time,
        )

        self.assertNotEqual(
            self.physics._controller_scale_key(),
            reordered._controller_scale_key(),
        )

    def test_controller_scale_cache_key_includes_braking_curvature_threshold(self) -> None:
        calibrated = LongitudinalVehiclePhysics(
            self.profile,
            self.circuit.track_length_m,
            self.circuit.base_lap_time,
            telemetry_braking_curvature_threshold=0.05,
        )

        self.assertNotEqual(
            self.physics._controller_scale_key(),
            calibrated._controller_scale_key(),
        )

    def test_telemetry_calibration_never_tightens_physical_geometry(self) -> None:
        telemetry_profile = SpeedProfile(
            progress=[0.0, 0.25, 0.5, 0.75],
            raw_speeds_mps=[20.0, 24.0, 38.0, 31.0],
            curvatures_1pm=[0.004, 0.006, 0.005, 0.007],
            signed_curvatures_1pm=[0.004, 0.006, 0.005, 0.007],
            braking_fractions=[0.0, 0.0, 0.0, 1.0],
        )
        calibrated = LongitudinalVehiclePhysics(
            telemetry_profile,
            4000.0,
            70.0,
            telemetry_speed_reference_weight=1.0,
            telemetry_braking_curvature_threshold=0.5,
        )

        for progress, raw_curvature in zip(
            telemetry_profile.progress,
            telemetry_profile.curvatures_1pm,
        ):
            self.assertLessEqual(
                calibrated._curvature_1pm(progress * 4000.0),
                raw_curvature + 1e-12,
            )

    def test_predictive_speed_cache_is_bounded_across_vehicle_states(
        self,
    ) -> None:
        distance_m = 0.27 * self.circuit.track_length_m
        first_target_speed_mps = self.physics.target_speed_mps(
            distance_m,
            VehiclePhysicsModifiers(mass_kg=700.0),
        )
        for index in range(PREDICTIVE_SPEED_CACHE_MAX_ENTRIES + 17):
            self.physics.target_speed_mps(
                distance_m,
                VehiclePhysicsModifiers(mass_kg=700.0 + index * 0.1),
            )

        self.assertEqual(
            len(self.physics._predictive_speed_cache),
            PREDICTIVE_SPEED_CACHE_MAX_ENTRIES,
        )
        self.assertAlmostEqual(
            self.physics.target_speed_mps(
                distance_m,
                VehiclePhysicsModifiers(mass_kg=700.0),
            ),
            first_target_speed_mps,
            places=12,
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

    def test_race_control_cap_can_be_below_nominal_physics_floor(self) -> None:
        distance_m = 0.27 * self.circuit.track_length_m

        stopped_target = self.physics.target_speed_mps(
            distance_m,
            VehiclePhysicsModifiers(maximum_speed_mps=0.0),
        )
        low_controlled_target = self.physics.target_speed_mps(
            distance_m,
            VehiclePhysicsModifiers(maximum_speed_mps=5.0),
        )

        self.assertEqual(stopped_target, 0.0)
        self.assertEqual(low_controlled_target, 5.0)

    def test_braking_transfers_load_forward_and_exposes_axle_state(self) -> None:
        slow_index = self.profile.raw_speeds_mps.index(
            min(self.profile.raw_speeds_mps)
        )
        braking = self.physics.advance(
            distance_m=(
                self.profile.progress[slow_index]
                * self.circuit.track_length_m
                - 100.0
            ) % self.circuit.track_length_m,
            speed_mps=90.0,
            delta_seconds=PHYSICS_STEP_SECONDS,
            modifiers=self.modifiers,
        )
        total_load_n = braking.front_normal_load_n + braking.rear_normal_load_n
        static_front_load_n = total_load_n * self.modifiers.front_aero_share

        self.assertLess(braking.acceleration_mps2, 0.0)
        self.assertGreater(braking.longitudinal_load_transfer_n, 0.0)
        self.assertGreater(braking.front_normal_load_n, static_front_load_n)
        self.assertGreaterEqual(braking.front_wheel_speed_rad_s, 0.0)
        self.assertGreaterEqual(braking.rear_wheel_speed_rad_s, 0.0)
        self.assertLessEqual(braking.front_axle_slip_ratio, 0.0)
        self.assertLessEqual(braking.rear_axle_slip_ratio, 0.0)
        self.assertGreater(braking.applied_brake_force_n, 0.0)

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

    def test_clean_attack_throttle_keeps_corner_exit_slip_below_event_band(self) -> None:
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
        self.assertEqual(result.handling_state, "stable")
        self.assertNotEqual(result.slip_angle_rad, 0.0)
        self.assertGreater(result.traction_slip_ratio, 0.02)
        self.assertLess(result.traction_slip_ratio, 0.05)
        self.assertAlmostEqual(
            result.tire_slide_energy_j,
            result.front_tire_slide_energy_j + result.rear_tire_slide_energy_j,
            places=9,
        )
        self.assertAlmostEqual(result.front_tire_slide_energy_j, 0.0, places=9)
        self.assertGreater(result.rear_tire_slide_energy_j, 0.0)
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
        self.assertAlmostEqual(
            result.tire_slide_energy_j,
            result.front_tire_slide_energy_j + result.rear_tire_slide_energy_j,
            places=9,
        )
        self.assertGreater(result.front_tire_slide_energy_j, 0.0)
        self.assertGreaterEqual(result.front_tire_slide_energy_j, result.rear_tire_slide_energy_j)
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
            distance_m=95.1,
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

    def test_legal_following_gap_cannot_cross_the_minimum_body_gap(self) -> None:
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
            following.leader_end_distance_m - following.minimum_gap_m + 1e-9,
        )
        self.assertLess(result.speed_mps, 80.0)

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

    def test_reference_frame_transport_is_not_clamped_as_manoeuvre_speed(self) -> None:
        straight_index = min(
            range(len(self.profile.curvatures_1pm)),
            key=lambda index: abs(self.profile.curvatures_1pm[index]),
        )
        distance_m = self.profile.progress[straight_index] * self.circuit.track_length_m
        result = self.physics.advance(
            distance_m=distance_m,
            speed_mps=55.0,
            delta_seconds=PHYSICS_STEP_SECONDS,
            modifiers=self.modifiers,
            lateral_offset_m=0.0,
            lateral_speed_mps=2.5,
            target_lateral_offset_m=0.05,
            target_lateral_speed_mps=2.5,
            reference_lateral_offset_m=0.0,
            reference_lateral_speed_mps=2.5,
            maximum_lateral_speed_mps=1.25,
            minimum_lateral_offset_m=-6.0,
            maximum_lateral_offset_m=6.0,
        )

        self.assertGreater(result.lateral_speed_mps, 1.25)
        self.assertGreater(result.lateral_offset_m, 0.0)

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

    def test_inward_recovery_does_not_keep_applying_the_outward_speed_cap(self) -> None:
        release_physics = LongitudinalVehiclePhysics(
            self.profile,
            self.circuit.track_length_m,
            self.circuit.base_lap_time,
            release_inward_recovery_speed_cap=True,
        )
        common = {
            "distance_m": 100.0,
            "speed_mps": 35.0,
            "delta_seconds": 0.1,
            "modifiers": self.modifiers,
            "lateral_offset_m": 3.5,
            "target_lateral_offset_m": 0.0,
            "minimum_lateral_offset_m": -6.0,
            "maximum_lateral_offset_m": 6.0,
            "nominal_minimum_lateral_offset_m": -3.0,
            "nominal_maximum_lateral_offset_m": 3.0,
        }
        moving_inward = release_physics.advance(
            **common,
            lateral_speed_mps=-1.0,
        )
        moving_outward = release_physics.advance(
            **common,
            lateral_speed_mps=1.0,
        )

        self.assertEqual(moving_inward.handling_state, "recovering")
        self.assertGreater(moving_inward.speed_mps, moving_outward.speed_mps)


if __name__ == "__main__":
    unittest.main()
