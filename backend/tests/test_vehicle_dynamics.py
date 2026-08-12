"""Focused tests for the shared planar dynamic bicycle model."""

from __future__ import annotations

import math
import unittest

from engines.full.runtime.vehicle_dynamics import (
    DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD,
    DynamicBicycleState,
    advance_dynamic_bicycle,
)


class DynamicBicycleModelTests(unittest.TestCase):
    def _advance(
        self,
        state: DynamicBicycleState,
        *,
        curvature_1pm: float,
        target_lateral_offset_m: float,
    ):
        return advance_dynamic_bicycle(
            state,
            speed_mps=60.0,
            curvature_1pm=curvature_1pm,
            target_lateral_offset_m=target_lateral_offset_m,
            target_lateral_speed_mps=0.0,
            delta_seconds=0.02,
            mass_kg=768.0,
            wheelbase_m=3.4,
            yaw_inertia_kgm2=1700.0,
            maximum_tire_force_n=25000.0,
            front_force_share=0.455,
            grip_factor=1.0,
        )

    def test_constant_radius_motion_converges_to_path_yaw_rate(self) -> None:
        curvature = 0.008
        state = DynamicBicycleState(0.0, 0.0, 0.0, 0.0)
        maximum_offset_m = 0.0
        for _ in range(500):
            result = self._advance(
                state,
                curvature_1pm=curvature,
                target_lateral_offset_m=0.0,
            )
            state = DynamicBicycleState(
                result.lateral_offset_m,
                result.lateral_speed_mps,
                result.heading_error_rad,
                result.yaw_rate_rad_s,
            )
            maximum_offset_m = max(maximum_offset_m, abs(state.lateral_offset_m))

        self.assertAlmostEqual(state.yaw_rate_rad_s, 60.0 * curvature, places=3)
        self.assertLess(abs(state.lateral_offset_m), 0.5)
        self.assertLess(maximum_offset_m, 2.5)
        self.assertNotEqual(result.front_slip_angle_rad, 0.0)
        self.assertNotEqual(result.rear_slip_angle_rad, 0.0)
        self.assertLessEqual(abs(state.heading_error_rad), math.radians(6.1))

    def test_lateral_acceleration_comes_from_tire_force_not_track_frame_jump(self) -> None:
        maximum_tire_force_n = 25_000.0
        mass_kg = 768.0
        result = advance_dynamic_bicycle(
            DynamicBicycleState(
                lateral_offset_m=0.0,
                lateral_speed_mps=-8.0,
                heading_error_rad=0.224,
                yaw_rate_rad_s=1.47,
            ),
            speed_mps=30.0,
            curvature_1pm=0.04,
            target_lateral_offset_m=0.0,
            target_lateral_speed_mps=0.0,
            delta_seconds=0.02,
            mass_kg=mass_kg,
            wheelbase_m=3.4,
            yaw_inertia_kgm2=1700.0,
            maximum_tire_force_n=maximum_tire_force_n,
            front_force_share=0.455,
            grip_factor=1.0,
        )

        self.assertLessEqual(
            abs(result.lateral_acceleration_mps2),
            maximum_tire_force_n / mass_kg + 1e-9,
        )

    def test_aero_loaded_constant_radius_does_not_ride_heading_clamp(self) -> None:
        """A feasible fast corner must not masquerade as a grip-limit loss."""
        speed_mps = 55.0
        curvature_1pm = 0.014
        state = DynamicBicycleState(0.0, 0.0, 0.0, 0.0)
        heading_clamp_steps = 0
        maximum_offset_m = 0.0

        for _ in range(500):
            result = advance_dynamic_bicycle(
                state,
                speed_mps=speed_mps,
                curvature_1pm=curvature_1pm,
                target_lateral_offset_m=0.0,
                target_lateral_speed_mps=0.0,
                delta_seconds=0.02,
                mass_kg=850.0,
                wheelbase_m=3.4,
                yaw_inertia_kgm2=1700.0,
                maximum_tire_force_n=45_000.0,
                nominal_tire_force_n=45_000.0,
                front_force_share=0.455,
                grip_factor=1.0,
            )
            state = DynamicBicycleState(
                result.lateral_offset_m,
                result.lateral_speed_mps,
                result.heading_error_rad,
                result.yaw_rate_rad_s,
            )
            maximum_offset_m = max(maximum_offset_m, abs(state.lateral_offset_m))
            if abs(state.heading_error_rad) >= (
                DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD - 1e-9
            ):
                heading_clamp_steps += 1

        self.assertLessEqual(heading_clamp_steps, 10)
        self.assertLess(maximum_offset_m, 1.1)
        self.assertLess(abs(state.lateral_offset_m), 0.3)
        self.assertAlmostEqual(
            state.yaw_rate_rad_s,
            speed_mps * curvature_1pm,
            places=3,
        )

    def test_straight_lane_change_approaches_target_without_position_snap(self) -> None:
        state = DynamicBicycleState(0.0, 0.0, 0.0, 0.0)
        first_step_offset_m = 0.0
        for index in range(500):
            result = self._advance(
                state,
                curvature_1pm=0.0,
                target_lateral_offset_m=3.0,
            )
            if index == 0:
                first_step_offset_m = result.lateral_offset_m
            state = DynamicBicycleState(
                result.lateral_offset_m,
                result.lateral_speed_mps,
                result.heading_error_rad,
                result.yaw_rate_rad_s,
            )

        self.assertLess(abs(first_step_offset_m), 0.02)
        self.assertAlmostEqual(state.lateral_offset_m, 3.0, places=3)
        self.assertLess(abs(state.lateral_speed_mps), 0.001)
        self.assertLess(abs(state.heading_error_rad), 0.001)


if __name__ == "__main__":
    unittest.main()
