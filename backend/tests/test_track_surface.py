import unittest

from data_loader import load_circuits
from models.schemas import RunoffSurface, TrackSide, TrackSurfaceZone
from simulation.track_physics import build_track_physics_profile
from simulation.track_surface import (
    CAR_WHEEL_TRACK_M,
    TRAJECTORY_LOW_KERB_ALLOWANCE_M,
    TrackSurfaceProfile,
    WheelSurface,
)


class TrackSurfaceProfileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        self.track = build_track_physics_profile(self.circuit)
        self.surface = TrackSurfaceProfile.for_circuit(self.circuit, self.track)

    def test_auto_profile_generates_real_per_side_kerb_ranges(self) -> None:
        self.assertTrue(self.surface.zones)
        self.assertTrue(all(zone.generated for zone in self.surface.zones))
        self.assertEqual(
            {zone.side for zone in self.surface.zones},
            {TrackSide.LEFT, TrackSide.RIGHT},
        )

    def test_white_line_kerb_runoff_and_grass_are_distinct(self) -> None:
        zone = self.surface.zones[0]
        progress = (zone.start + ((zone.end - zone.start) % 1.0) / 2.0) % 1.0
        track = self.track.at_progress(progress)
        boundary = track.left_width_m if zone.side == TrackSide.LEFT else -track.right_width_m
        direction = 1.0 if zone.side == TrackSide.LEFT else -1.0

        self.assertEqual(
            self.surface.classify_point(progress, boundary - direction * 0.05)[0],
            WheelSurface.TRACK,
        )
        self.assertEqual(
            self.surface.classify_point(progress, boundary + direction * 0.5)[0],
            WheelSurface.KERB_LOW,
        )
        self.assertEqual(
            self.surface.classify_point(
                progress,
                boundary + direction * (zone.kerb_width_m + 0.5),
            )[0],
            WheelSurface.ASPHALT_RUNOFF,
        )
        self.assertEqual(
            self.surface.classify_point(
                progress,
                boundary
                + direction * (zone.kerb_width_m + zone.runoff_width_m + 0.5),
            )[0],
            WheelSurface.GRASS,
        )

    def test_four_wheel_contact_records_track_limit_only_when_all_are_outside(self) -> None:
        progress = 0.0
        track = self.track.at_progress(progress)
        half_wheel_track = CAR_WHEEL_TRACK_M / 2.0
        partial = self.surface.vehicle_state(
            progress=progress,
            lateral_offset_m=track.left_width_m - half_wheel_track + 0.1,
            track_length_m=self.circuit.track_length_m,
        )
        fully_out = self.surface.vehicle_state(
            progress=progress,
            lateral_offset_m=track.left_width_m + half_wheel_track + 0.1,
            track_length_m=self.circuit.track_length_m,
        )
        self.assertFalse(partial.track_limits_active)
        self.assertTrue(fully_out.track_limits_active)
        self.assertEqual(len(fully_out.contacts), 4)

    def test_two_wheels_may_use_full_low_kerb_without_track_limits(self) -> None:
        zone = self.surface.zones[0]
        progress = (zone.start + ((zone.end - zone.start) % 1.0) / 2.0) % 1.0
        minimum_m, maximum_m = self.surface.trajectory_body_lateral_bounds(
            progress,
            body_width_m=1.9,
            edge_margin_m=0.35,
        )
        center_m = maximum_m if zone.side == TrackSide.LEFT else minimum_m
        state = self.surface.vehicle_state(
            progress=progress,
            lateral_offset_m=center_m,
            track_length_m=self.circuit.track_length_m,
        )

        self.assertFalse(state.track_limits_active)
        self.assertEqual(state.wheel_surfaces.count(WheelSurface.TRACK.value), 2)
        self.assertEqual(state.wheel_surfaces.count(WheelSurface.KERB_LOW.value), 2)

        optimized_left_m, optimized_right_m = (
            self.surface.trajectory_optimization_widths(progress)
        )
        body_left_m, body_right_m = self.surface.trajectory_body_widths(progress)
        if zone.side == TrackSide.LEFT:
            self.assertAlmostEqual(
                body_left_m - optimized_left_m,
                zone.kerb_width_m - TRAJECTORY_LOW_KERB_ALLOWANCE_M,
            )
        else:
            self.assertAlmostEqual(
                body_right_m - optimized_right_m,
                zone.kerb_width_m - TRAJECTORY_LOW_KERB_ALLOWANCE_M,
            )

    def test_explicit_gravel_zone_overrides_generated_profile(self) -> None:
        zone = TrackSurfaceZone(
            start=0.0,
            end=0.1,
            side=TrackSide.LEFT,
            kerb_width_m=0.0,
            runoff_surface=RunoffSurface.GRAVEL,
            runoff_width_m=8.0,
        )
        surface = TrackSurfaceProfile(self.track, [zone])
        boundary = self.track.at_progress(0.05).left_width_m
        state = surface.vehicle_state(
            progress=0.05,
            lateral_offset_m=boundary + CAR_WHEEL_TRACK_M / 2.0 + 0.5,
            track_length_m=self.circuit.track_length_m,
        )
        self.assertEqual(state.surface_state, WheelSurface.GRAVEL)
        self.assertGreater(state.drag_deceleration_mps2, 0.0)
        self.assertLess(state.lateral_grip_multiplier, 1.0)

    def test_trajectory_assessment_allows_low_kerb_but_penalizes_runoff(self) -> None:
        zone = self.surface.zones[0]
        progress = (zone.start + ((zone.end - zone.start) % 1.0) / 2.0) % 1.0
        direction = 1.0 if zone.side == TrackSide.LEFT else -1.0
        body_half_width_m = 1.9 / 2.0
        edge_margin_m = 0.35
        minimum_m, maximum_m = self.surface.trajectory_body_lateral_bounds(
            progress,
            body_width_m=1.9,
            edge_margin_m=edge_margin_m,
        )
        low_kerb_center_m = (
            maximum_m - 0.01
            if zone.side == TrackSide.LEFT
            else minimum_m + 0.01
        )
        low_kerb = self.surface.assess_trajectory(
            [progress],
            [low_kerb_center_m],
            track_length_m=self.circuit.track_length_m,
            body_width_m=1.9,
            body_length_m=5.0,
            edge_margin_m=edge_margin_m,
        )
        self.assertEqual(low_kerb.body_boundary_violations, 0)
        self.assertGreater(low_kerb.low_kerb_contacts, 0)
        self.assertGreater(low_kerb.cost_seconds, 0.0)

        track = self.track.at_progress(progress)
        boundary = (
            track.left_width_m
            if zone.side == TrackSide.LEFT
            else -track.right_width_m
        )
        runoff_center_m = boundary + direction * (zone.kerb_width_m + 0.5)
        runoff = self.surface.assess_trajectory(
            [progress],
            [runoff_center_m],
            track_length_m=self.circuit.track_length_m,
            body_width_m=1.9,
            body_length_m=5.0,
            edge_margin_m=edge_margin_m,
        )
        self.assertGreater(runoff.body_boundary_violations, 0)
        self.assertGreater(runoff.runoff_contacts, 0)
        self.assertGreater(runoff.cost_seconds, low_kerb.cost_seconds)

    def test_high_kerb_does_not_extend_trajectory_optimization_width(self) -> None:
        progress = 0.05
        zone = TrackSurfaceZone(
            start=0.0,
            end=0.1,
            side=TrackSide.LEFT,
            kerb_width_m=1.2,
            kerb_height="high",
            runoff_surface=RunoffSurface.ASPHALT,
            runoff_width_m=3.0,
        )
        surface = TrackSurfaceProfile(self.track, [zone])
        track = self.track.at_progress(progress)
        left_width_m, right_width_m = surface.trajectory_optimization_widths(progress)

        self.assertEqual(left_width_m, track.left_width_m)
        self.assertEqual(right_width_m, track.right_width_m)
        self.assertEqual(
            surface.trajectory_kerb_allowance_m(progress, TrackSide.LEFT),
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
