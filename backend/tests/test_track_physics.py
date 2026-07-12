"""Tests for metric track width and generated racing lines."""

from __future__ import annotations

import unittest

from data_loader import load_circuits
from simulation.track_physics import (
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
    TRACK_EDGE_MARGIN_M,
    build_track_physics_profile,
)


class TrackPhysicsProfileTests(unittest.TestCase):
    def test_physical_car_dimensions_match_game_specification(self) -> None:
        self.assertEqual(PHYSICAL_CAR_WIDTH_M, 1.9)
        self.assertEqual(PHYSICAL_CAR_LENGTH_M, 5.0)

    def test_generated_racing_line_uses_track_width_without_crossing_edge(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        profile = build_track_physics_profile(circuit)

        self.assertGreater(len(profile.samples), 20)
        self.assertGreater(
            max(sample.racing_line_offset_m for sample in profile.samples),
            min(sample.racing_line_offset_m for sample in profile.samples),
        )
        for sample in profile.samples:
            self.assertLessEqual(
                sample.racing_line_offset_m,
                sample.left_width_m - PHYSICAL_CAR_WIDTH_M / 2 - TRACK_EDGE_MARGIN_M,
            )
            self.assertGreaterEqual(
                sample.racing_line_offset_m,
                -sample.right_width_m + PHYSICAL_CAR_WIDTH_M / 2 + TRACK_EDGE_MARGIN_M,
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


if __name__ == "__main__":
    unittest.main()
