"""Tests for continuous towing and dirty-air effects."""

from __future__ import annotations

import unittest

from simulation.wake_model import compute_wake_effects


class WakeModelTests(unittest.TestCase):
    def test_gap_alignment_and_curvature_golden_map(self) -> None:
        """Pin the aerodynamic contract across distance, offset, and corner load."""
        golden = {
            (20.0, 0.0, 0.0): (
                0.950805,
                0.950805,
                0.0,
                0.904919,
                1.0,
                1.0,
                1.0,
            ),
            (20.0, 0.0, 0.00675): (
                0.950805,
                0.570483,
                0.475403,
                0.942952,
                0.926313,
                0.961968,
                0.990492,
            ),
            (20.0, 0.0, 0.012): (
                0.950805,
                0.190161,
                0.950805,
                0.980984,
                0.852625,
                0.923936,
                0.980984,
            ),
            (20.0, 2.0, 0.012): (
                0.310710,
                0.062142,
                0.310710,
                0.993786,
                0.951840,
                0.975143,
                0.993786,
            ),
            (20.0, 5.0, 0.012): (
                0.0,
                0.0,
                0.0,
                1.0,
                1.0,
                1.0,
                1.0,
            ),
            (40.0, 0.0, 0.0): (
                0.774663,
                0.774663,
                0.0,
                0.922534,
                1.0,
                1.0,
                1.0,
            ),
            (40.0, 2.0, 0.00675): (
                0.413722,
                0.248233,
                0.206861,
                0.975177,
                0.967937,
                0.983451,
                0.995863,
            ),
            (80.0, 0.0, 0.012): (
                0.276740,
                0.055348,
                0.276740,
                0.994465,
                0.957105,
                0.977861,
                0.994465,
            ),
        }

        for inputs, expected in golden.items():
            with self.subTest(gap_m=inputs[0], lateral_m=inputs[1], curvature=inputs[2]):
                effects = compute_wake_effects(*inputs, speed_mps=75.0)
                actual = tuple(
                    round(value, 6)
                    for value in (
                        effects.wake_strength,
                        effects.tow_strength,
                        effects.dirty_air_strength,
                        effects.drag_multiplier,
                        effects.downforce_multiplier,
                        effects.braking_grip_multiplier,
                        effects.lateral_grip_multiplier,
                    )
                )
                self.assertEqual(actual, expected)

    def test_close_aligned_car_receives_stronger_wake(self) -> None:
        close = compute_wake_effects(20.0, 0.0, 0.0, 75.0)
        far = compute_wake_effects(90.0, 0.0, 0.0, 75.0)

        self.assertGreater(close.wake_strength, far.wake_strength)
        self.assertLess(close.drag_multiplier, far.drag_multiplier)

    def test_moving_laterally_out_of_wake_removes_effect(self) -> None:
        aligned = compute_wake_effects(35.0, 0.2, 0.0, 75.0)
        offset = compute_wake_effects(35.0, 5.0, 0.0, 75.0)

        self.assertGreater(aligned.wake_strength, offset.wake_strength)
        self.assertAlmostEqual(offset.drag_multiplier, 1.0)

    def test_straight_wake_prioritizes_tow(self) -> None:
        effects = compute_wake_effects(30.0, 0.0, 0.0, 75.0)

        self.assertGreater(effects.tow_strength, effects.dirty_air_strength)
        self.assertEqual(effects.dirty_air_strength, 0.0)
        self.assertLess(effects.drag_multiplier, 1.0)

    def test_small_curvature_noise_does_not_activate_dirty_air(self) -> None:
        effects = compute_wake_effects(30.0, 0.0, 0.0010, 75.0)

        self.assertGreater(effects.tow_strength, 0.0)
        self.assertEqual(effects.dirty_air_strength, 0.0)
        self.assertEqual(effects.downforce_multiplier, 1.0)

    def test_corner_wake_prioritizes_dirty_air(self) -> None:
        straight = compute_wake_effects(30.0, 0.0, 0.0, 75.0)
        corner = compute_wake_effects(30.0, 0.0, 0.020, 75.0)

        self.assertGreater(corner.dirty_air_strength, corner.tow_strength)
        self.assertLess(corner.downforce_multiplier, straight.downforce_multiplier)
        self.assertLess(
            corner.braking_grip_multiplier,
            straight.braking_grip_multiplier,
        )

    def test_low_speed_wake_is_weak(self) -> None:
        slow = compute_wake_effects(20.0, 0.0, 0.0, 12.0)

        self.assertEqual(slow.wake_strength, 0.0)


if __name__ == "__main__":
    unittest.main()
