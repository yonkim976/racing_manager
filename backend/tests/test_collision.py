import unittest

from engines.full.runtime.collision import (
    BodyMotion,
    BodyPose,
    oriented_body_overlap,
    oriented_body_separation_m,
    swept_body_collision,
)


def _pose(x: float, y: float, *, heading: float = 0.0, vx: float = 0.0, vy: float = 0.0) -> BodyPose:
    return BodyPose(x, y, heading, 5.0, 1.9, vx, vy)


class CollisionGeometryTests(unittest.TestCase):
    def test_separated_rotated_bodies_do_not_overlap(self) -> None:
        self.assertIsNone(oriented_body_overlap(_pose(0.0, 0.0), _pose(0.0, 2.5, heading=0.1)))

    def test_rotated_body_overlap_returns_penetration_and_normal(self) -> None:
        result = oriented_body_overlap(_pose(0.0, 0.0), _pose(4.5, 0.2, heading=0.08))
        self.assertIsNotNone(result)
        penetration, normal = result or (0.0, (0.0, 0.0))
        self.assertGreaterEqual(penetration, 0.0)
        self.assertGreater(normal[0], 0.0)

    def test_oriented_body_separation_is_zero_on_overlap_and_positive_apart(self) -> None:
        self.assertEqual(
            oriented_body_separation_m(_pose(0.0, 0.0), _pose(4.0, 0.0)),
            0.0,
        )
        self.assertAlmostEqual(
            oriented_body_separation_m(_pose(0.0, 0.0), _pose(7.0, 0.0)),
            2.0,
        )

    def test_swept_test_finds_fast_rear_end_before_end_pose(self) -> None:
        first = BodyMotion(_pose(10.0, 0.0, vx=40.0), _pose(14.0, 0.0, vx=40.0))
        second = BodyMotion(_pose(0.0, 0.0, vx=100.0), _pose(10.0, 0.0, vx=100.0))
        contact = swept_body_collision(first, second, substeps=5)
        self.assertIsNotNone(contact)
        assert contact is not None
        self.assertGreater(contact.time_fraction, 0.0)
        self.assertLess(contact.time_fraction, 1.0)
        self.assertAlmostEqual(contact.impact_speed_mps, 60.0, places=3)
        self.assertEqual(contact.contact_type, "front_rear")

    def test_swept_test_detects_lateral_side_contact(self) -> None:
        first = BodyMotion(_pose(0.0, 0.0), _pose(5.0, 0.0, vx=50.0))
        second = BodyMotion(
            _pose(0.0, 3.0, vy=-12.0),
            _pose(5.0, 1.8, vx=50.0, vy=-12.0),
        )
        contact = swept_body_collision(first, second, substeps=5)
        self.assertIsNotNone(contact)
        assert contact is not None
        self.assertEqual(contact.contact_type, "side")
        self.assertGreater(contact.impact_speed_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
