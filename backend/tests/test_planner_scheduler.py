"""Tests for adaptive trajectory scheduling and progress spatial indexing."""

from __future__ import annotations

from types import SimpleNamespace
import unittest

from engines.full.runtime.planner_scheduler import (
    AdaptivePlannerScheduler,
    TrackSpatialIndex,
)


def _vehicle(
    driver_id: int,
    progress: float,
    *,
    retired: bool = False,
    finished: bool = False,
    in_pit: bool = False,
) -> SimpleNamespace:
    return SimpleNamespace(
        driver_id=driver_id,
        total_progress=progress,
        retired=retired,
        finished=finished,
        in_pit=in_pit,
    )


class AdaptivePlannerSchedulerTests(unittest.TestCase):
    def test_scheduler_exposes_all_four_validation_tiers(self) -> None:
        scheduler = AdaptivePlannerScheduler()

        cruise = scheduler.choose(
            emergency=False,
            battle=False,
            nearest_distance_m=None,
        )
        traffic = scheduler.choose(
            emergency=False,
            battle=False,
            nearest_distance_m=30.0,
        )
        battle = scheduler.choose(
            emergency=False,
            battle=True,
            nearest_distance_m=20.0,
        )
        critical = scheduler.choose(
            emergency=True,
            battle=True,
            nearest_distance_m=5.0,
        )

        self.assertEqual(
            tuple(item.tier_hz for item in (cruise, traffic, battle, critical)),
            (5, 10, 20, 50),
        )
        self.assertEqual(cruise.full_plan_interval_seconds, 1.0)
        self.assertEqual(critical.validation_interval_seconds, 0.02)
        self.assertGreater(
            critical.validation_interval_seconds,
            0.0,
        )
        self.assertEqual(
            battle.full_plan_interval_seconds,
            critical.full_plan_interval_seconds,
        )

    def test_spatial_index_matches_local_progress_neighborhood(self) -> None:
        vehicles = (
            _vehicle(1, 1.000),
            _vehicle(2, 1.025),
            _vehicle(3, 0.970),
            _vehicle(4, 1.200),
            _vehicle(5, 1.010, in_pit=True),
            _vehicle(6, 1.015, retired=True),
        )
        index = TrackSpatialIndex(track_length_m=1000.0, bucket_size_m=60.0)
        index.rebuild(vehicles)

        nearby = index.nearby(vehicles[0], radius_m=40.0)

        self.assertEqual(tuple(item.driver_id for item in nearby), (2, 3))

    def test_spatial_index_is_deterministic_for_equal_gaps(self) -> None:
        ego = _vehicle(10, 2.0)
        index = TrackSpatialIndex(track_length_m=1000.0)
        index.rebuild((_vehicle(3, 1.99), ego, _vehicle(2, 2.01)))

        self.assertEqual(
            tuple(item.driver_id for item in index.nearby(ego, 20.0)),
            (2, 3),
        )


if __name__ == "__main__":
    unittest.main()
