"""Regression tests for the explainable Bahrain racecraft benchmark."""

from __future__ import annotations

import unittest

from simulation.racecraft_benchmark import (
    BAHRAIN_RACECRAFT_SCENARIOS,
    run_bahrain_racecraft_benchmark,
)


class BahrainRacecraftBenchmarkTests(unittest.TestCase):
    def test_v1_decisions_reasons_and_pit_priority_are_deterministic(self) -> None:
        report = run_bahrain_racecraft_benchmark([99], duration_seconds=0.0)

        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["track_geometry_mode"], "planar_2d")
        self.assertEqual(
            report["summary"]["scenario_runs"],
            len(BAHRAIN_RACECRAFT_SCENARIOS),
        )
        self.assertEqual(report["summary"]["decision_match_rate"], 1.0)
        self.assertEqual(report["summary"]["reason_match_rate"], 1.0)
        self.assertEqual(
            report["summary"]["pit_merge_sequence_match_rate"],
            1.0,
        )

    def test_short_physical_window_remains_contact_and_track_limit_free(self) -> None:
        report = run_bahrain_racecraft_benchmark([42], duration_seconds=0.2)

        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["summary"]["contact_runs"], 0)
        self.assertEqual(report["summary"]["off_track_runs"], 0)


if __name__ == "__main__":
    unittest.main()
