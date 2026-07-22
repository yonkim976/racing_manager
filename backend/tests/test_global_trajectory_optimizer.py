"""Tests for the model-independent whole-lap trajectory search."""

from __future__ import annotations

import unittest

from simulation.global_trajectory_optimizer import (
    GlobalTrajectoryEvaluation,
    GlobalTrajectoryOptimizationRequest,
    GlobalTrajectoryOptimizer,
    GlobalTrajectoryOptimizerConfig,
)


class _QuadraticCostModel:
    def prepare_initial_offsets(self, offsets_m) -> tuple[float, ...]:
        return tuple(offsets_m)

    def prepare_candidate_offsets(self, offsets_m) -> tuple[float, ...]:
        return tuple(offsets_m)

    def evaluate(self, offsets_m, reference_offsets_m):
        offsets = tuple(offsets_m)
        objective = sum(offset * offset for offset in offsets)
        return GlobalTrajectoryEvaluation(
            offsets_m=offsets,
            path_points=tuple((float(index), offset) for index, offset in enumerate(offsets)),
            segment_lengths_m=tuple(1.0 for _ in offsets),
            curvatures_1pm=tuple(0.0 for _ in offsets),
            lap_time_seconds=objective,
            objective_cost=objective,
        )


class GlobalTrajectoryOptimizerTests(unittest.TestCase):
    def test_search_is_deterministic_and_improves_pluggable_cost(self) -> None:
        request = GlobalTrajectoryOptimizationRequest(
            initial_offsets_m=(1.0,) * 9,
            track_length_m=900.0,
        )
        optimizer = GlobalTrajectoryOptimizer(
            GlobalTrajectoryOptimizerConfig(
                optimization_steps_m=(0.5, 0.25),
                transition_radius_m=220.0,
            )
        )

        first = optimizer.optimize(request, _QuadraticCostModel())
        second = optimizer.optimize(request, _QuadraticCostModel())

        self.assertEqual(first, second)
        self.assertLess(
            first.evaluation.objective_cost,
            sum(offset * offset for offset in request.initial_offsets_m),
        )
        self.assertEqual(first.diagnostics.attempted_candidates, 9 * 2 * 2)
        self.assertEqual(
            first.diagnostics.evaluated_candidates,
            first.diagnostics.attempted_candidates,
        )
        self.assertGreater(first.diagnostics.accepted_updates, 0)
        self.assertGreaterEqual(first.diagnostics.duration_ms, 0.0)

    def test_request_rejects_empty_or_non_positive_tracks(self) -> None:
        with self.assertRaises(ValueError):
            GlobalTrajectoryOptimizationRequest(
                initial_offsets_m=(),
                track_length_m=100.0,
            )
        with self.assertRaises(ValueError):
            GlobalTrajectoryOptimizationRequest(
                initial_offsets_m=(0.0,),
                track_length_m=0.0,
            )


if __name__ == "__main__":
    unittest.main()
