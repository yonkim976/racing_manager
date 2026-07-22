"""Generic whole-lap lateral trajectory optimization interface.

The optimizer owns only the deterministic search policy.  Track geometry,
vehicle physics and tyre behaviour live behind ``GlobalTrajectoryCostModel`` so
the current curvature model can be replaced without changing the search or its
callers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, pi
from time import perf_counter
from typing import Protocol, Sequence

Point = tuple[float, float]


@dataclass(frozen=True)
class GlobalTrajectoryOptimizationRequest:
    """Whole-lap lateral offsets and track scale supplied to the optimizer."""

    initial_offsets_m: tuple[float, ...]
    track_length_m: float

    def __post_init__(self) -> None:
        if not self.initial_offsets_m:
            raise ValueError("global trajectory requires at least one offset sample")
        if self.track_length_m <= 0.0:
            raise ValueError("global trajectory track length must be positive")


@dataclass(frozen=True)
class GlobalTrajectoryOptimizerConfig:
    """Deterministic coordinate-search settings independent of the cost model."""

    optimization_steps_m: tuple[float, ...] = (1.25, 0.6, 0.3)
    transition_radius_m: float = 220.0
    objective_epsilon: float = 1e-6
    center_stride: int = 1


@dataclass(frozen=True)
class GlobalTrajectoryEvaluation:
    """One complete trajectory evaluated by a pluggable physical cost model."""

    offsets_m: tuple[float, ...]
    path_points: tuple[Point, ...]
    segment_lengths_m: tuple[float, ...]
    curvatures_1pm: tuple[float, ...]
    lap_time_seconds: float
    objective_cost: float
    target_speeds_mps: tuple[float, ...] = ()
    brake_utilization: tuple[float, ...] = ()
    throttle_utilization: tuple[float, ...] = ()


@dataclass(frozen=True)
class GlobalTrajectoryOptimizationDiagnostics:
    """Cold-build work counters used by performance regression tooling."""

    attempted_candidates: int
    evaluated_candidates: int
    accepted_updates: int
    initial_objective_cost: float
    final_objective_cost: float
    duration_ms: float = field(compare=False)


@dataclass(frozen=True)
class GlobalTrajectoryOptimizationResult:
    evaluation: GlobalTrajectoryEvaluation
    diagnostics: GlobalTrajectoryOptimizationDiagnostics


class GlobalTrajectoryCostModel(Protocol):
    """Geometry/physics adapter consumed by the generic coordinate search."""

    def prepare_initial_offsets(
        self,
        offsets_m: Sequence[float],
    ) -> tuple[float, ...]: ...

    def prepare_candidate_offsets(
        self,
        offsets_m: Sequence[float],
    ) -> tuple[float, ...]: ...

    def evaluate(
        self,
        offsets_m: Sequence[float],
        reference_offsets_m: Sequence[float],
    ) -> GlobalTrajectoryEvaluation | None: ...


class GlobalTrajectoryOptimizer:
    """Optimize a closed trajectory while delegating all physics to a model."""

    def __init__(self, config: GlobalTrajectoryOptimizerConfig | None = None) -> None:
        self.config = config or GlobalTrajectoryOptimizerConfig()

    def optimize(
        self,
        request: GlobalTrajectoryOptimizationRequest,
        cost_model: GlobalTrajectoryCostModel,
    ) -> GlobalTrajectoryOptimizationResult:
        started = perf_counter()
        initial_offsets = cost_model.prepare_initial_offsets(
            request.initial_offsets_m,
        )
        if len(initial_offsets) != len(request.initial_offsets_m):
            raise ValueError("cost model changed the trajectory sample count")

        reference_offsets = initial_offsets
        best = cost_model.evaluate(initial_offsets, reference_offsets)
        if best is None:
            raise ValueError("cost model rejected the prepared initial trajectory")
        initial_objective = best.objective_cost
        best_objective = best.objective_cost

        average_spacing_m = request.track_length_m / len(initial_offsets)
        window_radius = max(
            3,
            min(
                12,
                round(
                    self.config.transition_radius_m
                    / max(1.0, average_spacing_m)
                ),
            ),
        )
        attempted_candidates = 0
        evaluated_candidates = 0
        accepted_updates = 0

        for step_m in self.config.optimization_steps_m:
            for center_index in range(
                0,
                len(initial_offsets),
                max(1, self.config.center_stride),
            ):
                best_candidate: GlobalTrajectoryEvaluation | None = None
                for direction in (-1.0, 1.0):
                    attempted_candidates += 1
                    candidate_offsets = list(best.offsets_m)
                    for delta_index in range(-window_radius, window_radius + 1):
                        index = (center_index + delta_index) % len(candidate_offsets)
                        weight = 0.5 * (
                            1.0
                            + cos(
                                abs(delta_index)
                                / (window_radius + 1)
                                * pi
                            )
                        )
                        candidate_offsets[index] += direction * step_m * weight

                    prepared = cost_model.prepare_candidate_offsets(
                        candidate_offsets,
                    )
                    candidate = cost_model.evaluate(
                        prepared,
                        reference_offsets,
                    )
                    if candidate is None:
                        continue
                    evaluated_candidates += 1
                    if (
                        candidate.objective_cost + self.config.objective_epsilon
                        < best_objective
                    ):
                        best_candidate = candidate
                        best_objective = candidate.objective_cost

                if best_candidate is not None:
                    best = best_candidate
                    accepted_updates += 1

        duration_ms = (perf_counter() - started) * 1000.0
        return GlobalTrajectoryOptimizationResult(
            evaluation=best,
            diagnostics=GlobalTrajectoryOptimizationDiagnostics(
                attempted_candidates=attempted_candidates,
                evaluated_candidates=evaluated_candidates,
                accepted_updates=accepted_updates,
                initial_objective_cost=initial_objective,
                final_objective_cost=best.objective_cost,
                duration_ms=duration_ms,
            ),
        )
