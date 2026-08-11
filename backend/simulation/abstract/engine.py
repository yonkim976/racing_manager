"""Public abstract result engine boundary for Stages A through D."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence

from .qualifying import QualifyingEngine
from .progress_race import ProgressRaceCursor, ProgressRaceSummary, run_progress_race
from .race import run_abstract_race as _run_abstract_race
from .rng import IndependentRNG
from .state import AbstractQualifyingResult, AbstractRaceFrame, AbstractRaceResult, AbstractSessionSnapshot


class AbstractRaceEngine:
    """Owns an immutable session snapshot and computes abstract results.

    Stage D adds logical lockup, contact and pit-stop outcomes. Safety-car
    behavior and product-mode integration remain later stages.
    """

    def __init__(self, snapshot: AbstractSessionSnapshot):
        self.snapshot = snapshot
        self.rng = IndependentRNG(snapshot.session_seed)

    def run_qualifying(self, *, attempts_per_session: int = 2) -> AbstractQualifyingResult:
        return QualifyingEngine(
            self.snapshot,
            attempts_per_session=attempts_per_session,
            rng=self.rng,
        ).run()

    def qualifying(self, *, attempts_per_session: int = 2) -> AbstractQualifyingResult:
        return self.run_qualifying(attempts_per_session=attempts_per_session)

    def run_race(
        self,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = 0.10,
        max_ticks: int | None = None,
        pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
        frame_sink: Callable[[AbstractRaceFrame], None] | None = None,
    ) -> AbstractRaceResult:
        return _run_abstract_race(
            self.snapshot,
            grid_order=grid_order,
            total_laps=total_laps,
            tick_seconds=tick_seconds,
            max_ticks=max_ticks,
            pit_strategy=pit_strategy,
            frame_sink=frame_sink,
        )

    def race(self, **kwargs) -> AbstractRaceResult:
        return self.run_race(**kwargs)

    def create_progress_cursor(
        self,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = 0.50,
        enable_traffic: bool = True,
        enable_incidents: bool = True,
        enable_pit: bool = True,
        pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
    ) -> ProgressRaceCursor:
        """Create the experimental progress authority without changing v4."""

        return ProgressRaceCursor(
            self.snapshot,
            grid_order=grid_order,
            total_laps=total_laps,
            tick_seconds=tick_seconds,
            enable_traffic=enable_traffic,
            enable_incidents=enable_incidents,
            enable_pit=enable_pit,
            pit_strategy=pit_strategy,
        )

    def run_progress_race(
        self,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = 0.50,
        enable_traffic: bool = True,
        enable_incidents: bool = True,
        enable_pit: bool = True,
        pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
    ) -> ProgressRaceSummary:
        """Run the opt-in progress engine; the product default remains v4."""

        return run_progress_race(
            self.snapshot,
            grid_order=grid_order,
            total_laps=total_laps,
            tick_seconds=tick_seconds,
            enable_traffic=enable_traffic,
            enable_incidents=enable_incidents,
            enable_pit=enable_pit,
            pit_strategy=pit_strategy,
        )


def run_abstract_qualifying(
    snapshot: AbstractSessionSnapshot,
    *,
    attempts_per_session: int = 2,
) -> AbstractQualifyingResult:
    """Convenience function matching the engine method."""

    return AbstractRaceEngine(snapshot).run_qualifying(
        attempts_per_session=attempts_per_session,
    )


def run_abstract_race(
    snapshot: AbstractSessionSnapshot,
    *,
    grid_order: tuple[int | str, ...] | list[int | str] | None = None,
    total_laps: int = 10,
    tick_seconds: float = 0.10,
    max_ticks: int | None = None,
    pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
    frame_sink: Callable[[AbstractRaceFrame], None] | None = None,
) -> AbstractRaceResult:
    return _run_abstract_race(
        snapshot,
        grid_order=grid_order,
        total_laps=total_laps,
        tick_seconds=tick_seconds,
        max_ticks=max_ticks,
        pit_strategy=pit_strategy,
        frame_sink=frame_sink,
    )
