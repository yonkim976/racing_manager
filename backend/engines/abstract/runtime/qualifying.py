"""Result-only Q1/Q2/Q3 qualifying for the abstract engine."""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

from dataclasses import dataclass
from typing import Iterable

from .clock import LogicalClock
from .performance import SegmentRequirement, calculate_segment_time
from .rng import IndependentRNG
from .state import (
    AbstractEntrySnapshot,
    AbstractQualifyingResult,
    AbstractSessionSnapshot,
    DriverQualifyingResult,
    GridEntry,
    LogicalEvent,
    QualifyingRunResult,
    QualifyingSessionSummary,
)


SESSION_PLAN: tuple[tuple[str, int | None], ...] = (
    ("Q1", 15),
    ("Q2", 10),
    ("Q3", None),
)
SESSION_EVOLUTION_BASE = {"Q1": 0.0, "Q2": 0.003, "Q3": 0.006}
SECTOR_IDS = ("S1", "S2", "S3")


def _stable_driver_key(entry: AbstractEntrySnapshot) -> tuple[str, str]:
    return (str(entry.driver_id), entry.vehicle_id)


def _weather_factor(weather: str) -> float:
    return {"dry": 1.0, "light_rain": 1.025, "heavy_rain": 1.085}.get(weather, 1.0)


@dataclass(frozen=True, slots=True)
class _SessionRun:
    entry: AbstractEntrySnapshot
    run: QualifyingRunResult


class QualifyingEngine:
    """Calculate qualifying through explicit runs and sector timing."""

    def __init__(
        self,
        snapshot: AbstractSessionSnapshot,
        attempts_per_session: int = 2,
        rng: IndependentRNG | None = None,
    ):
        if attempts_per_session < 1 or attempts_per_session > 4:
            raise ValueError("attempts_per_session must be in the range 1..4")
        self.snapshot = snapshot
        self.attempts_per_session = attempts_per_session
        self.rng = rng or IndependentRNG(snapshot.session_seed)

    def run(self) -> AbstractQualifyingResult:
        snapshot = self.snapshot
        rng = self.rng
        # The streams are deliberately obtained by name.  No call below uses a
        # shared/global random state, and presentation calls can be added later.
        weather_stream = rng.stream("session:weather")
        track_stream = rng.stream("session:track_evolution")
        weather_noise = weather_stream.uniform(-0.0002, 0.0002)
        weather_multiplier = _weather_factor(snapshot.environment.weather) + weather_noise

        entries = tuple(sorted(snapshot.entries, key=_stable_driver_key))
        # Create the complete Stage A manifest up front.  The unused pit and
        # reliability streams belong to later result layers, but are already
        # independent and available without sharing a global RNG.
        for entry in entries:
            rng.team_pit(entry.team_id)
            rng.vehicle_reliability(entry.vehicle_id)
        by_driver = {entry.driver_id: entry for entry in entries}
        all_runs: dict[int | str, list[QualifyingRunResult]] = {
            entry.driver_id: [] for entry in entries
        }
        session_best: dict[str, dict[int | str, tuple[float, tuple[float, ...], str]]] = {}
        session_order: dict[int | str, list[str]] = {entry.driver_id: [] for entry in entries}
        eliminated_in: dict[int | str, str | None] = {entry.driver_id: None for entry in entries}
        events: list[LogicalEvent] = []
        summaries: list[QualifyingSessionSummary] = []
        pool = list(entries)
        eliminated_blocks: list[list[int | str]] = []
        final_order: list[int | str] = []
        clock = LogicalClock()

        events.append(
            LogicalEvent(
                event_id="qualifying:started",
                event_type="qualifying_started",
                logical_time_s=clock.time_s,
                payload=(
                    ("content_version", snapshot.content_version),
                    ("ruleset_version", snapshot.ruleset_version),
                ),
            )
        )

        for session_name, advance in SESSION_PLAN:
            if not pool:
                break
            session_start_s = clock.time_s
            session_evolution = max(
                0.0,
                SESSION_EVOLUTION_BASE[session_name]
                + track_stream.uniform(-0.00015, 0.00015),
            )
            eligible_ids = [entry.driver_id for entry in pool]
            events.append(
                LogicalEvent(
                    event_id=f"qualifying:{session_name}:started",
                    event_type="qualifying_session_started",
                    logical_time_s=session_start_s,
                    payload=(
                        ("session_name", session_name),
                        ("track_evolution", round(session_evolution, 9)),
                    ),
                )
            )

            session_runs: list[_SessionRun] = []
            for order_index, entry in enumerate(sorted(pool, key=_stable_driver_key)):
                session_order[entry.driver_id].append(session_name)
                for run_number in range(1, self.attempts_per_session + 1):
                    run = self._calculate_run(
                        entry=entry,
                        session_name=session_name,
                        session_start_s=session_start_s,
                        order_index=order_index,
                        run_number=run_number,
                        session_evolution=session_evolution,
                        weather_multiplier=weather_multiplier,
                        rng=rng,
                    )
                    all_runs[entry.driver_id].append(run)
                    session_runs.append(_SessionRun(entry=entry, run=run))
                    self._append_run_events(events, run)

            ranked = sorted(
                session_runs,
                key=lambda item: (item.run.flying_lap_time_s, str(item.entry.driver_id)),
            )
            best_for_session: dict[int | str, tuple[float, tuple[float, ...], str]] = {}
            for item in ranked:
                current = best_for_session.get(item.entry.driver_id)
                candidate = (
                    item.run.flying_lap_time_s,
                    item.run.sector_times_s,
                    item.run.run_id,
                )
                if current is None or candidate[0] < current[0]:
                    best_for_session[item.entry.driver_id] = candidate
            session_best[session_name] = best_for_session

            is_final = advance is None or advance >= len(pool)
            if is_final:
                advancing = [item.entry.driver_id for item in ranked if item.entry.driver_id in best_for_session]
                # ``ranked`` contains one row per run, so reduce to the best
                # session lap before fixing the final order.
                advancing = [
                    driver_id
                    for driver_id, _ in sorted(
                        best_for_session.items(),
                        key=lambda pair: (pair[1][0], str(pair[0])),
                    )
                ]
                final_order = advancing
                for driver_id in final_order:
                    events.append(
                        LogicalEvent(
                            event_id=f"qualifying:{session_name}:classified:{driver_id}",
                            event_type="driver_classified",
                            logical_time_s=session_start_s + 0.1,
                            driver_ids=(driver_id,),
                            payload=(("session_name", session_name),),
                        )
                    )
                advanced_ids: list[int | str] = list(final_order)
                eliminated_ids: list[int | str] = []
            else:
                ranked_drivers = [
                    driver_id
                    for driver_id, _ in sorted(
                        best_for_session.items(),
                        key=lambda pair: (pair[1][0], str(pair[0])),
                    )
                ]
                advanced_ids = ranked_drivers[:advance]
                eliminated_ids = ranked_drivers[advance:]
                eliminated_blocks.append(list(eliminated_ids))
                for driver_id in eliminated_ids:
                    eliminated_in[driver_id] = session_name
                    events.append(
                        LogicalEvent(
                            event_id=f"qualifying:{session_name}:eliminated:{driver_id}",
                            event_type="driver_eliminated",
                            logical_time_s=session_start_s + 0.1,
                            driver_ids=(driver_id,),
                            payload=(("session_name", session_name),),
                        )
                    )
                pool = [by_driver[driver_id] for driver_id in advanced_ids]

            summaries.append(
                QualifyingSessionSummary(
                    session_name=session_name,
                    eligible_driver_ids=tuple(eligible_ids),
                    advanced_driver_ids=tuple(advanced_ids),
                    eliminated_driver_ids=tuple(eliminated_ids),
                    track_evolution=session_evolution,
                )
            )
            session_end = max(
                (run.run.flying_lap_end_s for run in session_runs),
                default=session_start_s,
            )
            events.append(
                LogicalEvent(
                    event_id=f"qualifying:{session_name}:ended",
                    event_type="qualifying_session_ended",
                    logical_time_s=session_end,
                    payload=(
                        ("session_name", session_name),
                        ("advanced_count", len(advanced_ids)),
                    ),
                )
            )
            clock = clock.advance_to(session_end + 20.0)

            if is_final:
                break

        # The Q3 order is the front block, then Q2 eliminations, then Q1
        # eliminations.  Each elimination block remains fastest-to-slowest.
        grid_driver_ids = list(final_order)
        for block in reversed(eliminated_blocks):
            grid_driver_ids.extend(block)

        final_session_name = summaries[-1].session_name if summaries else "Q1"
        pole_time = (
            session_best[final_session_name][final_order[0]][0]
            if final_order
            else 0.0
        )
        driver_results: list[DriverQualifyingResult] = []
        grid: list[GridEntry] = []
        for position, driver_id in enumerate(grid_driver_ids, start=1):
            entered = session_order[driver_id]
            reached = entered[-1]
            best_time, best_sectors, best_run_id = session_best[reached][driver_id]
            entry = by_driver[driver_id]
            driver_results.append(
                DriverQualifyingResult(
                    driver_id=driver_id,
                    vehicle_id=entry.vehicle_id,
                    best_lap_time_s=best_time,
                    best_sector_times_s=best_sectors,
                    best_run_id=best_run_id,
                    runs=tuple(all_runs[driver_id]),
                    sessions_entered=tuple(entered),
                    advanced_to=reached,
                    eliminated_in=eliminated_in[driver_id],
                    grid_position=position,
                )
            )
            grid.append(
                GridEntry(
                    position=position,
                    driver_id=driver_id,
                    vehicle_id=entry.vehicle_id,
                    best_lap_time_s=best_time,
                    gap_s=max(0.0, best_time - pole_time),
                )
            )

        final_time = max((event.logical_time_s for event in events), default=0.0) + 0.1
        events.append(
            LogicalEvent(
                event_id="qualifying:grid:finalized",
                event_type="grid_finalized",
                logical_time_s=final_time,
                payload=(("driver_count", len(grid)),),
            )
        )
        return AbstractQualifyingResult(
            session_id=snapshot.session_id,
            session_seed=snapshot.session_seed,
            content_version=snapshot.content_version,
            ruleset_version=snapshot.ruleset_version,
            abstract_engine_version=snapshot.abstract_engine_version,
            driver_results=tuple(driver_results),
            sessions=tuple(summaries),
            grid=tuple(grid),
            logical_events=tuple(events),
        )

    def _calculate_run(
        self,
        *,
        entry: AbstractEntrySnapshot,
        session_name: str,
        session_start_s: float,
        order_index: int,
        run_number: int,
        session_evolution: float,
        weather_multiplier: float,
        rng: IndependentRNG,
    ) -> QualifyingRunResult:
        run_id = f"{session_name}:{entry.driver_id}:run{run_number}"
        departure_s = session_start_s + 5.0 + order_index * 1.35 + (run_number - 1) * 54.0
        out_lap_time_s = self.snapshot.track.base_lap_time_s * 0.78 + 1.0 + 0.12 * run_number
        flying_start_s = departure_s + out_lap_time_s
        pace_stream = rng.driver_pace(entry.driver_id)
        mistake_stream = rng.driver_mistake(entry.driver_id)
        traffic_active = (order_index + run_number) % 5 == 0
        traffic_penalty_s = 0.035 if traffic_active else 0.0
        sectors: list[float] = []
        mistakes: list[str] = []
        mistake_loss_total = 0.0
        segments_by_sector = {
            sector_id: tuple(
                segment
                for segment in sorted(self.snapshot.track.segments, key=lambda item: item.start_progress)
                if segment.sector_id == sector_id
            )
            for sector_id in SECTOR_IDS
        }
        for sector_index, sector_id in enumerate(SECTOR_IDS):
            sector_segments = segments_by_sector[sector_id]
            if not sector_segments:
                sectors.append(0.0)
                continue
            sector_variance = pace_stream.uniform(-1.0, 1.0) * (
                0.020 + (1.0 - entry.driver.consistency) * 0.14
            )
            sector_risk = 0.018 + (1.0 - entry.driver.consistency) * 0.10
            if any(segment.segment_type in {"heavy_braking", "technical"} for segment in sector_segments):
                sector_risk += 0.006
            mistake_loss = 0.0
            if mistake_stream.random() < sector_risk:
                mistake_loss = mistake_stream.uniform(0.045, 0.220)
                mistake_loss_total += mistake_loss
                mistakes.append(f"{sector_id.lower()}_mistake")
            traffic_share = traffic_penalty_s * (0.42, 0.34, 0.24)[sector_index]
            base_sum = sum(segment.base_time_s for segment in sector_segments)
            sector_sum = 0.0
            for segment in sector_segments:
                fraction = segment.base_time_s / base_sum
                breakdown = calculate_segment_time(
                    segment,
                    entry.vehicle.performance,
                    entry.driver,
                    entry.tire,
                    track_evolution=session_evolution,
                    traffic_penalty_s=traffic_share * fraction,
                    variance_s=sector_variance * fraction,
                    mistake_loss_s=mistake_loss * fraction,
                )
                sector_sum += breakdown.time_s
            # Weather is an environment condition, not a vehicle rating.  A
            # dry session stays at the neutral multiplier, while a future wet
            # snapshot can slow the same sector result without changing axes.
            sectors.append(sector_sum * weather_multiplier)

        flying_lap_time_s = sum(sectors)
        run = QualifyingRunResult(
            run_id=run_id,
            session_name=session_name,
            driver_id=entry.driver_id,
            vehicle_id=entry.vehicle_id,
            run_number=run_number,
            run_departure_s=departure_s,
            out_lap_time_s=out_lap_time_s,
            flying_lap_start_s=flying_start_s,
            sector_times_s=tuple(sectors),
            flying_lap_time_s=flying_lap_time_s,
            flying_lap_end_s=flying_start_s + flying_lap_time_s,
            traffic_penalty_s=traffic_penalty_s,
            track_evolution=session_evolution,
            mistake_loss_s=mistake_loss_total,
            mistakes=tuple(mistakes),
            tire=entry.tire,
        )
        return run

    @staticmethod
    def _append_run_events(events: list[LogicalEvent], run: QualifyingRunResult) -> None:
        events.append(
            LogicalEvent(
                event_id=f"{run.run_id}:departed",
                event_type="run_departed",
                logical_time_s=run.run_departure_s,
                driver_ids=(run.driver_id,),
                payload=(("session_name", run.session_name), ("run_number", run.run_number)),
            )
        )
        events.append(
            LogicalEvent(
                event_id=f"{run.run_id}:outlap_complete",
                event_type="out_lap_completed",
                logical_time_s=run.flying_lap_start_s,
                driver_ids=(run.driver_id,),
                payload=(("out_lap_time_s", round(run.out_lap_time_s, 9)),),
            )
        )
        cumulative = run.flying_lap_start_s
        for sector_index, sector_time in enumerate(run.sector_times_s, start=1):
            cumulative += sector_time
            events.append(
                LogicalEvent(
                    event_id=f"{run.run_id}:S{sector_index}",
                    event_type="flying_sector_completed",
                    logical_time_s=cumulative,
                    driver_ids=(run.driver_id,),
                    payload=(
                        ("sector", f"S{sector_index}"),
                        ("sector_time_s", round(sector_time, 9)),
                    ),
                )
            )
        for mistake in run.mistakes:
            events.append(
                LogicalEvent(
                    event_id=f"{run.run_id}:mistake:{mistake}",
                    event_type="qualifying_mistake",
                    logical_time_s=run.flying_lap_end_s,
                    driver_ids=(run.driver_id,),
                    payload=(("mistake", mistake), ("loss_s", round(run.mistake_loss_s, 9))),
                )
            )
        events.append(
            LogicalEvent(
                event_id=f"{run.run_id}:completed",
                event_type="flying_lap_completed",
                logical_time_s=run.flying_lap_end_s,
                driver_ids=(run.driver_id,),
                payload=(("lap_time_s", round(run.flying_lap_time_s, 9)),),
            )
        )


def run_abstract_qualifying(
    snapshot: AbstractSessionSnapshot,
    *,
    attempts_per_session: int = 2,
) -> AbstractQualifyingResult:
    return QualifyingEngine(snapshot, attempts_per_session=attempts_per_session).run()
