"""FULL timing loops, mini-sectors, gaps and lap history operations.

RaceEngine inherits TimingOpsMixin so existing call sites and tests keep the
same method names.  Behavior is unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import floor

from models.schemas import (
    DriverRaceHistoryInfo,
    DriverRaceState,
    DryTireRole,
    LapTimeInfo,
    PhysicalTireCompound,
    RaceEvent,
    RaceHistoryState,
    Team,
    TireCompound,
)
from .state_contract import TickPhase
from .tire_model import physical_compound_for, physical_compound_for_state

PROGRESS_EPSILON = 1e-9
TIMING_CROSSING_LAPS_TO_RETAIN = 4


@dataclass(frozen=True)
class TimingLoop:
    """One physical timing line inside a major circuit sector."""

    index: int
    progress: float
    sector_index: int
    mini_sector_index: int


class TimingOpsMixin:
    """Official timing loops, GAP/INT and lap history payloads."""

    def _init_timing_state(self) -> None:
        """Initialize timing loops, lap history and mini-sector bests."""
        self._timing_loops = self._build_timing_loops()
        self._lap_history: dict[int, list[LapTimeInfo]] = {}
        self._timing_crossings: dict[int, dict[tuple[int, int], float]] = {}
        self._timing_gap_valid: dict[int, bool] = {}
        self._interval_timing_gap_valid: dict[int, bool] = {}
        self._live_interval_seconds: dict[int, float | None] = {}
        self._last_sector_time: dict[int, float] = {}
        self._last_mini_sector_time: dict[int, float] = {}
        self._last_completed_mini_sector_index: dict[int, int | None] = {}
        self._session_best_mini_sector_times: list[float | None] = [
            None for _ in self._timing_loops
        ]
        self._personal_best_mini_sector_times: dict[
            int,
            list[float | None],
        ] = {}

    def _sector_ranges(self) -> tuple[tuple[float, float, int], ...]:
        """Return three ordered major sectors and their mini-sector counts."""
        sectors = self.circuit.sectors[:3]
        if (
            len(sectors) == 3
            and all(item.start is not None and item.end is not None for item in sectors)
        ):
            ranges = tuple(
                (
                    float(item.start),
                    float(item.end),
                    item.mini_sector_count,
                )
                for item in sectors
            )
            contiguous = (
                abs(ranges[0][0]) <= 1e-9
                and abs(ranges[-1][1] - 1.0) <= 1e-9
                and all(
                    abs(left[1] - right[0]) <= 1e-7
                    for left, right in zip(ranges, ranges[1:])
                )
            )
            if contiguous:
                return ranges
        return tuple(
            (
                index / 3.0,
                (index + 1) / 3.0,
                sectors[index].mini_sector_count if index < len(sectors) else 6,
            )
            for index in range(3)
        )

    def _build_timing_loops(self) -> tuple[TimingLoop, ...]:
        loops: list[TimingLoop] = []
        for sector_index, (start, end, mini_count) in enumerate(
            self._sector_ranges(),
            start=1,
        ):
            sector_length = end - start
            for mini_index in range(1, mini_count + 1):
                loops.append(
                    TimingLoop(
                        index=len(loops),
                        progress=start + sector_length * (mini_index - 1) / mini_count,
                        sector_index=sector_index,
                        mini_sector_index=mini_index,
                    )
                )
        return tuple(loops)

    def _timing_location(self, progress: float) -> tuple[int, int, int]:
        normalized = progress % 1.0
        current = self._timing_loops[0]
        for loop in self._timing_loops:
            if loop.progress > normalized + 1e-9:
                break
            current = loop
        return (
            current.sector_index,
            current.mini_sector_index,
            current.index + 1,
        )

    def _previous_timing_key(
        self,
        lap_index: int,
        loop_index: int,
    ) -> tuple[int, int]:
        if loop_index > 0:
            return lap_index, loop_index - 1
        return lap_index - 1, len(self._timing_loops) - 1

    def _previous_sector_start_key(
        self,
        lap_index: int,
        loop: TimingLoop,
    ) -> tuple[int, int] | None:
        sector_starts = [
            item for item in self._timing_loops if item.mini_sector_index == 1
        ]
        if loop.mini_sector_index != 1 or not sector_starts:
            return None
        position = next(
            index for index, item in enumerate(sector_starts) if item.index == loop.index
        )
        if position > 0:
            return lap_index, sector_starts[position - 1].index
        return lap_index - 1, sector_starts[-1].index

    def _record_timing_loop_crossings(
        self,
        state: DriverRaceState,
        previous_total_progress: float,
        current_total_progress: float,
        step_start_time: float,
        delta_seconds: float,
    ) -> None:
        """Record interpolated transponder times at every crossed timing line."""
        if current_total_progress <= previous_total_progress + 1e-12:
            return
        crossings = self._timing_crossings.setdefault(state.driver_id, {})
        span = current_total_progress - previous_total_progress
        for loop in self._timing_loops:
            lap_index = floor(previous_total_progress - loop.progress) + 1
            checkpoint = lap_index + loop.progress
            while checkpoint <= current_total_progress + 1e-12:
                ratio = min(
                    1.0,
                    max(0.0, (checkpoint - previous_total_progress) / span),
                )
                crossing_time = step_start_time + ratio * delta_seconds
                key = (lap_index, loop.index)
                crossings[key] = crossing_time

                previous_key = self._previous_timing_key(lap_index, loop.index)
                previous_time = crossings.get(previous_key)
                if previous_time is not None:
                    mini_sector_time = max(
                        0.0,
                        crossing_time - previous_time,
                    )
                    self._last_mini_sector_time[state.driver_id] = mini_sector_time
                    completed_index = previous_key[1]
                    self._last_completed_mini_sector_index[
                        state.driver_id
                    ] = completed_index
                    personal_bests = self._personal_best_mini_sector_times[
                        state.driver_id
                    ]
                    personal_best = personal_bests[completed_index]
                    if personal_best is None or mini_sector_time < personal_best:
                        personal_bests[completed_index] = mini_sector_time
                    session_best = self._session_best_mini_sector_times[
                        completed_index
                    ]
                    if session_best is None or mini_sector_time < session_best:
                        self._session_best_mini_sector_times[
                            completed_index
                        ] = mini_sector_time
                previous_sector_key = self._previous_sector_start_key(
                    lap_index,
                    loop,
                )
                if previous_sector_key is not None:
                    previous_sector_time = crossings.get(previous_sector_key)
                    if previous_sector_time is not None:
                        self._last_sector_time[state.driver_id] = max(
                            0.0,
                            crossing_time - previous_sector_time,
                        )
                lap_index += 1
                checkpoint = lap_index + loop.progress

    def _timing_gap_seconds_between(
        self,
        ahead: DriverRaceState,
        follower: DriverRaceState,
    ) -> float | None:
        """Return measured time separation at the latest common timing loop."""
        ahead_crossings = self._timing_crossings.get(ahead.driver_id, {})
        follower_crossings = self._timing_crossings.get(follower.driver_id, {})
        common = ahead_crossings.keys() & follower_crossings.keys()
        if not common:
            return None
        latest_key = max(
            common,
            key=lambda key: key[0] + self._timing_loops[key[1]].progress,
        )
        gap_seconds = follower_crossings[latest_key] - ahead_crossings[latest_key]
        if gap_seconds > 0.0005:
            return gap_seconds
        # A non-positive historical split can occur after an on-track position
        # change.  Clamping it to zero made whole timing columns show 0.000;
        # fall back to the live spatial estimate until the new order crosses a
        # common timing line in that order.
        return None

    def _spatial_live_gap_seconds_between(
        self,
        ahead: DriverRaceState,
        follower: DriverRaceState,
    ) -> float | None:
        """Estimate a continuously changing time gap from current track state."""
        progress_gap = abs(ahead.total_progress - follower.total_progress)
        if progress_gap <= PROGRESS_EPSILON:
            return None
        distance_m = progress_gap * self.track_length_m
        lap_estimate_seconds = self._progress_gap_to_seconds(
            progress_gap,
            follower,
        )
        average_speed_mps = 0.5 * (
            max(0.0, ahead.speed_kph / 3.6)
            + max(0.0, follower.speed_kph / 3.6)
        )
        if average_speed_mps < 8.0:
            return lap_estimate_seconds
        speed_estimate_seconds = distance_m / average_speed_mps
        if distance_m <= 250.0:
            speed_weight = 0.72
        elif distance_m <= 750.0:
            speed_weight = 0.55
        else:
            speed_weight = 0.30
        blended = (
            speed_estimate_seconds * speed_weight
            + lap_estimate_seconds * (1.0 - speed_weight)
        )
        return min(
            lap_estimate_seconds * 2.5,
            max(lap_estimate_seconds * 0.35, blended),
        )

    def _live_timing_gap_seconds_between(
        self,
        ahead: DriverRaceState,
        follower: DriverRaceState,
    ) -> tuple[float | None, bool]:
        """Blend an official loop anchor with live between-loop movement."""
        measured = self._timing_gap_seconds_between(ahead, follower)
        spatial = self._spatial_live_gap_seconds_between(ahead, follower)
        if measured is None:
            return spatial, False
        if spatial is None:
            return measured, True
        distance_m = max(
            0.0,
            (ahead.total_progress - follower.total_progress) * self.track_length_m,
        )
        live_weight = 0.55 if distance_m <= 750.0 else 0.35
        return (
            measured * (1.0 - live_weight) + spatial * live_weight,
            True,
        )

    def _current_mini_sector_splits(
        self,
        driver_id: int,
        lap_index: int,
    ) -> list[float | None]:
        """Return completed timing-loop segments for the driver's live lap."""
        crossings = self._timing_crossings.get(driver_id, {})
        splits: list[float | None] = []
        for loop in self._timing_loops:
            start_key = (lap_index, loop.index)
            if loop.index + 1 < len(self._timing_loops):
                end_key = (lap_index, loop.index + 1)
            else:
                end_key = (lap_index + 1, 0)
            start_time = crossings.get(start_key)
            end_time = crossings.get(end_key)
            splits.append(
                None
                if start_time is None or end_time is None
                else max(0.0, end_time - start_time)
            )
        return splits

    def _mini_sector_statuses(
        self,
        driver_id: int,
        splits: list[float | None],
    ) -> list[str]:
        """Map live mini sectors to the official timing colour convention."""
        personal_bests = self._personal_best_mini_sector_times[driver_id]
        statuses: list[str] = []
        for index, split in enumerate(splits):
            if split is None:
                statuses.append("pending")
                continue
            session_best = self._session_best_mini_sector_times[index]
            personal_best = personal_bests[index]
            if session_best is not None and split <= session_best + 0.0005:
                statuses.append("overall_best")
            elif personal_best is not None and split <= personal_best + 0.0005:
                statuses.append("personal_best")
            else:
                statuses.append("slower")
        return statuses

    def _lap_split_times(
        self,
        driver_id: int,
        lap: int,
    ) -> tuple[list[float], list[float]]:
        crossings = self._timing_crossings.get(driver_id, {})
        lap_index = lap - 1
        sector_starts = [
            item for item in self._timing_loops if item.mini_sector_index == 1
        ]
        sector_times: list[float] = []
        for index, start in enumerate(sector_starts):
            start_key = (lap_index, start.index)
            if index + 1 < len(sector_starts):
                end_key = (lap_index, sector_starts[index + 1].index)
            else:
                end_key = (lap_index + 1, sector_starts[0].index)
            if start_key not in crossings or end_key not in crossings:
                return [], []
            sector_times.append(crossings[end_key] - crossings[start_key])

        ordered_keys = [
            (lap_index, loop.index) for loop in self._timing_loops
        ] + [(lap_index + 1, self._timing_loops[0].index)]
        if any(key not in crossings for key in ordered_keys):
            return sector_times, []
        mini_times = [
            crossings[right] - crossings[left]
            for left, right in zip(ordered_keys, ordered_keys[1:])
        ]
        return sector_times, mini_times

    def _update_gaps(self) -> None:
        self._require_rules_phase("update race gaps")
        running = [s for s in self.driver_states.values() if not s.retired]
        if not running:
            return

        leader = next((s for s in running if s.position == 1), None)
        if leader is None:
            return

        for state in running:
            if state.driver_id == leader.driver_id:
                state.gap_to_leader = 0.0
                self._timing_gap_valid[state.driver_id] = True
            else:
                live_gap, anchored = self._live_timing_gap_seconds_between(
                    leader,
                    state,
                )
                self._timing_gap_valid[state.driver_id] = anchored
                state.gap_to_leader = max(0.0, live_gap or 0.0)

        running_by_position = {state.position: state for state in running}
        for state in running:
            if state.position <= 1:
                self._live_interval_seconds[state.driver_id] = None
                self._interval_timing_gap_valid[state.driver_id] = True
                continue
            ahead = running_by_position.get(state.position - 1)
            if ahead is None:
                self._live_interval_seconds[state.driver_id] = None
                self._interval_timing_gap_valid[state.driver_id] = False
                continue
            interval_seconds, anchored = self._live_timing_gap_seconds_between(
                ahead,
                state,
            )
            self._live_interval_seconds[state.driver_id] = interval_seconds
            self._interval_timing_gap_valid[state.driver_id] = anchored

    def _format_gap(self, seconds: float) -> str:
        return f"+{max(0.0, seconds):.3f}"

    def _format_interval(self, seconds: float | None) -> str:
        if seconds is None or seconds <= 0.001:
            return "—"
        return f"+{seconds:.3f}"

    def _progress_gap_to_seconds(self, progress_gap: float, state: DriverRaceState) -> float:
        """Convert a lap-fraction gap into an approximate live timing gap."""
        if progress_gap <= 0:
            return 0.0
        lap_time = self._base_lap_time_for_state(state)
        lap_time *= self._phase_lap_time_factor()
        return progress_gap * lap_time

    def _record_lap_time(
        self,
        driver_id: int,
        lap: int,
        lap_time: float,
        tire_compound: TireCompound,
        stint: int,
        *,
        pit_stop: bool = False,
        tire_role: DryTireRole | None = None,
        physical_tire_compound: PhysicalTireCompound | None = None,
    ) -> None:
        sector_times, mini_sector_times = self._lap_split_times(driver_id, lap)
        resolved_role = tire_role
        if resolved_role is None:
            try:
                resolved_role = DryTireRole(tire_compound.value)
            except ValueError:
                resolved_role = None
        self._lap_history.setdefault(driver_id, []).append(
            LapTimeInfo(
                lap=lap,
                lap_time=round(lap_time, 3),
                tire_compound=tire_compound.value,
                stint=stint,
                tire_role=resolved_role.value if resolved_role is not None else None,
                physical_tire_compound=(
                    physical_tire_compound
                    or physical_compound_for(tire_compound)
                ).value,
                pit_stop=pit_stop,
                sector_times=[round(value, 3) for value in sector_times],
                mini_sector_times=[
                    round(value, 3) for value in mini_sector_times
                ],
            )
        )
        self._prune_timing_crossings(driver_id, completed_lap=lap)

    def _prune_timing_crossings(
        self,
        driver_id: int,
        *,
        completed_lap: int,
    ) -> None:
        """Keep raw timing anchors bounded after their lap summary is stored."""
        crossings = self._timing_crossings.get(driver_id)
        if not crossings:
            return
        minimum_lap_index = completed_lap - TIMING_CROSSING_LAPS_TO_RETAIN
        if minimum_lap_index <= 0:
            return
        stale_keys = [
            key
            for key in crossings
            if key[0] < minimum_lap_index
        ]
        for key in stale_keys:
            del crossings[key]

    def _complete_lap(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        team: Team,
        lap_finish_time: float,
    ) -> list[RaceEvent]:
        self._require_tick_phase(TickPhase.RULES, "lap completion")
        events: list[RaceEvent] = []
        state.current_lap += 1
        state.total_progress = state.current_lap + state.progress

        lap_start = getattr(state, "_lap_start_time", 0.0)
        state.last_lap_time = max(lap_finish_time - lap_start, self.circuit.base_lap_time * 0.85)
        state._lap_start_time = lap_finish_time  # type: ignore[attr-defined]

        if state.best_lap_time <= 0 or state.last_lap_time < state.best_lap_time:
            state.best_lap_time = state.last_lap_time
        self._record_lap_time(
            driver_id,
            state.current_lap,
            state.last_lap_time,
            state.tire_compound,
            state.pit_count + 1,
            tire_role=state.tire_role,
            physical_tire_compound=physical_compound_for_state(state),
        )

        if state.pit_request is not None:
            tire = state.pit_request
            state.pit_request = None
            if self._has_pit_progress_anchors():
                state.tire_age += 1
                state.tire_wear = self._current_tire_wear(state)
                state.pit_request = tire
            else:
                self._start_pit_stop(driver_id, state, meta, team, tire, events)
        else:
            state.tire_age += 1
            state.tire_wear = self._current_tire_wear(state)

        if state.current_lap >= self.total_laps:
            state.finished = True
            state.progress = 1.0
            state.total_progress = self.total_laps
            if driver_id not in self._finish_order:
                self._finish_order.append(driver_id)
        elif not state.in_pit:
            self._refresh_lap_variation(driver_id)

        return events

    def build_timing_payload(self) -> dict:
        """Build the one-Hz mini-sector channel without a full tick model."""
        positions: list[dict] = []
        for state in self.driver_states.values():
            timing_lap_index = max(
                0,
                state.current_lap - (1 if state.finished else 0),
            )
            splits = self._current_mini_sector_splits(
                state.driver_id,
                timing_lap_index,
            )
            last_completed_index = self._last_completed_mini_sector_index.get(
                state.driver_id,
            )
            last_delta_to_best: float | None = None
            if last_completed_index is not None:
                session_best = self._session_best_mini_sector_times[
                    last_completed_index
                ]
                if session_best is not None:
                    last_delta_to_best = max(
                        0.0,
                        self._last_mini_sector_time.get(state.driver_id, 0.0)
                        - session_best,
                    )
            positions.append(
                {
                    "driver_id": state.driver_id,
                    "last_mini_sector_time": round(
                        self._last_mini_sector_time.get(state.driver_id, 0.0),
                        3,
                    ),
                    "last_mini_sector_delta_to_best": (
                        round(last_delta_to_best, 3)
                        if last_delta_to_best is not None
                        else None
                    ),
                    "mini_sector_splits": [
                        round(split, 3) if split is not None else None
                        for split in splits
                    ],
                    "mini_sector_statuses": self._mini_sector_statuses(
                        state.driver_id,
                        splits,
                    ),
                }
            )
        return {"type": "race_timing", "positions": positions}

    def build_history_state(
        self,
        since_by_driver: dict[int, int] | None = None,
    ) -> RaceHistoryState:
        """Build a full reconnect snapshot or an incremental lap update."""
        full_snapshot = since_by_driver is None
        histories: list[DriverRaceHistoryInfo] = []
        previous_lengths = since_by_driver or {}
        for state in self.driver_states.values():
            lap_history = self._lap_history.get(state.driver_id, [])
            if full_snapshot:
                start_index = 0
            else:
                previous_length = previous_lengths.get(state.driver_id, 0)
                start_index = (
                    previous_length
                    if 0 <= previous_length <= len(lap_history)
                    else 0
                )
                if start_index == len(lap_history):
                    continue
            histories.append(
                DriverRaceHistoryInfo(
                    driver_id=state.driver_id,
                    start_index=start_index,
                    lap_history=lap_history[start_index:],
                )
            )
        return RaceHistoryState(
            full_snapshot=full_snapshot,
            histories=histories,
        )
