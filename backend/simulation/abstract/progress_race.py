"""Experimental progress-authoritative race kernel.

This module is intentionally independent from the Stage 4 kinematic traffic
cursor.  It advances each car through deterministic timing segments and owns
only logical race distance, ordering and gaps.  World pose, lateral motion,
acceleration, collision geometry and presentation buffers are not result
authority here.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from math import ceil, exp, floor, isfinite
from typing import Any, Mapping, Sequence

from simulation.start_grid_geometry import (
    GRID_POLE_DISTANCE_BEHIND_LINE_M,
    GRID_SLOT_SPACING_M,
)

from .kinematics import GRID_HOLD_DURATION_S
from .pit import PitStopPlan
from .performance import SegmentRequirement, calculate_segment_time
from .rng import IndependentRNG
from .state import (
    AbstractEntrySnapshot,
    AbstractSessionSnapshot,
    LogicalEvent,
    TireConditionSnapshot,
    canonical_hash,
)


PROGRESS_RACE_ENGINE_VERSION = "abstract-progress-race-v5"
PROGRESS_RACE_RESULT_CONTRACT = "abstract-progress-summary-v5"
PROGRESS_TIMING_AUTHORITY_VERSION = "abstract-progress-timing-v1"
PROGRESS_RACECRAFT_AUTHORITY_VERSION = "abstract-progress-racecraft-v2"
DEFAULT_PROGRESS_TICK_SECONDS = 0.50
PROGRESS_DECISION_INTERVAL_S = 0.50
PROGRESS_MOTION_INTERVAL_S = 0.05
FOLLOW_WINDOW_M = 30.0
ATTACK_WINDOW_MIN_M = 6.0
ATTACK_WINDOW_MAX_M = 18.0
MIN_LOGICAL_GAP_M = 6.0
ATTACKS_ENABLED_AFTER_START_S = 10.0
ATTACK_SEGMENT_TYPES = {"straight", "heavy_braking", "traction"}
DRS_ELIGIBILITY_GAP_S = 1.0
DRS_INITIAL_ENABLE_AFTER_LEADER_LAPS = 1.0
DRS_SC_RESTART_DISABLED_LAPS = 1.0
ABSTRACT_WAKE_MAX_GAP_M = 120.0
ABSTRACT_TOW_PACE_GAIN_MAX = 0.010
ABSTRACT_DIRTY_AIR_PACE_LOSS_MAX = 0.014
ABSTRACT_DRS_PACE_GAIN = 0.020
DRS_TRAIN_MAX_INTERVAL_S = 1.20
MANEUVER_GROUP_MAX_GAP_M = 12.0
MANEUVER_GROUP_MIN_TRACK_WIDTH_M = 9.0
MANEUVER_GROUP_MAX_SIZE = 3
PROGRESS_PIT_ROUTE_LAP_FRACTION = 0.12
PROGRESS_PIT_PHASE_ROUTE_RANGES = {
    "entry": (0.0, 0.12),
    "lane": (0.12, 0.50),
    "stop": (0.50, 0.50),
    "exit": (0.50, 1.0),
}
VSC_DURATION_S = 15.0
SC_DEPLOY_DURATION_S = 4.0
SC_QUEUE_TARGET_GAP_M = 12.0
SC_QUEUE_FORMED_TOLERANCE_M = 35.0
SC_QUEUE_HOLD_DURATION_S = 8.0
SC_RESTART_READY_DURATION_S = 5.0
SC_IN_THIS_LAP_DURATION_S = 5.0
RESTART_ATTACK_COOLDOWN_S = 3.0
VSC_CONTACT_SEVERITY_THRESHOLD = 0.65
SC_CONTACT_SEVERITY_THRESHOLD = 0.92
VSC_SPIN_SEVERITY_THRESHOLD = 0.75
VSC_RUN_WIDE_SEVERITY_THRESHOLD = 0.95
TIMING_CROSSING_LAPS_TO_RETAIN = 4


@dataclass(frozen=True, slots=True)
class _ProgressTimingLoop:
    """One model timing line used for authoritative GAP/INT anchors."""

    index: int
    progress: float
    sector_index: int
    mini_sector_index: int


@dataclass(frozen=True, slots=True)
class ProgressVehicleState:
    """Minimal logical vehicle state without render or force fields."""

    driver_id: int | str
    vehicle_id: str
    position: int
    race_distance_m: float
    total_progress: float
    lap_number: int
    segment_id: str
    segment_fraction: float
    logical_speed_mps: float
    gap_to_ahead_m: float
    gap_to_leader_s: float
    interval_to_ahead_s: float
    timing_gap_valid: bool
    interval_timing_gap_valid: bool
    ahead_driver_id: int | str | None
    traffic_state: str
    target_driver_id: int | str | None
    drs_active: bool
    dirty_air_active: bool
    dirty_air_strength: float
    tow_strength: float
    attack_mode: str
    maneuver_side: int
    maneuver_progress: float
    drs_train_id: str | None
    drs_train_size: int
    drs_train_position: int | None
    drs_train_member_ids: tuple[int | str, ...]
    maneuver_group_id: str | None
    maneuver_group_size: int
    maneuver_group_member_ids: tuple[int | str, ...]
    maneuver_group_phase: str | None
    maneuver_group_corridor_index: int | None
    incident_state: str | None
    damage_level: float
    physical_compound: str
    tire_role: str
    stint_lap: int
    wear_laps: float
    temperature_band: str
    pit_state: str
    pit_lane_progress: float
    pit_stop_count: int
    pit_request_pending: bool
    pit_request_role: str | None
    pit_request_compound: str | None
    pace_mode: str
    race_status: str
    retirement_reason: str | None
    retirement_time_s: float | None
    finished: bool
    finish_time_s: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver_id": self.driver_id,
            "vehicle_id": self.vehicle_id,
            "position": self.position,
            "race_distance_m": round(self.race_distance_m, 9),
            "total_progress": round(self.total_progress, 9),
            "lap_number": self.lap_number,
            "segment_id": self.segment_id,
            "segment_fraction": round(self.segment_fraction, 9),
            "logical_speed_mps": round(self.logical_speed_mps, 9),
            "gap_to_ahead_m": round(self.gap_to_ahead_m, 9),
            "gap_to_leader_s": round(self.gap_to_leader_s, 9),
            "interval_to_ahead_s": round(self.interval_to_ahead_s, 9),
            "timing_gap_valid": self.timing_gap_valid,
            "interval_timing_gap_valid": self.interval_timing_gap_valid,
            "ahead_driver_id": self.ahead_driver_id,
            "traffic_state": self.traffic_state,
            "target_driver_id": self.target_driver_id,
            "drs_active": self.drs_active,
            "dirty_air_active": self.dirty_air_active,
            "dirty_air_strength": round(self.dirty_air_strength, 9),
            "tow_strength": round(self.tow_strength, 9),
            "attack_mode": self.attack_mode,
            "maneuver_side": self.maneuver_side,
            "maneuver_progress": round(self.maneuver_progress, 9),
            "drs_train_id": self.drs_train_id,
            "drs_train_size": self.drs_train_size,
            "drs_train_position": self.drs_train_position,
            "drs_train_member_ids": list(self.drs_train_member_ids),
            "maneuver_group_id": self.maneuver_group_id,
            "maneuver_group_size": self.maneuver_group_size,
            "maneuver_group_member_ids": list(self.maneuver_group_member_ids),
            "maneuver_group_phase": self.maneuver_group_phase,
            "maneuver_group_corridor_index": self.maneuver_group_corridor_index,
            "incident_state": self.incident_state,
            "damage_level": round(self.damage_level, 9),
            "physical_compound": self.physical_compound,
            "tire_role": self.tire_role,
            "stint_lap": self.stint_lap,
            "wear_laps": round(self.wear_laps, 9),
            "temperature_band": self.temperature_band,
            "pit_state": self.pit_state,
            "pit_lane_progress": round(self.pit_lane_progress, 9),
            "pit_stop_count": self.pit_stop_count,
            "pit_request_pending": self.pit_request_pending,
            "pit_request_role": self.pit_request_role,
            "pit_request_compound": self.pit_request_compound,
            "pace_mode": self.pace_mode,
            "race_status": self.race_status,
            "retirement_reason": self.retirement_reason,
            "retirement_time_s": (
                None
                if self.retirement_time_s is None
                else round(self.retirement_time_s, 9)
            ),
            "finished": self.finished,
            "finish_time_s": (
                None if self.finish_time_s is None else round(self.finish_time_s, 9)
            ),
        }


@dataclass(frozen=True, slots=True)
class ProgressRaceTick:
    tick_index: int
    logical_time_s: float
    vehicles: tuple[ProgressVehicleState, ...]
    race_control_state: str = "green"
    race_control_phase: str = "green"

    def __post_init__(self) -> None:
        positions = tuple(vehicle.position for vehicle in self.vehicles)
        if positions != tuple(range(1, len(self.vehicles) + 1)):
            raise ValueError("progress tick positions must be contiguous")
        driver_ids = tuple(vehicle.driver_id for vehicle in self.vehicles)
        if len(driver_ids) != len(set(driver_ids)):
            raise ValueError("progress tick contains duplicate drivers")

    def to_dict(self) -> dict[str, Any]:
        return {
            "tick_index": self.tick_index,
            "logical_time_s": round(self.logical_time_s, 9),
            "race_control_state": self.race_control_state,
            "race_control_phase": self.race_control_phase,
            "vehicles": [vehicle.to_dict() for vehicle in self.vehicles],
        }


@dataclass(frozen=True, slots=True)
class ProgressFinishResult:
    position: int
    driver_id: int | str
    status: str
    race_distance_m: float
    finish_time_s: float | None = None
    retirement_time_s: float | None = None
    retirement_reason: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position,
            "driver_id": self.driver_id,
            "status": self.status,
            "race_distance_m": round(self.race_distance_m, 9),
            "finish_time_s": (
                None if self.finish_time_s is None else round(self.finish_time_s, 9)
            ),
            "retirement_time_s": (
                None
                if self.retirement_time_s is None
                else round(self.retirement_time_s, 9)
            ),
            "retirement_reason": self.retirement_reason,
        }


@dataclass(frozen=True, slots=True)
class ProgressRaceSummary:
    """Tick-size-invariant authority produced by the progress kernel.

    ``integration_tick_count`` is diagnostic only and is deliberately excluded
    from the canonical hash so 0.05/0.10/0.50-second schedules can represent
    the same logical race result.
    """

    snapshot_hash: str
    total_laps: int
    grid_order: tuple[int | str, ...]
    classification: tuple[ProgressFinishResult, ...]
    logical_duration_s: float
    integration_tick_count: int
    logical_events: tuple[LogicalEvent, ...] = ()
    metrics: tuple[tuple[str, int], ...] = ()

    def __post_init__(self) -> None:
        if tuple(item.position for item in self.classification) != tuple(
            range(1, len(self.classification) + 1)
        ):
            raise ValueError("progress classification positions must be contiguous")
        finish_ids = tuple(item.driver_id for item in self.classification)
        if set(finish_ids) != set(self.grid_order) or len(finish_ids) != len(self.grid_order):
            raise ValueError("progress classification must contain the complete grid")
        if any(item.status not in {"finished", "retired"} for item in self.classification):
            raise ValueError("progress classification status must be finished or retired")
        seen_retirement = False
        for item in self.classification:
            if item.status == "retired":
                seen_retirement = True
            elif seen_retirement:
                raise ValueError("finishers must be classified before retired drivers")
        object.__setattr__(
            self,
            "logical_events",
            tuple(
                sorted(
                    self.logical_events,
                    key=lambda event: (round(event.logical_time_s, 9), event.event_id),
                )
            ),
        )
        object.__setattr__(self, "metrics", tuple(sorted(self.metrics)))

    @property
    def finish_order(self) -> tuple[int | str, ...]:
        return tuple(item.driver_id for item in self.classification)

    @property
    def retired_driver_ids(self) -> tuple[int | str, ...]:
        return tuple(
            item.driver_id for item in self.classification if item.status == "retired"
        )

    @property
    def logical_tick_count(self) -> int:
        """Compatibility value for bounded broadcast consumers."""

        return self.integration_tick_count

    @property
    def timing_checkpoints(self) -> tuple[Any, ...]:
        """Progress broadcast checkpoints are presentation-only and not hashed."""

        return ()

    def canonical_payload(self) -> dict[str, Any]:
        return {
            "result_contract": PROGRESS_RACE_RESULT_CONTRACT,
            "engine_version": PROGRESS_RACE_ENGINE_VERSION,
            "snapshot_hash": self.snapshot_hash,
            "total_laps": self.total_laps,
            "grid_order": list(self.grid_order),
            "classification": [item.to_dict() for item in self.classification],
            "logical_duration_s": round(self.logical_duration_s, 9),
            "logical_events": [event.to_dict() for event in self.logical_events],
            "metrics": {key: value for key, value in self.metrics},
        }

    @property
    def canonical_result_hash(self) -> str:
        return canonical_hash(self.canonical_payload())

    def to_dict(self) -> dict[str, Any]:
        return {
            **self.canonical_payload(),
            "integration_tick_count": self.integration_tick_count,
            "canonical_result_hash": self.canonical_result_hash,
        }


@dataclass(slots=True)
class _ProgressCar:
    entry: AbstractEntrySnapshot
    grid_index: int
    race_distance_m: float
    segment_index: int
    segment_elapsed_s: float
    segment_durations_s: tuple[float, ...]
    finish_time_s: float | None = None
    traffic_state: str = "clear"
    target_driver_id: int | str | None = None
    state_started_s: float = 0.0
    state_until_s: float = 0.0
    cooldown_until_s: float = 0.0
    maneuver_success: bool = False
    incident_state: str | None = None
    incident_until_s: float = 0.0
    damage_level: float = 0.0
    active_pace_multiplier: float = 1.0
    decision_distance_m: float = 0.0
    tire: TireConditionSnapshot = TireConditionSnapshot()
    stint_start_distance_m: float = 0.0
    stint_lap: int = 0
    pit_plans: tuple[PitStopPlan, ...] = ()
    pit_plan_index: int = 0
    pit_state: str = "none"
    pit_phase_started_s: float = 0.0
    pit_phase_until_s: float = 0.0
    pit_phase_durations_s: tuple[float, float, float, float] = ()
    pit_lane_progress: float = 0.0
    pit_route_start_distance_m: float = 0.0
    pit_route_target_distance_m: float = 0.0
    pit_stop_count: int = 0
    pit_merge_until_s: float = -1.0
    retirement_time_s: float | None = None
    retirement_reason: str | None = None
    pit_command_pending: bool = False
    pit_request_role: str | None = None
    pit_request_compound: str | None = None
    pit_command_backup_plans: tuple[PitStopPlan, ...] = ()
    pace_mode: str = "STANDARD"
    timing_crossings: dict[tuple[int, int], float] = field(default_factory=dict)
    drs_eligibility: dict[int, bool] = field(default_factory=dict)
    drs_active: bool = False
    dirty_air_active: bool = False
    dirty_air_strength: float = 0.0
    tow_strength: float = 0.0
    attack_mode: str = "none"
    maneuver_side: int = 0
    drs_train_id: str | None = None
    drs_train_size: int = 0
    drs_train_position: int | None = None
    drs_train_member_ids: tuple[int | str, ...] = ()
    maneuver_group_id: str | None = None
    maneuver_group_size: int = 0
    maneuver_group_member_ids: tuple[int | str, ...] = ()
    maneuver_group_phase: str | None = None
    maneuver_group_corridor_index: int | None = None


@dataclass(frozen=True, slots=True)
class ProgressRuntimeCheckpoint:
    """Bounded authority required to replay an unpublished progress future."""

    snapshot_hash: str
    total_laps: int
    tick_seconds: float
    tick_index: int
    logical_time_s: float
    authority_time_s: float
    next_decision_time_s: float
    next_motion_boundary_s: float
    cars: dict[int | str, _ProgressCar]
    race_order: tuple[int | str, ...]
    logical_event_count: int
    metrics: dict[str, int]
    rng_states: dict[str, object]
    finished: bool
    race_control_state: str
    race_control_phase: str
    race_control_phase_started_s: float
    race_control_phase_until_s: float
    sc_lap_deficit: dict[int | str, int]
    restart_attack_cooldown_until_s: float
    drs_enable_after_leader_distance_m: float
    command_sequence: int
    finish_event_driver_ids: frozenset[int | str]


def _progress_replacement_compound(current: str) -> str:
    """Select the next harder C1-C5 compound for the provisional stint."""

    return {
        "C5": "C4",
        "C4": "C3",
        "C3": "C2",
        "C2": "C1",
        "C1": "C1",
    }.get(current, current)


class ProgressRaceCursor:
    """Bounded event-driven progress cursor for one logical race."""

    def __init__(
        self,
        snapshot: AbstractSessionSnapshot,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = DEFAULT_PROGRESS_TICK_SECONDS,
        grid_hold_duration_s: float = GRID_HOLD_DURATION_S,
        enable_traffic: bool = True,
        enable_incidents: bool = True,
        enable_pit: bool = True,
        pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
    ) -> None:
        if total_laps < 1 or total_laps > 100:
            raise ValueError("total_laps must be in the range 1..100")
        if not isfinite(tick_seconds) or tick_seconds <= 0.0:
            raise ValueError("tick_seconds must be finite and positive")
        if not isfinite(grid_hold_duration_s) or grid_hold_duration_s < 0.0:
            raise ValueError("grid_hold_duration_s must be finite and non-negative")
        if not snapshot.track.segments:
            raise ValueError("progress race requires timing segments")

        self.snapshot = snapshot
        self.total_laps = int(total_laps)
        self.tick_seconds = float(tick_seconds)
        self.grid_hold_duration_s = float(grid_hold_duration_s)
        self.enable_traffic = bool(enable_traffic)
        self.enable_incidents = bool(enable_incidents)
        self.enable_pit = bool(enable_pit)
        self.lap_length_m = float(snapshot.track.track_length_m)
        self.finish_distance_m = self.total_laps * self.lap_length_m
        self.segments = tuple(
            sorted(snapshot.track.segments, key=lambda item: item.start_progress)
        )
        self._validate_segments(self.segments)
        self._timing_loops = self._build_timing_loops()

        entry_by_driver = snapshot.entry_by_driver_id()
        selected_grid = tuple(grid_order or snapshot.initial_grid_order)
        if set(selected_grid) != set(entry_by_driver) or len(selected_grid) != len(entry_by_driver):
            raise ValueError("grid_order must contain each snapshot driver exactly once")
        self.grid_order = selected_grid
        self._race_order = list(selected_grid)
        self._grid_index = {
            driver_id: index for index, driver_id in enumerate(self.grid_order)
        }
        self.rng = IndependentRNG(snapshot.session_seed)
        pit_plans = self._build_progress_pit_plans(pit_strategy)
        self._cars: dict[int | str, _ProgressCar] = {}
        for grid_index, driver_id in enumerate(self.grid_order):
            entry = entry_by_driver[driver_id]
            start_distance_m = -(
                GRID_POLE_DISTANCE_BEHIND_LINE_M + grid_index * GRID_SLOT_SPACING_M
            )
            durations = tuple(
                calculate_segment_time(
                    segment,
                    entry.vehicle.performance,
                    entry.driver,
                    entry.tire,
                    track_evolution=snapshot.environment.initial_track_evolution,
                ).time_s
                for segment in self.segments
            )
            # Locate again from the authoritative duration tuple so the
            # initialization path and subsequent event advancement share the
            # same exact segment timing values.
            segment_index, segment_fraction = self._segment_location(start_distance_m)
            self._cars[driver_id] = _ProgressCar(
                entry=entry,
                grid_index=grid_index,
                race_distance_m=start_distance_m,
                segment_index=segment_index,
                segment_elapsed_s=durations[segment_index] * segment_fraction,
                segment_durations_s=durations,
                decision_distance_m=start_distance_m,
                tire=entry.tire,
                stint_start_distance_m=0.0,
                pit_plans=pit_plans[driver_id],
            )

        self.tick_index = 0
        self.logical_time_s = 0.0
        self._authority_time_s = 0.0
        self._next_decision_time_s = (
            self.grid_hold_duration_s + PROGRESS_DECISION_INTERVAL_S
        )
        self._next_motion_boundary_s = PROGRESS_MOTION_INTERVAL_S
        self.logical_events: list[LogicalEvent] = [
            LogicalEvent(
                event_id="progress:race_started",
                event_type="race_started",
                logical_time_s=0.0,
                payload=(
                    ("engine_version", PROGRESS_RACE_ENGINE_VERSION),
                    ("racecraft_authority_version", PROGRESS_RACECRAFT_AUTHORITY_VERSION),
                    ("timing_authority_version", PROGRESS_TIMING_AUTHORITY_VERSION),
                    ("total_laps", self.total_laps),
                ),
            )
        ]
        self.metrics: dict[str, int] = {
            "attack_started_count": 0,
            "overtake_completed_count": 0,
            "attack_failed_count": 0,
            "incident_count": 0,
            "lockup_count": 0,
            "run_wide_count": 0,
            "spin_count": 0,
            "contact_count": 0,
            "pair_contact_count": 0,
            "retirement_count": 0,
            "implicit_rank_swap_count": 0,
            "distance_reversal_count": 0,
            "spacing_constraint_count": 0,
            "pit_requested_count": 0,
            "pit_stop_completed_count": 0,
            "pit_order_change_count": 0,
            "incident_order_change_count": 0,
            "vsc_period_count": 0,
            "safety_car_period_count": 0,
            "safety_car_queue_formed_count": 0,
            "restart_count": 0,
            "caution_suppressed_attack_count": 0,
            "local_yellow_count": 0,
            "drs_activation_count": 0,
            "dirty_air_active_decision_count": 0,
            "drs_train_formed_count": 0,
            "max_drs_train_size": 0,
            "maneuver_group_formed_count": 0,
            "maneuver_group_dissolved_count": 0,
            "max_maneuver_group_size": 0,
        }
        self.race_control_state = "green"
        self.race_control_phase = "green"
        self._race_control_phase_started_s = 0.0
        self._race_control_phase_until_s = -1.0
        self._sc_lap_deficit: dict[int | str, int] = {}
        self._restart_attack_cooldown_until_s = -1.0
        self._drs_enable_after_leader_distance_m = (
            DRS_INITIAL_ENABLE_AFTER_LEADER_LAPS * self.lap_length_m
        )
        self._command_sequence = 0
        self._finish_event_driver_ids: set[int | str] = set()
        self._finished = False
        self._current_tick = self._build_tick()

    @staticmethod
    def _validate_segments(segments: tuple[SegmentRequirement, ...]) -> None:
        expected_start = 0.0
        for segment in segments:
            if abs(segment.start_progress - expected_start) > 1e-9:
                raise ValueError("progress timing segments must cover the lap contiguously")
            expected_start = segment.end_progress
        if abs(expected_start - 1.0) > 1e-9:
            raise ValueError("progress timing segments must cover the complete lap")

    def _build_timing_loops(self) -> tuple[_ProgressTimingLoop, ...]:
        """Build stable transponder anchors from the snapshot mini sectors."""

        loops: list[_ProgressTimingLoop] = []
        for sector_index, sector in enumerate(self.snapshot.track.sectors, start=1):
            count = max(1, int(sector.mini_sector_count))
            span = sector.end_progress - sector.start_progress
            for mini_index in range(1, count + 1):
                loops.append(
                    _ProgressTimingLoop(
                        index=len(loops),
                        progress=(
                            sector.start_progress
                            + span * (mini_index - 1) / count
                        ),
                        sector_index=sector_index,
                        mini_sector_index=mini_index,
                    )
                )
        if not loops or abs(loops[0].progress) > 1e-9:
            raise ValueError("progress timing loops must start at the start line")
        return tuple(loops)

    def _build_progress_pit_plans(
        self,
        pit_strategy: Mapping[int | str, Sequence[int]] | None,
    ) -> dict[int | str, tuple[PitStopPlan, ...]]:
        entry_by_driver = self.snapshot.entry_by_driver_id()
        if pit_strategy is not None and not set(pit_strategy).issubset(entry_by_driver):
            raise ValueError("pit_strategy contains a driver outside the snapshot grid")
        plans: dict[int | str, tuple[PitStopPlan, ...]] = {}
        ordered_entries = tuple(
            sorted(self.snapshot.entries, key=lambda entry: str(entry.driver_id))
        )
        for index, entry in enumerate(ordered_entries):
            if not self.enable_pit:
                requested_laps: Sequence[int] = ()
            elif pit_strategy is None:
                if self.total_laps < 5:
                    requested_laps = ()
                else:
                    base_lap = max(2, self.total_laps // 2)
                    requested_laps = (
                        min(self.total_laps - 1, base_lap + index % 3),
                    )
            else:
                requested_laps = pit_strategy.get(entry.driver_id, ())
            stop_laps = tuple(sorted(set(int(lap) for lap in requested_laps)))
            if any(lap < 1 or lap >= self.total_laps for lap in stop_laps):
                raise ValueError(
                    "pit stop laps must be within the race and before the final lap"
                )
            compound = entry.tire.physical_compound
            driver_plans: list[PitStopPlan] = []
            stream = self.rng.team_pit(entry.team_id)
            for stop_index, stop_lap in enumerate(stop_laps):
                compound = _progress_replacement_compound(compound)
                driver_plans.append(
                    PitStopPlan(
                        driver_id=entry.driver_id,
                        stop_lap=stop_lap,
                        replacement_compound=compound,
                        replacement_role="MEDIUM" if stop_index == 0 else "HARD",
                        service_time_s=stream.uniform(1.90, 2.30),
                    )
                )
            plans[entry.driver_id] = tuple(driver_plans)
        return plans

    def _pit_phase_durations(self, plan: PitStopPlan) -> tuple[float, float, float, float]:
        quantum = PROGRESS_DECISION_INTERVAL_S
        total_time_s = self.snapshot.track.base_lap_time_s * 0.34 + 3.0
        service_time_s = ceil(plan.service_time_s / quantum) * quantum
        transit_time_s = max(12.0, total_time_s - service_time_s)
        entry_time_s = max(1.0, round(transit_time_s * 0.12 / quantum) * quantum)
        lane_time_s = max(3.0, round(transit_time_s * 0.35 / quantum) * quantum)
        exit_time_s = max(5.0, transit_time_s - entry_time_s - lane_time_s)
        exit_time_s = ceil(exit_time_s / quantum) * quantum
        return entry_time_s, lane_time_s, service_time_s, exit_time_s

    def _pit_track_progresses(self) -> tuple[float, float]:
        geometry = self.snapshot.track.display_geometry
        if geometry is None:
            return 0.88, 0.10
        return (
            float(getattr(geometry, "pit_entry_progress", 0.88)) % 1.0,
            float(getattr(geometry, "pit_exit_progress", 0.10)) % 1.0,
        )

    def _segment_location(self, race_distance_m: float) -> tuple[int, float]:
        local_progress = (race_distance_m % self.lap_length_m) / self.lap_length_m
        for index, segment in enumerate(self.segments):
            if segment.start_progress <= local_progress < segment.end_progress:
                fraction = (local_progress - segment.start_progress) / (
                    segment.end_progress - segment.start_progress
                )
                return index, fraction
        return len(self.segments) - 1, 1.0

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def current_tick(self) -> ProgressRaceTick:
        return self._current_tick

    def capture_runtime_checkpoint(self) -> ProgressRuntimeCheckpoint:
        """Capture all bounded authority needed to reproduce future ticks."""

        return ProgressRuntimeCheckpoint(
            snapshot_hash=self.snapshot.snapshot_hash,
            total_laps=self.total_laps,
            tick_seconds=self.tick_seconds,
            tick_index=self.tick_index,
            logical_time_s=self.logical_time_s,
            authority_time_s=self._authority_time_s,
            next_decision_time_s=self._next_decision_time_s,
            next_motion_boundary_s=self._next_motion_boundary_s,
            cars=deepcopy(self._cars),
            race_order=tuple(self._race_order),
            logical_event_count=len(self.logical_events),
            metrics=dict(self.metrics),
            rng_states=self.rng.export_states(),
            finished=self._finished,
            race_control_state=self.race_control_state,
            race_control_phase=self.race_control_phase,
            race_control_phase_started_s=self._race_control_phase_started_s,
            race_control_phase_until_s=self._race_control_phase_until_s,
            sc_lap_deficit=dict(self._sc_lap_deficit),
            restart_attack_cooldown_until_s=self._restart_attack_cooldown_until_s,
            drs_enable_after_leader_distance_m=(
                self._drs_enable_after_leader_distance_m
            ),
            command_sequence=self._command_sequence,
            finish_event_driver_ids=frozenset(self._finish_event_driver_ids),
        )

    def restore_runtime_checkpoint(self, checkpoint: ProgressRuntimeCheckpoint) -> None:
        """Discard unpublished future state and resume from ``checkpoint``."""

        if checkpoint.snapshot_hash != self.snapshot.snapshot_hash:
            raise ValueError("progress checkpoint belongs to a different snapshot")
        if checkpoint.total_laps != self.total_laps:
            raise ValueError("progress checkpoint total_laps does not match cursor")
        if abs(checkpoint.tick_seconds - self.tick_seconds) > 1e-12:
            raise ValueError("progress checkpoint tick_seconds does not match cursor")
        self.tick_index = checkpoint.tick_index
        self.logical_time_s = checkpoint.logical_time_s
        self._authority_time_s = checkpoint.authority_time_s
        self._next_decision_time_s = checkpoint.next_decision_time_s
        self._next_motion_boundary_s = checkpoint.next_motion_boundary_s
        self._cars = deepcopy(checkpoint.cars)
        self._race_order = list(checkpoint.race_order)
        del self.logical_events[checkpoint.logical_event_count :]
        self.metrics = dict(checkpoint.metrics)
        self.rng.restore_states(checkpoint.rng_states)
        self._finished = checkpoint.finished
        self.race_control_state = checkpoint.race_control_state
        self.race_control_phase = checkpoint.race_control_phase
        self._race_control_phase_started_s = checkpoint.race_control_phase_started_s
        self._race_control_phase_until_s = checkpoint.race_control_phase_until_s
        self._sc_lap_deficit = dict(checkpoint.sc_lap_deficit)
        self._restart_attack_cooldown_until_s = (
            checkpoint.restart_attack_cooldown_until_s
        )
        self._drs_enable_after_leader_distance_m = (
            checkpoint.drs_enable_after_leader_distance_m
        )
        self._command_sequence = checkpoint.command_sequence
        self._finish_event_driver_ids = set(checkpoint.finish_event_driver_ids)
        self._current_tick = self._build_tick()

    def _append_command_event(
        self,
        *,
        event_type: str,
        driver_id: int | str,
        payload: Sequence[tuple[str, Any]],
    ) -> LogicalEvent:
        self._command_sequence += 1
        event = LogicalEvent(
            event_id=(
                f"progress:command:{self._command_sequence}:{event_type}:"
                f"{driver_id}"
            ),
            event_type=event_type,
            logical_time_s=self.logical_time_s,
            driver_ids=(driver_id,),
            payload=tuple(payload) + (("command_sequence", self._command_sequence),),
        )
        self.logical_events.append(event)
        return event

    def set_pace_mode(self, driver_id: int | str, pace_mode: str) -> LogicalEvent:
        if driver_id not in self._cars:
            raise ValueError("driver is not part of the progress race")
        normalized = str(pace_mode).upper()
        if normalized not in {"CONSERVE", "STANDARD", "ATTACK"}:
            raise ValueError("pace mode must be CONSERVE, STANDARD or ATTACK")
        car = self._cars[driver_id]
        if self._is_terminal(car):
            raise ValueError("cannot change pace mode for a terminal driver")
        previous = car.pace_mode
        car.pace_mode = normalized
        car.active_pace_multiplier = self._race_control_pace_multiplier(
            car,
            self._pace_multiplier(car),
        )
        event = self._append_command_event(
            event_type="pace_mode_changed",
            driver_id=driver_id,
            payload=(
                ("pace_mode", normalized),
                ("previous_pace_mode", previous),
            ),
        )
        self._current_tick = self._build_tick()
        return event

    def request_pit(
        self,
        driver_id: int | str,
        *,
        tire_role: str,
        physical_compound: str,
    ) -> LogicalEvent:
        if not self.enable_pit:
            raise ValueError("pit operations are disabled")
        if driver_id not in self._cars:
            raise ValueError("driver is not part of the progress race")
        car = self._cars[driver_id]
        if self._is_terminal(car) or car.pit_state != "none":
            raise ValueError("driver cannot accept a pit call in the current state")
        current_lap_index = floor(max(0.0, car.race_distance_m) / self.lap_length_m)
        stop_lap = current_lap_index + 1
        if stop_lap >= self.total_laps:
            raise ValueError("pit call is too late for the remaining race distance")
        if not car.pit_command_pending:
            car.pit_command_backup_plans = car.pit_plans[car.pit_plan_index :]
        stream = self.rng.stream(
            f"team:{car.entry.team_id}:progress_command_pit:{self._command_sequence + 1}"
        )
        requested_plan = PitStopPlan(
            driver_id=driver_id,
            stop_lap=stop_lap,
            replacement_compound=str(physical_compound).upper(),
            replacement_role=str(tire_role).upper(),
            service_time_s=stream.uniform(1.90, 2.30),
        )
        future_plans = tuple(
            plan
            for plan in car.pit_command_backup_plans
            if plan.stop_lap > stop_lap
        )
        car.pit_plans = (
            car.pit_plans[: car.pit_plan_index]
            + (requested_plan,)
            + future_plans
        )
        car.pit_command_pending = True
        car.pit_request_role = requested_plan.replacement_role
        car.pit_request_compound = requested_plan.replacement_compound
        event = self._append_command_event(
            event_type="pit_call_registered",
            driver_id=driver_id,
            payload=(
                ("physical_compound", requested_plan.replacement_compound),
                ("pit_stop_lap", stop_lap),
                ("tire_role", requested_plan.replacement_role),
            ),
        )
        self._current_tick = self._build_tick()
        return event

    def cancel_pit(self, driver_id: int | str) -> LogicalEvent:
        if driver_id not in self._cars:
            raise ValueError("driver is not part of the progress race")
        car = self._cars[driver_id]
        if not car.pit_command_pending or car.pit_state != "none":
            raise ValueError("driver has no cancellable pit call")
        car.pit_plans = (
            car.pit_plans[: car.pit_plan_index] + car.pit_command_backup_plans
        )
        car.pit_command_pending = False
        car.pit_request_role = None
        car.pit_request_compound = None
        car.pit_command_backup_plans = ()
        event = self._append_command_event(
            event_type="pit_call_cancelled",
            driver_id=driver_id,
            payload=(),
        )
        self._current_tick = self._build_tick()
        return event

    @staticmethod
    def _is_terminal(car: _ProgressCar) -> bool:
        return car.finish_time_s is not None or car.retirement_time_s is not None

    def _is_pit_order_active(self, car: _ProgressCar, now: float | None = None) -> bool:
        logical_time_s = self._authority_time_s if now is None else now
        return car.pit_state != "none" or logical_time_s <= car.pit_merge_until_s + 1e-9

    def _segment_length_m(self, segment_index: int) -> float:
        segment = self.segments[segment_index]
        return (segment.end_progress - segment.start_progress) * self.lap_length_m

    def _segment_end_distance_m(self, car: _ProgressCar) -> float:
        segment = self.segments[car.segment_index]
        lap_index = floor(car.race_distance_m / self.lap_length_m)
        return (lap_index + segment.end_progress) * self.lap_length_m

    def _record_timing_loop_crossings(
        self,
        car: _ProgressCar,
        previous_distance_m: float,
        current_distance_m: float,
        step_start_time_s: float,
        delta_seconds: float,
    ) -> None:
        """Record interpolated logical times at every crossed timing line."""

        if current_distance_m <= previous_distance_m + 1e-12:
            return
        previous_total = previous_distance_m / self.lap_length_m
        current_total = current_distance_m / self.lap_length_m
        span = current_total - previous_total
        for loop in self._timing_loops:
            lap_index = floor(previous_total - loop.progress) + 1
            checkpoint = lap_index + loop.progress
            while checkpoint <= current_total + 1e-12:
                ratio = min(
                    1.0,
                    max(0.0, (checkpoint - previous_total) / span),
                )
                car.timing_crossings[(lap_index, loop.index)] = round(
                    step_start_time_s + ratio * max(0.0, delta_seconds),
                    9,
                )
                lap_index += 1
                checkpoint = lap_index + loop.progress

        minimum_lap = floor(current_total) - TIMING_CROSSING_LAPS_TO_RETAIN
        if minimum_lap > -1:
            car.timing_crossings = {
                key: value
                for key, value in car.timing_crossings.items()
                if key[0] >= minimum_lap
            }

    def _timing_gap_seconds_between(
        self,
        ahead: _ProgressCar,
        follower: _ProgressCar,
    ) -> float | None:
        """Return the measured separation at the latest common timing line."""

        common = ahead.timing_crossings.keys() & follower.timing_crossings.keys()
        if not common:
            return None
        latest_key = max(
            common,
            key=lambda key: key[0] + self._timing_loops[key[1]].progress,
        )
        gap_seconds = (
            follower.timing_crossings[latest_key]
            - ahead.timing_crossings[latest_key]
        )
        return gap_seconds if gap_seconds > 0.0005 else None

    def _logical_speed_mps(self, car: _ProgressCar) -> float:
        if self._is_terminal(car) or self.logical_time_s <= self.grid_hold_duration_s:
            return 0.0
        if car.pit_state != "none":
            route_span = (
                PROGRESS_PIT_PHASE_ROUTE_RANGES[car.pit_state][1]
                - PROGRESS_PIT_PHASE_ROUTE_RANGES[car.pit_state][0]
            )
            return max(
                0.0,
                (
                    car.pit_route_target_distance_m
                    - car.pit_route_start_distance_m
                )
                * route_span
                / max(car.pit_phase_until_s - car.pit_phase_started_s, 1e-9),
            )
        return self._base_speed_mps(car) * max(0.0, car.active_pace_multiplier)

    def _spatial_timing_gap_seconds_between(
        self,
        ahead: _ProgressCar,
        follower: _ProgressCar,
    ) -> float | None:
        """Estimate the live between-loop gap from progress-authority state."""

        distance_m = ahead.race_distance_m - follower.race_distance_m
        if distance_m <= 1e-9:
            return None
        follower_lap_s = sum(follower.segment_durations_s) / max(
            0.20,
            follower.active_pace_multiplier,
        )
        lap_estimate_s = distance_m / self.lap_length_m * follower_lap_s
        average_speed_mps = 0.5 * (
            self._logical_speed_mps(ahead) + self._logical_speed_mps(follower)
        )
        if average_speed_mps < 8.0:
            return max(0.0, lap_estimate_s)
        speed_estimate_s = distance_m / average_speed_mps
        if distance_m <= 250.0:
            speed_weight = 0.72
        elif distance_m <= 750.0:
            speed_weight = 0.55
        else:
            speed_weight = 0.30
        blended = (
            speed_estimate_s * speed_weight
            + lap_estimate_s * (1.0 - speed_weight)
        )
        return min(
            lap_estimate_s * 2.5,
            max(lap_estimate_s * 0.35, blended),
        )

    def _live_timing_gap_seconds_between(
        self,
        ahead: _ProgressCar,
        follower: _ProgressCar,
    ) -> tuple[float, bool]:
        """Blend a common timing-line split with live logical progress."""

        measured = self._timing_gap_seconds_between(ahead, follower)
        spatial = self._spatial_timing_gap_seconds_between(ahead, follower)
        if measured is None:
            return max(0.0, spatial or 0.0), False
        if spatial is None:
            return measured, True
        distance_m = max(0.0, ahead.race_distance_m - follower.race_distance_m)
        live_weight = 0.55 if distance_m <= 750.0 else 0.35
        return (
            measured * (1.0 - live_weight) + spatial * live_weight,
            True,
        )

    @staticmethod
    def _crossed_progress_anchor(
        previous_total_progress: float,
        current_total_progress: float,
        anchor_progress: float,
    ) -> bool:
        if current_total_progress <= previous_total_progress + 1e-12:
            return False
        return floor(current_total_progress - anchor_progress + 1e-9) > floor(
            previous_total_progress - anchor_progress + 1e-9
        )

    def _drs_zone_index(self, race_distance_m: float) -> int | None:
        progress = (race_distance_m / self.lap_length_m) % 1.0
        for index, zone in enumerate(self.snapshot.track.drs_zones):
            start = zone.start_progress
            end = zone.end_progress
            if start <= end and start <= progress <= end:
                return index
            if start > end and (progress >= start or progress <= end):
                return index
        return None

    def _drs_race_control_enabled(self) -> bool:
        if self.race_control_state != "green" or not self.snapshot.track.drs_zones:
            return False
        leader = self._cars[self._race_order[0]]
        return (
            leader.race_distance_m
            >= self._drs_enable_after_leader_distance_m - 1e-9
        )

    def _update_drs_detection_eligibility(
        self,
        car: _ProgressCar,
        previous_distance_m: float,
    ) -> None:
        if (
            not self.enable_traffic
            or not self._drs_race_control_enabled()
            or self._is_terminal(car)
        ):
            car.drs_active = False
            return
        previous_total = previous_distance_m / self.lap_length_m
        current_total = car.race_distance_m / self.lap_length_m
        if current_total <= previous_total + 1e-12:
            return
        order_index = self._race_order.index(car.entry.driver_id)
        ahead = self._cars[self._race_order[order_index - 1]] if order_index > 0 else None
        for zone_index, zone in enumerate(self.snapshot.track.drs_zones):
            if not self._crossed_progress_anchor(
                previous_total,
                current_total,
                zone.detection_progress,
            ):
                continue
            eligible = False
            if (
                ahead is not None
                and not self._is_terminal(ahead)
                and not self._is_pit_order_active(car)
                and not self._is_pit_order_active(ahead)
            ):
                gap_seconds, _ = self._live_timing_gap_seconds_between(ahead, car)
                eligible = 0.0 < gap_seconds <= DRS_ELIGIBILITY_GAP_S
            car.drs_eligibility[zone_index] = eligible

    @staticmethod
    def _smoothstep01(value: float) -> float:
        value = min(1.0, max(0.0, value))
        return value * value * (3.0 - 2.0 * value)

    def _update_racecraft_context(self) -> None:
        """Update bounded DRS, tow and dirty-air state from logical traffic."""

        if not self.enable_traffic:
            for car in self._cars.values():
                car.drs_active = False
                car.dirty_air_active = False
                car.dirty_air_strength = 0.0
                car.tow_strength = 0.0
                car.attack_mode = "none"
            return
        drs_enabled = self._drs_race_control_enabled()
        for position, driver_id in enumerate(self._race_order):
            car = self._cars[driver_id]
            was_drs_active = car.drs_active
            car.drs_active = False
            car.dirty_air_active = False
            car.dirty_air_strength = 0.0
            car.tow_strength = 0.0
            if (
                position == 0
                or self._is_terminal(car)
                or self._is_pit_order_active(car)
            ):
                if car.traffic_state not in {"attack", "side_by_side", "clearance"}:
                    car.attack_mode = "none"
                continue
            ahead = self._cars[self._race_order[position - 1]]
            if self._is_terminal(ahead) or self._is_pit_order_active(ahead):
                continue
            gap_m = ahead.race_distance_m - car.race_distance_m
            if 0.0 < gap_m < ABSTRACT_WAKE_MAX_GAP_M:
                closeness = self._smoothstep01(
                    (ABSTRACT_WAKE_MAX_GAP_M - gap_m)
                    / ABSTRACT_WAKE_MAX_GAP_M
                )
                segment_type = self.segments[car.segment_index].segment_type
                tow_factor, dirty_factor = {
                    "straight": (1.00, 0.00),
                    "heavy_braking": (0.25, 0.35),
                    "traction": (0.10, 0.55),
                    "sweeping": (0.05, 1.00),
                    "technical": (0.05, 0.80),
                }.get(segment_type, (0.05, 0.65))
                car.tow_strength = closeness * tow_factor
                car.dirty_air_strength = closeness * dirty_factor
                car.dirty_air_active = car.dirty_air_strength >= 0.08
                if car.dirty_air_active:
                    self.metrics["dirty_air_active_decision_count"] += 1

            zone_index = self._drs_zone_index(car.race_distance_m)
            car.drs_active = bool(
                drs_enabled
                and zone_index is not None
                and car.drs_eligibility.get(zone_index, False)
                and gap_m > 0.0
            )
            if car.drs_active and not was_drs_active:
                self.metrics["drs_activation_count"] += 1
            if car.traffic_state not in {"attack", "side_by_side", "clearance"}:
                if car.drs_active:
                    car.attack_mode = "drs"
                elif car.tow_strength >= 0.10:
                    car.attack_mode = "tow"
                elif car.dirty_air_active:
                    car.attack_mode = "dirty_air"
                else:
                    car.attack_mode = "none"

    def _clear_drs_state(self) -> None:
        for car in self._cars.values():
            car.drs_active = False
            car.drs_eligibility.clear()

    def _update_drs_trains(self) -> None:
        """Identify contiguous logical DRS trains without changing rank authority."""

        previous_by_driver = {
            car.entry.driver_id: car.drs_train_id for car in self._cars.values()
        }
        for car in self._cars.values():
            car.drs_train_id = None
            car.drs_train_size = 0
            car.drs_train_position = None
            car.drs_train_member_ids = ()
        if (
            not self.enable_traffic
            or self.race_control_state != "green"
            or not self._drs_race_control_enabled()
        ):
            return

        chains: list[list[_ProgressCar]] = []
        current: list[_ProgressCar] = []
        for car in self._ordered_cars():
            if self._is_terminal(car) or self._is_pit_order_active(car):
                if current:
                    chains.append(current)
                current = []
                continue
            if not current:
                current = [car]
                continue
            ahead = current[-1]
            interval_s, _ = self._live_timing_gap_seconds_between(ahead, car)
            if 0.0 < interval_s <= DRS_TRAIN_MAX_INTERVAL_S:
                current.append(car)
            else:
                chains.append(current)
                current = [car]
        if current:
            chains.append(current)

        for chain in chains:
            if len(chain) < 3:
                continue
            member_ids = tuple(car.entry.driver_id for car in chain)
            train_id = "drs-train:" + ":".join(str(item) for item in member_ids)
            if all(
                previous_by_driver.get(car.entry.driver_id) is None for car in chain
            ):
                self.metrics["drs_train_formed_count"] += 1
            self.metrics["max_drs_train_size"] = max(
                self.metrics["max_drs_train_size"], len(chain)
            )
            for index, car in enumerate(chain):
                car.drs_train_id = train_id
                car.drs_train_size = len(chain)
                car.drs_train_position = index
                car.drs_train_member_ids = member_ids
                if (
                    index > 0
                    and car.traffic_state not in {
                        "attack", "side_by_side", "defend", "clearance"
                    }
                ):
                    car.attack_mode = "drs_train"

    def _update_maneuver_groups(self, now: float) -> None:
        """Expose a bounded third-car opportunity around one authoritative pair.

        The active attacker/defender pair remains the only rank-changing unit.
        A nearby third car receives a presentation corridor and affects tactical
        pressure, but may only change rank through a later explicit pair event.
        """

        previous_groups: dict[str, tuple[int | str, ...]] = {}
        for car in self._cars.values():
            if car.maneuver_group_id:
                previous_groups.setdefault(
                    car.maneuver_group_id, car.maneuver_group_member_ids
                )
            car.maneuver_group_id = None
            car.maneuver_group_size = 0
            car.maneuver_group_member_ids = ()
            car.maneuver_group_phase = None
            car.maneuver_group_corridor_index = None

        if (
            not self.enable_traffic
            or self.race_control_state != "green"
            or self.snapshot.track.track_width_m < MANEUVER_GROUP_MIN_TRACK_WIDTH_M
        ):
            new_groups: dict[str, tuple[int | str, ...]] = {}
        else:
            new_groups = {}
            assigned: set[int | str] = set()
            attackers = sorted(
                (
                    car for car in self._cars.values()
                    if car.traffic_state in {"side_by_side", "clearance"}
                    and car.target_driver_id in self._cars
                ),
                key=lambda item: str(item.entry.driver_id),
            )
            for attacker in attackers:
                defender = self._cars[attacker.target_driver_id]
                pair_ids = {attacker.entry.driver_id, defender.entry.driver_id}
                if pair_ids & assigned:
                    continue
                if self.segments[attacker.segment_index].segment_type not in {
                    "straight", "heavy_braking"
                }:
                    continue
                pair_indices = sorted(
                    self._race_order.index(driver_id) for driver_id in pair_ids
                )
                trailing_index = pair_indices[-1] + 1
                if trailing_index >= len(self._race_order):
                    continue
                opportunist = self._cars[self._race_order[trailing_index]]
                if (
                    opportunist.entry.driver_id in assigned
                    or self._is_terminal(opportunist)
                    or self._is_pit_order_active(opportunist, now)
                    or opportunist.incident_state is not None
                    or opportunist.traffic_state not in {"clear", "follow"}
                ):
                    continue
                rear_car = self._cars[self._race_order[pair_indices[-1]]]
                gap_m = rear_car.race_distance_m - opportunist.race_distance_m
                group_id = (
                    f"maneuver-group:{attacker.entry.driver_id}:"
                    f"{defender.entry.driver_id}"
                )
                continuing_group = bool(
                    group_id in previous_groups
                    and opportunist.entry.driver_id in previous_groups[group_id]
                )
                opportunist_pressure = bool(
                    opportunist.drs_active or opportunist.tow_strength >= 0.35
                )
                if not (
                    0.0 <= gap_m <= MANEUVER_GROUP_MAX_GAP_M
                    and (continuing_group or opportunist_pressure)
                ):
                    continue

                members = tuple(
                    self._race_order[index]
                    for index in range(pair_indices[0], trailing_index + 1)
                )[:MANEUVER_GROUP_MAX_SIZE]
                if len(members) < 3:
                    continue
                phase = {
                    "side_by_side": "three_wide",
                    "clearance": "merge",
                }[attacker.traffic_state]
                attacker_index = 0 if attacker.maneuver_side < 0 else 2
                defender_index = 2 - attacker_index
                corridor_by_driver = {
                    attacker.entry.driver_id: attacker_index,
                    defender.entry.driver_id: defender_index,
                    opportunist.entry.driver_id: 1,
                }
                new_groups[group_id] = members
                assigned.update(members)
                for driver_id in members:
                    car = self._cars[driver_id]
                    car.maneuver_group_id = group_id
                    car.maneuver_group_size = len(members)
                    car.maneuver_group_member_ids = members
                    car.maneuver_group_phase = phase
                    car.maneuver_group_corridor_index = corridor_by_driver[driver_id]
                self.metrics["max_maneuver_group_size"] = max(
                    self.metrics["max_maneuver_group_size"], len(members)
                )

        for group_id in sorted(set(new_groups) - set(previous_groups)):
            members = new_groups[group_id]
            self.metrics["maneuver_group_formed_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=(
                        f"progress:maneuver-group:{group_id}:"
                        f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:formed"
                    ),
                    event_type="maneuver_group_formed",
                    logical_time_s=now,
                    driver_ids=members,
                    payload=(("group_size", len(members)), ("phase", "three_wide")),
                )
            )
        for group_id in sorted(set(previous_groups) - set(new_groups)):
            members = previous_groups[group_id]
            self.metrics["maneuver_group_dissolved_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=(
                        f"progress:maneuver-group:{group_id}:"
                        f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:dissolved"
                    ),
                    event_type="maneuver_group_dissolved",
                    logical_time_s=now,
                    driver_ids=members,
                )
            )

    def _base_speed_mps(self, car: _ProgressCar) -> float:
        return self._segment_length_m(car.segment_index) / car.segment_durations_s[
            car.segment_index
        ]

    def _pace_multiplier(self, car: _ProgressCar) -> float:
        if car.retirement_time_s is not None:
            return 0.0
        if car.pit_state != "none":
            return 0.0
        multiplier = max(0.90, 1.0 - car.damage_level * 0.30)
        if self.enable_pit:
            wear_penalty = (
                max(0.0, car.tire.wear_laps - 1.0)
                * 0.002
                / max(car.entry.vehicle.performance.tire_management, 0.1)
            )
            multiplier *= max(0.90, 1.0 - wear_penalty)
        if car.incident_state is not None and self._authority_time_s < car.incident_until_s:
            multiplier *= {
                "lockup": 0.55,
                "run_wide": 0.40,
                "spin": 0.0,
                "contact": 0.60,
            }.get(car.incident_state, 0.75)
        multiplier *= {
            "CONSERVE": 0.985,
            "STANDARD": 1.0,
            "ATTACK": 1.020,
        }[car.pace_mode]
        if not self.enable_traffic:
            return multiplier
        if car.traffic_state == "follow" and car.target_driver_id in self._cars:
            ahead = self._cars[car.target_driver_id]
            multiplier *= min(
                1.0,
                self._base_speed_mps(ahead) / max(self._base_speed_mps(car), 1e-9),
            )
        elif car.traffic_state == "attack":
            multiplier *= 1.025 if car.maneuver_success else 1.005
        elif car.traffic_state == "side_by_side":
            multiplier *= 1.055 if car.maneuver_success else 0.995
        elif car.traffic_state == "defend":
            multiplier *= 0.985
        elif car.traffic_state == "clearance":
            multiplier *= 1.010
        multiplier *= 1.0 + ABSTRACT_TOW_PACE_GAIN_MAX * car.tow_strength
        multiplier *= 1.0 - (
            ABSTRACT_DIRTY_AIR_PACE_LOSS_MAX * car.dirty_air_strength
        )
        if car.drs_active:
            multiplier *= 1.0 + ABSTRACT_DRS_PACE_GAIN
        return max(0.0, multiplier)

    def _set_race_distance(self, car: _ProgressCar, race_distance_m: float) -> None:
        previous = car.race_distance_m
        bounded = min(self.finish_distance_m, max(previous, race_distance_m))
        if bounded < previous - 1e-9:
            self.metrics["distance_reversal_count"] += 1
            return
        car.race_distance_m = bounded
        self._record_timing_loop_crossings(
            car,
            previous,
            bounded,
            self._authority_time_s,
            0.0,
        )
        segment_index, fraction = self._segment_location(bounded)
        car.segment_index = segment_index
        car.segment_elapsed_s = car.segment_durations_s[segment_index] * fraction
        if bounded >= self.finish_distance_m - 1e-9:
            car.race_distance_m = self.finish_distance_m
            car.finish_time_s = round(self._authority_time_s, 6)
        else:
            car.finish_time_s = None

    def _advance_car(
        self,
        car: _ProgressCar,
        motion_duration_s: float,
        motion_start_time_s: float,
        pace_multiplier: float,
    ) -> None:
        if (
            self._is_terminal(car)
            or motion_duration_s <= 0.0
            or pace_multiplier <= 0.0
        ):
            return
        remaining_progress_s = motion_duration_s * pace_multiplier
        consumed_progress_s = 0.0
        while remaining_progress_s > 1e-12 and car.finish_time_s is None:
            duration_s = car.segment_durations_s[car.segment_index]
            remaining_segment_s = max(0.0, duration_s - car.segment_elapsed_s)
            segment_end_m = self._segment_end_distance_m(car)
            if remaining_segment_s <= 1e-12:
                consume_s = 0.0
            else:
                consume_s = min(remaining_progress_s, remaining_segment_s)
                car.race_distance_m += (
                    self._segment_length_m(car.segment_index)
                    * consume_s
                    / duration_s
                )
                car.segment_elapsed_s += consume_s
                remaining_progress_s -= consume_s
                consumed_progress_s += consume_s

            if car.segment_elapsed_s < duration_s - 1e-10:
                break

            car.race_distance_m = segment_end_m
            if car.race_distance_m >= self.finish_distance_m - 1e-9:
                car.race_distance_m = self.finish_distance_m
                car.finish_time_s = round(
                    motion_start_time_s + consumed_progress_s / pace_multiplier,
                    6,
                )
                break
            car.segment_index = (car.segment_index + 1) % len(self.segments)
            car.segment_elapsed_s = 0.0

    def _advance_pit_car(
        self,
        car: _ProgressCar,
        motion_duration_s: float,
        motion_start_time_s: float,
    ) -> None:
        if car.pit_state == "none" or motion_duration_s <= 0.0:
            return
        route_start, route_end = PROGRESS_PIT_PHASE_ROUTE_RANGES[car.pit_state]
        phase_duration_s = max(
            car.pit_phase_until_s - car.pit_phase_started_s,
            PROGRESS_MOTION_INTERVAL_S,
        )
        phase_fraction = min(
            1.0,
            max(
                0.0,
                (motion_start_time_s + motion_duration_s - car.pit_phase_started_s)
                / phase_duration_s,
            ),
        )
        car.pit_lane_progress = route_start + (route_end - route_start) * phase_fraction
        route_distance_m = (
            car.pit_route_target_distance_m - car.pit_route_start_distance_m
        )
        race_distance_m = (
            car.pit_route_start_distance_m + route_distance_m * car.pit_lane_progress
        )
        car.race_distance_m = max(car.race_distance_m, race_distance_m)
        segment_index, fraction = self._segment_location(car.race_distance_m)
        car.segment_index = segment_index
        car.segment_elapsed_s = car.segment_durations_s[segment_index] * fraction

    def _recompute_segment_durations(self, car: _ProgressCar) -> None:
        car.segment_durations_s = tuple(
            calculate_segment_time(
                segment,
                car.entry.vehicle.performance,
                car.entry.driver,
                car.tire,
                track_evolution=self.snapshot.environment.initial_track_evolution,
            ).time_s
            for segment in self.segments
        )
        segment_index, fraction = self._segment_location(car.race_distance_m)
        car.segment_index = segment_index
        car.segment_elapsed_s = car.segment_durations_s[segment_index] * fraction

    def _refresh_tire_state(self, car: _ProgressCar) -> None:
        if not self.enable_pit or self._is_terminal(car):
            return
        wear_laps = max(
            0.0,
            (car.race_distance_m - car.stint_start_distance_m) / self.lap_length_m,
        )
        car.stint_lap = 0 if wear_laps <= 1e-9 else floor(wear_laps) + 1
        temperature_band = "hot" if wear_laps >= 18.0 else "optimal"
        car.tire = TireConditionSnapshot(
            physical_compound=car.tire.physical_compound,
            tire_role=car.tire.tire_role,
            wear_laps=wear_laps,
            temperature_band=temperature_band,
        )

    def _schedule_pit_stops(self, now: float) -> None:
        if not self.enable_pit:
            return
        for driver_id in tuple(self._race_order):
            car = self._cars[driver_id]
            if (
                self._is_terminal(car)
                or car.pit_state != "none"
                or car.incident_state is not None
                or car.traffic_state in {"attack", "side_by_side", "defend", "clearance"}
                or car.pit_plan_index >= len(car.pit_plans)
            ):
                continue
            plan = car.pit_plans[car.pit_plan_index]
            pit_entry_progress, pit_exit_progress = self._pit_track_progresses()
            pit_entry_distance_m = (
                plan.stop_lap - 1 + pit_entry_progress
            ) * self.lap_length_m
            if car.race_distance_m < pit_entry_distance_m:
                continue
            car.traffic_state = "clear"
            car.target_driver_id = None
            car.pit_state = "entry"
            car.pit_command_pending = False
            car.pit_request_role = None
            car.pit_request_compound = None
            car.pit_command_backup_plans = ()
            car.pit_phase_durations_s = self._pit_phase_durations(plan)
            car.pit_phase_started_s = now
            car.pit_phase_until_s = now + car.pit_phase_durations_s[0]
            car.pit_lane_progress = 0.0
            car.pit_route_start_distance_m = car.race_distance_m
            pit_route_lap_fraction = (pit_exit_progress - pit_entry_progress) % 1.0
            car.pit_route_target_distance_m = min(
                self.finish_distance_m,
                car.race_distance_m
                + self.lap_length_m * max(
                    PROGRESS_PIT_ROUTE_LAP_FRACTION,
                    pit_route_lap_fraction,
                ),
            )
            self.metrics["pit_requested_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:pit:{driver_id}:{plan.stop_lap}:requested",
                    event_type="pit_requested",
                    logical_time_s=now,
                    driver_ids=(driver_id,),
                    payload=(
                        ("pit_stop_lap", plan.stop_lap),
                        ("replacement_compound", plan.replacement_compound),
                        ("replacement_role", plan.replacement_role),
                    ),
                )
            )

    def _advance_pit_phases(self, now: float) -> None:
        for driver_id in tuple(self._race_order):
            car = self._cars[driver_id]
            if (
                self._is_terminal(car)
                or car.pit_state == "none"
                or now < car.pit_phase_until_s - 1e-9
            ):
                continue
            plan = car.pit_plans[car.pit_plan_index]
            entry_s, lane_s, service_s, exit_s = car.pit_phase_durations_s
            if car.pit_state == "entry":
                car.pit_state = "lane"
                car.pit_phase_started_s = now
                car.pit_phase_until_s = now + lane_s
                car.pit_lane_progress = 0.12
                event_type = "pit_lane_entry"
                suffix = "lane"
                payload = (("pit_stop_lap", plan.stop_lap),)
            elif car.pit_state == "lane":
                previous_compound = car.tire.physical_compound
                car.pit_state = "stop"
                car.pit_phase_started_s = now
                car.pit_phase_until_s = now + service_s
                car.pit_lane_progress = 0.50
                car.tire = TireConditionSnapshot(
                    physical_compound=plan.replacement_compound,
                    tire_role=plan.replacement_role,
                    wear_laps=0.0,
                    temperature_band="optimal",
                )
                car.stint_start_distance_m = car.race_distance_m
                car.stint_lap = 0
                self._recompute_segment_durations(car)
                event_type = "pit_stop_started"
                suffix = "service"
                payload = (
                    ("from_compound", previous_compound),
                    ("service_time_s", service_s),
                    ("to_compound", plan.replacement_compound),
                )
            elif car.pit_state == "stop":
                car.pit_state = "exit"
                car.pit_phase_started_s = now
                car.pit_phase_until_s = now + exit_s
                car.pit_lane_progress = 0.50
                event_type = "pit_stop_completed"
                suffix = "completed"
                payload = (("service_time_s", service_s),)
                self.metrics["pit_stop_completed_count"] += 1
            else:
                car.pit_state = "none"
                car.pit_phase_started_s = now
                car.pit_phase_until_s = now
                car.pit_lane_progress = 0.0
                car.pit_stop_count += 1
                car.pit_plan_index += 1
                car.pit_merge_until_s = now + PROGRESS_DECISION_INTERVAL_S
                event_type = "pit_exit"
                suffix = "exit"
                payload = (
                    ("pit_stop_count", car.pit_stop_count),
                    ("physical_compound", car.tire.physical_compound),
                )
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:pit:{driver_id}:{plan.stop_lap}:{suffix}",
                    event_type=event_type,
                    logical_time_s=now,
                    driver_ids=(driver_id,),
                    payload=payload,
                )
            )

    def _commit_pit_order_changes(self, now: float) -> None:
        changed = True
        while changed:
            changed = False
            for index in range(1, len(self._race_order)):
                ahead = self._cars[self._race_order[index - 1]]
                behind = self._cars[self._race_order[index]]
                if (
                    self._is_terminal(ahead)
                    or self._is_terminal(behind)
                    or behind.race_distance_m <= ahead.race_distance_m + 1e-9
                    or (
                        not self._is_pit_order_active(ahead, now)
                        and not self._is_pit_order_active(behind, now)
                    )
                    or self._pair_is_busy(ahead, behind)
                ):
                    continue
                self._race_order[index - 1], self._race_order[index] = (
                    self._race_order[index],
                    self._race_order[index - 1],
                )
                changed = True
                self.metrics["pit_order_change_count"] += 1
                pit_driver_ids = tuple(
                    car.entry.driver_id
                    for car in (ahead, behind)
                    if self._is_pit_order_active(car, now)
                )
                self.logical_events.append(
                    LogicalEvent(
                        event_id=(
                            f"progress:pit-order:{int(now / PROGRESS_DECISION_INTERVAL_S)}:"
                            f"{self.metrics['pit_order_change_count']}"
                        ),
                        event_type="pit_order_changed",
                        logical_time_s=now,
                        driver_ids=(ahead.entry.driver_id, behind.entry.driver_id),
                        payload=(
                            ("passed_driver_id", ahead.entry.driver_id),
                            ("passing_driver_id", behind.entry.driver_id),
                            ("pit_driver_ids", pit_driver_ids),
                        ),
                    )
                )
                break

    def _commit_incident_order_changes(self, now: float) -> None:
        """Commit passes caused by a slowed incident car as explicit events."""

        if self.race_control_state != "green":
            return
        changed = True
        while changed:
            changed = False
            for index in range(1, len(self._race_order)):
                ahead = self._cars[self._race_order[index - 1]]
                behind = self._cars[self._race_order[index]]
                if (
                    self._is_terminal(ahead)
                    or self._is_terminal(behind)
                    or behind.race_distance_m <= ahead.race_distance_m + 1e-9
                    or ahead.incident_state is None
                ):
                    continue
                self._race_order[index - 1], self._race_order[index] = (
                    self._race_order[index],
                    self._race_order[index - 1],
                )
                changed = True
                self.metrics["incident_order_change_count"] += 1
                self.logical_events.append(
                    LogicalEvent(
                        event_id=(
                            f"progress:incident-order:"
                            f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:"
                            f"{self.metrics['incident_order_change_count']}"
                        ),
                        event_type="incident_order_changed",
                        logical_time_s=now,
                        driver_ids=(ahead.entry.driver_id, behind.entry.driver_id),
                        payload=(
                            ("incident_driver_id", ahead.entry.driver_id),
                            ("passed_driver_id", ahead.entry.driver_id),
                            ("passing_driver_id", behind.entry.driver_id),
                        ),
                    )
                )
                break

    def _constrain_motion_to_logical_order(
        self,
        car: _ProgressCar,
        previous_distance_m: float,
    ) -> None:
        """Prevent a public integration step from creating an unapproved pass."""

        if not self.enable_traffic:
            return
        if self._is_terminal(car):
            return
        order_index = self._race_order.index(car.entry.driver_id)
        if order_index == 0:
            return
        ahead = self._cars[self._race_order[order_index - 1]]
        if self._is_terminal(ahead):
            return
        if self._is_pit_order_active(car) or self._is_pit_order_active(ahead):
            return
        successful_crossing = (
            car.target_driver_id == ahead.entry.driver_id
            and car.traffic_state == "side_by_side"
            and car.maneuver_success
        )
        if successful_crossing:
            if order_index < 2:
                return
            outer_ahead = self._cars[self._race_order[order_index - 2]]
            if self._is_terminal(outer_ahead):
                return
            maximum_distance_m = (
                outer_ahead.race_distance_m - MIN_LOGICAL_GAP_M
            )
        else:
            maximum_distance_m = ahead.race_distance_m - MIN_LOGICAL_GAP_M
            target = self._cars.get(car.target_driver_id)
            reserving_clearance = (
                car.traffic_state == "defend"
                and target is not None
                and target.traffic_state == "side_by_side"
                and target.maneuver_success
                and target.target_driver_id == car.entry.driver_id
            )
            if reserving_clearance:
                maximum_distance_m -= MIN_LOGICAL_GAP_M
        if car.race_distance_m <= maximum_distance_m + 1e-9:
            return
        car.race_distance_m = max(previous_distance_m, maximum_distance_m)
        car.finish_time_s = None
        segment_index, fraction = self._segment_location(car.race_distance_m)
        car.segment_index = segment_index
        car.segment_elapsed_s = car.segment_durations_s[segment_index] * fraction

    def _canonicalize_decision_state(self) -> None:
        """Quantize only at fixed authority boundaries, never external ticks."""

        for car in self._cars.values():
            if self._is_terminal(car):
                continue
            car.race_distance_m = round(car.race_distance_m, 9)
            segment_index, fraction = self._segment_location(car.race_distance_m)
            car.segment_index = segment_index
            car.segment_elapsed_s = car.segment_durations_s[segment_index] * fraction

    def _clear_pair(self, first: _ProgressCar, second: _ProgressCar, now: float) -> None:
        for car in (first, second):
            car.traffic_state = "clear"
            car.target_driver_id = None
            car.state_started_s = now
            car.state_until_s = now
            car.maneuver_success = False
            car.maneuver_side = 0
            car.attack_mode = "none"

    def _advance_maneuvers(self, now: float) -> None:
        if self.race_control_state != "green":
            return
        attackers = [
            car
            for car in self._cars.values()
            if car.traffic_state in {"attack", "side_by_side", "clearance"}
            and car.target_driver_id in self._cars
            and not self._is_terminal(car)
        ]
        for attacker in sorted(attackers, key=lambda car: str(car.entry.driver_id)):
            if attacker.traffic_state not in {"attack", "side_by_side", "clearance"}:
                continue
            defender = self._cars[attacker.target_driver_id]
            if self._is_terminal(defender):
                phase_at_abort = attacker.traffic_state
                if phase_at_abort != "clearance":
                    self.metrics["attack_failed_count"] += 1
                    self.logical_events.append(
                        LogicalEvent(
                            event_id=(
                                f"progress:attack:{attacker.entry.driver_id}:"
                                f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:target-terminal"
                            ),
                            event_type="attack_failed",
                            logical_time_s=now,
                            driver_ids=(
                                attacker.entry.driver_id,
                                defender.entry.driver_id,
                            ),
                            payload=(
                                ("phase_at_abort", phase_at_abort),
                                ("reason", "target_terminal"),
                            ),
                        )
                    )
                self._clear_pair(attacker, defender, now)
                continue
            if now < attacker.state_until_s - 1e-9:
                continue
            if attacker.traffic_state == "attack":
                attacker.traffic_state = "side_by_side"
                defender.traffic_state = "defend"
                attacker.state_started_s = now
                defender.state_started_s = now
                attacker.state_until_s = now + 2.0
                defender.state_until_s = now + 2.0
                self.logical_events.append(
                    LogicalEvent(
                        event_id=f"progress:attack:{attacker.entry.driver_id}:{int(now / PROGRESS_DECISION_INTERVAL_S)}:side",
                        event_type="attack_side_by_side",
                        logical_time_s=now,
                        driver_ids=(attacker.entry.driver_id, defender.entry.driver_id),
                    )
                )
                continue
            if attacker.traffic_state == "side_by_side":
                if attacker.maneuver_success:
                    self._set_race_distance(
                        attacker,
                        max(
                            attacker.race_distance_m,
                            defender.race_distance_m + MIN_LOGICAL_GAP_M,
                        ),
                    )
                    attacker_index = self._race_order.index(attacker.entry.driver_id)
                    defender_index = self._race_order.index(defender.entry.driver_id)
                    if attacker_index == defender_index + 1:
                        self._race_order[defender_index], self._race_order[attacker_index] = (
                            self._race_order[attacker_index],
                            self._race_order[defender_index],
                        )
                    elif attacker_index < defender_index:
                        # An incident affecting the defender may have already
                        # committed this crossing (and possibly other passes)
                        # as explicit incident-order events.
                        pass
                    else:
                        self.metrics["implicit_rank_swap_count"] += 1
                    attacker.traffic_state = "clearance"
                    defender.traffic_state = "defend"
                    attacker.state_started_s = now
                    defender.state_started_s = now
                    attacker.state_until_s = now + 1.5
                    defender.state_until_s = now + 1.5
                    self.metrics["overtake_completed_count"] += 1
                    self.logical_events.append(
                        LogicalEvent(
                            event_id=f"progress:attack:{attacker.entry.driver_id}:{int(now / PROGRESS_DECISION_INTERVAL_S)}:complete",
                            event_type="overtake_completed",
                            logical_time_s=now,
                            driver_ids=(attacker.entry.driver_id, defender.entry.driver_id),
                            payload=(("rank_swap", "logical_clearance"),),
                        )
                    )
                else:
                    self.metrics["attack_failed_count"] += 1
                    self.logical_events.append(
                        LogicalEvent(
                            event_id=f"progress:attack:{attacker.entry.driver_id}:{int(now / PROGRESS_DECISION_INTERVAL_S)}:failed",
                            event_type="attack_failed",
                            logical_time_s=now,
                            driver_ids=(attacker.entry.driver_id, defender.entry.driver_id),
                        )
                    )
                    attacker.cooldown_until_s = now + 15.0
                    defender.cooldown_until_s = now + 6.0
                    self._clear_pair(attacker, defender, now)
                continue
            attacker.cooldown_until_s = now + 20.0
            defender.cooldown_until_s = now + 8.0
            self._clear_pair(attacker, defender, now)

    def _pair_is_busy(self, first: _ProgressCar, second: _ProgressCar) -> bool:
        return first.traffic_state in {
            "attack",
            "side_by_side",
            "clearance",
            "defend",
        } or second.traffic_state in {
            "attack",
            "side_by_side",
            "clearance",
            "defend",
        }

    def _schedule_traffic(self, now: float) -> None:
        if (
            not self.enable_traffic
            or self.race_control_state != "green"
            or now < self._restart_attack_cooldown_until_s - 1e-9
        ):
            return
        for car in self._cars.values():
            if car.traffic_state == "follow":
                car.traffic_state = "clear"
                car.target_driver_id = None

        for position in range(1, len(self._race_order)):
            defender = self._cars[self._race_order[position - 1]]
            attacker = self._cars[self._race_order[position]]
            if (
                self._is_terminal(defender)
                or self._is_terminal(attacker)
                or self._is_pit_order_active(defender, now)
                or self._is_pit_order_active(attacker, now)
            ):
                continue
            gap_m = defender.race_distance_m - attacker.race_distance_m
            if self._pair_is_busy(attacker, defender):
                continue
            if gap_m <= FOLLOW_WINDOW_M:
                attacker.traffic_state = "follow"
                attacker.target_driver_id = defender.entry.driver_id
            if (
                now < self.grid_hold_duration_s + ATTACKS_ENABLED_AFTER_START_S
                or now < attacker.cooldown_until_s - 1e-9
                or now < defender.cooldown_until_s - 1e-9
                or not ATTACK_WINDOW_MIN_M <= gap_m <= ATTACK_WINDOW_MAX_M
                or self.segments[attacker.segment_index].segment_type
                not in ATTACK_SEGMENT_TYPES
                or attacker.incident_state is not None
                or defender.incident_state is not None
            ):
                continue
            closing_potential = self._base_speed_mps(attacker) - self._base_speed_mps(defender)
            skill_delta = attacker.entry.driver.overtaking - defender.entry.driver.defending
            if closing_potential <= 0.10 and skill_delta <= 0.02:
                continue
            stream = self.rng.stream(
                f"driver:{attacker.entry.driver_id}:progress_racecraft"
            )
            readiness = min(
                0.55,
                max(
                    0.08,
                    0.22
                    + closing_potential / 80.0
                    + skill_delta * 0.20
                    + attacker.tow_strength * 0.08
                    + (0.12 if attacker.drs_active else 0.0)
                    + (0.04 if attacker.drs_train_size >= 3 else 0.0)
                    - attacker.dirty_air_strength * 0.08
                    - self.snapshot.track.overtaking_difficulty * 0.15,
                ),
            )
            if stream.random() >= readiness:
                attacker.cooldown_until_s = now + 4.0
                continue
            success_probability = min(
                0.65,
                max(
                    0.18,
                    0.42
                    + closing_potential / 60.0
                    + skill_delta * 0.35
                    + attacker.tow_strength * 0.10
                    + (0.16 if attacker.drs_active else 0.0)
                    + (0.05 if attacker.drs_train_size >= 3 else 0.0)
                    - attacker.dirty_air_strength * 0.12
                    - self.snapshot.track.overtaking_difficulty * 0.25,
                ),
            )
            defender_index = self._race_order.index(defender.entry.driver_id)
            room_ahead_m = float("inf")
            if defender_index > 0:
                car_ahead = self._cars[self._race_order[defender_index - 1]]
                room_ahead_m = car_ahead.race_distance_m - defender.race_distance_m
            success = (
                room_ahead_m >= MIN_LOGICAL_GAP_M * 2.0
                and stream.random() < success_probability
            )
            attacker.traffic_state = "attack"
            attacker.target_driver_id = defender.entry.driver_id
            attacker.state_started_s = now
            attacker.state_until_s = now + 1.5
            attacker.maneuver_success = success
            attacker.attack_mode = (
                "drs"
                if attacker.drs_active
                else "tow" if attacker.tow_strength >= 0.10 else "braking"
            )
            attacker.maneuver_side = -1 if stream.random() < 0.5 else 1
            defender.traffic_state = "defend"
            defender.target_driver_id = attacker.entry.driver_id
            defender.state_started_s = now
            defender.state_until_s = now + 1.5
            defender.maneuver_success = False
            defender.attack_mode = "defend"
            defender.maneuver_side = -attacker.maneuver_side
            self.metrics["attack_started_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:attack:{attacker.entry.driver_id}:{int(now / PROGRESS_DECISION_INTERVAL_S)}:started",
                    event_type="attack_started",
                    logical_time_s=now,
                    driver_ids=(attacker.entry.driver_id, defender.entry.driver_id),
                    payload=(
                        ("attack_mode", attacker.attack_mode),
                        ("maneuver_side", attacker.maneuver_side),
                        ("planned_success", success),
                    ),
                )
            )

    def _abort_maneuvers_for_caution(self, now: float) -> None:
        attackers = sorted(
            (
                car
                for car in self._cars.values()
                if car.traffic_state in {"attack", "side_by_side", "clearance"}
                and car.target_driver_id in self._cars
                and not self._is_terminal(car)
            ),
            key=lambda car: str(car.entry.driver_id),
        )
        for attacker in attackers:
            if attacker.traffic_state not in {"attack", "side_by_side", "clearance"}:
                continue
            defender = self._cars[attacker.target_driver_id]
            phase_at_abort = attacker.traffic_state
            if phase_at_abort == "clearance":
                self._clear_pair(attacker, defender, now)
                continue
            attacker_index = self._race_order.index(attacker.entry.driver_id)
            defender_index = self._race_order.index(defender.entry.driver_id)
            crossed_before_caution = (
                attacker_index == defender_index + 1
                and attacker.race_distance_m > defender.race_distance_m + 1e-9
            )
            if crossed_before_caution:
                self._race_order[defender_index], self._race_order[attacker_index] = (
                    self._race_order[attacker_index],
                    self._race_order[defender_index],
                )
                self.metrics["overtake_completed_count"] += 1
                self.logical_events.append(
                    LogicalEvent(
                        event_id=(
                            f"progress:attack:{attacker.entry.driver_id}:"
                            f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:caution-crossing"
                        ),
                        event_type="overtake_completed",
                        logical_time_s=now,
                        driver_ids=(
                            attacker.entry.driver_id,
                            defender.entry.driver_id,
                        ),
                        payload=(("rank_swap", "crossing_before_caution"),),
                    )
                )
                self._clear_pair(attacker, defender, now)
                continue
            self.metrics["attack_failed_count"] += 1
            self.metrics["caution_suppressed_attack_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=(
                        f"progress:attack:{attacker.entry.driver_id}:"
                        f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:caution"
                    ),
                    event_type="attack_failed",
                    logical_time_s=now,
                    driver_ids=(attacker.entry.driver_id, defender.entry.driver_id),
                    payload=(
                        ("phase_at_abort", phase_at_abort),
                        ("reason", "race_control_caution"),
                    ),
                )
            )
            attacker.cooldown_until_s = max(attacker.cooldown_until_s, now + 15.0)
            defender.cooldown_until_s = max(defender.cooldown_until_s, now + 8.0)
            self._clear_pair(attacker, defender, now)

    def _close_maneuvers_at_race_finish(self, now: float) -> None:
        # The integration tick may end after the last authoritative terminal
        # timestamp when a coarse presentation step is used.  Race-finish
        # events must therefore use the result boundary, not the outer tick,
        # so their IDs and timestamps remain invariant across tick sizes.
        terminal_times = [
            time_s
            for car in self._cars.values()
            for time_s in (car.finish_time_s, car.retirement_time_s)
            if time_s is not None
        ]
        if terminal_times:
            now = round(max(terminal_times), 9)
        attackers = sorted(
            (
                car
                for car in self._cars.values()
                if car.traffic_state in {"attack", "side_by_side", "clearance"}
                and car.target_driver_id in self._cars
            ),
            key=lambda car: str(car.entry.driver_id),
        )
        for attacker in attackers:
            if attacker.traffic_state not in {"attack", "side_by_side", "clearance"}:
                continue
            defender = self._cars[attacker.target_driver_id]
            phase_at_abort = attacker.traffic_state
            if phase_at_abort != "clearance":
                self.metrics["attack_failed_count"] += 1
                self.logical_events.append(
                    LogicalEvent(
                        event_id=(
                            f"progress:attack:{attacker.entry.driver_id}:"
                            f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:race-finish"
                        ),
                        event_type="attack_failed",
                        logical_time_s=now,
                        driver_ids=(
                            attacker.entry.driver_id,
                            defender.entry.driver_id,
                        ),
                        payload=(
                            ("phase_at_abort", phase_at_abort),
                            ("reason", "race_finished"),
                        ),
                    )
                )
            self._clear_pair(attacker, defender, now)

    def _active_sc_queue(self) -> list[_ProgressCar]:
        return [
            self._cars[driver_id]
            for driver_id in self._race_order
            if not self._is_terminal(self._cars[driver_id])
            and self._cars[driver_id].pit_state == "none"
        ]

    def _refresh_sc_lap_deficits(self) -> None:
        queue = self._active_sc_queue()
        if not queue:
            return
        leader_distance_m = queue[0].race_distance_m
        active_ids = {car.entry.driver_id for car in queue}
        self._sc_lap_deficit = {
            driver_id: deficit
            for driver_id, deficit in self._sc_lap_deficit.items()
            if driver_id in active_ids
        }
        for car in queue:
            driver_id = car.entry.driver_id
            if driver_id in self._sc_lap_deficit:
                continue
            distance_behind_m = max(0.0, leader_distance_m - car.race_distance_m)
            self._sc_lap_deficit[driver_id] = int(
                floor(distance_behind_m / self.lap_length_m + 1e-9)
            )

    def _sc_queue_target_distance(
        self,
        car: _ProgressCar,
        *,
        queue: Sequence[_ProgressCar] | None = None,
    ) -> float:
        active_queue = list(queue) if queue is not None else self._active_sc_queue()
        if not active_queue:
            return car.race_distance_m
        index = next(
            index
            for index, candidate in enumerate(active_queue)
            if candidate.entry.driver_id == car.entry.driver_id
        )
        leader = active_queue[0]
        lap_deficit = self._sc_lap_deficit.get(car.entry.driver_id, 0)
        return (
            leader.race_distance_m
            - lap_deficit * self.lap_length_m
            - index * SC_QUEUE_TARGET_GAP_M
        )

    def _sc_queue_is_formed(self) -> bool:
        self._refresh_sc_lap_deficits()
        queue = self._active_sc_queue()
        if len(queue) < 2:
            return True
        return all(
            max(0.0, self._sc_queue_target_distance(car, queue=queue) - car.race_distance_m)
            <= SC_QUEUE_FORMED_TOLERANCE_M
            for car in queue[1:]
        )

    def _set_race_control_phase(
        self,
        *,
        state: str,
        phase: str,
        now: float,
        duration_s: float | None,
    ) -> None:
        self.race_control_state = state
        self.race_control_phase = phase
        self._race_control_phase_started_s = now
        self._race_control_phase_until_s = (
            -1.0 if duration_s is None else round(now + duration_s, 9)
        )

    def _emit_local_yellow(
        self,
        *,
        now: float,
        cause: str,
        driver_ids: Sequence[int | str],
        duration_s: float,
    ) -> None:
        self.metrics["local_yellow_count"] += 1
        self.logical_events.append(
            LogicalEvent(
                event_id=(
                    f"progress:local-yellow:{self.metrics['local_yellow_count']}:"
                    f"{int(now / PROGRESS_DECISION_INTERVAL_S)}"
                ),
                event_type="local_yellow_started",
                logical_time_s=now,
                driver_ids=tuple(driver_ids),
                payload=(
                    ("cause", cause),
                    ("duration_s", duration_s),
                ),
            )
        )

    def _request_race_control(
        self,
        kind: str,
        *,
        now: float,
        cause: str,
        driver_ids: Sequence[int | str] = (),
        severity: float = 0.0,
    ) -> None:
        if kind not in {"vsc", "sc"}:
            raise ValueError("race control request must be vsc or sc")
        if self._finished or all(self._is_terminal(car) for car in self._cars.values()):
            return
        if self.race_control_state == "sc":
            return
        if self.race_control_state == "vsc" and kind == "vsc":
            return

        self._abort_maneuvers_for_caution(now)
        self._clear_drs_state()
        if kind == "vsc":
            self._set_race_control_phase(
                state="vsc",
                phase="active",
                now=now,
                duration_s=VSC_DURATION_S,
            )
            self.metrics["vsc_period_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:vsc:{self.metrics['vsc_period_count']}:started",
                    event_type="vsc_started",
                    logical_time_s=now,
                    driver_ids=tuple(driver_ids),
                    payload=(
                        ("cause", cause),
                        ("severity", round(severity, 9)),
                    ),
                )
            )
            return

        upgraded_from_vsc = self.race_control_state == "vsc"
        self._set_race_control_phase(
            state="sc",
            phase="deploying",
            now=now,
            duration_s=SC_DEPLOY_DURATION_S,
        )
        self._sc_lap_deficit = {}
        self._refresh_sc_lap_deficits()
        self.metrics["safety_car_period_count"] += 1
        self.logical_events.append(
            LogicalEvent(
                event_id=(
                    f"progress:sc:{self.metrics['safety_car_period_count']}:deployed"
                ),
                event_type="safety_car_deployed",
                logical_time_s=now,
                driver_ids=tuple(driver_ids),
                payload=(
                    ("cause", cause),
                    ("severity", round(severity, 9)),
                    ("upgraded_from_vsc", upgraded_from_vsc),
                ),
            )
        )

    def _advance_race_control(self, now: float) -> None:
        if self.race_control_state == "green":
            return
        if self.race_control_state == "vsc":
            if now < self._race_control_phase_until_s - 1e-9:
                return
            period = self.metrics["vsc_period_count"]
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:vsc:{period}:ended",
                    event_type="vsc_ended",
                    logical_time_s=now,
                )
            )
            self._set_race_control_phase(
                state="green", phase="green", now=now, duration_s=None
            )
            self._restart_attack_cooldown_until_s = now + RESTART_ATTACK_COOLDOWN_S
            return

        period = self.metrics["safety_car_period_count"]
        if self.race_control_phase == "deploying":
            if now < self._race_control_phase_until_s - 1e-9:
                return
            self._set_race_control_phase(
                state="sc", phase="catch_up", now=now, duration_s=None
            )
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:sc:{period}:catch-up",
                    event_type="safety_car_catch_up_started",
                    logical_time_s=now,
                )
            )

        if self.race_control_phase == "catch_up":
            if not self._sc_queue_is_formed():
                return
            self._set_race_control_phase(
                state="sc",
                phase="queue_formed",
                now=now,
                duration_s=SC_QUEUE_HOLD_DURATION_S,
            )
            self.metrics["safety_car_queue_formed_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:sc:{period}:queue-formed",
                    event_type="safety_car_queue_formed",
                    logical_time_s=now,
                    payload=(("queue_size", len(self._active_sc_queue())),),
                )
            )
            return

        if now < self._race_control_phase_until_s - 1e-9:
            return
        if self.race_control_phase == "queue_formed":
            self._set_race_control_phase(
                state="sc",
                phase="restart_ready",
                now=now,
                duration_s=SC_RESTART_READY_DURATION_S,
            )
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:sc:{period}:restart-ready",
                    event_type="safety_car_restart_ready",
                    logical_time_s=now,
                )
            )
            return
        if self.race_control_phase == "restart_ready":
            self._set_race_control_phase(
                state="sc",
                phase="in_this_lap",
                now=now,
                duration_s=SC_IN_THIS_LAP_DURATION_S,
            )
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:sc:{period}:in-this-lap",
                    event_type="safety_car_in_this_lap",
                    logical_time_s=now,
                )
            )
            return
        if self.race_control_phase == "in_this_lap":
            self._set_race_control_phase(
                state="green", phase="green", now=now, duration_s=None
            )
            self._restart_attack_cooldown_until_s = now + RESTART_ATTACK_COOLDOWN_S
            leader = self._cars[self._race_order[0]]
            self._drs_enable_after_leader_distance_m = (
                leader.race_distance_m
                + DRS_SC_RESTART_DISABLED_LAPS * self.lap_length_m
            )
            self._sc_lap_deficit = {}
            self.metrics["restart_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:sc:{period}:restarted",
                    event_type="race_restarted",
                    logical_time_s=now,
                    payload=(("control", "leader_release"),),
                )
            )

    def _race_control_pace_multiplier(
        self,
        car: _ProgressCar,
        normal_multiplier: float,
    ) -> float:
        if self.race_control_state == "green":
            return normal_multiplier
        if self.race_control_state == "vsc":
            return min(normal_multiplier, 0.65)
        if car.pit_state != "none" or self._is_terminal(car):
            return normal_multiplier

        self._refresh_sc_lap_deficits()
        queue = self._active_sc_queue()
        if not queue:
            return normal_multiplier
        base_speed_mps = self._base_speed_mps(car)
        leader = queue[0]
        leader_target_speed_mps = min(45.0, self._base_speed_mps(leader) * 0.48)
        if car.entry.driver_id == leader.entry.driver_id:
            desired_speed_mps = leader_target_speed_mps
        else:
            target_distance_m = self._sc_queue_target_distance(car, queue=queue)
            closing_distance_m = max(0.0, target_distance_m - car.race_distance_m)
            desired_speed_mps = leader_target_speed_mps + min(
                18.0, closing_distance_m * 0.08
            )
        desired_multiplier = desired_speed_mps / max(base_speed_mps, 1e-9)
        return max(0.15, min(normal_multiplier, desired_multiplier))

    def _retire_car(self, car: _ProgressCar, *, now: float, reason: str) -> None:
        if self._is_terminal(car):
            return
        target = self._cars.get(car.target_driver_id)
        paired_attacker = None
        paired_defender = None
        if car.traffic_state in {"attack", "side_by_side"} and target:
            paired_attacker, paired_defender = car, target
        elif (
            target is not None
            and target.traffic_state in {"attack", "side_by_side"}
            and target.target_driver_id == car.entry.driver_id
        ):
            paired_attacker, paired_defender = target, car
        if paired_attacker is not None and paired_defender is not None:
            phase_at_abort = paired_attacker.traffic_state
            self.metrics["attack_failed_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=(
                        f"progress:attack:{paired_attacker.entry.driver_id}:"
                        f"{int(now / PROGRESS_DECISION_INTERVAL_S)}:retirement"
                    ),
                    event_type="attack_failed",
                    logical_time_s=now,
                    driver_ids=(
                        paired_attacker.entry.driver_id,
                        paired_defender.entry.driver_id,
                    ),
                    payload=(
                        ("phase_at_abort", phase_at_abort),
                        ("reason", "driver_retired"),
                    ),
                )
            )
            self._clear_pair(paired_attacker, paired_defender, now)
        elif target is not None and target.target_driver_id == car.entry.driver_id:
            self._clear_pair(car, target, now)
        car.traffic_state = "clear"
        car.target_driver_id = None
        car.maneuver_success = False
        car.pit_state = "none"
        car.pit_lane_progress = 0.0
        car.pit_merge_until_s = -1.0
        car.incident_state = "terminal_damage"
        car.incident_until_s = now
        car.retirement_time_s = round(now, 6)
        car.retirement_reason = reason
        self.metrics["retirement_count"] += 1
        self._race_order = [
            driver_id
            for driver_id in self._race_order
            if self._cars[driver_id].retirement_time_s is None
        ] + sorted(
            (
                driver_id
                for driver_id in self._race_order
                if self._cars[driver_id].retirement_time_s is not None
            ),
            key=lambda driver_id: (
                -self._cars[driver_id].race_distance_m,
                float(self._cars[driver_id].retirement_time_s),
                self._cars[driver_id].grid_index,
            ),
        )
        self.logical_events.append(
            LogicalEvent(
                event_id=f"progress:retirement:{car.entry.driver_id}:{int(now / PROGRESS_DECISION_INTERVAL_S)}",
                event_type="driver_retired",
                logical_time_s=now,
                driver_ids=(car.entry.driver_id,),
                payload=(
                    ("damage_level", round(car.damage_level, 9)),
                    ("race_distance_m", round(car.race_distance_m, 6)),
                    ("reason", reason),
                ),
            )
        )
        self._request_race_control(
            "sc",
            now=now,
            cause=reason,
            driver_ids=(car.entry.driver_id,),
            severity=car.damage_level,
        )

    def _apply_pair_contact(
        self,
        first: _ProgressCar,
        second: _ProgressCar,
        *,
        severity: float,
        now: float,
    ) -> None:
        if self._is_terminal(first) or self._is_terminal(second):
            return
        severity = min(1.0, max(0.0, severity))
        first.damage_level = min(1.0, first.damage_level + 0.05 + severity * 0.25)
        second.damage_level = min(1.0, second.damage_level + 0.03 + severity * 0.18)
        duration_s = 1.0 + round(severity * 6.0) * 0.5
        for car in (first, second):
            car.incident_state = "contact"
            car.incident_until_s = now + duration_s
            car.maneuver_success = False
        self.metrics["incident_count"] += 1
        self.metrics["contact_count"] += 1
        self.metrics["pair_contact_count"] += 1
        pair = tuple(sorted((first.entry.driver_id, second.entry.driver_id), key=str))
        self.logical_events.append(
            LogicalEvent(
                event_id=(
                    f"progress:contact:{pair[0]}:{pair[1]}:"
                    f"{int(now / PROGRESS_DECISION_INTERVAL_S)}"
                ),
                event_type="contact_started",
                logical_time_s=now,
                driver_ids=pair,
                payload=(
                    ("duration_s", duration_s),
                    ("first_driver_id", first.entry.driver_id),
                    ("first_damage_level", round(first.damage_level, 9)),
                    ("second_driver_id", second.entry.driver_id),
                    ("second_damage_level", round(second.damage_level, 9)),
                    ("severity", round(severity, 9)),
                ),
            )
        )
        for car in (first, second):
            reliability = car.entry.vehicle.performance.reliability
            retirement_probability = min(
                0.90,
                max(
                    0.0,
                    (car.damage_level - 0.22) * 1.50
                    + max(0.0, severity - 0.70) * 0.50
                    + max(0.0, 1.0 - reliability) * 0.20,
                ),
            )
            stream = self.rng.vehicle_reliability(car.entry.vehicle_id)
            if car.damage_level >= 0.75 or stream.random() < retirement_probability:
                self._retire_car(car, now=now, reason="contact_damage")
        requested_control = (
            "sc"
            if severity >= SC_CONTACT_SEVERITY_THRESHOLD
            else "vsc" if severity >= VSC_CONTACT_SEVERITY_THRESHOLD else None
        )
        if requested_control is not None:
            self._request_race_control(
                requested_control,
                now=now,
                cause="pair_contact",
                driver_ids=(first.entry.driver_id, second.entry.driver_id),
                severity=severity,
            )
        elif self.race_control_state == "green":
            self._emit_local_yellow(
                now=now,
                cause="pair_contact",
                driver_ids=(first.entry.driver_id, second.entry.driver_id),
                duration_s=duration_s,
            )

    def _evaluate_pair_contacts(self, now: float) -> None:
        if not self.enable_incidents:
            return
        seen_pairs: set[tuple[str, str]] = set()
        attackers = sorted(
            (
                car
                for car in self._cars.values()
                if car.traffic_state == "side_by_side"
                and car.target_driver_id in self._cars
                and not self._is_terminal(car)
            ),
            key=lambda car: str(car.entry.driver_id),
        )
        for attacker in attackers:
            if (
                self.race_control_state != "green"
                or attacker.traffic_state != "side_by_side"
                or attacker.target_driver_id not in self._cars
            ):
                continue
            defender = self._cars[attacker.target_driver_id]
            pair_key = tuple(
                sorted(
                    (str(attacker.entry.driver_id), str(defender.entry.driver_id))
                )
            )
            if (
                pair_key in seen_pairs
                or self._is_terminal(defender)
                or attacker.pit_state != "none"
                or defender.pit_state != "none"
                or attacker.incident_state is not None
                or defender.incident_state is not None
            ):
                continue
            seen_pairs.add(pair_key)
            risk_factor = (
                1.0
                + self.snapshot.track.overtaking_difficulty * 0.50
                + max(0.0, 1.0 - attacker.entry.driver.consistency)
                + max(0.0, 1.0 - defender.entry.driver.consistency)
            )
            probability = 1.0 - exp(
                -0.008 * risk_factor * PROGRESS_DECISION_INTERVAL_S
            )
            stream = self.rng.driver_contact(attacker.entry.driver_id)
            if stream.random() >= probability:
                continue
            self._apply_pair_contact(
                attacker,
                defender,
                severity=stream.random(),
                now=now,
            )

    def _incident_rate_per_s(self, car: _ProgressCar) -> float:
        segment_type = self.segments[car.segment_index].segment_type
        base_rate = {
            "straight": 0.000025,
            "heavy_braking": 0.000420,
            "traction": 0.000120,
            "sweeping": 0.000090,
            "technical": 0.000150,
        }.get(segment_type, 0.000120)
        consistency = car.entry.driver.consistency
        traffic_factor = {
            "clear": 1.0,
            "follow": 1.20,
            "attack": 1.65,
            "side_by_side": 2.20,
            "defend": 1.55,
            "clearance": 1.25,
        }.get(car.traffic_state, 1.0)
        weather_factor = 2.0 if self.snapshot.environment.weather != "dry" else 1.0
        tire_factor = {
            "cold": 1.25,
            "optimal": 1.0,
            "hot": 1.35,
        }.get(car.tire.temperature_band, 1.0)
        proximity_factor = 1.0
        if car.target_driver_id in self._cars:
            target = self._cars[car.target_driver_id]
            gap_m = abs(target.race_distance_m - car.race_distance_m)
            proximity_factor += max(0.0, 1.0 - gap_m / FOLLOW_WINDOW_M) * 0.80
        return (
            base_rate
            * (1.0 + max(0.0, 1.0 - consistency) * 2.5)
            * traffic_factor
            * weather_factor
            * tire_factor
            * proximity_factor
        )

    def _evaluate_incidents(self, now: float) -> None:
        for driver_id in self._race_order:
            car = self._cars[driver_id]
            if self._is_terminal(car) or car.pit_state != "none":
                continue
            if car.incident_state is not None:
                if now >= car.incident_until_s - 1e-9:
                    car.incident_state = None
                    car.incident_until_s = now
                else:
                    continue
            if not self.enable_incidents:
                continue
            stream = self.rng.driver_incident(driver_id)
            rate = self._incident_rate_per_s(car)
            probability = 1.0 - exp(-rate * PROGRESS_DECISION_INTERVAL_S)
            if stream.random() >= probability:
                continue
            kind_value = stream.random()
            segment_type = self.segments[car.segment_index].segment_type
            if car.traffic_state in {"side_by_side", "attack", "defend"} and kind_value < 0.30:
                kind = "contact"
            elif segment_type == "heavy_braking" and kind_value < 0.70:
                kind = "lockup"
            elif kind_value < 0.82:
                kind = "run_wide"
            else:
                kind = "spin"
            severity = stream.random()
            if kind == "contact":
                target = self._cars.get(car.target_driver_id)
                if (
                    target is not None
                    and not self._is_terminal(target)
                    and target.pit_state == "none"
                ):
                    self._apply_pair_contact(
                        car,
                        target,
                        severity=severity,
                        now=now,
                    )
                    continue
                kind = "run_wide"
            duration_s = {
                "lockup": 0.5 + round(severity * 2.0) * 0.5,
                "run_wide": 1.0 + round(severity * 4.0) * 0.5,
                "spin": 3.0 + round(severity * 8.0) * 0.5,
                "contact": 1.0 + round(severity * 6.0) * 0.5,
            }[kind]
            car.incident_state = kind
            car.incident_until_s = now + duration_s
            if kind == "contact":
                car.damage_level = min(1.0, car.damage_level + 0.01 + severity * 0.04)
            self.metrics["incident_count"] += 1
            self.metrics[f"{kind}_count"] += 1
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:incident:{driver_id}:{int(now / PROGRESS_DECISION_INTERVAL_S)}",
                    event_type=f"{kind}_started",
                    logical_time_s=now,
                    driver_ids=(driver_id,),
                    payload=(
                        ("duration_s", duration_s),
                        ("hazard_probability", round(probability, 12)),
                        ("severity", round(severity, 9)),
                    ),
                )
            )
            if kind == "spin" and severity >= VSC_SPIN_SEVERITY_THRESHOLD:
                self._request_race_control(
                    "vsc",
                    now=now,
                    cause="spin",
                    driver_ids=(driver_id,),
                    severity=severity,
                )
            elif kind == "run_wide" and severity >= VSC_RUN_WIDE_SEVERITY_THRESHOLD:
                self._request_race_control(
                    "vsc",
                    now=now,
                    cause="run_wide",
                    driver_ids=(driver_id,),
                    severity=severity,
                )
            elif kind == "spin" or (kind == "run_wide" and severity >= 0.70):
                self._emit_local_yellow(
                    now=now,
                    cause=kind,
                    driver_ids=(driver_id,),
                    duration_s=duration_s,
                )

    def _audit_logical_order(self) -> None:
        if not self.enable_traffic:
            return
        for ahead_id, behind_id in zip(self._race_order, self._race_order[1:]):
            ahead = self._cars[ahead_id]
            behind = self._cars[behind_id]
            if self._is_terminal(ahead) or self._is_terminal(behind):
                continue
            if self._is_pit_order_active(ahead) or self._is_pit_order_active(behind):
                continue
            if behind.race_distance_m <= ahead.race_distance_m + 1e-9:
                continue
            active_crossing = (
                behind.target_driver_id == ahead_id
                and behind.traffic_state in {"side_by_side", "clearance"}
            )
            if not active_crossing:
                self.metrics["implicit_rank_swap_count"] += 1

    def _enforce_logical_spacing(self) -> None:
        """Keep rank authority without ever moving a published car backwards."""

        if not self.enable_traffic:
            return
        pairs = tuple(zip(self._race_order, self._race_order[1:]))
        for ahead_id, behind_id in reversed(pairs):
            ahead = self._cars[ahead_id]
            behind = self._cars[behind_id]
            if self._is_terminal(ahead) or self._is_terminal(behind):
                continue
            if self._is_pit_order_active(ahead) or self._is_pit_order_active(behind):
                continue
            successful_crossing = (
                behind.target_driver_id == ahead_id
                and behind.traffic_state == "side_by_side"
                and behind.maneuver_success
            )
            if successful_crossing:
                continue
            maximum_distance_m = ahead.race_distance_m - MIN_LOGICAL_GAP_M
            if behind.race_distance_m <= maximum_distance_m + 1e-9:
                continue
            # A rank-changing event can put a newly ordered pair closer than
            # the public logical gap. Pulling the following car backwards
            # makes the broadcast visibly rewind. Preserve both published
            # distances and propagate the required clearance forward instead.
            self._set_race_distance(
                ahead,
                behind.race_distance_m + MIN_LOGICAL_GAP_M,
            )
            self.metrics["spacing_constraint_count"] += 1

    def _refresh_pace_multipliers(self) -> None:
        for car in self._cars.values():
            normal_multiplier = self._pace_multiplier(car)
            car.active_pace_multiplier = self._race_control_pace_multiplier(
                car, normal_multiplier
            )
            if car.traffic_state == "follow":
                self.metrics["spacing_constraint_count"] += 1

    def _emit_new_finish_events(self) -> None:
        for car in sorted(self._cars.values(), key=lambda item: str(item.entry.driver_id)):
            driver_id = car.entry.driver_id
            if car.finish_time_s is None or driver_id in self._finish_event_driver_ids:
                continue
            self._finish_event_driver_ids.add(driver_id)
            self.logical_events.append(
                LogicalEvent(
                    event_id=f"progress:finish:{driver_id}",
                    event_type="driver_finished",
                    logical_time_s=car.finish_time_s,
                    driver_ids=(driver_id,),
                    payload=(("finish_time_s", car.finish_time_s),),
                )
            )

    def _process_decision_boundary(self, now: float) -> None:
        self._canonicalize_decision_state()
        self._emit_new_finish_events()
        self._advance_race_control(now)
        self._advance_pit_phases(now)
        self._commit_pit_order_changes(now)
        self._advance_maneuvers(now)
        self._commit_incident_order_changes(now)
        self._enforce_logical_spacing()
        self._schedule_pit_stops(now)
        self._update_racecraft_context()
        self._update_drs_trains()
        self._schedule_traffic(now)
        self._update_maneuver_groups(now)
        for car in self._cars.values():
            self._refresh_tire_state(car)
        self._evaluate_pair_contacts(now)
        self._evaluate_incidents(now)
        self._audit_logical_order()
        self._refresh_pace_multipliers()
        for car in self._cars.values():
            car.decision_distance_m = car.race_distance_m

    def _ordered_cars(self) -> list[_ProgressCar]:
        return [self._cars[driver_id] for driver_id in self._race_order]

    def _build_tick(self) -> ProgressRaceTick:
        ordered = self._ordered_cars()
        leader = ordered[0]
        vehicles: list[ProgressVehicleState] = []
        for position, car in enumerate(ordered, start=1):
            ahead = ordered[position - 2] if position > 1 else None
            duration_s = car.segment_durations_s[car.segment_index]
            logical_speed_mps = self._logical_speed_mps(car)
            gap_m = (
                0.0
                if ahead is None
                else max(0.0, ahead.race_distance_m - car.race_distance_m)
            )
            gap_to_leader_s, timing_gap_valid = (
                (0.0, True)
                if position == 1
                else self._live_timing_gap_seconds_between(leader, car)
            )
            interval_to_ahead_s, interval_timing_gap_valid = (
                (0.0, True)
                if ahead is None
                else self._live_timing_gap_seconds_between(ahead, car)
            )
            segment_fraction = min(
                1.0,
                max(0.0, car.segment_elapsed_s / max(duration_s, 1e-12)),
            )
            total_progress = car.race_distance_m / self.lap_length_m
            maneuver_progress = (
                min(
                    1.0,
                    max(
                        0.0,
                        (self.logical_time_s - car.state_started_s)
                        / max(car.state_until_s - car.state_started_s, 1e-9),
                    ),
                )
                if car.traffic_state
                in {"attack", "side_by_side", "defend", "clearance"}
                else 0.0
            )
            vehicles.append(
                ProgressVehicleState(
                    driver_id=car.entry.driver_id,
                    vehicle_id=car.entry.vehicle_id,
                    position=position,
                    race_distance_m=car.race_distance_m,
                    total_progress=total_progress,
                    lap_number=(
                        self.total_laps
                        if car.finish_time_s is not None
                        else min(
                            self.total_laps,
                            max(1, floor(max(0.0, total_progress)) + 1),
                        )
                    ),
                    segment_id=self.segments[car.segment_index].segment_id,
                    segment_fraction=segment_fraction,
                    logical_speed_mps=logical_speed_mps,
                    gap_to_ahead_m=gap_m,
                    gap_to_leader_s=gap_to_leader_s,
                    interval_to_ahead_s=interval_to_ahead_s,
                    timing_gap_valid=timing_gap_valid,
                    interval_timing_gap_valid=interval_timing_gap_valid,
                    ahead_driver_id=None if ahead is None else ahead.entry.driver_id,
                    traffic_state=car.traffic_state,
                    target_driver_id=car.target_driver_id,
                    drs_active=car.drs_active,
                    dirty_air_active=car.dirty_air_active,
                    dirty_air_strength=car.dirty_air_strength,
                    tow_strength=car.tow_strength,
                    attack_mode=car.attack_mode,
                    maneuver_side=car.maneuver_side,
                    maneuver_progress=maneuver_progress,
                    drs_train_id=car.drs_train_id,
                    drs_train_size=car.drs_train_size,
                    drs_train_position=car.drs_train_position,
                    drs_train_member_ids=car.drs_train_member_ids,
                    maneuver_group_id=car.maneuver_group_id,
                    maneuver_group_size=car.maneuver_group_size,
                    maneuver_group_member_ids=car.maneuver_group_member_ids,
                    maneuver_group_phase=car.maneuver_group_phase,
                    maneuver_group_corridor_index=(
                        car.maneuver_group_corridor_index
                    ),
                    incident_state=car.incident_state,
                    damage_level=car.damage_level,
                    physical_compound=car.tire.physical_compound,
                    tire_role=car.tire.tire_role,
                    stint_lap=car.stint_lap,
                    wear_laps=car.tire.wear_laps,
                    temperature_band=car.tire.temperature_band,
                    pit_state=car.pit_state,
                    pit_lane_progress=car.pit_lane_progress,
                    pit_stop_count=car.pit_stop_count,
                    pit_request_pending=car.pit_command_pending,
                    pit_request_role=car.pit_request_role,
                    pit_request_compound=car.pit_request_compound,
                    pace_mode=car.pace_mode,
                    race_status=(
                        "retired"
                        if car.retirement_time_s is not None
                        else "finished" if car.finish_time_s is not None else "running"
                    ),
                    retirement_reason=car.retirement_reason,
                    retirement_time_s=car.retirement_time_s,
                    finished=car.finish_time_s is not None,
                    finish_time_s=car.finish_time_s,
                )
            )
        return ProgressRaceTick(
            tick_index=self.tick_index,
            logical_time_s=self.logical_time_s,
            vehicles=tuple(vehicles),
            race_control_state=self.race_control_state,
            race_control_phase=self.race_control_phase,
        )

    def advance_one_tick(self) -> ProgressRaceTick:
        if self._finished:
            raise RuntimeError("progress race is already finished")
        tick_end_s = (self.tick_index + 1) * self.tick_seconds
        while self._authority_time_s < tick_end_s - 1e-12:
            chunk_end_s = min(
                tick_end_s,
                self._next_decision_time_s,
                self._next_motion_boundary_s,
            )
            motion_start_s = max(self._authority_time_s, self.grid_hold_duration_s)
            motion_duration_s = max(0.0, chunk_end_s - motion_start_s)
            previous_distances_m = {
                driver_id: car.race_distance_m
                for driver_id, car in self._cars.items()
            }
            for driver_id in sorted(self._cars, key=str):
                car = self._cars[driver_id]
                if car.pit_state == "none":
                    self._advance_car(
                        car,
                        motion_duration_s,
                        motion_start_s,
                        car.active_pace_multiplier,
                    )
                else:
                    self._advance_pit_car(
                        car,
                        motion_duration_s,
                        motion_start_s,
                    )
            self._authority_time_s = chunk_end_s
            if abs(self._authority_time_s - self._next_motion_boundary_s) <= 1e-9:
                for driver_id in tuple(self._race_order):
                    self._constrain_motion_to_logical_order(
                        self._cars[driver_id],
                        previous_distances_m[driver_id],
                    )
                self._next_motion_boundary_s = round(
                    self._next_motion_boundary_s + PROGRESS_MOTION_INTERVAL_S,
                    9,
                )
            for driver_id, car in self._cars.items():
                self._record_timing_loop_crossings(
                    car,
                    previous_distances_m[driver_id],
                    car.race_distance_m,
                    motion_start_s,
                    motion_duration_s,
                )
                self._update_drs_detection_eligibility(
                    car,
                    previous_distances_m[driver_id],
                )
            if abs(self._authority_time_s - self._next_decision_time_s) <= 1e-9:
                self._process_decision_boundary(self._authority_time_s)
                self._next_decision_time_s = round(
                    self._next_decision_time_s + PROGRESS_DECISION_INTERVAL_S,
                    9,
                )
        self.tick_index += 1
        self.logical_time_s = tick_end_s
        self._finished = all(
            self._is_terminal(car) for car in self._cars.values()
        )
        if self._finished:
            self._emit_new_finish_events()
            self._close_maneuvers_at_race_finish(tick_end_s)
        self._current_tick = self._build_tick()
        return self._current_tick

    def run_to_finish(self, *, max_ticks: int | None = None) -> ProgressRaceSummary:
        limit = max_ticks
        if limit is None:
            slowest_lap_s = max(sum(car.segment_durations_s) for car in self._cars.values())
            planned_stop_count = max(
                (len(car.pit_plans) for car in self._cars.values()),
                default=0,
            )
            limit = int(
                (
                    self.grid_hold_duration_s
                    + slowest_lap_s
                    * (self.total_laps + 12.0 + planned_stop_count)
                )
                / self.tick_seconds
            ) + 2
        while not self._finished:
            if self.tick_index >= limit:
                raise RuntimeError("progress race exceeded max_ticks")
            self.advance_one_tick()

        ordered = sorted(
            self._cars.values(),
            key=lambda car: (
                1 if car.retirement_time_s is not None else 0,
                (
                    -car.race_distance_m
                    if car.retirement_time_s is not None
                    else float(car.finish_time_s)
                ),
                (
                    float(car.retirement_time_s)
                    if car.retirement_time_s is not None
                    else 0.0
                ),
                car.grid_index,
            ),
        )
        classification = tuple(
            ProgressFinishResult(
                position=position,
                driver_id=car.entry.driver_id,
                status="retired" if car.retirement_time_s is not None else "finished",
                race_distance_m=car.race_distance_m,
                finish_time_s=car.finish_time_s,
                retirement_time_s=car.retirement_time_s,
                retirement_reason=car.retirement_reason,
            )
            for position, car in enumerate(ordered, start=1)
        )
        return ProgressRaceSummary(
            snapshot_hash=self.snapshot.snapshot_hash,
            total_laps=self.total_laps,
            grid_order=self.grid_order,
            classification=classification,
            logical_duration_s=max(
                (
                    item.finish_time_s
                    if item.finish_time_s is not None
                    else float(item.retirement_time_s)
                )
                for item in classification
            ),
            integration_tick_count=self.tick_index,
            logical_events=tuple(self.logical_events),
            metrics=tuple(self.metrics.items()),
        )


def run_progress_race(
    snapshot: AbstractSessionSnapshot,
    *,
    grid_order: tuple[int | str, ...] | list[int | str] | None = None,
    total_laps: int = 10,
    tick_seconds: float = DEFAULT_PROGRESS_TICK_SECONDS,
    enable_traffic: bool = True,
    enable_incidents: bool = True,
    enable_pit: bool = True,
    pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
) -> ProgressRaceSummary:
    """Run the experimental progress engine without selecting it as default."""

    return ProgressRaceCursor(
        snapshot,
        grid_order=grid_order,
        total_laps=total_laps,
        tick_seconds=tick_seconds,
        enable_traffic=enable_traffic,
        enable_incidents=enable_incidents,
        enable_pit=enable_pit,
        pit_strategy=pit_strategy,
    ).run_to_finish()
