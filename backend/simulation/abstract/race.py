"""Stage D deterministic abstract race with traffic, incidents and pit stops."""

from __future__ import annotations

import copy
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from math import ceil, floor, sqrt
from typing import Any, Callable

from .incidents import assess_contact, assess_lockup
from .pit import PIT_PHASES, PitStopPlan, build_pit_plans
from .performance import SegmentRequirement
from .pose import SingleVehiclePoseSynthesizer
from .kinematics import (
    GRID_HOLD_DURATION_S,
    GRID_LATERAL_HOLD_AFTER_LIGHTS_S,
    GRID_LATERAL_REJOIN_DURATION_S,
    GRID_LAUNCH_FOLLOWING_BLEND_DURATION_S,
    GRID_LAUNCH_MIN_LONGITUDINAL_GAP_M,
    KINEMATIC_LOGICAL_TICK_SECONDS,
    KinematicStep,
    LateralTrajectory,
    LongitudinalKinematicPlan,
    RacingLineDistanceContract,
    build_longitudinal_plan,
)
from .racecraft import (
    BODY_CLEARANCE_MARGIN_M,
    MIN_LONGITUDINAL_GAP_M,
    TrafficCorridorReservation,
    accepted_swept_distance_m,
    audit_accepted_frame_reservations,
    audit_reservation_intersections,
    local_corridor_assessment,
    swept_vehicle_intersects_reservation,
)
from .rng import IndependentRNG
from .state import (
    AbstractCommandRecord,
    AbstractEntrySnapshot,
    AbstractRaceFrame,
    AbstractRaceResult,
    AbstractSessionSnapshot,
    AbstractTimingCheckpoint,
    LogicalEvent,
    RaceVehicleState,
    TireConditionSnapshot,
)


# Provisional Stage 4 presentation guard.  The curve/chord diagnostic leaves
# a small margin below the 370 km/h contract for world-space sampling error;
# this is not an official F1 speed or vehicle-performance value.
TRAFFIC_MAX_SPEED_MPS = 98.0
MANEUVER_DEADLINE_S = 4.0
REJOIN_DURATION_S = 1.5
# Extra fallback margin keeps the immutable reservation alive while an
# aborted maneuver waits for a safe longitudinal gap before rejoining.
RESERVATION_HORIZON_S = MANEUVER_DEADLINE_S + REJOIN_DURATION_S + 2.5


@dataclass
class _MutableCar:
    entry: AbstractEntrySnapshot
    total_progress: float
    tire: TireConditionSnapshot
    pit_plans: tuple[PitStopPlan, ...] = ()
    pit_plan_index: int = 0
    pit_state: str = "none"
    pit_phase_ticks_remaining: int = 0
    pit_phase_total_ticks: int = 0
    pit_lane_progress: float = 0.0
    pit_service_time_s: float = 0.0
    pit_stop_count: int = 0
    pit_command_pending: bool = False
    pit_command_backup: PitStopPlan | None = None
    pit_request_role: str | None = None
    pit_request_compound: str | None = None
    stint_lap: int = 0
    damage_level: float = 0.0
    incident_state: str | None = None
    incident_remaining_ticks: int = 0
    incident_cooldown_ticks: int = 0
    last_lockup_segment_id: str | None = None
    lateral_offset_m: float = 0.0
    visual_state: str = "normal"
    maneuver: str = "normal"
    corridor_id: str | None = None
    corridor_side: str | None = None
    line_distance_m: float = 0.0
    speed_mps: float = 0.0
    last_step: KinematicStep | None = None
    kinematic_plan: LongitudinalKinematicPlan | None = None
    pace_multiplier: float = 1.0
    pace_mode: str = "STANDARD"
    lateral_trajectory: LateralTrajectory | None = None
    lateral_velocity_mps: float = 0.0
    lateral_acceleration_mps2: float = 0.0


@dataclass
class _TrafficManeuver:
    """Mutable runtime phase for one immutable two-wide reservation."""

    attacker_id: int | str
    defender_id: int | str
    corridor_id: str
    reservation: TrafficCorridorReservation
    start_time_s: float
    deadline_time_s: float
    phase: str
    attacker_target_offset_m: float
    defender_target_offset_m: float
    attacker_trajectory: LateralTrajectory
    defender_trajectory: LateralTrajectory
    defended: bool = False
    contact_checked: bool = False
    crossing_confirmed: bool = False
    clearance_confirmed: bool = False
    aborted: bool = False
    forced_outcome: str | None = None
    force_contact: bool = False


@dataclass(frozen=True, slots=True)
class AbstractTrafficTick:
    """One resumable traffic transition and its bounded output."""

    frame: AbstractRaceFrame
    logical_events: tuple[LogicalEvent, ...]
    checkpoints: tuple[AbstractTimingCheckpoint, ...]
    active_reservations: tuple[TrafficCorridorReservation, ...] = ()


@dataclass(frozen=True, slots=True)
class AbstractCursorRuntimeCheckpoint:
    """Bounded mutable authority required to rewind unpublished future ticks."""

    tick_index: int
    logical_time_s: float
    cars: dict[int | str, _MutableCar]
    order: tuple[int | str, ...]
    pace_multiplier: dict[int | str, float]
    last_checkpoint_laps: dict[int | str, int]
    pair_next_attack_eligible_time_s: dict[tuple[int | str, int | str], float]
    next_attack_eligible_time_s: dict[int | str, float]
    maneuvers: dict[str, _TrafficManeuver]
    terminal_corridor_ids: set[str]
    metrics: dict[str, Any]
    rng_states: dict[str, object]
    event_count: int
    timing_checkpoint_count: int
    finish_order_count: int
    command_count: int
    finished: bool


class AbstractRaceEngine:
    """Abstract race engine with logical traffic, incidents and pit operations."""

    def __init__(self, snapshot: AbstractSessionSnapshot):
        self.snapshot = snapshot
        self.pose_synthesizer = SingleVehiclePoseSynthesizer(snapshot)

    def run(
        self,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = 0.10,
        max_ticks: int | None = None,
        pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
        frame_sink: Callable[[AbstractRaceFrame], None] | None = None,
    ) -> AbstractRaceResult:
        cursor = AbstractTrafficSimulationCursor(
            self.snapshot,
            grid_order=grid_order,
            total_laps=total_laps,
            tick_seconds=tick_seconds,
            max_ticks=max_ticks,
            pit_strategy=pit_strategy,
        )
        if frame_sink is not None:
            frame_sink(cursor.current_frame)
        try:
            while not cursor.finished:
                tick = cursor.advance_one_tick()
                if frame_sink is not None:
                    frame_sink(tick.frame)
            return cursor.finalize()
        finally:
            cursor.dispose()

    race = run

    @staticmethod
    def _speed_factor(car: _MutableCar) -> float:
        factor = 1.0
        if car.pit_state != "none":
            factor *= 0.18
        if car.incident_state == "lockup":
            factor *= 0.55
        elif car.incident_state == "contact":
            factor *= 0.90
        factor *= max(0.86, 1.0 - car.damage_level * 0.08)
        return factor

    def _segment_at_progress(self, progress: float) -> SegmentRequirement:
        normalized = progress % 1.0
        segments = self.snapshot.track.segments
        for index, segment in enumerate(segments):
            # Progress values at a segment boundary can be a few ulps below
            # the authored decimal after a lap wrap (e.g. 1.099 % 1.0).
            # Treat that value as the next segment so callers never re-enter
            # the preceding segment with zero remaining distance.
            if abs(normalized - segment.end_progress) < 1e-10:
                return segments[(index + 1) % len(segments)]
            if segment.start_progress <= normalized < segment.end_progress:
                return segment
        return segments[-1]

    def _update_tire_state(self, car: _MutableCar, raw_delta: float, total_laps: int) -> None:
        wear_laps = car.tire.wear_laps + max(raw_delta, 0.0)
        segment = self._segment_at_progress(car.total_progress)
        if wear_laps >= 2.0 or (segment.segment_type in {"sweeping", "heavy_braking"} and wear_laps >= 1.0):
            temperature_band = "hot"
        else:
            temperature_band = "optimal"
        car.tire = replace(
            car.tire,
            wear_laps=wear_laps,
            temperature_band=temperature_band,
        )
        car.stint_lap = max(0, min(total_laps, int(max(car.total_progress, 0.0)) + 1))

    def _advance_pit_phases(
        self,
        *,
        logical_time_s: float,
        cars: dict[int | str, _MutableCar],
        tick_seconds: float,
        events: list[LogicalEvent],
    ) -> None:
        for driver_id in sorted(cars, key=str):
            car = cars[driver_id]
            if car.pit_state == "none":
                continue
            car.pit_phase_ticks_remaining -= 1
            phase_index = PIT_PHASES.index(car.pit_state)
            phase_fraction = (
                1.0 - car.pit_phase_ticks_remaining / max(car.pit_phase_total_ticks, 1)
            )
            car.pit_lane_progress = min(
                0.99,
                (phase_index + max(0.0, phase_fraction)) / len(PIT_PHASES),
            )
            if car.pit_phase_ticks_remaining > 0:
                continue
            plan = car.pit_plans[car.pit_plan_index]
            if car.pit_state == "entry":
                car.pit_state = "lane"
                car.pit_phase_ticks_remaining = 4
                car.pit_phase_total_ticks = 4
                events.append(
                    LogicalEvent(
                        event_id=f"pit:{driver_id}:{plan.stop_lap}:lane",
                        event_type="pit_lane_entry",
                        logical_time_s=logical_time_s,
                        driver_ids=(driver_id,),
                        payload=(("pit_stop_lap", plan.stop_lap),),
                    )
                )
            elif car.pit_state == "lane":
                car.pit_state = "stop"
                car.pit_phase_ticks_remaining = max(1, ceil(plan.service_time_s / tick_seconds))
                car.pit_phase_total_ticks = car.pit_phase_ticks_remaining
                car.tire = replace(
                    car.tire,
                    physical_compound=plan.replacement_compound,
                    tire_role=plan.replacement_role,
                    wear_laps=0.0,
                    temperature_band="optimal",
                )
                events.append(
                    LogicalEvent(
                        event_id=f"pit:{driver_id}:{plan.stop_lap}:service",
                        event_type="pit_stop_started",
                        logical_time_s=logical_time_s,
                        driver_ids=(driver_id,),
                        payload=(
                            ("from_compound", car.entry.tire.physical_compound),
                            ("service_time_s", plan.service_time_s),
                            ("to_compound", plan.replacement_compound),
                        ),
                    )
                )
            elif car.pit_state == "stop":
                car.pit_state = "exit"
                car.pit_phase_ticks_remaining = 3
                car.pit_phase_total_ticks = 3
                events.append(
                    LogicalEvent(
                        event_id=f"pit:{driver_id}:{plan.stop_lap}:completed",
                        event_type="pit_stop_completed",
                        logical_time_s=logical_time_s,
                        driver_ids=(driver_id,),
                        payload=(("service_time_s", plan.service_time_s),),
                    )
                )
            else:
                car.pit_state = "none"
                car.pit_phase_ticks_remaining = 0
                car.pit_phase_total_ticks = 0
                car.pit_lane_progress = 0.0
                car.pit_stop_count += 1
                car.pit_plan_index += 1
                events.append(
                    LogicalEvent(
                        event_id=f"pit:{driver_id}:{plan.stop_lap}:exit",
                        event_type="pit_exit",
                        logical_time_s=logical_time_s,
                        driver_ids=(driver_id,),
                        payload=(("pit_stop_count", car.pit_stop_count),),
                    )
                )

    def _advance_incident_states(
        self,
        *,
        logical_time_s: float,
        cars: dict[int | str, _MutableCar],
        events: list[LogicalEvent],
    ) -> None:
        for driver_id in sorted(cars, key=str):
            car = cars[driver_id]
            car.incident_cooldown_ticks = max(0, car.incident_cooldown_ticks - 1)
            if car.incident_state is None:
                continue
            car.incident_remaining_ticks -= 1
            if car.incident_remaining_ticks > 0:
                continue
            incident = car.incident_state
            car.incident_state = None
            if car.corridor_id is None and car.pit_state == "none":
                car.lateral_offset_m = 0.0
                car.visual_state = "normal"
                car.maneuver = "normal"
            events.append(
                LogicalEvent(
                    event_id=f"incident:{incident}:{driver_id}:ended:{logical_time_s:.3f}",
                    event_type=f"{incident}_ended",
                    logical_time_s=logical_time_s,
                    driver_ids=(driver_id,),
                )
            )

    def _schedule_pit_stops(
        self,
        *,
        tick: int,
        logical_time_s: float,
        cars: dict[int | str, _MutableCar],
        events: list[LogicalEvent],
    ) -> None:
        for driver_id in sorted(cars, key=str):
            car = cars[driver_id]
            if car.pit_state != "none" or car.incident_state is not None:
                continue
            if car.pit_plan_index >= len(car.pit_plans):
                continue
            plan = car.pit_plans[car.pit_plan_index]
            if car.total_progress < plan.stop_lap:
                continue
            car.pit_state = "entry"
            car.pit_phase_ticks_remaining = 2
            car.pit_phase_total_ticks = 2
            car.pit_lane_progress = 0.0
            car.pit_service_time_s = plan.service_time_s
            car.pit_command_pending = False
            car.pit_command_backup = None
            car.pit_request_role = None
            car.pit_request_compound = None
            events.append(
                LogicalEvent(
                    event_id=f"pit:{driver_id}:{plan.stop_lap}:requested",
                    event_type="pit_requested",
                    logical_time_s=logical_time_s,
                    driver_ids=(driver_id,),
                    payload=(
                        ("pit_stop_lap", plan.stop_lap),
                        ("tick", tick),
                    ),
                )
            )

    def _evaluate_lockups(
        self,
        *,
        tick: int,
        logical_time_s: float,
        cars: dict[int | str, _MutableCar],
        incident_streams: Mapping[int | str, Any],
        events: list[LogicalEvent],
    ) -> None:
        for driver_id in sorted(cars, key=str):
            car = cars[driver_id]
            segment = self._segment_at_progress(car.total_progress)
            check_key = f"{int(max(car.total_progress, 0.0))}:{segment.segment_id}"
            if car.last_lockup_segment_id == check_key:
                continue
            car.last_lockup_segment_id = check_key
            if (
                car.pit_state != "none"
                or car.incident_state is not None
                or car.incident_cooldown_ticks > 0
                or segment.segment_type != "heavy_braking"
            ):
                continue
            stream = incident_streams[driver_id]
            assessment = assess_lockup(
                segment_type=segment.segment_type,
                braking=car.entry.vehicle.performance.braking,
                tire=car.tire,
                driver_consistency=car.entry.driver.consistency,
                attack_mode=car.maneuver in {"overlap", "defend"},
                random_value=stream.random(),
                severity_value=stream.random(),
                axle_value=stream.random(),
                track_condition=self.snapshot.environment.weather,
            )
            if not assessment.triggered:
                continue
            car.incident_state = "lockup"
            car.incident_remaining_ticks = 3
            car.incident_cooldown_ticks = 20
            car.visual_state = "lockup"
            car.maneuver = "lockup"
            if assessment.run_wide:
                car.lateral_offset_m = 1.0
            car.tire = replace(
                car.tire,
                wear_laps=car.tire.wear_laps + 0.10,
                temperature_band="hot",
            )
            events.append(
                LogicalEvent(
                    event_id=f"incident:lockup:{driver_id}:{tick}",
                    event_type="lockup_started",
                    logical_time_s=logical_time_s,
                    driver_ids=(driver_id,),
                    payload=tuple(sorted(assessment.to_dict().items())),
                )
            )



class AbstractTrafficSimulationCursor:
    """Resumable, bounded twenty-car logical traffic simulation.

    Instant and broadcast both consume this cursor.  It owns only the current
    accepted state, active local corridor reservations and compact result
    authority; it never retains a race-wide pose array.  All distance updates
    use the Stage 3 compiled racing-line arc-length contract.
    """

    def __init__(
        self,
        snapshot: AbstractSessionSnapshot,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = KINEMATIC_LOGICAL_TICK_SECONDS,
        max_ticks: int | None = None,
        pit_strategy: Mapping[int | str, Sequence[int]] | None = None,
    ) -> None:
        if total_laps < 1 or total_laps > 100:
            raise ValueError("total_laps must be in the range 1..100")
        if tick_seconds <= 0.0:
            raise ValueError("traffic tick_seconds must be positive")
        self.snapshot = snapshot
        self.total_laps = total_laps
        self.tick_seconds = tick_seconds
        self.kernel = AbstractRaceEngine(snapshot)
        self.synthesizer = SingleVehiclePoseSynthesizer(snapshot)
        self.distance_contract: RacingLineDistanceContract = self.synthesizer.distance_contract
        geometry = snapshot.track.display_geometry
        if geometry is None or len(geometry.grid_slots) < len(snapshot.entries):
            raise ValueError("traffic cursor requires shared grid display geometry")
        self.geometry = geometry
        self._lateral_limit_lut = self._build_lateral_limit_lut()
        entry_by_driver = snapshot.entry_by_driver_id()
        self.order = list(grid_order or snapshot.initial_grid_order)
        if set(self.order) != set(entry_by_driver) or len(self.order) != len(entry_by_driver):
            raise ValueError("grid_order must contain each snapshot driver exactly once")
        self.initial_grid_order = tuple(self.order)
        self.rng = IndependentRNG(snapshot.session_seed)
        pit_streams = {
            entry.driver_id: self.rng.team_pit(entry.team_id)
            for entry in snapshot.entries
        }
        pit_plans = build_pit_plans(
            snapshot.entries,
            total_laps=total_laps,
            pit_strategy=pit_strategy,
            pit_streams=pit_streams,
        )
        self.pace_multiplier = {
            entry.driver_id: 1.0 + self.rng.driver_pace(entry.driver_id).uniform(-0.0012, 0.0012)
            for entry in snapshot.entries
        }
        self.mistake_streams = {
            entry.driver_id: self.rng.driver_mistake(entry.driver_id)
            for entry in snapshot.entries
        }
        self.incident_streams = {
            entry.driver_id: self.rng.driver_incident(entry.driver_id)
            for entry in snapshot.entries
        }
        # The contact stream remains isolated for the existing logical event
        # contract.  Stage 4 does not add new contact mechanics.
        self.contact_streams = {
            entry.driver_id: self.rng.driver_contact(entry.driver_id)
            for entry in snapshot.entries
        }
        self.cars: dict[int | str, _MutableCar] = {}
        for position, driver_id in enumerate(self.order, start=1):
            entry = entry_by_driver[driver_id]
            slot = geometry.grid_slots[position - 1]
            line_distance = self.distance_contract.distance_at_total_progress(slot.progress)
            plan = build_longitudinal_plan(
                snapshot,
                driver_id,
                distance_contract=self.distance_contract,
            )
            hold_step = KinematicStep(
                distance_m=line_distance,
                speed_mps=0.0,
                longitudinal_acceleration_mps2=0.0,
                displacement_m=0.0,
                target_speed_mps=0.0,
                previous_distance_m=line_distance,
                previous_speed_mps=0.0,
                adjustment_reason="grid_hold",
            )
            limits = self.distance_contract.lateral_limits_at_progress(
                slot.progress,
                car_width_m=geometry.car_width_m,
            )
            self.cars[driver_id] = _MutableCar(
                entry=entry,
                total_progress=self.distance_contract.total_progress_at_distance(line_distance),
                tire=entry.tire,
                pit_plans=pit_plans[driver_id],
                line_distance_m=line_distance,
                last_step=hold_step,
                kinematic_plan=plan,
                pace_multiplier=self.pace_multiplier[driver_id],
                lateral_offset_m=slot.lateral_offset_m,
                lateral_trajectory=LateralTrajectory.create(
                    start_offset_m=slot.lateral_offset_m,
                    end_offset_m=0.0,
                    start_time_s=(
                        GRID_HOLD_DURATION_S + GRID_LATERAL_HOLD_AFTER_LIGHTS_S
                    ),
                    duration_s=GRID_LATERAL_REJOIN_DURATION_S,
                    target_limits_m=limits,
                ),
            )
        self.tick_index = 0
        self.logical_time_s = 0.0
        self.lights_out_tick = int(ceil(GRID_HOLD_DURATION_S / tick_seconds))
        self._standing_start_active = True
        self.max_ticks = (
            int((total_laps + 3.0) * snapshot.track.base_lap_time_s / tick_seconds * 3.0)
            if max_ticks is None
            else max_ticks
        )
        if self.max_ticks < 0:
            raise ValueError("max_ticks must not be negative")
        self.events: list[LogicalEvent] = [
            LogicalEvent(
                event_id="race:started",
                event_type="race_started",
                logical_time_s=0.0,
                payload=(
                    ("total_laps", total_laps),
                    ("driver_count", len(self.cars)),
                    ("source_mode", "abstract"),
                ),
            )
        ]
        self.command_log: list[AbstractCommandRecord] = []
        self.timing_checkpoints: list[AbstractTimingCheckpoint] = [
            self._checkpoint("race:start", "race_start", 0)
        ]
        self.finish_order: list[int | str] = []
        self.last_checkpoint_laps = {
            driver_id: int(max(0.0, floor(self.cars[driver_id].total_progress)))
            for driver_id in self.order
        }
        self.pair_next_attack_eligible_time_s: dict[tuple[int | str, int | str], float] = {}
        self.next_attack_eligible_time_s: dict[int | str, float] = {
            driver_id: 0.0 for driver_id in self.cars
        }
        self.maneuvers: dict[str, _TrafficManeuver] = {}
        self._forecast_target_cache: dict[tuple[int | str, float], float] = {}
        self._terminal_corridor_ids: set[str] = set()
        self._current_frame = self._build_frame()
        self._finished = False
        self._finalized = False
        self._disposed = False
        self.metrics: dict[str, int | float] = {
            "post_integrator_distance_correction_count": 0,
            "post_integrator_max_correction_m": 0.0,
            "world_chord_precommit_replan_count": 0,
            "crossing_before_rank_swap_count": 0,
            "body_overlap_count": 0,
            "corridor_conflict_count": 0,
            "admitted_reservation_conflict_count": 0,
            "third_vehicle_occupancy_conflict_count": 0,
            "reservation_body_clearance_violation_count": 0,
            "same_lane_longitudinal_overlap_count": 0,
            "minimum_body_clearance_m": float("inf"),
            "minimum_corridor_clearance_m": float("inf"),
            "attack_start_count": 0,
            "overtake_completed_count": 0,
            "attack_aborted_count": 0,
            "attack_terminal_event_count": 0,
            "attack_terminal_event_duplicate_count": 0,
            "cooldown_violation_count": 0,
            "cooldown_min_s": float("inf"),
            "third_vehicle_conflict_rejection_count": 0,
            "reservation_conflict_rejection_count": 0,
            "corridor_rejection_reason_counts": {},
        }

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def current_frame(self) -> AbstractRaceFrame:
        if self._disposed:
            raise RuntimeError("traffic cursor is disposed")
        return self._current_frame

    @property
    def current_tick(self) -> int:
        return self.tick_index

    @property
    def current_time_s(self) -> float:
        return self.logical_time_s

    @property
    def active_reservation_count(self) -> int:
        return len(self.maneuvers)

    @property
    def active_reservations(self) -> tuple[TrafficCorridorReservation, ...]:
        return tuple(
            self.maneuvers[corridor_id].reservation
            for corridor_id in sorted(self.maneuvers)
        )

    def export_runtime_checkpoint(self) -> AbstractCursorRuntimeCheckpoint:
        """Capture mutable authority without copying immutable track geometry."""

        if self._disposed:
            raise RuntimeError("traffic cursor is disposed")
        return AbstractCursorRuntimeCheckpoint(
            tick_index=self.tick_index,
            logical_time_s=self.logical_time_s,
            cars={driver_id: copy.copy(car) for driver_id, car in self.cars.items()},
            order=tuple(self.order),
            pace_multiplier=dict(self.pace_multiplier),
            last_checkpoint_laps=dict(self.last_checkpoint_laps),
            pair_next_attack_eligible_time_s=dict(self.pair_next_attack_eligible_time_s),
            next_attack_eligible_time_s=dict(self.next_attack_eligible_time_s),
            maneuvers={
                corridor_id: copy.copy(maneuver)
                for corridor_id, maneuver in self.maneuvers.items()
            },
            terminal_corridor_ids=set(self._terminal_corridor_ids),
            metrics=copy.deepcopy(self.metrics),
            rng_states=self.rng.export_states(),
            event_count=len(self.events),
            timing_checkpoint_count=len(self.timing_checkpoints),
            finish_order_count=len(self.finish_order),
            command_count=len(self.command_log),
            finished=self._finished,
        )

    def restore_runtime_checkpoint(
        self,
        checkpoint: AbstractCursorRuntimeCheckpoint,
    ) -> None:
        """Restore one checkpoint and discard every later speculative result."""

        if self._disposed:
            raise RuntimeError("traffic cursor is disposed")
        if checkpoint.tick_index > self.tick_index:
            raise ValueError("cannot restore a checkpoint from the future")
        self.tick_index = checkpoint.tick_index
        self.logical_time_s = checkpoint.logical_time_s
        self.cars = {
            driver_id: copy.copy(car)
            for driver_id, car in checkpoint.cars.items()
        }
        self.order = list(checkpoint.order)
        self.pace_multiplier = dict(checkpoint.pace_multiplier)
        self.last_checkpoint_laps = dict(checkpoint.last_checkpoint_laps)
        self.pair_next_attack_eligible_time_s = dict(
            checkpoint.pair_next_attack_eligible_time_s
        )
        self.next_attack_eligible_time_s = dict(checkpoint.next_attack_eligible_time_s)
        self.maneuvers = {
            corridor_id: copy.copy(maneuver)
            for corridor_id, maneuver in checkpoint.maneuvers.items()
        }
        self._terminal_corridor_ids = set(checkpoint.terminal_corridor_ids)
        self.metrics = copy.deepcopy(checkpoint.metrics)
        self.rng.restore_states(checkpoint.rng_states)
        del self.events[checkpoint.event_count:]
        del self.timing_checkpoints[checkpoint.timing_checkpoint_count:]
        del self.finish_order[checkpoint.finish_order_count:]
        del self.command_log[checkpoint.command_count:]
        self._forecast_target_cache.clear()
        self._finished = checkpoint.finished
        self._finalized = False
        self._current_frame = self._build_frame()

    def _record_interactive_command(
        self,
        command: str,
        payload: dict[str, Any],
        *,
        event_type: str,
        driver_id: int | str,
    ) -> LogicalEvent:
        sequence = len(self.command_log)
        record = AbstractCommandRecord(
            sequence=sequence,
            command=command,
            logical_time_s=self.logical_time_s,
            payload=payload,
        )
        self.command_log.append(record)
        event = LogicalEvent(
            event_id=f"command:{sequence}:{command}:{driver_id}",
            event_type=event_type,
            logical_time_s=self.logical_time_s + self.tick_seconds,
            driver_ids=(driver_id,),
            payload=tuple(sorted(payload.items())),
        )
        self.events.append(event)
        return event

    def set_pace_mode(self, driver_id: int | str, pace_mode: str) -> LogicalEvent:
        """Apply a bounded pit-wall pace request at the current logical tick."""

        normalized = str(pace_mode).upper()
        factors = {"CONSERVE": 0.985, "STANDARD": 1.0, "ATTACK": 1.015}
        if normalized not in factors:
            raise ValueError("pace mode must be CONSERVE, STANDARD or ATTACK")
        car = self.cars.get(driver_id)
        if car is None:
            raise ValueError("unknown driver")
        if car.total_progress >= self.total_laps:
            raise ValueError("driver has already finished")
        previous = car.pace_mode
        car.pace_mode = normalized
        car.pace_multiplier = self.pace_multiplier[driver_id] * factors[normalized]
        self._forecast_target_cache.clear()
        self._current_frame = self._build_frame()
        return self._record_interactive_command(
            "set_pace_mode",
            {
                "driver_id": driver_id,
                "from_pace_mode": previous,
                "pace_mode": normalized,
            },
            event_type="pace_mode_changed",
            driver_id=driver_id,
        )

    def request_pit(
        self,
        driver_id: int | str,
        *,
        tire_role: str,
        physical_compound: str,
    ) -> LogicalEvent:
        """Replace the driver's next automatic stop with a player pit call."""

        car = self.cars.get(driver_id)
        if car is None:
            raise ValueError("unknown driver")
        if car.total_progress >= self.total_laps:
            raise ValueError("driver has already finished")
        if car.pit_state != "none":
            raise ValueError("driver is already in the pit sequence")
        if car.pit_command_pending:
            raise ValueError("pit stop is already requested")
        stop_lap = int(floor(max(0.0, car.total_progress))) + 1
        if stop_lap >= self.total_laps:
            raise ValueError("too late to request a pit stop")
        backup = (
            car.pit_plans[car.pit_plan_index]
            if car.pit_plan_index < len(car.pit_plans)
            else None
        )
        service_time_s = (
            backup.service_time_s
            if backup is not None
            else self.rng.team_pit(car.entry.team_id).uniform(1.90, 2.30)
        )
        plan = PitStopPlan(
            driver_id=driver_id,
            stop_lap=stop_lap,
            replacement_compound=physical_compound,
            replacement_role=tire_role,
            service_time_s=service_time_s,
        )
        completed = car.pit_plans[:car.pit_plan_index]
        later = car.pit_plans[car.pit_plan_index + (1 if backup is not None else 0):]
        car.pit_plans = completed + (plan,) + later
        car.pit_command_pending = True
        car.pit_command_backup = backup
        car.pit_request_role = tire_role
        car.pit_request_compound = physical_compound
        self._current_frame = self._build_frame()
        return self._record_interactive_command(
            "pit_call",
            {
                "driver_id": driver_id,
                "physical_compound": physical_compound,
                "stop_lap": stop_lap,
                "tire_role": tire_role,
            },
            event_type="pit_call_registered",
            driver_id=driver_id,
        )

    def cancel_pit(self, driver_id: int | str) -> LogicalEvent:
        """Cancel a player request until the abstract pit sequence starts."""

        car = self.cars.get(driver_id)
        if car is None:
            raise ValueError("unknown driver")
        if not car.pit_command_pending or car.pit_state != "none":
            raise ValueError("no cancellable pit request")
        completed = car.pit_plans[:car.pit_plan_index]
        later = car.pit_plans[car.pit_plan_index + 1:]
        restored = (car.pit_command_backup,) if car.pit_command_backup is not None else ()
        car.pit_plans = completed + restored + later
        previous_role = car.pit_request_role
        previous_compound = car.pit_request_compound
        car.pit_command_pending = False
        car.pit_command_backup = None
        car.pit_request_role = None
        car.pit_request_compound = None
        self._current_frame = self._build_frame()
        return self._record_interactive_command(
            "pit_cancel",
            {
                "driver_id": driver_id,
                "physical_compound": previous_compound,
                "tire_role": previous_role,
            },
            event_type="pit_call_cancelled",
            driver_id=driver_id,
        )

    def _record_attack_terminal(self, corridor_id: str) -> None:
        """Count exactly one terminal outcome per admitted corridor."""

        if corridor_id in self._terminal_corridor_ids:
            self.metrics["attack_terminal_event_duplicate_count"] += 1
            return
        self._terminal_corridor_ids.add(corridor_id)
        self.metrics["attack_terminal_event_count"] += 1

    def inject_maneuver_fixture(
        self,
        *,
        attacker_id: int | str,
        defender_id: int | str,
        progress: float,
        gap_m: float,
        attacker_speed_mps: float,
        defender_speed_mps: float,
        outcome: str,
        finish_ready: bool = False,
        force_contact: bool = False,
    ) -> str:
        """Install a deterministic Gate 4R scenario at the injection boundary.

        This is intentionally an explicit test/diagnostic hook.  Production
        runs never call it and therefore retain normal RNG admission.  The
        hook still uses Bahrain/RBR compiled geometry, accepted line
        distance, the same lateral trajectory and the same maneuver updater.
        """

        if outcome not in {"success", "defense", "abort"}:
            raise ValueError("fixture outcome must be success, defense or abort")
        if attacker_id not in self.cars or defender_id not in self.cars:
            raise ValueError("fixture drivers must be present in the snapshot")
        if gap_m <= self.geometry.car_length_m:
            raise ValueError("fixture gap must leave initial body clearance")
        defender_index = self.order.index(defender_id)
        attacker_index = self.order.index(attacker_id)
        if attacker_index != defender_index + 1:
            self.order.pop(attacker_index)
            defender_index = self.order.index(defender_id)
            self.order.insert(defender_index + 1, attacker_id)
            attacker_index = defender_index + 1
        defender = self.cars[defender_id]
        attacker = self.cars[attacker_id]
        defender_line = self.distance_contract.distance_at_total_progress(progress)
        attacker_line = defender_line - gap_m
        for car, line_distance, speed in (
            (defender, defender_line, defender_speed_mps),
            (attacker, attacker_line, attacker_speed_mps),
        ):
            car.line_distance_m = line_distance
            car.total_progress = self.distance_contract.total_progress_at_distance(line_distance)
            car.speed_mps = max(0.0, speed)
            car.lateral_offset_m = 0.0
            car.lateral_velocity_mps = 0.0
            car.lateral_acceleration_mps2 = 0.0
            car.last_step = KinematicStep(
                distance_m=line_distance,
                speed_mps=car.speed_mps,
                longitudinal_acceleration_mps2=0.0,
                displacement_m=0.0,
                target_speed_mps=car.speed_mps,
                previous_distance_m=line_distance,
                previous_speed_mps=car.speed_mps,
                adjustment_reason="deterministic_fixture_initial_state",
            )
        # Keep the injected order physically admissible for the bounded
        # scenario.  The normal grid is intentionally not rewritten by
        # production code; this only prevents unrelated grid cars from
        # occupying the insertion envelope of a deterministic fixture.
        for distance_index, driver_id in enumerate(
            self.order[attacker_index + 1 :],
            start=1,
        ):
            other = self.cars[driver_id]
            other_line = (
                self.distance_contract.line_length_m * self.total_laps
                if finish_ready
                else max(0.0, attacker_line - distance_index * 40.0)
            )
            other.line_distance_m = other_line
            other.total_progress = self.distance_contract.total_progress_at_distance(other_line)
            other.lateral_offset_m = 0.0
            other.lateral_velocity_mps = 0.0
            other.lateral_acceleration_mps2 = 0.0
            other.lateral_trajectory = None
            other.last_step = replace(
                other.last_step,
                distance_m=other_line,
                previous_distance_m=other_line,
            ) if other.last_step is not None else None
        # The fixture represents an in-race injection; do not replay the
        # initial grid hold over the injected accepted speeds.
        self.lights_out_tick = min(self.lights_out_tick, self.tick_index)
        self._standing_start_active = False
        route_end_line = defender_line + 14.0
        attacker_limits = self._lateral_limits_over_arc(attacker_line, route_end_line)
        defender_limits = self._lateral_limits_over_arc(defender_line, route_end_line)
        attacker_target = min(attacker_limits[1] - 0.20, 1.10)
        defender_target = max(defender_limits[0] + 0.20, -1.10)
        lateral_min = min(attacker_target, defender_target) - self.geometry.car_width_m * 0.5
        lateral_max = max(attacker_target, defender_target) + self.geometry.car_width_m * 0.5
        segment, corner_phase, _ = self._segment_phase(attacker.total_progress)
        corridor_id = f"fixture:{self.tick_index}:{attacker_id}:{defender_id}:{outcome}"
        fixture_horizon = RESERVATION_HORIZON_S
        fixture_attacker_swept = accepted_swept_distance_m(
            attacker.speed_mps,
            fixture_horizon,
            target_speed_mps=TRAFFIC_MAX_SPEED_MPS,
        )
        fixture_defender_swept = accepted_swept_distance_m(
            defender.speed_mps,
            fixture_horizon,
            target_speed_mps=TRAFFIC_MAX_SPEED_MPS,
        )
        reservation = TrafficCorridorReservation(
            corridor_id=corridor_id,
            attacker_id=attacker_id,
            defender_id=defender_id,
            start_arc_distance_m=min(attacker_line, defender_line) - self.geometry.car_length_m * 0.5,
            end_arc_distance_m=max(
                attacker_line + fixture_attacker_swept,
                defender_line + fixture_defender_swept,
            ) + self.geometry.car_length_m * 0.5,
            start_time_s=self.logical_time_s,
            expiry_time_s=self.logical_time_s + fixture_horizon,
            lateral_min_m=lateral_min,
            lateral_max_m=lateral_max,
            side_assignment=((attacker_id, "outside"), (defender_id, "inside")),
            required_width_m=2.0 * self.geometry.car_width_m + 0.75,
            available_width_m=12.0,
            segment_type=segment.segment_type,
            corner_phase=corner_phase,
            third_vehicle_ids=(),
            admission_reason="deterministic-test-fixture",
            vehicle_length_m=self.geometry.car_length_m,
            vehicle_width_m=self.geometry.car_width_m,
        )
        attacker_traj = LateralTrajectory.create(
            start_offset_m=attacker.lateral_offset_m,
            end_offset_m=attacker_target,
            start_time_s=self.logical_time_s + 0.30,
            duration_s=0.75,
            target_limits_m=attacker_limits,
        )
        defender_traj = LateralTrajectory.create(
            start_offset_m=defender.lateral_offset_m,
            end_offset_m=defender_target,
            start_time_s=self.logical_time_s + 0.30,
            duration_s=0.75,
            target_limits_m=defender_limits,
        )
        maneuver = _TrafficManeuver(
            attacker_id=attacker_id,
            defender_id=defender_id,
            corridor_id=corridor_id,
            reservation=reservation,
            start_time_s=self.logical_time_s,
            deadline_time_s=self.logical_time_s + 4.0,
            phase="approach",
            attacker_target_offset_m=attacker_target,
            defender_target_offset_m=defender_target,
            attacker_trajectory=attacker_traj,
            defender_trajectory=defender_traj,
            defended=outcome == "defense",
            forced_outcome=outcome,
            force_contact=force_contact,
        )
        self.maneuvers[corridor_id] = maneuver
        for car, side, trajectory, visual in (
            (attacker, "outside", attacker_traj, "attack"),
            (defender, "inside", defender_traj, "defend"),
        ):
            car.corridor_id = corridor_id
            car.corridor_side = side
            car.maneuver = "approach"
            car.visual_state = visual
            car.lateral_trajectory = trajectory
        pair = tuple(sorted((attacker_id, defender_id), key=str))
        self.pair_next_attack_eligible_time_s[pair] = self.logical_time_s + 8.0
        self.next_attack_eligible_time_s[attacker_id] = self.logical_time_s + 8.0
        self.next_attack_eligible_time_s[defender_id] = self.logical_time_s + 8.0
        self.metrics["attack_start_count"] += 1
        self.events.append(
            LogicalEvent(
                event_id=f"{corridor_id}:approach",
                event_type="attack_approach_started",
                logical_time_s=self.logical_time_s,
                driver_ids=(attacker_id, defender_id),
                payload=(
                    ("corridor_id", corridor_id),
                    ("maneuver_phase", "approach"),
                    ("reservation", reservation.to_dict()),
                    ("fixture_outcome", outcome),
                ),
            )
        )
        self.events.append(
            LogicalEvent(
                event_id=f"{corridor_id}:started",
                event_type="attack_started",
                logical_time_s=self.logical_time_s,
                driver_ids=(attacker_id, defender_id),
                payload=(
                    ("corridor_id", corridor_id),
                    ("maneuver_phase", "approach"),
                    ("reservation", reservation.to_dict()),
                    ("fixture_outcome", outcome),
                    ("lap_number", int(max(0.0, floor(attacker.total_progress))) + 1),
                ),
            )
        )
        self._current_frame = self._build_frame()
        return corridor_id

    def _checkpoint(self, checkpoint_id: str, kind: str, lap_number: int) -> AbstractTimingCheckpoint:
        return AbstractTimingCheckpoint(
            checkpoint_id=checkpoint_id,
            checkpoint_kind=kind,
            tick_index=self.tick_index,
            logical_time_s=self.logical_time_s,
            lap_number=lap_number,
            order=tuple(self.order),
            progress_by_driver=tuple(
                (driver_id, self.cars[driver_id].total_progress)
                for driver_id in sorted(self.cars, key=str)
            ),
        )

    def _active_for(self, driver_id: int | str) -> _TrafficManeuver | None:
        active = [
            item for item in self.maneuvers.values()
            if driver_id in (item.attacker_id, item.defender_id)
        ]
        return sorted(active, key=lambda item: item.corridor_id)[0] if active else None

    def _pair_active(self, behind_id: int | str, ahead_id: int | str) -> _TrafficManeuver | None:
        for item in self.maneuvers.values():
            if (
                (item.attacker_id == behind_id and item.defender_id == ahead_id)
                or {item.attacker_id, item.defender_id} == {behind_id, ahead_id}
            ):
                return item
        return None

    def _segment_phase(self, progress: float) -> tuple[SegmentRequirement, str, float]:
        segment = self.kernel._segment_at_progress(progress)
        normalized = progress % 1.0
        span = max(1e-9, segment.end_progress - segment.start_progress)
        ratio = (normalized - segment.start_progress) / span
        if ratio < 0.35:
            phase = "entry"
        elif ratio < 0.65:
            phase = "apex"
        else:
            phase = "exit"
        end_distance = self.distance_contract.distance_at_total_progress(
            floor(progress) + segment.end_progress
        )
        return segment, phase, end_distance

    def _build_lateral_limit_lut(self) -> tuple[tuple[float, float], ...]:
        sample_count = 2048
        return tuple(
            self.distance_contract.lateral_limits_at_progress(
                index / sample_count,
                car_width_m=self.geometry.car_width_m,
            )
            for index in range(sample_count)
        )

    def _lateral_limits_at_line_distance(
        self,
        line_distance_m: float,
    ) -> tuple[float, float]:
        sample_count = len(self._lateral_limit_lut)
        local_distance = line_distance_m % self.distance_contract.line_length_m
        sample_position = local_distance / self.distance_contract.line_length_m * sample_count
        lower_index = int(sample_position) % sample_count
        upper_index = (lower_index + 1) % sample_count
        fraction = sample_position - int(sample_position)
        lower_a, upper_a = self._lateral_limit_lut[lower_index]
        lower_b, upper_b = self._lateral_limit_lut[upper_index]
        return (
            lower_a + (lower_b - lower_a) * fraction,
            upper_a + (upper_b - upper_a) * fraction,
        )

    def _lateral_limits_over_arc(
        self,
        start_line_distance_m: float,
        end_line_distance_m: float,
    ) -> tuple[float, float]:
        """Return the common local envelope over one planned transition arc."""

        lower = float("-inf")
        upper = float("inf")
        span = max(0.0, end_line_distance_m - start_line_distance_m)
        for sample_index in range(33):
            line_distance = start_line_distance_m + span * sample_index / 32.0
            limits = self._lateral_limits_at_line_distance(line_distance)
            lower = max(lower, limits[0])
            upper = min(upper, limits[1])
        if lower > upper:
            raise ValueError("lateral transition has no common local track envelope")
        return lower, upper

    def _following_envelope(
        self,
        *,
        logical_time_s: float,
        front_braking: bool,
    ) -> tuple[float, float]:
        """Blend staggered-grid spacing into the normal racing envelope.

        Grid cars begin closer longitudinally because their lateral positions
        alternate.  Applying the normal single-file following gap immediately
        at lights-out serializes the launch.  This smooth transition lets the
        field move together before establishing the normal gap, while closing
        speed and braking-distance guards remain active throughout.
        """

        if not self._standing_start_active:
            return MIN_LONGITUDINAL_GAP_M, 0.65 if front_braking else 0.25
        launch_elapsed_s = max(0.0, logical_time_s - GRID_HOLD_DURATION_S)
        ratio = min(
            1.0,
            launch_elapsed_s / GRID_LAUNCH_FOLLOWING_BLEND_DURATION_S,
        )
        smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        minimum_gap_m = GRID_LAUNCH_MIN_LONGITUDINAL_GAP_M + (
            MIN_LONGITUDINAL_GAP_M - GRID_LAUNCH_MIN_LONGITUDINAL_GAP_M
        ) * smooth_ratio
        racing_reaction_time_s = 0.65 if front_braking else 0.25
        return minimum_gap_m, racing_reaction_time_s * smooth_ratio

    def _following_ahead_id(self, driver_id: int | str) -> int | str | None:
        """Return the physical front boundary for a one- or two-wide group.

        During an overtake the immutable rank order is not swapped until
        crossing and clearance are confirmed.  The attacker must therefore
        follow the car ahead of the defender, while a car behind the pair must
        follow whichever participant is physically rearmost.  Using only the
        adjacent rank lets an attacker drive into the next car in a dense
        launch train and lets a follower enter the reserved corridor.
        """

        order_index = self.order.index(driver_id)
        if order_index <= 0:
            return None
        immediate_id = self.order[order_index - 1]
        pair = self._pair_active(driver_id, immediate_id)
        if (
            pair is not None
            and pair.attacker_id == driver_id
            and pair.defender_id == immediate_id
            and pair.phase in {"pull_out", "overlap", "crossing_confirmed"}
        ):
            return self.order[order_index - 2] if order_index >= 2 else None

        immediate_maneuver = self._active_for(immediate_id)
        if (
            immediate_maneuver is not None
            and immediate_maneuver.attacker_id == immediate_id
            and immediate_maneuver.phase
            in {"pull_out", "overlap", "crossing_confirmed"}
        ):
            attacker = self.cars[immediate_maneuver.attacker_id]
            defender = self.cars[immediate_maneuver.defender_id]
            return (
                defender.entry.driver_id
                if defender.line_distance_m <= attacker.line_distance_m
                else attacker.entry.driver_id
            )
        return immediate_id

    def _accepted_step(self, driver_id: int | str) -> KinematicStep:
        car = self.cars[driver_id]
        if self.tick_index + 1 <= self.lights_out_tick:
            return KinematicStep(
                distance_m=car.line_distance_m,
                speed_mps=0.0,
                longitudinal_acceleration_mps2=0.0,
                displacement_m=0.0,
                target_speed_mps=0.0,
                previous_distance_m=car.line_distance_m,
                previous_speed_mps=0.0,
                adjustment_reason="grid_hold",
            )
        plan = car.kinematic_plan
        if plan is None:
            raise RuntimeError("traffic car has no kinematic plan")
        active = self._active_for(driver_id)
        target = plan.target_speed_at_line_distance(car.line_distance_m)
        target *= car.pace_multiplier * self.kernel._speed_factor(car)
        if active is not None and active.attacker_id == driver_id and not active.defended and active.phase in {
            "pull_out",
            "overlap",
            "crossing_confirmed",
        }:
            # The maneuver is a provisional tactical pace bias, not a force
            # or throttle model.  It is bounded by the same accepted-speed
            # and acceleration guards and is only active while two-wide.
            target *= 1.35
            if active.forced_outcome == "success":
                target = TRAFFIC_MAX_SPEED_MPS
        if active is not None and active.defender_id == driver_id and active.forced_outcome == "success":
            target = 0.0
        if (
            active is not None
            and active.attacker_id == driver_id
            and active.phase == "fall_back_to_safe_gap"
            and not active.aborted
        ):
            target *= 1.45
        if active is not None and active.aborted:
            target *= 0.70
        target = min(TRAFFIC_MAX_SPEED_MPS, max(0.0, target))

        safe_next_speed_limit: float | None = None
        ahead_id = self._following_ahead_id(driver_id)
        if ahead_id is not None:
            ahead = self.cars[ahead_id]
            # IDM-like conservative safe speed.  ``gap`` is the centre
            # distance on the shared arc; body length is removed before
            # applying the provisional racing gap.
            if ahead.line_distance_m > car.line_distance_m:
                gap_m = ahead.line_distance_m - car.line_distance_m
                body_gap = max(0.0, gap_m - self.geometry.car_length_m)
                closing_speed = max(0.0, car.speed_mps - ahead.speed_mps)
                braking_distance = closing_speed * closing_speed / (2.0 * 50.0)
                ahead_plan = ahead.kinematic_plan
                ahead_target = (
                    ahead_plan.target_speed_at_line_distance(ahead.line_distance_m)
                    * ahead.pace_multiplier
                    * self.kernel._speed_factor(ahead)
                    if ahead_plan is not None
                    else ahead.speed_mps
                )
                anticipated_ahead_speed = min(ahead.speed_mps, max(0.0, ahead_target))
                relative_stop_distance = max(
                    0.0,
                    (car.speed_mps * car.speed_mps - anticipated_ahead_speed * anticipated_ahead_speed)
                    / (2.0 * 50.0),
                )
                front_braking = ahead.speed_mps - anticipated_ahead_speed > 0.25
                minimum_gap_m, reaction_time_s = self._following_envelope(
                    logical_time_s=self.logical_time_s + self.tick_seconds,
                    front_braking=front_braking,
                )
                reaction_buffer = max(car.speed_mps, ahead.speed_mps) * reaction_time_s
                desired_gap = minimum_gap_m + max(
                    braking_distance,
                    relative_stop_distance,
                ) + reaction_buffer
                available_gap = body_gap - desired_gap
                safe_speed = max(
                    0.0,
                    ahead.speed_mps + available_gap / self.tick_seconds,
                )
                target = min(target, safe_speed)
                if gap_m < desired_gap:
                    # The front car's accepted step is the only tactical
                    # reference.  Capping the requested speed here causes
                    # early bounded braking; it is not a post-step clamp.
                    target = min(target, anticipated_ahead_speed)
                max_safe_displacement = max(
                    0.0,
                    gap_m
                    - self.geometry.car_length_m
                    - minimum_gap_m
                    - braking_distance
                    - reaction_buffer
                    + ahead.speed_mps * self.tick_seconds,
                )
                safe_next_speed_limit = max(
                    0.0,
                    2.0 * max_safe_displacement / self.tick_seconds - car.speed_mps,
                )

        previous_speed = min(TRAFFIC_MAX_SPEED_MPS, max(0.0, car.speed_mps))
        requested_acceleration = (target - previous_speed) / self.tick_seconds
        acceleration = min(18.0, max(-50.0, requested_acceleration))
        next_speed = min(
            TRAFFIC_MAX_SPEED_MPS,
            max(0.0, previous_speed + acceleration * self.tick_seconds),
        )
        if safe_next_speed_limit is not None:
            next_speed = min(next_speed, safe_next_speed_limit)
            acceleration = (next_speed - previous_speed) / self.tick_seconds
            # The controller never writes a distance correction after this
            # point; it records the accepted speed and derives ds atomically.
            if acceleration < -50.0:
                next_speed = max(0.0, previous_speed - 50.0 * self.tick_seconds)
                acceleration = (next_speed - previous_speed) / self.tick_seconds
        displacement = (previous_speed + next_speed) * 0.5 * self.tick_seconds
        maximum_displacement = previous_speed * self.tick_seconds + 0.5
        if displacement > maximum_displacement + 1e-12:
            # This is an accepted pre-commit speed re-evaluation, never a
            # distance-only correction after the step.
            next_speed = max(0.0, 2.0 * maximum_displacement / self.tick_seconds - previous_speed)
            acceleration = (next_speed - previous_speed) / self.tick_seconds
            displacement = (previous_speed + next_speed) * 0.5 * self.tick_seconds
        return KinematicStep(
            distance_m=car.line_distance_m + displacement,
            speed_mps=next_speed,
            longitudinal_acceleration_mps2=acceleration,
            displacement_m=displacement,
            target_speed_mps=target,
            previous_distance_m=car.line_distance_m,
            previous_speed_mps=previous_speed,
            integrator_adjusted=False,
        )

    def _smooth_incident_offsets(self, previous_offsets: Mapping[int | str, float]) -> None:
        for driver_id in sorted(self.cars, key=str):
            car = self.cars[driver_id]
            previous = previous_offsets[driver_id]
            if car.corridor_id is not None:
                # Existing logical lockup/contact bookkeeping must not move a
                # car into an active two-wide reservation.  Stage 4 keeps the
                # accepted corridor trajectory authoritative and lets the
                # incident state expire without adding Stage 6 motion.
                car.lateral_offset_m = previous
                continue
            if abs(car.lateral_offset_m - previous) <= 1e-9:
                continue
            limits = self.distance_contract.lateral_limits_at_progress(
                car.total_progress,
                car_width_m=self.geometry.car_width_m,
            )
            car.lateral_trajectory = LateralTrajectory.create(
                start_offset_m=previous,
                end_offset_m=car.lateral_offset_m,
                start_time_s=self.logical_time_s,
                duration_s=0.75,
                target_limits_m=limits,
            )
            car.lateral_offset_m = previous

    def _set_rejoin_trajectory(self, car: _MutableCar, now: float) -> None:
        limits = self.distance_contract.lateral_limits_at_progress(
            car.total_progress,
            car_width_m=self.geometry.car_width_m,
        )
        car.lateral_trajectory = LateralTrajectory.create(
            start_offset_m=car.lateral_offset_m,
            end_offset_m=0.0,
            start_time_s=now,
            duration_s=1.5,
            target_limits_m=limits,
        )

    def _hold_lateral_corridor(self, car: _MutableCar, now: float) -> None:
        """Freeze the current accepted lateral position until clearance.

        An aborted maneuver may spend several logical seconds waiting for a
        safe longitudinal fallback.  Letting the original pull-out curve
        expire during that wait would silently rejoin into an overlapping
        body.  The hold is a state transition at the accepted tick; it does
        not snap the car to the racing line.
        """

        if car.lateral_trajectory is not None:
            car.lateral_offset_m = car.lateral_trajectory.offset_at(now)
        car.lateral_trajectory = None
        car.lateral_velocity_mps = 0.0
        car.lateral_acceleration_mps2 = 0.0

    def _refresh_lateral_states(self, now: float) -> None:
        for driver_id in sorted(self.cars, key=str):
            car = self.cars[driver_id]
            trajectory = car.lateral_trajectory
            if trajectory is None:
                continue
            if (
                self._standing_start_active
                and now
                <= GRID_HOLD_DURATION_S + GRID_LAUNCH_FOLLOWING_BLEND_DURATION_S
                + 1e-9
                and car.speed_mps <= 1e-9
                and now < trajectory.start_time_s + trajectory.duration_s - 1e-9
            ):
                # Lateral progress is presentation motion and must never make
                # a longitudinally stopped car slide across the circuit.  Move
                # the trajectory's time origin forward by this tick so it can
                # resume continuously once the car is moving again.
                car.lateral_trajectory = replace(
                    trajectory,
                    start_time_s=trajectory.start_time_s + self.tick_seconds,
                )
                car.lateral_velocity_mps = 0.0
                car.lateral_acceleration_mps2 = 0.0
                continue
            car.lateral_offset_m = trajectory.offset_at(now)
            car.lateral_velocity_mps = trajectory.velocity_at(now)
            car.lateral_acceleration_mps2 = trajectory.acceleration_at(now)
            self.synthesizer.validate_lateral_offset(car.line_distance_m, car.lateral_offset_m)

    def _anticipated_third_vehicle_ids(
        self,
        *,
        attacker_id: int | str,
        defender_id: int | str,
        start_arc_distance_m: float,
        end_arc_distance_m: float,
        start_time_s: float,
        expiry_time_s: float,
        lateral_min_m: float,
        lateral_max_m: float,
    ) -> tuple[int | str, ...]:
        # The admission-time explanation and the accepted-frame audit use
        # the same full reservation horizon through fallback/rejoin.
        horizon_s = max(0.0, expiry_time_s - start_time_s)
        attacker = self.cars[attacker_id]
        defender = self.cars[defender_id]
        attacker_target_speed = self._forecast_target_speed_mps(attacker, horizon_s)
        defender_target_speed = self._forecast_target_speed_mps(defender, horizon_s)
        third: list[int | str] = []
        # The pair is adjacent in the authoritative rank order.  The accepted
        # following controller keeps the attacker behind the car ahead of the
        # pair and the next follower behind the rearmost participant, so the
        # two immediate outer neighbours are the admission boundary.  The
        # independent accepted-frame audit still checks all 20 cars.
        attacker_index = self.order.index(attacker_id)
        defender_index = self.order.index(defender_id)
        pair_front_index = min(attacker_index, defender_index)
        pair_rear_index = max(attacker_index, defender_index)
        candidate_ids = {
            self.order[index]
            for index in (pair_front_index - 1, pair_rear_index + 1)
            if 0 <= index < len(self.order)
        } - {attacker_id, defender_id}

        for driver_id in sorted(candidate_ids, key=str):
            other = self.cars[driver_id]
            # Forecast the slowest target speed on the vehicle's projected
            # route.  This catches a third car that is about to brake for a
            # corner while avoiding an assumption that every car instantly
            # stops.  The accepted-frame audit remains independent of the
            # admission-time explanation list, but consumes this same plan
            # forecast when the cursor performs its own accepted-state audit.
            target_speed = self._forecast_target_speed_mps(other, horizon_s)
            if swept_vehicle_intersects_reservation(
                reservation_start_arc_m=start_arc_distance_m,
                reservation_end_arc_m=end_arc_distance_m,
                reservation_horizon_s=horizon_s,
                vehicle_start_arc_m=other.line_distance_m,
                vehicle_speed_mps=other.speed_mps,
                vehicle_target_speed_mps=target_speed,
                vehicle_lateral_offset_m=other.lateral_offset_m,
                line_length_m=self.distance_contract.line_length_m,
                vehicle_length_m=self.geometry.car_length_m,
                vehicle_width_m=self.geometry.car_width_m,
                lateral_min_m=lateral_min_m,
                lateral_max_m=lateral_max_m,
                reservation_attacker_start_arc_m=attacker.line_distance_m,
                reservation_attacker_speed_mps=attacker.speed_mps,
                reservation_attacker_target_speed_mps=attacker_target_speed,
                reservation_defender_start_arc_m=defender.line_distance_m,
                reservation_defender_speed_mps=defender.speed_mps,
                reservation_defender_target_speed_mps=defender_target_speed,
            ):
                third.append(driver_id)
        return tuple(third)

    def _forecast_target_speed_mps(self, car: _MutableCar, horizon_s: float) -> float:
        """Return the deterministic horizon target used by accepted steps.

        This is a traffic-envelope forecast, not a new vehicle-performance
        model.  It samples the already compiled longitudinal plan at fixed
        fractions so the admission and accepted-frame audit share one
        bounded future-speed assumption without consuming RNG.
        """

        cache_key = (car.entry.driver_id, round(float(horizon_s), 9))
        cached = self._forecast_target_cache.get(cache_key)
        if cached is not None:
            return cached
        target_speed = car.speed_mps
        if car.kinematic_plan is None:
            self._forecast_target_cache[cache_key] = target_speed
            return target_speed
        projected_distance = accepted_swept_distance_m(
            car.speed_mps,
            horizon_s,
            target_speed_mps=car.speed_mps,
        )
        target_speed = min(
            target_speed,
            *(
                car.kinematic_plan.target_speed_at_line_distance(
                    car.line_distance_m + projected_distance * fraction
                )
                * max(1.0, car.pace_multiplier)
                * self.kernel._speed_factor(car)
                for fraction in (0.25, 0.50, 0.75, 1.0)
            ),
        )
        self._forecast_target_cache[cache_key] = target_speed
        return target_speed

    def _schedule_maneuvers(self, now: float, events: list[LogicalEvent]) -> None:
        if (
            self._standing_start_active
            and now
            < GRID_HOLD_DURATION_S + GRID_LAUNCH_FOLLOWING_BLEND_DURATION_S
            - 1e-9
        ):
            return
        for position in range(len(self.order) - 1, 0, -1):
            attacker_id = self.order[position]
            defender_id = self.order[position - 1]
            pair = (attacker_id, defender_id)
            if self._active_for(attacker_id) or self._active_for(defender_id):
                continue
            cooldown_pair = tuple(sorted(pair, key=str))
            if now < self.pair_next_attack_eligible_time_s.get(cooldown_pair, 0.0) - 1e-9:
                continue
            if any(
                now < self.next_attack_eligible_time_s.get(driver_id, 0.0) - 1e-9
                for driver_id in pair
            ):
                continue
            attacker = self.cars[attacker_id]
            defender = self.cars[defender_id]
            if (
                attacker.pit_state != "none"
                or defender.pit_state != "none"
                or attacker.incident_state is not None
                or defender.incident_state is not None
            ):
                continue
            gap_m = defender.line_distance_m - attacker.line_distance_m
            closing_speed = max(0.0, attacker.speed_mps - defender.speed_mps)
            attacker_plan = attacker.kinematic_plan
            defender_plan = defender.kinematic_plan
            if attacker_plan is not None and defender_plan is not None:
                projected_attacker_speed = attacker_plan.target_speed_at_line_distance(attacker.line_distance_m) * attacker.pace_multiplier
                projected_defender_speed = defender_plan.target_speed_at_line_distance(defender.line_distance_m) * defender.pace_multiplier
                closing_speed = max(closing_speed, projected_attacker_speed - projected_defender_speed)
            braking_distance = max(0.0, attacker.speed_mps - defender.speed_mps) ** 2 / (2.0 * 50.0)
            minimum_attack_gap = self.geometry.car_length_m + MIN_LONGITUDINAL_GAP_M + braking_distance + 1.0
            if gap_m < minimum_attack_gap:
                continue
            segment, corner_phase, segment_end_distance = self._segment_phase(attacker.total_progress)
            remaining_distance = (segment_end_distance - attacker.line_distance_m) % self.distance_contract.line_length_m
            sample = self.geometry.compiled_profile.at_progress(attacker.total_progress)
            local_route_end = max(attacker.line_distance_m, defender.line_distance_m) + 14.0
            attacker_limits = self._lateral_limits_over_arc(attacker.line_distance_m, local_route_end)
            defender_limits = self._lateral_limits_over_arc(defender.line_distance_m, local_route_end)
            attacker_target = min(attacker_limits[1] - 0.20, 1.10)
            defender_target = max(defender_limits[0] + 0.20, -1.10)
            if attacker_target - defender_target < self.geometry.car_width_m + BODY_CLEARANCE_MARGIN_M:
                continue
            lateral_min = min(attacker_target, defender_target) - self.geometry.car_width_m * 0.5
            lateral_max = max(attacker_target, defender_target) + self.geometry.car_width_m * 0.5
            # The immutable reservation covers the complete projected
            # approach→fallback/rejoin occupancy, not only the first 14m.
            # Pair cooldown is separate and remains eight logical seconds.
            expiry = now + RESERVATION_HORIZON_S
            attacker_target_speed = self._forecast_target_speed_mps(
                attacker,
                RESERVATION_HORIZON_S,
            )
            defender_target_speed = self._forecast_target_speed_mps(
                defender,
                RESERVATION_HORIZON_S,
            )
            attacker_swept = accepted_swept_distance_m(
                attacker.speed_mps,
                RESERVATION_HORIZON_S,
                target_speed_mps=attacker_target_speed,
            )
            defender_swept = accepted_swept_distance_m(
                defender.speed_mps,
                RESERVATION_HORIZON_S,
                target_speed_mps=defender_target_speed,
            )
            start_arc = min(attacker.line_distance_m, defender.line_distance_m) - self.geometry.car_length_m * 0.5
            end_arc = max(
                attacker.line_distance_m + attacker_swept,
                defender.line_distance_m + defender_swept,
            ) + self.geometry.car_length_m * 0.5
            third_vehicle_ids = self._anticipated_third_vehicle_ids(
                attacker_id=attacker_id,
                defender_id=defender_id,
                start_arc_distance_m=start_arc,
                # Admission and accepted-frame audit use the same expanded
                # time-aligned envelope.  ``third_vehicle_ids`` remains an
                # explanation snapshot; accepted frames still inspect all
                # non-participants independently.
                end_arc_distance_m=end_arc,
                start_time_s=now,
                expiry_time_s=expiry,
                lateral_min_m=lateral_min,
                lateral_max_m=lateral_max,
            )
            provisional_reservation = TrafficCorridorReservation(
                corridor_id=f"corridor:{self.tick_index + 1}:{attacker_id}:{defender_id}",
                attacker_id=attacker_id,
                defender_id=defender_id,
                start_arc_distance_m=start_arc,
                end_arc_distance_m=end_arc,
                start_time_s=now,
                expiry_time_s=expiry,
                lateral_min_m=lateral_min,
                lateral_max_m=lateral_max,
                side_assignment=((attacker_id, "outside"), (defender_id, "inside")),
                required_width_m=2.0 * self.geometry.car_width_m + 0.75,
                available_width_m=sample.left_width_m + sample.right_width_m,
                segment_type=segment.segment_type,
                corner_phase=corner_phase,
                third_vehicle_ids=third_vehicle_ids,
                admission_reason="local-geometry-following-clearance",
                vehicle_length_m=self.geometry.car_length_m,
                vehicle_width_m=self.geometry.car_width_m,
                occupancy_model="time_aligned",
            )
            reservation_audit = audit_reservation_intersections(
                (*self.active_reservations, provisional_reservation),
                line_length_m=self.distance_contract.line_length_m,
            )
            reservation_conflict = reservation_audit["admitted_reservation_conflict_count"] > 0
            third_conflict = bool(third_vehicle_ids)
            assessment = local_corridor_assessment(
                local_left_width_m=sample.left_width_m,
                local_right_width_m=sample.right_width_m,
                racing_line_offset_m=sample.racing_line_offset_m,
                vehicle_width_m=self.geometry.car_width_m,
                vehicle_length_m=self.geometry.car_length_m,
                segment_type=segment.segment_type,
                corner_phase=corner_phase,
                remaining_distance_m=remaining_distance,
                gap_m=gap_m,
                closing_speed_mps=closing_speed,
                third_vehicle_conflict=third_conflict,
                reservation_conflict=reservation_conflict,
            )
            if not assessment.allowed:
                # A rejected attack opportunity is not a corridor conflict.
                # Conflicts are reserved for an admitted maneuver whose
                # local space reservation later overlaps another admitted
                # reservation; rejection itself is expected traffic logic.
                reasons = self.metrics["corridor_rejection_reason_counts"]
                reasons[assessment.reason_code] = reasons.get(assessment.reason_code, 0) + 1
                if assessment.reason_code == "corridor_conflict":
                    if assessment.third_vehicle_conflict:
                        self.metrics["third_vehicle_conflict_rejection_count"] += 1
                    if assessment.reservation_conflict:
                        self.metrics["reservation_conflict_rejection_count"] += 1
                continue
            random_value = self.mistake_streams[attacker_id].random()
            attacker_score = (
                attacker.entry.vehicle.performance.power
                + attacker.entry.vehicle.performance.traction
                + attacker.entry.driver.overtaking
            )
            defender_score = (
                defender.entry.vehicle.performance.power
                + defender.entry.vehicle.performance.braking
                + defender.entry.driver.defending
            )
            readiness = min(0.88, max(0.28, 0.48 + (attacker_score - defender_score) * 0.10 + closing_speed / 120.0))
            if random_value >= readiness:
                continue
            corridor_id = provisional_reservation.corridor_id
            attacker_traj = LateralTrajectory.create(
                start_offset_m=attacker.lateral_offset_m,
                end_offset_m=attacker_target,
                start_time_s=now + 0.30,
                duration_s=0.75,
                target_limits_m=attacker_limits,
            )
            defender_traj = LateralTrajectory.create(
                start_offset_m=defender.lateral_offset_m,
                end_offset_m=defender_target,
                start_time_s=now + 0.30,
                duration_s=0.75,
                target_limits_m=defender_limits,
            )
            maneuver = _TrafficManeuver(
                attacker_id=attacker_id,
                defender_id=defender_id,
                corridor_id=corridor_id,
                reservation=provisional_reservation,
                start_time_s=now,
                deadline_time_s=now + MANEUVER_DEADLINE_S,
                phase="approach",
                attacker_target_offset_m=attacker_target,
                defender_target_offset_m=defender_target,
                attacker_trajectory=attacker_traj,
                defender_trajectory=defender_traj,
                defended=random_value >= readiness * 0.55,
            )
            self.maneuvers[corridor_id] = maneuver
            self.pair_next_attack_eligible_time_s[cooldown_pair] = now + 8.0
            for driver_id in pair:
                self.next_attack_eligible_time_s[driver_id] = max(
                    self.next_attack_eligible_time_s.get(driver_id, 0.0),
                    now + 8.0,
                )
            attacker.corridor_id = corridor_id
            attacker.corridor_side = "outside"
            attacker.maneuver = "approach"
            attacker.visual_state = "attack"
            attacker.lateral_trajectory = attacker_traj
            defender.corridor_id = corridor_id
            defender.corridor_side = "inside"
            defender.maneuver = "approach"
            defender.visual_state = "defend"
            defender.lateral_trajectory = defender_traj
            self.metrics["attack_start_count"] += 1
            events.append(
                LogicalEvent(
                    event_id=f"{corridor_id}:approach",
                    event_type="attack_approach_started",
                    logical_time_s=now,
                    driver_ids=(attacker_id, defender_id),
                    payload=(
                        ("corridor_id", corridor_id),
                        ("maneuver_phase", "approach"),
                        ("reservation", provisional_reservation.to_dict()),
                    ),
                )
            )
            events.append(
                LogicalEvent(
                    event_id=f"{corridor_id}:started",
                    event_type="attack_started",
                    logical_time_s=now,
                    driver_ids=(attacker_id, defender_id),
                    payload=tuple(sorted({
                        "corridor_id": corridor_id,
                        "segment_type": segment.segment_type,
                        "corner_phase": corner_phase,
                        "corridor_width_m": assessment.corridor_width_m,
                        "required_width_m": assessment.required_width_m,
                        "gap_m": gap_m,
                        "closing_speed_mps": closing_speed,
                        "maneuver_phase": "approach",
                        "reservation": provisional_reservation.to_dict(),
                        "pair_next_attack_eligible_time_s": now + 8.0,
                        "attacker_next_attack_eligible_time_s": now + 8.0,
                        "defender_next_attack_eligible_time_s": now + 8.0,
                        "lap_number": int(max(0.0, floor(attacker.total_progress))) + 1,
                    }.items())),
                )
            )

    def _update_maneuvers(self, now: float, events: list[LogicalEvent]) -> None:
        for corridor_id in sorted(tuple(self.maneuvers)):
            maneuver = self.maneuvers[corridor_id]
            attacker = self.cars[maneuver.attacker_id]
            defender = self.cars[maneuver.defender_id]
            separation = abs(attacker.lateral_offset_m - defender.lateral_offset_m)
            if maneuver.phase == "approach" and now >= maneuver.start_time_s + 0.30 - 1e-9:
                maneuver.phase = "pull_out"
                attacker.maneuver = "pull_out"
                defender.maneuver = "pull_out"
                events.append(
                    LogicalEvent(
                        event_id=f"{corridor_id}:pull_out",
                        event_type="attack_pull_out_started",
                        logical_time_s=now,
                        driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                        payload=(
                            ("maneuver_phase", "pull_out"),
                            ("corridor_id", corridor_id),
                        ),
                    )
                )
            if maneuver.phase == "pull_out" and now >= maneuver.start_time_s + 1.55 - 1e-9:
                maneuver.phase = "overlap"
                attacker.maneuver = "overlap"
                defender.maneuver = "overlap"
                events.append(
                    LogicalEvent(
                        event_id=f"{corridor_id}:overlap",
                        event_type="attack_overlap_started",
                        logical_time_s=now,
                        driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                        payload=(("maneuver_phase", "overlap"),),
                    )
                )
            if maneuver.phase in {"overlap", "crossing_confirmed"}:
                if (
                    maneuver.forced_outcome not in {"defense", "abort"}
                    and attacker.line_distance_m > defender.line_distance_m + 0.01
                    and separation >= self.geometry.car_width_m + 0.15
                ):
                    if not maneuver.crossing_confirmed:
                        maneuver.crossing_confirmed = True
                        maneuver.phase = "crossing_confirmed"
                        events.append(
                            LogicalEvent(
                                event_id=f"{corridor_id}:crossing",
                                event_type="overtake_crossing_confirmed",
                                logical_time_s=now,
                                driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                                payload=(("maneuver_phase", "crossing_confirmed"),),
                            )
                        )
            if maneuver.phase == "crossing_confirmed":
                # In a reserved two-wide corridor the lateral body clearance
                # is the clearance authority.  A positive centre crossing
                # plus the side separation is sufficient to atomically swap
                # rank; the cars then rejoin with their accepted distances.
                if (
                    attacker.line_distance_m - defender.line_distance_m
                    >= self.geometry.car_length_m + MIN_LONGITUDINAL_GAP_M + 1.0
                ):
                    contact = assess_contact(
                        corridor_active=True,
                        closing_speed_mps=max(0.0, attacker.speed_mps - defender.speed_mps),
                        gap_m=max(0.0, attacker.line_distance_m - defender.line_distance_m),
                        overtaking_difficulty=self.snapshot.track.overtaking_difficulty,
                        attacker_risk_tolerance=attacker.entry.driver.overtaking,
                        random_value=(
                            1.0
                            if maneuver.forced_outcome == "success"
                            else self.contact_streams[maneuver.attacker_id].random()
                        ),
                        severity_value=(
                            0.0
                            if maneuver.forced_outcome == "success"
                            else self.contact_streams[maneuver.attacker_id].random()
                        ),
                    )
                    if contact.triggered and attacker.line_distance_m - defender.line_distance_m >= self.geometry.car_length_m + MIN_LONGITUDINAL_GAP_M:
                        severity = max(0.0, min(1.0, contact.severity))
                        for car in (attacker, defender):
                            car.damage_level = min(1.0, car.damage_level + contact.damage_delta)
                            car.incident_state = "contact"
                            car.incident_remaining_ticks = max(car.incident_remaining_ticks, 3)
                            car.incident_cooldown_ticks = max(car.incident_cooldown_ticks, 20)
                            car.visual_state = "contact"
                        maneuver.aborted = True
                        maneuver.phase = "yield_or_abort"
                        self._hold_lateral_corridor(attacker, now)
                        self._hold_lateral_corridor(defender, now)
                        attacker.maneuver = "yield_or_abort"
                        defender.maneuver = "yield_or_abort"
                        self.metrics["attack_aborted_count"] += 1
                        self._record_attack_terminal(corridor_id)
                        events.append(
                            LogicalEvent(
                                event_id=f"{corridor_id}:contact",
                                event_type="contact_started",
                                logical_time_s=now,
                                driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                                payload=tuple(sorted(contact.to_dict().items())),
                            )
                        )
                        events.append(
                            LogicalEvent(
                                event_id=f"{corridor_id}:contact_aborted",
                                event_type="attack_aborted",
                                logical_time_s=now,
                                driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                                payload=(("reason_code", "contact"),),
                            )
                        )
                        continue
                    attacker_index = self.order.index(maneuver.attacker_id)
                    defender_index = self.order.index(maneuver.defender_id)
                    if attacker_index == defender_index + 1:
                        def insertion_clearance(front_id: int | str, rear_id: int | str) -> float:
                            front = self.cars[front_id]
                            rear = self.cars[rear_id]
                            closing = max(0.0, rear.speed_mps - front.speed_mps)
                            braking = closing * closing / (2.0 * 50.0)
                            return (
                                self.geometry.car_length_m
                                + MIN_LONGITUDINAL_GAP_M
                                + 1.0
                                + closing * self.tick_seconds
                                + 0.5 * braking
                            )

                        new_front_id = (
                            self.order[defender_index - 1]
                            if defender_index > 0
                            else None
                        )
                        if new_front_id is not None:
                            new_front_gap = (
                                self.cars[new_front_id].line_distance_m
                                - attacker.line_distance_m
                            )
                            if (
                                new_front_gap
                                < insertion_clearance(new_front_id, maneuver.attacker_id)
                            ):
                                # The attacker has cleared the defender but
                                # cannot yet be inserted safely ahead of the
                                # next car.  Keep the accepted two-wide
                                # corridor; rank remains unchanged until the
                                # complete insertion clearance exists.
                                continue
                        new_behind_id = (
                            self.order[attacker_index + 1]
                            if attacker_index + 1 < len(self.order)
                            else None
                        )
                        if new_behind_id is not None:
                            new_behind_gap = (
                                attacker.line_distance_m
                                - self.cars[new_behind_id].line_distance_m
                            )
                            if (
                                new_behind_gap
                                < insertion_clearance(maneuver.attacker_id, new_behind_id)
                            ):
                                continue
                            defender_following_gap = (
                                defender.line_distance_m
                                - self.cars[new_behind_id].line_distance_m
                            )
                            defender_following_margin = (
                                self.geometry.car_length_m
                                + MIN_LONGITUDINAL_GAP_M
                                + 1.0
                                + max(
                                    defender.speed_mps,
                                    self.cars[new_behind_id].speed_mps,
                                )
                                * 0.25
                            )
                            if defender_following_gap < defender_following_margin:
                                continue
                        self.order[defender_index], self.order[attacker_index] = (
                            self.order[attacker_index],
                            self.order[defender_index],
                        )
                        maneuver.clearance_confirmed = True
                        maneuver.phase = "clearance_confirmed"
                        events.append(
                            LogicalEvent(
                                event_id=f"{corridor_id}:clearance",
                                event_type="overtake_clearance_confirmed",
                                logical_time_s=now,
                                driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                                payload=(
                                    ("maneuver_phase", "clearance_confirmed"),
                                    ("corridor_id", corridor_id),
                                ),
                            )
                        )
                        maneuver.phase = "rejoin"
                        self._set_rejoin_trajectory(attacker, now)
                        self._set_rejoin_trajectory(defender, now)
                        attacker.maneuver = "rejoin"
                        defender.maneuver = "rejoin"
                        self.metrics["overtake_completed_count"] += 1
                        self._record_attack_terminal(corridor_id)
                        events.append(
                            LogicalEvent(
                                event_id=f"{corridor_id}:completed",
                                event_type="overtake_completed",
                                logical_time_s=now,
                                driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                                payload=(
                                    ("corridor_id", corridor_id),
                                    ("maneuver_phase", "clearance_confirmed"),
                                    ("rank_swap", "atomic"),
                                ),
                            )
                        )
                        continue
                    else:
                        self.metrics["crossing_before_rank_swap_count"] += 1
            elif maneuver.phase in {"pull_out", "overlap"} and now >= maneuver.deadline_time_s:
                maneuver.aborted = True
                # Do not collapse the lateral corridor while the cars are
                # still longitudinally overlapped.  The two-wide state is
                # retained until a safe signed/absolute body clearance is
                # accepted, then _update_maneuvers starts the rejoin curve.
                maneuver.phase = "yield_or_abort"
                self._hold_lateral_corridor(attacker, now)
                self._hold_lateral_corridor(defender, now)
                attacker.maneuver = "yield_or_abort"
                defender.maneuver = "yield_or_abort"
                self.metrics["attack_aborted_count"] += 1
                self._record_attack_terminal(corridor_id)
                terminal_event_type = "defense_hold" if maneuver.defended else "attack_aborted"
                reason_code = "defender_hold" if maneuver.defended else "no_clearance"
                events.append(
                    LogicalEvent(
                        event_id=f"{corridor_id}:aborted",
                        event_type=terminal_event_type,
                        logical_time_s=now,
                        driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                        payload=(("reason_code", reason_code),),
                    )
                )
                continue
            elif maneuver.phase == "yield_or_abort":
                maneuver.phase = "fall_back_to_safe_gap"
                attacker.maneuver = "fall_back_to_safe_gap"
                defender.maneuver = "fall_back_to_safe_gap"
                events.append(
                    LogicalEvent(
                        event_id=f"{corridor_id}:fallback",
                        event_type="attack_fallback_started",
                        logical_time_s=now,
                        driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                        payload=(
                            ("maneuver_phase", "fall_back_to_safe_gap"),
                            ("corridor_id", corridor_id),
                        ),
                    )
                )
                continue
            elif maneuver.phase == "fall_back_to_safe_gap":
                separation_distance = abs(attacker.line_distance_m - defender.line_distance_m)
                if (
                    not maneuver.contact_checked
                    and separation_distance >= self.geometry.car_length_m + MIN_LONGITUDINAL_GAP_M
                ):
                    contact = assess_contact(
                        corridor_active=True,
                        closing_speed_mps=max(0.0, attacker.speed_mps - defender.speed_mps),
                        gap_m=separation_distance,
                        overtaking_difficulty=self.snapshot.track.overtaking_difficulty,
                        attacker_risk_tolerance=attacker.entry.driver.overtaking,
                        random_value=(
                            0.0
                            if maneuver.force_contact
                            else self.contact_streams[maneuver.attacker_id].random()
                        ),
                        severity_value=(
                            0.5
                            if maneuver.force_contact
                            else self.contact_streams[maneuver.attacker_id].random()
                        ),
                    )
                    maneuver.contact_checked = True
                    if contact.triggered:
                        for car in (attacker, defender):
                            car.damage_level = min(1.0, car.damage_level + contact.damage_delta)
                            car.incident_state = "contact"
                            car.incident_remaining_ticks = max(car.incident_remaining_ticks, 3)
                            car.incident_cooldown_ticks = max(car.incident_cooldown_ticks, 20)
                            car.visual_state = "contact"
                        events.append(
                            LogicalEvent(
                                event_id=f"{corridor_id}:contact",
                                event_type="contact_started",
                                logical_time_s=now,
                                driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                                payload=tuple(sorted(contact.to_dict().items())),
                            )
                        )
                signed_separation = attacker.line_distance_m - defender.line_distance_m
                has_signed_clearance = (
                    not maneuver.aborted
                    and signed_separation
                    >= (
                        self.geometry.car_length_m
                        + MIN_LONGITUDINAL_GAP_M
                        + 1.0
                        + max(0.0, defender.speed_mps - attacker.speed_mps) * 1.5
                    )
                )
                has_aborted_fallback_clearance = (
                    maneuver.aborted
                    and signed_separation
                    <= -(
                        self.geometry.car_length_m
                        + MIN_LONGITUDINAL_GAP_M
                        + 1.0
                        + max(0.0, attacker.speed_mps - defender.speed_mps) * 1.5
                    )
                )
                if has_signed_clearance or has_aborted_fallback_clearance:
                    maneuver.phase = "rejoin"
                    self._set_rejoin_trajectory(attacker, now)
                    self._set_rejoin_trajectory(defender, now)
                    attacker.maneuver = "rejoin"
                    defender.maneuver = "rejoin"
                continue
            if maneuver.phase == "rejoin":
                end_time = max(
                    attacker.lateral_trajectory.start_time_s + attacker.lateral_trajectory.duration_s
                    if attacker.lateral_trajectory is not None else now,
                    defender.lateral_trajectory.start_time_s + defender.lateral_trajectory.duration_s
                    if defender.lateral_trajectory is not None else now,
                )
                if now >= end_time - 1e-9:
                    for car in (attacker, defender):
                        car.lateral_trajectory = None
                        car.lateral_offset_m = 0.0
                        car.lateral_velocity_mps = 0.0
                        car.lateral_acceleration_mps2 = 0.0
                        car.corridor_id = None
                        car.corridor_side = None
                        car.maneuver = "normal"
                        if car.incident_state is None:
                            car.visual_state = "normal"
                    events.append(
                        LogicalEvent(
                            event_id=f"{corridor_id}:rejoin",
                            event_type="rejoin_complete",
                            logical_time_s=now,
                            driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                        )
                    )
                    self.maneuvers.pop(corridor_id, None)

    def _mark_finished(self, now: float, events: list[LogicalEvent]) -> None:
        candidates = sorted(
            (
                self.cars[driver_id].total_progress,
                driver_id,
            )
            for driver_id in self.order
            if self.cars[driver_id].total_progress >= self.total_laps
            and driver_id not in self.finish_order
        )
        for _, driver_id in reversed(candidates):
            self.finish_order.append(driver_id)
            events.append(
                LogicalEvent(
                    event_id=f"race:finish:{driver_id}",
                    event_type="driver_finished",
                    logical_time_s=now,
                    driver_ids=(driver_id,),
                    payload=(("finish_position", len(self.finish_order)),),
                )
            )

    def _close_maneuvers_at_finish(self, now: float, events: list[LogicalEvent]) -> None:
        """Close unfinished reservations without rewriting accepted state."""

        for corridor_id in sorted(tuple(self.maneuvers)):
            maneuver = self.maneuvers[corridor_id]
            if maneuver.phase not in {"rejoin", "rejoin_complete"}:
                events.append(
                    LogicalEvent(
                        event_id=f"{corridor_id}:finish_abort",
                        event_type="attack_aborted",
                        logical_time_s=now,
                        driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                        payload=(
                            ("corridor_id", corridor_id),
                            ("reason_code", "race_finished"),
                            ("maneuver_phase", "yield_or_abort"),
                        ),
                    )
                )
                self.metrics["attack_aborted_count"] += 1
                self._record_attack_terminal(corridor_id)
            for driver_id in (maneuver.attacker_id, maneuver.defender_id):
                car = self.cars[driver_id]
                car.corridor_id = None
                car.corridor_side = None
                car.lateral_trajectory = None
                car.lateral_velocity_mps = 0.0
                car.lateral_acceleration_mps2 = 0.0
                car.maneuver = "normal"
                if car.incident_state is None:
                    car.visual_state = "normal"
            events.append(
                LogicalEvent(
                    event_id=f"{corridor_id}:rejoin_complete",
                    event_type="rejoin_complete",
                    logical_time_s=now,
                    driver_ids=(maneuver.attacker_id, maneuver.defender_id),
                    payload=(
                        ("corridor_id", corridor_id),
                        ("reason_code", "race_finished"),
                    ),
                )
            )
            self.maneuvers.pop(corridor_id, None)

    def _record_body_clearance_metrics(self) -> None:
        for ahead_id, behind_id in zip(self.order, self.order[1:]):
            ahead = self.cars[ahead_id]
            behind = self.cars[behind_id]
            longitudinal_gap = abs(ahead.line_distance_m - behind.line_distance_m)
            lateral_gap = abs(ahead.lateral_offset_m - behind.lateral_offset_m)
            active = self._pair_active(behind_id, ahead_id)
            if longitudinal_gap < self.geometry.car_length_m and lateral_gap < self.geometry.car_width_m + 0.15:
                if active is None or lateral_gap < self.geometry.car_width_m + 0.15:
                    self.metrics["body_overlap_count"] += 1

    def _accepted_lateral_at(self, car: _MutableCar, logical_time_s: float) -> float:
        if car.lateral_trajectory is None:
            return car.lateral_offset_m
        return car.lateral_trajectory.offset_at(logical_time_s)

    def _recompute_following_steps_before_commit(
        self,
        steps: dict[int | str, KinematicStep],
        next_time: float,
    ) -> None:
        """Re-evaluate unsafe next speeds before any state is committed.

        This is part of the accepted-step calculation, not a progress clamp:
        the returned ``KinematicStep`` owns distance, speed and acceleration
        together and is the only value written to the cars.
        """

        for behind_id in self.order[1:]:
            order_index = self.order.index(behind_id)
            immediate_id = self.order[order_index - 1]
            immediate_pair = self._pair_active(behind_id, immediate_id)
            if (
                immediate_pair is not None
                and immediate_pair.attacker_id == behind_id
                and immediate_pair.phase
                in {"pull_out", "overlap", "crossing_confirmed"}
                and abs(
                    self._accepted_lateral_at(self.cars[behind_id], next_time)
                    - self._accepted_lateral_at(self.cars[immediate_id], next_time)
                )
                < self.geometry.car_width_m + BODY_CLEARANCE_MARGIN_M
            ):
                # Until the two bodies have lateral clearance, the defender
                # remains the attacker's pre-commit longitudinal boundary.
                ahead_id = immediate_id
            else:
                ahead_id = self._following_ahead_id(behind_id)
            if ahead_id is None:
                continue
            ahead = self.cars[ahead_id]
            behind = self.cars[behind_id]
            if ahead.line_distance_m <= behind.line_distance_m:
                continue
            ahead_step = steps[ahead_id]
            behind_step = steps[behind_id]
            predicted_gap = (
                ahead.line_distance_m + ahead_step.displacement_m
                - behind.line_distance_m - behind_step.displacement_m
            )
            predicted_closing_speed = max(
                0.0,
                behind_step.previous_speed_mps - ahead_step.speed_mps,
            )
            braking_distance = predicted_closing_speed * predicted_closing_speed / (2.0 * 50.0)
            anticipated_ahead_speed = min(
                ahead_step.speed_mps,
                max(0.0, ahead_step.target_speed_mps),
            )
            relative_stop_distance = max(
                0.0,
                (
                    behind_step.previous_speed_mps * behind_step.previous_speed_mps
                    - anticipated_ahead_speed * anticipated_ahead_speed
                )
                / (2.0 * 50.0),
            )
            front_braking = (
                ahead_step.previous_speed_mps - anticipated_ahead_speed > 0.25
            )
            minimum_gap_m, reaction_time_s = self._following_envelope(
                logical_time_s=next_time,
                front_braking=front_braking,
            )
            reaction_buffer = (
                max(behind_step.previous_speed_mps, ahead_step.speed_mps)
                * reaction_time_s
            )
            required_gap = (
                self.geometry.car_length_m
                + minimum_gap_m
                + max(braking_distance, relative_stop_distance)
                + reaction_buffer
            )
            if predicted_gap >= required_gap - 1e-9:
                continue
            behind_target = min(behind_step.target_speed_mps, anticipated_ahead_speed)
            allowed_displacement = max(
                0.0,
                ahead.line_distance_m + ahead_step.displacement_m
                - behind.line_distance_m
                - required_gap,
            )
            previous_speed = behind_step.previous_speed_mps
            next_speed = max(
                0.0,
                2.0 * allowed_displacement / self.tick_seconds - previous_speed,
            )
            next_speed = max(
                next_speed,
                max(0.0, previous_speed - 50.0 * self.tick_seconds),
            )
            next_speed = min(next_speed, behind_target)
            next_speed = max(0.0, max(next_speed, previous_speed - 50.0 * self.tick_seconds))
            acceleration = (next_speed - previous_speed) / self.tick_seconds
            displacement = (previous_speed + next_speed) * 0.5 * self.tick_seconds
            steps[behind_id] = KinematicStep(
                distance_m=behind.line_distance_m + displacement,
                speed_mps=next_speed,
                longitudinal_acceleration_mps2=acceleration,
                displacement_m=displacement,
                target_speed_mps=behind_step.target_speed_mps,
                previous_distance_m=behind.line_distance_m,
                previous_speed_mps=previous_speed,
                integrator_adjusted=False,
                adjustment_reason="precommit_following_replan",
            )

    def _recompute_world_chord_before_commit(
        self,
        driver_id: int | str,
        step: KinematicStep,
        next_time: float,
    ) -> KinematicStep:
        """Keep the world-space tick envelope part of the accepted step.

        The arc-length integrator is authoritative for distance.  The
        compiled pose sampler can still produce a slightly longer Euclidean
        chord on a tight curve, so the next speed is re-evaluated before the
        step is committed.  No x/y or distance value is clamped afterwards.
        """

        car = self.cars[driver_id]
        if (
            self._standing_start_active
            and next_time
            <= GRID_HOLD_DURATION_S + GRID_LAUNCH_FOLLOWING_BLEND_DURATION_S
            + 1e-9
            and step.speed_mps <= 1e-9
        ):
            # A stopped accepted step also pauses any lateral trajectory in
            # ``_refresh_lateral_states``.  Returning it unchanged prevents
            # the chord guard from treating a lateral-only preview as motion
            # and trapping the vehicle at zero speed on coarse tick sizes.
            return step
        previous_pose = self.synthesizer.pose_at_line_distance(
            driver_id,
            car.line_distance_m,
            simulation_time_s=self.logical_time_s,
            physics_frame=self.tick_index,
            speed_mps=car.speed_mps,
            lateral_offset_m=car.lateral_offset_m,
            plan=car.kinematic_plan,
        )
        next_lateral = self._accepted_lateral_at(car, next_time)
        chord_limit = car.speed_mps * self.tick_seconds + 0.5

        def candidate(next_speed: float) -> tuple[KinematicStep, float]:
            bounded_speed = min(step.speed_mps, max(0.0, next_speed))
            acceleration = (bounded_speed - car.speed_mps) / self.tick_seconds
            displacement = (car.speed_mps + bounded_speed) * 0.5 * self.tick_seconds
            candidate_step = KinematicStep(
                distance_m=car.line_distance_m + displacement,
                speed_mps=bounded_speed,
                longitudinal_acceleration_mps2=acceleration,
                displacement_m=displacement,
                target_speed_mps=step.target_speed_mps,
                previous_distance_m=car.line_distance_m,
                previous_speed_mps=car.speed_mps,
                integrator_adjusted=False,
                adjustment_reason="precommit_world_chord_replan",
            )
            next_pose = self.synthesizer.pose_at_line_distance(
                driver_id,
                candidate_step.distance_m,
                simulation_time_s=next_time,
                physics_frame=self.tick_index + 1,
                speed_mps=bounded_speed,
                longitudinal_acceleration_mps2=acceleration,
                lateral_offset_m=next_lateral,
                plan=car.kinematic_plan,
            )
            from math import hypot

            chord = hypot(
                next_pose.world_x_m - previous_pose.world_x_m,
                next_pose.world_y_m - previous_pose.world_y_m,
            )
            return candidate_step, chord

        if candidate(step.speed_mps)[1] <= chord_limit + 1e-9:
            return step

        low_speed = max(0.0, car.speed_mps - 50.0 * self.tick_seconds)
        low_step, low_chord = candidate(low_speed)
        if low_chord > chord_limit + 1e-9:
            # A lower accepted speed is still the only safe deterministic
            # choice if a sampler segment is unusually curved.
            self.metrics["world_chord_precommit_replan_count"] += 1
            return low_step
        high_speed = step.speed_mps
        accepted = low_step
        for _ in range(32):
            middle = (low_speed + high_speed) * 0.5
            middle_step, middle_chord = candidate(middle)
            if middle_chord <= chord_limit + 1e-9:
                accepted = middle_step
                low_speed = middle
            else:
                high_speed = middle
        self.metrics["world_chord_precommit_replan_count"] += 1
        return accepted
    def _build_frame(self) -> AbstractRaceFrame:
        vehicles: list[RaceVehicleState] = []
        for position, driver_id in enumerate(self.order, start=1):
            car = self.cars[driver_id]
            ahead_id = self.order[position - 2] if position > 1 else None
            ahead = self.cars[ahead_id] if ahead_id is not None else None
            gap_m = max(0.0, ahead.line_distance_m - car.line_distance_m) if ahead else 0.0
            interval_s = gap_m / max(1.0, ahead.speed_mps) if ahead else 0.0
            pose = self.synthesizer.pose_at_line_distance(
                driver_id,
                car.line_distance_m,
                simulation_time_s=self.logical_time_s,
                physics_frame=self.tick_index,
                speed_mps=car.speed_mps,
                longitudinal_acceleration_mps2=(car.last_step.longitudinal_acceleration_mps2 if car.last_step else 0.0),
                lateral_offset_m=car.lateral_offset_m,
                lateral_velocity_mps=car.lateral_velocity_mps,
                lateral_acceleration_mps2=car.lateral_acceleration_mps2,
                visual_state=car.visual_state,
                plan=car.kinematic_plan,
            )
            segment = self.kernel._segment_at_progress(car.total_progress)
            vehicles.append(
                RaceVehicleState(
                    driver_id=driver_id,
                    vehicle_id=car.entry.vehicle_id,
                    position=position,
                    total_progress=car.total_progress,
                    progress=pose.progress,
                    lap_number=pose.lap_number,
                    sector_id=segment.sector_id,
                    progress_rate_per_s=car.speed_mps / max(self.distance_contract.line_length_m, 1e-9),
                    gap_to_ahead_m=gap_m,
                    interval_to_ahead_s=interval_s,
                    ahead_driver_id=ahead_id,
                    lateral_offset_m=car.lateral_offset_m,
                    visual_state=car.visual_state,
                    maneuver=car.maneuver,
                    corridor_id=car.corridor_id,
                    corridor_side=car.corridor_side,
                    world_x_m=pose.world_x_m,
                    world_y_m=pose.world_y_m,
                    heading_rad=pose.heading_rad,
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
                    damage_level=car.damage_level,
                    incident_state=car.incident_state,
                    reliability_state="nominal",
                    line_distance_m=car.line_distance_m,
                    speed_mps=car.speed_mps,
                    longitudinal_acceleration_mps2=(car.last_step.longitudinal_acceleration_mps2 if car.last_step else 0.0),
                    lateral_velocity_mps=car.lateral_velocity_mps,
                    lateral_acceleration_mps2=car.lateral_acceleration_mps2,
                )
            )
        return AbstractRaceFrame(
            tick_index=self.tick_index,
            logical_time_s=self.logical_time_s,
            vehicles=tuple(vehicles),
        )

    def pose_for(self, driver_id: int | str):
        vehicle = next(item for item in self._current_frame.vehicles if item.driver_id == driver_id)
        return self.synthesizer.pose_at_line_distance(
            driver_id,
            vehicle.line_distance_m,
            simulation_time_s=self.logical_time_s,
            physics_frame=self.tick_index,
            speed_mps=vehicle.speed_mps,
            longitudinal_acceleration_mps2=vehicle.longitudinal_acceleration_mps2,
            lateral_offset_m=vehicle.lateral_offset_m,
            lateral_velocity_mps=vehicle.lateral_velocity_mps,
            lateral_acceleration_mps2=vehicle.lateral_acceleration_mps2,
            visual_state=vehicle.visual_state,
            plan=self.cars[driver_id].kinematic_plan,
        )

    def advance_one_tick(self) -> AbstractTrafficTick:
        if self._disposed:
            raise RuntimeError("traffic cursor is disposed")
        if self._finished:
            raise RuntimeError("traffic cursor is already finished")
        if self.tick_index >= self.max_ticks:
            raise RuntimeError("abstract traffic exceeded max_ticks")
        self._forecast_target_cache.clear()
        previous_offsets = {
            driver_id: self.cars[driver_id].lateral_offset_m
            for driver_id in self.cars
        }
        next_tick = self.tick_index + 1
        next_time = next_tick * self.tick_seconds
        steps = {
            driver_id: self._accepted_step(driver_id)
            for driver_id in sorted(self.cars, key=str)
        }
        self._recompute_following_steps_before_commit(steps, next_time)
        for driver_id in sorted(self.cars, key=str):
            steps[driver_id] = self._recompute_world_chord_before_commit(
                driver_id,
                steps[driver_id],
                next_time,
            )
        for driver_id in sorted(self.cars, key=str):
            car = self.cars[driver_id]
            step = steps[driver_id]
            car.line_distance_m = step.distance_m
            car.speed_mps = step.speed_mps
            car.last_step = step
            car.total_progress = self.distance_contract.total_progress_at_distance(step.distance_m)
        events_before = len(self.events)
        self.kernel._advance_pit_phases(
            logical_time_s=next_time,
            cars=self.cars,
            tick_seconds=self.tick_seconds,
            events=self.events,
        )
        self.kernel._advance_incident_states(
            logical_time_s=next_time,
            cars=self.cars,
            events=self.events,
        )
        self._smooth_incident_offsets(previous_offsets)
        for driver_id in sorted(self.cars, key=str):
            car = self.cars[driver_id]
            wear_factor = {
                "CONSERVE": 0.97,
                "STANDARD": 1.0,
                "ATTACK": 1.04,
            }[car.pace_mode]
            self.kernel._update_tire_state(
                car,
                steps[driver_id].displacement_m
                / max(self.distance_contract.line_length_m, 1e-9)
                * wear_factor,
                self.total_laps,
            )
        self.kernel._schedule_pit_stops(
            tick=next_tick,
            logical_time_s=next_time,
            cars=self.cars,
            events=self.events,
        )
        self.kernel._evaluate_lockups(
            tick=next_tick,
            logical_time_s=next_time,
            cars=self.cars,
            incident_streams=self.incident_streams,
            events=self.events,
        )
        self._smooth_incident_offsets(previous_offsets)
        self.tick_index = next_tick
        self.logical_time_s = next_time
        self._refresh_lateral_states(next_time)
        self._update_maneuvers(next_time, self.events)
        self._schedule_maneuvers(next_time, self.events)
        self._record_body_clearance_metrics()
        self._mark_finished(next_time, self.events)
        if len(self.finish_order) == len(self.order) and self.maneuvers:
            self._close_maneuvers_at_finish(next_time, self.events)
        checkpoint_laps = {
            driver_id: min(
                self.total_laps,
                int(max(0.0, floor(self.cars[driver_id].total_progress))),
            )
            for driver_id in self.order
        }
        if any(
            checkpoint_laps[driver_id] > self.last_checkpoint_laps[driver_id]
            for driver_id in self.order
        ):
            lap_number = max(checkpoint_laps.values())
            self.timing_checkpoints.append(
                self._checkpoint(
                    f"lap:{lap_number}:tick:{next_tick}",
                    "lap_boundary",
                    lap_number,
                )
            )
            self.last_checkpoint_laps = checkpoint_laps
        if len(self.finish_order) == len(self.order):
            self.events.append(
                LogicalEvent(
                    event_id="race:finished",
                    event_type="race_finished",
                    logical_time_s=next_time,
                    payload=(("driver_count", len(self.finish_order)),),
                )
            )
            self.timing_checkpoints.append(
                self._checkpoint("race:finish", "race_finish", self.total_laps)
            )
            self._finished = True
        self._current_frame = self._build_frame()
        active_reservations = self.active_reservations
        frame_audit = audit_accepted_frame_reservations(
            self._current_frame,
            active_reservations,
            line_length_m=self.distance_contract.line_length_m,
            vehicle_length_m=self.geometry.car_length_m,
            vehicle_width_m=self.geometry.car_width_m,
            vehicle_target_speed_mps_by_id={
                driver_id: self._forecast_target_speed_mps(
                    self.cars[driver_id],
                    max(
                        0.0,
                        max(
                            (
                                reservation.expiry_time_s - reservation.start_time_s
                                for reservation in active_reservations
                            ),
                            default=0.0,
                        ),
                    ),
                )
                for driver_id in sorted(self.cars, key=str)
            },
        )
        for metric_name in (
            "admitted_reservation_conflict_count",
            "third_vehicle_occupancy_conflict_count",
            "reservation_body_clearance_violation_count",
            "same_lane_longitudinal_overlap_count",
        ):
            self.metrics[metric_name] += frame_audit[metric_name]
        self.metrics["corridor_conflict_count"] += (
            frame_audit["admitted_reservation_conflict_count"]
            + frame_audit["third_vehicle_occupancy_conflict_count"]
            + frame_audit["reservation_body_clearance_violation_count"]
        )
        self.metrics["minimum_body_clearance_m"] = min(
            self.metrics["minimum_body_clearance_m"],
            frame_audit["minimum_body_clearance_m"],
        )
        self.metrics["minimum_corridor_clearance_m"] = min(
            self.metrics["minimum_corridor_clearance_m"],
            frame_audit["minimum_corridor_clearance_m"],
        )
        new_events = tuple(self.events[events_before:])
        new_checkpoints = tuple(
            checkpoint for checkpoint in self.timing_checkpoints
            if checkpoint.tick_index == next_tick
        )
        return AbstractTrafficTick(
            self._current_frame,
            new_events,
            new_checkpoints,
            active_reservations=active_reservations,
        )

    def finalize(self) -> AbstractRaceResult:
        if self._disposed:
            raise RuntimeError("traffic cursor is disposed")
        if not self._finished:
            raise RuntimeError("traffic cursor has not finished")
        if not self._finalized:
            self._finalized = True
        return AbstractRaceResult(
            session_id=self.snapshot.session_id,
            session_seed=self.snapshot.session_seed,
            content_version=self.snapshot.content_version,
            ruleset_version=self.snapshot.ruleset_version,
            abstract_engine_version=self.snapshot.abstract_engine_version,
            snapshot_hash=self.snapshot.snapshot_hash,
            total_laps=self.total_laps,
            grid_order=self.initial_grid_order,
            finish_order=tuple(self.finish_order),
            logical_events=tuple(self.events),
            timing_checkpoints=tuple(self.timing_checkpoints),
            command_log=tuple(self.command_log),
            logical_tick_count=self.tick_index,
        )

    def dispose(self) -> None:
        self._disposed = True
        self.cars.clear()
        self.order.clear()
        self.maneuvers.clear()
        self.pair_next_attack_eligible_time_s.clear()
        self.next_attack_eligible_time_s.clear()
        self.events.clear()
        self.timing_checkpoints.clear()
        self.command_log.clear()
        self.finish_order.clear()
        self._terminal_corridor_ids.clear()
        self._current_frame = None  # type: ignore[assignment]


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
    return AbstractRaceEngine(snapshot).run(
        grid_order=grid_order,
        total_laps=total_laps,
        tick_seconds=tick_seconds,
        max_ticks=max_ticks,
        pit_strategy=pit_strategy,
        frame_sink=frame_sink,
    )
