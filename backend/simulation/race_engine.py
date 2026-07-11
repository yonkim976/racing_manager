"""Core race simulation engine."""

from __future__ import annotations

from dataclasses import dataclass
import random

from models.schemas import (
    Circuit,
    Driver,
    DriverPositionInfo,
    DriverRaceState,
    LapTimeInfo,
    PaceMode,
    RaceEvent,
    RaceTickState,
    Team,
    TireCompound,
    TrackSegmentType,
)
from simulation.ai_strategy import choose_pit_tire, should_pit
from simulation.events import roll_events
from simulation.incidents import (
    Incident,
    IncidentCause,
    IncidentSeverity,
    escalate_collision,
    roll_solo_incident,
)
from simulation.physics import (
    GAME_TICK_SECONDS,
    car_performance_multiplier,
    compute_effective_lap_time,
    compute_progress_delta,
    driver_pace_multiplier,
)
from simulation.pit_stop import compute_pit_components
from simulation.speed_profile import SpeedProfile, build_speed_profile
from simulation.track_geometry import progress_range_length, segment_at_progress
from simulation.tire_model import compute_managed_tire_age, compute_tire_performance, compute_wear

PACE_MODE_EFFECTS: dict[PaceMode, dict[str, float]] = {
    PaceMode.CONSERVE: {"lap_time_delta": 0.55, "tire_usage_multiplier": 0.82},
    PaceMode.STANDARD: {"lap_time_delta": 0.0, "tire_usage_multiplier": 1.0},
    PaceMode.ATTACK: {"lap_time_delta": -0.45, "tire_usage_multiplier": 1.28},
}
PACE_MODE_ATTACK_FACTORS: dict[PaceMode, float] = {
    PaceMode.CONSERVE: 0.55,
    PaceMode.STANDARD: 0.85,
    PaceMode.ATTACK: 1.15,
}
TRACK_SEGMENT_SPEED_FACTORS: dict[TrackSegmentType, float] = {
    TrackSegmentType.STRAIGHT: 1.18,
    TrackSegmentType.SWEEPING: 1.05,
    TrackSegmentType.HEAVY_BRAKING: 0.68,
    TrackSegmentType.TECHNICAL: 0.78,
    TrackSegmentType.TRACTION: 0.88,
}
RACING_ACCELERATION_MPS2 = 12.0
RACING_BRAKING_MPS2 = 34.0
MIN_RACING_SPEED_MPS = 25.0
MAX_RACING_SPEED_MPS = 100.0
DRS_SPEED_MULTIPLIER = 1.04
DRS_MAX_RACING_SPEED_MPS = 105.0
GRID_SLOT_PROGRESS_GAP = 0.0028
VSC_DURATION_SECONDS = 25.0
VSC_LAP_TIME_FACTOR = 1.4
SC_LAP_TIME_FACTOR = 1.8
SC_CATCH_UP_FAST_LAP_TIME_FACTOR = 1.08
SC_CATCH_UP_NEAR_LAP_TIME_FACTOR = 1.25
SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR = 1.55
SC_UNLAP_LAP_TIME_FACTOR = 1.1
SC_LEAD_PROGRESS_GAP = 0.012  # lap-fraction gap between the SC and the on-track leader
SC_CAR_LENGTH_M = 5.6
SC_MAX_GAP_CAR_LENGTHS = 10.0
SC_QUEUE_TARGET_CAR_LENGTHS = 7.0
SC_DEPLOY_PIT_SECONDS = 2.5
SC_WITHDRAW_PIT_SECONDS = 3.0
SC_CLEANUP_SECONDS = 35.0
SC_ADDITIONAL_INCIDENT_SECONDS = 18.0
SC_PIT_WEAR_THRESHOLD = 0.30  # AI takes the "free" SC pit once tires are this worn
SC_PIT_PROBABILITY = 0.4  # per-eligible-driver chance to dive in under SC (avoids all-stop)
PIT_BOX_LANE_FRACTION = 0.5  # where along the pit lane (0..1) the box / stop sits
PIT_LANE_SPEED_LIMIT_KPH = 60.0
PROGRESS_EPSILON = 1e-9
INCIDENT_MINOR_TIME_PENALTY = (1.5, 4.0)
INCIDENT_MINOR_TIRE_USAGE = 0.01
TRAFFIC_GAP_SECONDS = 1.0
DIRTY_AIR_MAX_PENALTY = 0.32
TRAFFIC_ATTACK_MAX_BONUS = 0.12
DRS_MAX_BONUS = 0.30
DEFENSE_MAX_BLOCK = 0.22
BATTLE_EVENT_GAP_SECONDS = 0.80
BATTLE_EVENT_COOLDOWN_SECONDS = 14.0
BATTLE_EVENT_SEGMENTS = {
    TrackSegmentType.STRAIGHT,
    TrackSegmentType.HEAVY_BRAKING,
}
BATTLE_ATTACK_EFFECT_SECONDS = 4.5
BATTLE_DEFEND_EFFECT_SECONDS = 3.5
BATTLE_LOCKUP_EFFECT_SECONDS = 4.0
BATTLE_ATTACKER_LAP_TIME_DELTA = -3.2
BATTLE_DEFENDER_PRESSURE_LAP_TIME_DELTA = 1.15
BATTLE_DEFEND_ATTACKER_LAP_TIME_DELTA = 2.35
BATTLE_LOCKUP_LAP_TIME_DELTA = 5.5
BATTLE_ATTACK_TIRE_USAGE_MULTIPLIER = 1.08
BATTLE_DEFEND_TIRE_USAGE_MULTIPLIER = 1.04
BATTLE_LOCKUP_TIRE_USAGE_MULTIPLIER = 1.20
BATTLE_SIDE_BY_SIDE_SCORE_MARGIN = 0.06
BATTLE_FORCED_WIDE_SCORE_MARGIN = 0.18
BATTLE_SIDE_BY_SIDE_EFFECT_SECONDS = 6.0
BATTLE_SIDE_BY_SIDE_BREAK_GAP_SECONDS = 1.25
BATTLE_CORNER_EXIT_EFFECT_SECONDS = 2.5
BATTLE_FORCED_WIDE_EFFECT_SECONDS = 4.0
BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA = 0.45
BATTLE_INSIDE_ENTRY_LAP_TIME_DELTA = -0.15
BATTLE_OUTSIDE_ENTRY_LAP_TIME_DELTA = 0.12
BATTLE_INSIDE_EXIT_LAP_TIME_DELTA = 0.75
BATTLE_OUTSIDE_EXIT_LAP_TIME_DELTA = -0.55
BATTLE_DEFENSIVE_LINE_LAP_TIME_DELTA = 0.18
BATTLE_DEFENSIVE_LINE_EXIT_LAP_TIME_DELTA = 0.35
BATTLE_TRACTION_LOSS_EFFECT_SECONDS = 3.0
BATTLE_RUN_WIDE_EFFECT_SECONDS = 4.0
BATTLE_MINOR_CONTACT_EFFECT_SECONDS = 5.0
BATTLE_TRACTION_LOSS_LAP_TIME_DELTA = 1.4
BATTLE_RUN_WIDE_LAP_TIME_DELTA = 2.6
BATTLE_MINOR_CONTACT_ATTACKER_LAP_TIME_DELTA = 3.6
BATTLE_MINOR_CONTACT_DEFENDER_LAP_TIME_DELTA = 2.6
BATTLE_FORCED_WIDE_ATTACKER_LAP_TIME_DELTA = -2.2
BATTLE_FORCED_WIDE_DEFENDER_LAP_TIME_DELTA = 3.2
BATTLE_FORCED_WIDE_EXIT_DELAY_SECONDS = 1.8
BATTLE_FORCED_WIDE_RESULT_BASE_PROBABILITY = 0.42
BATTLE_FORCED_WIDE_ATTACKER_EXIT_LAP_TIME_DELTA = 1.25
BATTLE_FORCED_WIDE_ATTACKER_EXIT_EFFECT_SECONDS = 2.5
BATTLE_SIDE_BY_SIDE_TIRE_USAGE_MULTIPLIER = 1.06
BATTLE_INSIDE_LINE_TIRE_USAGE_MULTIPLIER = 1.02
BATTLE_OUTSIDE_LINE_TIRE_USAGE_MULTIPLIER = 1.01
BATTLE_DEFENSIVE_LINE_TIRE_USAGE_MULTIPLIER = 1.02
BATTLE_TRACTION_LOSS_TIRE_USAGE_MULTIPLIER = 1.06
BATTLE_RUN_WIDE_TIRE_USAGE_MULTIPLIER = 1.08
BATTLE_MINOR_CONTACT_TIRE_USAGE_MULTIPLIER = 1.12
BATTLE_FORCED_WIDE_ATTACKER_EXIT_TIRE_USAGE_MULTIPLIER = 1.04
BATTLE_FORCED_WIDE_TIRE_USAGE_MULTIPLIER = 1.10
BATTLE_SIDE_BY_SIDE_RESULT_BASE_PROBABILITY = 0.18
ATTACK_LINE_INSIDE = "inside"
ATTACK_LINE_OUTSIDE = "outside"
DEFENDER_LINE_RACING = "racing_line"
DEFENDER_LINE_DEFENSIVE = "defensive_line"
AI_PACE_MIN_COOLDOWN_SECONDS = 8.0
AI_PACE_MAX_COOLDOWN_SECONDS = 12.0
AI_ATTACK_GAP_SECONDS = 1.0
AI_DEFEND_GAP_SECONDS = 0.85
AI_LOW_TIRE_LIFE = 0.25
AI_CRITICAL_TIRE_LIFE = 0.12
AI_FRESH_TIRE_USAGE = 2.0


@dataclass
class BattleEffect:
    """Short local pace effect created by an overtake or defense event."""

    remaining_seconds: float
    lap_time_delta: float
    tire_usage_multiplier: float = 1.0


@dataclass
class SideBySideBattle:
    """A short-lived state where two adjacent cars are racing through a corner."""

    attacker_id: int
    defender_id: int
    remaining_seconds: float
    attacker_line: str
    defender_line: str
    segment_name: str = "the corner"


@dataclass
class ForcedWideAftermath:
    """A delayed corner-exit result after a forceful inside move."""

    attacker_id: int
    defender_id: int
    remaining_seconds: float
    segment_name: str = "the corner"


class RaceEngine:
    """Tick-based race simulation for a single session."""

    def __init__(
        self,
        circuit: Circuit,
        drivers: list[Driver],
        teams: dict[int, Team],
        player_team_id: int,
        player_driver_ids: list[int],
        starting_tires: dict[int, TireCompound] | None = None,
        grid_order: list[int] | None = None,
        seed: int | None = None,
    ):
        self.circuit = circuit
        self.drivers = {d.id: d for d in drivers}
        self.teams = teams
        self.player_team_id = player_team_id
        self.player_driver_ids = set(player_driver_ids)
        self.starting_tires = starting_tires or {}
        self.grid_order = grid_order or []
        self.rng = random.Random(seed)

        self.total_laps = circuit.total_laps
        self.current_lap = 1
        self.race_elapsed = 0.0
        self.weather = "dry"
        self.safety_car = False
        self.race_phase = "green"  # green | vsc | sc
        self._phase_until = 0.0  # race_elapsed deadline for VSC
        self.safety_car_stage = "inactive"
        self._safety_car_visible = False
        self._safety_car_route = "track"
        self._safety_car_pit_lane_progress = 0.0
        self._safety_car_queue_formed = False
        self._sc_cleanup_until = 0.0
        self._sc_caught_driver_ids: set[int] = set()
        self._sc_unlap_driver_ids: set[int] = set()
        self._sc_unlap_targets: dict[int, float] = {}
        self._sc_withdraw_target: float | None = None
        self._sc_restart_target: float | None = None
        self._sc_restart_accel_progress: float | None = None
        self.pit_window_open = False
        self.finished = False
        self.speed_multiplier = 1
        self.paused = False
        self.track_length_m = max(1.0, float(circuit.track_length_m))
        self._speed_profile: SpeedProfile | None = build_speed_profile(
            circuit,
            acceleration_mps2=RACING_ACCELERATION_MPS2,
            braking_mps2=RACING_BRAKING_MPS2,
        )
        self._segment_speed_normalizer = self._compute_segment_speed_normalizer()

        self.driver_states: dict[int, DriverRaceState] = {}
        self._lap_random: dict[int, float] = {}
        self._tire_random: dict[int, float] = {}
        self._progress_rate: dict[int, float] = {}
        self._safety_car_total_progress: float | None = None
        self._safety_car_progress_rate = 0.0
        # Multi-phase pit stop state (keyed by driver_id while in_pit).
        self._pit_phase: dict[int, str] = {}  # "in" | "stop" | "out"
        self._pit_phase_remaining: dict[int, float] = {}
        self._pit_phase_duration: dict[int, float] = {}
        self._pit_lane_half_time: dict[int, float] = {}
        self._pit_tire_change_time: dict[int, float] = {}
        self._pit_elapsed: dict[int, float] = {}
        self._pit_stop_elapsed: dict[int, float] = {}
        self._pit_tire: dict[int, TireCompound] = {}
        self._pit_entry_compound: dict[int, TireCompound] = {}
        self._lap_history: dict[int, list[LapTimeInfo]] = {}
        self._driver_meta: dict[int, dict] = {}
        self._finish_order: list[int] = []
        self._battle_event_cooldown: dict[int, float] = {}
        self._battle_effects: dict[int, BattleEffect] = {}
        self._side_by_side_battles: dict[tuple[int, int], SideBySideBattle] = {}
        self._forced_wide_aftermaths: dict[tuple[int, int], ForcedWideAftermath] = {}
        self._pending_incidents: list[Incident] = []
        self._ai_pace_cooldown: dict[int, float] = {}

        self._init_grid(drivers)

    def _init_grid(self, drivers: list[Driver]) -> None:
        """Initialize driver states from a supplied grid or simulated qualifying pace."""
        qualifying = []
        for driver in drivers:
            team = self.teams[driver.team_id]
            pace = driver_pace_multiplier(driver.stats.pace)
            car_performance = car_performance_multiplier(team.car_performance)
            score = car_performance * pace + self.rng.uniform(-0.005, 0.005)
            qualifying.append((score, driver))

        qualifying.sort(key=lambda x: x[0], reverse=True)
        fallback_order = [driver for _, driver in qualifying]
        driver_by_id = {driver.id: driver for driver in drivers}
        seen_driver_ids: set[int] = set()
        ordered_drivers = []

        for driver_id in self.grid_order:
            driver = driver_by_id.get(driver_id)
            if driver is None or driver.id in seen_driver_ids:
                continue
            ordered_drivers.append(driver)
            seen_driver_ids.add(driver.id)

        for driver in fallback_order:
            if driver.id in seen_driver_ids:
                continue
            ordered_drivers.append(driver)
            seen_driver_ids.add(driver.id)

        for position, driver in enumerate(ordered_drivers, start=1):
            team = self.teams[driver.team_id]
            starting_compound = self.starting_tires.get(driver.id, TireCompound.MEDIUM)
            grid_offset = (position - 1) * GRID_SLOT_PROGRESS_GAP
            self.driver_states[driver.id] = DriverRaceState(
                driver_id=driver.id,
                position=position,
                progress=-grid_offset,
                current_lap=0,
                total_progress=-grid_offset,
                tire_compound=starting_compound,
                tire_age=0,
                tire_wear=0.0,
                gap_to_leader=0.0,
                in_pit=False,
                pit_count=0,
                retired=False,
                total_time=0.0,
                speed_kph=self._initial_speed_kph(-grid_offset),
                pace_mode=PaceMode.STANDARD,
            )
            self._driver_meta[driver.id] = {
                "abbreviation": driver.abbreviation,
                "full_name": driver.name,
                "team_name": team.name,
                "team_color": team.color,
                "car_performance": car_performance_multiplier(team.car_performance),
                "pit_crew_skill": team.pit_crew_skill,
                "pace": driver_pace_multiplier(driver.stats.pace),
                "consistency": driver.stats.consistency,
                "tire_management": driver.stats.tire_management,
                "overtaking": driver.stats.overtaking,
                "defending": driver.stats.defending,
            }
            self._lap_history[driver.id] = []
            self._refresh_lap_variation(driver.id)
            setattr(self.driver_states[driver.id], "_lap_start_time", 0.0)

    def _consistency_variation_spread(self, consistency: float) -> float:
        """Return per-lap random swing from driver consistency."""
        return 0.10 + (1.0 - consistency) * 1.35

    def _segment_base_speed_factor(self, segment) -> float:
        factor = getattr(segment, "speed_factor", None)
        if factor is not None:
            return factor
        return TRACK_SEGMENT_SPEED_FACTORS.get(segment.type, 1.0)

    def _compute_segment_speed_normalizer(self) -> float:
        """Normalize local segment speeds so their lap average stays near target pace."""
        if not self.circuit.segments:
            return 1.0

        weighted_time = 0.0
        covered_progress = 0.0
        for segment in self.circuit.segments:
            length = progress_range_length(segment.start, segment.end)
            factor = self._segment_base_speed_factor(segment)
            if length <= 0 or factor <= 0:
                continue
            covered_progress += length
            weighted_time += length / factor

        missing_progress = max(0.0, 1.0 - covered_progress)
        weighted_time += missing_progress
        if weighted_time <= 0:
            return 1.0
        return weighted_time

    def _segment_speed_factor_at_progress(self, progress: float) -> float:
        segment = segment_at_progress(self.circuit, progress)
        if segment is None:
            return 1.0
        return self._segment_base_speed_factor(segment) * self._segment_speed_normalizer

    def _speed_factor_at_progress(self, progress: float) -> float:
        if self._speed_profile is not None:
            return self._speed_profile.factor_at_progress(progress)
        return self._segment_speed_factor_at_progress(progress)

    def _average_speed_mps(self, lap_time: float) -> float:
        return self.track_length_m / max(1.0, lap_time)

    def _clamp_racing_speed(self, speed_mps: float, *, drs_active: bool = False) -> float:
        max_speed = DRS_MAX_RACING_SPEED_MPS if drs_active else MAX_RACING_SPEED_MPS
        return min(max_speed, max(MIN_RACING_SPEED_MPS, speed_mps))

    def _initial_speed_kph(self, progress: float) -> float:
        speed_mps = self._average_speed_mps(self.circuit.base_lap_time)
        speed_mps *= self._speed_factor_at_progress(progress)
        return round(self._clamp_racing_speed(speed_mps) * 3.6, 3)

    def _target_speed_mps_for_lap_time(self, state: DriverRaceState, lap_time: float) -> float:
        speed_mps = self._average_speed_mps(lap_time)
        segment = segment_at_progress(self.circuit, state.progress)
        drs_speed_active = bool(
            state.drs_active and segment is not None and segment.type == TrackSegmentType.STRAIGHT
        )
        speed_mps *= self._speed_factor_at_progress(state.progress)
        if drs_speed_active:
            speed_mps *= DRS_SPEED_MULTIPLIER
        return self._clamp_racing_speed(speed_mps, drs_active=drs_speed_active)

    def _advance_speed_mps(self, current_mps: float, target_mps: float, delta: float) -> float:
        if delta <= 0:
            return current_mps
        rate = RACING_ACCELERATION_MPS2 if target_mps > current_mps else RACING_BRAKING_MPS2
        max_change = rate * delta
        diff = target_mps - current_mps
        if abs(diff) <= max_change:
            return target_mps
        return current_mps + max_change * (1 if diff > 0 else -1)

    def _distance_based_progress_delta(
        self,
        state: DriverRaceState,
        lap_time: float,
        delta: float,
    ) -> float:
        target_speed = self._target_speed_mps_for_lap_time(state, lap_time)
        current_speed = max(0.0, state.speed_kph / 3.6)
        speed_mps = self._advance_speed_mps(current_speed, target_speed, delta)
        state.speed_kph = round(speed_mps * 3.6, 3)
        if self.track_length_m <= 0:
            return compute_progress_delta(delta, lap_time)
        return (speed_mps * delta) / self.track_length_m

    def _pit_phase_speed_kph(self, driver_id: int) -> float:
        phase = self._pit_phase.get(driver_id)
        if phase in {"in", "out"}:
            return PIT_LANE_SPEED_LIMIT_KPH
        return 0.0

    def _dead_tire_lap_penalty(self, tire_wear: float) -> float:
        """Return extra lap-time loss as tire life approaches 0%."""
        dead_tire_ratio = max(0.0, (tire_wear - 0.75) / 0.25)
        return 1.35 * (dead_tire_ratio ** 1.7)

    def _pace_mode_effect(self, state: DriverRaceState) -> dict[str, float]:
        return PACE_MODE_EFFECTS.get(state.pace_mode, PACE_MODE_EFFECTS[PaceMode.STANDARD])

    def _pace_mode_attack_factor(self, state: DriverRaceState) -> float:
        return PACE_MODE_ATTACK_FACTORS.get(state.pace_mode, PACE_MODE_ATTACK_FACTORS[PaceMode.STANDARD])

    def _effective_tire_age(self, state: DriverRaceState) -> float:
        """Return driver-managed tire age from accumulated tire usage."""
        raw_age = state.tire_usage
        if raw_age <= 0 and state.tire_age > 0:
            raw_age = float(state.tire_age)
        tire_management = self._driver_meta[state.driver_id]["tire_management"]
        return compute_managed_tire_age(raw_age, tire_management)

    def _current_tire_wear(self, state: DriverRaceState) -> float:
        return compute_wear(state.tire_compound, self._effective_tire_age(state))

    def _base_lap_time_for_state(self, state: DriverRaceState) -> float:
        """Estimate lap time before traffic effects."""
        meta = self._driver_meta[state.driver_id]
        tire_perf = compute_tire_performance(
            state.tire_compound,
            self._effective_tire_age(state),
            self._tire_random[state.driver_id],
        )
        lap_time = compute_effective_lap_time(
            self.circuit.base_lap_time,
            meta["car_performance"],
            meta["pace"],
            tire_perf,
            self.rng,
            self._lap_random[state.driver_id],
        )
        pace_effect = self._pace_mode_effect(state)
        segment_delta = self._segment_lap_time_delta(state)
        return max(
            self.circuit.base_lap_time * 0.75,
            lap_time + pace_effect["lap_time_delta"] + segment_delta,
        )

    def _segment_lap_time_delta(self, state: DriverRaceState) -> float:
        """Return a small lap-time modifier for the current driving segment."""
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is None:
            return 0.0

        meta = self._driver_meta[state.driver_id]
        car_edge = meta["car_performance"] - 1.0
        pace_edge = meta["pace"] - 1.0
        consistency_loss = 1.0 - meta["consistency"]
        tire_management_loss = 1.0 - meta["tire_management"]
        tire_wear = self._current_tire_wear(state)

        if segment.type == TrackSegmentType.STRAIGHT:
            return -2.20 * car_edge - 0.60 * pace_edge
        if segment.type == TrackSegmentType.SWEEPING:
            return -0.80 * car_edge - 1.40 * pace_edge + 0.08 * tire_wear
        if segment.type == TrackSegmentType.HEAVY_BRAKING:
            return (
                -0.22 * (meta["overtaking"] - 0.85)
                - 0.80 * pace_edge
                + 0.18 * tire_wear
                + 0.05 * consistency_loss
            )
        if segment.type == TrackSegmentType.TECHNICAL:
            return -1.80 * pace_edge + 0.24 * tire_wear + 0.08 * consistency_loss
        if segment.type == TrackSegmentType.TRACTION:
            return -1.00 * pace_edge + 0.30 * tire_wear + 0.10 * tire_management_loss
        return 0.0

    def _running_by_position(self) -> dict[int, DriverRaceState]:
        return {
            state.position: state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        }

    def _is_drs_zone(self, progress: float) -> bool:
        """Return whether current track progress is inside a configured DRS zone."""
        normalized = progress % 1.0
        for zone in self.circuit.drs_zones:
            start = float(zone.start) % 1.0
            end = float(zone.end) % 1.0
            if start <= end and start <= normalized <= end:
                return True
            if start > end and (normalized >= start or normalized <= end):
                return True
        return False

    def _battle_lap_time_delta(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
    ) -> float:
        """Return traffic lap-time effect when following another car closely."""
        state.drs_active = False
        state.dirty_air_active = False
        if car_ahead is None or state.in_pit or state.retired or state.finished:
            return 0.0

        progress_gap = car_ahead.total_progress - state.total_progress
        gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
        if gap_seconds <= 0 or gap_seconds > TRAFFIC_GAP_SECONDS:
            return 0.0

        state.dirty_air_active = True
        state.drs_active = self._is_drs_zone(state.progress)
        attacker = self._driver_meta[state.driver_id]
        defender = self._driver_meta[car_ahead.driver_id]
        closeness = 0.35 + 0.65 * (1.0 - min(gap_seconds, TRAFFIC_GAP_SECONDS) / TRAFFIC_GAP_SECONDS)
        difficulty = self.circuit.overtaking_difficulty
        tire_life = max(0.0, 1.0 - self._current_tire_wear(state))

        dirty_air = DIRTY_AIR_MAX_PENALTY * closeness
        traffic_attack_bonus = (
            TRAFFIC_ATTACK_MAX_BONUS
            * attacker["overtaking"]
            * self._pace_mode_attack_factor(state)
            * (1.0 - 0.35 * difficulty)
            * (0.65 + 0.35 * tire_life)
            * closeness
        )
        drs_bonus = 0.0
        if state.drs_active:
            drs_bonus = (
                DRS_MAX_BONUS
                * attacker["overtaking"]
                * self._pace_mode_attack_factor(state)
                * (1.0 - 0.55 * difficulty)
                * (0.65 + 0.35 * tire_life)
                * closeness
            )
        defense_block = (
            DEFENSE_MAX_BLOCK
            * defender["defending"]
            * (0.45 + 0.55 * difficulty)
            * closeness
        )
        return dirty_air - traffic_attack_bonus - drs_bonus + defense_block

    def _battle_event_probability(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        gap_seconds: float,
    ) -> float:
        """Return chance of a visible attack/defend event on this tick."""
        if car_ahead is None or gap_seconds <= 0 or gap_seconds > BATTLE_EVENT_GAP_SECONDS:
            return 0.0
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is None or segment.type not in BATTLE_EVENT_SEGMENTS:
            return 0.0

        attacker = self._driver_meta[state.driver_id]
        closeness = 1.0 - min(gap_seconds, BATTLE_EVENT_GAP_SECONDS) / BATTLE_EVENT_GAP_SECONDS
        segment_base = 0.008 if segment.type == TrackSegmentType.STRAIGHT else 0.014
        drs_bonus = 0.012 if state.drs_active else 0.0
        attack_bonus = 0.006 * attacker["overtaking"] * self._pace_mode_attack_factor(state)
        difficulty_penalty = 0.010 * self.circuit.overtaking_difficulty
        return max(
            0.0,
            min(0.045, segment_base + drs_bonus + attack_bonus - difficulty_penalty + 0.012 * closeness),
        )

    def _battle_mistake_probability(self, state: DriverRaceState, gap_seconds: float) -> float:
        """Return chance that a failed high-pressure attack costs time."""
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is None or segment.type != TrackSegmentType.HEAVY_BRAKING:
            return 0.0

        meta = self._driver_meta[state.driver_id]
        closeness = 1.0 - min(gap_seconds, BATTLE_EVENT_GAP_SECONDS) / BATTLE_EVENT_GAP_SECONDS
        tire_wear = self._current_tire_wear(state)
        pace_risk = {
            PaceMode.CONSERVE: 0.00,
            PaceMode.STANDARD: 0.03,
            PaceMode.ATTACK: 0.08,
        }.get(state.pace_mode, 0.03)
        probability = (
            0.02
            + pace_risk
            + 0.08 * (1.0 - meta["consistency"])
            + 0.08 * tire_wear
            + 0.05 * closeness
        )
        return min(0.24, max(0.0, probability))

    def _set_battle_effect(
        self,
        driver_id: int,
        lap_time_delta: float,
        duration: float,
        tire_usage_multiplier: float = 1.0,
    ) -> None:
        current = self._battle_effects.get(driver_id)
        if current is not None and abs(current.lap_time_delta) > abs(lap_time_delta):
            current.remaining_seconds = max(current.remaining_seconds, duration)
            current.tire_usage_multiplier = max(current.tire_usage_multiplier, tire_usage_multiplier)
            return

        self._battle_effects[driver_id] = BattleEffect(
            remaining_seconds=duration,
            lap_time_delta=lap_time_delta,
            tire_usage_multiplier=tire_usage_multiplier,
        )

    def _battle_pair_key(self, driver_a_id: int, driver_b_id: int) -> tuple[int, int]:
        return tuple(sorted((driver_a_id, driver_b_id)))

    def _choose_attack_line(self, attacker_state: DriverRaceState) -> str:
        """Choose whether the attacking driver commits inside or around the outside."""
        attacker = self._driver_meta[attacker_state.driver_id]
        tire_life = max(0.0, 1.0 - self._current_tire_wear(attacker_state))
        inside_probability = (
            0.46
            + 0.18 * (attacker["overtaking"] - 0.85)
            + 0.10 * (self._pace_mode_attack_factor(attacker_state) - 0.85)
            + (0.06 if attacker_state.drs_active else 0.0)
            + 0.05 * (tire_life - 0.5)
        )
        inside_probability = min(0.76, max(0.32, inside_probability))
        return (
            ATTACK_LINE_INSIDE
            if self.rng.random() < inside_probability
            else ATTACK_LINE_OUTSIDE
        )

    def _choose_defender_line(
        self,
        defender_state: DriverRaceState,
        attacker_state: DriverRaceState,
    ) -> str:
        """Choose whether the defender protects the inside or stays on racing line."""
        defender = self._driver_meta[defender_state.driver_id]
        defensive_probability = (
            0.28
            + 0.45 * (defender["defending"] - 0.75)
            + 0.08 * (self._pace_mode_attack_factor(attacker_state) - 0.85)
            + (0.08 if attacker_state.drs_active else 0.0)
        )
        defensive_probability = min(0.82, max(0.22, defensive_probability))
        return (
            DEFENDER_LINE_DEFENSIVE
            if self.rng.random() < defensive_probability
            else DEFENDER_LINE_RACING
        )

    def _side_by_side_battle_for_driver(self, driver_id: int) -> SideBySideBattle | None:
        for battle in self._side_by_side_battles.values():
            if driver_id in (battle.attacker_id, battle.defender_id):
                return battle
        return None

    def _is_side_by_side_active(self, driver_id: int) -> bool:
        return self._side_by_side_battle_for_driver(driver_id) is not None

    def _clear_side_by_side_for_driver(self, driver_id: int) -> None:
        for key, battle in list(self._side_by_side_battles.items()):
            if driver_id in (battle.attacker_id, battle.defender_id):
                del self._side_by_side_battles[key]

    def _start_side_by_side_battle(
        self,
        attacker_id: int,
        defender_id: int,
        attacker_line: str = ATTACK_LINE_INSIDE,
        defender_line: str = DEFENDER_LINE_RACING,
        segment_name: str = "the corner",
    ) -> None:
        self._clear_side_by_side_for_driver(attacker_id)
        self._clear_side_by_side_for_driver(defender_id)
        key = self._battle_pair_key(attacker_id, defender_id)
        self._side_by_side_battles[key] = SideBySideBattle(
            attacker_id=attacker_id,
            defender_id=defender_id,
            remaining_seconds=BATTLE_SIDE_BY_SIDE_EFFECT_SECONDS,
            attacker_line=attacker_line,
            defender_line=defender_line,
            segment_name=segment_name,
        )

    def _side_by_side_gap_seconds(self, battle: SideBySideBattle) -> float:
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        progress_gap = abs(defender.total_progress - attacker.total_progress)
        trailing = attacker if attacker.total_progress <= defender.total_progress else defender
        return self._progress_gap_to_seconds(progress_gap, trailing)

    def _side_by_side_battle_is_valid(self, battle: SideBySideBattle) -> bool:
        attacker = self.driver_states.get(battle.attacker_id)
        defender = self.driver_states.get(battle.defender_id)
        if attacker is None or defender is None:
            return False
        if (
            attacker.retired
            or defender.retired
            or attacker.finished
            or defender.finished
            or attacker.in_pit
            or defender.in_pit
        ):
            return False
        if abs(attacker.position - defender.position) != 1:
            return False
        if attacker.position < defender.position:
            return False
        return self._side_by_side_gap_seconds(battle) <= BATTLE_SIDE_BY_SIDE_BREAK_GAP_SECONDS

    def _prune_side_by_side_battles(self) -> list[RaceEvent]:
        events: list[RaceEvent] = []
        for key, battle in list(self._side_by_side_battles.items()):
            is_valid = self._side_by_side_battle_is_valid(battle)
            if battle.remaining_seconds <= 0:
                if is_valid:
                    result_event = self._apply_side_by_side_exit_effect(battle)
                    if result_event is not None:
                        events.append(result_event)
                del self._side_by_side_battles[key]
            elif not is_valid:
                del self._side_by_side_battles[key]
        return events

    def _tick_side_by_side_battles(self, delta: float) -> list[RaceEvent]:
        for battle in self._side_by_side_battles.values():
            battle.remaining_seconds -= delta
        return self._prune_side_by_side_battles()

    def _side_by_side_lap_time_delta(self, state: DriverRaceState) -> float:
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if battle is None:
            return 0.0

        delta = BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA
        if state.driver_id == battle.attacker_id:
            if battle.attacker_line == ATTACK_LINE_INSIDE:
                delta += BATTLE_INSIDE_ENTRY_LAP_TIME_DELTA
            else:
                delta += BATTLE_OUTSIDE_ENTRY_LAP_TIME_DELTA
        elif battle.defender_line == DEFENDER_LINE_DEFENSIVE:
            delta += BATTLE_DEFENSIVE_LINE_LAP_TIME_DELTA
        return delta

    def _side_by_side_tire_usage_multiplier(self, state: DriverRaceState) -> float:
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if battle is None:
            return 1.0

        multiplier = BATTLE_SIDE_BY_SIDE_TIRE_USAGE_MULTIPLIER
        if state.driver_id == battle.attacker_id:
            if battle.attacker_line == ATTACK_LINE_INSIDE:
                multiplier *= BATTLE_INSIDE_LINE_TIRE_USAGE_MULTIPLIER
            else:
                multiplier *= BATTLE_OUTSIDE_LINE_TIRE_USAGE_MULTIPLIER
        elif battle.defender_line == DEFENDER_LINE_DEFENSIVE:
            multiplier *= BATTLE_DEFENSIVE_LINE_TIRE_USAGE_MULTIPLIER
        return multiplier

    def _apply_side_by_side_exit_effect(self, battle: SideBySideBattle) -> RaceEvent | None:
        if battle.attacker_line == ATTACK_LINE_INSIDE:
            self._set_battle_effect(
                battle.attacker_id,
                BATTLE_INSIDE_EXIT_LAP_TIME_DELTA,
                BATTLE_CORNER_EXIT_EFFECT_SECONDS,
                BATTLE_INSIDE_LINE_TIRE_USAGE_MULTIPLIER,
            )
        else:
            self._set_battle_effect(
                battle.attacker_id,
                BATTLE_OUTSIDE_EXIT_LAP_TIME_DELTA,
                BATTLE_CORNER_EXIT_EFFECT_SECONDS,
                BATTLE_OUTSIDE_LINE_TIRE_USAGE_MULTIPLIER,
            )

        if battle.defender_line == DEFENDER_LINE_DEFENSIVE:
            self._set_battle_effect(
                battle.defender_id,
                BATTLE_DEFENSIVE_LINE_EXIT_LAP_TIME_DELTA,
                BATTLE_CORNER_EXIT_EFFECT_SECONDS,
                BATTLE_DEFENSIVE_LINE_TIRE_USAGE_MULTIPLIER,
            )

        return self._maybe_side_by_side_result_event(battle)

    def _side_by_side_result_probability(self, battle: SideBySideBattle) -> float:
        attacker_state = self.driver_states[battle.attacker_id]
        defender_state = self.driver_states[battle.defender_id]
        attacker = self._driver_meta[battle.attacker_id]
        defender = self._driver_meta[battle.defender_id]
        tire_wear = max(
            self._current_tire_wear(attacker_state),
            self._current_tire_wear(defender_state),
        )
        pressure = BATTLE_SIDE_BY_SIDE_RESULT_BASE_PROBABILITY
        pressure += 0.10 * (1.0 - attacker["consistency"])
        pressure += 0.08 * (1.0 - defender["consistency"])
        pressure += 0.08 * tire_wear
        pressure += 0.06 if attacker_state.pace_mode == PaceMode.ATTACK else 0.0
        pressure += 0.08 if battle.attacker_line == ATTACK_LINE_OUTSIDE else 0.0
        pressure += 0.10 if battle.defender_line == DEFENDER_LINE_DEFENSIVE else 0.0
        return min(0.62, max(0.08, pressure))

    def _side_by_side_result_type(self, battle: SideBySideBattle) -> str:
        roll = self.rng.random()
        if battle.defender_line == DEFENDER_LINE_DEFENSIVE:
            if roll < 0.45:
                return "minor_contact"
            if roll < 0.75:
                return "run_wide"
            return "traction_loss"
        if battle.attacker_line == ATTACK_LINE_OUTSIDE:
            if roll < 0.55:
                return "traction_loss"
            if roll < 0.85:
                return "run_wide"
            return "minor_contact"
        if roll < 0.55:
            return "run_wide"
        if roll < 0.85:
            return "traction_loss"
        return "minor_contact"

    def _maybe_escalate_contact(
        self,
        attacker_id: int,
        defender_id: int,
        segment_type: str | None = None,
    ) -> bool:
        """Queue an incident if a light contact escalates. Returns True if escalated."""
        severity = escalate_collision(self.rng, segment_type=segment_type)
        if severity == IncidentSeverity.MINOR:
            return False
        self._pending_incidents.append(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=severity,
                primary_driver_id=attacker_id,
                secondary_driver_id=defender_id if severity == IncidentSeverity.CRASH else None,
            )
        )
        return True

    def _maybe_side_by_side_result_event(self, battle: SideBySideBattle) -> RaceEvent | None:
        if self.rng.random() >= self._side_by_side_result_probability(battle):
            return None

        result_type = self._side_by_side_result_type(battle)
        attacker = self._driver_meta[battle.attacker_id]
        defender = self._driver_meta[battle.defender_id]
        if result_type == "minor_contact":
            if self._maybe_escalate_contact(battle.attacker_id, battle.defender_id):
                return None
            self._set_battle_effect(
                battle.attacker_id,
                BATTLE_MINOR_CONTACT_ATTACKER_LAP_TIME_DELTA,
                BATTLE_MINOR_CONTACT_EFFECT_SECONDS,
                BATTLE_MINOR_CONTACT_TIRE_USAGE_MULTIPLIER,
            )
            self._set_battle_effect(
                battle.defender_id,
                BATTLE_MINOR_CONTACT_DEFENDER_LAP_TIME_DELTA,
                BATTLE_MINOR_CONTACT_EFFECT_SECONDS,
                BATTLE_MINOR_CONTACT_TIRE_USAGE_MULTIPLIER,
            )
            return RaceEvent(
                type="minor_contact",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} and {defender['full_name']} make light contact"
                    f" exiting {battle.segment_name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}와 {defender['full_name']}가"
                    f" {battle.segment_name} 탈출 구간에서 가볍게 접촉합니다"
                ),
            )

        self._set_battle_effect(
            battle.attacker_id,
            BATTLE_RUN_WIDE_LAP_TIME_DELTA
            if result_type == "run_wide"
            else BATTLE_TRACTION_LOSS_LAP_TIME_DELTA,
            BATTLE_RUN_WIDE_EFFECT_SECONDS
            if result_type == "run_wide"
            else BATTLE_TRACTION_LOSS_EFFECT_SECONDS,
            BATTLE_RUN_WIDE_TIRE_USAGE_MULTIPLIER
            if result_type == "run_wide"
            else BATTLE_TRACTION_LOSS_TIRE_USAGE_MULTIPLIER,
        )
        if result_type == "run_wide":
            return RaceEvent(
                type="run_wide",
                driver=attacker["abbreviation"],
                message=f"{attacker['full_name']} runs wide exiting {battle.segment_name}",
                message_ko=f"{attacker['full_name']}가 {battle.segment_name} 탈출에서 바깥으로 밀려납니다",
            )
        return RaceEvent(
            type="traction_loss",
            driver=attacker["abbreviation"],
            message=f"{attacker['full_name']} loses traction exiting {battle.segment_name}",
            message_ko=f"{attacker['full_name']}가 {battle.segment_name} 탈출에서 접지력을 잃습니다",
        )

    def _clear_forced_wide_aftermath_for_driver(self, driver_id: int) -> None:
        for key, aftermath in list(self._forced_wide_aftermaths.items()):
            if driver_id in (aftermath.attacker_id, aftermath.defender_id):
                del self._forced_wide_aftermaths[key]

    def _start_forced_wide_aftermath(
        self,
        attacker_id: int,
        defender_id: int,
        segment_name: str,
    ) -> None:
        self._clear_forced_wide_aftermath_for_driver(attacker_id)
        self._clear_forced_wide_aftermath_for_driver(defender_id)
        key = self._battle_pair_key(attacker_id, defender_id)
        self._forced_wide_aftermaths[key] = ForcedWideAftermath(
            attacker_id=attacker_id,
            defender_id=defender_id,
            remaining_seconds=BATTLE_FORCED_WIDE_EXIT_DELAY_SECONDS,
            segment_name=segment_name,
        )

    def _forced_wide_aftermath_is_valid(self, aftermath: ForcedWideAftermath) -> bool:
        attacker = self.driver_states.get(aftermath.attacker_id)
        defender = self.driver_states.get(aftermath.defender_id)
        if attacker is None or defender is None:
            return False
        return not (
            attacker.retired
            or defender.retired
            or attacker.finished
            or defender.finished
            or attacker.in_pit
            or defender.in_pit
        )

    def _forced_wide_result_probability(self, aftermath: ForcedWideAftermath) -> float:
        attacker_state = self.driver_states[aftermath.attacker_id]
        defender_state = self.driver_states[aftermath.defender_id]
        attacker = self._driver_meta[aftermath.attacker_id]
        defender = self._driver_meta[aftermath.defender_id]
        tire_wear = max(
            self._current_tire_wear(attacker_state),
            self._current_tire_wear(defender_state),
        )
        pressure = BATTLE_FORCED_WIDE_RESULT_BASE_PROBABILITY
        pressure += 0.08 if attacker_state.pace_mode == PaceMode.ATTACK else 0.0
        pressure += 0.08 * tire_wear
        pressure += 0.08 * (1.0 - attacker["consistency"])
        pressure += 0.12 * (1.0 - defender["consistency"])
        pressure -= 0.08 * max(0.0, defender["defending"] - 0.82)
        return min(0.72, max(0.18, pressure))

    def _forced_wide_result_type(self, aftermath: ForcedWideAftermath) -> str:
        defender = self._driver_meta[aftermath.defender_id]
        defender_tire_wear = self._current_tire_wear(self.driver_states[aftermath.defender_id])
        roll = self.rng.random()
        defender_control = 0.45 * defender["defending"] + 0.55 * defender["consistency"]
        if roll < 0.38 + 0.18 * (1.0 - defender_control):
            return "run_wide"
        if roll < 0.68 + 0.18 * defender_tire_wear:
            return "traction_loss"
        if roll < 0.90:
            return "attacker_traction_loss"
        return "minor_contact"

    def _tick_forced_wide_aftermaths(self, delta: float) -> list[RaceEvent]:
        events: list[RaceEvent] = []
        for key, aftermath in list(self._forced_wide_aftermaths.items()):
            aftermath.remaining_seconds -= delta
            if aftermath.remaining_seconds > 0:
                continue
            if self._forced_wide_aftermath_is_valid(aftermath):
                result_event = self._maybe_forced_wide_result_event(aftermath)
                if result_event is not None:
                    events.append(result_event)
            del self._forced_wide_aftermaths[key]
        return events

    def _maybe_forced_wide_result_event(self, aftermath: ForcedWideAftermath) -> RaceEvent | None:
        if self.rng.random() >= self._forced_wide_result_probability(aftermath):
            return None

        result_type = self._forced_wide_result_type(aftermath)
        attacker = self._driver_meta[aftermath.attacker_id]
        defender = self._driver_meta[aftermath.defender_id]
        if result_type == "minor_contact":
            if self._maybe_escalate_contact(aftermath.attacker_id, aftermath.defender_id):
                return None
            self._set_battle_effect(
                aftermath.attacker_id,
                BATTLE_MINOR_CONTACT_ATTACKER_LAP_TIME_DELTA,
                BATTLE_MINOR_CONTACT_EFFECT_SECONDS,
                BATTLE_MINOR_CONTACT_TIRE_USAGE_MULTIPLIER,
            )
            self._set_battle_effect(
                aftermath.defender_id,
                BATTLE_MINOR_CONTACT_DEFENDER_LAP_TIME_DELTA,
                BATTLE_MINOR_CONTACT_EFFECT_SECONDS,
                BATTLE_MINOR_CONTACT_TIRE_USAGE_MULTIPLIER,
            )
            return RaceEvent(
                type="minor_contact",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} and {defender['full_name']} touch"
                    f" after the forced-wide move at {aftermath.segment_name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}와 {defender['full_name']}가"
                    f" {aftermath.segment_name}에서 바깥으로 밀어낸 뒤 접촉합니다"
                ),
            )

        if result_type == "attacker_traction_loss":
            self._set_battle_effect(
                aftermath.attacker_id,
                BATTLE_FORCED_WIDE_ATTACKER_EXIT_LAP_TIME_DELTA,
                BATTLE_FORCED_WIDE_ATTACKER_EXIT_EFFECT_SECONDS,
                BATTLE_FORCED_WIDE_ATTACKER_EXIT_TIRE_USAGE_MULTIPLIER,
            )
            return RaceEvent(
                type="traction_loss",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} loses traction on the tight exit"
                    f" after forcing {defender['full_name']} wide at {aftermath.segment_name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 {defender['full_name']}를"
                    f" {aftermath.segment_name}에서 바깥으로 밀어낸 뒤"
                    " 좁은 탈출 라인에서 접지력을 잃습니다"
                ),
            )

        self._set_battle_effect(
            aftermath.defender_id,
            BATTLE_RUN_WIDE_LAP_TIME_DELTA
            if result_type == "run_wide"
            else BATTLE_TRACTION_LOSS_LAP_TIME_DELTA,
            BATTLE_RUN_WIDE_EFFECT_SECONDS
            if result_type == "run_wide"
            else BATTLE_TRACTION_LOSS_EFFECT_SECONDS,
            BATTLE_RUN_WIDE_TIRE_USAGE_MULTIPLIER
            if result_type == "run_wide"
            else BATTLE_TRACTION_LOSS_TIRE_USAGE_MULTIPLIER,
        )
        if result_type == "run_wide":
            return RaceEvent(
                type="run_wide",
                driver=defender["abbreviation"],
                message=(
                    f"{defender['full_name']} runs wide after being forced out"
                    f" at {aftermath.segment_name}"
                ),
                message_ko=(
                    f"{defender['full_name']}가 {aftermath.segment_name}에서"
                    " 바깥으로 밀려나 코스를 넓게 사용합니다"
                ),
            )
        return RaceEvent(
            type="traction_loss",
            driver=defender["abbreviation"],
            message=(
                f"{defender['full_name']} loses traction after being forced wide"
                f" at {aftermath.segment_name}"
            ),
            message_ko=(
                f"{defender['full_name']}가 {aftermath.segment_name}에서"
                " 바깥으로 밀려난 뒤 접지력을 잃습니다"
            ),
        )

    def _side_by_side_message(
        self,
        attacker_name: str,
        defender_name: str,
        segment_name: str,
        attacker_line: str,
        defender_line: str,
    ) -> str:
        attack_text = (
            "dives to the inside"
            if attacker_line == ATTACK_LINE_INSIDE
            else "hangs around the outside"
        )
        defense_text = (
            "as "
            + defender_name
            + " covers the inside"
            if defender_line == DEFENDER_LINE_DEFENSIVE
            else "against " + defender_name + " on the racing line"
        )
        return f"{attacker_name} {attack_text} {defense_text} into {segment_name}"

    def _side_by_side_message_ko(
        self,
        attacker_name: str,
        defender_name: str,
        segment_name: str,
        attacker_line: str,
        defender_line: str,
    ) -> str:
        attack_text = (
            "안쪽 라인으로 파고듭니다"
            if attacker_line == ATTACK_LINE_INSIDE
            else "바깥 라인으로 버팁니다"
        )
        defense_text = (
            f"{defender_name}가 안쪽을 막아서는 가운데"
            if defender_line == DEFENDER_LINE_DEFENSIVE
            else f"{defender_name}의 레이싱 라인 바깥에서"
        )
        return f"{attacker_name}가 {defense_text} {segment_name}에 {attack_text}"

    def _battle_effect_lap_time_delta(self, state: DriverRaceState) -> float:
        if state.retired or state.finished or state.in_pit:
            return 0.0
        side_by_side_delta = self._side_by_side_lap_time_delta(state)
        effect = self._battle_effects.get(state.driver_id)
        if effect is None:
            return side_by_side_delta
        return effect.lap_time_delta + side_by_side_delta

    def _battle_effect_tire_usage_multiplier(self, state: DriverRaceState) -> float:
        if state.retired or state.finished or state.in_pit:
            return 1.0
        side_by_side_multiplier = self._side_by_side_tire_usage_multiplier(state)
        effect = self._battle_effects.get(state.driver_id)
        if effect is None:
            return side_by_side_multiplier
        return effect.tire_usage_multiplier * side_by_side_multiplier

    def _maybe_battle_event(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
    ) -> RaceEvent | None:
        """Emit a visible battle event when a close chase reaches an overtaking section."""
        if car_ahead is None or state.retired or state.finished or state.in_pit:
            return None
        if (
            self._is_side_by_side_active(state.driver_id)
            or self._is_side_by_side_active(car_ahead.driver_id)
        ):
            return None
        if self._battle_event_cooldown.get(state.driver_id, 0.0) > 0:
            return None
        if self._battle_event_cooldown.get(car_ahead.driver_id, 0.0) > 0:
            return None

        progress_gap = car_ahead.total_progress - state.total_progress
        gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
        probability = self._battle_event_probability(state, car_ahead, gap_seconds)
        if probability <= 0.0 or self.rng.random() >= probability:
            return None

        segment = segment_at_progress(self.circuit, state.progress)
        attacker = self._driver_meta[state.driver_id]
        defender = self._driver_meta[car_ahead.driver_id]

        mistake_probability = self._battle_mistake_probability(state, gap_seconds)
        if mistake_probability > 0.0 and self.rng.random() < mistake_probability:
            self._battle_event_cooldown[state.driver_id] = BATTLE_EVENT_COOLDOWN_SECONDS
            self._battle_event_cooldown[car_ahead.driver_id] = BATTLE_EVENT_COOLDOWN_SECONDS
            self._set_battle_effect(
                state.driver_id,
                BATTLE_LOCKUP_LAP_TIME_DELTA,
                BATTLE_LOCKUP_EFFECT_SECONDS,
                BATTLE_LOCKUP_TIRE_USAGE_MULTIPLIER,
            )
            return RaceEvent(
                type="lockup",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} locks up attacking {defender['full_name']}"
                    f" into {segment.name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 {defender['full_name']}를 공격하다"
                    f" {segment.name} 진입에서 락업을 일으킵니다"
                ),
            )

        attack_score = (
            attacker["overtaking"]
            * self._pace_mode_attack_factor(state)
            * (1.10 if state.drs_active else 1.0)
        )
        defense_score = defender["defending"] * (0.88 + 0.24 * self.circuit.overtaking_difficulty)
        attack_score += self.rng.uniform(-0.04, 0.04)

        self._battle_event_cooldown[state.driver_id] = BATTLE_EVENT_COOLDOWN_SECONDS
        self._battle_event_cooldown[car_ahead.driver_id] = BATTLE_EVENT_COOLDOWN_SECONDS

        score_margin = attack_score - defense_score
        if (
            segment.type == TrackSegmentType.HEAVY_BRAKING
            and abs(score_margin) <= BATTLE_SIDE_BY_SIDE_SCORE_MARGIN
        ):
            attacker_line = self._choose_attack_line(state)
            defender_line = self._choose_defender_line(car_ahead, state)
            self._start_side_by_side_battle(
                state.driver_id,
                car_ahead.driver_id,
                attacker_line,
                defender_line,
                segment.name,
            )
            return RaceEvent(
                type="side_by_side",
                driver=attacker["abbreviation"],
                message=self._side_by_side_message(
                    attacker["full_name"],
                    defender["full_name"],
                    segment.name,
                    attacker_line,
                    defender_line,
                ),
                message_ko=self._side_by_side_message_ko(
                    attacker["full_name"],
                    defender["full_name"],
                    segment.name,
                    attacker_line,
                    defender_line,
                ),
            )

        if (
            segment.type == TrackSegmentType.HEAVY_BRAKING
            and score_margin >= BATTLE_FORCED_WIDE_SCORE_MARGIN
        ):
            self._set_battle_effect(
                state.driver_id,
                BATTLE_FORCED_WIDE_ATTACKER_LAP_TIME_DELTA,
                BATTLE_FORCED_WIDE_EFFECT_SECONDS,
                BATTLE_ATTACK_TIRE_USAGE_MULTIPLIER,
            )
            self._set_battle_effect(
                car_ahead.driver_id,
                BATTLE_FORCED_WIDE_DEFENDER_LAP_TIME_DELTA,
                BATTLE_FORCED_WIDE_EFFECT_SECONDS,
                BATTLE_FORCED_WIDE_TIRE_USAGE_MULTIPLIER,
            )
            self._start_forced_wide_aftermath(
                state.driver_id,
                car_ahead.driver_id,
                segment.name,
            )
            return RaceEvent(
                type="forced_wide",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} takes the inside line"
                    f" and forces {defender['full_name']} wide at {segment.name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 안쪽 라인을 잡고"
                    f" {defender['full_name']}를 {segment.name}에서 바깥으로 밀어냅니다"
                ),
            )

        if attack_score > defense_score:
            self._set_battle_effect(
                state.driver_id,
                BATTLE_ATTACKER_LAP_TIME_DELTA,
                BATTLE_ATTACK_EFFECT_SECONDS,
                BATTLE_ATTACK_TIRE_USAGE_MULTIPLIER,
            )
            self._set_battle_effect(
                car_ahead.driver_id,
                BATTLE_DEFENDER_PRESSURE_LAP_TIME_DELTA,
                BATTLE_ATTACK_EFFECT_SECONDS,
                BATTLE_DEFEND_TIRE_USAGE_MULTIPLIER,
            )
            drs_text = " with DRS" if state.drs_active else ""
            return RaceEvent(
                type="attack",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} attacks {defender['full_name']}"
                    f"{drs_text} into {segment.name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 {defender['full_name']}를"
                    f" {segment.name} 진입에서 공격합니다"
                    + (" (DRS 사용)" if state.drs_active else "")
                ),
            )

        self._set_battle_effect(
            state.driver_id,
            BATTLE_DEFEND_ATTACKER_LAP_TIME_DELTA,
            BATTLE_DEFEND_EFFECT_SECONDS,
            BATTLE_DEFEND_TIRE_USAGE_MULTIPLIER,
        )
        return RaceEvent(
            type="defend",
            driver=defender["abbreviation"],
            message=(
                f"{defender['full_name']} defends from {attacker['full_name']}"
                f" through {segment.name}"
            ),
            message_ko=(
                f"{defender['full_name']}가 {segment.name}에서"
                f" {attacker['full_name']}의 공격을 막아냅니다"
            ),
        )

    def _tick_battle_cooldowns(self, delta: float) -> None:
        for driver_id, cooldown in list(self._battle_event_cooldown.items()):
            remaining = cooldown - delta
            if remaining <= 0:
                del self._battle_event_cooldown[driver_id]
            else:
                self._battle_event_cooldown[driver_id] = remaining

    def _tick_battle_effects(self, delta: float) -> None:
        for driver_id, effect in list(self._battle_effects.items()):
            effect.remaining_seconds -= delta
            if effect.remaining_seconds <= 0:
                del self._battle_effects[driver_id]

    def _tick_ai_pace_cooldowns(self, delta: float) -> None:
        for driver_id, cooldown in list(self._ai_pace_cooldown.items()):
            remaining = cooldown - delta
            if remaining <= 0:
                del self._ai_pace_cooldown[driver_id]
            else:
                self._ai_pace_cooldown[driver_id] = remaining

    def _ai_pace_decision_cooldown(self) -> float:
        return self.rng.uniform(AI_PACE_MIN_COOLDOWN_SECONDS, AI_PACE_MAX_COOLDOWN_SECONDS)

    def _gap_seconds_between(
        self,
        car_ahead: DriverRaceState | None,
        chaser: DriverRaceState | None,
    ) -> float | None:
        if car_ahead is None or chaser is None:
            return None
        progress_gap = car_ahead.total_progress - chaser.total_progress
        if progress_gap <= 0:
            return None
        return self._progress_gap_to_seconds(progress_gap, chaser)

    def _choose_ai_pace_mode(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        car_behind: DriverRaceState | None,
    ) -> PaceMode:
        tire_life = max(0.0, 1.0 - self._current_tire_wear(state))
        remaining_laps = max(0, self.total_laps - state.current_lap)
        endgame_laps = max(3, int(self.total_laps * 0.12))
        is_endgame = remaining_laps <= endgame_laps
        fresh_after_stop = state.pit_count > 0 and state.tire_usage <= AI_FRESH_TIRE_USAGE
        pit_in_push = state.pit_request is not None and tire_life > AI_LOW_TIRE_LIFE

        if tire_life <= AI_CRITICAL_TIRE_LIFE:
            return PaceMode.CONSERVE

        gap_ahead = self._gap_seconds_between(car_ahead, state)
        gap_behind = self._gap_seconds_between(state, car_behind)

        if tire_life <= AI_LOW_TIRE_LIFE and not is_endgame and not pit_in_push:
            return PaceMode.CONSERVE

        if pit_in_push or fresh_after_stop:
            return PaceMode.ATTACK

        if gap_ahead is not None and gap_ahead <= AI_ATTACK_GAP_SECONDS:
            return PaceMode.ATTACK if tire_life > AI_LOW_TIRE_LIFE else PaceMode.STANDARD

        if gap_behind is not None and gap_behind <= AI_DEFEND_GAP_SECONDS:
            defending = self._driver_meta[state.driver_id]["defending"]
            if tire_life > AI_LOW_TIRE_LIFE and defending >= 0.82:
                return PaceMode.ATTACK
            return PaceMode.STANDARD

        if is_endgame:
            return PaceMode.ATTACK if tire_life > 0.35 else PaceMode.STANDARD

        return PaceMode.STANDARD

    def _build_pass_events(self, previous_positions: dict[int, int]) -> list[RaceEvent]:
        """Return pass events for drivers whose position improved this tick."""
        previous_by_position = {
            position: driver_id
            for driver_id, position in previous_positions.items()
        }
        events: list[RaceEvent] = []

        for state in sorted(self.driver_states.values(), key=lambda s: s.position):
            if state.retired or state.finished or state.in_pit:
                continue
            previous_position = previous_positions.get(state.driver_id)
            if previous_position is None or state.position >= previous_position:
                continue

            passed_driver_id = previous_by_position.get(state.position)
            if passed_driver_id is None or passed_driver_id == state.driver_id:
                continue
            passed_state = self.driver_states.get(passed_driver_id)
            if passed_state is None or passed_state.retired or passed_state.finished:
                continue

            meta = self._driver_meta[state.driver_id]
            passed_meta = self._driver_meta[passed_driver_id]
            events.append(
                RaceEvent(
                    type="pass",
                    driver=meta["abbreviation"],
                    message=(
                        f"{meta['full_name']} passes {passed_meta['full_name']}"
                        f" for P{state.position}"
                    ),
                    message_ko=(
                        f"{meta['full_name']}가 {passed_meta['full_name']}를 추월하며"
                        f" P{state.position}로 올라섭니다"
                    ),
                )
            )

        return events

    def _refresh_lap_variation(self, driver_id: int) -> None:
        """Refresh per-lap pace and tire condition variance."""
        state = self.driver_states[driver_id]
        meta = self._driver_meta[driver_id]
        wear = self._current_tire_wear(state)
        tire_amplitude = 0.0015 + 0.0045 * wear
        consistency_amplitude = self._consistency_variation_spread(meta["consistency"])
        tire_penalty = self._dead_tire_lap_penalty(wear)
        self._lap_random[driver_id] = (
            self.rng.uniform(-consistency_amplitude, consistency_amplitude)
            + tire_penalty
        )
        self._tire_random[driver_id] = self.rng.uniform(-tire_amplitude, tire_amplitude)

    def _segment_type_at(self, state: DriverRaceState) -> str | None:
        seg = segment_at_progress(self.circuit, state.progress)
        if seg is None:
            return None
        return getattr(seg.type, "value", seg.type)

    def _race_progress(self) -> float:
        if self.total_laps <= 0:
            return 0.0
        return min(1.0, self.current_lap / self.total_laps)

    def _leader_lap(self) -> int:
        return max(
            (s.current_lap for s in self.driver_states.values() if not s.retired),
            default=0,
        )

    def _phase_lap_time_factor(self, state: DriverRaceState | None = None) -> float:
        if self.race_phase == "sc":
            if state is not None and state.driver_id in self._sc_unlap_driver_ids:
                return SC_UNLAP_LAP_TIME_FACTOR
            if self.safety_car_stage == "restart":
                leader = self._on_track_leader()
                if (
                    leader is not None
                    and self._sc_restart_accel_progress is not None
                    and leader.total_progress >= self._sc_restart_accel_progress
                ):
                    return 1.0
            if state is None:
                return SC_CATCH_UP_FAST_LAP_TIME_FACTOR
            return self._sc_lap_time_factor_for_gap(state)
        if self.race_phase == "vsc":
            return VSC_LAP_TIME_FACTOR
        return 1.0

    def _race_phase_remaining_seconds(self) -> float:
        if self.race_phase != "vsc":
            return 0.0
        return max(0.0, self._phase_until - self.race_elapsed)

    def _race_phase_remaining_laps(self) -> int:
        if self.race_phase != "sc":
            return 0
        return 1 if self.safety_car_stage in {"in_this_lap", "restart"} else 0

    def _trigger_vsc(self, events: list[RaceEvent]) -> None:
        """Deploy a virtual safety car (skipped if a full SC is already out)."""
        if self.race_phase == "sc":
            return
        target = self.race_elapsed + VSC_DURATION_SECONDS
        if self.race_phase == "vsc":
            self._phase_until = max(self._phase_until, target)
            return
        self.race_phase = "vsc"
        self._phase_until = target
        events.append(
            RaceEvent(
                type="vsc_start",
                message="Virtual Safety Car deployed",
                message_ko="버추얼 세이프티카가 발동됩니다",
            )
        )

    def _trigger_safety_car(self, events: list[RaceEvent]) -> None:
        """Deploy a full safety car (upgrades an active VSC)."""
        already_sc = self.race_phase == "sc"
        self.race_phase = "sc"
        self.safety_car = True
        cleanup_target = self.race_elapsed + (
            SC_ADDITIONAL_INCIDENT_SECONDS if already_sc else SC_CLEANUP_SECONDS
        )
        self._sc_cleanup_until = max(self._sc_cleanup_until, cleanup_target)
        if not already_sc:
            self.pit_window_open = True
            self._initialize_safety_car_progress()
            self._ai_sc_pit_decisions()
            events.append(
                RaceEvent(
                    type="sc_start",
                    message="Safety Car deployed — it is leaving the pits",
                    message_ko="세이프티카가 피트에서 출동합니다",
                )
            )
            events.append(
                RaceEvent(
                    type="pit_window",
                    message="Pit window open under the safety car",
                    message_ko="세이프티카 동안 피트 기회가 열렸습니다",
                )
            )

    def set_race_control_phase_for_testing(self, phase: str) -> list[RaceEvent]:
        """Force a race-control phase for the removable development UI."""
        requested_phase = str(phase).lower()
        events: list[RaceEvent] = []

        if requested_phase == "vsc":
            self._trigger_vsc(events)
            return events
        if requested_phase == "sc":
            self._trigger_safety_car(events)
            return events
        if requested_phase != "green":
            raise ValueError(f"Unsupported race phase: {phase}")

        ended = self.race_phase
        if ended == "green":
            return events
        if ended == "sc":
            self._sc_cleanup_until = self.race_elapsed
            self._sc_unlap_driver_ids.clear()
            self._sc_unlap_targets.clear()
            self._begin_sc_in_this_lap(events)
            return events
        self._finish_race_phase(ended, events)
        return events

    def _on_track_leader(self) -> DriverRaceState | None:
        running = [
            state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        ]
        if not running:
            return None
        return min(running, key=lambda state: state.position)

    def _initialize_safety_car_progress(self) -> None:
        leader = self._on_track_leader()
        if leader is None:
            self._safety_car_total_progress = None
            self._safety_car_progress_rate = 0.0
            return
        exit_progress = self._pit_exit_progress()
        if exit_progress is None:
            exit_progress = (leader.progress + SC_LEAD_PROGRESS_GAP) % 1.0
        self._safety_car_total_progress = self._total_progress_at_or_after(
            leader.total_progress,
            exit_progress,
        )
        self._safety_car_progress_rate = 0.0
        self.safety_car_stage = "deploying"
        self._safety_car_visible = True
        self._safety_car_route = "pit"
        self._safety_car_pit_lane_progress = 0.82
        self._safety_car_queue_formed = False
        self._sc_caught_driver_ids.clear()
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._sc_withdraw_target = None
        self._sc_restart_target = None
        self._sc_restart_accel_progress = None

    def _total_progress_at_or_after(self, reference: float, progress: float) -> float:
        target = int(reference // 1.0) + (progress % 1.0)
        if target <= reference + PROGRESS_EPSILON:
            target += 1.0
        return target

    def _sc_max_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS) / self.track_length_m

    def _sc_target_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS) / self.track_length_m

    def _sc_queue_predecessor(self, state: DriverRaceState) -> DriverRaceState | None:
        """Return the next eligible car ahead in the frozen SC running order."""
        running = sorted(
            (
                candidate
                for candidate in self.driver_states.values()
                if not candidate.retired
                and not candidate.finished
                and not candidate.in_pit
                and candidate.driver_id not in self._sc_unlap_driver_ids
            ),
            key=lambda candidate: candidate.position,
        )
        for index, candidate in enumerate(running):
            if candidate.driver_id != state.driver_id:
                continue
            return running[index - 1] if index > 0 else None
        return None

    def _sc_lap_time_factor_for_gap(self, state: DriverRaceState) -> float:
        """Choose a smooth catch-up pace from the gap to the queued car ahead."""
        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            ahead_is_queued = True
        else:
            ahead_total = predecessor.total_progress
            ahead_is_queued = predecessor.driver_id in self._sc_caught_driver_ids

        if ahead_total is None:
            return SC_CATCH_UP_FAST_LAP_TIME_FACTOR

        gap = max(0.0, ahead_total - state.total_progress)
        target_gap = self._sc_target_gap_progress()
        max_gap = self._sc_max_gap_progress()
        caught = state.driver_id in self._sc_caught_driver_ids and ahead_is_queued

        if caught:
            if gap <= target_gap + PROGRESS_EPSILON:
                return SC_LAP_TIME_FACTOR
            recovery_range = max(PROGRESS_EPSILON, max_gap - target_gap)
            recovery_ratio = min(1.0, (gap - target_gap) / recovery_range)
            return SC_LAP_TIME_FACTOR - (
                (SC_LAP_TIME_FACTOR - SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR)
                * recovery_ratio
            )

        if not ahead_is_queued:
            return SC_CATCH_UP_FAST_LAP_TIME_FACTOR

        far_gap = max(max_gap, 0.5)
        approach_range = max(PROGRESS_EPSILON, far_gap - max_gap)
        far_ratio = min(1.0, max(0.0, gap - max_gap) / approach_range)
        return SC_CATCH_UP_NEAR_LAP_TIME_FACTOR - (
            (SC_CATCH_UP_NEAR_LAP_TIME_FACTOR - SC_CATCH_UP_FAST_LAP_TIME_FACTOR)
            * far_ratio
        )

    def _set_state_total_progress(self, state: DriverRaceState, total: float) -> None:
        lap = int(total // 1.0)
        state.current_lap = lap
        state.progress = total - lap
        state.total_progress = total

    def _advance_safety_car(self, delta: float, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self._safety_car_total_progress is None:
            self._safety_car_progress_rate = 0.0
            return

        if self.safety_car_stage == "deploying":
            remaining = 1.0 - self._safety_car_pit_lane_progress
            advance = delta * 0.18 / SC_DEPLOY_PIT_SECONDS
            self._safety_car_pit_lane_progress = min(1.0, self._safety_car_pit_lane_progress + advance)
            self._safety_car_progress_rate = 0.0
            if remaining <= advance + PROGRESS_EPSILON:
                self._safety_car_route = "track"
                self.safety_car_stage = "collecting"
                events.append(
                    RaceEvent(
                        type="sc_track_join",
                        message="Safety Car joins the track from the Pit Lane exit",
                        message_ko="세이프티카가 피트 출구에서 트랙에 합류합니다",
                    )
                )
            return

        if self.safety_car_stage == "restart":
            self._safety_car_progress_rate = 0.0
            self._safety_car_pit_lane_progress = min(
                1.0,
                self._safety_car_pit_lane_progress + delta / SC_WITHDRAW_PIT_SECONDS,
            )
            if self._safety_car_pit_lane_progress >= 0.35:
                self._safety_car_visible = False
            return

        previous = self._safety_car_total_progress
        self._safety_car_progress_rate = 1.0 / max(
            self.circuit.base_lap_time * SC_LAP_TIME_FACTOR,
            1.0,
        )
        self._safety_car_total_progress += self._safety_car_progress_rate * delta

        if (
            self.safety_car_stage == "in_this_lap"
            and self._sc_withdraw_target is not None
            and previous < self._sc_withdraw_target <= self._safety_car_total_progress
        ):
            self._safety_car_total_progress = self._sc_withdraw_target
            self._safety_car_progress_rate = 0.0
            self._safety_car_route = "pit"
            self._safety_car_pit_lane_progress = 0.0
            self.safety_car_stage = "restart"
            leader = self._on_track_leader()
            if leader is not None:
                self._sc_restart_target = self._total_progress_at_or_after(
                    leader.total_progress,
                    0.0,
                )
                restart_distance = self.rng.uniform(0.04, 0.09)
                self._sc_restart_accel_progress = max(
                    leader.total_progress,
                    self._sc_restart_target - restart_distance,
                )
            events.append(
                RaceEvent(
                    type="sc_pit",
                    message="Safety Car enters the Pit Lane — the leader controls the restart",
                    message_ko="세이프티카가 피트로 들어가며 선두가 재출발을 통제합니다",
                )
            )

    def _finish_race_phase(self, ended: str, events: list[RaceEvent]) -> None:
        """Reset phase state and publish the matching green-flag event."""
        self.race_phase = "green"
        self.safety_car = False
        self._phase_until = 0.0
        self._safety_car_total_progress = None
        self._safety_car_progress_rate = 0.0
        self.safety_car_stage = "inactive"
        self._safety_car_visible = False
        self._safety_car_route = "track"
        self._safety_car_pit_lane_progress = 0.0
        self._safety_car_queue_formed = False
        self._sc_cleanup_until = 0.0
        self._sc_caught_driver_ids.clear()
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._sc_withdraw_target = None
        self._sc_restart_target = None
        self._sc_restart_accel_progress = None
        self.pit_window_open = False
        if ended == "sc":
            events.append(
                RaceEvent(
                    type="sc_end",
                    message="Safety Car in this lap — racing resumes",
                    message_ko="세이프티카가 들어옵니다 — 레이스 재개",
                )
            )
        else:
            events.append(
                RaceEvent(
                    type="vsc_end",
                    message="Virtual Safety Car ending — green flag",
                    message_ko="버추얼 세이프티카 해제 — 그린 플래그",
                )
            )

    def _tick_race_phase(self, delta: float, events: list[RaceEvent]) -> None:
        """Advance VSC timing or the physical Safety Car route."""
        if self.race_phase == "green":
            return
        if self.race_phase == "sc":
            self._advance_safety_car(delta, events)
            return
        if self.race_elapsed >= self._phase_until:
            self._finish_race_phase("vsc", events)

    def _sync_safety_car_queue(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self.safety_car_stage in {"deploying", "restart"}:
            return
        if self._safety_car_total_progress is None:
            return

        running = sorted(
            (
                state
                for state in self.driver_states.values()
                if not state.retired and not state.finished and not state.in_pit
            ),
            key=lambda state: state.position,
        )
        active_ids = {state.driver_id for state in running}
        self._sc_caught_driver_ids.intersection_update(active_ids)

        max_gap = self._sc_max_gap_progress()
        target_gap = self._sc_target_gap_progress()
        ahead_total = self._safety_car_total_progress
        ahead_is_queued = True

        for state in running:
            if state.driver_id in self._sc_unlap_driver_ids:
                continue

            gap = ahead_total - state.total_progress
            caught = state.driver_id in self._sc_caught_driver_ids
            if caught and (not ahead_is_queued or gap > max_gap + PROGRESS_EPSILON):
                self._sc_caught_driver_ids.discard(state.driver_id)
                caught = False

            if ahead_is_queued and gap <= max_gap + PROGRESS_EPSILON:
                self._sc_caught_driver_ids.add(state.driver_id)
                maximum_total = ahead_total - target_gap
                if state.total_progress > maximum_total:
                    self._set_state_total_progress(state, maximum_total)
                caught = True
            else:
                caught = False

            ahead_total = state.total_progress
            ahead_is_queued = caught

        queue_ids = {
            state.driver_id
            for state in running
            if state.driver_id not in self._sc_unlap_driver_ids
        }
        formed = bool(queue_ids) and queue_ids.issubset(self._sc_caught_driver_ids)
        was_formed = self._safety_car_queue_formed
        self._safety_car_queue_formed = formed

        if formed and not was_formed and self.safety_car_stage == "collecting":
            self.safety_car_stage = "queued"
            events.append(
                RaceEvent(
                    type="sc_queue",
                    message="The field is queued behind the Safety Car",
                    message_ko="전체 차량이 세이프티카 뒤에 대열을 형성했습니다",
                )
            )
        elif not formed and self.safety_car_stage == "queued":
            self.safety_car_stage = "collecting"

    def _eligible_sc_unlap_drivers(self) -> list[DriverRaceState]:
        leader = self._on_track_leader()
        if leader is None or leader.current_lap >= self.total_laps - 1:
            return []
        return sorted(
            (
                state
                for state in self.driver_states.values()
                if not state.retired
                and not state.finished
                and not state.in_pit
                and state.driver_id != leader.driver_id
                and state.current_lap < leader.current_lap
            ),
            key=lambda state: state.position,
        )

    def _start_sc_unlapping(
        self,
        drivers: list[DriverRaceState],
        events: list[RaceEvent],
    ) -> None:
        self.safety_car_stage = "unlapping"
        self._safety_car_queue_formed = False
        self._sc_unlap_driver_ids = {state.driver_id for state in drivers}
        self._sc_unlap_targets = {
            state.driver_id: state.total_progress + 1.0 for state in drivers
        }
        self._sc_caught_driver_ids.difference_update(self._sc_unlap_driver_ids)
        events.append(
            RaceEvent(
                type="unlap_start",
                message="Lapped cars may now overtake",
                message_ko="랩 다운 차량의 추월이 허용됩니다",
            )
        )

    def _complete_sc_unlapping(self, events: list[RaceEvent]) -> None:
        unlapping = [
            self.driver_states[driver_id]
            for driver_id in self._sc_unlap_driver_ids
            if driver_id in self.driver_states
            and not self.driver_states[driver_id].retired
            and not self.driver_states[driver_id].finished
        ]
        main_queue = sorted(
            (
                state
                for state in self.driver_states.values()
                if not state.retired
                and not state.finished
                and not state.in_pit
                and state.driver_id not in self._sc_unlap_driver_ids
            ),
            key=lambda state: state.position,
        )
        tail_total = (
            main_queue[-1].total_progress
            if main_queue
            else (self._safety_car_total_progress or 0.0) - self._sc_target_gap_progress()
        )
        gap = self._sc_target_gap_progress()
        for index, state in enumerate(sorted(unlapping, key=lambda item: item.position), start=1):
            self._set_state_total_progress(state, tail_total - index * gap)
            state._lap_start_time = state.total_time  # type: ignore[attr-defined]
            self._sc_caught_driver_ids.add(state.driver_id)
            meta = self._driver_meta[state.driver_id]
            events.append(
                RaceEvent(
                    type="unlap",
                    driver=meta["abbreviation"],
                    message=f"{meta['full_name']} rejoins at the back of the Safety Car queue",
                    message_ko=f"{meta['full_name']}가 랩을 회복하고 대열 뒤에 합류합니다",
                )
            )
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._safety_car_queue_formed = True

    def _begin_sc_in_this_lap(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self.safety_car_stage in {"in_this_lap", "restart"}:
            return
        if self._safety_car_total_progress is None:
            self._initialize_safety_car_progress()
        if self._safety_car_total_progress is None:
            return
        if self.safety_car_stage == "deploying":
            self._safety_car_route = "track"
            self._safety_car_pit_lane_progress = 1.0
        entry_progress = self._pit_entry_progress()
        if entry_progress is None:
            entry_progress = 0.98
        self.safety_car_stage = "in_this_lap"
        self._sc_withdraw_target = self._total_progress_at_or_after(
            self._safety_car_total_progress,
            entry_progress,
        )
        events.append(
            RaceEvent(
                type="sc_in_this_lap",
                message="Safety Car in this lap",
                message_ko="이번 랩에 세이프티카가 들어갑니다",
            )
        )

    def _tick_safety_car_after_cars(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc":
            return

        self._sync_safety_car_queue(events)

        if (
            self.safety_car_stage == "queued"
            and self._safety_car_queue_formed
            and self.race_elapsed >= self._sc_cleanup_until
        ):
            lapped = self._eligible_sc_unlap_drivers()
            if lapped:
                self._start_sc_unlapping(lapped, events)
            else:
                self._begin_sc_in_this_lap(events)

        if self.safety_car_stage == "unlapping" and self._sc_unlap_targets:
            completed = all(
                self.driver_states[driver_id].total_progress
                >= target - PROGRESS_EPSILON
                for driver_id, target in self._sc_unlap_targets.items()
                if driver_id in self.driver_states
            )
            if completed:
                self._complete_sc_unlapping(events)
                self._begin_sc_in_this_lap(events)

        if self.safety_car_stage == "restart" and self._sc_restart_target is not None:
            leader = self._on_track_leader()
            if leader is not None and leader.total_progress >= self._sc_restart_target:
                self._finish_race_phase("sc", events)

    def _apply_incident(self, incident: Incident, events: list[RaceEvent]) -> None:
        """Apply an incident's effect and trigger SC/VSC for severe outcomes."""
        primary = self.driver_states.get(incident.primary_driver_id)
        if primary is None or primary.retired or primary.finished:
            return
        meta = self._driver_meta[incident.primary_driver_id]

        if incident.severity == IncidentSeverity.MINOR:
            penalty = self.rng.uniform(*INCIDENT_MINOR_TIME_PENALTY)
            primary.total_time += penalty
            primary.tire_usage += INCIDENT_MINOR_TIRE_USAGE
            if incident.cause == IncidentCause.MECHANICAL:
                message = f"{meta['full_name']} reports a problem — loses time"
                message_ko = f"{meta['full_name']}가 차량 문제로 시간을 잃습니다"
            else:
                message = f"{meta['full_name']} runs wide — loses time"
                message_ko = f"{meta['full_name']}가 코스를 벗어나 시간을 잃습니다"
            events.append(
                RaceEvent(type="incident", driver=meta["abbreviation"], message=message, message_ko=message_ko)
            )
            return

        # CAR_STOPPED or CRASH: the primary driver is out.
        primary.retired = True
        if incident.cause == IncidentCause.MECHANICAL:
            message = f"{meta['full_name']} stops on track — mechanical failure"
            message_ko = f"{meta['full_name']}가 기계 결함으로 트랙에 멈춥니다"
        elif incident.severity == IncidentSeverity.CRASH:
            message = f"{meta['full_name']} crashes out"
            message_ko = f"{meta['full_name']}가 충돌로 리타이어합니다"
        else:
            message = f"{meta['full_name']} stops on track"
            message_ko = f"{meta['full_name']}가 트랙에 멈춰 섭니다"
        events.append(
            RaceEvent(type="retirement", driver=meta["abbreviation"], message=message, message_ko=message_ko)
        )

        if incident.secondary_driver_id is not None:
            secondary = self.driver_states.get(incident.secondary_driver_id)
            if secondary is not None and not secondary.retired and not secondary.finished:
                secondary.retired = True
                smeta = self._driver_meta[incident.secondary_driver_id]
                events.append(
                    RaceEvent(
                        type="retirement",
                        driver=smeta["abbreviation"],
                        message=f"{smeta['full_name']} is collected in the incident",
                        message_ko=f"{smeta['full_name']}가 사고에 휘말려 리타이어합니다",
                    )
                )

        if incident.severity == IncidentSeverity.CRASH:
            self._trigger_safety_car(events)
        else:
            self._trigger_vsc(events)

    def get_grid_order(self) -> list[dict]:
        """Return starting grid for setup response."""
        ordered = sorted(
            self.driver_states.values(),
            key=lambda s: s.position,
        )
        result = []
        for state in ordered:
            meta = self._driver_meta[state.driver_id]
            result.append(
                {
                    "driver_id": state.driver_id,
                    "name": meta["abbreviation"],
                    "team": meta["team_name"],
                    "position": state.position,
                }
            )
        return result

    def set_speed(self, multiplier: int) -> None:
        if multiplier in (1, 2, 5):
            self.speed_multiplier = multiplier

    def pause(self) -> None:
        self.paused = True

    def resume(self) -> None:
        self.paused = False

    def request_pit(self, driver_id: int, tire_choice: TireCompound) -> str | None:
        """Register a pit request. Returns error message or None."""
        state = self.driver_states.get(driver_id)
        if state is None:
            return "Unknown driver"
        if state.retired:
            return "Driver has retired"
        if state.in_pit:
            return "Driver already in pit"
        if state.pit_request is not None:
            return "Pit already requested"
        if driver_id not in self.player_driver_ids:
            return "Not a player driver"
        state.pit_request = tire_choice
        return None

    def _pit_entry_progress(self) -> float | None:
        pit_lane = self.circuit.pit_lane
        if pit_lane is None or pit_lane.entry_progress is None:
            return None
        return pit_lane.entry_progress % 1.0

    def _pit_exit_progress(self) -> float | None:
        pit_lane = self.circuit.pit_lane
        if pit_lane is None or pit_lane.exit_progress is None:
            return None
        return pit_lane.exit_progress % 1.0

    def _has_pit_progress_anchors(self) -> bool:
        return self._pit_entry_progress() is not None and self._pit_exit_progress() is not None

    def _progress_distance(self, start: float, target: float) -> float:
        return (target - start) % 1.0

    def _crossed_progress(self, start: float, delta: float, target: float) -> bool:
        distance = self._progress_distance(start % 1.0, target % 1.0)
        return PROGRESS_EPSILON < distance <= delta + PROGRESS_EPSILON

    def _should_enter_pit(
        self,
        state: DriverRaceState,
        previous_progress: float,
        progress_delta: float,
    ) -> bool:
        entry = self._pit_entry_progress()
        if entry is None or state.pit_request is None:
            return False
        return self._crossed_progress(previous_progress, progress_delta, entry)

    def _start_pit_stop(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        team: Team,
        tire: TireCompound,
        events: list[RaceEvent],
    ) -> None:
        lane_time, tire_change_time = compute_pit_components(
            self.circuit.pit_loss_time,
            team.pit_crew_skill,
            self.rng,
        )
        half_lane = lane_time / 2.0
        state.in_pit = True
        self._pit_phase[driver_id] = "in"
        self._pit_phase_duration[driver_id] = half_lane
        self._pit_phase_remaining[driver_id] = half_lane
        self._pit_lane_half_time[driver_id] = half_lane
        self._pit_tire_change_time[driver_id] = tire_change_time
        self._pit_elapsed[driver_id] = 0.0
        self._pit_stop_elapsed[driver_id] = 0.0
        self._pit_tire[driver_id] = tire
        self._pit_entry_compound[driver_id] = state.tire_compound
        state.speed_kph = PIT_LANE_SPEED_LIMIT_KPH
        events.append(
            RaceEvent(
                type="pit_entry",
                driver=meta["abbreviation"],
                message=f"{meta['full_name']} pits for {tire.value} tires",
                message_ko=f"{meta['full_name']}가 {tire.value} 타이어로 교체하기 위해 피트인합니다",
            )
        )

    def set_pace_mode(self, driver_id: int, pace_mode: PaceMode) -> str | None:
        """Set pace command for a player driver. Returns error message or None."""
        state = self.driver_states.get(driver_id)
        if state is None:
            return "Unknown driver"
        if state.retired:
            return "Driver has retired"
        if state.finished:
            return "Driver has finished"
        if driver_id not in self.player_driver_ids:
            return "Not a player driver"
        state.pace_mode = pace_mode
        return None

    def tick(self, delta_game_seconds: float | None = None) -> list[RaceEvent]:
        """Advance simulation by delta_game_seconds. Returns new events."""
        if self.finished or self.paused:
            return []

        delta = delta_game_seconds or GAME_TICK_SECONDS
        self.race_elapsed += delta
        events: list[RaceEvent] = []
        previous_positions = {
            driver_id: state.position
            for driver_id, state in self.driver_states.items()
        }
        self._tick_race_phase(delta, events)
        racing = self.race_phase == "green"
        self._tick_battle_cooldowns(delta)
        if racing:
            events.extend(self._tick_side_by_side_battles(delta))
            events.extend(self._tick_forced_wide_aftermaths(delta))
            if self._pending_incidents:
                for incident in self._pending_incidents:
                    self._apply_incident(incident, events)
                self._pending_incidents.clear()
        running_by_position = self._running_by_position()
        self._tick_ai_pace_cooldowns(delta)
        self._run_ai_pace_modes(running_by_position)

        for driver_id, state in self.driver_states.items():
            state.drs_active = False
            state.dirty_air_active = False
            if state.retired or state.finished:
                self._progress_rate[driver_id] = 0.0
                state.speed_kph = 0.0
                continue

            meta = self._driver_meta[driver_id]
            team = self.teams[self.drivers[driver_id].team_id]

            if state.in_pit:
                self._progress_rate[driver_id] = 0.0
                self._tick_in_pit(driver_id, state, meta, delta, events)
                if state.in_pit:
                    state.speed_kph = self._pit_phase_speed_kph(driver_id)
                continue

            event_result = roll_events(
                meta["abbreviation"],
                meta["full_name"],
                self.rng,
                enable_events=False,
            )
            if event_result.retired:
                self._progress_rate[driver_id] = 0.0
                state.speed_kph = 0.0
                state.retired = True
                if event_result.event:
                    events.append(event_result.event)
                continue
            if event_result.time_penalty > 0:
                state.total_time += event_result.time_penalty
                if event_result.event:
                    events.append(event_result.event)

            lap_time = self._base_lap_time_for_state(state)
            pace_effect = self._pace_mode_effect(state)
            car_ahead = running_by_position.get(state.position - 1)
            if racing:
                lap_time += self._battle_lap_time_delta(state, car_ahead)
                battle_event = self._maybe_battle_event(state, car_ahead)
                if battle_event is not None:
                    events.append(battle_event)

                incident = roll_solo_incident(
                    self.rng,
                    driver_id,
                    delta_seconds=delta,
                    consistency=meta["consistency"],
                    tire_wear=self._current_tire_wear(state),
                    segment_type=self._segment_type_at(state),
                    attack_mode=state.pace_mode == PaceMode.ATTACK,
                    dirty_air=state.dirty_air_active,
                    race_progress=self._race_progress(),
                )
                if incident is not None:
                    self._apply_incident(incident, events)
                    if state.retired:
                        continue
            lap_time += self._battle_effect_lap_time_delta(state)

            lap_time *= self._phase_lap_time_factor(state)

            previous_progress = state.progress
            progress_delta = self._distance_based_progress_delta(state, lap_time, delta)
            self._progress_rate[driver_id] = progress_delta / delta if delta > 0 else 0.0
            state.progress += progress_delta
            tire_usage_multiplier = (
                pace_effect["tire_usage_multiplier"]
                * self._battle_effect_tire_usage_multiplier(state)
            )
            state.tire_usage += progress_delta * tire_usage_multiplier
            state.total_time += delta
            state.total_progress = state.current_lap + state.progress
            state.tire_wear = self._current_tire_wear(state)

            if self._should_enter_pit(state, previous_progress, progress_delta):
                tire = state.pit_request
                state.pit_request = None
                entry = self._pit_entry_progress()
                if tire is not None and entry is not None:
                    self._progress_rate[driver_id] = 0.0
                    state.progress = entry
                    state.total_progress = state.current_lap + state.progress
                    self._start_pit_stop(driver_id, state, meta, team, tire, events)
                continue

            while state.progress >= 1.0 and not state.in_pit and not state.finished:
                progress_after_line = state.progress - 1.0
                lap_finish_time = state.total_time
                if progress_delta > 0:
                    lap_finish_time -= (progress_after_line / progress_delta) * delta

                state.progress -= 1.0
                lap_events = self._complete_lap(driver_id, state, meta, team, lap_finish_time)
                events.extend(lap_events)

        self._run_ai_strategy()
        self._update_positions()
        self._tick_safety_car_after_cars(events)
        self._update_positions()
        self._prune_side_by_side_battles()
        if racing:
            events.extend(self._build_pass_events(previous_positions))
        self._update_gaps()
        self._tick_battle_effects(delta)

        leader_lap = max(
            (s.current_lap for s in self.driver_states.values() if not s.retired),
            default=0,
        )
        self.current_lap = min(leader_lap + 1, self.total_laps)

        if self._check_race_finished():
            self.finished = True

        return events

    def _tick_in_pit(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        delta: float,
        events: list[RaceEvent],
    ) -> None:
        """Advance a car through the pit lane: drive in -> stop -> drive out.

        Time accumulates (``_pit_elapsed`` and, while stationary,
        ``_pit_stop_elapsed``) instead of counting down, and the car's pit-lane
        position is reported so the front end can animate it along the lane.
        """
        state.total_time += delta
        self._pit_elapsed[driver_id] += delta
        if self._pit_phase[driver_id] == "stop":
            self._pit_stop_elapsed[driver_id] += delta

        self._pit_phase_remaining[driver_id] -= delta
        if self._pit_phase_remaining[driver_id] > 0:
            self._sync_pit_race_progress(driver_id, state)
            return

        leftover = -self._pit_phase_remaining[driver_id]
        phase = self._pit_phase[driver_id]

        if phase == "in":
            # Arrived at the box; begin the stationary tire change.
            self._pit_phase[driver_id] = "stop"
            duration = self._pit_tire_change_time[driver_id]
            self._pit_phase_duration[driver_id] = duration
            self._pit_phase_remaining[driver_id] = max(0.0, duration - leftover)
            self._pit_stop_elapsed[driver_id] += min(leftover, duration)
        elif phase == "stop":
            # Tire change complete; fit fresh rubber and drive back out.
            new_compound = self._pit_tire[driver_id]
            state.tire_compound = new_compound
            state.tire_age = 0
            state.tire_usage = 0.0
            state.tire_wear = 0.0
            self._pit_phase[driver_id] = "out"
            duration = self._pit_lane_half_time[driver_id]
            self._pit_phase_duration[driver_id] = duration
            self._pit_phase_remaining[driver_id] = max(0.0, duration - leftover)
        else:  # "out" finished -> rejoin the track
            new_compound = self._pit_tire[driver_id]
            entry = self._pit_entry_progress()
            exit_ = self._pit_exit_progress()
            pit_entry_stint = state.pit_count + 1
            state.in_pit = False
            state.pit_count += 1
            if entry is not None and exit_ is not None:
                if exit_ <= entry:
                    state.current_lap += 1
                    lap_start = getattr(state, "_lap_start_time", 0.0)
                    state.last_lap_time = max(
                        state.total_time - lap_start,
                        self.circuit.base_lap_time * 0.85,
                    )
                    if state.best_lap_time <= 0 or state.last_lap_time < state.best_lap_time:
                        state.best_lap_time = state.last_lap_time
                    self._record_lap_time(
                        driver_id,
                        state.current_lap,
                        state.last_lap_time,
                        self._pit_entry_compound.get(driver_id, new_compound),
                        pit_entry_stint,
                        pit_stop=True,
                    )
                    state._lap_start_time = state.total_time  # type: ignore[attr-defined]
                state.progress = exit_
            state.total_progress = state.current_lap + state.progress
            self._refresh_lap_variation(driver_id)
            self._clear_pit_state(driver_id)
            events.append(
                RaceEvent(
                    type="pit_exit",
                    driver=meta["abbreviation"],
                    message=f"{meta['full_name']} exits pits on {new_compound.value} tires",
                    message_ko=(
                        f"{meta['full_name']}가 {new_compound.value} 타이어로 피트를 빠져나옵니다"
                    ),
                )
            )

        if state.in_pit:
            self._sync_pit_race_progress(driver_id, state)

    def _clear_pit_state(self, driver_id: int) -> None:
        for store in (
            self._pit_phase,
            self._pit_phase_remaining,
            self._pit_phase_duration,
            self._pit_lane_half_time,
            self._pit_tire_change_time,
            self._pit_elapsed,
            self._pit_stop_elapsed,
            self._pit_tire,
            self._pit_entry_compound,
        ):
            store.pop(driver_id, None)

    def _pit_lane_progress(self, driver_id: int) -> float:
        """Position along the pit lane (0=entry, 1=exit) for an in-pit car."""
        phase = self._pit_phase.get(driver_id)
        if phase is None:
            return 0.0
        duration = self._pit_phase_duration.get(driver_id, 0.0)
        remaining = self._pit_phase_remaining.get(driver_id, 0.0)
        frac = 1.0 - (remaining / duration) if duration > 0 else 1.0
        frac = min(1.0, max(0.0, frac))
        box = PIT_BOX_LANE_FRACTION
        if phase == "in":
            return box * frac
        if phase == "stop":
            return box
        return box + (1.0 - box) * frac

    def _sync_pit_race_progress(
        self,
        driver_id: int,
        state: DriverRaceState,
    ) -> None:
        """Map pit-lane travel onto continuous lap distance for live order."""
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        if entry is None or exit_ is None:
            return

        lane_progress = self._pit_lane_progress(driver_id)
        route_distance = self._progress_distance(entry, exit_)
        state.total_progress = state.current_lap + entry + route_distance * lane_progress

    def _pit_lane_progress_rate(self, driver_id: int) -> float:
        """Pit-lane progress rate per game second for front-end prediction."""
        phase = self._pit_phase.get(driver_id)
        duration = self._pit_phase_duration.get(driver_id, 0.0)
        if phase is None or duration <= 0:
            return 0.0

        box = PIT_BOX_LANE_FRACTION
        if phase == "in":
            return box / duration
        if phase == "out":
            return (1.0 - box) / duration
        return 0.0

    def _record_lap_time(
        self,
        driver_id: int,
        lap: int,
        lap_time: float,
        tire_compound: TireCompound,
        stint: int,
        *,
        pit_stop: bool = False,
    ) -> None:
        self._lap_history.setdefault(driver_id, []).append(
            LapTimeInfo(
                lap=lap,
                lap_time=round(lap_time, 3),
                tire_compound=tire_compound.value,
                stint=stint,
                pit_stop=pit_stop,
            )
        )

    def _complete_lap(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        team: Team,
        lap_finish_time: float,
    ) -> list[RaceEvent]:
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

    def _run_ai_strategy(self) -> None:
        for driver_id, state in self.driver_states.items():
            if state.retired or state.finished:
                continue
            if driver_id in self.player_driver_ids:
                continue
            remaining = self.total_laps - state.current_lap
            tire_management = self._driver_meta[driver_id]["tire_management"]
            if should_pit(state, remaining, is_player=False, tire_management=tire_management):
                state.pit_request = choose_pit_tire(state, remaining)

    def _ai_sc_pit_decisions(self) -> None:
        """Let AI take the cheap "free" pit stop when the safety car is deployed.

        Decided once at SC deployment (not every tick). Only drivers whose tires
        are worn past ``SC_PIT_WEAR_THRESHOLD`` are eligible, and a per-driver
        probability keeps the whole field from stacking into the pits at once.
        """
        for driver_id, state in self.driver_states.items():
            if driver_id in self.player_driver_ids:
                continue
            if (
                state.retired
                or state.finished
                or state.in_pit
                or state.pit_request is not None
            ):
                continue
            remaining = self.total_laps - state.current_lap
            if remaining <= 3:
                continue
            if self._current_tire_wear(state) < SC_PIT_WEAR_THRESHOLD:
                continue
            if self.rng.random() < SC_PIT_PROBABILITY:
                state.pit_request = choose_pit_tire(state, remaining)

    def _run_ai_pace_modes(self, running_by_position: dict[int, DriverRaceState]) -> None:
        for driver_id, state in self.driver_states.items():
            if driver_id in self.player_driver_ids:
                continue
            if state.retired or state.finished or state.in_pit:
                continue
            if self._ai_pace_cooldown.get(driver_id, 0.0) > 0:
                continue

            car_ahead = running_by_position.get(state.position - 1)
            car_behind = running_by_position.get(state.position + 1)
            next_mode = self._choose_ai_pace_mode(state, car_ahead, car_behind)
            state.pace_mode = next_mode
            self._ai_pace_cooldown[driver_id] = self._ai_pace_decision_cooldown()

    def _update_positions(self) -> None:
        active = [
            s for s in self.driver_states.values() if not s.retired and not s.finished
        ]
        finished = [s for s in self.driver_states.values() if s.finished and not s.retired]
        retired = [s for s in self.driver_states.values() if s.retired]

        if self.race_phase == "green":
            active.sort(key=lambda s: (-s.total_progress, s.total_time))
        else:
            # Cars on track cannot overtake under SC/VSC, but a pit-lane car can
            # gain or lose places according to its live race distance.
            on_track = sorted((s for s in active if not s.in_pit), key=lambda s: s.position)
            in_pit = sorted(
                (s for s in active if s.in_pit),
                key=lambda s: (-s.total_progress, s.position),
            )
            active = on_track
            for pit_car in in_pit:
                insert_at = len(active)
                for index, other in enumerate(active):
                    ahead_by_distance = pit_car.total_progress > (
                        other.total_progress + PROGRESS_EPSILON
                    )
                    tied_ahead_by_order = (
                        abs(pit_car.total_progress - other.total_progress)
                        <= PROGRESS_EPSILON
                        and pit_car.position < other.position
                    )
                    if ahead_by_distance or tied_ahead_by_order:
                        insert_at = index
                        break
                active.insert(insert_at, pit_car)
        finished.sort(
            key=lambda s: (
                self._finish_order.index(s.driver_id)
                if s.driver_id in self._finish_order
                else len(self._finish_order),
                s.total_time,
            )
        )

        ordered = finished + active + retired
        for position, state in enumerate(ordered, start=1):
            state.position = position

    def _update_gaps(self) -> None:
        running = [s for s in self.driver_states.values() if not s.retired]
        if not running:
            return

        leader = next((s for s in running if s.position == 1), None)
        if leader is None:
            return

        for state in running:
            if state.driver_id == leader.driver_id:
                state.gap_to_leader = 0.0
            else:
                progress_gap = max(0.0, leader.total_progress - state.total_progress)
                state.gap_to_leader = self._progress_gap_to_seconds(progress_gap, state)

    def _check_race_finished(self) -> bool:
        active = [
            s for s in self.driver_states.values()
            if not s.retired and not s.finished
        ]
        return len(active) == 0

    def _format_gap(self, seconds: float) -> str:
        return f"+{max(0.0, seconds):.3f}"

    def _format_interval(self, seconds: float) -> str:
        if seconds <= 0.001:
            return "—"
        return f"+{seconds:.3f}"

    def _progress_gap_to_seconds(self, progress_gap: float, state: DriverRaceState) -> float:
        """Convert a lap-fraction gap into an approximate live timing gap."""
        if progress_gap <= 0:
            return 0.0
        lap_time = self._base_lap_time_for_state(state)
        lap_time *= self._phase_lap_time_factor()
        return progress_gap * lap_time

    def build_tick_state(self, events: list[RaceEvent] | None = None) -> RaceTickState:
        """Build WebSocket tick payload."""
        positions: list[DriverPositionInfo] = []
        sorted_states = sorted(
            self.driver_states.values(),
            key=lambda s: s.position,
        )

        running_by_pos = {
            s.position: s for s in sorted_states if not s.retired
        }

        for state in sorted_states:
            meta = self._driver_meta[state.driver_id]

            interval = "—"
            if not state.retired and state.position > 1:
                ahead = running_by_pos.get(state.position - 1)
                if ahead is not None:
                    progress_gap = max(0.0, ahead.total_progress - state.total_progress)
                    interval = self._format_interval(
                        self._progress_gap_to_seconds(progress_gap, state)
                    )

            if state.retired:
                gap = "—"
                interval = "—"
            elif state.finished:
                gap = "FIN"
                interval = "—"
            elif state.position == 1:
                gap = "LEADER"
            else:
                gap = self._format_gap(state.gap_to_leader)

            side_by_side_active = self._is_side_by_side_active(state.driver_id)
            state.side_by_side_active = side_by_side_active
            positions.append(
                DriverPositionInfo(
                    driver_id=state.driver_id,
                    name=meta["abbreviation"],
                    full_name=meta["full_name"],
                    team=meta["team_name"],
                    team_color=meta["team_color"],
                    position=state.position,
                    progress=round(state.progress, 4),
                    progress_rate=round(
                        0.0
                        if self.paused or state.in_pit or state.retired or state.finished
                        else self._progress_rate.get(state.driver_id, 0.0),
                        6,
                    ),
                    speed_kph=round(
                        0.0
                        if state.retired or state.finished
                        else state.speed_kph,
                        1,
                    ),
                    gap=gap,
                    interval=interval,
                    tire_compound=state.tire_compound.value,
                    tire_age=state.tire_age,
                    tire_wear=round(self._current_tire_wear(state), 3),
                    pace_mode=state.pace_mode.value,
                    last_lap_time=round(state.last_lap_time, 3),
                    best_lap_time=round(state.best_lap_time, 3),
                    in_pit=state.in_pit,
                    pit_count=state.pit_count,
                    pit_phase=self._pit_phase.get(state.driver_id),
                    pit_lane_progress=round(self._pit_lane_progress(state.driver_id), 4),
                    pit_lane_progress_rate=round(
                        0.0 if self.paused else self._pit_lane_progress_rate(state.driver_id),
                        6,
                    ),
                    pit_elapsed=round(self._pit_elapsed.get(state.driver_id, 0.0), 1),
                    pit_stop_elapsed=round(self._pit_stop_elapsed.get(state.driver_id, 0.0), 1),
                    lap_history=self._lap_history.get(state.driver_id, []),
                    retired=state.retired,
                    finished=state.finished,
                    drs_active=state.drs_active,
                    dirty_air_active=state.dirty_air_active,
                    side_by_side_active=side_by_side_active,
                )
            )

        return RaceTickState(
            lap=self.current_lap,
            total_laps=self.total_laps,
            weather=self.weather,
            safety_car=self.safety_car,
            race_phase=self.race_phase,
            race_phase_remaining_seconds=round(self._race_phase_remaining_seconds(), 1),
            race_phase_remaining_laps=self._race_phase_remaining_laps(),
            safety_car_stage=self.safety_car_stage,
            safety_car_visible=self._safety_car_visible,
            safety_car_route=self._safety_car_route,
            safety_car_progress=round(
                (self._safety_car_total_progress or 0.0) % 1.0,
                6,
            ),
            safety_car_progress_rate=round(self._safety_car_progress_rate, 6),
            safety_car_pit_lane_progress=round(self._safety_car_pit_lane_progress, 6),
            safety_car_queue_formed=self._safety_car_queue_formed,
            overtaking_allowed=self.race_phase == "green",
            restart_line_progress=0.0,
            pit_window_open=self.pit_window_open,
            race_elapsed=round(self.race_elapsed, 3),
            speed_multiplier=self.speed_multiplier,
            paused=self.paused,
            positions=positions,
            events=events or [],
        )

    def build_results(self) -> list[dict]:
        """Final classification."""
        results = []
        for state in sorted(self.driver_states.values(), key=lambda s: s.position):
            meta = self._driver_meta[state.driver_id]
            results.append(
                {
                    "position": state.position,
                    "driver_id": state.driver_id,
                    "name": meta["abbreviation"],
                    "full_name": meta["full_name"],
                    "team": meta["team_name"],
                    "total_time": round(state.total_time, 3),
                    "retired": state.retired,
                }
            )
        return results
