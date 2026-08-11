"""FULL overtake, defense and maneuver-group racecraft operations.

RaceEngine inherits RacecraftMixin so existing call sites and tests keep the
same method and type names.  Behavior is unchanged; this module only relocates
the racecraft domain.  Do not confuse with ``racecraft_benchmark.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin, sqrt
from typing import Any

from models.schemas import DriverRaceState, PaceMode, RaceEvent, TrackSegmentType
from .collision import BodyPose, oriented_body_overlap, oriented_body_separation_m
from .local_trajectory_planner import LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
from simulation.track_geometry import segment_at_progress
from simulation.track_physics import (
    DRIVING_LINE_DEFENSIVE,
    DRIVING_LINE_INSIDE,
    DRIVING_LINE_OUTSIDE,
    DRIVING_LINE_RACING,
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
)
from simulation.vehicle_dynamics import maximum_braking_deceleration_mps2
from .vehicle_physics import PHYSICS_MAX_LATERAL_SPEED_MPS, PHYSICS_STEP_SECONDS

PROGRESS_EPSILON = 1e-9

ATTACK_LINE_CHOICE_INTERVAL_SECONDS = 0.20
MANEUVER_PULL_OUT_MIN_CLEARANCE_M = 0.15
MANEUVER_REAR_PLANNING_CLEARANCE_M = 20.0
TRAFFIC_GAP_SECONDS = 1.0
DIRTY_AIR_MAX_PENALTY = 0.32
TRAFFIC_ATTACK_MAX_BONUS = 0.12
DRS_MAX_BONUS = 0.30
DEFENSE_MAX_BLOCK = 0.22
BATTLE_EVENT_GAP_SECONDS = 1.00
BATTLE_EVENT_COOLDOWN_SECONDS = 10.0
BATTLE_EVENT_SEGMENTS = {
    TrackSegmentType.STRAIGHT,
    TrackSegmentType.HEAVY_BRAKING,
}
BATTLE_ATTACK_EFFECT_SECONDS = 4.5
BATTLE_DEFEND_EFFECT_SECONDS = 3.5
BATTLE_ATTACKER_LAP_TIME_DELTA = -3.2
BATTLE_DEFENDER_PRESSURE_LAP_TIME_DELTA = 1.15
BATTLE_DEFEND_ATTACKER_LAP_TIME_DELTA = 2.35
BATTLE_ATTACK_TIRE_USAGE_MULTIPLIER = 1.08
BATTLE_DEFEND_TIRE_USAGE_MULTIPLIER = 1.04
BRAKE_INPUT_ERROR_SECONDS = 0.80
BRAKE_INPUT_ERROR_INTENSITY = 0.22
THROTTLE_INPUT_ERROR_SECONDS = 1.20
THROTTLE_INPUT_ERROR_INTENSITY = 0.22
BATTLE_SIDE_BY_SIDE_SCORE_MARGIN = 0.06
BATTLE_FORCED_WIDE_SCORE_MARGIN = 0.18
MANEUVER_OVERLAP_GAP_M = PHYSICAL_CAR_LENGTH_M * 1.15
MANEUVER_BREAK_GAP_M = 65.0
MANEUVER_CLEAR_GAP_M = PHYSICAL_CAR_LENGTH_M
MANEUVER_MAX_DURATION_SECONDS = 16.0
MANEUVER_APPROACH_MAX_SECONDS = 10.0
MANEUVER_PREP_BUMPER_GAP_M = 1.25
MANEUVER_TOW_PREP_BUMPER_GAP_M = 2.50
MANEUVER_ATTACK_DECISION_BUMPER_GAP_M = 22.0
MANEUVER_HEAVY_BRAKING_MAX_BUMPER_GAP_M = 4.5
MANEUVER_PULL_OUT_BUMPER_GAP_M = 4.5
MANEUVER_CLEARANCE_MARGIN_M = 0.25
MANEUVER_CONTACT_BUFFER_M = 0.25
MANEUVER_LIVE_CLEARANCE_BUFFER_M = 0.85
MANEUVER_PASS_CLEARANCE_MARGIN_M = 0.10
MANEUVER_RESERVATION_DISTANCE_M = 160.0
MANEUVER_MIN_STRAIGHT_DISTANCE_M = 70.0
MANEUVER_PULL_OUT_CORNER_PREVIEW_SECONDS = 1.8
MANEUVER_PULL_OUT_MIN_CORNER_PREVIEW_M = 55.0
MANEUVER_LINE_COMMIT_PREVIEW_SECONDS = 1.55
MANEUVER_LINE_COMMIT_MIN_PREVIEW_M = 70.0
MANEUVER_CORNER_LUNGE_EXTENSION_M = 4.0
MANEUVER_CLEAR_HOLD_SECONDS = 0.35
MANEUVER_ABORT_HOLD_SECONDS = 0.8
MANEUVER_YIELD_MIN_SECONDS = 0.12
MANEUVER_YIELD_MAX_SECONDS = 4.0
MANEUVER_ABORT_REJOIN_SECONDS = 1.25
MANEUVER_REJOIN_TOLERANCE_M = 0.45
CORNER_ENTRY_TURN_SIGNAL = 0.08
CORNER_EXIT_TURN_SIGNAL = 0.035
CORNER_EXIT_HOLD_SECONDS = 0.30
CORNER_CORRIDOR_MARGIN_M = 0.35
CORNER_EDGE_ALLOWANCE_M = 0.35
CORNER_INSIDE_BRAKING_FACTOR = 0.98
CORNER_INSIDE_TRACTION_FACTOR = 0.96
CORNER_PREDICTION_HORIZON_SECONDS = 2.5
CORNER_PREDICTION_STEP_SECONDS = 0.1
CORNER_CORRIDOR_TRANSITION_SECONDS = 0.65
CORNER_APEX_LOOKAHEAD_M = 160.0
CORNER_BRAKING_MARGIN_M = 5.0
BATTLE_CORNER_EXIT_EFFECT_SECONDS = 2.5
BATTLE_FORCED_WIDE_EFFECT_SECONDS = 4.0
BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA = 0.45
BATTLE_INSIDE_ENTRY_LAP_TIME_DELTA = -0.15
BATTLE_OUTSIDE_ENTRY_LAP_TIME_DELTA = 0.12
BATTLE_INSIDE_EXIT_LAP_TIME_DELTA = 0.75
BATTLE_OUTSIDE_EXIT_LAP_TIME_DELTA = -0.55
BATTLE_DEFENSIVE_LINE_LAP_TIME_DELTA = 0.18
BATTLE_DEFENSIVE_LINE_EXIT_LAP_TIME_DELTA = 0.35
BATTLE_RUN_WIDE_EFFECT_SECONDS = 4.0
BATTLE_RUN_WIDE_LAP_TIME_DELTA = 2.6
BATTLE_FORCED_WIDE_ATTACKER_LAP_TIME_DELTA = -2.2
BATTLE_FORCED_WIDE_DEFENDER_LAP_TIME_DELTA = 3.2
BATTLE_FORCED_WIDE_EXIT_DELAY_SECONDS = 1.8
BATTLE_FORCED_WIDE_RESULT_BASE_PROBABILITY = 0.42
BATTLE_SIDE_BY_SIDE_TIRE_USAGE_MULTIPLIER = 1.06
BATTLE_INSIDE_LINE_TIRE_USAGE_MULTIPLIER = 1.02
BATTLE_OUTSIDE_LINE_TIRE_USAGE_MULTIPLIER = 1.01
BATTLE_DEFENSIVE_LINE_TIRE_USAGE_MULTIPLIER = 1.02
BATTLE_RUN_WIDE_TIRE_USAGE_MULTIPLIER = 1.08
BATTLE_FORCED_WIDE_TIRE_USAGE_MULTIPLIER = 1.10
BATTLE_SIDE_BY_SIDE_RESULT_BASE_PROBABILITY = 0.18
MANEUVER_GROUP_MAX_CARS = 4
MANEUVER_GROUP_CORRIDOR_MARGIN_M = 0.35
MANEUVER_GROUP_LONGITUDINAL_WINDOW_M = 24.0
ATTACK_LINE_INSIDE = "inside"
ATTACK_LINE_OUTSIDE = "outside"
DEFENDER_LINE_RACING = "racing_line"
DEFENDER_LINE_DEFENSIVE = "defensive_line"
AI_ATTACK_GAP_SECONDS = 1.0
AI_ATTACK_EXIT_GAP_SECONDS = 1.35
AI_DEFEND_GAP_SECONDS = 0.85
AI_DEFEND_EXIT_GAP_SECONDS = 1.15

@dataclass
class BattleEffect:
    """Short local pace effect created by an overtake or defense event."""

    remaining_seconds: float
    lap_time_delta: float
    tire_usage_multiplier: float = 1.0


@dataclass
class SideBySideBattle:
    """Physical overtake maneuver shared by an attacker and defender."""

    attacker_id: int
    defender_id: int
    remaining_seconds: float
    attacker_line: str
    defender_line: str
    segment_name: str = "the corner"
    phase: str = "approach"
    elapsed_seconds: float = 0.0
    phase_elapsed_seconds: float = 0.0
    committed_seconds: float = 0.0
    corner_entry_candidate: bool = False
    line_committed: bool = False
    line_commit_progress: float = 0.0
    line_commit_distance_m: float = 0.0
    attacker_committed_lateral_bias_m: float = 0.0
    defender_committed_lateral_bias_m: float = 0.0
    corner_active: bool = False
    corner_authorized: bool = False
    corner_turn_direction: int = 0
    corner_entry_advantage_m: float = 0.0
    corner_entry_brake_delta: float = 0.0
    corner_exit_hold_seconds: float = 0.0
    corner_count: int = 0
    forced_wide_driver_id: int | None = None
    trajectory_candidate_id: str = ""
    trajectory_lateral_bias_m: float = 0.0
    trajectory_minimum_clearance_m: float = 0.0
    trajectory_authorized: bool = False
    corner_prediction_horizon_seconds: float = 0.0
    corner_prediction_minimum_clearance_m: float = 0.0
    corner_prediction_braking_margin_m: float = 0.0
    corner_prediction_rejection_reason: str = ""
    corner_attacker_trajectory_candidate_id: str = ""
    corner_defender_trajectory_candidate_id: str = ""
    corner_trajectory_replan_count: int = 0
    corner_inside_driver_id: int | None = None
    pass_confirmed: bool = False
    pass_event_emitted: bool = False
    pass_clearance_m: float = 0.0
    yielding_driver_id: int | None = None
    yield_attacker_lateral_offset_m: float = 0.0
    yield_defender_lateral_offset_m: float = 0.0
    abort_attacker_lateral_offset_m: float = 0.0
    abort_defender_lateral_offset_m: float = 0.0
    abort_rejoin_started: bool = False
    abort_rejoin_elapsed_seconds: float = 0.0


@dataclass(frozen=True)
class ManeuverGroup:
    """Connected 2/3/4-car occupancy replacing exclusive pair ownership.

    Pair maneuvers remain the rule-resolution edges.  The group is the
    authoritative shared occupancy around those edges, so one driver may be
    constrained by vehicles on both sides without either edge being deleted.
    """

    group_id: str
    member_ids: tuple[int, ...]
    pair_keys: tuple[tuple[int, int], ...]
    phase: str
    minimum_lateral_m: float
    maximum_lateral_m: float
    corner_turn_direction: int = 0
    corner_priority_ids: tuple[int, ...] = ()

    @property
    def size(self) -> int:
        return len(self.member_ids)


@dataclass(frozen=True)
class OvertakeOpportunityAssessment:
    """Explain one AI attack/hold decision using the live physical inputs."""

    decision: str
    viable: bool
    reason_code: str
    segment_name: str | None
    segment_type: str | None
    bumper_gap_m: float
    remaining_distance_m: float
    available_width_m: float
    required_width_m: float
    relative_speed_mps: float
    expected_closing_speed_mps: float
    distance_to_clear_m: float
    available_closing_distance_m: float
    tow_strength: float
    drs_active: bool
    attack_probability: float
    mistake_probability: float


@dataclass(frozen=True)
class LocalPullOutDecision:
    """Collision-free lateral candidate used to begin a straight-line pass."""

    candidate_id: str
    lateral_bias_m: float
    target_lateral_offset_m: float
    minimum_clearance_m: float
    attacker_line: str


@dataclass(frozen=True)
class PendingOvertakeCommand:
    """A command-stage trajectory choice awaiting rules confirmation."""

    attacker_id: int
    defender_id: int
    decision: LocalPullOutDecision


@dataclass(frozen=True)
class BattleIntent:
    """Command-stage overtake decision committed by RULES after physics."""

    attacker_id: int
    defender_id: int
    event_kind: str
    segment_name: str
    attacker_line: str = ATTACK_LINE_INSIDE
    defender_line: str = DEFENDER_LINE_RACING
    trajectory_decision: LocalPullOutDecision | None = None


@dataclass(frozen=True)
class CornerCorridorPrediction:
    """Short-horizon physical validation for two-car corner occupancy."""

    authorized: bool
    rejection_reason: str
    horizon_seconds: float
    minimum_clearance_m: float
    braking_margin_m: float


@dataclass
class ForcedWideAftermath:
    """A delayed corner-exit result after a forceful inside move."""

    attacker_id: int
    defender_id: int
    remaining_seconds: float
    segment_name: str = "the corner"


class RacecraftMixin:
    """Side-by-side battles, maneuver groups, attack/defend decisions."""

    def _init_racecraft_state(self) -> None:
        """Initialize overtake / side-by-side / maneuver-group state."""
        self._pending_overtake_commands: dict[int, PendingOvertakeCommand] = {}
        self._battle_event_cooldown: dict[int, float] = {}
        self._battle_effects: dict[int, BattleEffect] = {}
        self._side_by_side_battles: dict[tuple[int, int], SideBySideBattle] = {}
        self._maneuver_groups: dict[str, ManeuverGroup] = {}
        self._maneuver_group_signatures: set[tuple[int, ...]] = set()
        self._forced_wide_aftermaths: dict[tuple[int, int], ForcedWideAftermath] = {}

    def _maneuver_execution_factors(self, state: DriverRaceState) -> tuple[float, float]:
        """Map racecraft to small physical control margins during a maneuver."""
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if battle is None or battle.phase in {"approach", "merge", "yield", "abort"}:
            return 1.0, 1.0
        meta = self._driver_meta[state.driver_id]
        skill = (
            meta["overtaking"]
            if state.driver_id == battle.attacker_id
            else meta["defending"]
        )
        # This is control quality, not an artificial speed bonus: a skilled
        # driver uses slightly more of the available tyre force under braking
        # and on corner exit without changing engine power or aero drag.
        execution = 0.985 + 0.03 * skill
        return execution, execution

    def _cancel_maneuvers_for_neutralization(self) -> None:
        """Remove competitive paths and wake penalties as SC/VSC takes control."""
        affected_driver_ids = {
            driver_id
            for battle in self._side_by_side_battles.values()
            for driver_id in (battle.attacker_id, battle.defender_id)
        }
        self._side_by_side_battles.clear()
        self._maneuver_groups.clear()
        self._maneuver_group_signatures.clear()
        for driver_id in affected_driver_ids:
            self._local_trajectory_plans.pop(driver_id, None)
            self._local_trajectory_plan_ages[driver_id] = (
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
            )
            self._local_trajectory_selected_ages[driver_id] = 0.0
        for state in self.driver_states.values():
            self._reset_wake_state(state)

    def _bumper_gap_m(
        self,
        follower: DriverRaceState,
        leader: DriverRaceState,
    ) -> float:
        center_gap_m = self._center_gap_m(follower, leader)
        half_lengths_m = 0.5 * (follower.car_length_m + leader.car_length_m)
        return center_gap_m - half_lengths_m

    def _pair_maneuver(
        self,
        follower: DriverRaceState,
        leader: DriverRaceState,
    ) -> SideBySideBattle | None:
        battle = self._side_by_side_battles.get(
            self._battle_pair_key(follower.driver_id, leader.driver_id)
        )
        if (
            battle is not None
            and battle.attacker_id == follower.driver_id
            and battle.defender_id == leader.driver_id
        ):
            return battle
        return None

    def _maneuver_following_gaps_m(
        self,
        follower: DriverRaceState,
        leader: DriverRaceState,
        battle: SideBySideBattle,
    ) -> tuple[float, float]:
        """Return progressive centre-to-centre desired/minimum gaps."""
        half_lengths_m = 0.5 * (follower.car_length_m + leader.car_length_m)
        if battle.phase == "approach":
            return (
                half_lengths_m + MANEUVER_PREP_BUMPER_GAP_M,
                half_lengths_m + MANEUVER_CONTACT_BUFFER_M,
            )
        if battle.phase in {"yield", "abort"}:
            return (
                half_lengths_m + MANEUVER_PREP_BUMPER_GAP_M,
                half_lengths_m + MANEUVER_CONTACT_BUFFER_M,
            )

        lateral_separation_m = abs(
            follower.lateral_offset_m - leader.lateral_offset_m
        )
        full_clearance_m = PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
        clearance_ratio = min(1.0, max(0.0, lateral_separation_m / full_clearance_m))
        desired_gap_m = (
            half_lengths_m + MANEUVER_PREP_BUMPER_GAP_M
        ) * (1.0 - clearance_ratio)

        if lateral_separation_m <= PHYSICAL_CAR_WIDTH_M:
            minimum_gap_m = half_lengths_m + MANEUVER_CONTACT_BUFFER_M
        else:
            margin_ratio = min(
                1.0,
                (lateral_separation_m - PHYSICAL_CAR_WIDTH_M)
                / MANEUVER_CLEARANCE_MARGIN_M,
            )
            minimum_gap_m = (
                half_lengths_m + MANEUVER_CONTACT_BUFFER_M
            ) * (1.0 - margin_ratio)
        return max(minimum_gap_m, desired_gap_m), minimum_gap_m

    def _physics_v2_passing_authorized(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState,
    ) -> bool:
        unlapping = state.driver_id in self._sc_unlap_driver_ids
        if (
            not unlapping
            and not self._overtaking_candidate_allowed(state, car_ahead)
        ):
            return False
        lateral_clearance = abs(state.lateral_offset_m - car_ahead.lateral_offset_m)
        required_lateral_clearance = self._physical_lateral_clearance_required_m(
            state,
            car_ahead,
        )
        if (
            self.start_sequence_enabled
            and self.race_started
            and self._grid_launch_distance_m(state) < self._grid_launch_merge_distance_m()
            and self._grid_launch_distance_m(car_ahead) < self._grid_launch_merge_distance_m()
            and lateral_clearance
            >= required_lateral_clearance
        ):
            return True
        battle = self._pair_maneuver(state, car_ahead)
        if (
            battle is not None
            and battle.phase == "overlap"
            and lateral_clearance
            >= required_lateral_clearance
        ):
            # Corner corridors intentionally use the racing-line longitudinal
            # physics for both cars. Their distinct lateral targets still make
            # them separate physical lanes, so the follower constraint must not
            # pin the attacker behind once genuine overlap is established.
            return True
        if (
            not unlapping
            and self._physics_v2_virtual_line(state)
            == self._physics_v2_virtual_line(car_ahead)
        ):
            return False
        return lateral_clearance >= required_lateral_clearance

    def _physical_lateral_clearance_required_m(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
    ) -> float:
        """Return body-safe lateral clearance including each car's yaw."""

        def lateral_half_extent(state: DriverRaceState) -> float:
            heading = abs(float(state.slip_angle_rad))
            return (
                0.5 * state.car_width_m * abs(cos(heading))
                + 0.5 * state.car_length_m * abs(sin(heading))
            )

        return (
            lateral_half_extent(first)
            + lateral_half_extent(second)
            + MANEUVER_CLEARANCE_MARGIN_M
        )

    def _physical_longitudinal_half_extents_m(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
    ) -> float:
        """Return summed longitudinal body extents including yaw."""

        def longitudinal_half_extent(state: DriverRaceState) -> float:
            heading = abs(float(state.slip_angle_rad))
            return (
                0.5 * state.car_length_m * abs(cos(heading))
                + 0.5 * state.car_width_m * abs(sin(heading))
            )

        return longitudinal_half_extent(first) + longitudinal_half_extent(second)

    def _battle_lap_time_delta(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
    ) -> float:
        """Return traffic lap-time effect when following another car closely."""
        effects = self._update_wake_state(state, car_ahead)
        if car_ahead is None or state.in_pit or state.retired or state.finished:
            return 0.0

        progress_gap = car_ahead.total_progress - state.total_progress
        gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
        if gap_seconds <= 0 or gap_seconds > TRAFFIC_GAP_SECONDS:
            return 0.0

        attacker = self._driver_meta[state.driver_id]
        defender = self._driver_meta[car_ahead.driver_id]
        closeness = max(
            effects.wake_strength,
            0.35 + 0.65 * (
                1.0 - min(gap_seconds, TRAFFIC_GAP_SECONDS) / TRAFFIC_GAP_SECONDS
            ),
        )
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
        if (
            not self._overtaking_candidate_allowed(state, car_ahead)
            or self._start_overtake_lockout_remaining > 0.0
        ):
            return 0.0
        if (
            self.start_sequence_enabled
            and self._grid_launch_distance_m(state) < self._grid_launch_merge_distance_m()
            and car_ahead is not None
            and not self._grid_launch_pair_is_physically_separate(state, car_ahead)
        ):
            return 0.0
        if car_ahead is None or gap_seconds <= 0 or gap_seconds > BATTLE_EVENT_GAP_SECONDS:
            return 0.0
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is None or not self._segment_overtake_start_allowed(segment):
            return 0.0

        attacker = self._driver_meta[state.driver_id]
        closeness = 1.0 - min(gap_seconds, BATTLE_EVENT_GAP_SECONDS) / BATTLE_EVENT_GAP_SECONDS
        segment_base = 0.040 if segment.type == TrackSegmentType.STRAIGHT else 0.017
        drs_bonus = 0.018 if state.drs_active else 0.0
        attack_bonus = 0.008 * attacker["overtaking"] * self._pace_mode_attack_factor(state)
        difficulty_penalty = 0.007 * self.circuit.overtaking_difficulty
        opportunity_adjustment = 0.006 * (0.5 - segment.overtake_risk)
        return max(
            0.0,
            min(
                0.075,
                segment_base
                + drs_bonus
                + attack_bonus
                - difficulty_penalty
                + opportunity_adjustment
                + 0.014 * closeness,
            ),
        )

    @staticmethod

    def _probability_for_elapsed_time(
        reference_probability: float,
        elapsed_seconds: float,
    ) -> float:
        """Convert a 50 Hz reference chance into a time-invariant hazard.

        AI decisions must not become more aggressive merely because physics is
        sampled more often.  At the current 20 ms step this returns the original
        probability exactly; smaller or larger steps retain the same chance over
        equal simulated time.
        """
        probability = min(1.0, max(0.0, reference_probability))
        elapsed = max(0.0, elapsed_seconds)
        if probability <= 0.0 or elapsed <= 0.0:
            return 0.0
        if probability >= 1.0:
            return 1.0
        return 1.0 - (1.0 - probability) ** (elapsed / PHYSICS_STEP_SECONDS)

    def _segment_overtake_start_allowed(self, segment) -> bool:
        """Use an explicit circuit override, falling back to the legacy types."""
        if segment.overtake_start_allowed is not None:
            return segment.overtake_start_allowed
        return segment.type in BATTLE_EVENT_SEGMENTS

    def _remaining_segment_distance_m(
        self,
        state: DriverRaceState,
        segment,
    ) -> float:
        normalized = state.progress % 1.0
        if segment.start <= segment.end:
            remaining = max(0.0, segment.end - normalized)
        elif normalized >= segment.start:
            remaining = (1.0 - normalized) + segment.end
        else:
            remaining = max(0.0, segment.end - normalized)
        return remaining * self.track_length_m

    def _overtake_opportunity_is_viable(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState,
    ) -> bool:
        """Reject attacks that cannot physically reach overlap in the available zone."""
        return self._overtake_opportunity_assessment(state, car_ahead).viable

    def _overtake_opportunity_assessment(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState,
    ) -> OvertakeOpportunityAssessment:
        """Return the same attack gate with benchmark-friendly reason data."""
        segment = segment_at_progress(self.circuit, state.progress)
        bumper_gap_m = self._bumper_gap_m(state, car_ahead)
        minimum_lateral, maximum_lateral = (
            self._track_surface.safety_lateral_bounds(state.progress)
        )
        available_width_m = maximum_lateral - minimum_lateral
        required_width_m = (
            state.car_width_m
            + car_ahead.car_width_m
            + 2.0 * MANEUVER_CLEARANCE_MARGIN_M
        )
        relative_speed_mps = (
            state.speed_kph - car_ahead.speed_kph
        ) / 3.6
        remaining_distance_m = (
            self._remaining_segment_distance_m(state, segment)
            if segment is not None
            else 0.0
        )
        distance_to_clear_m = max(
            0.0,
            self._center_gap_m(state, car_ahead)
            + 0.5 * (state.car_length_m + car_ahead.car_length_m)
            + MANEUVER_PASS_CLEARANCE_MARGIN_M,
        )
        expected_closing_speed_mps = 0.0
        available_closing_distance_m = 0.0

        def assessment(
            viable: bool,
            reason_code: str,
        ) -> OvertakeOpportunityAssessment:
            gap_seconds = self._progress_gap_to_seconds(
                car_ahead.total_progress - state.total_progress,
                state,
            )
            active_maneuver = self._pair_maneuver(state, car_ahead) is not None
            return OvertakeOpportunityAssessment(
                decision=("attack" if viable else ("abort" if active_maneuver else "hold")),
                viable=viable,
                reason_code=reason_code,
                segment_name=(segment.name if segment is not None else None),
                segment_type=(
                    getattr(segment.type, "value", str(segment.type))
                    if segment is not None
                    else None
                ),
                bumper_gap_m=bumper_gap_m,
                remaining_distance_m=remaining_distance_m,
                available_width_m=available_width_m,
                required_width_m=required_width_m,
                relative_speed_mps=relative_speed_mps,
                expected_closing_speed_mps=expected_closing_speed_mps,
                distance_to_clear_m=distance_to_clear_m,
                available_closing_distance_m=available_closing_distance_m,
                tow_strength=state.tow_strength,
                drs_active=state.drs_active,
                attack_probability=self._battle_event_probability(
                    state,
                    car_ahead,
                    gap_seconds,
                ),
                mistake_probability=self._battle_mistake_probability(
                    state,
                    gap_seconds,
                ),
            )

        if not self._overtaking_candidate_allowed(state, car_ahead):
            return assessment(False, "race_control_or_state_block")
        if segment is None:
            return assessment(False, "segment_missing")
        if not self._segment_overtake_start_allowed(segment):
            return assessment(False, "segment_disallows_attack")
        if bumper_gap_m < 0.0:
            return assessment(False, "attacker_already_overlapping")
        if bumper_gap_m > MANEUVER_ATTACK_DECISION_BUMPER_GAP_M:
            return assessment(False, "gap_too_large")
        if segment.type == TrackSegmentType.HEAVY_BRAKING:
            if bumper_gap_m > MANEUVER_HEAVY_BRAKING_MAX_BUMPER_GAP_M:
                return assessment(False, "braking_gap_too_large")
            if available_width_m < required_width_m:
                return assessment(False, "insufficient_track_width")
            return assessment(True, "heavy_braking_window_open")

        if remaining_distance_m < MANEUVER_MIN_STRAIGHT_DISTANCE_M:
            return assessment(False, "insufficient_segment_distance")

        speed_mps = max(30.0, state.speed_kph / 3.6)
        attacker = self._driver_meta[state.driver_id]
        defender = self._driver_meta[car_ahead.driver_id]
        racecraft_delta = max(
            0.0,
            attacker["overtaking"] - defender["defending"],
        )
        expected_closing_speed_mps = max(
            max(0.0, relative_speed_mps),
            0.9
            + 3.0 * state.tow_strength
            + (2.8 if state.drs_active else 0.0)
            + 1.5 * racecraft_delta,
        )
        available_time_s = remaining_distance_m / speed_mps
        available_closing_distance_m = expected_closing_speed_mps * available_time_s
        if distance_to_clear_m > available_closing_distance_m:
            return assessment(False, "insufficient_closing_distance")
        return assessment(True, "straight_window_open")

    def explain_overtake_decision(
        self,
        attacker_id: int,
        defender_id: int,
    ) -> OvertakeOpportunityAssessment:
        """Public deterministic diagnostic used by racecraft benchmarks."""
        attacker = self.driver_states[attacker_id]
        defender = self.driver_states[defender_id]
        self._update_wake_state(attacker, defender, [defender])
        return self._overtake_opportunity_assessment(attacker, defender)

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
            + 0.08 * (segment.overtake_risk - 0.5)
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

    def _maneuver_group_capacity_for(
        self,
        member_ids: set[int],
    ) -> int:
        """Return physical side-by-side capacity at the shared track section."""
        states = [self.driver_states[driver_id] for driver_id in member_ids]
        if not states:
            return 0
        bounds = [
            self._physics_v2_nominal_lateral_bounds(state)
            or self._track_surface.safety_lateral_bounds(state.progress)
            for state in states
        ]
        common_minimum_m = max(bound[0] for bound in bounds)
        common_maximum_m = min(bound[1] for bound in bounds)
        usable_width_m = max(0.0, common_maximum_m - common_minimum_m)
        representative_width_m = max(state.car_width_m for state in states)
        slot_width_m = representative_width_m + MANEUVER_GROUP_CORRIDOR_MARGIN_M
        return max(
            1,
            min(
                MANEUVER_GROUP_MAX_CARS,
                int(
                    (usable_width_m + MANEUVER_GROUP_CORRIDOR_MARGIN_M)
                    // max(0.1, slot_width_m)
                ),
            ),
        )

    def _battle_component_members(self, seed_ids: set[int]) -> set[int]:
        """Collect every driver connected to either seed by an active edge."""
        members = set(seed_ids)
        changed = True
        while changed:
            changed = False
            for battle in self._side_by_side_battles.values():
                if battle.phase in {"yield", "abort", "merge"}:
                    continue
                pair = {battle.attacker_id, battle.defender_id}
                if members & pair and not pair <= members:
                    members.update(pair)
                    changed = True
        return members

    def _can_add_maneuver_edge(self, attacker_id: int, defender_id: int) -> bool:
        """Allow another pair edge only when the resulting group fits the road."""
        key = self._battle_pair_key(attacker_id, defender_id)
        if key in self._side_by_side_battles:
            return False
        members = self._battle_component_members({attacker_id, defender_id})
        if len(members) > MANEUVER_GROUP_MAX_CARS:
            return False
        states = [self.driver_states[driver_id] for driver_id in members]
        longitudinal_span_m = (
            max(state.total_progress for state in states)
            - min(state.total_progress for state in states)
        ) * self.track_length_m
        if longitudinal_span_m > MANEUVER_GROUP_LONGITUDINAL_WINDOW_M:
            return False
        return len(members) <= self._maneuver_group_capacity_for(members)

    def _maneuver_group_corner_order(
        self,
        member_ids: tuple[int, ...],
        turn_direction: int,
    ) -> tuple[int, ...]:
        """Rank corner rights by front-axle overlap, then current inside slot."""
        direction = 1 if turn_direction >= 0 else -1
        return tuple(
            sorted(
                member_ids,
                key=lambda driver_id: (
                    -(
                        self.driver_states[driver_id].total_progress
                        * self.track_length_m
                        + 0.5 * self.driver_states[driver_id].wheelbase_m
                    ),
                    -direction
                    * self.driver_states[driver_id].lateral_offset_m,
                    driver_id,
                ),
            )
        )

    def _rebuild_maneuver_groups(self) -> list[RaceEvent]:
        """Build connected group occupancy and emit only size transitions."""
        adjacency: dict[int, set[int]] = {}
        active_pairs: dict[tuple[int, int], SideBySideBattle] = {}
        for key, battle in self._side_by_side_battles.items():
            if battle.phase in {"yield", "abort", "merge"}:
                continue
            active_pairs[key] = battle
            adjacency.setdefault(battle.attacker_id, set()).add(battle.defender_id)
            adjacency.setdefault(battle.defender_id, set()).add(battle.attacker_id)

        groups: dict[str, ManeuverGroup] = {}
        visited: set[int] = set()
        for seed in sorted(adjacency):
            if seed in visited:
                continue
            members: set[int] = set()
            pending = [seed]
            while pending:
                driver_id = pending.pop()
                if driver_id in visited:
                    continue
                visited.add(driver_id)
                members.add(driver_id)
                pending.extend(sorted(adjacency.get(driver_id, ()), reverse=True))
            ordered_members = tuple(
                sorted(
                    members,
                    key=lambda driver_id: (
                        self.driver_states[driver_id].lateral_offset_m,
                        driver_id,
                    ),
                )
            )
            pair_keys = tuple(
                sorted(
                    key
                    for key in active_pairs
                    if set(key) <= members
                )
            )
            phases = {active_pairs[key].phase for key in pair_keys}
            overlap_edge_count = sum(
                active_pairs[key].phase in {"overlap", "clear"}
                for key in pair_keys
            )
            fully_parallel = overlap_edge_count >= len(members) - 1
            phase = (
                "four_wide"
                if len(members) == 4 and fully_parallel
                else "three_wide"
                if len(members) == 3 and fully_parallel
                else "overlap"
                if "overlap" in phases
                else "approach"
            )
            group_id = "mg-" + "-".join(str(item) for item in sorted(members))
            representative = max(
                (self.driver_states[item] for item in members),
                key=lambda state: state.total_progress,
            )
            turn_signal = self._track_physics.at_progress(
                representative.progress
            ).turn_signal
            corner_turn_direction = (
                1
                if turn_signal >= CORNER_ENTRY_TURN_SIGNAL
                else -1
                if turn_signal <= -CORNER_ENTRY_TURN_SIGNAL
                else 0
            )
            corner_priority_ids = (
                self._maneuver_group_corner_order(
                    tuple(sorted(members)),
                    corner_turn_direction,
                )
                if corner_turn_direction != 0
                else ()
            )
            groups[group_id] = ManeuverGroup(
                group_id=group_id,
                member_ids=ordered_members,
                pair_keys=pair_keys,
                phase=phase,
                minimum_lateral_m=min(
                    self.driver_states[item].lateral_offset_m for item in members
                ),
                maximum_lateral_m=max(
                    self.driver_states[item].lateral_offset_m for item in members
                ),
                corner_turn_direction=corner_turn_direction,
                corner_priority_ids=corner_priority_ids,
            )

        next_signatures = {
            tuple(sorted(group.member_ids))
            for group in groups.values()
            if group.phase in {"three_wide", "four_wide"}
        }
        events: list[RaceEvent] = []
        for signature in sorted(next_signatures - self._maneuver_group_signatures):
            label = f"{len(signature)}-wide"
            names = [self._driver_meta[item]["abbreviation"] for item in signature]
            events.append(
                RaceEvent(
                    type="maneuver_group_formed",
                    driver=names[0],
                    message=f"{' / '.join(names)} run {label}",
                    message_ko=f"{' / '.join(names)} 차량이 {label} 대열을 형성합니다",
                    payload={
                        "member_ids": ",".join(str(item) for item in signature),
                        "size": len(signature),
                        "phase": "four_wide" if len(signature) == 4 else "three_wide",
                    },
                )
            )
        for signature in sorted(self._maneuver_group_signatures - next_signatures):
            events.append(
                RaceEvent(
                    type="maneuver_group_dissolved",
                    driver=self._driver_meta[signature[0]]["abbreviation"],
                    message=f"The {len(signature)}-wide group separates",
                    message_ko=f"{len(signature)}-wide 대열이 해소됩니다",
                    payload={
                        "member_ids": ",".join(str(item) for item in signature),
                        "size": len(signature),
                    },
                )
            )
        self._maneuver_groups = groups
        self._maneuver_group_signatures = next_signatures
        return events

    def _enforce_maneuver_group_corner_capacity(self) -> list[RaceEvent]:
        """Make the lowest-priority cars yield before a narrowing corner."""
        events: list[RaceEvent] = []
        for group in self._maneuver_groups.values():
            if (
                group.size < 3
                or group.corner_turn_direction == 0
                or not group.corner_priority_ids
            ):
                continue
            capacity = self._maneuver_group_capacity_for(set(group.member_ids))
            if capacity >= group.size:
                continue
            yielding_ids = group.corner_priority_ids[max(1, capacity):]
            for yielding_id in yielding_ids:
                changed = False
                for pair_key in group.pair_keys:
                    if yielding_id not in pair_key:
                        continue
                    battle = self._side_by_side_battles.get(pair_key)
                    if battle is None or battle.phase in {"abort", "merge"}:
                        continue
                    self._set_maneuver_phase(battle, "abort")
                    changed = True
                if not changed:
                    continue
                meta = self._driver_meta[yielding_id]
                events.append(
                    RaceEvent(
                        type="maneuver_group_corner_yield",
                        driver=meta["abbreviation"],
                        message=(
                            f"{meta['full_name']} yields as the corner narrows "
                            f"from {group.size}-wide to {capacity}-wide"
                        ),
                        message_ko=(
                            f"{meta['full_name']}가 코너 폭이 좁아져 "
                            f"{group.size}-wide에서 {capacity}-wide로 양보합니다"
                        ),
                        payload={
                            "driver_id": yielding_id,
                            "group_id": group.group_id,
                            "group_size": group.size,
                            "corner_capacity": capacity,
                        },
                    )
                )
        return events

    def _maneuver_group_for_driver(self, driver_id: int) -> ManeuverGroup | None:
        for group in self._maneuver_groups.values():
            if driver_id in group.member_ids:
                return group
        return None

    def _candidate_line_horizon_score(
        self,
        state: DriverRaceState,
        line_name: str,
        opponent: DriverRaceState | None,
    ) -> float:
        """Estimate physical travel time and usable space through corner exit."""
        track_profile = self._track_physics_for_driver(state)
        physics_by_line = self._vehicle_physics_for_driver(state)
        physics = physics_by_line.get(
            line_name,
            physics_by_line[DRIVING_LINE_RACING],
        )
        modifiers = self._physics_v2_modifiers(state)
        # The tactical comparison only needs the time shape across the 280 m
        # braking/exit horizon.  The old 20 m segment loop queried each shared
        # boundary twice (28 predictive speed calls per line).  Eight unique
        # 40 m samples preserve the same trapezoidal travel-time comparison
        # while keeping a 10 Hz, 20-car tactical pass affordable at 2x.
        horizon_offsets_m = tuple(float(value) for value in range(0, 281, 40))
        horizon_distances_m = [
            track_profile.line_distance_at_total_progress(
                line_name,
                state.total_progress + offset_m / self.track_length_m,
            )
            for offset_m in horizon_offsets_m
        ]
        horizon_speeds_mps = [
            physics.target_speed_mps(distance_m, modifiers)
            for distance_m in horizon_distances_m
        ]
        total_time = 0.0
        for start_distance, end_distance, start_speed, end_speed in zip(
            horizon_distances_m,
            horizon_distances_m[1:],
            horizon_speeds_mps,
            horizon_speeds_mps[1:],
        ):
            total_time += max(0.0, end_distance - start_distance) / max(
                1.0,
                0.5 * (start_speed + end_speed),
            )

        score = -total_time
        if opponent is not None:
            opponent_profile = self._track_physics_for_driver(opponent)
            required_clearance_m = PHYSICAL_CAR_WIDTH_M + 0.35
            for lookahead_m in (60.0, 120.0, 180.0):
                lookahead_progress = (
                    state.progress + lookahead_m / self.track_length_m
                )
                candidate_offset = track_profile.line_offset_at_progress(
                    line_name,
                    lookahead_progress,
                )
                opponent_offset = opponent_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    lookahead_progress,
                )
                # Current lateral motion matters during the pull-out but is
                # decayed before the apex; a driver cannot keep changing
                # direction throughout the braking zone.
                decay = max(0.0, 1.0 - lookahead_m / 180.0)
                opponent_offset += opponent.lateral_speed_mps * 0.45 * decay
                clearance = abs(candidate_offset - opponent_offset)
                score -= (
                    max(0.0, required_clearance_m - clearance)
                    * 0.18
                )
                score += min(
                    1.0,
                    clearance / required_clearance_m,
                ) * 0.012
        return score

    def _choose_attack_line(
        self,
        attacker_state: DriverRaceState,
        defender_state: DriverRaceState,
    ) -> str:
        """Choose the quickest viable short-horizon trajectory around a defender."""
        cache_bucket = int(
            self.race_elapsed / ATTACK_LINE_CHOICE_INTERVAL_SECONDS + 1e-9
        )
        if cache_bucket != self._attack_line_choice_cache_bucket:
            self._attack_line_choice_cache_bucket = cache_bucket
            self._attack_line_choice_cache.clear()
        cache_key = (attacker_state.driver_id, defender_state.driver_id)
        cached_line = self._attack_line_choice_cache.get(cache_key)
        if cached_line is not None:
            return cached_line

        scores = {
            ATTACK_LINE_INSIDE: self._candidate_line_horizon_score(
                attacker_state,
                ATTACK_LINE_INSIDE,
                defender_state,
            ),
            ATTACK_LINE_OUTSIDE: self._candidate_line_horizon_score(
                attacker_state,
                ATTACK_LINE_OUTSIDE,
                defender_state,
            ),
        }
        clearance_lookahead_m = min(
            60.0,
            max(25.0, attacker_state.speed_kph / 3.6 * 0.35),
        )
        clearance_progress = (
            attacker_state.progress
            + clearance_lookahead_m / self.track_length_m
        )
        defender_target_m = self._track_physics_for_driver(
            defender_state
        ).line_offset_at_progress(
            DRIVING_LINE_RACING,
            clearance_progress,
        )
        clearances = {
            line_name: abs(
                self._track_physics_for_driver(
                    attacker_state
                ).line_offset_at_progress(
                    line_name,
                    clearance_progress,
                )
                - defender_target_m
            )
            for line_name in (ATTACK_LINE_INSIDE, ATTACK_LINE_OUTSIDE)
        }
        required_clearance_m = (
            PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
        )
        physically_separate = [
            line_name
            for line_name, clearance_m in clearances.items()
            if clearance_m >= required_clearance_m
        ]
        if physically_separate:
            selected_line = max(
                physically_separate,
                key=lambda line_name: (
                    scores[line_name],
                    clearances[line_name],
                ),
            )
        else:
            selected_line = max(scores, key=scores.get)
        self._attack_line_choice_cache[cache_key] = selected_line
        return selected_line

    def _local_pull_out_decision(
        self,
        attacker_state: DriverRaceState,
        defender_state: DriverRaceState,
    ) -> LocalPullOutDecision | None:
        """Choose a physically clear local candidate for a straight pull-out."""
        if not self._overtaking_candidate_allowed(
            attacker_state,
            defender_state,
        ):
            return None
        segment = segment_at_progress(self.circuit, attacker_state.progress)
        if (
            segment is None
            or segment.type != TrackSegmentType.STRAIGHT
            or not self._segment_overtake_start_allowed(segment)
            or not segment.side_by_side_allowed
        ):
            return None
        plan = self._local_trajectory_plans.get(attacker_state.driver_id)
        if (
            plan is None
            or self._local_trajectory_plan_ages.get(
                attacker_state.driver_id,
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS + 1.0,
            )
            > LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS + 0.05
        ):
            return None
        defender_occupancy = next(
            (
                occupancy
                for occupancy in plan.opponent_occupancies
                if occupancy.driver_id == defender_state.driver_id
            ),
            None,
        )
        if defender_occupancy is None:
            return None

        transition_index = min(
            range(len(defender_occupancy.samples)),
            key=lambda index: abs(
                defender_occupancy.samples[index].time_seconds - 1.20
            ),
        )
        required_lateral_separation_m = (
            0.5 * (attacker_state.car_width_m + defender_state.car_width_m)
            + MANEUVER_CLEARANCE_MARGIN_M
        )
        viable_candidates = []
        for candidate in plan.candidates:
            if (
                not candidate.viable
                or abs(candidate.lateral_bias_m) <= 1e-9
                or candidate.minimum_opponent_clearance_m
                < MANEUVER_PULL_OUT_MIN_CLEARANCE_M
            ):
                continue
            lateral_separation_m = abs(
                candidate.lateral_offsets_m[transition_index]
                - defender_occupancy.samples[
                    transition_index
                ].body_pose.lateral_m
            )
            if lateral_separation_m < required_lateral_separation_m:
                continue
            viable_candidates.append(candidate)
        if not viable_candidates:
            return None

        candidate = min(
            viable_candidates,
            key=lambda item: (
                item.objective_cost,
                abs(item.lateral_bias_m),
                item.candidate_id,
            ),
        )
        target_lateral_offset_m = candidate.lateral_offsets_m[transition_index]
        lookahead_progress = (
            attacker_state.progress + 100.0 / self.track_length_m
        )
        track_profile = self._track_physics_for_driver(attacker_state)
        attacker_line = min(
            (ATTACK_LINE_INSIDE, ATTACK_LINE_OUTSIDE),
            key=lambda line_name: abs(
                track_profile.line_offset_at_progress(
                    line_name,
                    lookahead_progress,
                )
                - target_lateral_offset_m
            ),
        )
        return LocalPullOutDecision(
            candidate_id=candidate.candidate_id,
            lateral_bias_m=candidate.lateral_bias_m,
            target_lateral_offset_m=target_lateral_offset_m,
            minimum_clearance_m=candidate.minimum_opponent_clearance_m,
            attacker_line=attacker_line,
        )

    def _choose_defender_line(
        self,
        defender_state: DriverRaceState,
        attacker_state: DriverRaceState,
        attacker_line: str,
    ) -> str:
        """Balance short-horizon pace against blocking the attacker's trajectory."""
        defender = self._driver_meta[defender_state.driver_id]
        racing_score = self._candidate_line_horizon_score(
            defender_state,
            DEFENDER_LINE_RACING,
            attacker_state,
        )
        defensive_score = self._candidate_line_horizon_score(
            defender_state,
            DEFENDER_LINE_DEFENSIVE,
            attacker_state,
        )
        lookahead = defender_state.progress + 100.0 / self.track_length_m
        defender_profile = self._track_physics_for_driver(defender_state)
        attacker_profile = self._track_physics_for_driver(attacker_state)
        defense_offset = defender_profile.line_offset_at_progress(
            DEFENDER_LINE_DEFENSIVE,
            lookahead,
        )
        attack_offset = attacker_profile.line_offset_at_progress(
            attacker_line,
            lookahead,
        )
        block_alignment = 1.0 - min(1.0, abs(defense_offset - attack_offset) / 4.0)
        defensive_score += 0.20 * defender["defending"] * block_alignment
        if attacker_state.drs_active:
            defensive_score += 0.04
        defensive_score += self.rng.uniform(-0.01, 0.01)
        racing_score += self.rng.uniform(-0.01, 0.01)
        return (
            DEFENDER_LINE_DEFENSIVE
            if defensive_score > racing_score
            else DEFENDER_LINE_RACING
        )

    def _side_by_side_battle_for_driver(self, driver_id: int) -> SideBySideBattle | None:
        phase_priority = {
            "overlap": 0,
            "clear": 1,
            "pull_out": 2,
            "approach": 3,
            "merge": 4,
            "yield": 5,
            "abort": 6,
        }
        selected: SideBySideBattle | None = None
        selected_key: tuple[int, float, tuple[int, int]] | None = None
        for battle in self._side_by_side_battles.values():
            if driver_id not in (battle.attacker_id, battle.defender_id):
                continue
            candidate_key = (
                phase_priority.get(battle.phase, 9),
                abs(self._maneuver_longitudinal_advantage_m(battle)),
                self._battle_pair_key(battle.attacker_id, battle.defender_id),
            )
            if selected_key is None or candidate_key < selected_key:
                selected = battle
                selected_key = candidate_key
        return selected

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
        phase: str = "approach",
        trajectory_decision: LocalPullOutDecision | None = None,
    ) -> None:
        key = self._battle_pair_key(attacker_id, defender_id)
        if not self._can_add_maneuver_edge(attacker_id, defender_id):
            return
        battle = SideBySideBattle(
            attacker_id=attacker_id,
            defender_id=defender_id,
            remaining_seconds=MANEUVER_MAX_DURATION_SECONDS,
            attacker_line=attacker_line,
            defender_line=defender_line,
            segment_name=segment_name,
            phase=phase,
            trajectory_candidate_id=(
                trajectory_decision.candidate_id
                if trajectory_decision is not None
                else ""
            ),
            trajectory_lateral_bias_m=(
                trajectory_decision.lateral_bias_m
                if trajectory_decision is not None
                else 0.0
            ),
            trajectory_minimum_clearance_m=(
                trajectory_decision.minimum_clearance_m
                if trajectory_decision is not None
                else 0.0
            ),
            trajectory_authorized=trajectory_decision is not None,
        )
        self._side_by_side_battles[key] = battle
        if phase in {"yield", "abort"}:
            self._capture_maneuver_transition_offsets(battle, phase)

    def _maneuver_neighborhood_available(
        self,
        attacker: DriverRaceState,
        defender: DriverRaceState,
    ) -> bool:
        """Reserve one predictive maneuver per local traffic cluster."""
        pair_progress = (attacker.total_progress, defender.total_progress)
        pair_driver_ids = {attacker.driver_id, defender.driver_id}
        for other in self.driver_states.values():
            if (
                other.driver_id in pair_driver_ids
                or other.in_pit
                or other.retired
                or other.finished
            ):
                continue
            if any(
                abs(other.total_progress - progress) * self.track_length_m
                < 35.0
                for progress in pair_progress
            ):
                return False
        for battle in self._side_by_side_battles.values():
            for driver_id in (battle.attacker_id, battle.defender_id):
                participant = self.driver_states.get(driver_id)
                if participant is None:
                    continue
                if any(
                    abs(participant.total_progress - progress)
                    * self.track_length_m
                    < MANEUVER_RESERVATION_DISTANCE_M
                    for progress in pair_progress
                ):
                    return False
        return True

    def _capture_maneuver_transition_offsets(
        self,
        battle: SideBySideBattle,
        phase: str,
    ) -> None:
        attacker_offset_m = self.driver_states[
            battle.attacker_id
        ].lateral_offset_m
        defender_offset_m = self.driver_states[
            battle.defender_id
        ].lateral_offset_m
        if phase == "yield":
            battle.yield_attacker_lateral_offset_m = attacker_offset_m
            battle.yield_defender_lateral_offset_m = defender_offset_m
        elif phase == "abort":
            battle.abort_attacker_lateral_offset_m = attacker_offset_m
            battle.abort_defender_lateral_offset_m = defender_offset_m

        for driver_id in (battle.attacker_id, battle.defender_id):
            self._local_trajectory_plans.pop(driver_id, None)
            self._local_trajectory_plan_ages[driver_id] = (
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
            )
            self._local_trajectory_selected_ages[driver_id] = 0.0

    def _set_maneuver_phase(self, battle: SideBySideBattle, phase: str) -> None:
        battle.phase = phase
        battle.phase_elapsed_seconds = 0.0
        if phase in {"yield", "abort"}:
            self._capture_maneuver_transition_offsets(battle, phase)
        if phase == "abort":
            battle.abort_rejoin_started = False
            battle.abort_rejoin_elapsed_seconds = 0.0
        if phase in {"clear", "merge", "yield", "abort"}:
            battle.corner_active = False
            battle.corner_exit_hold_seconds = 0.0

    def _begin_maneuver_yield(
        self,
        battle: SideBySideBattle,
        yielding_driver_id: int | None = None,
    ) -> None:
        battle.yielding_driver_id = yielding_driver_id or battle.attacker_id
        self._set_maneuver_phase(battle, "yield")

    def _required_pass_clearance_m(self, battle: SideBySideBattle) -> float:
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        # Battle lines and lateral separation decide whether two cars can
        # coexist without contact.  They must not turn a centre-point order
        # change into a completed pass: the attacking car's complete body must
        # clear the defending car longitudinally first.
        return (
            0.5 * (attacker.car_length_m + defender.car_length_m)
            + MANEUVER_PASS_CLEARANCE_MARGIN_M
        )

    def _confirm_maneuver_pass(self, battle: SideBySideBattle) -> None:
        """Mark a pass complete only after both physical bodies are clear."""
        battle.pass_clearance_m = self._maneuver_longitudinal_advantage_m(battle)
        battle.pass_confirmed = True
        self._set_maneuver_phase(battle, "clear")

    def _maneuver_longitudinal_advantage_m(self, battle: SideBySideBattle) -> float:
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        return (attacker.total_progress - defender.total_progress) * self.track_length_m

    def _maneuver_lateral_separation_m(self, battle: SideBySideBattle) -> float:
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        return abs(attacker.lateral_offset_m - defender.lateral_offset_m)

    def _corner_corridor_for_driver(
        self,
        battle: SideBySideBattle,
        driver_id: int,
    ) -> str:
        inside_driver_id = battle.corner_inside_driver_id
        if inside_driver_id is None:
            attacker_is_inside = battle.attacker_line == ATTACK_LINE_INSIDE
            inside_driver_id = (
                battle.attacker_id if attacker_is_inside else battle.defender_id
            )
        return "inside" if driver_id == inside_driver_id else "outside"

    def _set_corner_turn_direction(
        self,
        battle: SideBySideBattle,
        turn_direction: int,
    ) -> None:
        if turn_direction == 0:
            return
        normalized_direction = 1 if turn_direction > 0 else -1
        if battle.corner_inside_driver_id is None:
            # The pull-out lattice decides which *physical* side each car
            # occupies.  Tactical labels must never make the pair swap sides
            # under braking, so the car already nearest the apex owns the
            # initial inside corridor.
            battle.corner_inside_driver_id = max(
                (battle.attacker_id, battle.defender_id),
                key=lambda driver_id: normalized_direction
                * self.driver_states[driver_id].lateral_offset_m,
            )
        elif (
            battle.corner_turn_direction != 0
            and battle.corner_turn_direction != normalized_direction
        ):
            # At a direction change, the car already nearest the new inside
            # keeps that side. This prevents simultaneous lane crossing.
            battle.corner_inside_driver_id = max(
                (battle.attacker_id, battle.defender_id),
                key=lambda driver_id: normalized_direction
                * self.driver_states[driver_id].lateral_offset_m,
            )
            for driver_id in (battle.attacker_id, battle.defender_id):
                self._local_trajectory_plans.pop(driver_id, None)
                self._local_trajectory_plan_ages[driver_id] = (
                    LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
                )
                self._local_trajectory_selected_ages[driver_id] = 0.0
        battle.corner_turn_direction = normalized_direction

    def _corner_corridor_targets(
        self,
        battle: SideBySideBattle,
        progress: float,
    ) -> dict[int, float] | None:
        """Allocate non-overlapping inside/outside center corridors."""
        track = self._track_physics.at_progress(progress)
        turn_direction = battle.corner_turn_direction or (
            1 if track.turn_signal > 0.0 else -1 if track.turn_signal < 0.0 else 0
        )
        if turn_direction == 0:
            return None

        half_body_progress = (
            PHYSICAL_CAR_LENGTH_M / 2.0 / self.track_length_m
        )
        body_bounds = tuple(
            self._track_surface.trajectory_body_lateral_bounds(
                progress + progress_delta,
                body_width_m=PHYSICAL_CAR_WIDTH_M,
                edge_margin_m=0.0,
            )
            for progress_delta in (
                -half_body_progress,
                0.0,
                half_body_progress,
            )
        )
        minimum_center = max(bounds[0] for bounds in body_bounds)
        maximum_center = min(bounds[1] for bounds in body_bounds)
        required_separation = PHYSICAL_CAR_WIDTH_M + CORNER_CORRIDOR_MARGIN_M
        if maximum_center - minimum_center < required_separation:
            return None

        inside_driver_id = battle.corner_inside_driver_id
        if inside_driver_id is None:
            inside_driver_id = (
                battle.attacker_id
                if battle.attacker_line == ATTACK_LINE_INSIDE
                else battle.defender_id
            )
        outside_driver_id = (
            battle.defender_id
            if inside_driver_id == battle.attacker_id
            else battle.attacker_id
        )
        # This is only a collision-free bootstrap until both local plans
        # are available. It deliberately derives from the vehicle-specific
        # optimized racing paths rather than the legacy inside/outside lines.
        racing_midpoint = 0.5 * (
            self._track_physics_for_driver(inside_driver_id).line_offset_at_progress(
                DRIVING_LINE_RACING,
                progress,
            )
            + self._track_physics_for_driver(outside_driver_id).line_offset_at_progress(
                DRIVING_LINE_RACING,
                progress,
            )
        )
        inside_target = (
            racing_midpoint + turn_direction * required_separation / 2.0
        )
        outside_target = (
            racing_midpoint - turn_direction * required_separation / 2.0
        )

        low = min(inside_target, outside_target)
        high = max(inside_target, outside_target)
        if low < minimum_center:
            shift = minimum_center - low
            inside_target += shift
            outside_target += shift
        high = max(inside_target, outside_target)
        if high > maximum_center:
            shift = high - maximum_center
            inside_target -= shift
            outside_target -= shift

        return {
            inside_driver_id: inside_target,
            outside_driver_id: outside_target,
        }

    def _corner_local_role_target(
        self,
        battle: SideBySideBattle,
        state: DriverRaceState,
    ) -> float:
        """Give opponent prediction a safe seed before its first local plan."""
        track = self._track_physics.at_progress(state.progress)
        turn_direction = (
            1
            if track.turn_signal > 0.0
            else -1 if track.turn_signal < 0.0 else battle.corner_turn_direction
        )
        racing_offset_m = self._track_physics_for_driver(
            state
        ).line_offset_at_progress(DRIVING_LINE_RACING, state.progress)
        role = self._corner_corridor_for_driver(battle, state.driver_id)
        target_m = racing_offset_m
        if role == "outside":
            target_m -= turn_direction * (
                PHYSICAL_CAR_WIDTH_M + CORNER_CORRIDOR_MARGIN_M
            )
        minimum_m, maximum_m = self._track_surface.trajectory_body_lateral_bounds(
            state.progress,
            body_width_m=state.car_width_m,
            edge_margin_m=0.35,
        )
        return max(minimum_m, min(maximum_m, target_m))

    def _corner_braking_margin_m(
        self,
        state: DriverRaceState,
        line_name: str,
    ) -> float:
        track_profile = self._track_physics_for_driver(state)
        physics_by_line = self._vehicle_physics_for_driver(state)
        physics = physics_by_line.get(
            line_name,
            physics_by_line[DRIVING_LINE_RACING],
        )
        lookahead_distances_m = tuple(
            float(distance_m)
            for distance_m in range(
                0,
                int(CORNER_APEX_LOOKAHEAD_M) + 1,
                10,
            )
        )
        apex_distance_m = max(
            lookahead_distances_m,
            key=lambda distance_m: abs(
                self._track_physics.at_progress(
                    state.progress + distance_m / self.track_length_m
                ).turn_signal
            ),
        )
        apex_total_progress = (
            state.total_progress + apex_distance_m / self.track_length_m
        )
        apex_line_distance_m = track_profile.line_distance_at_total_progress(
            line_name,
            apex_total_progress,
        )
        modifiers = self._physics_v2_modifiers(state)
        target_speed_mps = physics.target_speed_mps(
            apex_line_distance_m,
            modifiers,
        )
        speed_mps = max(0.0, state.speed_kph / 3.6)
        current_line_distance_m = track_profile.line_distance_at_total_progress(
            line_name,
            state.total_progress,
        )
        maximum_deceleration_mps2 = maximum_braking_deceleration_mps2(
            physics._curvature_1pm(current_line_distance_m),
            speed_mps,
            modifiers,
        )
        required_braking_distance_m = max(
            0.0,
            (
                speed_mps * speed_mps
                - target_speed_mps * target_speed_mps
            )
            / max(2.0, 2.0 * maximum_deceleration_mps2),
        )
        return (
            apex_distance_m
            + CORNER_BRAKING_MARGIN_M
            - required_braking_distance_m
        )

    def _predict_corner_corridor_poses(
        self,
        battle: SideBySideBattle,
        state: DriverRaceState,
        line_name: str,
    ) -> tuple[tuple[BodyPose, ...], str]:
        track_profile = self._track_physics_for_driver(state)
        physics_by_line = self._vehicle_physics_for_driver(state)
        physics = physics_by_line.get(
            line_name,
            physics_by_line[DRIVING_LINE_RACING],
        )
        modifiers = self._physics_v2_modifiers(state)
        total_progress = state.total_progress
        speed_mps = max(0.0, state.speed_kph / 3.6)
        initial_lateral_m = state.lateral_offset_m
        previous_lateral_m = initial_lateral_m
        previous_longitudinal_m = total_progress * self.track_length_m
        poses: list[BodyPose] = []
        sample_count = round(
            CORNER_PREDICTION_HORIZON_SECONDS
            / CORNER_PREDICTION_STEP_SECONDS
        )
        for index in range(sample_count + 1):
            time_seconds = index * CORNER_PREDICTION_STEP_SECONDS
            if index > 0:
                line_distance_m = track_profile.line_distance_at_total_progress(
                    line_name,
                    total_progress,
                )
                target_speed_mps = physics.target_speed_mps(
                    line_distance_m,
                    modifiers,
                )
                if speed_mps > target_speed_mps:
                    acceleration_mps2 = -min(
                        maximum_braking_deceleration_mps2(
                            physics._curvature_1pm(line_distance_m),
                            speed_mps,
                            modifiers,
                        ),
                        (speed_mps - target_speed_mps)
                        / CORNER_PREDICTION_STEP_SECONDS,
                    )
                else:
                    acceleration_mps2 = min(
                        8.0,
                        (target_speed_mps - speed_mps)
                        / CORNER_PREDICTION_STEP_SECONDS,
                    )
                next_speed_mps = max(
                    0.0,
                    speed_mps
                    + acceleration_mps2 * CORNER_PREDICTION_STEP_SECONDS,
                )
                total_progress += (
                    (speed_mps + next_speed_mps)
                    * 0.5
                    * CORNER_PREDICTION_STEP_SECONDS
                    / self.track_length_m
                )
                speed_mps = next_speed_mps

            corridor_targets = self._corner_corridor_targets(
                battle,
                total_progress,
            )
            if corridor_targets is None:
                return (), "no_corridor"
            transition_ratio = min(
                1.0,
                time_seconds / CORNER_CORRIDOR_TRANSITION_SECONDS,
            )
            smooth_ratio = transition_ratio * transition_ratio * (
                3.0 - 2.0 * transition_ratio
            )
            desired_lateral_m = corridor_targets[state.driver_id]
            lateral_m = initial_lateral_m + (
                desired_lateral_m - initial_lateral_m
            ) * smooth_ratio
            longitudinal_m = total_progress * self.track_length_m
            if index == 0:
                lateral_speed_mps = state.lateral_speed_mps
                heading_rad = state.slip_angle_rad
            else:
                lateral_speed_mps = (
                    lateral_m - previous_lateral_m
                ) / CORNER_PREDICTION_STEP_SECONDS
                lateral_speed_mps = max(
                    -PHYSICS_MAX_LATERAL_SPEED_MPS,
                    min(PHYSICS_MAX_LATERAL_SPEED_MPS, lateral_speed_mps),
                )
                heading_rad = max(
                    -0.35,
                    min(
                        0.35,
                        atan2(
                            lateral_m - previous_lateral_m,
                            max(
                                1e-6,
                                longitudinal_m - previous_longitudinal_m,
                            ),
                        ),
                    ),
                )

            half_length_progress = (
                state.car_length_m / 2.0 / self.track_length_m
            )
            rear_lateral_m = (
                lateral_m - state.car_length_m / 2.0 * heading_rad
            )
            front_lateral_m = (
                lateral_m + state.car_length_m / 2.0 * heading_rad
            )
            rear_bounds = self._track_surface.trajectory_body_lateral_bounds(
                total_progress - half_length_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=0.0,
            )
            front_bounds = self._track_surface.trajectory_body_lateral_bounds(
                total_progress + half_length_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=0.0,
            )
            if (
                rear_lateral_m < rear_bounds[0]
                or rear_lateral_m > rear_bounds[1]
                or front_lateral_m < front_bounds[0]
                or front_lateral_m > front_bounds[1]
            ):
                return (), "track_boundary"
            poses.append(
                BodyPose(
                    longitudinal_m=longitudinal_m,
                    lateral_m=lateral_m,
                    heading_rad=heading_rad,
                    length_m=state.car_length_m,
                    width_m=state.car_width_m,
                    longitudinal_speed_mps=speed_mps,
                    lateral_speed_mps=lateral_speed_mps,
                )
            )
            previous_lateral_m = lateral_m
            previous_longitudinal_m = longitudinal_m
        return tuple(poses), ""

    def _corner_corridor_prediction(
        self,
        battle: SideBySideBattle,
    ) -> CornerCorridorPrediction:
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        defender_line = (
            ATTACK_LINE_OUTSIDE
            if battle.attacker_line == ATTACK_LINE_INSIDE
            else ATTACK_LINE_INSIDE
        )
        attacker_braking_margin_m = self._corner_braking_margin_m(
            attacker,
            battle.attacker_line,
        )
        defender_braking_margin_m = self._corner_braking_margin_m(
            defender,
            defender_line,
        )
        braking_margin_m = min(
            attacker_braking_margin_m,
            defender_braking_margin_m,
        )
        if braking_margin_m < 0.0:
            return CornerCorridorPrediction(
                authorized=False,
                rejection_reason="insufficient_braking",
                horizon_seconds=CORNER_PREDICTION_HORIZON_SECONDS,
                minimum_clearance_m=0.0,
                braking_margin_m=braking_margin_m,
            )

        attacker_poses, attacker_rejection = (
            self._predict_corner_corridor_poses(
                battle,
                attacker,
                battle.attacker_line,
            )
        )
        if attacker_rejection:
            return CornerCorridorPrediction(
                False,
                attacker_rejection,
                CORNER_PREDICTION_HORIZON_SECONDS,
                0.0,
                braking_margin_m,
            )
        defender_poses, defender_rejection = (
            self._predict_corner_corridor_poses(
                battle,
                defender,
                defender_line,
            )
        )
        if defender_rejection:
            return CornerCorridorPrediction(
                False,
                defender_rejection,
                CORNER_PREDICTION_HORIZON_SECONDS,
                0.0,
                braking_margin_m,
            )

        minimum_clearance_m = float("inf")
        for attacker_pose, defender_pose in zip(
            attacker_poses,
            defender_poses,
        ):
            minimum_clearance_m = min(
                minimum_clearance_m,
                oriented_body_separation_m(attacker_pose, defender_pose),
            )
            if oriented_body_overlap(attacker_pose, defender_pose) is not None:
                return CornerCorridorPrediction(
                    False,
                    "predicted_collision",
                    CORNER_PREDICTION_HORIZON_SECONDS,
                    0.0,
                    braking_margin_m,
                )
        return CornerCorridorPrediction(
            True,
            "",
            CORNER_PREDICTION_HORIZON_SECONDS,
            minimum_clearance_m,
            braking_margin_m,
        )

    def _try_commit_corner(self, battle: SideBySideBattle) -> bool:
        """Grant corner occupancy once the attacker's front axle is alongside."""
        if battle.corner_active or not battle.corner_entry_candidate:
            return battle.corner_active
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        configured_segment = segment_at_progress(
            self.circuit,
            attacker.progress,
        )
        if (
            configured_segment is not None
            and configured_segment.type == TrackSegmentType.STRAIGHT
        ):
            return False
        track = self._track_physics.at_progress(attacker.progress)
        if abs(track.turn_signal) < CORNER_ENTRY_TURN_SIGNAL:
            return False

        advantage_m = self._maneuver_longitudinal_advantage_m(battle)
        braking_reached = (
            attacker.brake > 0.02
            or defender.brake > 0.02
            or self._segment_type_at(attacker)
            in {
                TrackSegmentType.HEAVY_BRAKING.value,
                TrackSegmentType.TECHNICAL.value,
                TrackSegmentType.TRACTION.value,
                TrackSegmentType.SWEEPING.value,
            }
        )
        targets = self._corner_corridor_targets(battle, attacker.progress)
        geometric_authorized = (
            advantage_m >= -MANEUVER_OVERLAP_GAP_M
            and braking_reached
            and targets is not None
        )
        battle.corner_prediction_horizon_seconds = (
            CORNER_PREDICTION_HORIZON_SECONDS
        )
        battle.corner_prediction_minimum_clearance_m = 0.0
        battle.corner_prediction_braking_margin_m = 0.0
        battle.corner_prediction_rejection_reason = ""
        authorized = False
        if geometric_authorized:
            self._set_corner_turn_direction(
                battle,
                1 if track.turn_signal > 0.0 else -1,
            )
            prediction = self._corner_corridor_prediction(battle)
            authorized = prediction.authorized
            battle.corner_prediction_horizon_seconds = (
                prediction.horizon_seconds
            )
            battle.corner_prediction_minimum_clearance_m = (
                prediction.minimum_clearance_m
            )
            battle.corner_prediction_braking_margin_m = (
                prediction.braking_margin_m
            )
            battle.corner_prediction_rejection_reason = (
                prediction.rejection_reason
            )
        elif targets is None:
            battle.corner_prediction_rejection_reason = "no_corridor"
        elif advantage_m < -MANEUVER_OVERLAP_GAP_M:
            battle.corner_prediction_rejection_reason = "not_alongside"
        else:
            battle.corner_prediction_rejection_reason = "braking_not_reached"
        battle.corner_authorized = authorized
        battle.corner_entry_advantage_m = advantage_m
        battle.corner_entry_brake_delta = attacker.brake - defender.brake
        if not authorized:
            return False

        battle.corner_active = True
        battle.corner_exit_hold_seconds = 0.0
        battle.corner_count += 1
        for driver_id in (battle.attacker_id, battle.defender_id):
            self._local_trajectory_plans.pop(driver_id, None)
            self._local_trajectory_plan_ages[driver_id] = (
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
            )
            self._local_trajectory_selected_ages[driver_id] = 0.0
        return True

    def _corner_maneuver_factors(
        self,
        state: DriverRaceState,
    ) -> tuple[float, float]:
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if battle is None or not battle.corner_active or battle.phase != "overlap":
            return 1.0, 1.0
        if self._corner_corridor_for_driver(battle, state.driver_id) == "inside":
            return CORNER_INSIDE_BRAKING_FACTOR, CORNER_INSIDE_TRACTION_FACTOR
        return 1.0, 1.0

    def _maneuver_racing_line_error_m(self, state: DriverRaceState) -> float:
        racing_offset = self._track_physics_for_driver(
            state
        ).line_offset_at_progress(
            DRIVING_LINE_RACING,
            state.progress,
        )
        return abs(state.lateral_offset_m - racing_offset)

    def _distance_to_next_corner_entry_m(
        self,
        state: DriverRaceState,
        maximum_distance_m: float,
    ) -> float | None:
        maximum = max(0.0, maximum_distance_m)
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is not None and segment.type == TrackSegmentType.STRAIGHT:
            # Small centreline-curvature noise can exist inside a configured
            # straight (notably on climbs). It must not be mistaken for the
            # next braking corner and cancel an otherwise safe pull-out.
            remaining_segment_m = self._remaining_segment_distance_m(
                state,
                segment,
            )
            return (
                remaining_segment_m
                if remaining_segment_m <= maximum
                else None
            )
        for distance_m in range(0, int(maximum) + 5, 5):
            sample = self._track_physics.at_progress(
                state.progress + distance_m / self.track_length_m
            )
            if abs(sample.turn_signal) >= CORNER_ENTRY_TURN_SIGNAL:
                return float(distance_m)
        return None

    def _next_corner_turn_direction(
        self,
        state: DriverRaceState,
        maximum_distance_m: float,
    ) -> int:
        """Return the first material turn direction inside a braking preview."""
        for distance_m in range(0, int(max(0.0, maximum_distance_m)) + 5, 5):
            sample = self._track_physics.at_progress(
                state.progress + distance_m / self.track_length_m
            )
            if abs(sample.turn_signal) < CORNER_ENTRY_TURN_SIGNAL:
                continue
            return 1 if sample.turn_signal > 0.0 else -1
        return 0

    def _commit_maneuver_lines_before_braking(
        self,
        battle: SideBySideBattle,
    ) -> None:
        """Freeze each driver's chosen corridor before corner deceleration.

        FIA's single defensive move and moving-under-braking restrictions are
        represented as one continuous lateral choice.  The stored values are
        biases from each vehicle-specific racing line, so the cars still
        follow the track curvature rather than holding an absolute canvas
        coordinate.
        """
        if (
            battle.line_committed
            or battle.phase not in {"pull_out", "overlap"}
            or battle.corner_active
        ):
            return
        attacker = self.driver_states[battle.attacker_id]
        defender = self.driver_states[battle.defender_id]
        if (
            self._maneuver_lateral_separation_m(battle)
            < PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
        ):
            return
        preview_m = max(
            MANEUVER_LINE_COMMIT_MIN_PREVIEW_M,
            max(attacker.speed_kph, defender.speed_kph)
            / 3.6
            * MANEUVER_LINE_COMMIT_PREVIEW_SECONDS,
        )
        corner_distance_m = self._distance_to_next_corner_entry_m(
            attacker,
            preview_m,
        )
        if corner_distance_m is None:
            return

        battle.line_committed = True
        battle.line_commit_progress = attacker.total_progress
        battle.line_commit_distance_m = corner_distance_m
        attacker_racing_m = self._track_physics_for_driver(
            attacker
        ).line_offset_at_progress(DRIVING_LINE_RACING, attacker.progress)
        defender_racing_m = self._track_physics_for_driver(
            defender
        ).line_offset_at_progress(DRIVING_LINE_RACING, defender.progress)
        battle.attacker_committed_lateral_bias_m = (
            attacker.lateral_offset_m - attacker_racing_m
        )
        battle.defender_committed_lateral_bias_m = (
            defender.lateral_offset_m - defender_racing_m
        )
        turn_direction = self._next_corner_turn_direction(attacker, preview_m)
        if turn_direction:
            self._set_corner_turn_direction(battle, turn_direction)
        for driver_id in (battle.attacker_id, battle.defender_id):
            self._local_trajectory_plans.pop(driver_id, None)
            self._local_trajectory_plan_ages[driver_id] = (
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
            )
            self._local_trajectory_selected_ages[driver_id] = 0.0

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
        return abs(self._maneuver_longitudinal_advantage_m(battle)) <= MANEUVER_BREAK_GAP_M

    def _prune_side_by_side_battles(self) -> list[RaceEvent]:
        for key, battle in list(self._side_by_side_battles.items()):
            if not self._side_by_side_battle_is_valid(battle):
                del self._side_by_side_battles[key]
        return []

    def _maneuver_overlap_event(self, battle: SideBySideBattle) -> RaceEvent:
        attacker = self._driver_meta[battle.attacker_id]
        defender = self._driver_meta[battle.defender_id]
        return RaceEvent(
            type="side_by_side",
            driver=attacker["abbreviation"],
            message=self._side_by_side_message(
                attacker["full_name"],
                defender["full_name"],
                battle.segment_name,
                battle.attacker_line,
                battle.defender_line,
            ),
            message_ko=self._side_by_side_message_ko(
                attacker["full_name"],
                defender["full_name"],
                battle.segment_name,
                battle.attacker_line,
                battle.defender_line,
            ),
        )

    def _maneuver_pass_event(self, battle: SideBySideBattle) -> RaceEvent:
        attacker_state = self.driver_states[battle.attacker_id]
        attacker = self._driver_meta[battle.attacker_id]
        defender = self._driver_meta[battle.defender_id]
        required_clearance_m = self._required_pass_clearance_m(battle)
        return RaceEvent(
            type="pass",
            driver=attacker["abbreviation"],
            message=(
                f"{attacker['full_name']} passes {defender['full_name']}"
                f" for P{attacker_state.position}"
            ),
            message_ko=(
                f"{attacker['full_name']}가 {defender['full_name']}를 추월하며"
                f" P{attacker_state.position}로 올라섭니다"
            ),
            payload={
                "attacker_id": attacker_state.driver_id,
                "defender_id": battle.defender_id,
                "clearance_m": round(battle.pass_clearance_m, 6),
                "required_clearance_m": round(required_clearance_m, 6),
            },
        )

    def _maneuver_abort_event(self, battle: SideBySideBattle) -> RaceEvent:
        attacker = self._driver_meta[battle.attacker_id]
        defender = self._driver_meta[battle.defender_id]
        return RaceEvent(
            type="overtake_abort",
            driver=attacker["abbreviation"],
            message=(
                f"{attacker['full_name']} abandons the move on"
                f" {defender['full_name']} and slots back in"
            ),
            message_ko=(
                f"{attacker['full_name']}가 {defender['full_name']} 추월 시도를 접고"
                " 다시 레이싱 라인으로 복귀합니다"
            ),
        )

    def _advance_maneuver(
        self,
        battle: SideBySideBattle,
        delta: float,
    ) -> tuple[list[RaceEvent], bool]:
        """Advance one maneuver from physical separation and body clearance."""
        events: list[RaceEvent] = []
        phase_at_start = battle.phase
        battle.elapsed_seconds += delta
        battle.phase_elapsed_seconds += delta
        if phase_at_start not in {"approach", "clear", "merge", "yield", "abort"}:
            battle.committed_seconds += delta
        battle.remaining_seconds = max(
            0.0,
            MANEUVER_MAX_DURATION_SECONDS - battle.committed_seconds,
        )
        advantage_m = self._maneuver_longitudinal_advantage_m(battle)
        lateral_separation_m = self._maneuver_lateral_separation_m(battle)

        attacker = self.driver_states[battle.attacker_id]
        active_segment = segment_at_progress(self.circuit, attacker.progress)
        self._commit_maneuver_lines_before_braking(battle)
        if (
            battle.phase in {"pull_out", "overlap"}
            and active_segment is not None
            and not active_segment.side_by_side_allowed
        ):
            self._set_maneuver_phase(battle, "abort")
            events.append(self._maneuver_abort_event(battle))
            return events, False

        if battle.phase not in {"clear", "merge", "yield", "abort"}:
            if (
                battle.phase == "approach"
                and battle.elapsed_seconds >= MANEUVER_APPROACH_MAX_SECONDS
            ):
                self._set_maneuver_phase(battle, "abort")
                events.append(self._maneuver_abort_event(battle))
            elif (
                battle.committed_seconds >= MANEUVER_MAX_DURATION_SECONDS
                and not battle.corner_active
            ):
                self._set_maneuver_phase(battle, "abort")
                events.append(self._maneuver_abort_event(battle))
            elif advantage_m < -MANEUVER_BREAK_GAP_M:
                self._set_maneuver_phase(battle, "abort")
                events.append(self._maneuver_abort_event(battle))

        if battle.phase == "approach":
            attacker = self.driver_states[battle.attacker_id]
            defender = self.driver_states[battle.defender_id]
            bumper_gap_m = self._bumper_gap_m(attacker, defender)
            planned_corridor_ready = (
                lateral_separation_m
                >= PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
                and bumper_gap_m <= MANEUVER_ATTACK_DECISION_BUMPER_GAP_M
            )
            if (
                (
                    battle.trajectory_authorized
                    and bumper_gap_m <= MANEUVER_PULL_OUT_BUMPER_GAP_M
                )
                or planned_corridor_ready
            ):
                self._set_maneuver_phase(battle, "pull_out")

        elif battle.phase == "pull_out":
            has_lateral_clearance = (
                lateral_separation_m
                >= PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
            )
            has_longitudinal_overlap = abs(advantage_m) <= MANEUVER_OVERLAP_GAP_M
            if has_lateral_clearance and has_longitudinal_overlap:
                self._set_maneuver_phase(battle, "overlap")
                battle.corner_entry_candidate = True
                events.append(self._maneuver_overlap_event(battle))
            else:
                corner_preview_m = max(
                    MANEUVER_PULL_OUT_MIN_CORNER_PREVIEW_M,
                    max(0.0, attacker.speed_kph / 3.6)
                    * MANEUVER_PULL_OUT_CORNER_PREVIEW_SECONDS,
                )
                if self._distance_to_next_corner_entry_m(
                    attacker,
                    corner_preview_m,
                ) is not None:
                    can_continue_into_braking = (
                        has_lateral_clearance
                        and advantage_m
                        >= -(
                            MANEUVER_OVERLAP_GAP_M
                            + MANEUVER_CORNER_LUNGE_EXTENSION_M
                        )
                    )
                    if not can_continue_into_braking:
                        self._set_maneuver_phase(battle, "abort")
                        events.append(self._maneuver_abort_event(battle))
                        return events, False

        elif battle.phase == "overlap":
            attacker = self.driver_states[battle.attacker_id]
            track = self._track_physics.at_progress(attacker.progress)
            entering_corner = (
                active_segment is not None
                and active_segment.type != TrackSegmentType.STRAIGHT
                and abs(track.turn_signal) >= CORNER_ENTRY_TURN_SIGNAL
            )
            if entering_corner and battle.corner_entry_candidate and not battle.corner_active:
                if not self._try_commit_corner(battle):
                    self._begin_maneuver_yield(battle)
                    events.append(self._maneuver_abort_event(battle))
                    return events, False

            if battle.corner_active:
                decisive_gap_m = max(
                    2.0 * PHYSICAL_CAR_LENGTH_M,
                    self._required_pass_clearance_m(battle),
                )
                if advantage_m >= decisive_gap_m:
                    battle.corner_active = False
                    self._confirm_maneuver_pass(battle)
                    return events, False
                if advantage_m <= -decisive_gap_m:
                    battle.corner_active = False
                    self._set_maneuver_phase(battle, "abort")
                    events.append(self._maneuver_abort_event(battle))
                    return events, False
                if abs(track.turn_signal) >= CORNER_EXIT_TURN_SIGNAL:
                    self._set_corner_turn_direction(
                        battle,
                        1 if track.turn_signal > 0.0 else -1,
                    )
                    battle.corner_exit_hold_seconds = 0.0
                else:
                    lookahead = [
                        self._track_physics.at_progress(
                            attacker.progress + distance_m / self.track_length_m
                        )
                        for distance_m in (20.0, 40.0, 70.0)
                    ]
                    next_corner = max(
                        lookahead,
                        key=lambda sample: abs(sample.turn_signal),
                    )
                    if abs(next_corner.turn_signal) >= CORNER_ENTRY_TURN_SIGNAL:
                        self._set_corner_turn_direction(
                            battle,
                            1 if next_corner.turn_signal > 0.0 else -1,
                        )
                        battle.corner_exit_hold_seconds = 0.0
                    else:
                        battle.corner_exit_hold_seconds += delta

                if battle.corner_exit_hold_seconds >= CORNER_EXIT_HOLD_SECONDS:
                    battle.corner_active = False
                    battle.corner_entry_candidate = False
                    if advantage_m >= self._required_pass_clearance_m(battle):
                        self._confirm_maneuver_pass(battle)
                    elif advantage_m <= -MANEUVER_CLEAR_GAP_M:
                        self._set_maneuver_phase(battle, "abort")
                        events.append(self._maneuver_abort_event(battle))
                # A nose advantage inside the corner is not a completed pass.
                return events, False

            if advantage_m >= self._required_pass_clearance_m(battle):
                self._confirm_maneuver_pass(battle)
            elif advantage_m < -MANEUVER_BREAK_GAP_M:
                self._set_maneuver_phase(battle, "abort")
                events.append(self._maneuver_abort_event(battle))

        elif battle.phase == "clear":
            if battle.phase_elapsed_seconds >= MANEUVER_CLEAR_HOLD_SECONDS:
                self._set_maneuver_phase(battle, "merge")

        elif battle.phase == "yield":
            yielding_id = battle.yielding_driver_id or battle.attacker_id
            other_id = (
                battle.defender_id
                if yielding_id == battle.attacker_id
                else battle.attacker_id
            )
            yielding = self.driver_states[yielding_id]
            other = self.driver_states[other_id]
            rear_clearance_m = (
                other.total_progress - yielding.total_progress
            ) * self.track_length_m
            half_lengths_m = 0.5 * (
                yielding.car_length_m + other.car_length_m
            )
            desired_clearance_m = half_lengths_m + MANEUVER_PREP_BUMPER_GAP_M
            minimum_clearance_m = half_lengths_m + MANEUVER_CONTACT_BUFFER_M
            normally_clear = (
                battle.phase_elapsed_seconds >= MANEUVER_YIELD_MIN_SECONDS
                and rear_clearance_m >= desired_clearance_m
            )
            timed_safe_clear = (
                battle.phase_elapsed_seconds >= MANEUVER_YIELD_MAX_SECONDS
                and rear_clearance_m >= minimum_clearance_m
            )
            if normally_clear or timed_safe_clear:
                self._set_maneuver_phase(battle, "abort")

        elif battle.phase == "merge":
            attacker = self.driver_states[battle.attacker_id]
            defender = self.driver_states[battle.defender_id]
            rejoined = (
                self._maneuver_racing_line_error_m(attacker) <= MANEUVER_REJOIN_TOLERANCE_M
                and self._maneuver_racing_line_error_m(defender) <= MANEUVER_REJOIN_TOLERANCE_M
            )
            if rejoined and battle.phase_elapsed_seconds >= MANEUVER_CLEAR_HOLD_SECONDS:
                return events, True

        elif battle.phase == "abort":
            yielding_id = battle.yielding_driver_id or battle.attacker_id
            other_id = (
                battle.defender_id
                if yielding_id == battle.attacker_id
                else battle.attacker_id
            )
            yielding = self.driver_states[yielding_id]
            other = self.driver_states[other_id]
            required_rear_clearance_m = (
                0.5 * (yielding.car_length_m + other.car_length_m)
                + MANEUVER_PREP_BUMPER_GAP_M
            )
            rear_clearance_m = (
                other.total_progress - yielding.total_progress
            ) * self.track_length_m
            if (
                not battle.abort_rejoin_started
                and rear_clearance_m >= required_rear_clearance_m
            ):
                battle.abort_rejoin_started = True
            if battle.abort_rejoin_started:
                battle.abort_rejoin_elapsed_seconds += delta

            attacker = self.driver_states[battle.attacker_id]
            defender = self.driver_states[battle.defender_id]
            rejoined = (
                self._maneuver_racing_line_error_m(attacker)
                <= MANEUVER_REJOIN_TOLERANCE_M
                and self._maneuver_racing_line_error_m(defender)
                <= MANEUVER_REJOIN_TOLERANCE_M
            )
            if (
                rejoined
                and battle.phase_elapsed_seconds >= MANEUVER_ABORT_HOLD_SECONDS
                and battle.abort_rejoin_elapsed_seconds
                >= MANEUVER_ABORT_REJOIN_SECONDS
            ):
                return events, True

        return events, False

    def _tick_side_by_side_battles(self, delta: float) -> list[RaceEvent]:
        if self.race_phase != "green":
            self._cancel_maneuvers_for_neutralization()
            return []
        events: list[RaceEvent] = []
        for key, battle in list(self._side_by_side_battles.items()):
            if not self._side_by_side_battle_is_valid(battle):
                del self._side_by_side_battles[key]
                continue
            attacker = self.driver_states[battle.attacker_id]
            defender = self.driver_states[battle.defender_id]
            if (
                not battle.pass_confirmed
                and battle.phase not in {"yield", "abort"}
                and not self._overtaking_candidate_allowed(attacker, defender)
            ):
                self._set_maneuver_phase(battle, "abort")
                events.append(self._maneuver_abort_event(battle))
            battle_events, complete = self._advance_maneuver(battle, delta)
            events.extend(battle_events)
            if complete:
                del self._side_by_side_battles[key]
        # Ranking and the feed consume the same confirmed physical state.
        self._update_positions()
        events.extend(self._build_confirmed_maneuver_pass_events())
        events.extend(self._rebuild_maneuver_groups())
        corner_yield_events = self._enforce_maneuver_group_corner_capacity()
        if corner_yield_events:
            events.extend(corner_yield_events)
            events.extend(self._rebuild_maneuver_groups())
        return events

    def _build_confirmed_maneuver_pass_events(self) -> list[RaceEvent]:
        events: list[RaceEvent] = []
        for battle in self._side_by_side_battles.values():
            if not battle.pass_confirmed or battle.pass_event_emitted:
                continue
            attacker = self.driver_states[battle.attacker_id]
            defender = self.driver_states[battle.defender_id]
            clearance_m = self._maneuver_longitudinal_advantage_m(battle)
            if (
                clearance_m + PROGRESS_EPSILON
                < self._required_pass_clearance_m(battle)
                or attacker.position >= defender.position
            ):
                continue
            battle.pass_clearance_m = clearance_m
            battle.pass_event_emitted = True
            events.append(self._maneuver_pass_event(battle))
        return events

    def _side_by_side_lap_time_delta(self, state: DriverRaceState) -> float:
        # Line geometry and the vehicle force model now create the pace cost.
        return 0.0

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
            if roll < 0.65:
                return "run_wide"
            return "traction_loss"
        if battle.attacker_line == ATTACK_LINE_OUTSIDE:
            if roll < 0.65:
                return "traction_loss"
            return "run_wide"
        if roll < 0.65:
            return "run_wide"
        return "traction_loss"

    def _maybe_side_by_side_result_event(self, battle: SideBySideBattle) -> RaceEvent | None:
        if self.rng.random() >= self._side_by_side_result_probability(battle):
            return None

        result_type = self._side_by_side_result_type(battle)
        attacker = self._driver_meta[battle.attacker_id]
        if result_type == "run_wide":
            self._set_battle_effect(
                battle.attacker_id,
                BATTLE_RUN_WIDE_LAP_TIME_DELTA,
                BATTLE_RUN_WIDE_EFFECT_SECONDS,
                BATTLE_RUN_WIDE_TIRE_USAGE_MULTIPLIER,
            )
            return RaceEvent(
                type="run_wide",
                driver=attacker["abbreviation"],
                message=f"{attacker['full_name']} runs wide exiting {battle.segment_name}",
                message_ko=f"{attacker['full_name']}가 {battle.segment_name} 탈출에서 바깥으로 밀려납니다",
            )
        self._start_driver_input_error(
            battle.attacker_id,
            kind="throttle",
            duration_seconds=THROTTLE_INPUT_ERROR_SECONDS,
            intensity=THROTTLE_INPUT_ERROR_INTENSITY,
            trigger="side_by_side_exit",
            segment_name=battle.segment_name,
            opponent_id=battle.defender_id,
        )
        return None

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
        key = self._battle_pair_key(attacker_id, defender_id)
        if key in self._forced_wide_aftermaths:
            return
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
        return "attacker_traction_loss"

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
        if result_type == "attacker_traction_loss":
            self._start_driver_input_error(
                aftermath.attacker_id,
                kind="throttle",
                duration_seconds=THROTTLE_INPUT_ERROR_SECONDS,
                intensity=THROTTLE_INPUT_ERROR_INTENSITY,
                trigger="tight_exit_after_forcing_wide",
                segment_name=aftermath.segment_name,
                opponent_id=aftermath.defender_id,
            )
            return None

        if result_type == "run_wide":
            self._set_battle_effect(
                aftermath.defender_id,
                BATTLE_RUN_WIDE_LAP_TIME_DELTA,
                BATTLE_RUN_WIDE_EFFECT_SECONDS,
                BATTLE_RUN_WIDE_TIRE_USAGE_MULTIPLIER,
            )
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
        self._start_driver_input_error(
            aftermath.defender_id,
            kind="throttle",
            duration_seconds=THROTTLE_INPUT_ERROR_SECONDS,
            intensity=THROTTLE_INPUT_ERROR_INTENSITY,
            trigger="forced_wide_exit",
            segment_name=aftermath.segment_name,
            opponent_id=aftermath.attacker_id,
        )
        return None

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

    def _commit_battle_intent(self, intent: BattleIntent) -> RaceEvent | None:
        """Apply one command decision and publish its RULES-stage event."""
        self._require_rules_phase("commit battle intent")
        state = self.driver_states.get(intent.attacker_id)
        car_ahead = self.driver_states.get(intent.defender_id)
        if (
            state is None
            or car_ahead is None
            or not self._overtaking_candidate_allowed(state, car_ahead)
            or not self._can_add_maneuver_edge(state.driver_id, car_ahead.driver_id)
            or (
                intent.event_kind not in {"grid_attack", "grid_defend", "lockup"}
                and not self._maneuver_neighborhood_available(state, car_ahead)
            )
        ):
            return None
        self._battle_event_cooldown[state.driver_id] = BATTLE_EVENT_COOLDOWN_SECONDS
        self._battle_event_cooldown[car_ahead.driver_id] = BATTLE_EVENT_COOLDOWN_SECONDS
        attacker = self._driver_meta[state.driver_id]
        defender = self._driver_meta[car_ahead.driver_id]

        if intent.event_kind == "lockup":
            self._start_driver_input_error(
                state.driver_id,
                kind="brake",
                duration_seconds=BRAKE_INPUT_ERROR_SECONDS,
                intensity=BRAKE_INPUT_ERROR_INTENSITY,
                trigger="battle_braking_error",
                segment_name=intent.segment_name,
                opponent_id=car_ahead.driver_id,
            )
            return None

        if intent.event_kind == "grid_attack":
            return RaceEvent(
                type="attack",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} gets a run on"
                    f" {defender['full_name']} at the start"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 스타트에서"
                    f" {defender['full_name']}에게 추월을 시도합니다"
                ),
            )
        if intent.event_kind == "grid_defend":
            return RaceEvent(
                type="defend",
                driver=defender["abbreviation"],
                message=(
                    f"{defender['full_name']} holds position from"
                    f" {attacker['full_name']} at the start"
                ),
                message_ko=(
                    f"{defender['full_name']}가 스타트에서"
                    f" {attacker['full_name']}의 공격을 막아냅니다"
                ),
            )

        self._start_side_by_side_battle(
            state.driver_id,
            car_ahead.driver_id,
            intent.attacker_line,
            intent.defender_line,
            intent.segment_name,
            trajectory_decision=intent.trajectory_decision,
        )
        if intent.event_kind == "heavy_balanced":
            return RaceEvent(
                type="attack",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} commits to a move on"
                    f" {defender['full_name']} into {intent.segment_name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 {intent.segment_name} 진입에서"
                    f" {defender['full_name']} 추월을 시도합니다"
                ),
            )
        if intent.event_kind == "heavy_forced":
            return RaceEvent(
                type="attack",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} launches a committed attack on"
                    f" {defender['full_name']} at {intent.segment_name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 {intent.segment_name}에서"
                    f" {defender['full_name']}에게 적극적인 추월을 시도합니다"
                ),
            )
        if intent.event_kind == "attack":
            drs_text = " with DRS" if state.drs_active else ""
            return RaceEvent(
                type="attack",
                driver=attacker["abbreviation"],
                message=(
                    f"{attacker['full_name']} attacks {defender['full_name']}"
                    f"{drs_text} into {intent.segment_name}"
                ),
                message_ko=(
                    f"{attacker['full_name']}가 {defender['full_name']}를"
                    f" {intent.segment_name} 진입에서 공격합니다"
                    + (" (DRS 사용)" if state.drs_active else "")
                ),
            )
        return RaceEvent(
            type="defend",
            driver=defender["abbreviation"],
            message=(
                f"{defender['full_name']} defends from {attacker['full_name']}"
                f" through {intent.segment_name}"
            ),
            message_ko=(
                f"{defender['full_name']}가 {intent.segment_name}에서"
                f" {attacker['full_name']}의 공격을 막아냅니다"
            ),
        )

    def _maybe_battle_event(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        *,
        commit: bool = True,
        delta_seconds: float = PHYSICS_STEP_SECONDS,
    ) -> RaceEvent | BattleIntent | None:
        """Evaluate one battle intent, optionally committing it immediately."""
        if (
            car_ahead is None
            or not self._overtaking_candidate_allowed(state, car_ahead)
            or self._is_side_by_side_active(state.driver_id)
            or self._is_side_by_side_active(car_ahead.driver_id)
            or self._battle_event_cooldown.get(state.driver_id, 0.0) > 0
            or self._battle_event_cooldown.get(car_ahead.driver_id, 0.0) > 0
        ):
            return None
        progress_gap = car_ahead.total_progress - state.total_progress
        gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
        probability = self._probability_for_elapsed_time(
            self._battle_event_probability(state, car_ahead, gap_seconds),
            delta_seconds,
        )
        if probability <= 0.0 or not self._overtake_opportunity_is_viable(state, car_ahead):
            return None
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is None:
            return None
        trajectory_decision = None
        pending_overtake = None
        if segment.type == TrackSegmentType.STRAIGHT:
            if self._grid_launch_pair_is_physically_separate(state, car_ahead):
                lookahead = state.progress + 100.0 / self.track_length_m
                track_profile = self._track_physics_for_driver(state)
                attacker_line = min(
                    (ATTACK_LINE_INSIDE, ATTACK_LINE_OUTSIDE),
                    key=lambda line_name: abs(
                        track_profile.line_offset_at_progress(line_name, lookahead)
                        - state.lateral_offset_m
                    ),
                )
                trajectory_decision = LocalPullOutDecision(
                    candidate_id="grid_launch_lane",
                    lateral_bias_m=state.lateral_offset_m,
                    target_lateral_offset_m=state.lateral_offset_m,
                    minimum_clearance_m=(
                        abs(state.lateral_offset_m - car_ahead.lateral_offset_m)
                        - 0.5 * (state.car_width_m + car_ahead.car_width_m)
                    ),
                    attacker_line=attacker_line,
                )
            else:
                pending_overtake = self._pending_overtake_commands.get(state.driver_id)
                trajectory_decision = (
                    pending_overtake.decision
                    if pending_overtake is not None
                    and pending_overtake.defender_id == car_ahead.driver_id
                    else self._local_pull_out_decision(state, car_ahead)
                )
        else:
            # A new maneuver must be committed and validated on the preceding
            # straight.  Once braking has begun, existing side-by-side battles
            # may continue but no fresh tactical lane change is created. A
            # pressure-induced braking mistake may still occur without
            # granting an unvalidated side-by-side corridor.
            if self.rng.random() >= probability:
                return None
            mistake_probability = self._probability_for_elapsed_time(
                self._battle_mistake_probability(state, gap_seconds),
                delta_seconds,
            )
            if (
                mistake_probability > 0.0
                and self.rng.random() < mistake_probability
            ):
                intent = BattleIntent(
                    state.driver_id,
                    car_ahead.driver_id,
                    "lockup",
                    segment.name,
                )
                return self._commit_battle_intent(intent) if commit else intent
            return None
        if trajectory_decision is None:
            return None
        command_stage_attack = bool(
            pending_overtake is not None
            and pending_overtake.defender_id == car_ahead.driver_id
            and state.pace_mode == PaceMode.ATTACK
            and segment.type == TrackSegmentType.STRAIGHT
            and trajectory_decision.minimum_clearance_m
            >= MANEUVER_PULL_OUT_MIN_CLEARANCE_M
        )
        if not command_stage_attack and self.rng.random() >= probability:
            return None

        attacker = self._driver_meta[state.driver_id]
        defender = self._driver_meta[car_ahead.driver_id]
        mistake_probability = self._probability_for_elapsed_time(
            self._battle_mistake_probability(state, gap_seconds),
            delta_seconds,
        )
        if mistake_probability > 0.0 and self.rng.random() < mistake_probability:
            intent = BattleIntent(
                state.driver_id,
                car_ahead.driver_id,
                "lockup",
                segment.name,
                trajectory_decision=trajectory_decision,
            )
            return self._commit_battle_intent(intent) if commit else intent

        attack_score = (
            attacker["overtaking"]
            * self._pace_mode_attack_factor(state)
            * (1.10 if state.drs_active else 1.0)
        )
        defense_score = defender["defending"] * (
            0.88 + 0.24 * self.circuit.overtaking_difficulty
        )
        attack_score += self.rng.uniform(-0.04, 0.04)
        score_margin = attack_score - defense_score
        attacker_line = (
            trajectory_decision.attacker_line
            if trajectory_decision is not None
            else self._choose_attack_line(state, car_ahead)
        )
        defender_line = self._choose_defender_line(car_ahead, state, attacker_line)
        if trajectory_decision is not None:
            # The lattice approved the pull-out against the defender's current
            # predicted corridor.  Letting the defender choose a new blocking
            # line in the same commit invalidates that proof and can make both
            # cars cross the track together.  Once a physical pull-out is
            # reserved, the defender must hold the racing corridor until the
            # battle controller allocates explicit side-by-side corridors.
            defender_line = DEFENDER_LINE_RACING
        if (
            trajectory_decision is not None
            and trajectory_decision.candidate_id == "grid_launch_lane"
        ):
            event_kind = "grid_attack" if attack_score > defense_score else "grid_defend"
        else:
            if score_margin >= BATTLE_SIDE_BY_SIDE_SCORE_MARGIN:
                defender_line = DEFENDER_LINE_RACING
            if (
                segment.type == TrackSegmentType.HEAVY_BRAKING
                and abs(score_margin) <= BATTLE_SIDE_BY_SIDE_SCORE_MARGIN
            ):
                event_kind = "heavy_balanced"
            elif (
                segment.type == TrackSegmentType.HEAVY_BRAKING
                and score_margin >= BATTLE_FORCED_WIDE_SCORE_MARGIN
            ):
                event_kind = "heavy_forced"
            elif attack_score > defense_score:
                event_kind = "attack"
            else:
                event_kind = "defend"
        intent = BattleIntent(
            state.driver_id,
            car_ahead.driver_id,
            event_kind,
            segment.name,
            attacker_line,
            defender_line,
            trajectory_decision,
        )
        return self._commit_battle_intent(intent) if commit else intent

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
