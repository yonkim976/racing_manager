"""Core race simulation engine."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, floor, hypot, pi, sin, sqrt
import random

from models.schemas import (
    Circuit,
    Driver,
    DriverPoseInfo,
    DriverPositionInfo,
    DriverRaceHistoryInfo,
    DriverRaceState,
    LapTimeInfo,
    PaceMode,
    RaceEvent,
    RaceHistoryState,
    RacePoseState,
    RaceTickState,
    Team,
    TireCompound,
    TrackSegmentType,
    VehicleTrajectorySample,
)
from simulation.ai_strategy import choose_pit_tire, should_pit
from simulation.car_performance import CarPerformanceFactors, car_performance_factors
from simulation.collision import (
    BodyMotion,
    BodyPose,
    interpolate_pose,
    oriented_body_overlap,
    oriented_body_separation_m,
    swept_body_collision,
)
from simulation.events import roll_events
from simulation.incidents import (
    Incident,
    IncidentCause,
    IncidentSeverity,
    escalate_collision,
    roll_solo_incident,
)
from simulation.local_trajectory_planner import (
    LocalTrajectoryPlan,
    LocalTrajectoryPlanner,
    LocalTrajectoryPlannerWeights,
    LocalTrajectoryPlanningRequest,
    NearbyVehiclePredictionInput,
)
from simulation.physics import (
    GAME_TICK_SECONDS,
    compute_effective_lap_time,
    driver_pace_multiplier,
)
from simulation.fixed_step import FixedStepAccumulator
from simulation.pit_stop import compute_pit_components
from simulation.speed_profile import SpeedProfile, build_speed_profile
from simulation.track_physics import (
    DRIVING_LINE_DEFENSIVE,
    DRIVING_LINE_INSIDE,
    DRIVING_LINE_OUTSIDE,
    DRIVING_LINE_RACING,
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
    RACING_LINE_KERB_ALLOWANCE_M,
    TRACK_EDGE_MARGIN_M,
    TrackPhysicsProfile,
    build_track_physics_profile,
    build_vehicle_track_physics_profile,
)
from simulation.track_surface import TrackSurfaceProfile, VehicleSurfaceState
from simulation.vehicle_physics import (
    LOCKUP_SLIP_RATIO_THRESHOLD,
    PHYSICS_MAX_LATERAL_SPEED_MPS,
    PHYSICS_STEP_SECONDS,
    TRACTION_LOSS_SLIP_RATIO_THRESHOLD,
    LongitudinalVehiclePhysics,
    VehicleFollowingConstraint,
    VehiclePhysicsModifiers,
)
from simulation.vehicle_dimensions import PHYSICAL_CAR_WHEELBASE_M
from simulation.vehicle_dynamics import maximum_braking_deceleration_mps2
from simulation.planner_scheduler import (
    AdaptivePlannerScheduler,
    TrackSpatialIndex,
)
from simulation.track_geometry import (
    FLAT_TRACK_ELEVATION_M,
    FLAT_TRACK_GRADE,
    segment_at_progress,
)
from simulation.tire_model import (
    COMPOUND_SPECS,
    TirePhysicsFactors,
    advance_tire_thermal_state,
    compute_managed_tire_age,
    compute_tire_performance,
    compute_tire_physics_factors,
    compute_wear,
    tire_blanket_temperature_c,
)
from simulation.trajectory_physics import (
    TireTrajectorySpec,
    VehicleTrajectorySpec,
)
from simulation.state_contract import (
    CollisionFact,
    TELEMETRY_SOURCE_DERIVED,
    PhysicsStepResult,
    ProgressCrossingFact,
    TickPhase,
)
from simulation.wake_model import (
    WAKE_MAX_GAP_M,
    WAKE_MIN_GAP_M,
    WakeEffects,
    compute_wake_effects,
)

MAX_PHYSICS_DIAGNOSTIC_SAMPLES = 256
TIMING_CROSSING_LAPS_TO_RETAIN = 4

PACE_MODE_EFFECTS: dict[PaceMode, dict[str, float]] = {
    PaceMode.CONSERVE: {"lap_time_delta": 0.0, "tire_usage_multiplier": 0.82},
    PaceMode.STANDARD: {"lap_time_delta": 0.0, "tire_usage_multiplier": 1.0},
    PaceMode.ATTACK: {"lap_time_delta": 0.0, "tire_usage_multiplier": 1.28},
}
FUEL_CONSUMPTION_KG_PER_KM = 0.34
FUEL_LOAD_RESERVE_FACTOR = 1.03
MAX_INITIAL_FUEL_MASS_KG = 110.0
MAX_FUEL_FLOW_KG_PER_SECOND = 0.028
IDLE_FUEL_FLOW_KG_PER_SECOND = 0.0007
TIRE_SLIDE_WEAR_LAPS_PER_JOULE = 1.0e-6
PACE_MODE_ATTACK_FACTORS: dict[PaceMode, float] = {
    PaceMode.CONSERVE: 0.55,
    PaceMode.STANDARD: 0.85,
    PaceMode.ATTACK: 1.15,
}
PACE_MODE_DRS_FACTORS: dict[PaceMode, dict[str, float]] = {
    PaceMode.CONSERVE: {"target_speed": 1.015},
    PaceMode.STANDARD: {"target_speed": 1.025},
    PaceMode.ATTACK: {"target_speed": 1.035},
}
PACE_MODE_PHYSICS_FACTORS: dict[PaceMode, dict[str, float]] = {
    PaceMode.CONSERVE: {
        "pace": 0.985,
        "grip": 0.992,
        "braking": 0.988,
        "traction": 0.990,
    },
    PaceMode.STANDARD: {
        "pace": 1.0,
        "grip": 1.0,
        "braking": 1.0,
        "traction": 1.0,
    },
    PaceMode.ATTACK: {
        "pace": 1.020,
        "grip": 1.004,
        "braking": 1.006,
        "traction": 1.008,
    },
}
PACE_MODE_PLANNER_WEIGHTS: dict[PaceMode, LocalTrajectoryPlannerWeights] = {
    PaceMode.CONSERVE: LocalTrajectoryPlannerWeights(
        forward_progress=0.85,
        extra_path_distance=1.20,
        lateral_deviation=1.25,
        surface=1.25,
        tire_surface=1.40,
        traffic=1.25,
    ),
    PaceMode.STANDARD: LocalTrajectoryPlannerWeights(),
    PaceMode.ATTACK: LocalTrajectoryPlannerWeights(
        forward_progress=1.15,
        extra_path_distance=0.90,
        lateral_deviation=0.80,
        surface=0.90,
        tire_surface=0.80,
        traffic=0.90,
    ),
}
PACE_MODE_INTENSITIES: dict[PaceMode, float] = {
    PaceMode.CONSERVE: -1.0,
    PaceMode.STANDARD: 0.0,
    PaceMode.ATTACK: 1.0,
}
PACE_MODE_TRANSITION_SECONDS = 1.5
RACING_ACCELERATION_MPS2 = 12.0
RACING_BRAKING_MPS2 = 34.0
GRID_SLOT_PROGRESS_GAP = 0.0028
GRID_POLE_DISTANCE_BEHIND_LINE_M = 8.0
GRID_SLOT_SPACING_M = 8.0
GRID_COLUMN_OFFSET_M = 2.35
GRID_FIRST_LIGHT_SECONDS = 0.25
GRID_LIGHT_INTERVAL_SECONDS = 0.62
GRID_LIGHTS_OUT_SECONDS = GRID_FIRST_LIGHT_SECONDS + GRID_LIGHT_INTERVAL_SECONDS * 5
GRID_LIGHTS_OUT_DISPLAY_SECONDS = 0.70
GRID_LAUNCH_LANE_HOLD_M = 35.0
GRID_LAUNCH_MERGE_DISTANCE_M = 135.0
GRID_OVERTAKE_LOCKOUT_SECONDS = 0.50
# The selected path is controlled by 50 Hz physics and tactical intent is
# refreshed at 10 Hz.  Rebuilding the full 3 s × 5/7-candidate occupancy
# lattice at 5 Hz for all 20 cars is redundant; 1 Hz leaves nine tactical
# opportunities between routine path rebuilds.  Safety/battle transitions can
# still force an immediate replan by setting the per-driver plan age.
LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS = 1.00
AI_TACTICAL_DECISION_INTERVAL_SECONDS = 0.10
ATTACK_LINE_CHOICE_INTERVAL_SECONDS = 0.20
LOCAL_TRAJECTORY_OPPONENT_RADIUS_M = 150.0
LOCAL_TRAJECTORY_DENSE_TRAFFIC_RADIUS_M = 30.0
LOCAL_TRAJECTORY_MAX_OPPONENTS = 4
LOCAL_TRAJECTORY_MIN_SELECTED_CLEARANCE_M = 0.35
MANEUVER_PULL_OUT_MIN_CLEARANCE_M = 0.15
MANEUVER_MIN_PREDICTED_COLLISION_TIME_SECONDS = 0.75
MANEUVER_REAR_PLANNING_CLEARANCE_M = 20.0
VSC_DURATION_SECONDS = 25.0
VSC_LAP_TIME_FACTOR = 1.4
SC_LAP_TIME_FACTOR = 1.8
SC_COLLECTION_LAP_TIME_FACTOR = 2.5
SC_CATCH_UP_FAST_LAP_TIME_FACTOR = 0.85
SC_CATCH_UP_NEAR_LAP_TIME_FACTOR = 1.30
SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR = 1.60
SC_UNLAP_LAP_TIME_FACTOR = 1.0
SC_LEAD_PROGRESS_GAP = 0.012  # lap-fraction gap between the SC and the on-track leader
SC_CAR_LENGTH_M = PHYSICAL_CAR_LENGTH_M
SC_MAX_GAP_CAR_LENGTHS = 10.0
SC_RELEASE_GAP_CAR_LENGTHS = 12.0
SC_QUEUE_TARGET_CAR_LENGTHS = 7.0
SC_CATCH_UP_MAX_SPEED_KPH = 300.0
SC_NEAR_QUEUE_MAX_SPEED_KPH = 220.0
SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS = 40.0
SC_CAUGHT_MAX_SPEED_KPH = 180.0
SC_QUEUE_APPROACH_REACTION_SECONDS = 0.65
SC_QUEUE_APPROACH_DECELERATION_MPS2 = 5.0
SC_QUEUE_APPROACH_MAX_GAP_M = 800.0
SC_QUEUE_PROPAGATION_HORIZON_M = 600.0
SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH = 24.0
SC_UNLAP_MAX_SPEED_KPH = 280.0
SC_LEADER_ACQUISITION_SPEED_KPH = 5.0
SC_DEPLOY_PIT_SECONDS = 2.5
SC_WITHDRAW_PIT_SECONDS = 3.0
SC_CLEANUP_SECONDS = 35.0
SC_ADDITIONAL_INCIDENT_SECONDS = 18.0
SC_PIT_WEAR_THRESHOLD = 0.30  # AI takes the "free" SC pit once tires are this worn
SC_PIT_PROBABILITY = 0.4  # per-eligible-driver chance to dive in under SC (avoids all-stop)
PIT_LANE_SPEED_LIMIT_KPH = 80.0
PIT_ENTRY_BRAKING_MPS2 = 35.0
PIT_BOX_BRAKING_MPS2 = 10.0
PIT_LANE_ACCELERATION_MPS2 = 5.0
PIT_EXIT_ACCELERATION_MPS2 = 9.0
PIT_TEAM_BOX_SPACING_M = 16.0
PIT_BOX_LATERAL_BLEND_DISTANCE_M = 22.0
PIT_MERGE_BRAKING_MPS2 = 7.0
PIT_MERGE_HOLD_DISTANCE_M = 4.0
PIT_MERGE_LONGITUDINAL_MARGIN_M = 7.0
PIT_MERGE_LATERAL_MARGIN_M = 0.60
PIT_MERGE_GROUP_CLEARANCE_SECONDS = 0.40
PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M = 5.0
PIT_ROUTE_TANGENT_LEAD_M = 8.0
PROGRESS_EPSILON = 1e-9
INCIDENT_MINOR_TIME_PENALTY = (1.5, 4.0)
INCIDENT_MINOR_TIRE_USAGE = 0.01
TRAFFIC_GAP_SECONDS = 1.0
DIRTY_AIR_MAX_PENALTY = 0.32
TRAFFIC_ATTACK_MAX_BONUS = 0.12
DRS_MAX_BONUS = 0.30
DRS_DRAG_MULTIPLIER = 0.82
DRS_DOWNFORCE_MULTIPLIER = 0.92
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
FOLLOWING_MIN_BUMPER_GAP_M = 1.25
MANEUVER_LIVE_CLEARANCE_BUFFER_M = 0.85
MANEUVER_PASS_CLEARANCE_MARGIN_M = 0.10
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
COLLISION_PAIR_COOLDOWN_SECONDS = 0.8
COLLISION_DISPLAY_SECONDS = 0.7
COLLISION_ESCALATION_IMPACT_MPS = 5.0
COLLISION_MIN_REPORT_IMPACT_MPS = 1.5
COLLISION_STATIC_PENETRATION_TOLERANCE_M = 0.05
HAZARD_DETECTION_DISTANCE_M = 300.0
HAZARD_CLEAR_RADIUS_M = 300.0
HAZARD_PASS_CLEARANCE_M = 15.0
HAZARD_LATERAL_MARGIN_M = 0.55
HAZARD_MOVING_RELEASE_MARGIN_M = 0.15
HAZARD_BRAKING_DECELERATION_MPS2 = 28.0
HAZARD_REACTION_SECONDS = 0.70
HAZARD_TRAFFIC_WINDOW_M = 8.0
HAZARD_CORRIDOR_PREFERENCE_PENALTY = 4.0
HAZARD_CORRIDOR_TARGET_MARGIN_M = 0.8
HAZARD_EVASIVE_LATERAL_ACCELERATION_MPS2 = 4.0
HAZARD_EVASIVE_TIME_BUFFER_SECONDS = 0.5
HAZARD_CLUSTER_LONGITUDINAL_GAP_M = 12.0
HAZARD_BLOCKED_CLEAR_SECONDS = 5.0
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
AI_PACE_MIN_COOLDOWN_SECONDS = 8.0
AI_PACE_MAX_COOLDOWN_SECONDS = 12.0
AI_PACE_INITIAL_STAGGER_MIN_SECONDS = 0.5
AI_PACE_INITIAL_STAGGER_MAX_SECONDS = 5.0
AI_ATTACK_GAP_SECONDS = 1.0
AI_ATTACK_EXIT_GAP_SECONDS = 1.35
AI_DEFEND_GAP_SECONDS = 0.85
AI_DEFEND_EXIT_GAP_SECONDS = 1.15
AI_LOW_TIRE_LIFE = 0.25
AI_CONSERVE_EXIT_TIRE_LIFE = 0.32
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
class TimingLoop:
    """One physical timing line inside a major circuit sector."""

    index: int
    progress: float
    sector_index: int
    mini_sector_index: int


@dataclass(frozen=True)
class PitMergeDecision:
    """Main-track priority result for one predicted pit-exit occupancy."""

    state: str
    conflict_driver_id: int | None = None
    conflict_group_id: str | None = None
    conflict_group_member_ids: tuple[int, ...] = ()


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


@dataclass
class DriverInputError:
    """A temporary AI control error; physics, not RNG, decides its outcome."""

    kind: str
    remaining_seconds: float
    intensity: float
    trigger: str
    segment_name: str = ""
    opponent_id: int | None = None


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
        start_sequence_enabled: bool = True,
    ):
        self.circuit = circuit
        self.drivers = {d.id: d for d in drivers}
        self.teams = teams
        self.player_team_id = player_team_id
        self.player_driver_ids = set(player_driver_ids)
        self.starting_tires = starting_tires or {}
        self.grid_order = grid_order or []
        self.rng = random.Random(seed)
        self.start_sequence_enabled = start_sequence_enabled

        self.total_laps = circuit.total_laps
        self.current_lap = 1
        self.race_elapsed = 0.0
        self._physics_frame = 0
        self._physics_accumulator = FixedStepAccumulator(PHYSICS_STEP_SECONDS)
        # These values are diagnostics, not race history. Keeping every 50 Hz
        # tick for a full race made both containers grow without bound.
        self._physics_step_deltas: deque[float] = deque(
            maxlen=MAX_PHYSICS_DIAGNOSTIC_SAMPLES,
        )
        self._tick_phase = TickPhase.TELEMETRY
        self._tick_phase_history: list[TickPhase] = []
        self._setup_mode = True
        self._consumed_physics_frame_ids: deque[int] = deque(
            maxlen=MAX_PHYSICS_DIAGNOSTIC_SAMPLES,
        )
        self._last_consumed_physics_frame_id = 0
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
        self._sc_running_order: list[int] = []
        self._sc_pit_exit_order_targets: dict[int, float] = {}
        self._sc_withdraw_target: float | None = None
        self._sc_restart_target: float | None = None
        self._sc_restart_accel_progress: float | None = None
        self.pit_window_open = False
        self.finished = False
        self.speed_multiplier = 1
        self.paused = False
        self.race_started = not start_sequence_enabled
        self.start_sequence_phase = "grid" if start_sequence_enabled else "racing"
        self.start_light_count = 0
        self._start_sequence_elapsed = 0.0
        self._lights_out_display_remaining = 0.0
        self._start_overtake_lockout_remaining = 0.0
        self.track_length_m = max(1.0, float(circuit.track_length_m))
        self._timing_loops = self._build_timing_loops()
        self._track_physics: TrackPhysicsProfile = build_track_physics_profile(circuit)
        self._track_surface = TrackSurfaceProfile.for_circuit(circuit, self._track_physics)
        self._vehicle_physics_by_line = self._build_vehicle_physics_by_line()
        self._vehicle_physics = self._vehicle_physics_by_line[DRIVING_LINE_RACING]
        self._speed_profile = self._vehicle_physics.profile
        self._track_physics_by_driver: dict[int, TrackPhysicsProfile] = {}
        self._vehicle_physics_by_driver_line: dict[
            int,
            dict[str, LongitudinalVehiclePhysics],
        ] = {}
        self._local_trajectory_planners: dict[int, LocalTrajectoryPlanner] = {}
        self._local_trajectory_plans: dict[int, LocalTrajectoryPlan] = {}
        self._pending_overtake_commands: dict[int, PendingOvertakeCommand] = {}
        self._ai_tactical_decision_elapsed = 0.0
        self._local_trajectory_plan_ages: dict[int, float] = {}
        self._local_trajectory_selected_ages: dict[int, float] = {}
        self._local_trajectory_validation_ages: dict[int, float] = {}
        self._local_trajectory_last_safe_plans: dict[int, LocalTrajectoryPlan] = {}
        self._local_trajectory_replan_counts: dict[int, int] = {}
        self._adaptive_planner_scheduler = AdaptivePlannerScheduler()
        self._planner_spatial_index = TrackSpatialIndex(self.track_length_m)
        self._pace_mode_replan_required: set[int] = set()
        self._pace_mode_intensity: dict[int, float] = {}
        self._pace_mode_transition_source: dict[int, float] = {}
        self._pace_mode_transition_elapsed: dict[int, float] = {}
        self._pace_mode_transition_target: dict[int, PaceMode] = {}
        self._pace_mode_transition_from: dict[int, PaceMode] = {}

        self.driver_states: dict[int, DriverRaceState] = {}
        self._lap_random: dict[int, float] = {}
        self._tire_random: dict[int, float] = {}
        self._progress_rate: dict[int, float] = {}
        self._safety_car_total_progress: float | None = None
        self._safety_car_progress_rate = 0.0
        self._safety_car_speed_mps = 0.0
        # Distance-integrated pit stop state (keyed by driver_id while in_pit).
        self._pit_phase: dict[int, str] = {}  # "in" | "stop" | "out"
        self._pit_phase_remaining: dict[int, float] = {}
        self._pit_phase_duration: dict[int, float] = {}
        self._pit_lane_half_time: dict[int, float] = {}
        self._pit_route_progress: dict[int, float] = {}
        self._pit_route_speed_mps: dict[int, float] = {}
        self._pit_merge_state: dict[int, str] = {}
        self._pit_merge_conflict_driver_id: dict[int, int | None] = {}
        self._pit_merge_conflict_group_id: dict[int, str | None] = {}
        self._pit_merge_conflict_group_member_ids: dict[int, tuple[int, ...]] = {}
        self._pit_tire_change_time: dict[int, float] = {}
        self._pit_elapsed: dict[int, float] = {}
        self._pit_stop_elapsed: dict[int, float] = {}
        self._pit_tire: dict[int, TireCompound] = {}
        self._pit_entry_compound: dict[int, TireCompound] = {}
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
        self._driver_meta: dict[int, dict] = {}
        self._finish_order: list[int] = []
        self._battle_event_cooldown: dict[int, float] = {}
        self._battle_effects: dict[int, BattleEffect] = {}
        self._side_by_side_battles: dict[tuple[int, int], SideBySideBattle] = {}
        self._attack_line_choice_cache_bucket = -1
        self._attack_line_choice_cache: dict[tuple[int, int], str] = {}
        self._maneuver_groups: dict[str, ManeuverGroup] = {}
        self._maneuver_group_signatures: set[tuple[int, ...]] = set()
        self._forced_wide_by_driver: dict[int, int] = {}
        self._pending_forced_wide_events: list[RaceEvent] = []
        self._pending_physical_handling_events: list[RaceEvent] = []
        self._physical_handling_event_cooldown: dict[int, float] = {}
        self._driver_input_errors: dict[int, DriverInputError] = {}
        self._forced_wide_aftermaths: dict[tuple[int, int], ForcedWideAftermath] = {}
        self._pending_incidents: list[Incident] = []
        self._ai_pace_cooldown: dict[int, float] = {}
        self._collision_pair_cooldown: dict[tuple[int, int], float] = {}
        self._contact_display_remaining: dict[int, float] = {}
        self._hazard_tail_targets: dict[int, tuple[int, float]] = {}
        self._hazard_activated_at: dict[int, float] = {}
        self._grid_start_progress: dict[int, float] = {}
        self._grid_lateral_offsets: dict[int, float] = {}

        self._init_grid(drivers)
        self._initialize_driver_trajectory_physics()
        self._initialize_authoritative_vehicle_telemetry()

    def _build_vehicle_physics_by_line(
        self,
        track_physics: TrackPhysicsProfile | None = None,
    ) -> dict[str, LongitudinalVehiclePhysics]:
        track_profile = track_physics or self._track_physics
        physics_by_line: dict[str, LongitudinalVehiclePhysics] = {}
        calibration = self.circuit.physics_calibration
        controller_options = {
            "planner_braking_utilization": (
                calibration.planner_braking_utilization if calibration else 1.0
            ),
            "controller_sample_distance_m": (
                calibration.controller_sample_distance_m if calibration else 50.0
            ),
            "controller_speed_scale_floor": (
                calibration.controller_speed_scale_floor if calibration else 0.60
            ),
            "telemetry_speed_reference_weight": (
                calibration.telemetry_speed_reference_weight if calibration else 0.0
            ),
            "telemetry_braking_curvature_threshold": (
                calibration.telemetry_braking_curvature_threshold
                if calibration
                else 0.5
            ),
            "telemetry_max_braking_utilization": (
                calibration.telemetry_max_braking_utilization
                if calibration
                else 1.0
            ),
            "telemetry_braking_speed_reserve": (
                calibration.telemetry_braking_speed_reserve
                if calibration
                else 1.0
            ),
            "braking_longitudinal_grip_factor": (
                calibration.braking_longitudinal_grip_factor
                if calibration
                else 1.0
            ),
            "brake_control_error_fraction": (
                calibration.brake_control_error_fraction
                if calibration
                else 0.1
            ),
        }
        telemetry_reference = (
            sorted(calibration.telemetry_reference, key=lambda item: item.progress)
            if calibration
            else []
        )

        def telemetry_speed_mps(progress: float) -> float | None:
            if not telemetry_reference:
                return None
            normalized = progress % 1.0
            following_index = next(
                (
                    index
                    for index, item in enumerate(telemetry_reference)
                    if item.progress > normalized
                ),
                0,
            )
            previous_index = (following_index - 1) % len(telemetry_reference)
            previous = telemetry_reference[previous_index]
            following = telemetry_reference[following_index]
            start = previous.progress
            end = following.progress
            target = normalized
            if following_index == 0:
                end += 1.0
                if target < start:
                    target += 1.0
            ratio = (target - start) / max(1e-9, end - start)
            speed_kph = previous.speed_kph + (
                following.speed_kph - previous.speed_kph
            ) * ratio
            return speed_kph / 3.6

        def telemetry_braking_fraction(progress: float) -> float:
            if not telemetry_reference:
                return 0.0
            normalized = progress % 1.0
            following_index = next(
                (
                    index
                    for index, item in enumerate(telemetry_reference)
                    if item.progress > normalized
                ),
                0,
            )
            previous_index = (following_index - 1) % len(telemetry_reference)
            previous = telemetry_reference[previous_index]
            following = telemetry_reference[following_index]
            start = previous.progress
            end = following.progress
            target = normalized
            if following_index == 0:
                end += 1.0
                if target < start:
                    target += 1.0
            ratio = (target - start) / max(1e-9, end - start)
            return previous.braking_fraction + (
                following.braking_fraction - previous.braking_fraction
            ) * ratio
        racing_prediction = max(
            1.0,
            track_profile.predicted_line_lap_times.get(
                DRIVING_LINE_RACING,
                self.circuit.base_lap_time,
            ),
        )
        for line_name, samples in track_profile.driving_line_samples.items():
            if not samples:
                continue
            profile = SpeedProfile(
                progress=[sample.path_progress for sample in samples],
                raw_speeds_mps=[
                    telemetry_speed_mps(sample.center_progress)
                    or (
                        96.0
                        if abs(sample.curvature_1pm) <= 1e-5
                        else min(
                            96.0,
                            max(28.0, sqrt(34.0 / abs(sample.curvature_1pm))),
                        )
                    )
                    for sample in samples
                ],
                curvatures_1pm=[abs(sample.curvature_1pm) for sample in samples],
                signed_curvatures_1pm=[sample.curvature_1pm for sample in samples],
                braking_fractions=[
                    telemetry_braking_fraction(sample.center_progress)
                    for sample in samples
                ],
            )
            predicted = max(
                1.0,
                track_profile.predicted_line_lap_times.get(
                    line_name,
                    racing_prediction,
                ),
            )
            racing_reference_lap_time = (
                calibration.reference_lap_time_seconds
                if calibration and calibration.reference_lap_time_seconds is not None
                else self.circuit.base_lap_time
            )
            reference_lap_time = (
                racing_reference_lap_time * predicted / racing_prediction
            )
            physics_by_line[line_name] = LongitudinalVehiclePhysics(
                profile,
                track_profile.length_for_line(line_name),
                reference_lap_time,
                **controller_options,
            )
        if DRIVING_LINE_RACING not in physics_by_line:
            fallback_profile = build_speed_profile(
                self.circuit,
                acceleration_mps2=RACING_ACCELERATION_MPS2,
                braking_mps2=RACING_BRAKING_MPS2,
            )
            physics_by_line[DRIVING_LINE_RACING] = LongitudinalVehiclePhysics(
                fallback_profile,
                self.track_length_m,
                (
                    calibration.reference_lap_time_seconds
                    if calibration and calibration.reference_lap_time_seconds is not None
                    else self.circuit.base_lap_time
                ),
                **controller_options,
            )
        return physics_by_line

    def _track_physics_for_driver(
        self,
        state_or_driver_id: DriverRaceState | int,
    ) -> TrackPhysicsProfile:
        driver_id = (
            state_or_driver_id.driver_id
            if isinstance(state_or_driver_id, DriverRaceState)
            else state_or_driver_id
        )
        return self._track_physics_by_driver.get(driver_id, self._track_physics)

    def _vehicle_physics_for_driver(
        self,
        state_or_driver_id: DriverRaceState | int,
    ) -> dict[str, LongitudinalVehiclePhysics]:
        driver_id = (
            state_or_driver_id.driver_id
            if isinstance(state_or_driver_id, DriverRaceState)
            else state_or_driver_id
        )
        return self._vehicle_physics_by_driver_line.get(
            driver_id,
            self._vehicle_physics_by_line,
        )

    def _initialize_driver_trajectory_physics(self) -> None:
        """Attach cached car/compound-specific lines after grid state exists."""
        physics_by_profile_id: dict[
            int,
            dict[str, LongitudinalVehiclePhysics],
        ] = {}
        for driver_id, state in self.driver_states.items():
            meta = self._driver_meta[driver_id]
            vehicle = VehicleTrajectorySpec.from_car_performance(
                meta["car_factors"]
            )
            tire = TireTrajectorySpec.from_tire_physics(
                state.tire_compound,
                compute_tire_physics_factors(state.tire_compound, 0.0),
            )
            track_profile = build_vehicle_track_physics_profile(
                self.circuit,
                vehicle,
                tire,
            )
            profile_id = id(track_profile)
            physics_by_line = physics_by_profile_id.get(profile_id)
            if physics_by_line is None:
                physics_by_line = self._build_vehicle_physics_by_line(track_profile)
                physics_by_profile_id[profile_id] = physics_by_line
            self._track_physics_by_driver[driver_id] = track_profile
            self._vehicle_physics_by_driver_line[driver_id] = physics_by_line
            self._local_trajectory_planners[driver_id] = LocalTrajectoryPlanner(
                track_profile,
                self._track_surface,
                physics_by_line,
                track_length_m=self.track_length_m,
            )
            # Preserve the established 0.1/0.2 s two-wave bootstrap so cars
            # approaching a newly stopped vehicle do not plan prematurely,
            # while the later steady-state rebuild cadence remains 1 Hz.
            first_plan_delay_s = 0.1 if driver_id % 2 == 0 else 0.2
            self._local_trajectory_plan_ages[driver_id] = (
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
                - first_plan_delay_s
            )
            self._local_trajectory_selected_ages[driver_id] = 0.0
            self._local_trajectory_validation_ages[driver_id] = 0.0
            self._local_trajectory_replan_counts[driver_id] = 0
            if not self.start_sequence_enabled:
                offset = track_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
                state.lateral_offset_m = offset
                state.target_lateral_offset_m = offset
                state.speed_kph = self._initial_speed_kph(
                    state.progress,
                    meta["car_factors"],
                    meta["pace"],
                    state.tire_compound,
                    track_profile=track_profile,
                    physics_by_line=physics_by_line,
                )

    def _init_grid(self, drivers: list[Driver]) -> None:
        """Initialize driver states from a supplied grid or simulated qualifying pace."""
        qualifying = []
        for driver in drivers:
            team = self.teams[driver.team_id]
            pace = driver_pace_multiplier(driver.stats.pace)
            car_factors = car_performance_factors(team)
            score = car_factors.qualifying * pace + self.rng.uniform(-0.005, 0.005)
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
            car_factors = car_performance_factors(team)
            starting_compound = self.starting_tires.get(driver.id, TireCompound.MEDIUM)
            if self.start_sequence_enabled:
                grid_distance_m = (
                    GRID_POLE_DISTANCE_BEHIND_LINE_M
                    + (position - 1) * GRID_SLOT_SPACING_M
                )
                grid_progress = -grid_distance_m / self.track_length_m
                grid_lateral_offset_m = (
                    GRID_COLUMN_OFFSET_M
                    if position % 2 == 1
                    else -GRID_COLUMN_OFFSET_M
                )
                initial_speed_kph = 0.0
            else:
                grid_progress = -(position - 1) * GRID_SLOT_PROGRESS_GAP
                initial_track_sample = self._track_physics.at_progress(grid_progress)
                grid_lateral_offset_m = initial_track_sample.racing_line_offset_m
                initial_speed_kph = self._initial_speed_kph(
                    grid_progress,
                    car_factors,
                    driver_pace_multiplier(driver.stats.pace),
                    starting_compound,
                )
            self._grid_start_progress[driver.id] = grid_progress
            self._grid_lateral_offsets[driver.id] = grid_lateral_offset_m
            initial_fuel_mass_kg = min(
                MAX_INITIAL_FUEL_MASS_KG,
                self._expected_fuel_per_lap_kg()
                * self.total_laps
                * FUEL_LOAD_RESERVE_FACTOR,
            )
            blanket_temperature_c = tire_blanket_temperature_c(starting_compound)
            self.driver_states[driver.id] = DriverRaceState(
                driver_id=driver.id,
                position=position,
                progress=grid_progress,
                current_lap=0,
                total_progress=grid_progress,
                tire_compound=starting_compound,
                tire_age=0,
                tire_wear=0.0,
                gap_to_leader=0.0,
                in_pit=False,
                pit_count=0,
                retired=False,
                total_time=0.0,
                speed_kph=initial_speed_kph,
                brake=1.0 if self.start_sequence_enabled else 0.0,
                total_distance_m=0.0,
                pace_mode=PaceMode.STANDARD,
                lateral_offset_m=grid_lateral_offset_m,
                target_lateral_offset_m=grid_lateral_offset_m,
                car_width_m=PHYSICAL_CAR_WIDTH_M,
                car_length_m=PHYSICAL_CAR_LENGTH_M,
                wheelbase_m=PHYSICAL_CAR_WHEELBASE_M,
                tire_surface_temperature_c=blanket_temperature_c,
                tire_core_temperature_c=blanket_temperature_c,
                fuel_mass_kg=initial_fuel_mass_kg,
                fuel_laps_remaining=(
                    initial_fuel_mass_kg / self._expected_fuel_per_lap_kg()
                ),
            )
            self._driver_meta[driver.id] = {
                "abbreviation": driver.abbreviation,
                "full_name": driver.name,
                "team_name": team.name,
                "team_color": team.color,
                "car_factors": car_factors,
                "pit_crew_skill": team.pit_crew_skill,
                "pace": driver_pace_multiplier(driver.stats.pace),
                "consistency": driver.stats.consistency,
                "tire_management": driver.stats.tire_management,
                "overtaking": driver.stats.overtaking,
                "defending": driver.stats.defending,
            }
            self._lap_history[driver.id] = []
            self._timing_crossings[driver.id] = {}
            if abs(grid_progress) <= 1e-12:
                self._timing_crossings[driver.id][(0, 0)] = 0.0
            self._timing_gap_valid[driver.id] = False
            self._interval_timing_gap_valid[driver.id] = False
            self._live_interval_seconds[driver.id] = None
            self._last_sector_time[driver.id] = 0.0
            self._last_mini_sector_time[driver.id] = 0.0
            self._last_completed_mini_sector_index[driver.id] = None
            self._personal_best_mini_sector_times[driver.id] = [
                None for _ in self._timing_loops
            ]
            self._pace_mode_intensity[driver.id] = 0.0
            self._pace_mode_transition_source[driver.id] = 0.0
            self._pace_mode_transition_elapsed[driver.id] = (
                PACE_MODE_TRANSITION_SECONDS
            )
            self._pace_mode_transition_target[driver.id] = PaceMode.STANDARD
            self._pace_mode_transition_from[driver.id] = PaceMode.STANDARD
            if driver.id not in self.player_driver_ids:
                stagger_span = (
                    AI_PACE_INITIAL_STAGGER_MAX_SECONDS
                    - AI_PACE_INITIAL_STAGGER_MIN_SECONDS
                )
                stagger_ratio = ((driver.id * 47) % 101) / 100.0
                self._ai_pace_cooldown[driver.id] = (
                    AI_PACE_INITIAL_STAGGER_MIN_SECONDS
                    + stagger_span * stagger_ratio
                )
            self._refresh_lap_variation(driver.id)
            setattr(self.driver_states[driver.id], "_lap_start_time", 0.0)

    def _consistency_variation_spread(self, consistency: float) -> float:
        """Return per-lap random swing from driver consistency."""
        return 0.10 + (1.0 - consistency) * 1.35

    def _initial_speed_kph(
        self,
        progress: float,
        car_factors: CarPerformanceFactors,
        driver_pace: float,
        tire_compound: TireCompound,
        *,
        track_profile: TrackPhysicsProfile | None = None,
        physics_by_line: dict[str, LongitudinalVehiclePhysics] | None = None,
    ) -> float:
        selected_track = track_profile or self._track_physics
        selected_physics = physics_by_line or self._vehicle_physics_by_line
        tire_factors = compute_tire_physics_factors(tire_compound, 0.0)
        modifiers = VehiclePhysicsModifiers(
            power=1.0,
            grip=tire_factors.lateral_grip,
            pace=driver_pace,
            braking=tire_factors.braking_grip,
            traction=tire_factors.traction_grip,
            mass_kg=car_factors.mass_kg,
            effective_power_kw=car_factors.effective_power_kw,
            drivetrain_efficiency=car_factors.drivetrain_efficiency,
            corner_drag_area_m2=car_factors.corner_drag_area_m2,
            straight_drag_area_m2=car_factors.straight_drag_area_m2,
            corner_downforce_area_m2=car_factors.corner_downforce_area_m2,
            straight_downforce_area_m2=car_factors.straight_downforce_area_m2,
            brake_force_n=car_factors.brake_force_n,
            mechanical_grip=car_factors.mechanical_grip,
            front_aero_share=car_factors.front_aero_share,
            traction_factor=car_factors.traction_factor,
        )
        speed_mps = selected_physics[DRIVING_LINE_RACING].target_speed_mps(
            selected_track.line_distance_at_total_progress(
                DRIVING_LINE_RACING,
                progress,
            ),
            modifiers,
        )
        return round(speed_mps * 3.6, 3)

    def _physics_v2_modifiers(
        self,
        state: DriverRaceState,
        surface: VehicleSurfaceState | None = None,
    ) -> VehiclePhysicsModifiers:
        meta = self._driver_meta[state.driver_id]
        car_factors = meta["car_factors"]
        tire_factors = self._current_tire_physics(state)
        braking_execution, traction_execution = self._maneuver_execution_factors(state)
        corner_braking, corner_traction = self._corner_maneuver_factors(state)
        state.tire_wear = tire_factors.wear
        state.tire_lateral_grip = tire_factors.lateral_grip
        state.tire_traction_grip = tire_factors.traction_grip
        state.tire_braking_grip = tire_factors.braking_grip
        pace_mode_factors = self._interpolated_pace_mode_values(
            state,
            PACE_MODE_PHYSICS_FACTORS,
        )
        drs_target_speed_factor = self._interpolated_pace_mode_values(
            state,
            PACE_MODE_DRS_FACTORS,
        )["target_speed"]
        battle_lap_delta = self._battle_effect_lap_time_delta(state)
        battle_speed_factor = self.circuit.base_lap_time / max(
            1.0,
            self.circuit.base_lap_time + battle_lap_delta,
        )
        surface_grip = surface.lateral_grip_multiplier if surface else 1.0
        surface_braking = surface.braking_grip_multiplier if surface else 1.0
        surface_traction = surface.traction_grip_multiplier if surface else 1.0
        damage = max(0.0, min(1.0, state.collision_damage))
        input_error = self._driver_input_errors.get(state.driver_id)
        return VehiclePhysicsModifiers(
            power=(0.0 if state.fuel_mass_kg <= 0.0 else 1.0 - 0.08 * damage),
            grip=(
                tire_factors.lateral_grip
                * state.wake_lateral_grip_multiplier
                * surface_grip
                * pace_mode_factors["grip"]
                * (1.0 - 0.06 * damage)
            ),
            pace=(
                meta["pace"]
                * pace_mode_factors["pace"]
                * battle_speed_factor
            ),
            speed_limit_factor=(
                self._phase_vehicle_speed_factor(state)
                * (0.35 if state.emergency_braking else 1.0)
                * (drs_target_speed_factor if state.drs_active else 1.0)
            ),
            maximum_speed_mps=self._race_control_speed_cap_mps(state),
            braking=(
                tire_factors.braking_grip
                * state.wake_braking_grip_multiplier
                * braking_execution
                * surface_braking
                * corner_braking
                * pace_mode_factors["braking"]
                * (1.0 - 0.05 * damage)
            ),
            traction=(
                tire_factors.traction_grip
                * traction_execution
                * surface_traction
                * corner_traction
                * pace_mode_factors["traction"]
                * (1.0 - 0.05 * damage)
            ),
            mass_kg=car_factors.mass_kg + state.fuel_mass_kg,
            effective_power_kw=car_factors.effective_power_kw,
            drivetrain_efficiency=car_factors.drivetrain_efficiency,
            corner_drag_area_m2=car_factors.corner_drag_area_m2,
            straight_drag_area_m2=car_factors.straight_drag_area_m2,
            corner_downforce_area_m2=car_factors.corner_downforce_area_m2,
            straight_downforce_area_m2=car_factors.straight_downforce_area_m2,
            brake_force_n=car_factors.brake_force_n,
            mechanical_grip=car_factors.mechanical_grip,
            front_aero_share=car_factors.front_aero_share,
            traction_factor=car_factors.traction_factor,
            drag_multiplier=(
                state.wake_drag_multiplier
                * (DRS_DRAG_MULTIPLIER if state.drs_active else 1.0)
                * (1.0 + 0.10 * damage)
            ),
            downforce_multiplier=(
                state.wake_downforce_multiplier
                * (DRS_DOWNFORCE_MULTIPLIER if state.drs_active else 1.0)
                * (1.0 - 0.12 * damage)
            ),
            predictive_downforce_multiplier=(
                state.wake_downforce_multiplier * (1.0 - 0.12 * damage)
            ),
            surface_drag_deceleration_mps2=(
                surface.drag_deceleration_mps2 if surface else 0.0
            ),
            brake_modulation_error=(
                input_error.intensity
                if input_error is not None and input_error.kind == "brake"
                else 0.0
            ),
            throttle_modulation_error=(
                input_error.intensity
                if input_error is not None and input_error.kind == "throttle"
                else 0.0
            ),
            emergency_braking=state.emergency_braking,
            wheelbase_m=state.wheelbase_m,
            yaw_inertia_kgm2=(
                1700.0
                * (car_factors.mass_kg + state.fuel_mass_kg)
                / max(1.0, car_factors.mass_kg)
            ),
        )

    def _track_condition_at_progress(
        self,
        _progress: float,
    ) -> tuple[float, float]:
        """Return the planar runtime contract for every circuit.

        Track sources may preserve elevation, grade and banking metadata, but
        those estimated channels are intentionally inactive in first-stage
        physics and telemetry.
        """
        return FLAT_TRACK_ELEVATION_M, FLAT_TRACK_GRADE

    def _race_control_speed_cap_mps(
        self,
        state: DriverRaceState,
    ) -> float | None:
        """Return a hard SC speed ceiling in addition to the delta target."""
        if self.race_phase != "sc":
            return None
        if state.driver_id in self._sc_unlap_driver_ids:
            return SC_UNLAP_MAX_SPEED_KPH / 3.6
        base_cap_mps = (
            SC_CAUGHT_MAX_SPEED_KPH / 3.6
            if state.driver_id in self._sc_caught_driver_ids
            else SC_CATCH_UP_MAX_SPEED_KPH / 3.6
        )
        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            ahead_is_queued = True
            ahead_speed_mps = self._safety_car_speed_mps
        else:
            ahead_total = predecessor.total_progress
            ahead_is_queued = (
                predecessor.driver_id in self._sc_caught_driver_ids
                or self._sc_queue_control_reaches(predecessor)
            )
            ahead_speed_mps = max(0.0, predecessor.speed_kph / 3.6)
        gap_m = (
            (ahead_total - state.total_progress) * self.track_length_m
            if ahead_total is not None
            else float("inf")
        )
        if (
            ahead_is_queued
            and 0.0 <= gap_m
            <= max(
                SC_QUEUE_APPROACH_MAX_GAP_M,
                SC_CAR_LENGTH_M * SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS,
            )
        ):
            follower_speed_mps = max(0.0, state.speed_kph / 3.6)
            closing_speed_mps = max(0.0, follower_speed_mps - ahead_speed_mps)
            available_braking_distance_m = max(
                0.0,
                gap_m
                - SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
                - closing_speed_mps * SC_QUEUE_APPROACH_REACTION_SECONDS,
            )
            safe_approach_speed_mps = sqrt(
                ahead_speed_mps * ahead_speed_mps
                + 2.0
                * SC_QUEUE_APPROACH_DECELERATION_MPS2
                * available_braking_distance_m
            )
            approach_cap_mps = min(base_cap_mps, safe_approach_speed_mps)
            if gap_m <= SC_CAR_LENGTH_M * SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS:
                approach_cap_mps = min(
                    approach_cap_mps,
                    SC_NEAR_QUEUE_MAX_SPEED_KPH / 3.6,
                )
            return approach_cap_mps
        return base_cap_mps

    def _apply_surface_state(
        self,
        state: DriverRaceState,
        surface: VehicleSurfaceState,
    ) -> None:
        state.surface_state = surface.surface_state.value
        state.wheel_surfaces = surface.wheel_surfaces
        state.kerb_contact = surface.kerb_contact
        state.off_track = surface.off_track
        state.off_track_cause = "driver_error" if surface.off_track else ""
        state.track_limits_active = surface.track_limits_active
        state.surface_grip_multiplier = round(surface.lateral_grip_multiplier, 4)
        state.surface_drag_deceleration_mps2 = round(
            surface.drag_deceleration_mps2,
            4,
        )

        if not surface.off_track:
            self._forced_wide_by_driver.pop(state.driver_id, None)

        maneuver_group = self._maneuver_group_for_driver(state.driver_id)
        if (
            surface.off_track
            and maneuver_group is not None
            and maneuver_group.size >= 3
            and maneuver_group.corner_turn_direction != 0
        ):
            direction = maneuver_group.corner_turn_direction
            state_inside_coordinate = direction * state.lateral_offset_m
            candidates = []
            for opponent_id in maneuver_group.member_ids:
                if opponent_id == state.driver_id:
                    continue
                opponent = self.driver_states[opponent_id]
                if direction * opponent.lateral_offset_m <= state_inside_coordinate:
                    continue
                longitudinal_gap_m = abs(
                    opponent.total_progress - state.total_progress
                ) * self.track_length_m
                if longitudinal_gap_m > 0.5 * (
                    state.car_length_m + opponent.car_length_m
                ) + 0.5:
                    continue
                lateral_separation_m = abs(
                    state.lateral_offset_m - opponent.lateral_offset_m
                )
                required_m = 0.5 * (
                    state.car_width_m + opponent.car_width_m
                ) + CORNER_CORRIDOR_MARGIN_M
                if lateral_separation_m < required_m + 0.15:
                    candidates.append((lateral_separation_m, opponent_id))
            if candidates:
                _, forcing_driver_id = min(candidates)
                self._record_forced_wide(state.driver_id, forcing_driver_id)
                return

        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if (
            surface.off_track
            and battle is not None
            and battle.corner_active
            and self._corner_corridor_for_driver(battle, state.driver_id) == "outside"
        ):
            opponent_id = (
                battle.defender_id
                if state.driver_id == battle.attacker_id
                else battle.attacker_id
            )
            opponent = self.driver_states[opponent_id]
            separation = abs(state.lateral_offset_m - opponent.lateral_offset_m)
            required = PHYSICAL_CAR_WIDTH_M + CORNER_CORRIDOR_MARGIN_M
            if separation < required + 0.15:
                battle.forced_wide_driver_id = state.driver_id
                self._record_forced_wide(state.driver_id, opponent_id)

    def _record_forced_wide(
        self,
        forced_driver_id: int,
        forcing_driver_id: int,
    ) -> None:
        """Record one physical forcing relationship without pair ownership."""
        forced = self.driver_states[forced_driver_id]
        forced.off_track_cause = "forced_wide"
        if self._forced_wide_by_driver.get(forced_driver_id) == forcing_driver_id:
            return
        self._forced_wide_by_driver[forced_driver_id] = forcing_driver_id
        segment = segment_at_progress(self.circuit, forced.progress)
        segment_name = segment.name if segment is not None else "the corner"
        self._start_forced_wide_aftermath(
            forcing_driver_id,
            forced_driver_id,
            segment_name,
        )
        forcing_meta = self._driver_meta[forcing_driver_id]
        forced_meta = self._driver_meta[forced_driver_id]
        self._pending_forced_wide_events.append(
            RaceEvent(
                type="forced_wide",
                driver=forced_meta["abbreviation"],
                message=(
                    f"{forcing_meta['full_name']} forces "
                    f"{forced_meta['full_name']} wide at {segment_name}"
                ),
                message_ko=(
                    f"{forcing_meta['full_name']}가 {segment_name}에서 "
                    f"{forced_meta['full_name']}를 바깥으로 밀어냅니다"
                ),
                payload={
                    "forcing_driver_id": forcing_driver_id,
                    "forced_driver_id": forced_driver_id,
                },
            )
        )

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

    def _physics_v2_progress_delta(
        self,
        state: DriverRaceState,
        delta: float,
        car_ahead: DriverRaceState | None,
        car_behind: DriverRaceState | None,
        start_snapshot: dict[int, tuple[float, float]],
    ) -> float:
        self._require_tick_phase(TickPhase.PHYSICS, "vehicle physics integration")
        track_profile = self._track_physics_for_driver(state)
        physics_by_line = self._vehicle_physics_for_driver(state)
        active_line = self._physics_v2_virtual_line(state)
        if active_line not in physics_by_line:
            active_line = DRIVING_LINE_RACING
        state.racing_line = active_line
        line_physics = physics_by_line[active_line]
        start_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            state.total_progress,
        )
        elevation_m, track_grade = self._track_condition_at_progress(state.progress)
        state.track_elevation_m = round(elevation_m, 3)
        state.track_grade = round(track_grade, 6)
        track_sample = track_profile.at_progress(state.progress)
        surface = self._track_surface.vehicle_state(
            progress=state.progress,
            lateral_offset_m=state.lateral_offset_m,
            track_length_m=self.track_length_m,
            slip_angle_rad=state.slip_angle_rad,
        )
        self._apply_surface_state(state, surface)
        if self._local_trajectory_basic_planning_allowed(state, active_line):
            self._update_local_trajectory_plan(
                state,
                active_line,
                None,
                delta,
                surface=surface,
            )
        else:
            nearby_states = self._local_trajectory_nearby_states(state)
            nearest_distance_m = (
                abs(nearby_states[0].total_progress - state.total_progress)
                * self.track_length_m
                if nearby_states
                else None
            )
            reactive_schedule = self._adaptive_planner_scheduler.choose(
                emergency=(
                    state.avoidance_active
                    or state.emergency_braking
                    or state.local_yellow_active
                    or self._nearest_active_hazard(state) is not None
                ),
                battle=(
                    self._side_by_side_battle_for_driver(state.driver_id)
                    is not None
                ),
                nearest_distance_m=nearest_distance_m,
            )
            # The 50 Hz physics/avoidance controller remains authoritative
            # when a lattice is unsafe or ineligible.  Keep the selected tier
            # observable even though the expensive plan is deliberately
            # discarded rather than regenerated during that condition.
            state.planner_tier_hz = reactive_schedule.tier_hz
            state.planner_mode = reactive_schedule.mode
            state.planner_nearby_vehicle_count = len(nearby_states)
            state.planner_fallback_active = False
            self._local_trajectory_plans.pop(state.driver_id, None)
            self._local_trajectory_plan_ages[state.driver_id] = (
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
            )
            self._local_trajectory_selected_ages[state.driver_id] = 0.0
        state.target_lateral_offset_m = self._physics_v2_target_lateral_offset(
            state,
            track_sample,
            active_line,
            car_ahead,
            car_behind,
        )
        target_lateral_speed_mps = self._physics_v2_target_lateral_speed(
            state,
            active_line,
            state.target_lateral_offset_m,
        )
        tactical_lateral_motion = bool(
            state.avoidance_active
            or self._side_by_side_battle_for_driver(state.driver_id) is not None
            or state.driver_id in self._pending_overtake_commands
        )
        maximum_lateral_speed_mps = (
            PHYSICS_MAX_LATERAL_SPEED_MPS
            if tactical_lateral_motion or self.race_phase != "green"
            else 1.25
        )
        minimum_lateral, maximum_lateral = self._track_surface.safety_lateral_bounds(
            state.progress
        )
        nominal_lateral_bounds = self._physics_v2_nominal_lateral_bounds(state)
        following = self._physics_v2_following_constraint(
            state,
            car_ahead,
            start_snapshot,
            active_line,
            delta,
        )
        previous_wheel_lock_ratio = state.wheel_lock_ratio
        previous_traction_slip_ratio = state.traction_slip_ratio
        result = line_physics.advance(
            distance_m=start_distance_m,
            speed_mps=max(0.0, state.speed_kph / 3.6),
            delta_seconds=delta,
            modifiers=self._physics_v2_modifiers(state, surface),
            following=following,
            lateral_offset_m=state.lateral_offset_m,
            lateral_speed_mps=state.lateral_speed_mps,
            target_lateral_offset_m=state.target_lateral_offset_m,
            target_lateral_speed_mps=target_lateral_speed_mps,
            reference_lateral_offset_m=track_profile.line_offset_at_progress(
                active_line,
                state.progress,
            ),
            # The reference offset only changes the origin of the track-relative
            # coordinate.  It is not another lateral velocity command.  Passing
            # the target speed here as well made a line-speed change appear as
            # an instantaneous +/-8 m/s body slip before the controller moved
            # the car at all.
            reference_lateral_speed_mps=0.0,
            maximum_lateral_speed_mps=maximum_lateral_speed_mps,
            minimum_lateral_offset_m=minimum_lateral,
            maximum_lateral_offset_m=maximum_lateral,
            nominal_minimum_lateral_offset_m=(
                nominal_lateral_bounds[0]
                if nominal_lateral_bounds is not None
                else None
            ),
            nominal_maximum_lateral_offset_m=(
                nominal_lateral_bounds[1]
                if nominal_lateral_bounds is not None
                else None
            ),
            heading_error_rad=state.slip_angle_rad,
            yaw_rate_rad_s=state.yaw_rate_rad_s,
        )
        self._consume_fuel(
            state,
            throttle=result.throttle,
            delta_seconds=delta,
        )
        self._update_tire_thermal_state(state, result, delta)
        traveled_distance_m = max(0.0, result.distance_m - start_distance_m)
        state.total_distance_m += traveled_distance_m
        state.speed_kph = round(result.speed_mps * 3.6, 3)
        state.acceleration_mps2 = round(result.acceleration_mps2, 4)
        state.lateral_acceleration_mps2 = round(
            result.lateral_acceleration_mps2,
            4,
        )
        state.drive_force_n = round(result.drive_force_n, 2)
        state.target_speed_kph = round(result.target_speed_mps * 3.6, 3)
        state.throttle = round(result.throttle, 4)
        state.brake = round(result.brake, 4)
        state.lateral_offset_m = round(result.lateral_offset_m, 4)
        state.lateral_speed_mps = round(result.lateral_speed_mps, 4)
        state.grip_utilization = round(result.grip_utilization, 4)
        state.handling_state = result.handling_state
        state.slip_angle_rad = round(result.slip_angle_rad, 5)
        state.yaw_rate_rad_s = round(result.yaw_rate_rad_s, 6)
        state.steering_angle_rad = round(result.steering_angle_rad, 6)
        state.wheel_lock_ratio = round(result.wheel_lock_ratio, 5)
        state.traction_slip_ratio = round(result.traction_slip_ratio, 5)
        state.tire_slide_energy_j = round(result.tire_slide_energy_j, 2)
        handling_event_cooldown = self._physical_handling_event_cooldown.get(
            state.driver_id,
            0.0,
        )
        meta = self._driver_meta[state.driver_id]
        input_error = self._driver_input_errors.get(state.driver_id)
        input_error_payload = (
            {
                "trigger": input_error.trigger,
                "segment_name": input_error.segment_name,
                "opponent_id": input_error.opponent_id,
            }
            if input_error is not None
            else {"trigger": "physical_limit"}
        )
        if (
            handling_event_cooldown <= 0.0
            and result.wheel_lock_ratio >= LOCKUP_SLIP_RATIO_THRESHOLD
            and previous_wheel_lock_ratio < LOCKUP_SLIP_RATIO_THRESHOLD
        ):
            self._pending_physical_handling_events.append(
                RaceEvent(
                    type="lockup",
                    driver=meta["abbreviation"],
                    message=f"{meta['full_name']} locks a tyre under braking",
                    message_ko=f"{meta['full_name']}가 제동 한계를 넘어 타이어 락업을 일으킵니다",
                    payload={
                        "driver_id": state.driver_id,
                        "wheel_lock_ratio": round(result.wheel_lock_ratio, 4),
                        "tire_slide_energy_j": round(result.tire_slide_energy_j, 1),
                        **input_error_payload,
                    },
                )
            )
            self._physical_handling_event_cooldown[state.driver_id] = 3.0
        elif (
            handling_event_cooldown <= 0.0
            and result.traction_slip_ratio >= TRACTION_LOSS_SLIP_RATIO_THRESHOLD
            and previous_traction_slip_ratio < TRACTION_LOSS_SLIP_RATIO_THRESHOLD
        ):
            self._pending_physical_handling_events.append(
                RaceEvent(
                    type="traction_loss",
                    driver=meta["abbreviation"],
                    message=f"{meta['full_name']} loses traction on corner exit",
                    message_ko=f"{meta['full_name']}가 코너 탈출에서 트랙션을 잃습니다",
                    payload={
                        "driver_id": state.driver_id,
                        "traction_slip_ratio": round(
                            result.traction_slip_ratio,
                            4,
                        ),
                        "tire_slide_energy_j": round(result.tire_slide_energy_j, 1),
                        **input_error_payload,
                    },
                )
            )
            self._physical_handling_event_cooldown[state.driver_id] = 3.0
        end_total_progress = track_profile.total_progress_at_line_distance(
            active_line,
            result.distance_m,
        )
        if result.handling_state == "stable":
            if surface.surface_state.value == "kerb_high":
                state.handling_state = "kerb_strike"
            elif surface.surface_state.value == "kerb_low":
                state.handling_state = "kerb"
            elif surface.off_track:
                state.handling_state = "off_track"
        return max(0.0, end_total_progress - state.total_progress)

    def _physics_v2_nominal_lateral_bounds(
        self,
        state: DriverRaceState,
    ) -> tuple[float, float] | None:
        """Keep stable cars body-safe while allowing genuine incidents to run wide."""
        forced_wide = any(
            state.driver_id in (aftermath.attacker_id, aftermath.defender_id)
            for aftermath in self._forced_wide_aftermaths.values()
        )
        if state.avoidance_active or forced_wide:
            return None

        half_length_m = state.car_length_m / 2.0
        half_length_progress = half_length_m / self.track_length_m
        rear_minimum, rear_maximum = (
            self._track_surface.trajectory_body_lateral_bounds(
                state.progress - half_length_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=TRACK_EDGE_MARGIN_M,
            )
        )
        front_minimum, front_maximum = (
            self._track_surface.trajectory_body_lateral_bounds(
                state.progress + half_length_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=TRACK_EDGE_MARGIN_M,
            )
        )
        preview_progress = (
            state.progress
            + max(0.0, state.speed_kph / 3.6) * 0.25 / self.track_length_m
            + half_length_progress
        )
        preview_minimum, preview_maximum = (
            self._track_surface.trajectory_body_lateral_bounds(
                preview_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=TRACK_EDGE_MARGIN_M,
            )
        )
        heading_shift_m = half_length_m * state.slip_angle_rad
        minimum = max(
            rear_minimum + heading_shift_m,
            front_minimum - heading_shift_m,
            preview_minimum - heading_shift_m,
        )
        maximum = min(
            rear_maximum + heading_shift_m,
            front_maximum - heading_shift_m,
            preview_maximum + heading_shift_m,
        )
        if minimum > maximum:
            midpoint = 0.5 * (minimum + maximum)
            return midpoint, midpoint
        return minimum, maximum

    def _local_trajectory_planning_allowed(
        self,
        state: DriverRaceState,
        active_line: str,
    ) -> bool:
        if not self._local_trajectory_basic_planning_allowed(state, active_line):
            return False
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        maneuver_group = self._maneuver_group_for_driver(state.driver_id)
        corner_battle = bool(
            battle is not None
            and battle.phase == "overlap"
            and battle.corner_active
            and active_line == DRIVING_LINE_RACING
        )
        group_shared_occupancy = False
        if maneuver_group is not None and maneuver_group.size >= 3:
            segment = segment_at_progress(self.circuit, state.progress)
            group_shared_occupancy = bool(
                segment is not None
                and segment.side_by_side_allowed
            )
        nearby_states = self._local_trajectory_nearby_states(state)
        close_states = tuple(
            other
            for other in nearby_states
            if abs(other.total_progress - state.total_progress)
            * self.track_length_m
            <= LOCAL_TRAJECTORY_DENSE_TRAFFIC_RADIUS_M
        )
        close_straight_chase = False
        if close_states and not corner_battle and not group_shared_occupancy:
            segment = segment_at_progress(self.circuit, state.progress)
            close_straight_chase = (
                len(close_states) <= LOCAL_TRAJECTORY_MAX_OPPONENTS
                and segment is not None
                and segment.type == TrackSegmentType.STRAIGHT
                and self._segment_overtake_start_allowed(segment)
                and segment.side_by_side_allowed
            )
            if not close_straight_chase:
                return False
        if (
            len(nearby_states) > LOCAL_TRAJECTORY_MAX_OPPONENTS
            and not close_straight_chase
            and not corner_battle
            and not group_shared_occupancy
        ):
            return False
        return True

    def _local_trajectory_basic_planning_allowed(
        self,
        state: DriverRaceState,
        active_line: str,
    ) -> bool:
        """Cheap per-frame eligibility; traffic scans run only at planner cadence."""
        if (
            self.race_phase != "green"
            or not self.race_started
            or state.in_pit
            or state.retired
            or state.finished
            or state.hazard_active
            or state.avoidance_active
            or state.emergency_braking
        ):
            return False
        if self._grid_launch_target_lateral_offset(state, 0.0) is not None:
            return False
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        maneuver_group = self._maneuver_group_for_driver(state.driver_id)
        corner_battle = bool(
            battle is not None
            and battle.phase == "overlap"
            and battle.corner_active
            and active_line == DRIVING_LINE_RACING
        )
        group_shared_occupancy = False
        if maneuver_group is not None and maneuver_group.size >= 3:
            segment = segment_at_progress(self.circuit, state.progress)
            group_shared_occupancy = bool(
                segment is not None
                and segment.side_by_side_allowed
            )
        if battle is not None and not corner_battle and not group_shared_occupancy:
            return False
        if active_line != DRIVING_LINE_RACING:
            return False
        if self._nearest_active_hazard(state) is not None:
            return False
        if any(
            state.driver_id in (aftermath.attacker_id, aftermath.defender_id)
            for aftermath in self._forced_wide_aftermaths.values()
        ):
            return False
        return state.driver_id not in self._battle_effects

    def _local_trajectory_nearby_states(
        self,
        state: DriverRaceState,
    ) -> tuple[DriverRaceState, ...]:
        if self._setup_mode:
            # Unit/scenario setup may reposition cars without advancing a
            # physics frame. Refresh once on demand in that non-runtime mode.
            self._planner_spatial_index.rebuild(self.driver_states.values())
        return tuple(
            self._planner_spatial_index.nearby(
                state,
                LOCAL_TRAJECTORY_OPPONENT_RADIUS_M,
            )
        )

    def _update_local_trajectory_plan(
        self,
        state: DriverRaceState,
        active_line: str,
        modifiers: VehiclePhysicsModifiers | None,
        delta: float,
        *,
        surface: VehicleSurfaceState | None = None,
    ) -> None:
        # The caller performs cheap safety/phase guards every physics frame;
        # the full nearby-traffic scan runs only when a regular replan is due.
        planner = self._local_trajectory_planners.get(state.driver_id)
        if planner is None:
            return
        delta_seconds = max(0.0, delta)
        schedule_nearby_states = self._local_trajectory_nearby_states(state)
        nearest_distance_m = (
            abs(
                schedule_nearby_states[0].total_progress
                - state.total_progress
            )
            * self.track_length_m
            if schedule_nearby_states
            else None
        )
        schedule = self._adaptive_planner_scheduler.choose(
            emergency=(
                state.avoidance_active
                or state.emergency_braking
                or state.local_yellow_active
            ),
            battle=self._side_by_side_battle_for_driver(state.driver_id) is not None,
            nearest_distance_m=nearest_distance_m,
        )
        state.planner_tier_hz = schedule.tier_hz
        state.planner_mode = schedule.mode
        state.planner_nearby_vehicle_count = len(schedule_nearby_states)
        validation_age_seconds = round(
            self._local_trajectory_validation_ages.get(state.driver_id, 0.0)
            + delta_seconds,
            12,
        )
        if validation_age_seconds >= schedule.validation_interval_seconds - 1e-9:
            validation_age_seconds = 0.0
            cached_plan = self._local_trajectory_plans.get(state.driver_id)
            if cached_plan is not None and not cached_plan.selected.viable:
                fallback = self._local_trajectory_last_safe_plans.get(state.driver_id)
                if fallback is not None:
                    self._local_trajectory_plans[state.driver_id] = fallback
                    state.planner_fallback_active = True
        self._local_trajectory_validation_ages[state.driver_id] = (
            validation_age_seconds
        )
        selected_age_seconds = round(
            self._local_trajectory_selected_ages.get(state.driver_id, 0.0)
            + delta_seconds,
            12,
        )
        age_seconds = round(
            self._local_trajectory_plan_ages.get(
                state.driver_id,
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
            )
            + delta_seconds,
            12,
        )
        if age_seconds < schedule.full_plan_interval_seconds - 1e-9:
            self._local_trajectory_plan_ages[state.driver_id] = age_seconds
            self._local_trajectory_selected_ages[
                state.driver_id
            ] = selected_age_seconds
            return
        if not self._local_trajectory_planning_allowed(state, active_line):
            self._local_trajectory_plans.pop(state.driver_id, None)
            self._local_trajectory_plan_ages[state.driver_id] = 0.0
            self._local_trajectory_selected_ages[state.driver_id] = 0.0
            return
        if modifiers is None:
            # Build planning inputs only when the expensive lattice is truly
            # due.  The authoritative controller computes a fresh modifier
            # set afterwards, preserving the established post-plan semantics.
            modifiers = self._physics_v2_modifiers(state, surface)
        previous_plan = self._local_trajectory_plans.get(state.driver_id)
        force_pace_mode_replan = state.driver_id in self._pace_mode_replan_required
        planner_weights = self._interpolated_pace_mode_planner_weights(state)
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        maneuver_group = self._maneuver_group_for_driver(state.driver_id)
        group_shared_occupancy = bool(
            maneuver_group is not None and maneuver_group.size >= 3
        )
        corner_battle = bool(
            battle is not None
            and battle.phase == "overlap"
            and battle.corner_active
            and not group_shared_occupancy
        )
        nearby_states = schedule_nearby_states
        if not corner_battle:
            nearby_states = nearby_states[:LOCAL_TRAJECTORY_MAX_OPPONENTS]
        if corner_battle and battle is not None:
            opponent_id = (
                battle.defender_id
                if state.driver_id == battle.attacker_id
                else battle.attacker_id
            )
            nearby_states = tuple(
                other for other in nearby_states if other.driver_id == opponent_id
            )

        def predicted_target_lateral_offset(other: DriverRaceState) -> float:
            if not corner_battle or battle is None:
                return other.target_lateral_offset_m
            opponent_plan = self._local_trajectory_plans.get(other.driver_id)
            if opponent_plan is not None and opponent_plan.selected.viable:
                return opponent_plan.selected.control_target_lateral_offset_m
            return self._corner_local_role_target(battle, other)

        nearby_vehicles = tuple(
            NearbyVehiclePredictionInput(
                driver_id=other.driver_id,
                total_progress=other.total_progress,
                speed_mps=max(0.0, other.speed_kph / 3.6),
                acceleration_mps2=other.acceleration_mps2,
                lateral_offset_m=other.lateral_offset_m,
                lateral_speed_mps=other.lateral_speed_mps,
                target_lateral_offset_m=predicted_target_lateral_offset(other),
                heading_offset_rad=other.slip_angle_rad,
                body_width_m=other.car_width_m,
                body_length_m=other.car_length_m,
                corner_role=(
                    self._corner_corridor_for_driver(
                        battle,
                        other.driver_id,
                    )
                    if corner_battle and battle is not None
                    else None
                ),
                corner_turn_direction=(
                    battle.corner_turn_direction
                    if corner_battle and battle is not None
                    else 0
                ),
                corner_pair_separation_m=(
                    PHYSICAL_CAR_WIDTH_M + CORNER_CORRIDOR_MARGIN_M
                    if corner_battle and battle is not None
                    else 0.0
                ),
            )
            for other in nearby_states
        )
        corner_role = (
            self._corner_corridor_for_driver(battle, state.driver_id)
            if corner_battle and battle is not None
            else None
        )
        candidate_lateral_biases_m: tuple[float, ...] | None = None
        if nearby_vehicles and corner_role is None:
            racing_offset_m = self._track_physics_for_driver(
                state
            ).line_offset_at_progress(DRIVING_LINE_RACING, state.progress)
            current_corridor_bias_m = state.lateral_offset_m - racing_offset_m
            candidate_count = (
                planner.config.traffic_candidate_count
                if nearest_distance_m is not None and nearest_distance_m <= 45.0
                else planner.config.candidate_count
            )
            candidate_lateral_biases_m = tuple(
                current_corridor_bias_m + bias_m
                for bias_m in planner.lateral_biases_m(
                    candidate_count
                )
            )
        next_plan = planner.plan(
            LocalTrajectoryPlanningRequest(
                total_progress=state.total_progress,
                speed_mps=max(0.0, state.speed_kph / 3.6),
                acceleration_mps2=state.acceleration_mps2,
                lateral_offset_m=state.lateral_offset_m,
                lateral_speed_mps=state.lateral_speed_mps,
                body_width_m=state.car_width_m,
                body_length_m=state.car_length_m,
                modifiers=modifiers,
                line_name=active_line,
                nearby_vehicles=nearby_vehicles,
                tire_wear=state.tire_wear,
                tire_lateral_grip=state.tire_lateral_grip,
                tire_braking_grip=state.tire_braking_grip,
                tire_traction_grip=state.tire_traction_grip,
                weights=planner_weights,
                previous_selected_candidate_id=(
                    previous_plan.selected_candidate_id
                    if previous_plan is not None and not force_pace_mode_replan
                    else None
                ),
                previous_selected_age_seconds=(
                    0.0 if force_pace_mode_replan else selected_age_seconds
                ),
                candidate_lateral_biases_m=(
                    (-0.75, -0.375, 0.0, 0.375, 0.75)
                    if corner_role is not None
                    else candidate_lateral_biases_m
                ),
                corner_role=corner_role,
                corner_turn_direction=(
                    battle.corner_turn_direction
                    if corner_battle and battle is not None
                    else 0
                ),
                corner_pair_separation_m=(
                    PHYSICAL_CAR_WIDTH_M + CORNER_CORRIDOR_MARGIN_M
                    if corner_role is not None
                    else 0.0
                ),
            )
        )
        self._local_trajectory_replan_counts[state.driver_id] = (
            self._local_trajectory_replan_counts.get(state.driver_id, 0) + 1
        )
        state.planner_replan_count = self._local_trajectory_replan_counts[
            state.driver_id
        ]
        state.planner_generation_ms = round(next_plan.generation_duration_ms, 4)
        if next_plan.selected.viable:
            self._local_trajectory_last_safe_plans[state.driver_id] = next_plan
            state.planner_fallback_active = False
        else:
            fallback = self._local_trajectory_last_safe_plans.get(state.driver_id)
            if fallback is not None:
                next_plan = fallback
                state.planner_fallback_active = True
        self._local_trajectory_plans[state.driver_id] = next_plan
        self._local_trajectory_selected_ages[state.driver_id] = (
            selected_age_seconds
            if not force_pace_mode_replan
            and previous_plan is not None
            and previous_plan.selected_candidate_id
            == next_plan.selected_candidate_id
            else 0.0
        )
        self._local_trajectory_plan_ages[state.driver_id] = 0.0
        self._pace_mode_replan_required.discard(state.driver_id)
        if corner_battle and battle is not None:
            if state.driver_id == battle.attacker_id:
                battle.corner_attacker_trajectory_candidate_id = (
                    next_plan.selected_candidate_id
                )
            else:
                battle.corner_defender_trajectory_candidate_id = (
                    next_plan.selected_candidate_id
                )
            battle.corner_trajectory_replan_count += 1

    def _active_stopped_hazards(self) -> list[DriverRaceState]:
        return [
            state
            for state in self.driver_states.values()
            if state.hazard_active and state.vehicle_status == "stopped_on_track"
        ]

    def _stopped_hazard_cluster(
        self,
        seed: DriverRaceState,
    ) -> tuple[DriverRaceState, ...]:
        """Return stopped cars that form one connected physical obstruction."""
        active = self._active_stopped_hazards()
        cluster = {seed.driver_id}
        changed = True
        while changed:
            changed = False
            for candidate in active:
                if candidate.driver_id in cluster:
                    continue
                if any(
                    abs(candidate.total_progress - member.total_progress)
                    * self.track_length_m
                    <= HAZARD_CLUSTER_LONGITUDINAL_GAP_M
                    for member in active
                    if member.driver_id in cluster
                ):
                    cluster.add(candidate.driver_id)
                    changed = True
        return tuple(
            sorted(
                (
                    hazard
                    for hazard in active
                    if hazard.driver_id in cluster
                ),
                key=lambda hazard: (hazard.total_progress, hazard.driver_id),
            )
        )

    def _nearest_active_hazard_cluster(
        self,
        state: DriverRaceState,
        maximum_distance_m: float = HAZARD_DETECTION_DISTANCE_M,
    ) -> tuple[tuple[DriverRaceState, ...], DriverRaceState, float] | None:
        nearest = self._nearest_active_hazard(state, maximum_distance_m)
        if nearest is None:
            return None
        nearest_hazard, distance_m = nearest
        return (
            self._stopped_hazard_cluster(nearest_hazard),
            nearest_hazard,
            distance_m,
        )

    @staticmethod
    def _hazard_cluster_anchor(
        hazards: tuple[DriverRaceState, ...],
    ) -> DriverRaceState:
        return min(hazards, key=lambda hazard: hazard.driver_id)

    def _hazard_cluster_has_passable_corridor(
        self,
        hazards: tuple[DriverRaceState, ...],
    ) -> bool:
        """Check whether a full-width car center can pass either outer side."""
        if not hazards:
            return True
        car_width_m = PHYSICAL_CAR_WIDTH_M
        left_target_m = max(
            hazard.lateral_offset_m
            + 0.5 * (car_width_m + hazard.car_width_m)
            + HAZARD_LATERAL_MARGIN_M
            + HAZARD_CORRIDOR_TARGET_MARGIN_M
            for hazard in hazards
        )
        right_target_m = min(
            hazard.lateral_offset_m
            - 0.5 * (car_width_m + hazard.car_width_m)
            - HAZARD_LATERAL_MARGIN_M
            - HAZARD_CORRIDOR_TARGET_MARGIN_M
            for hazard in hazards
        )
        left_limit_m = min(
            self._track_physics.at_progress(hazard.progress).left_width_m
            - car_width_m / 2.0
            - TRACK_EDGE_MARGIN_M
            for hazard in hazards
        )
        right_limit_m = max(
            -self._track_physics.at_progress(hazard.progress).right_width_m
            + car_width_m / 2.0
            + TRACK_EDGE_MARGIN_M
            for hazard in hazards
        )
        candidate_targets_m = [left_target_m, right_target_m]
        ordered = sorted(hazards, key=lambda hazard: hazard.lateral_offset_m)
        for right_hazard, left_hazard in zip(ordered, ordered[1:]):
            right_edge_m = (
                right_hazard.lateral_offset_m
                + 0.5 * (car_width_m + right_hazard.car_width_m)
                + HAZARD_LATERAL_MARGIN_M
                + HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            left_edge_m = (
                left_hazard.lateral_offset_m
                - 0.5 * (car_width_m + left_hazard.car_width_m)
                - HAZARD_LATERAL_MARGIN_M
                - HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            if right_edge_m <= left_edge_m:
                candidate_targets_m.append(0.5 * (right_edge_m + left_edge_m))
        return any(
            right_limit_m <= target_m <= left_limit_m
            for target_m in candidate_targets_m
        )

    def _activate_stopped_hazard(
        self,
        state: DriverRaceState,
        cause: IncidentCause,
    ) -> None:
        state.vehicle_status = "stopped_on_track"
        state.hazard_active = True
        state.hazard_cause = cause.value
        self._hazard_activated_at[state.driver_id] = self.race_elapsed
        state.speed_kph = 0.0
        state.acceleration_mps2 = 0.0
        state.target_speed_kph = 0.0
        state.throttle = 0.0
        state.brake = 1.0
        state.lateral_speed_mps = 0.0
        self._progress_rate[state.driver_id] = 0.0

        running = [
            candidate
            for candidate in self.driver_states.values()
            if not candidate.retired
            and not candidate.finished
            and not candidate.in_pit
            and candidate.driver_id != state.driver_id
        ]
        if not running:
            self._hazard_tail_targets.pop(state.driver_id, None)
            return
        tail = max(running, key=lambda candidate: candidate.position)
        pass_target = self._total_progress_at_or_after(
            tail.total_progress,
            state.progress,
        ) + HAZARD_PASS_CLEARANCE_M / self.track_length_m
        self._hazard_tail_targets[state.driver_id] = (
            tail.driver_id,
            pass_target,
        )

    def _nearest_active_hazard(
        self,
        state: DriverRaceState,
        maximum_distance_m: float = HAZARD_DETECTION_DISTANCE_M,
    ) -> tuple[DriverRaceState, float] | None:
        nearest: tuple[DriverRaceState, float] | None = None
        for hazard in self._active_stopped_hazards():
            if hazard.driver_id == state.driver_id:
                continue
            distance_m = (
                hazard.total_progress - state.total_progress
            ) * self.track_length_m
            if distance_m <= 0.0 or distance_m > maximum_distance_m:
                continue
            if nearest is None or distance_m < nearest[1]:
                nearest = hazard, distance_m
        return nearest

    def _local_yellow_active_for(self, state: DriverRaceState) -> bool:
        """Return whether a driver is approaching an active 300m hazard zone."""
        return bool(
            self.race_phase == "green"
            and not state.in_pit
            and not state.retired
            and not state.finished
            and self._nearest_active_hazard(state) is not None
        )

    def _overtaking_candidate_allowed(
        self,
        attacker: DriverRaceState,
        defender: DriverRaceState | None = None,
    ) -> bool:
        """Central race-control gate for competitive overtake candidates."""
        if not self.race_started or self.race_phase != "green":
            return False
        checked = (attacker,) if defender is None else (attacker, defender)
        return all(
            not state.in_pit
            and not state.retired
            and not state.finished
            and not state.hazard_active
            and not state.avoidance_active
            and not state.emergency_braking
            and not self._local_yellow_active_for(state)
            for state in checked
        )

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

    def _reset_avoidance_state(self, state: DriverRaceState) -> None:
        state.avoidance_active = False
        state.avoidance_ttc_seconds = 0.0
        state.emergency_braking = False

    def _hazard_required_lateral_clearance_m(
        self,
        state: DriverRaceState,
        hazard: DriverRaceState,
    ) -> float:
        return (
            0.5 * (state.car_width_m + hazard.car_width_m)
            + HAZARD_LATERAL_MARGIN_M
        )

    def _hazard_preferred_avoidance_side(
        self,
        state: DriverRaceState,
        hazard: DriverRaceState,
    ) -> str:
        """Reserve alternating escape corridors for an approaching field.

        A stopped car is detected by several cars at nearly the same time.  A
        driver-id preference lets the whole queue choose the geometrically
        shortest side and creates a single-file blockage.  Ordering cars by
        their live distance to this hazard keeps the assignment deterministic
        while spreading the queue across both usable corridors.
        """
        approaching: list[tuple[float, int]] = []
        for other in self.driver_states.values():
            if (
                other.driver_id == hazard.driver_id
                or other.retired
                or other.finished
                or other.in_pit
            ):
                continue
            distance_m = (
                hazard.total_progress - other.total_progress
            ) * self.track_length_m
            if 0.0 < distance_m <= HAZARD_DETECTION_DISTANCE_M:
                approaching.append((distance_m, other.driver_id))
        approaching.sort(key=lambda item: (item[0], item[1]))
        rank = next(
            (
                index
                for index, (_, driver_id) in enumerate(approaching)
                if driver_id == state.driver_id
            ),
            0,
        )
        return "left" if (rank + hazard.driver_id) % 2 == 0 else "right"

    def _hazard_avoidance_target(
        self,
        state: DriverRaceState,
        track_sample,
        base_target: float,
    ) -> float | None:
        nearest_cluster = self._nearest_active_hazard_cluster(state)
        if nearest_cluster is None:
            state.avoidance_hazard_driver_id = None
            state.avoidance_side = ""
            state.avoidance_target_lateral_offset_m = base_target
            return None
        hazards, hazard, distance_m = nearest_cluster
        cluster_anchor = self._hazard_cluster_anchor(hazards)
        cluster_driver_ids = {item.driver_id for item in hazards}
        previous_hazard_id = state.avoidance_hazard_driver_id
        previous_side = state.avoidance_side
        required_clearances = {
            item.driver_id: self._hazard_required_lateral_clearance_m(
                state,
                item,
            )
            for item in hazards
        }
        required_clearance = required_clearances[hazard.driver_id]
        closest_bumper_gap_m = min(
            max(
                0.0,
                (item.total_progress - state.total_progress)
                * self.track_length_m
                - 0.5 * (state.car_length_m + item.car_length_m),
            )
            for item in hazards
            if item.total_progress > state.total_progress
        )
        closing_speed_mps = max(0.1, state.speed_kph / 3.6)
        state.avoidance_hazard_driver_id = cluster_anchor.driver_id
        state.avoidance_ttc_seconds = round(
            min(99.0, closest_bumper_gap_m / closing_speed_mps),
            3,
        )

        positive_limit = (
            track_sample.left_width_m
            - state.car_width_m / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        negative_limit = (
            track_sample.right_width_m
            - state.car_width_m / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        minimum_target = -negative_limit
        maximum_target = positive_limit
        if minimum_target > maximum_target:
            center_target = 0.5 * (minimum_target + maximum_target)
            minimum_target = center_target
            maximum_target = center_target
        base_target = max(minimum_target, min(maximum_target, base_target))
        if all(
            abs(base_target - item.lateral_offset_m)
            >= required_clearances[item.driver_id]
            for item in hazards
        ):
            state.avoidance_active = True
            state.avoidance_side = (
                "left"
                if base_target
                > max(item.lateral_offset_m for item in hazards)
                else "right"
            )
            state.avoidance_target_lateral_offset_m = base_target
            return base_target

        raw_candidates = [
            (
                "left",
                max(
                    item.lateral_offset_m
                    + required_clearances[item.driver_id]
                    for item in hazards
                )
                + HAZARD_CORRIDOR_TARGET_MARGIN_M,
            ),
            (
                "right",
                min(
                    item.lateral_offset_m
                    - required_clearances[item.driver_id]
                    for item in hazards
                )
                - HAZARD_CORRIDOR_TARGET_MARGIN_M,
            ),
        ]
        preferred_side = self._hazard_preferred_avoidance_side(
            state,
            cluster_anchor,
        )
        ordered_hazards = sorted(
            hazards,
            key=lambda item: item.lateral_offset_m,
        )
        cluster_center_m = sum(
            item.lateral_offset_m for item in hazards
        ) / len(hazards)
        for right_hazard, left_hazard in zip(
            ordered_hazards,
            ordered_hazards[1:],
        ):
            right_edge_m = (
                right_hazard.lateral_offset_m
                + required_clearances[right_hazard.driver_id]
                + HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            left_edge_m = (
                left_hazard.lateral_offset_m
                - required_clearances[left_hazard.driver_id]
                - HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            if right_edge_m > left_edge_m:
                continue
            middle_target_m = 0.5 * (right_edge_m + left_edge_m)
            raw_candidates.append(
                (
                    "left"
                    if middle_target_m > cluster_center_m
                    else (
                        "right"
                        if middle_target_m < cluster_center_m
                        else preferred_side
                    ),
                    middle_target_m,
                )
            )
        candidates: list[tuple[float, str, float, float]] = []
        for side, raw_target in raw_candidates:
            target = max(minimum_target, min(maximum_target, raw_target))
            if any(
                abs(target - item.lateral_offset_m)
                < required_clearances[item.driver_id] - 1e-6
                for item in hazards
            ):
                continue
            traffic_penalty = 0.0
            for other in self.driver_states.values():
                if (
                    other.driver_id == state.driver_id
                    or other.driver_id in cluster_driver_ids
                    or (other.retired and not other.hazard_active)
                    or other.finished
                    or other.in_pit
                ):
                    continue
                longitudinal_gap_m = abs(
                    other.total_progress - hazard.total_progress
                ) * self.track_length_m
                if longitudinal_gap_m > HAZARD_TRAFFIC_WINDOW_M:
                    continue
                other_clearance = 0.5 * (
                    state.car_width_m + other.car_width_m
                ) + MANEUVER_CLEARANCE_MARGIN_M
                lateral_gap_m = abs(target - other.lateral_offset_m)
                traffic_penalty += max(
                    0.0,
                    other_clearance - lateral_gap_m,
                ) * 100.0
            corridor_preference_penalty = (
                0.0
                if side == preferred_side
                else HAZARD_CORRIDOR_PREFERENCE_PENALTY
            )
            score = (
                abs(target - base_target)
                + 0.35 * abs(target - state.lateral_offset_m)
                + traffic_penalty
                + corridor_preference_penalty
            )
            candidates.append((score, side, target, traffic_penalty))

        state.avoidance_active = True
        if not candidates:
            state.avoidance_side = "blocked"
            state.avoidance_target_lateral_offset_m = base_target
            state.emergency_braking = True
            return base_target

        stable_candidate = next(
            (
                candidate
                for candidate in candidates
                if previous_hazard_id == cluster_anchor.driver_id
                and candidate[1] == previous_side
            ),
            None,
        )
        _, side, target, traffic_penalty = stable_candidate or min(
            candidates,
            key=lambda candidate: candidate[0],
        )
        state.avoidance_side = side
        state.avoidance_target_lateral_offset_m = target
        return target

    def _hazard_following_constraint(
        self,
        state: DriverRaceState,
        active_line: str,
    ) -> VehicleFollowingConstraint | None:
        track_profile = self._track_physics_for_driver(state)
        nearest_cluster = self._nearest_active_hazard_cluster(state)
        if nearest_cluster is None:
            return None
        hazards, hazard, distance_m = nearest_cluster
        required_clearances = {
            item.driver_id: self._hazard_required_lateral_clearance_m(
                state,
                item,
            )
            for item in hazards
        }
        lateral_separations_m = {
            item.driver_id: abs(
                state.lateral_offset_m - item.lateral_offset_m
            )
            for item in hazards
        }
        if all(
            lateral_separations_m[item.driver_id]
            >= required_clearances[item.driver_id]
            for item in hazards
        ):
            return None

        follower_speed_mps = max(0.0, state.speed_kph / 3.6)
        half_lengths_m = 0.5 * (state.car_length_m + hazard.car_length_m)
        minimum_gap_m = half_lengths_m + 0.35
        has_escape_corridor = (
            state.avoidance_active
            and state.avoidance_side in {"left", "right"}
            and all(
                abs(
                    state.avoidance_target_lateral_offset_m
                    - item.lateral_offset_m
                )
                >= required_clearances[item.driver_id] - 1e-6
                for item in hazards
            )
        )
        if has_escape_corridor:
            moving_away_from_hazard = (
                (
                    state.avoidance_target_lateral_offset_m
                    - state.lateral_offset_m
                )
                * state.lateral_speed_mps
                > 0.0
            )
            bodies_clear = all(
                lateral_separations_m[item.driver_id]
                >= (
                    0.5 * (state.car_width_m + item.car_width_m)
                    + HAZARD_MOVING_RELEASE_MARGIN_M
                )
                for item in hazards
            )
            if moving_away_from_hazard and bodies_clear:
                # Once the two physical bodies are clear and lateral velocity
                # is still outward, holding the car at the obstacle for one
                # more 100 ms race tick creates a visible false stop.
                return None
            # The stationary-car constraint remains as a final body-overlap
            # guard, but it must not ask every evading car to stop at the full
            # longitudinal braking distance.  The lateral controller is
            # already moving the car into a clear corridor; only lower the
            # speed when the remaining time is genuinely tight.
            lateral_remaining_m = max(
                (
                    max(
                        0.0,
                        required_clearances[item.driver_id]
                        - lateral_separations_m[item.driver_id],
                    )
                    for item in hazards
                ),
                default=0.0,
            )
            lateral_clearance_seconds = sqrt(
                2.0
                * lateral_remaining_m
                / HAZARD_EVASIVE_LATERAL_ACCELERATION_MPS2
            )
            bumper_gap_m = max(0.0, distance_m - half_lengths_m)
            time_to_bumper_seconds = bumper_gap_m / max(0.1, follower_speed_mps)
            if (
                time_to_bumper_seconds
                <= lateral_clearance_seconds + HAZARD_EVASIVE_TIME_BUFFER_SECONDS
            ):
                state.emergency_braking = True
            desired_gap_m = minimum_gap_m + 0.5
            hazard_distance_m = track_profile.line_distance_at_total_progress(
                active_line,
                hazard.total_progress,
            )
            return VehicleFollowingConstraint(
                leader_distance_m=hazard_distance_m,
                leader_speed_mps=0.0,
                leader_end_distance_m=hazard_distance_m,
                leader_end_speed_mps=0.0,
                desired_gap_m=desired_gap_m,
                minimum_gap_m=minimum_gap_m,
            )

        stopping_gap_m = (
            minimum_gap_m
            + follower_speed_mps * HAZARD_REACTION_SECONDS
            + follower_speed_mps * follower_speed_mps
            / (2.0 * HAZARD_BRAKING_DECELERATION_MPS2)
        )
        desired_gap_m = min(
            HAZARD_DETECTION_DISTANCE_M,
            max(minimum_gap_m + 2.0, stopping_gap_m),
        )
        if distance_m <= desired_gap_m:
            state.emergency_braking = True
        hazard_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            hazard.total_progress,
        )
        return VehicleFollowingConstraint(
            leader_distance_m=hazard_distance_m,
            leader_speed_mps=0.0,
            leader_end_distance_m=hazard_distance_m,
            leader_end_speed_mps=0.0,
            desired_gap_m=desired_gap_m,
            minimum_gap_m=minimum_gap_m,
        )

    def _hazard_corridor_following_constraint(
        self,
        state: DriverRaceState,
        active_line: str,
        start_snapshot: dict[int, tuple[float, float]],
    ) -> VehicleFollowingConstraint | None:
        """Follow the nearest moving car assigned to the same escape corridor."""
        track_profile = self._track_physics_for_driver(state)
        if (
            not state.avoidance_active
            or state.avoidance_hazard_driver_id is None
            or state.avoidance_side not in {"left", "right"}
        ):
            return None
        same_corridor = [
            candidate
            for candidate in self.driver_states.values()
            if candidate.driver_id != state.driver_id
            and not candidate.retired
            and not candidate.finished
            and not candidate.in_pit
            and candidate.avoidance_active
            and candidate.avoidance_hazard_driver_id
            == state.avoidance_hazard_driver_id
            and candidate.avoidance_side == state.avoidance_side
            and candidate.total_progress > state.total_progress
        ]
        if not same_corridor:
            return None
        leader = min(
            same_corridor,
            key=lambda candidate: candidate.total_progress - state.total_progress,
        )
        leader_snapshot = start_snapshot.get(leader.driver_id)
        follower_snapshot = start_snapshot.get(state.driver_id)
        if leader_snapshot is None or follower_snapshot is None:
            return None
        leader_total_progress, leader_speed_mps = leader_snapshot
        follower_total_progress, follower_speed_mps = follower_snapshot
        if leader_total_progress <= follower_total_progress + PROGRESS_EPSILON:
            return None
        half_lengths_m = 0.5 * (state.car_length_m + leader.car_length_m)
        minimum_gap_m = half_lengths_m + 0.35
        desired_gap_m = min(
            18.0,
            max(8.0, minimum_gap_m + max(0.0, follower_speed_mps) * 0.12),
        )
        return VehicleFollowingConstraint(
            leader_distance_m=track_profile.line_distance_at_total_progress(
                active_line,
                leader_total_progress,
            ),
            leader_speed_mps=max(0.0, leader_speed_mps),
            leader_end_distance_m=track_profile.line_distance_at_total_progress(
                active_line,
                leader.total_progress,
            ),
            leader_end_speed_mps=max(0.0, leader.speed_kph / 3.6),
            desired_gap_m=desired_gap_m,
            minimum_gap_m=minimum_gap_m,
        )

    def _physics_v2_virtual_line(self, state: DriverRaceState) -> str:
        maneuver_group = self._maneuver_group_for_driver(state.driver_id)
        if maneuver_group is not None and maneuver_group.size >= 3:
            # The local occupancy planner owns all distinct group corridors;
            # mapping a shared member back to one pair's template would pull
            # the middle car across another member.
            return DRIVING_LINE_RACING
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if battle is not None:
            if battle.phase == "overlap" and battle.corner_active:
                return DRIVING_LINE_RACING
            if battle.phase in {"approach", "merge", "yield", "abort"}:
                return DRIVING_LINE_RACING
            if state.driver_id == battle.attacker_id:
                return battle.attacker_line
            return battle.defender_line

        pending_overtake = self._pending_overtake_commands.get(state.driver_id)
        if pending_overtake is not None:
            return pending_overtake.decision.attacker_line

        for aftermath in self._forced_wide_aftermaths.values():
            if state.driver_id == aftermath.attacker_id:
                return ATTACK_LINE_INSIDE
            if state.driver_id == aftermath.defender_id:
                return ATTACK_LINE_OUTSIDE

        effect = self._battle_effects.get(state.driver_id)
        if effect is not None and effect.lap_time_delta < 0:
            return ATTACK_LINE_INSIDE
        if state.driver_id in self._sc_unlap_driver_ids:
            return ATTACK_LINE_OUTSIDE
        return DEFENDER_LINE_RACING

    def _physics_v2_target_lateral_offset(
        self,
        state: DriverRaceState,
        track_sample,
        line: str,
        car_ahead: DriverRaceState | None,
        car_behind: DriverRaceState | None,
    ) -> float:
        track_profile = self._track_physics_for_driver(state)
        base = track_profile.line_offset_at_progress(line, state.progress)
        launch_target = self._grid_launch_target_lateral_offset(state, base)
        if launch_target is not None:
            base = launch_target
        hazard_target = self._hazard_avoidance_target(
            state,
            track_sample,
            base,
        )
        if hazard_target is not None:
            return hazard_target
        if self.race_phase != "green" and state.driver_id not in self._sc_unlap_driver_ids:
            return track_profile.line_offset_at_progress(
                DRIVING_LINE_RACING,
                state.progress,
            )
        if launch_target is not None:
            # Grid columns are already two valid physical lanes. Preserve them
            # while start battles are evaluated instead of making every car
            # merge to the same racing line before an attack can exist.
            return launch_target
        pending_overtake = self._pending_overtake_commands.get(state.driver_id)
        if pending_overtake is not None:
            base = (
                track_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
                + pending_overtake.decision.lateral_bias_m
            )
        maneuver_group = self._maneuver_group_for_driver(state.driver_id)
        battle = (
            None
            if maneuver_group is not None and maneuver_group.size >= 3
            else self._side_by_side_battle_for_driver(state.driver_id)
        )
        if battle is not None and battle.phase in {"yield", "abort"}:
            # A rejected corner attempt must separate longitudinally before
            # either car crosses back toward the racing line.  Freezing the
            # live corridors removes the one-frame target jump that looked
            # like the rear car was ejected at corner entry.
            if state.driver_id == battle.attacker_id:
                transition_start_m = (
                    battle.yield_attacker_lateral_offset_m
                    if battle.phase == "yield"
                    else battle.abort_attacker_lateral_offset_m
                )
            else:
                transition_start_m = (
                    battle.yield_defender_lateral_offset_m
                    if battle.phase == "yield"
                    else battle.abort_defender_lateral_offset_m
                )
            target = transition_start_m
            if battle.phase == "abort" and battle.abort_rejoin_started:
                transition_ratio = min(
                    1.0,
                    battle.abort_rejoin_elapsed_seconds
                    / MANEUVER_ABORT_REJOIN_SECONDS,
                )
                smooth_ratio = transition_ratio * transition_ratio * (
                    3.0 - 2.0 * transition_ratio
                )
                target = transition_start_m + (
                    base - transition_start_m
                ) * smooth_ratio
            minimum_lateral, maximum_lateral = (
                self._track_surface.safety_lateral_bounds(state.progress)
            )
            return max(minimum_lateral, min(maximum_lateral, target))
        if (
            battle is not None
            and battle.line_committed
            and battle.phase in {"pull_out", "overlap"}
            and not battle.corner_active
        ):
            # Once deceleration is imminent neither driver may make a second
            # reactive move.  Follow the curvature of the optimized reference
            # line while retaining the corridor chosen before braking.
            committed_bias_m = (
                battle.attacker_committed_lateral_bias_m
                if state.driver_id == battle.attacker_id
                else battle.defender_committed_lateral_bias_m
            )
            target = (
                track_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
                + committed_bias_m
            )
            minimum_lateral, maximum_lateral = (
                self._track_surface.trajectory_body_lateral_bounds(
                    state.progress,
                    body_width_m=state.car_width_m,
                    edge_margin_m=TRACK_EDGE_MARGIN_M,
                )
            )
            return max(minimum_lateral, min(maximum_lateral, target))
        if (
            battle is not None
            and battle.phase == "overlap"
            and battle.corner_active
        ):
            corner_plan = self._local_trajectory_plans.get(state.driver_id)
            if corner_plan is not None and corner_plan.selected.viable:
                return corner_plan.selected.control_target_lateral_offset_m
            corridor_targets = self._corner_corridor_targets(battle, state.progress)
            if corridor_targets is not None:
                return corridor_targets[state.driver_id]
        local_plan = self._local_trajectory_plans.get(state.driver_id)
        if (
            battle is not None
            and state.driver_id == battle.attacker_id
            and battle.phase == "approach"
        ):
            bumper_gap_m = self._bumper_gap_m(
                state,
                self.driver_states[battle.defender_id],
            )
            if (
                battle.trajectory_authorized
                and bumper_gap_m <= MANEUVER_PULL_OUT_BUMPER_GAP_M
            ):
                base = track_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                ) + battle.trajectory_lateral_bias_m
            elif bumper_gap_m <= MANEUVER_ATTACK_DECISION_BUMPER_GAP_M:
                # A tactical intent may be created between the slower lattice
                # planner updates.  Use the already scored physical template
                # rather than leaving the attacker trapped on the defender's
                # racing line until the approach timer expires.
                base = track_profile.line_offset_at_progress(
                    battle.attacker_line,
                    state.progress,
                )
            else:
                base = track_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
        elif (
            battle is not None
            and battle.trajectory_authorized
            and state.driver_id == battle.attacker_id
            and battle.phase == "pull_out"
        ):
            base = track_profile.line_offset_at_progress(
                DRIVING_LINE_RACING,
                state.progress,
            ) + battle.trajectory_lateral_bias_m
        if (
            battle is not None
            and state.driver_id == battle.attacker_id
            and battle.phase in {"pull_out", "overlap"}
        ):
            defender = self.driver_states[battle.defender_id]
            live_separation_m = (
                state.lateral_offset_m - defender.lateral_offset_m
            )
            if abs(live_separation_m) >= 0.5 * PHYSICAL_CAR_WIDTH_M:
                maneuver_side = 1.0 if live_separation_m >= 0.0 else -1.0
            else:
                maneuver_side = (
                    1.0
                    if base >= defender.lateral_offset_m
                    else -1.0
                )
            live_clearance_target = (
                defender.lateral_offset_m
                + maneuver_side
                * (
                    PHYSICAL_CAR_WIDTH_M
                    + MANEUVER_CLEARANCE_MARGIN_M
                    + MANEUVER_LIVE_CLEARANCE_BUFFER_M
                )
            )
            if abs(base - defender.lateral_offset_m) < abs(
                live_clearance_target - defender.lateral_offset_m
            ):
                base = live_clearance_target
        elif (
            line == DRIVING_LINE_RACING
            and local_plan is not None
            and local_plan.selected.viable
            and local_plan.opponent_occupancies
            and (
                local_plan.selected.minimum_opponent_clearance_m
                >= LOCAL_TRAJECTORY_MIN_SELECTED_CLEARANCE_M
            )
        ):
            base = local_plan.selected.control_target_lateral_offset_m
        positive_limit = (
            track_sample.left_width_m
            - PHYSICAL_CAR_WIDTH_M / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        negative_limit = (
            track_sample.right_width_m
            - PHYSICAL_CAR_WIDTH_M / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        if line == DRIVING_LINE_RACING and abs(track_sample.turn_signal) >= 0.08:
            positive_limit += RACING_LINE_KERB_ALLOWANCE_M
            negative_limit += RACING_LINE_KERB_ALLOWANCE_M

        target = max(-negative_limit, min(positive_limit, base))
        if (
            self.circuit.track_width_m <= 11.5
            and battle is None
            and pending_overtake is None
            and not state.avoidance_active
            and (
                local_plan is None
                or not local_plan.opponent_occupancies
            )
        ):
            # Satellite-derived narrow-track boundaries leave little reserve
            # for trail-braking understeer.  A clean reference lap keeps a
            # two-metre dynamic recovery corridor; battles and avoidance are
            # still allowed to use the full legal body envelope above.
            recovery_margin_m = 2.0
            target = max(
                -(max(0.0, negative_limit - recovery_margin_m)),
                min(
                    max(0.0, positive_limit - recovery_margin_m),
                    target,
                ),
            )
        if (
            battle is not None
            and battle.phase == "overlap"
            and not battle.corner_active
        ):
            attacker = self.driver_states[battle.attacker_id]
            defender = self.driver_states[battle.defender_id]
            attacker_side_delta = (
                attacker.lateral_offset_m - defender.lateral_offset_m
            )
            if abs(attacker_side_delta) >= 0.5 * PHYSICAL_CAR_WIDTH_M:
                attacker_side = 1.0 if attacker_side_delta > 0.0 else -1.0
            else:
                attacker_side = (
                    1.0
                    if battle.attacker_line == ATTACK_LINE_INSIDE
                    else -1.0
                )
            required_separation = (
                PHYSICAL_CAR_WIDTH_M
                + MANEUVER_CLEARANCE_MARGIN_M
                + MANEUVER_LIVE_CLEARANCE_BUFFER_M
            )
            if state.driver_id == battle.attacker_id:
                target = defender.lateral_offset_m + attacker_side * required_separation
            else:
                target = attacker.lateral_offset_m - attacker_side * required_separation
            target = max(-negative_limit, min(positive_limit, target))
            return target
        opponent = None
        gap_m = float("inf")
        if line in (DRIVING_LINE_INSIDE, DRIVING_LINE_OUTSIDE) and car_ahead is not None:
            opponent = car_ahead
            gap_m = (car_ahead.total_progress - state.total_progress) * self.track_length_m
        elif line == DRIVING_LINE_DEFENSIVE and car_behind is not None:
            opponent = car_behind
            gap_m = (state.total_progress - car_behind.total_progress) * self.track_length_m
        if opponent is None or gap_m <= 0.0 or gap_m > 100.0:
            return target

        predicted_opponent_offset = (
            opponent.lateral_offset_m + opponent.lateral_speed_mps * 0.8
        )
        minimum_clearance = PHYSICAL_CAR_WIDTH_M + 0.35
        candidates = [
            max(-negative_limit, min(positive_limit, candidate))
            for candidate in (
                target - 1.8,
                target - 0.9,
                target - 0.45,
                target,
                target + 0.45,
                target + 0.9,
                target + 1.8,
                predicted_opponent_offset - minimum_clearance - 0.15,
                predicted_opponent_offset + minimum_clearance + 0.15,
            )
        ]

        def candidate_score(candidate: float) -> float:
            clearance = abs(candidate - predicted_opponent_offset)
            collision_penalty = max(0.0, minimum_clearance - clearance) * 30.0
            clearance_bonus = min(1.0, max(0.0, clearance - minimum_clearance)) * 0.2
            return (
                -0.35 * abs(candidate - target)
                -0.05 * abs(candidate - state.lateral_offset_m)
                - collision_penalty
                + clearance_bonus
            )

        return max(candidates, key=candidate_score)

    def _physics_v2_target_lateral_speed(
        self,
        state: DriverRaceState,
        line: str,
        target_lateral_offset_m: float,
    ) -> float:
        """Feed the path's lateral motion forward instead of chasing it late."""
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if (
            state.avoidance_active
            or battle is not None
            or state.driver_id in self._pending_overtake_commands
        ):
            lateral_response_seconds = (
                0.80
                if state.driver_id in self._pending_overtake_commands
                else 0.80
            )
            return max(
                -PHYSICS_MAX_LATERAL_SPEED_MPS,
                min(
                    PHYSICS_MAX_LATERAL_SPEED_MPS,
                    (
                        target_lateral_offset_m - state.lateral_offset_m
                    ) / lateral_response_seconds,
                ),
            )

        local_plan = self._local_trajectory_plans.get(state.driver_id)
        if (
            local_plan is not None
            and local_plan.selected.viable
            and local_plan.opponent_occupancies
            and local_plan.selected.minimum_opponent_clearance_m
            >= LOCAL_TRAJECTORY_MIN_SELECTED_CLEARANCE_M
        ):
            # Planner lateral speed belongs to the selected avoidance path.
            # A traffic-free plan is only a path prior; its one-second-old
            # control sample must not override the live racing-line target.
            clean_line_lateral_speed_limit_mps = PHYSICS_MAX_LATERAL_SPEED_MPS
            return max(
                -clean_line_lateral_speed_limit_mps,
                min(
                    clean_line_lateral_speed_limit_mps,
                    local_plan.selected.control_target_lateral_speed_mps,
                ),
            )

        # The active line pose already curves through the future racing-line
        # offsets.  Feeding the offset derivative in again commands a second
        # lateral motion in the line-relative bicycle frame and used to drive
        # clean cars into +/-4 m/s oscillations.  With no traffic manoeuvre the
        # controller only needs to remove local offset error.
        return 0.0

    def _grid_launch_distance_m(self, state: DriverRaceState) -> float:
        start_progress = self._grid_start_progress.get(state.driver_id)
        if start_progress is None:
            return float("inf")
        return max(0.0, (state.total_progress - start_progress) * self.track_length_m)

    def _grid_launch_target_lateral_offset(
        self,
        state: DriverRaceState,
        racing_line_offset_m: float,
    ) -> float | None:
        if not self.start_sequence_enabled or not self.race_started:
            return None
        launch_distance_m = self._grid_launch_distance_m(state)
        if launch_distance_m >= GRID_LAUNCH_MERGE_DISTANCE_M:
            return None
        grid_offset_m = self._grid_lateral_offsets.get(state.driver_id)
        if grid_offset_m is None:
            return None
        if launch_distance_m <= GRID_LAUNCH_LANE_HOLD_M:
            return grid_offset_m
        merge_span_m = max(
            1.0,
            GRID_LAUNCH_MERGE_DISTANCE_M - GRID_LAUNCH_LANE_HOLD_M,
        )
        ratio = min(
            1.0,
            max(0.0, (launch_distance_m - GRID_LAUNCH_LANE_HOLD_M) / merge_span_m),
        )
        smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        return grid_offset_m + (racing_line_offset_m - grid_offset_m) * smooth_ratio

    def _grid_launch_pair_is_physically_separate(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
    ) -> bool:
        if not self.start_sequence_enabled or not self.race_started:
            return False
        if (
            self._grid_launch_distance_m(first) >= GRID_LAUNCH_MERGE_DISTANCE_M
            or self._grid_launch_distance_m(second) >= GRID_LAUNCH_MERGE_DISTANCE_M
        ):
            return False
        return abs(first.lateral_offset_m - second.lateral_offset_m) >= (
            0.5 * (first.car_width_m + second.car_width_m)
            + MANEUVER_CLEARANCE_MARGIN_M
        )

    def _center_gap_m(
        self,
        follower: DriverRaceState,
        leader: DriverRaceState,
    ) -> float:
        return (leader.total_progress - follower.total_progress) * self.track_length_m

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
        if (
            self.start_sequence_enabled
            and self.race_started
            and self._grid_launch_distance_m(state) < GRID_LAUNCH_MERGE_DISTANCE_M
            and self._grid_launch_distance_m(car_ahead) < GRID_LAUNCH_MERGE_DISTANCE_M
            and lateral_clearance
            >= PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
        ):
            return True
        battle = self._pair_maneuver(state, car_ahead)
        if (
            battle is not None
            and battle.phase == "overlap"
            and lateral_clearance
            >= PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
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
        return lateral_clearance >= PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M

    def _nearest_physical_following_leader(
        self,
        state: DriverRaceState,
        start_snapshot: dict[int, tuple[float, float]],
        preferred_leader: DriverRaceState | None,
    ) -> DriverRaceState | None:
        """Find the nearest body in the projected lane, independent of timing order."""
        follower_snapshot = start_snapshot.get(state.driver_id)
        if follower_snapshot is None:
            return preferred_leader
        follower_progress, _ = follower_snapshot
        candidates: list[tuple[float, DriverRaceState]] = []
        for candidate in self.driver_states.values():
            if (
                candidate.driver_id == state.driver_id
                or candidate.in_pit
                or candidate.retired
                or candidate.finished
            ):
                continue
            leader_snapshot = start_snapshot.get(candidate.driver_id)
            if leader_snapshot is None:
                continue
            progress_gap = leader_snapshot[0] - follower_progress
            if progress_gap <= PROGRESS_EPSILON:
                continue
            longitudinal_gap_m = progress_gap * self.track_length_m
            if longitudinal_gap_m > HAZARD_DETECTION_DISTANCE_M:
                continue
            current_lateral_gap_m = abs(
                candidate.lateral_offset_m - state.lateral_offset_m
            )
            projected_lateral_gap_m = abs(
                (
                    candidate.lateral_offset_m
                    + candidate.lateral_speed_mps * 0.6
                )
                - (
                    state.lateral_offset_m
                    + state.lateral_speed_mps * 0.6
                )
            )
            required_lateral_clearance_m = (
                0.5 * (state.car_width_m + candidate.car_width_m)
                + MANEUVER_CLEARANCE_MARGIN_M
            )
            shares_projected_lane = (
                min(current_lateral_gap_m, projected_lateral_gap_m)
                < required_lateral_clearance_m
            )
            if not shares_projected_lane:
                continue
            if self._physics_v2_passing_authorized(state, candidate):
                continue
            candidates.append((longitudinal_gap_m, candidate))
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def _physics_v2_following_constraint(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        start_snapshot: dict[int, tuple[float, float]],
        active_line: str,
        delta: float,
    ) -> VehicleFollowingConstraint | None:
        track_profile = self._track_physics_for_driver(state)
        corridor_constraint = self._hazard_corridor_following_constraint(
            state,
            active_line,
            start_snapshot,
        )
        hazard_constraint = self._hazard_following_constraint(
            state,
            active_line,
        )
        if corridor_constraint is not None and (
            hazard_constraint is None
            or corridor_constraint.leader_distance_m
            < hazard_constraint.leader_distance_m
        ):
            return corridor_constraint
        if hazard_constraint is not None:
            return hazard_constraint
        safety_car_constraint = self._safety_car_following_constraint(
            state,
            car_ahead,
            active_line,
            delta,
        )
        if safety_car_constraint is not None:
            return safety_car_constraint
        car_ahead = self._nearest_physical_following_leader(
            state,
            start_snapshot,
            car_ahead,
        )
        if (
            car_ahead is None
            or car_ahead.in_pit
            or car_ahead.retired
            or car_ahead.finished
            or self._physics_v2_passing_authorized(state, car_ahead)
        ):
            return None
        if (
            state.avoidance_active
            and car_ahead.avoidance_active
            and state.avoidance_hazard_driver_id
            == car_ahead.avoidance_hazard_driver_id
            and state.avoidance_side in {"left", "right"}
            and car_ahead.avoidance_side in {"left", "right"}
            and state.avoidance_side != car_ahead.avoidance_side
            and abs(state.lateral_offset_m - car_ahead.lateral_offset_m)
            >= PHYSICAL_CAR_WIDTH_M + MANEUVER_CLEARANCE_MARGIN_M
        ):
            # Emergency routing may place adjacent cars in separate physical
            # corridors.  They may pass one another geometrically while the SC
            # sporting order remains frozen and is restored after the hazard.
            return None
        leader_snapshot = start_snapshot.get(car_ahead.driver_id)
        if leader_snapshot is None:
            return None

        leader_total_progress, leader_speed_mps = leader_snapshot
        follower_total_progress, follower_speed_mps = start_snapshot.get(
            state.driver_id,
            (state.total_progress, state.speed_kph / 3.6),
        )
        leader_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            leader_total_progress,
        )
        follower_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            follower_total_progress,
        )
        if leader_distance_m <= follower_distance_m + PROGRESS_EPSILON:
            # Frozen sporting positions can briefly differ from physical order,
            # especially while lapped cars circulate around the SC queue.
            return None
        current_gap_m = max(0.0, leader_distance_m - follower_distance_m)
        minimum_gap_m = (
            0.5 * (state.car_length_m + car_ahead.car_length_m)
            + FOLLOWING_MIN_BUMPER_GAP_M
        )
        dynamic_gap_m = min(28.0, max(10.0, minimum_gap_m + follower_speed_mps * 0.22))
        if (
            self.race_phase == "sc"
            and (
                state.driver_id in self._sc_caught_driver_ids
                or car_ahead.driver_id in self._sc_caught_driver_ids
            )
        ):
            desired_gap_m = SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
        elif self.race_phase == "sc":
            # Cars still catching the queue use a normal dynamic following gap;
            # imposing the final SC spacing too early creates a stop-and-go wave
            # that can strand the tail of the field several corners behind.
            desired_gap_m = dynamic_gap_m
        elif self.race_phase == "vsc":
            desired_gap_m = max(dynamic_gap_m, current_gap_m)
        else:
            battle = self._pair_maneuver(state, car_ahead)
            if battle is not None:
                desired_gap_m, minimum_gap_m = self._maneuver_following_gaps_m(
                    state,
                    car_ahead,
                    battle,
                )
            else:
                live_gap_seconds = current_gap_m / max(15.0, follower_speed_mps)
                active_segment = segment_at_progress(
                    self.circuit,
                    state.progress,
                )
                if (
                    live_gap_seconds <= BATTLE_EVENT_GAP_SECONDS
                    and active_segment is not None
                    and active_segment.type == TrackSegmentType.STRAIGHT
                ):
                    desired_gap_m = (
                        0.5 * (state.car_length_m + car_ahead.car_length_m)
                        + MANEUVER_TOW_PREP_BUMPER_GAP_M
                    )
                else:
                    desired_gap_m = dynamic_gap_m
        return VehicleFollowingConstraint(
            leader_distance_m=leader_distance_m,
            leader_speed_mps=leader_speed_mps,
            leader_end_distance_m=track_profile.line_distance_at_total_progress(
                active_line,
                car_ahead.total_progress,
            ),
            leader_end_speed_mps=max(0.0, car_ahead.speed_kph / 3.6),
            desired_gap_m=desired_gap_m,
            minimum_gap_m=minimum_gap_m,
        )

    def _safety_car_following_constraint(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        active_line: str,
        delta: float,
    ) -> VehicleFollowingConstraint | None:
        """Represent the on-track Safety Car as the queue leader without snapping cars."""
        track_profile = self._track_physics_for_driver(state)
        if (
            self.race_phase != "sc"
            or self.safety_car_stage in {"deploying", "restart"}
            or self._safety_car_total_progress is None
        ):
            return None
        on_track_leader = self._on_track_leader()
        if on_track_leader is None or on_track_leader.driver_id != state.driver_id:
            return None
        if (
            car_ahead is not None
            and not car_ahead.in_pit
            and not car_ahead.retired
            and not car_ahead.finished
        ):
            return None

        end_progress = self._safety_car_total_progress
        start_progress = end_progress - self._safety_car_progress_rate * max(0.0, delta)
        leader_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            start_progress,
        )
        leader_end_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            end_progress,
        )
        follower_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            state.total_progress,
        )
        if leader_end_distance_m <= follower_distance_m + PROGRESS_EPSILON:
            return None
        safety_car_speed_mps = max(0.0, self._safety_car_speed_mps)
        return VehicleFollowingConstraint(
            leader_distance_m=leader_distance_m,
            leader_speed_mps=safety_car_speed_mps,
            leader_end_distance_m=leader_end_distance_m,
            leader_end_speed_mps=safety_car_speed_mps,
            desired_gap_m=SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS,
            minimum_gap_m=SC_CAR_LENGTH_M + FOLLOWING_MIN_BUMPER_GAP_M,
        )

    def _body_pose(self, state: DriverRaceState) -> BodyPose:
        return BodyPose(
            longitudinal_m=state.total_progress * self.track_length_m,
            lateral_m=state.lateral_offset_m,
            heading_rad=state.slip_angle_rad,
            length_m=state.car_length_m,
            width_m=state.car_width_m,
            longitudinal_speed_mps=max(0.0, state.speed_kph / 3.6),
            lateral_speed_mps=state.lateral_speed_mps,
        )

    def _enter_tick_phase(self, phase: TickPhase) -> None:
        """Record the authoritative command -> physics -> rules -> telemetry order."""
        expected_previous = {
            TickPhase.COMMAND: {TickPhase.TELEMETRY},
            TickPhase.PHYSICS: {TickPhase.COMMAND},
            TickPhase.RULES: {TickPhase.PHYSICS},
            TickPhase.TELEMETRY: {TickPhase.RULES},
        }
        if self._tick_phase not in expected_previous[phase]:
            raise RuntimeError(
                f"Invalid simulation phase transition: "
                f"{self._tick_phase.value} -> {phase.value}"
            )
        self._tick_phase = phase
        self._tick_phase_history.append(phase)

    def _require_tick_phase(self, phase: TickPhase, operation: str) -> None:
        if self._tick_phase is not phase:
            raise RuntimeError(
                f"{operation} is only valid during {phase.value}; "
                f"current phase is {self._tick_phase.value}"
            )

    def _require_rules_phase(self, operation: str) -> None:
        """Guard a rules mutation; constructor setup is outside the tick."""
        if self._setup_mode:
            return
        self._require_tick_phase(TickPhase.RULES, operation)

    def complete_setup(self) -> None:
        """Close the pre-tick setup window before exercising runtime rules."""
        self._setup_mode = False

    def _consume_physics_step_result(self, result: PhysicsStepResult) -> None:
        """Mark one PHYSICS result as consumed exactly once by RULES."""
        self._require_tick_phase(TickPhase.RULES, "consume physics step result")
        frame_id = result.physics_frame_id
        if frame_id <= self._last_consumed_physics_frame_id:
            raise RuntimeError(
                "PhysicsStepResult frame ids must be consumed exactly once "
                "in strictly increasing order"
            )
        self._consumed_physics_frame_ids.append(frame_id)
        self._last_consumed_physics_frame_id = frame_id

    def _pit_route_points_m(
        self,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> list[tuple[float, float]]:
        """Return the compiled pit route in the immutable local metre frame."""
        track_profile = track_profile or self._track_physics
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        coordinate_frame = track_profile.coordinate_frame
        if (
            entry is None
            or exit_ is None
            or coordinate_frame is None
            or len(self.circuit.pit_lane_coords) < 2
        ):
            return []

        entry_x, entry_y, entry_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            entry,
        )
        exit_x, exit_y, exit_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            exit_,
        )
        interior_points = [
            coordinate_frame.to_local_m(x, y)
            for x, y in self.circuit.pit_lane_coords
        ]
        # Imported pit paths commonly repeat each track anchor with a point
        # only centimetres away in the lateral direction. Keeping that point
        # creates an artificial 90-degree tangent on pit-in/pit-out even
        # though the remaining route is correctly aligned. Collapse the
        # near-anchor samples and use the first meaningful route segment.
        while interior_points and hypot(
            interior_points[0][0] - entry_x,
            interior_points[0][1] - entry_y,
        ) < PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M:
            interior_points.pop(0)
        while interior_points and hypot(
            interior_points[-1][0] - exit_x,
            interior_points[-1][1] - exit_y,
        ) < PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M:
            interior_points.pop()

        points = [(entry_x, entry_y)]
        if interior_points and hypot(
            interior_points[0][0] - entry_x,
            interior_points[0][1] - entry_y,
        ) > 2.0 * PIT_ROUTE_TANGENT_LEAD_M:
            points.append(
                (
                    entry_x + cos(entry_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                    entry_y + sin(entry_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                )
            )
        points.extend(interior_points)
        if interior_points and hypot(
            interior_points[-1][0] - exit_x,
            interior_points[-1][1] - exit_y,
        ) > 2.0 * PIT_ROUTE_TANGENT_LEAD_M:
            points.append(
                (
                    exit_x - cos(exit_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                    exit_y - sin(exit_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                )
            )
        points.append((exit_x, exit_y))

        deduplicated = [points[0]]
        for point in points[1:]:
            if hypot(point[0] - deduplicated[-1][0], point[1] - deduplicated[-1][1]) > 1e-9:
                deduplicated.append(point)
        return deduplicated

    def _pit_route_length_m(
        self,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> float:
        points = self._pit_route_points_m(track_profile)
        return sum(
            hypot(next_point[0] - point[0], next_point[1] - point[1])
            for point, next_point in zip(points, points[1:])
        )

    def get_pit_route_coords(self) -> list[list[float]]:
        """Expose the exact route used by pit physics in circuit coordinates."""
        coordinate_frame = self._track_physics.coordinate_frame
        if coordinate_frame is None:
            return [list(point) for point in self.circuit.pit_lane_coords]
        return [
            list(coordinate_frame.from_local_m(x_m, y_m))
            for x_m, y_m in self._pit_route_points_m()
        ]

    def _pit_lane_pose_at_progress_m(
        self,
        lane_progress: float,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> tuple[float, float, float]:
        """Interpolate the authoritative pit-lane pose by route distance."""
        track_profile = track_profile or self._track_physics
        points = self._pit_route_points_m(track_profile)
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        if not points or entry is None or exit_ is None:
            return track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                entry or 0.0,
            )

        clamped = min(1.0, max(0.0, lane_progress))
        if clamped <= 0.0:
            x, y, heading = track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                entry,
            )
            return x, y, heading
        if clamped >= 1.0:
            x, y, heading = track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                exit_,
            )
            return x, y, heading

        segment_lengths = [
            hypot(next_point[0] - point[0], next_point[1] - point[1])
            for point, next_point in zip(points, points[1:])
        ]
        total_length = sum(segment_lengths)
        if total_length <= 1e-9:
            return points[0][0], points[0][1], 0.0
        target_distance = clamped * total_length
        traversed = 0.0
        for index, segment_length in enumerate(segment_lengths):
            if traversed + segment_length >= target_distance:
                ratio = (target_distance - traversed) / max(segment_length, 1e-9)
                point = points[index]
                next_point = points[index + 1]
                heading = atan2(
                    next_point[1] - point[1],
                    next_point[0] - point[0],
                )
                return (
                    point[0] + (next_point[0] - point[0]) * ratio,
                    point[1] + (next_point[1] - point[1]) * ratio,
                    heading,
                )
            traversed += segment_length
        point = points[-2]
        next_point = points[-1]
        return point[0], point[1], atan2(
            next_point[1] - point[1],
            next_point[0] - point[0],
        )

    def _pit_box_progress_for_team(self, team_id: int) -> float:
        """Return one stable longitudinal pit-box position per constructor."""
        pit_lane = self.circuit.pit_lane
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or route_length_m <= 1e-9:
            return 0.5
        team_ids = sorted({driver.team_id for driver in self.drivers.values()})
        if team_id not in team_ids or len(team_ids) <= 1:
            return pit_lane.box_progress
        team_index = team_ids.index(team_id)
        centered_index = team_index - (len(team_ids) - 1) / 2.0
        assigned = (
            pit_lane.box_progress
            + centered_index * PIT_TEAM_BOX_SPACING_M / route_length_m
        )
        operational_margin = max(
            0.01,
            (PIT_BOX_LATERAL_BLEND_DISTANCE_M + 4.0) / route_length_m,
        )
        return min(
            pit_lane.speed_limit_end - operational_margin,
            max(pit_lane.speed_limit_start + operational_margin, assigned),
        )

    def _pit_box_progress_for_driver(self, driver_id: int) -> float:
        driver = self.drivers.get(driver_id)
        if driver is None:
            pit_lane = self.circuit.pit_lane
            return pit_lane.box_progress if pit_lane is not None else 0.5
        return self._pit_box_progress_for_team(driver.team_id)

    def _pit_box_lateral_offset_m(
        self,
        driver_id: int,
        lane_progress: float,
    ) -> float:
        """Blend from the pit travel lane into the assigned garage-front box."""
        pit_lane = self.circuit.pit_lane
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or route_length_m <= 1e-9:
            return 0.0
        stall_progress = self._pit_box_progress_for_driver(driver_id)
        blend_progress = PIT_BOX_LATERAL_BLEND_DISTANCE_M / route_length_m
        distance = abs(lane_progress - stall_progress)
        if distance >= blend_progress:
            return 0.0
        normalized = 1.0 - distance / max(1e-9, blend_progress)
        smooth = normalized * normalized * (3.0 - 2.0 * normalized)
        return pit_lane.box_offset * smooth

    def _pit_vehicle_pose_at_progress_m(
        self,
        driver_id: int,
        lane_progress: float,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> tuple[float, float, float]:
        """Return the pit vehicle pose including the garage-front lateral path."""
        track_profile = track_profile or self._track_physics
        route_length_m = self._pit_route_length_m(track_profile)

        def offset_point(progress: float) -> tuple[float, float]:
            x, y, heading = self._pit_lane_pose_at_progress_m(
                progress,
                track_profile,
            )
            offset_m = self._pit_box_lateral_offset_m(driver_id, progress)
            return (
                x - sin(heading) * offset_m,
                y + cos(heading) * offset_m,
            )

        clamped = min(1.0, max(0.0, lane_progress))
        x, y = offset_point(clamped)
        sample_progress = max(1e-6, 0.30 / max(1.0, route_length_m))
        before = max(0.0, clamped - sample_progress)
        after = min(1.0, clamped + sample_progress)
        before_x, before_y = offset_point(before)
        after_x, after_y = offset_point(after)
        if after - before <= 1e-12:
            _, _, heading = self._pit_lane_pose_at_progress_m(
                clamped,
                track_profile,
            )
        else:
            heading = atan2(after_y - before_y, after_x - before_x)
        return x, y, heading

    def _initialize_authoritative_vehicle_telemetry(self) -> None:
        """Place stopped cars in their actual grid boxes before the first light."""
        for state in self.driver_states.values():
            self._set_authoritative_vehicle_telemetry(state)

    def _set_authoritative_vehicle_telemetry(self, state: DriverRaceState) -> None:
        track_profile = self._track_physics_for_driver(state)
        if state.in_pit:
            line_x, line_y, line_heading = self._pit_vehicle_pose_at_progress_m(
                state.driver_id,
                self._pit_lane_progress(state.driver_id),
                track_profile,
            )
            offset_delta = 0.0
        else:
            # A tactical line name is a controller choice, not a new world
            # coordinate frame. Switching lines must not remap an integrated
            # body pose onto another arc-length curve and teleport the car.
            active_line = DRIVING_LINE_RACING
            line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
                active_line,
                state.progress,
            )
            line_offset = track_profile.line_offset_at_progress(
                active_line,
                state.progress,
            )
            offset_delta = state.lateral_offset_m - line_offset
        state.world_x_m = round(line_x - sin(line_heading) * offset_delta, 4)
        state.world_y_m = round(line_y + cos(line_heading) * offset_delta, 4)

        heading = line_heading + state.slip_angle_rad
        state.heading_rad = round(heading, 6)

        speed_mps = max(0.0, state.speed_kph / 3.6)
        state.velocity_x_mps = round(speed_mps * cos(heading), 4)
        state.velocity_y_mps = round(speed_mps * sin(heading), 4)
        state.acceleration_x_mps2 = round(
            state.acceleration_mps2 * cos(heading)
            - state.lateral_acceleration_mps2 * sin(heading),
            4,
        )
        state.acceleration_y_mps2 = round(
            state.acceleration_mps2 * sin(heading)
            + state.lateral_acceleration_mps2 * cos(heading),
            4,
        )
        if speed_mps <= 0.5:
            state.gear = 0
            state.engine_rpm = 0.0
        else:
            state.gear = min(8, max(1, int(speed_mps / 12.0) + 1))
            state.engine_rpm = round(
                1500.0 + speed_mps * (100.0 - state.gear * 5.0),
                1,
            )
        state.simulation_time_s = round(self.race_elapsed, 6)
        state.physics_frame = self._physics_frame
        state.telemetry_source = TELEMETRY_SOURCE_DERIVED

    def _update_authoritative_vehicle_telemetry(self, delta: float) -> None:
        """Update physics-owned state fields from the completed fixed step."""
        self._require_tick_phase(TickPhase.RULES, "update physics-derived vehicle state")
        del delta
        for state in self.driver_states.values():
            self._set_authoritative_vehicle_telemetry(state)

    def _refresh_vehicle_telemetry(self, delta: float) -> None:
        """Build telemetry output without mutating authoritative state.

        The authoritative values are updated at the end of RULES by
        ``_update_authoritative_vehicle_telemetry``.  This method remains as a
        named phase hook and deliberately performs no state writes so repeated
        snapshot construction is safe.
        """
        self._require_tick_phase(TickPhase.TELEMETRY, "refresh telemetry")
        del delta

    def _start_driver_input_error(
        self,
        driver_id: int,
        *,
        kind: str,
        duration_seconds: float,
        intensity: float,
        trigger: str,
        segment_name: str = "",
        opponent_id: int | None = None,
    ) -> None:
        """Inject a bounded control mistake without claiming a physical outcome."""
        current = self._driver_input_errors.get(driver_id)
        if current is not None and current.kind == kind:
            current.remaining_seconds = max(current.remaining_seconds, duration_seconds)
            current.intensity = max(current.intensity, intensity)
            current.trigger = trigger
            current.segment_name = segment_name
            current.opponent_id = opponent_id
            return
        self._driver_input_errors[driver_id] = DriverInputError(
            kind=kind,
            remaining_seconds=duration_seconds,
            intensity=intensity,
            trigger=trigger,
            segment_name=segment_name,
            opponent_id=opponent_id,
        )

    def _tick_driver_input_errors(self, delta: float) -> None:
        for driver_id, input_error in list(self._driver_input_errors.items()):
            input_error.remaining_seconds -= delta
            if input_error.remaining_seconds <= 0.0:
                del self._driver_input_errors[driver_id]

    def _tick_collision_states(self, delta: float) -> None:
        self._tick_driver_input_errors(delta)
        for driver_id, remaining in list(
            self._physical_handling_event_cooldown.items()
        ):
            remaining -= delta
            if remaining <= 0.0:
                del self._physical_handling_event_cooldown[driver_id]
            else:
                self._physical_handling_event_cooldown[driver_id] = remaining
        for key, remaining in list(self._collision_pair_cooldown.items()):
            remaining -= delta
            if remaining <= 0.0:
                del self._collision_pair_cooldown[key]
            else:
                self._collision_pair_cooldown[key] = remaining
        for driver_id, remaining in list(self._contact_display_remaining.items()):
            remaining -= delta
            if remaining > 0.0:
                self._contact_display_remaining[driver_id] = remaining
                continue
            del self._contact_display_remaining[driver_id]
            state = self.driver_states.get(driver_id)
            if state is None:
                continue
            state.contact_active = False
            state.contact_opponent_id = None
            state.contact_impact_speed_mps = 0.0
            state.contact_type = ""
            state.contact_severity = ""
            state.contact_normal_longitudinal = 0.0
            state.contact_normal_lateral = 0.0

    def _record_contact_state(
        self,
        state: DriverRaceState,
        opponent_id: int,
        *,
        impact_speed_mps: float,
        contact_type: str,
        severity: IncidentSeverity,
        progress: float,
        lateral_offset_m: float,
        normal_longitudinal: float,
        normal_lateral: float,
    ) -> None:
        state.contact_active = True
        state.contact_opponent_id = opponent_id
        state.contact_impact_speed_mps = round(impact_speed_mps, 3)
        state.contact_type = contact_type
        state.contact_severity = severity.value
        state.contact_progress = progress % 1.0
        state.contact_lateral_offset_m = round(lateral_offset_m, 4)
        state.contact_normal_longitudinal = round(normal_longitudinal, 4)
        state.contact_normal_lateral = round(normal_lateral, 4)
        self._contact_display_remaining[state.driver_id] = COLLISION_DISPLAY_SECONDS

    def _apply_collision_response(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
        first_motion: BodyMotion,
        second_motion: BodyMotion,
        contact,
        delta: float,
    ) -> tuple[BodyPose, BodyPose]:
        """Resolve at impact, exchange momentum, then finish the remaining tick."""
        safe_fraction = max(0.0, contact.time_fraction - 1e-4)
        first_pose = interpolate_pose(first_motion, safe_fraction)
        second_pose = interpolate_pose(second_motion, safe_fraction)

        if contact.time_fraction <= 1e-9 and contact.penetration_m > 0.0:
            correction = 0.5 * (contact.penetration_m + 0.005)
            first_pose = BodyPose(
                first_pose.longitudinal_m - contact.normal_longitudinal * correction,
                first_pose.lateral_m - contact.normal_lateral * correction,
                first_pose.heading_rad,
                first_pose.length_m,
                first_pose.width_m,
                first_pose.longitudinal_speed_mps,
                first_pose.lateral_speed_mps,
            )
            second_pose = BodyPose(
                second_pose.longitudinal_m + contact.normal_longitudinal * correction,
                second_pose.lateral_m + contact.normal_lateral * correction,
                second_pose.heading_rad,
                second_pose.length_m,
                second_pose.width_m,
                second_pose.longitudinal_speed_mps,
                second_pose.lateral_speed_mps,
            )

        self._set_state_total_progress(
            first,
            first_pose.longitudinal_m / self.track_length_m,
        )
        self._set_state_total_progress(
            second,
            second_pose.longitudinal_m / self.track_length_m,
        )
        first.lateral_offset_m = round(first_pose.lateral_m, 4)
        second.lateral_offset_m = round(second_pose.lateral_m, 4)

        first_speed = max(0.0, first_pose.longitudinal_speed_mps)
        second_speed = max(0.0, second_pose.longitudinal_speed_mps)
        impact = max(0.0, contact.impact_speed_mps)
        if contact.contact_type == "front_rear":
            faster_is_first = first_speed >= second_speed
            slower_speed = min(first_speed, second_speed)
            speed_difference = abs(first_speed - second_speed)
            slower_after = max(0.0, slower_speed - min(2.0, 0.03 * impact))
            faster_after = max(
                0.0,
                slower_after
                + 0.10 * speed_difference
                - min(4.0, 0.08 * impact),
            )
            if faster_is_first:
                first_speed, second_speed = faster_after, slower_after
            else:
                first_speed, second_speed = slower_after, faster_after
        else:
            # A static overlap is already resolved by positional correction.
            # Do not invent a minimum loss or lateral kick when two cars have
            # no closing speed; that made harmless close running look like a
            # magnetic rejection.  Real impact still dissipates energy.
            energy_loss = min(8.0, 0.16 * impact)
            first_speed -= energy_loss
            second_speed -= energy_loss

        first.speed_kph = round(max(0.0, first_speed) * 3.6, 3)
        second.speed_kph = round(max(0.0, second_speed) * 3.6, 3)
        lateral_impulse = min(4.0, 0.18 * impact)
        first.lateral_speed_mps = max(
            -4.0,
            min(
                4.0,
                first_pose.lateral_speed_mps
                - contact.normal_lateral * lateral_impulse,
            ),
        )
        second.lateral_speed_mps = max(
            -4.0,
            min(
                4.0,
                second_pose.lateral_speed_mps
                + contact.normal_lateral * lateral_impulse,
            ),
        )
        yaw_impulse = min(0.12, 0.008 * impact)
        first.slip_angle_rad = max(
            -0.20,
            min(
                0.20,
                first.slip_angle_rad - contact.normal_lateral * yaw_impulse,
            ),
        )
        second.slip_angle_rad = max(
            -0.20,
            min(
                0.20,
                second.slip_angle_rad + contact.normal_lateral * yaw_impulse,
            ),
        )

        # Continue through the unused part of this physics tick at the
        # post-impact speed. Previously both cars stayed at the impact point
        # until the next tick; progress reconciliation then interpreted that
        # truncated motion as a near stop, which invited a following car into
        # an otherwise minor contact.
        if contact.contact_type == "front_rear":
            leader, follower = (
                (first, second)
                if first.total_progress >= second.total_progress
                else (second, first)
            )
            follower.speed_kph = min(follower.speed_kph, leader.speed_kph)
        remaining_seconds = max(0.0, delta * (1.0 - contact.time_fraction))
        if remaining_seconds > 0.0:
            self._set_state_total_progress(
                first,
                first.total_progress
                + (first.speed_kph / 3.6) * remaining_seconds / self.track_length_m,
            )
            self._set_state_total_progress(
                second,
                second.total_progress
                + (second.speed_kph / 3.6) * remaining_seconds / self.track_length_m,
            )
        for _ in range(4):
            residual_overlap = oriented_body_overlap(
                self._body_pose(first),
                self._body_pose(second),
            )
            if residual_overlap is None or residual_overlap[0] <= 1e-6:
                break
            penetration, normal = residual_overlap
            correction = 0.5 * (penetration + 0.005)
            self._set_state_total_progress(
                first,
                first.total_progress
                - normal[0] * correction / self.track_length_m,
            )
            self._set_state_total_progress(
                second,
                second.total_progress
                + normal[0] * correction / self.track_length_m,
            )
            first.lateral_offset_m = round(
                first.lateral_offset_m - normal[1] * correction,
                4,
            )
            second.lateral_offset_m = round(
                second.lateral_offset_m + normal[1] * correction,
                4,
            )
        collision_deceleration = impact / max(delta, PHYSICS_STEP_SECONDS) * 0.20
        first.acceleration_mps2 = min(first.acceleration_mps2, -collision_deceleration)
        second.acceleration_mps2 = min(second.acceleration_mps2, -collision_deceleration)
        first.handling_state = "contact"
        second.handling_state = "contact"
        first.tire_usage += min(0.02, impact * 0.0008)
        second.tire_usage += min(0.02, impact * 0.0008)
        return self._body_pose(first), self._body_pose(second)

    def _resolve_vehicle_collisions(
        self,
        start_poses: dict[int, BodyPose],
        delta: float,
        *,
        collect_facts: bool = False,
    ) -> list[CollisionFact] | list[RaceEvent]:
        running = [
            state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        ]
        if self.race_phase != "green":
            self._resolve_static_body_penetrations(running)
            return []
        facts: list[CollisionFact] = []
        end_poses = {
            state.driver_id: self._body_pose(state)
            for state in running
        }
        for first_index, first in enumerate(running):
            if first.retired:
                continue
            first_start = start_poses.get(first.driver_id)
            if first_start is None:
                continue
            for second in running[first_index + 1:]:
                if first.retired:
                    break
                if second.retired:
                    continue
                key = self._battle_pair_key(first.driver_id, second.driver_id)
                if self._collision_pair_cooldown.get(key, 0.0) > 0.0:
                    continue
                second_start = start_poses.get(second.driver_id)
                if second_start is None:
                    continue
                first_end = end_poses[first.driver_id]
                second_end = end_poses[second.driver_id]
                start_gap = abs(
                    first_start.longitudinal_m - second_start.longitudinal_m
                )
                end_gap = abs(first_end.longitudinal_m - second_end.longitudinal_m)
                if min(start_gap, end_gap) > 14.0:
                    continue
                first_motion = BodyMotion(first_start, first_end)
                second_motion = BodyMotion(second_start, second_end)
                contact = swept_body_collision(
                    first_motion,
                    second_motion,
                    substeps=max(1, round(delta / PHYSICS_STEP_SECONDS)),
                )
                if contact is None:
                    continue
                if (
                    contact.impact_speed_mps < COLLISION_MIN_REPORT_IMPACT_MPS
                    and contact.penetration_m
                    < COLLISION_STATIC_PENETRATION_TOLERANCE_M
                ):
                    continue

                first_pose, second_pose = self._apply_collision_response(
                    first,
                    second,
                    first_motion,
                    second_motion,
                    contact,
                    delta,
                )
                end_poses[first.driver_id] = first_pose
                end_poses[second.driver_id] = second_pose
                # PHYSICS records contact facts only.  Severity, damage
                # escalation and visible events belong to RULES.
                contact_progress = (
                    0.5 * (first.total_progress + second.total_progress)
                ) % 1.0
                contact_lateral = 0.5 * (
                    first.lateral_offset_m + second.lateral_offset_m
                )
                self._record_contact_state(
                    first,
                    second.driver_id,
                    impact_speed_mps=contact.impact_speed_mps,
                    contact_type=contact.contact_type,
                    severity=IncidentSeverity.MINOR,
                    progress=contact_progress,
                    lateral_offset_m=contact_lateral,
                    normal_longitudinal=contact.normal_longitudinal,
                    normal_lateral=contact.normal_lateral,
                )
                self._record_contact_state(
                    second,
                    first.driver_id,
                    impact_speed_mps=contact.impact_speed_mps,
                    contact_type=contact.contact_type,
                    severity=IncidentSeverity.MINOR,
                    progress=contact_progress,
                    lateral_offset_m=contact_lateral,
                    normal_longitudinal=-contact.normal_longitudinal,
                    normal_lateral=-contact.normal_lateral,
                )
                self._collision_pair_cooldown[key] = COLLISION_PAIR_COOLDOWN_SECONDS
                facts.append(
                    CollisionFact(
                        first_driver_id=first.driver_id,
                        second_driver_id=second.driver_id,
                        impact_speed_mps=contact.impact_speed_mps,
                        contact_type=contact.contact_type,
                        normal_longitudinal=contact.normal_longitudinal,
                        normal_lateral=contact.normal_lateral,
                        first_speed_mps=first_start.longitudinal_speed_mps,
                        second_speed_mps=second_start.longitudinal_speed_mps,
                    )
                )
        self._resolve_static_body_penetrations(running)
        if collect_facts:
            return facts
        events: list[RaceEvent] = []
        self._apply_collision_facts(facts, events)
        return events

    def _apply_collision_facts(
        self,
        facts: list[CollisionFact],
        events: list[RaceEvent],
    ) -> None:
        """Resolve raw PHYSICS contacts into RULES outcomes exactly once."""
        self._require_rules_phase("apply collision facts")
        for fact in facts:
            first = self.driver_states.get(fact.first_driver_id)
            second = self.driver_states.get(fact.second_driver_id)
            if first is None or second is None:
                continue
            severity = IncidentSeverity.MINOR
            if fact.impact_speed_mps >= COLLISION_ESCALATION_IMPACT_MPS:
                severity = escalate_collision(
                    self.rng,
                    segment_type=self._segment_type_at(first),
                    impact_speed_mps=fact.impact_speed_mps,
                )
            damage_increment = min(
                0.35,
                0.006 * fact.impact_speed_mps
                + (
                    0.30
                    if severity == IncidentSeverity.CRASH
                    else 0.12
                    if severity == IncidentSeverity.CAR_STOPPED
                    else 0.0
                ),
            )
            first.collision_damage = min(1.0, first.collision_damage + damage_increment)
            second.collision_damage = min(1.0, second.collision_damage + damage_increment)
            first.contact_severity = severity.value
            second.contact_severity = severity.value

            first_meta = self._driver_meta[first.driver_id]
            second_meta = self._driver_meta[second.driver_id]
            events.append(
                RaceEvent(
                    type="minor_contact" if severity == IncidentSeverity.MINOR else "collision",
                    driver=first_meta["abbreviation"],
                    message=(
                        f"{first_meta['full_name']} and {second_meta['full_name']}"
                        f" make {fact.contact_type.replace('_', '-')} contact"
                        f" at {fact.impact_speed_mps:.1f}m/s relative speed"
                    ),
                    message_ko=(
                        f"{first_meta['full_name']}와 {second_meta['full_name']}가"
                        f" 상대속도 {fact.impact_speed_mps:.1f}m/s로 실제 접촉합니다"
                    ),
                )
            )
            battle = self._side_by_side_battles.get(
                self._battle_pair_key(first.driver_id, second.driver_id)
            )
            if battle is not None:
                self._set_maneuver_phase(battle, "abort")
            if severity != IncidentSeverity.MINOR:
                primary, secondary = (
                    (first, second)
                    if fact.first_speed_mps >= fact.second_speed_mps
                    else (second, first)
                )
                self._apply_incident(
                    Incident(
                        cause=IncidentCause.COLLISION,
                        severity=severity,
                        primary_driver_id=primary.driver_id,
                        secondary_driver_id=(
                            secondary.driver_id
                            if severity == IncidentSeverity.CRASH
                            else None
                        ),
                    ),
                    events,
                )

    def _resolve_static_body_penetrations(
        self,
        states: list[DriverRaceState],
    ) -> None:
        """Settle multi-car contact chains with minimum translation vectors."""
        active = sorted(
            (state for state in states if not state.retired),
            key=lambda state: -state.total_progress,
        )
        for _ in range(24):
            moved = False
            for first_index, first in enumerate(active):
                for second in active[first_index + 1:]:
                    if abs(first.total_progress - second.total_progress) * self.track_length_m > 8.0:
                        continue
                    overlap = oriented_body_overlap(
                        self._body_pose(first),
                        self._body_pose(second),
                    )
                    if overlap is None or overlap[0] <= 1e-5:
                        continue
                    penetration, normal = overlap
                    correction = penetration + 0.002
                    if abs(normal[0]) >= abs(normal[1]):
                        # Preserve the leading car and propagate the exact minimum
                        # translation rearward through a contact train. Symmetric
                        # corrections can otherwise reopen an already solved pair.
                        self._set_state_total_progress(
                            second,
                            second.total_progress
                            + normal[0] * correction / self.track_length_m,
                        )
                        second.lateral_offset_m = round(
                            second.lateral_offset_m + normal[1] * correction,
                            4,
                        )
                    else:
                        half_correction = 0.5 * correction
                        self._set_state_total_progress(
                            first,
                            first.total_progress
                            - normal[0] * half_correction / self.track_length_m,
                        )
                        self._set_state_total_progress(
                            second,
                            second.total_progress
                            + normal[0] * half_correction / self.track_length_m,
                        )
                        first.lateral_offset_m = round(
                            first.lateral_offset_m - normal[1] * half_correction,
                            4,
                        )
                        second.lateral_offset_m = round(
                            second.lateral_offset_m + normal[1] * half_correction,
                            4,
                        )
                    moved = True
            if not moved:
                break

    def _resolve_physics_v2_same_line_gaps(self) -> None:
        """Compatibility hook; physical following and swept bodies own gaps.

        The former implementation copied the leading car's speed and progress
        rate into a close follower after physics had already run.  At corner
        entry this produced non-contact drops of more than 200 km/h in one
        20 ms step—the visible "ejection" stutter.  A close car now decelerates
        only through the tyre-force-limited controller above; actual overlap
        remains the responsibility of swept collision resolution.
        """
        return

    def _reconcile_progress_rates(
        self,
        start_snapshot: dict[int, tuple[float, float]],
        delta: float,
    ) -> None:
        """Publish the motion that survived final gap and race-control resolution."""
        if delta <= 0.0:
            return
        for driver_id, (start_progress, _) in start_snapshot.items():
            state = self.driver_states.get(driver_id)
            if state is None or state.in_pit or state.retired or state.finished:
                self._progress_rate[driver_id] = 0.0
                continue
            actual_rate = max(0.0, (state.total_progress - start_progress) / delta)
            computed_rate = max(0.0, self._progress_rate.get(driver_id, 0.0))
            if actual_rate + 1e-9 < computed_rate:
                active_line = self._physics_v2_virtual_line(state)
                track_profile = self._track_physics_for_driver(state)
                line_length_m = max(
                    1.0,
                    track_profile.length_for_line(active_line),
                )
                state.speed_kph = min(
                    state.speed_kph,
                    actual_rate * line_length_m * 3.6,
                )
            self._progress_rate[driver_id] = actual_rate

    def _pit_phase_speed_kph(self, driver_id: int) -> float:
        return self._pit_route_speed_mps.get(driver_id, 0.0) * 3.6

    def _dead_tire_lap_penalty(self, tire_wear: float) -> float:
        """Return extra lap-time loss as tire life approaches 0%."""
        dead_tire_ratio = max(0.0, (tire_wear - 0.75) / 0.25)
        return 1.35 * (dead_tire_ratio ** 1.7)

    def _pace_mode_effect(self, state: DriverRaceState) -> dict[str, float]:
        return self._interpolated_pace_mode_values(state, PACE_MODE_EFFECTS)

    def _synchronize_direct_pace_mode_assignment(
        self,
        state: DriverRaceState,
    ) -> None:
        """Keep tests/tools that assign state.pace_mode directly deterministic."""
        target_mode = self._pace_mode_transition_target.get(state.driver_id)
        if target_mode == state.pace_mode:
            return
        intensity = PACE_MODE_INTENSITIES.get(state.pace_mode, 0.0)
        self._pace_mode_intensity[state.driver_id] = intensity
        self._pace_mode_transition_source[state.driver_id] = intensity
        self._pace_mode_transition_elapsed[state.driver_id] = (
            PACE_MODE_TRANSITION_SECONDS
        )
        self._pace_mode_transition_target[state.driver_id] = state.pace_mode
        self._pace_mode_transition_from[state.driver_id] = state.pace_mode

    def _effective_pace_mode_intensity(self, state: DriverRaceState) -> float:
        self._synchronize_direct_pace_mode_assignment(state)
        return self._pace_mode_intensity.get(
            state.driver_id,
            PACE_MODE_INTENSITIES.get(state.pace_mode, 0.0),
        )

    def _pace_mode_transition_progress(self, state: DriverRaceState) -> float:
        self._synchronize_direct_pace_mode_assignment(state)
        return min(
            1.0,
            max(
                0.0,
                self._pace_mode_transition_elapsed.get(
                    state.driver_id,
                    PACE_MODE_TRANSITION_SECONDS,
                )
                / PACE_MODE_TRANSITION_SECONDS,
            ),
        )

    def _pace_mode_bracket(
        self,
        state: DriverRaceState,
    ) -> tuple[PaceMode, PaceMode, float]:
        intensity = max(-1.0, min(1.0, self._effective_pace_mode_intensity(state)))
        if intensity <= 0.0:
            return PaceMode.CONSERVE, PaceMode.STANDARD, intensity + 1.0
        return PaceMode.STANDARD, PaceMode.ATTACK, intensity

    def _interpolated_pace_mode_values(
        self,
        state: DriverRaceState,
        values_by_mode: dict[PaceMode, dict[str, float]],
    ) -> dict[str, float]:
        lower_mode, upper_mode, ratio = self._pace_mode_bracket(state)
        lower = values_by_mode[lower_mode]
        upper = values_by_mode[upper_mode]
        return {
            key: lower[key] + (upper[key] - lower[key]) * ratio
            for key in lower
        }

    def _interpolated_pace_mode_planner_weights(
        self,
        state: DriverRaceState,
    ) -> LocalTrajectoryPlannerWeights:
        lower_mode, upper_mode, ratio = self._pace_mode_bracket(state)
        lower = PACE_MODE_PLANNER_WEIGHTS[lower_mode]
        upper = PACE_MODE_PLANNER_WEIGHTS[upper_mode]

        def blend(attribute: str) -> float:
            lower_value = getattr(lower, attribute)
            return lower_value + (
                getattr(upper, attribute) - lower_value
            ) * ratio

        return LocalTrajectoryPlannerWeights(
            forward_progress=blend("forward_progress"),
            extra_path_distance=blend("extra_path_distance"),
            lateral_deviation=blend("lateral_deviation"),
            surface=blend("surface"),
            tire_surface=blend("tire_surface"),
            traffic=blend("traffic"),
        )

    def _tick_pace_mode_transitions(self, delta: float) -> None:
        delta_seconds = max(0.0, delta)
        for state in self.driver_states.values():
            self._synchronize_direct_pace_mode_assignment(state)
            driver_id = state.driver_id
            elapsed = min(
                PACE_MODE_TRANSITION_SECONDS,
                self._pace_mode_transition_elapsed.get(
                    driver_id,
                    PACE_MODE_TRANSITION_SECONDS,
                )
                + delta_seconds,
            )
            self._pace_mode_transition_elapsed[driver_id] = elapsed
            ratio = elapsed / PACE_MODE_TRANSITION_SECONDS
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            source = self._pace_mode_transition_source.get(
                driver_id,
                PACE_MODE_INTENSITIES.get(state.pace_mode, 0.0),
            )
            target = PACE_MODE_INTENSITIES.get(state.pace_mode, 0.0)
            self._pace_mode_intensity[driver_id] = source + (
                target - source
            ) * smooth_ratio
            if elapsed >= PACE_MODE_TRANSITION_SECONDS:
                self._pace_mode_intensity[driver_id] = target
                self._pace_mode_transition_source[driver_id] = target
                self._pace_mode_transition_from[driver_id] = state.pace_mode

    def _apply_pace_mode(
        self,
        state: DriverRaceState,
        pace_mode: PaceMode,
    ) -> None:
        current_intensity = self._effective_pace_mode_intensity(state)
        if state.pace_mode == pace_mode:
            return
        if current_intensity < -0.5:
            from_mode = PaceMode.CONSERVE
        elif current_intensity > 0.5:
            from_mode = PaceMode.ATTACK
        else:
            from_mode = PaceMode.STANDARD
        self._pace_mode_transition_source[state.driver_id] = current_intensity
        self._pace_mode_transition_elapsed[state.driver_id] = 0.0
        self._pace_mode_transition_target[state.driver_id] = pace_mode
        self._pace_mode_transition_from[state.driver_id] = from_mode
        state.pace_mode = pace_mode
        self._pace_mode_replan_required.add(state.driver_id)
        self._local_trajectory_plan_ages[state.driver_id] = (
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS
        )

    def _pace_mode_attack_factor(self, state: DriverRaceState) -> float:
        lower_mode, upper_mode, ratio = self._pace_mode_bracket(state)
        lower = PACE_MODE_ATTACK_FACTORS[lower_mode]
        return lower + (PACE_MODE_ATTACK_FACTORS[upper_mode] - lower) * ratio

    def _effective_tire_age(self, state: DriverRaceState) -> float:
        """Return driver-managed tire age from accumulated tire usage."""
        raw_age = state.tire_usage
        if raw_age <= 0 and state.tire_age > 0:
            raw_age = float(state.tire_age)
        tire_management = self._driver_meta[state.driver_id]["tire_management"]
        return compute_managed_tire_age(raw_age, tire_management)

    def _current_tire_wear(self, state: DriverRaceState) -> float:
        return compute_wear(state.tire_compound, self._effective_tire_age(state))

    def _current_tire_physics(self, state: DriverRaceState) -> TirePhysicsFactors:
        return compute_tire_physics_factors(
            state.tire_compound,
            self._effective_tire_age(state),
            self._tire_random.get(state.driver_id, 0.0),
            state.tire_surface_temperature_c,
        )

    def _expected_fuel_per_lap_kg(self) -> float:
        return max(
            0.1,
            self.track_length_m / 1000.0 * FUEL_CONSUMPTION_KG_PER_KM,
        )

    def _consume_fuel(
        self,
        state: DriverRaceState,
        *,
        throttle: float,
        delta_seconds: float,
    ) -> None:
        pace_fuel_factor = {
            PaceMode.CONSERVE: 0.96,
            PaceMode.STANDARD: 1.0,
            PaceMode.ATTACK: 1.03,
        }[state.pace_mode]
        flow_kg_s = (
            IDLE_FUEL_FLOW_KG_PER_SECOND
            + MAX_FUEL_FLOW_KG_PER_SECOND * max(0.0, min(1.0, throttle))
        ) * pace_fuel_factor
        burned_kg = min(
            state.fuel_mass_kg,
            flow_kg_s * max(0.0, delta_seconds),
        )
        state.fuel_mass_kg = max(0.0, state.fuel_mass_kg - burned_kg)
        state.fuel_burned_kg += burned_kg
        state.fuel_laps_remaining = (
            state.fuel_mass_kg / self._expected_fuel_per_lap_kg()
        )

    def _update_tire_thermal_state(
        self,
        state: DriverRaceState,
        result,
        delta_seconds: float,
    ) -> None:
        thermal = advance_tire_thermal_state(
            state.tire_compound,
            surface_temperature_c=state.tire_surface_temperature_c,
            core_temperature_c=state.tire_core_temperature_c,
            delta_seconds=delta_seconds,
            speed_mps=result.speed_mps,
            lateral_acceleration_mps2=result.lateral_acceleration_mps2,
            throttle=result.throttle,
            brake=result.brake,
            slide_energy_j=result.tire_slide_energy_j,
        )
        state.tire_surface_temperature_c = round(
            thermal.surface_temperature_c,
            4,
        )
        state.tire_core_temperature_c = round(thermal.core_temperature_c, 4)
        state.tire_thermal_grip = round(thermal.thermal_grip, 5)

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
            meta["car_factors"].qualifying,
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
        car_factors = meta["car_factors"]
        pace_edge = meta["pace"] - 1.0
        consistency_loss = 1.0 - meta["consistency"]
        tire_management_loss = 1.0 - meta["tire_management"]
        tire_wear = self._current_tire_wear(state)

        if segment.type == TrackSegmentType.STRAIGHT:
            return -2.20 * (car_factors.top_speed - 1.0) - 0.60 * pace_edge
        if segment.type == TrackSegmentType.SWEEPING:
            return -0.80 * (car_factors.high_speed_grip - 1.0) - 1.40 * pace_edge + 0.08 * tire_wear
        if segment.type == TrackSegmentType.HEAVY_BRAKING:
            return (
                -0.22 * (meta["overtaking"] - 0.85)
                -0.65 * (car_factors.braking - 1.0)
                - 0.80 * pace_edge
                + 0.18 * tire_wear
                + 0.05 * consistency_loss
            )
        if segment.type == TrackSegmentType.TECHNICAL:
            return -0.70 * (car_factors.low_speed_grip - 1.0) - 1.80 * pace_edge + 0.24 * tire_wear + 0.08 * consistency_loss
        if segment.type == TrackSegmentType.TRACTION:
            return -0.75 * (car_factors.traction - 1.0) - 1.00 * pace_edge + 0.30 * tire_wear + 0.10 * tire_management_loss
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

    def _reset_wake_state(self, state: DriverRaceState) -> None:
        state.drs_active = False
        state.dirty_air_active = False
        state.wake_strength = 0.0
        state.tow_strength = 0.0
        state.dirty_air_strength = 0.0
        state.wake_source_driver_id = None
        state.wake_longitudinal_gap_m = 0.0
        state.wake_lateral_separation_m = 0.0
        state.wake_drag_multiplier = 1.0
        state.wake_downforce_multiplier = 1.0
        state.wake_braking_grip_multiplier = 1.0
        state.wake_lateral_grip_multiplier = 1.0

    def _wake_effects_from_source(
        self,
        state: DriverRaceState,
        source: DriverRaceState,
        curvature_1pm: float | None = None,
    ) -> tuple[WakeEffects, float, float]:
        progress_gap = source.total_progress - state.total_progress
        gap_m = progress_gap * self.track_length_m
        lateral_separation_m = state.lateral_offset_m - source.lateral_offset_m
        if gap_m <= WAKE_MIN_GAP_M or gap_m >= WAKE_MAX_GAP_M:
            return WakeEffects(), gap_m, lateral_separation_m
        if curvature_1pm is None:
            active_line = self._physics_v2_virtual_line(state)
            physics_by_line = self._vehicle_physics_for_driver(state)
            line_physics = physics_by_line.get(
                active_line,
                physics_by_line[DRIVING_LINE_RACING],
            )
            track_profile = self._track_physics_for_driver(state)
            line_distance_m = track_profile.line_distance_at_total_progress(
                active_line,
                state.total_progress,
            )
            curvature_1pm = line_physics._raw_curvature_1pm(line_distance_m)
        return (
            compute_wake_effects(
                longitudinal_gap_m=gap_m,
                lateral_separation_m=lateral_separation_m,
                curvature_1pm=curvature_1pm,
                speed_mps=max(0.0, state.speed_kph / 3.6),
            ),
            gap_m,
            lateral_separation_m,
        )

    def _update_wake_state(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        wake_candidates: list[DriverRaceState] | None = None,
    ) -> WakeEffects:
        """Select the strongest aligned source while keeping DRS rank-based."""
        self._reset_wake_state(state)
        if state.in_pit or state.retired or state.finished:
            return WakeEffects()

        candidates = wake_candidates if wake_candidates is not None else [car_ahead]
        active_line = self._physics_v2_virtual_line(state)
        physics_by_line = self._vehicle_physics_for_driver(state)
        line_physics = physics_by_line.get(
            active_line,
            physics_by_line[DRIVING_LINE_RACING],
        )
        track_profile = self._track_physics_for_driver(state)
        line_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            state.total_progress,
        )
        curvature_1pm = line_physics._raw_curvature_1pm(line_distance_m)
        best: tuple[WakeEffects, DriverRaceState, float, float] | None = None
        for candidate in candidates:
            if (
                candidate is None
                or candidate.driver_id == state.driver_id
                or candidate.in_pit
                or candidate.retired
                or candidate.finished
            ):
                continue
            effects, gap_m, lateral_separation_m = self._wake_effects_from_source(
                state,
                candidate,
                curvature_1pm,
            )
            if effects.wake_strength <= 1e-6:
                continue
            if best is None or (
                effects.wake_strength,
                -gap_m,
                -candidate.driver_id,
            ) > (
                best[0].wake_strength,
                -best[2],
                -best[1].driver_id,
            ):
                best = (effects, candidate, gap_m, lateral_separation_m)

        effects = best[0] if best is not None else WakeEffects()
        if best is not None:
            state.wake_source_driver_id = best[1].driver_id
            state.wake_longitudinal_gap_m = best[2]
            state.wake_lateral_separation_m = best[3]
        state.wake_strength = effects.wake_strength
        state.tow_strength = effects.tow_strength
        state.dirty_air_strength = effects.dirty_air_strength
        state.wake_drag_multiplier = effects.drag_multiplier
        state.wake_downforce_multiplier = effects.downforce_multiplier
        state.wake_braking_grip_multiplier = effects.braking_grip_multiplier
        state.wake_lateral_grip_multiplier = effects.lateral_grip_multiplier
        state.dirty_air_active = effects.dirty_air_strength >= 0.04

        if car_ahead is not None:
            progress_gap = car_ahead.total_progress - state.total_progress
            gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
            state.drs_active = (
                0.0 < gap_seconds <= TRAFFIC_GAP_SECONDS
                and self._is_drs_zone(state.progress)
                and self._overtaking_candidate_allowed(state, car_ahead)
            )
        return effects

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
            and self._grid_launch_distance_m(state) < GRID_LAUNCH_MERGE_DISTANCE_M
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
            managed_defender_conflict = (
                candidate.body_boundary_violations == 0
                and candidate.predicted_collision_count > 0
                and set(candidate.conflicting_driver_ids)
                <= {defender_state.driver_id}
                and candidate.first_collision_time_seconds is not None
                and candidate.first_collision_time_seconds
                >= MANEUVER_MIN_PREDICTED_COLLISION_TIME_SECONDS
            )
            if (
                (not candidate.viable and not managed_defender_conflict)
                or abs(candidate.lateral_bias_m) <= 1e-9
                or (
                    not managed_defender_conflict
                    and candidate.minimum_opponent_clearance_m
                    < MANEUVER_PULL_OUT_MIN_CLEARANCE_M
                )
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
            battle.corner_inside_driver_id = (
                battle.attacker_id
                if battle.attacker_line == ATTACK_LINE_INSIDE
                else battle.defender_id
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
        if segment.type == TrackSegmentType.STRAIGHT and trajectory_decision is None:
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
        fuel_margin_laps = state.fuel_laps_remaining - remaining_laps
        compound_spec = COMPOUND_SPECS[state.tire_compound]
        hot_tire_threshold_c = (
            compound_spec.optimal_temperature_c
            + compound_spec.operating_window_c
            + 8.0
        )

        if tire_life <= AI_CRITICAL_TIRE_LIFE or fuel_margin_laps < 0.6:
            return PaceMode.CONSERVE

        if state.tire_surface_temperature_c >= hot_tire_threshold_c:
            return PaceMode.CONSERVE

        if self.race_phase != "green":
            return PaceMode.CONSERVE

        gap_ahead = self._gap_seconds_between(car_ahead, state)
        gap_behind = self._gap_seconds_between(state, car_behind)

        conserve_tire_threshold = (
            AI_CONSERVE_EXIT_TIRE_LIFE
            if state.pace_mode == PaceMode.CONSERVE
            else AI_LOW_TIRE_LIFE
        )
        if tire_life <= conserve_tire_threshold and not is_endgame and not pit_in_push:
            return PaceMode.CONSERVE

        if pit_in_push or fresh_after_stop:
            if fuel_margin_laps < 1.2:
                return PaceMode.STANDARD
            return PaceMode.ATTACK

        attack_gap_threshold = (
            AI_ATTACK_EXIT_GAP_SECONDS
            if state.pace_mode == PaceMode.ATTACK
            else AI_ATTACK_GAP_SECONDS
        )
        if gap_ahead is not None and gap_ahead <= attack_gap_threshold:
            if fuel_margin_laps < 1.2:
                return PaceMode.STANDARD
            return PaceMode.ATTACK if tire_life > AI_LOW_TIRE_LIFE else PaceMode.STANDARD

        defend_gap_threshold = (
            AI_DEFEND_EXIT_GAP_SECONDS
            if state.pace_mode == PaceMode.ATTACK
            else AI_DEFEND_GAP_SECONDS
        )
        if gap_behind is not None and gap_behind <= defend_gap_threshold:
            defending = self._driver_meta[state.driver_id]["defending"]
            if tire_life > AI_LOW_TIRE_LIFE and defending >= 0.82:
                return PaceMode.ATTACK
            return PaceMode.STANDARD

        if is_endgame:
            return PaceMode.ATTACK if tire_life > 0.35 else PaceMode.STANDARD

        return PaceMode.STANDARD

    def _build_pass_events(self, previous_positions: dict[int, int]) -> list[RaceEvent]:
        """Return pass events for drivers whose position improved this tick."""
        self._require_rules_phase("build pass events")
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

            physical_clearance_m = (
                state.total_progress - passed_state.total_progress
            ) * self.track_length_m
            required_clearance_m = 0.5 * (
                state.car_length_m + passed_state.car_length_m
            ) + MANEUVER_PASS_CLEARANCE_MARGIN_M
            if physical_clearance_m < required_clearance_m - PROGRESS_EPSILON:
                continue

            battle = self._side_by_side_battles.get(
                self._battle_pair_key(state.driver_id, passed_driver_id)
            )
            if battle is not None:
                # Live timing may swap while noses overlap. The feed announces
                # the pass from the maneuver's OVERLAP -> CLEAR transition.
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
                    payload={
                        "attacker_id": state.driver_id,
                        "defender_id": passed_state.driver_id,
                        "clearance_m": round(physical_clearance_m, 6),
                        "required_clearance_m": round(required_clearance_m, 6),
                    },
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

    def _phase_vehicle_speed_factor(
        self,
        state: DriverRaceState | None = None,
    ) -> float:
        """Convert race-control pace into a physically safe speed ceiling.

        A car catching the Safety Car queue may exceed its normal target only
        on a segment explicitly classified as straight.  Race control must
        never multiply the calibrated heavy-braking or corner speed above the
        green-flag envelope. Slower VSC, queued-SC and near-queue factors still
        apply fully.
        """
        requested = 1.0 / max(1e-6, self._phase_lap_time_factor(state))
        if self.race_phase == "sc" and requested > 1.0:
            segment = (
                segment_at_progress(self.circuit, state.progress)
                if state is not None
                else None
            )
            return (
                requested
                if segment is not None and segment.type == TrackSegmentType.STRAIGHT
                else 1.0
            )
        return requested

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
            self._cancel_maneuvers_for_neutralization()
            return
        target = self.race_elapsed + VSC_DURATION_SECONDS
        if self.race_phase == "vsc":
            self._cancel_maneuvers_for_neutralization()
            self._phase_until = max(self._phase_until, target)
            return
        self._cancel_maneuvers_for_neutralization()
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
        self._cancel_maneuvers_for_neutralization()
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

    def retire_driver_for_testing(self, driver_id: int) -> list[RaceEvent]:
        """Create a repeatable stopped-car retirement for browser integration tests."""
        state = self.driver_states.get(int(driver_id))
        if state is None:
            raise ValueError(f"Unknown driver: {driver_id}")
        if state.retired or state.finished:
            raise ValueError(f"Driver is not running: {driver_id}")
        events: list[RaceEvent] = []
        self._apply_incident(
            Incident(
                cause=IncidentCause.MECHANICAL,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=state.driver_id,
            ),
            events,
        )
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
        self._initialize_sc_running_order()
        leader = self._on_track_leader()
        if leader is None:
            self._safety_car_total_progress = None
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            return
        exit_progress = self._pit_exit_progress()
        if exit_progress is None:
            exit_progress = (leader.progress + SC_LEAD_PROGRESS_GAP) % 1.0
        self._safety_car_total_progress = self._total_progress_at_or_after(
            leader.total_progress,
            exit_progress,
        )
        self._safety_car_progress_rate = 0.0
        self._safety_car_speed_mps = 0.0
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

    def _sc_release_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_RELEASE_GAP_CAR_LENGTHS) / self.track_length_m

    def _sc_target_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS) / self.track_length_m

    def _safety_car_line_2_progress(self) -> float | None:
        """Return the circuit progress where a pit-out order becomes final."""
        pit_lane = self.circuit.pit_lane
        if pit_lane is None:
            return None
        if pit_lane.safety_car_line_2_progress is not None:
            return pit_lane.safety_car_line_2_progress % 1.0
        return self._pit_exit_progress()

    def _initialize_sc_running_order(self) -> None:
        self._sc_running_order = [
            state.driver_id
            for state in sorted(
                (
                    state
                    for state in self.driver_states.values()
                    if not state.retired and not state.finished and not state.in_pit
                ),
                key=lambda state: state.position,
            )
        ]
        self._sc_pit_exit_order_targets.clear()

    def _remove_from_sc_running_order(self, driver_id: int) -> None:
        self._sc_running_order = [
            item for item in self._sc_running_order if item != driver_id
        ]
        self._sc_pit_exit_order_targets.pop(driver_id, None)
        self._sc_caught_driver_ids.discard(driver_id)

    @staticmethod
    def _insert_by_live_race_distance(
        ordered: list[DriverRaceState],
        state: DriverRaceState,
    ) -> None:
        insert_at = len(ordered)
        for index, other in enumerate(ordered):
            ahead_by_distance = state.total_progress > (
                other.total_progress + PROGRESS_EPSILON
            )
            tied_ahead_by_order = (
                abs(state.total_progress - other.total_progress)
                <= PROGRESS_EPSILON
                and state.position < other.position
            )
            if ahead_by_distance or tied_ahead_by_order:
                insert_at = index
                break
        ordered.insert(insert_at, state)

    def _refresh_sc_running_order(self) -> None:
        """Drop ineligible entries and recover any untracked on-track cars."""
        eligible = {
            state.driver_id: state
            for state in self.driver_states.values()
            if not state.retired
            and not state.finished
            and not state.in_pit
            and state.driver_id not in self._sc_pit_exit_order_targets
        }
        refreshed = [
            driver_id
            for driver_id in self._sc_running_order
            if driver_id in eligible
        ]
        known = set(refreshed)
        missing = sorted(
            (
                state
                for driver_id, state in eligible.items()
                if driver_id not in known
            ),
            key=lambda state: state.position,
        )
        for state in missing:
            ordered_states = [eligible[driver_id] for driver_id in refreshed]
            self._insert_by_live_race_distance(ordered_states, state)
            refreshed = [item.driver_id for item in ordered_states]
        self._sc_running_order = refreshed

    def _sc_ordered_on_track_states(
        self,
        *,
        include_pending: bool = True,
    ) -> list[DriverRaceState]:
        self._refresh_sc_running_order()
        ordered = [
            self.driver_states[driver_id]
            for driver_id in self._sc_running_order
            if driver_id in self.driver_states
        ]
        if include_pending:
            pending = sorted(
                (
                    self.driver_states[driver_id]
                    for driver_id in self._sc_pit_exit_order_targets
                    if driver_id in self.driver_states
                    and not self.driver_states[driver_id].retired
                    and not self.driver_states[driver_id].finished
                    and not self.driver_states[driver_id].in_pit
                ),
                key=lambda state: (-state.total_progress, state.position),
            )
            for state in pending:
                self._insert_by_live_race_distance(ordered, state)
        return ordered

    def _begin_sc_pit_exit_ordering(self, state: DriverRaceState) -> None:
        """Keep a pit-out result provisional until the car reaches SC2."""
        if self.race_phase != "sc":
            return
        self._remove_from_sc_running_order(state.driver_id)
        line_progress = self._safety_car_line_2_progress()
        if line_progress is None:
            target = state.total_progress
        else:
            target = int(state.total_progress // 1.0) + line_progress
        self._sc_pit_exit_order_targets[state.driver_id] = target

    def _commit_ready_sc_pit_exit_orders(self) -> None:
        """Freeze each pit-out car into its physical order at the SC2 line."""
        if self.race_phase != "sc":
            return
        self._refresh_sc_running_order()
        ready = sorted(
            (
                self.driver_states[driver_id]
                for driver_id, target in self._sc_pit_exit_order_targets.items()
                if driver_id in self.driver_states
                and (
                    self.driver_states[driver_id].retired
                    or self.driver_states[driver_id].finished
                    or self.driver_states[driver_id].total_progress
                    >= target - PROGRESS_EPSILON
                )
            ),
            key=lambda state: (-state.total_progress, state.position),
        )
        for state in ready:
            self._sc_pit_exit_order_targets.pop(state.driver_id, None)
            if state.retired or state.finished or state.in_pit:
                continue
            ordered = [
                self.driver_states[driver_id]
                for driver_id in self._sc_running_order
                if driver_id in self.driver_states
            ]
            self._insert_by_live_race_distance(ordered, state)
            self._sc_running_order = [item.driver_id for item in ordered]
            self._sc_caught_driver_ids.discard(state.driver_id)

    def _sc_queue_predecessor(self, state: DriverRaceState) -> DriverRaceState | None:
        """Return the next eligible car ahead in the frozen SC running order."""
        running = [
            candidate
            for candidate in self._sc_ordered_on_track_states()
            if candidate.driver_id not in self._sc_unlap_driver_ids
        ]
        for index, candidate in enumerate(running):
            if candidate.driver_id != state.driver_id:
                continue
            return running[index - 1] if index > 0 else None
        return None

    def _sc_queue_control_reaches(
        self,
        state: DriverRaceState,
        visited: set[int] | None = None,
    ) -> bool:
        """Return whether the car is within the queue-tail control horizon."""
        return self._sc_queue_control_distance_m(state, visited) is not None

    def _sc_queue_control_distance_m(
        self,
        state: DriverRaceState,
        visited: set[int] | None = None,
    ) -> float | None:
        """Return cumulative distance back from the caught SC queue.

        The horizon is cumulative rather than 800 m per predecessor.  The
        latter chained across the whole field and slowed distant rear cars
        before they had caught the queue, preventing timely formation.
        """
        if (
            self.race_phase != "sc"
            or self.safety_car_stage in {"deploying", "restart"}
            or self._safety_car_total_progress is None
        ):
            return None
        if state.driver_id in self._sc_caught_driver_ids:
            return 0.0
        seen = set() if visited is None else visited
        if state.driver_id in seen:
            return None
        seen.add(state.driver_id)
        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            distance_ahead_m = 0.0
        else:
            ahead_total = predecessor.total_progress
            distance_ahead_m = self._sc_queue_control_distance_m(predecessor, seen)
            if distance_ahead_m is None:
                return None
        gap_m = (ahead_total - state.total_progress) * self.track_length_m
        if gap_m < 0.0:
            return None
        cumulative_distance_m = distance_ahead_m + gap_m
        if cumulative_distance_m > SC_QUEUE_PROPAGATION_HORIZON_M:
            return None
        return cumulative_distance_m

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
        state.total_distance_m = total * self.track_length_m

    def _advance_safety_car(self, delta: float, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self._safety_car_total_progress is None:
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            return

        if self.safety_car_stage == "deploying":
            remaining = 1.0 - self._safety_car_pit_lane_progress
            advance = delta * 0.18 / SC_DEPLOY_PIT_SECONDS
            self._safety_car_pit_lane_progress = min(1.0, self._safety_car_pit_lane_progress + advance)
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
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
            self._safety_car_speed_mps = 0.0
            self._safety_car_pit_lane_progress = min(
                1.0,
                self._safety_car_pit_lane_progress + delta / SC_WITHDRAW_PIT_SECONDS,
            )
            if self._safety_car_pit_lane_progress >= 0.35:
                self._safety_car_visible = False
            return

        previous = self._safety_car_total_progress
        safety_car_distance_m = self._track_physics.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            self._safety_car_total_progress,
        )
        safety_car_lap_time_factor = (
            SC_COLLECTION_LAP_TIME_FACTOR
            if self.safety_car_stage == "collecting"
            else SC_LAP_TIME_FACTOR
        )
        target_safety_car_speed_mps = self._vehicle_physics.target_speed_mps(
            safety_car_distance_m,
            VehiclePhysicsModifiers(
                speed_limit_factor=1.0 / safety_car_lap_time_factor,
                maximum_speed_mps=SC_CAUGHT_MAX_SPEED_KPH / 3.6,
            ),
        )
        if self.safety_car_stage == "collecting" and not self._sc_caught_driver_ids:
            leader = self._on_track_leader()
            if (
                leader is not None
                and self._safety_car_total_progress - leader.total_progress
                > 2.0 * self._sc_max_gap_progress()
            ):
                # If the SC exits just after the leader has passed the pit
                # exit, it waits at a controlled pace to pick that leader up
                # instead of forcing the field to chase almost a full lap.
                target_safety_car_speed_mps = min(
                    target_safety_car_speed_mps,
                    SC_LEADER_ACQUISITION_SPEED_KPH / 3.6,
                )
        speed_delta_mps = target_safety_car_speed_mps - self._safety_car_speed_mps
        maximum_speed_change_mps = (
            6.0 * delta if speed_delta_mps >= 0.0 else 8.0 * delta
        )
        self._safety_car_speed_mps += max(
            -maximum_speed_change_mps,
            min(maximum_speed_change_mps, speed_delta_mps),
        )
        self._safety_car_progress_rate = (
            self._safety_car_speed_mps / self.track_length_m
        )
        self._safety_car_total_progress += self._safety_car_progress_rate * delta

        if (
            self.safety_car_stage == "in_this_lap"
            and self._sc_withdraw_target is not None
            and previous < self._sc_withdraw_target <= self._safety_car_total_progress
        ):
            self._safety_car_total_progress = self._sc_withdraw_target
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
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
        self._safety_car_speed_mps = 0.0
        self.safety_car_stage = "inactive"
        self._safety_car_visible = False
        self._safety_car_route = "track"
        self._safety_car_pit_lane_progress = 0.0
        self._safety_car_queue_formed = False
        self._sc_cleanup_until = 0.0
        self._sc_caught_driver_ids.clear()
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._sc_running_order.clear()
        self._sc_pit_exit_order_targets.clear()
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

        running = self._sc_ordered_on_track_states()
        active_ids = {state.driver_id for state in running}
        self._sc_caught_driver_ids.intersection_update(active_ids)

        max_gap = self._sc_max_gap_progress()
        release_gap = self._sc_release_gap_progress()
        queue_locked = self.safety_car_stage == "queued"
        ahead_total = self._safety_car_total_progress
        ahead_is_queued = True
        ahead_speed_mps = max(0.0, self._safety_car_speed_mps)

        for state in running:
            if state.driver_id in self._sc_unlap_driver_ids:
                continue

            gap = ahead_total - state.total_progress
            caught = state.driver_id in self._sc_caught_driver_ids
            if caught and not queue_locked and (
                not ahead_is_queued
                or gap < -PROGRESS_EPSILON
                or gap > release_gap + PROGRESS_EPSILON
            ):
                self._sc_caught_driver_ids.discard(state.driver_id)
                caught = False

            if (
                ahead_is_queued
                and gap >= -PROGRESS_EPSILON
                and gap <= max_gap + PROGRESS_EPSILON
                and (
                    (caught and queue_locked)
                    or max(0.0, state.speed_kph / 3.6 - ahead_speed_mps)
                    <= SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH / 3.6
                )
            ):
                self._sc_caught_driver_ids.add(state.driver_id)
                caught = True
            else:
                caught = False

            ahead_total = state.total_progress
            ahead_is_queued = caught
            ahead_speed_mps = max(0.0, state.speed_kph / 3.6)

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

    def _physical_track_distance_m(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
    ) -> float:
        progress_gap = abs((first.progress % 1.0) - (second.progress % 1.0))
        return min(progress_gap, 1.0 - progress_gap) * self.track_length_m

    def _tick_stopped_hazard_clearance(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc":
            return
        running = [
            state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        ]
        active_hazards = list(self._active_stopped_hazards())
        blocked_clear_ids: set[int] = set()
        assessed_cluster_ids: set[int] = set()
        for hazard in active_hazards:
            if hazard.driver_id in assessed_cluster_ids:
                continue
            cluster = self._stopped_hazard_cluster(hazard)
            cluster_ids = {item.driver_id for item in cluster}
            assessed_cluster_ids.update(cluster_ids)
            latest_activation = max(
                self._hazard_activated_at.get(
                    item.driver_id,
                    self.race_elapsed,
                )
                for item in cluster
            )
            if (
                not self._hazard_cluster_has_passable_corridor(cluster)
                and self.race_elapsed - latest_activation
                >= HAZARD_BLOCKED_CLEAR_SECONDS
            ):
                blocked_clear_ids.update(cluster_ids)

        for hazard in active_hazards:
            tail_target = self._hazard_tail_targets.get(hazard.driver_id)
            tail_passed = False
            if tail_target is not None:
                tail_driver_id, target_progress = tail_target
                tail = self.driver_states.get(tail_driver_id)
                tail_passed = bool(
                    tail is not None
                    and not tail.retired
                    and tail.total_progress >= target_progress - PROGRESS_EPSILON
                )
            no_car_within_radius = not any(
                self._physical_track_distance_m(hazard, state)
                <= HAZARD_CLEAR_RADIUS_M
                for state in running
            )
            blocked_clearance = hazard.driver_id in blocked_clear_ids
            if (
                not tail_passed
                and not no_car_within_radius
                and not blocked_clearance
            ):
                continue
            hazard.hazard_active = False
            hazard.vehicle_status = "cleared"
            self._hazard_tail_targets.pop(hazard.driver_id, None)
            self._hazard_activated_at.pop(hazard.driver_id, None)
            meta = self._driver_meta[hazard.driver_id]
            reason = (
                "the tail of the field has passed"
                if tail_passed
                else (
                    "the blocked circuit has been neutralized under Safety Car"
                    if blocked_clearance
                    else "the 300m safety zone is clear"
                )
            )
            reason_ko = (
                "최후미 차량이 사고 지점을 통과해"
                if tail_passed
                else (
                    "통과할 물리적 공간이 없어 세이프티카 아래 사고 구역을 통제한 뒤"
                    if blocked_clearance
                    else "반경 300m에 주행 차량이 없어"
                )
            )
            events.append(
                RaceEvent(
                    type="hazard_cleared",
                    driver=meta["abbreviation"],
                    message=(
                        f"{meta['full_name']}'s stopped car is removed after {reason}"
                    ),
                    message_ko=(
                        f"{reason_ko} {meta['full_name']}의 정지 차량을 제거합니다"
                    ),
                )
            )

    def _tick_safety_car_after_cars(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc":
            return

        self._tick_stopped_hazard_clearance(events)
        self._sync_safety_car_queue(events)

        if (
            self.safety_car_stage == "queued"
            and self._safety_car_queue_formed
            and self.race_elapsed >= self._sc_cleanup_until
            and not self._active_stopped_hazards()
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
        stopped_states = [primary]
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
                stopped_states.append(secondary)
                smeta = self._driver_meta[incident.secondary_driver_id]
                events.append(
                    RaceEvent(
                        type="retirement",
                        driver=smeta["abbreviation"],
                        message=f"{smeta['full_name']} is collected in the incident",
                        message_ko=f"{smeta['full_name']}가 사고에 휘말려 리타이어합니다",
                    )
                )

        for stopped_state in stopped_states:
            self._activate_stopped_hazard(stopped_state, incident.cause)
        self._trigger_safety_car(events)

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

    def get_grid_slots(self) -> list[dict]:
        """Return the exact staggered boxes used by the vehicle simulation."""
        return [
            {
                "position": state.position,
                "progress": self._grid_start_progress[state.driver_id] % 1.0,
                "lateral_offset_m": self._grid_lateral_offsets[state.driver_id],
            }
            for state in sorted(self.driver_states.values(), key=lambda item: item.position)
        ]

    def _advance_start_sequence(self, delta: float) -> list[RaceEvent]:
        """Hold the field in its boxes and release everyone on lights out."""
        self._start_sequence_elapsed += max(0.0, delta)
        if self._start_sequence_elapsed < GRID_FIRST_LIGHT_SECONDS:
            self.start_sequence_phase = "grid"
            self.start_light_count = 0
            return []

        illuminated = int(
            (self._start_sequence_elapsed - GRID_FIRST_LIGHT_SECONDS)
            / GRID_LIGHT_INTERVAL_SECONDS
        ) + 1
        self.start_sequence_phase = "lights"
        self.start_light_count = min(5, max(1, illuminated))
        if self._start_sequence_elapsed < GRID_LIGHTS_OUT_SECONDS:
            return []

        self.race_started = True
        self.start_sequence_phase = "lights_out"
        self.start_light_count = 0
        self._lights_out_display_remaining = GRID_LIGHTS_OUT_DISPLAY_SECONDS
        self._start_overtake_lockout_remaining = GRID_OVERTAKE_LOCKOUT_SECONDS
        for state in self.driver_states.values():
            state.speed_kph = 0.0
            state.acceleration_mps2 = 0.0
            state.target_speed_kph = 0.0
            state.throttle = 0.0
            state.brake = 0.0
        return [
            RaceEvent(
                type="race_start",
                driver="",
                message="Lights out — the race is underway",
                message_ko="라이트 아웃 — 레이스가 시작됩니다",
            )
        ]

    def _tick_start_display(self, delta: float) -> None:
        if self.start_sequence_phase != "lights_out":
            return
        self._lights_out_display_remaining = max(
            0.0,
            self._lights_out_display_remaining - delta,
        )
        if self._lights_out_display_remaining <= 0.0:
            self.start_sequence_phase = "racing"

    def set_speed(self, multiplier: int) -> bool:
        if multiplier not in (1, 2):
            return False
        self.speed_multiplier = multiplier
        return True

    def pause_race(self) -> None:
        self.paused = True

    def resume_race(self) -> None:
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
        self._require_tick_phase(TickPhase.RULES, "pit stop state transition")
        lane_time, tire_change_time = compute_pit_components(
            self.circuit.pit_loss_time,
            team.pit_crew_skill,
            self.rng,
        )
        half_lane = lane_time / 2.0
        # A car leaving the racing surface cannot remain in an on-track
        # maneuver graph; stale edges would override live pit ordering.
        self._clear_side_by_side_for_driver(driver_id)
        self._pending_overtake_commands.pop(driver_id, None)
        if self.race_phase == "sc":
            self._remove_from_sc_running_order(driver_id)
        state.in_pit = True
        self._pit_phase[driver_id] = "in"
        self._pit_phase_duration[driver_id] = 0.0
        self._pit_phase_remaining[driver_id] = 0.0
        self._pit_lane_half_time[driver_id] = half_lane
        self._pit_route_progress[driver_id] = 0.0
        self._pit_route_speed_mps[driver_id] = max(0.0, state.speed_kph / 3.6)
        self._pit_merge_state[driver_id] = "approach"
        self._pit_merge_conflict_driver_id[driver_id] = None
        self._pit_merge_conflict_group_id[driver_id] = None
        self._pit_merge_conflict_group_member_ids[driver_id] = ()
        self._pit_tire_change_time[driver_id] = tire_change_time
        self._pit_elapsed[driver_id] = 0.0
        self._pit_stop_elapsed[driver_id] = 0.0
        self._pit_tire[driver_id] = tire
        self._pit_entry_compound[driver_id] = state.tire_compound
        state.grip_utilization = 0.0
        state.handling_state = "stable"
        state.slip_angle_rad = 0.0
        state.wheel_lock_ratio = 0.0
        state.traction_slip_ratio = 0.0
        state.tire_slide_energy_j = 0.0
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
        self._apply_pace_mode(state, pace_mode)
        return None

    def tick(self, delta_game_seconds: float | None = None) -> list[RaceEvent]:
        """Advance by elapsed game time through the residual-time accumulator."""
        if self.finished or self.paused:
            return []

        delta = (
            GAME_TICK_SECONDS
            if delta_game_seconds is None
            else max(0.0, float(delta_game_seconds))
        )
        if not self.race_started:
            return self._advance_start_sequence(delta)
        self._setup_mode = False
        progress_before = {
            driver_id: state.total_progress
            for driver_id, state in self.driver_states.items()
        }
        fixed_steps = self._physics_accumulator.consume(delta)
        events: list[RaceEvent] = []
        for _ in range(fixed_steps):
            if self.finished or self.paused:
                break
            events.extend(self._tick_fixed_step(PHYSICS_STEP_SECONDS))
        if fixed_steps and self.race_phase != "green":
            for driver_id, state in self.driver_states.items():
                profile = self._track_physics_for_driver(state)
                state.target_lateral_offset_m = profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    progress_before.get(driver_id, state.progress),
                )
        return events

    def _tick_fixed_step(self, delta: float) -> list[RaceEvent]:
        """Run one complete authoritative 50 Hz tick."""
        if abs(delta - PHYSICS_STEP_SECONDS) > 1e-9:
            raise ValueError("authoritative physics ticks must be exactly 0.02 seconds")
        self._tick_start_display(delta)
        self._start_overtake_lockout_remaining = max(
            0.0,
            self._start_overtake_lockout_remaining - delta,
        )
        self.race_elapsed += PHYSICS_STEP_SECONDS
        self._physics_frame += 1
        self._physics_step_deltas.append(delta)
        self._tick_phase_history = []
        self._enter_tick_phase(TickPhase.COMMAND)
        events: list[RaceEvent] = []
        progress_facts: list[ProgressCrossingFact] = []
        in_pit_driver_ids: list[int] = []
        self._tick_collision_states(delta)
        previous_positions = {
            driver_id: state.position
            for driver_id, state in self.driver_states.items()
        }
        racing = self.race_phase == "green"
        self._tick_battle_cooldowns(delta)
        running_by_position = self._running_by_position()
        physics_start_snapshot = {
            state.driver_id: (
                state.total_progress,
                max(0.0, state.speed_kph / 3.6),
            )
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        }
        collision_start_poses = {
            state.driver_id: self._body_pose(state)
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        }
        self._tick_ai_pace_cooldowns(delta)
        self._run_ai_pace_modes(running_by_position)
        self._tick_pace_mode_transitions(delta)
        self._ai_tactical_decision_elapsed = round(
            self._ai_tactical_decision_elapsed + delta,
            12,
        )
        tactical_decision_due = (
            self._ai_tactical_decision_elapsed
            >= AI_TACTICAL_DECISION_INTERVAL_SECONDS - 1e-9
        )
        tactical_decision_delta = 0.0
        if tactical_decision_due:
            tactical_decision_delta = self._ai_tactical_decision_elapsed
            self._ai_tactical_decision_elapsed = 0.0
            self._pending_overtake_commands = {}
        elif not racing:
            self._pending_overtake_commands.clear()
        command_battle_intents: list[BattleIntent] = []
        if racing and tactical_decision_due:
            # Build a straight-line command before evaluating the visible
            # battle intent.  The command is the physical input for this
            # PHYSICS step; creating it after ``_maybe_battle_event`` meant
            # the intent could never consume the same-frame trajectory.
            for state in self.driver_states.values():
                car_ahead = running_by_position.get(state.position - 1)
                segment = (
                    segment_at_progress(self.circuit, state.progress)
                    if car_ahead is not None
                    else None
                )
                if (
                    car_ahead is None
                    or segment is None
                    or segment.type != TrackSegmentType.STRAIGHT
                    or not segment.side_by_side_allowed
                    or not self._overtaking_candidate_allowed(state, car_ahead)
                    or not self._overtake_opportunity_is_viable(state, car_ahead)
                ):
                    continue
                decision = self._local_pull_out_decision(state, car_ahead)
                if decision is None:
                    track_profile = self._track_physics_for_driver(state)
                    fallback_line = self._choose_attack_line(
                        state,
                        car_ahead,
                    )
                    racing_offset = track_profile.line_offset_at_progress(
                        DRIVING_LINE_RACING,
                        state.progress,
                    )
                    target_offset = track_profile.line_offset_at_progress(
                        fallback_line,
                        state.progress,
                    )
                    decision = LocalPullOutDecision(
                        candidate_id="battle_intent_command",
                        lateral_bias_m=target_offset - racing_offset,
                        target_lateral_offset_m=target_offset,
                        minimum_clearance_m=abs(
                            target_offset - car_ahead.lateral_offset_m
                        )
                        - 0.5 * (state.car_width_m + car_ahead.car_width_m),
                        attacker_line=fallback_line,
                    )
                if decision.minimum_clearance_m >= MANEUVER_PULL_OUT_MIN_CLEARANCE_M:
                    self._pending_overtake_commands[state.driver_id] = (
                        PendingOvertakeCommand(
                            attacker_id=state.driver_id,
                            defender_id=car_ahead.driver_id,
                            decision=decision,
                        )
                    )

            for state in self.driver_states.values():
                car_ahead = running_by_position.get(state.position - 1)
                if car_ahead is None:
                    continue
                intent = self._maybe_battle_event(
                    state,
                    car_ahead,
                    commit=False,
                    delta_seconds=tactical_decision_delta,
                )
                if isinstance(intent, BattleIntent):
                    command_battle_intents.append(intent)
                    if intent.event_kind in {"grid_attack", "grid_defend"}:
                        continue
                    decision = intent.trajectory_decision
                    intent_attacker_line = intent.attacker_line
                else:
                    segment = segment_at_progress(self.circuit, state.progress)
                    if (
                        segment is None
                        or not self._overtaking_candidate_allowed(state, car_ahead)
                        or not self._overtake_opportunity_is_viable(state, car_ahead)
                    ):
                        continue
                    if segment.type == TrackSegmentType.HEAVY_BRAKING:
                        intent_attacker_line = self._choose_attack_line(
                            state,
                            car_ahead,
                        )
                    elif (
                        segment.type == TrackSegmentType.STRAIGHT
                        and segment.side_by_side_allowed
                    ):
                        decision = self._local_pull_out_decision(state, car_ahead)
                        intent_attacker_line = (
                            decision.attacker_line
                            if decision is not None
                            else self._choose_attack_line(state, car_ahead)
                        )
                    else:
                        continue
                    if segment.type == TrackSegmentType.HEAVY_BRAKING:
                        decision = None
                if decision is None:
                    track_profile = self._track_physics_for_driver(state)
                    racing_offset = track_profile.line_offset_at_progress(
                        DRIVING_LINE_RACING,
                        state.progress,
                    )
                    target_offset = track_profile.line_offset_at_progress(
                        intent_attacker_line,
                        state.progress,
                    )
                    decision = LocalPullOutDecision(
                        candidate_id="battle_intent_command",
                        lateral_bias_m=target_offset - racing_offset,
                        target_lateral_offset_m=target_offset,
                        minimum_clearance_m=abs(
                            target_offset - car_ahead.lateral_offset_m
                        )
                        - 0.5
                        * (state.car_width_m + car_ahead.car_width_m),
                        attacker_line=intent_attacker_line,
                    )
                self._pending_overtake_commands[state.driver_id] = (
                    PendingOvertakeCommand(
                        attacker_id=state.driver_id,
                        defender_id=car_ahead.driver_id,
                        decision=decision,
                    )
                )

        self._enter_tick_phase(TickPhase.PHYSICS)

        iteration_states = sorted(
            self.driver_states.values(),
            key=lambda item: item.position,
        )
        self._planner_spatial_index.rebuild(iteration_states)

        for state in iteration_states:
            driver_id = state.driver_id
            self._reset_avoidance_state(state)
            if state.retired or state.finished:
                self._progress_rate[driver_id] = 0.0
                state.speed_kph = 0.0
                continue

            meta = self._driver_meta[driver_id]
            team = self.teams[self.drivers[driver_id].team_id]

            if state.in_pit:
                self._progress_rate[driver_id] = 0.0
                in_pit_driver_ids.append(driver_id)
                continue

            pace_effect = self._pace_mode_effect(state)
            car_ahead = running_by_position.get(state.position - 1)
            car_behind = running_by_position.get(state.position + 1)
            previous_progress = state.progress
            state.racing_line = self._physics_v2_virtual_line(state)
            progress_delta = self._physics_v2_progress_delta(
                state,
                delta,
                car_ahead,
                car_behind,
                physics_start_snapshot,
            )
            self._progress_rate[driver_id] = progress_delta / delta if delta > 0 else 0.0
            state.progress += progress_delta
            progress_facts.append(
                ProgressCrossingFact(
                    driver_id=driver_id,
                    previous_progress=previous_progress,
                    progress_delta=progress_delta,
                )
            )
            tire_usage_multiplier = (
                pace_effect["tire_usage_multiplier"]
                * self._battle_effect_tire_usage_multiplier(state)
            )
            state.tire_usage += (
                progress_delta * tire_usage_multiplier
                + state.tire_slide_energy_j * TIRE_SLIDE_WEAR_LAPS_PER_JOULE
            )
            state.total_time += delta
            state.total_progress = state.current_lap + state.progress
            state.tire_wear = self._current_tire_wear(state)
            self._record_timing_loop_crossings(
                state,
                state.current_lap + previous_progress,
                state.total_progress,
                state.total_time - delta,
                delta,
            )

        collision_facts = self._resolve_vehicle_collisions(
            collision_start_poses,
            delta,
            collect_facts=True,
        )
        if not all(isinstance(fact, CollisionFact) for fact in collision_facts):
            raise RuntimeError("PHYSICS collision resolver returned non-physical events")
        physics_result = PhysicsStepResult(
            physics_frame_id=self._physics_frame,
            delta_seconds=delta,
            progress_crossings=tuple(progress_facts),
            collision_facts=tuple(collision_facts),
        )
        self._enter_tick_phase(TickPhase.RULES)
        self._consume_physics_step_result(physics_result)
        if self._pending_forced_wide_events:
            events.extend(self._pending_forced_wide_events)
            self._pending_forced_wide_events.clear()
        if self._pending_physical_handling_events:
            events.extend(self._pending_physical_handling_events)
            self._pending_physical_handling_events.clear()
        self._tick_race_phase(delta, events)
        if racing:
            events.extend(self._tick_forced_wide_aftermaths(delta))
        self._apply_collision_facts(list(physics_result.collision_facts), events)
        for driver_id in in_pit_driver_ids:
            state = self.driver_states.get(driver_id)
            if state is not None and state.in_pit:
                self._tick_in_pit(
                    driver_id,
                    state,
                    self._driver_meta[driver_id],
                    delta,
                    events,
                )

        for fact in physics_result.progress_crossings:
            state = self.driver_states.get(fact.driver_id)
            if state is None or state.retired or state.finished or state.in_pit:
                continue
            meta = self._driver_meta[fact.driver_id]
            team = self.teams[self.drivers[fact.driver_id].team_id]
            if self._should_enter_pit(
                state,
                fact.previous_progress,
                fact.progress_delta,
            ):
                tire = state.pit_request
                state.pit_request = None
                entry = self._pit_entry_progress()
                if tire is not None and entry is not None:
                    self._progress_rate[fact.driver_id] = 0.0
                    state.progress = entry
                    state.total_progress = state.current_lap + state.progress
                    state.total_distance_m = state.total_progress * self.track_length_m
                    self._start_pit_stop(fact.driver_id, state, meta, team, tire, events)
                continue
            while state.progress >= 1.0 and not state.in_pit and not state.finished:
                progress_after_line = state.progress - 1.0
                lap_finish_time = state.total_time
                if fact.progress_delta > 0:
                    lap_finish_time -= (
                        progress_after_line / fact.progress_delta
                    ) * delta
                state.progress -= 1.0
                events.extend(
                    self._complete_lap(
                        fact.driver_id,
                        state,
                        meta,
                        team,
                        lap_finish_time,
                    )
                )

        if self._pending_incidents:
            for incident in self._pending_incidents:
                self._apply_incident(incident, events)
            self._pending_incidents.clear()
        for state in self.driver_states.values():
            if state.retired or state.finished or state.in_pit:
                continue
            event_result = roll_events(
                self._driver_meta[state.driver_id]["abbreviation"],
                self._driver_meta[state.driver_id]["full_name"],
                self.rng,
                enable_events=False,
            )
            if event_result.retired:
                state.speed_kph = 0.0
                state.retired = True
                if event_result.event:
                    events.append(event_result.event)
            elif event_result.time_penalty > 0:
                state.total_time += event_result.time_penalty
                if event_result.event:
                    events.append(event_result.event)

            if racing and not state.retired and not state.finished:
                incident = roll_solo_incident(
                    self.rng,
                    state.driver_id,
                    delta_seconds=delta,
                    consistency=self._driver_meta[state.driver_id]["consistency"],
                    tire_wear=self._current_tire_wear(state),
                    segment_type=self._segment_type_at(state),
                    attack_mode=state.pace_mode == PaceMode.ATTACK,
                    dirty_air=state.dirty_air_active,
                    race_progress=self._race_progress(),
                )
                if incident is not None:
                    self._apply_incident(incident, events)

        if racing:
            wake_update_due = self._physics_frame % 2 == 0
            for state in self.driver_states.values():
                if wake_update_due:
                    car_ahead = running_by_position.get(state.position - 1)
                    self._update_wake_state(
                        state,
                        car_ahead,
                        iteration_states,
                    )
            for intent in command_battle_intents:
                battle_event = self._commit_battle_intent(intent)
                if battle_event is not None:
                    events.append(battle_event)
        self._run_ai_strategy()
        self._resolve_physics_v2_same_line_gaps()
        self._update_positions()
        self._tick_safety_car_after_cars(events)
        self._resolve_physics_v2_same_line_gaps()
        self._reconcile_progress_rates(physics_start_snapshot, delta)
        self._update_positions()
        events.extend(self._tick_side_by_side_battles(delta))
        for state in self.driver_states.values():
            battle = self._side_by_side_battle_for_driver(state.driver_id)
            if battle is None:
                continue
            state.racing_line = (
                battle.attacker_line
                if state.driver_id == battle.attacker_id
                else battle.defender_line
            )
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

        self._update_authoritative_vehicle_telemetry(delta)
        self._finalize_event_positions(events)
        self._enter_tick_phase(TickPhase.TELEMETRY)
        self._refresh_vehicle_telemetry(delta)

        return events

    def _finalize_event_positions(self, events: list[RaceEvent]) -> None:
        """Attach final RULES-state poses to events that identify a vehicle."""
        for event in events:
            driver_id = event.payload.get("attacker_id")
            if not isinstance(driver_id, int) and event.type == "pit_exit":
                driver_id = event.payload.get("driver_id")
            if not isinstance(driver_id, int):
                continue
            state = self.driver_states.get(driver_id)
            if state is None:
                continue
            event.payload.update(
                {
                    "world_x_m": state.world_x_m,
                    "world_y_m": state.world_y_m,
                    "heading_rad": state.heading_rad,
                }
            )

    def _tick_in_pit(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        delta: float,
        events: list[RaceEvent],
    ) -> None:
        """Integrate the car along the physical pit route by distance and speed."""
        state.total_time += delta
        self._pit_elapsed[driver_id] += delta
        phase = self._pit_phase[driver_id]
        if phase == "stop":
            self._pit_merge_state[driver_id] = "stop"
            self._pit_stop_elapsed[driver_id] += delta
            self._pit_phase_remaining[driver_id] = max(
                0.0,
                self._pit_phase_remaining[driver_id] - delta,
            )
            state.speed_kph = 0.0
            state.acceleration_mps2 = 0.0
            state.throttle = 0.0
            state.brake = 1.0
            if self._pit_phase_remaining[driver_id] <= 0.0:
                new_compound = self._pit_tire[driver_id]
                state.tire_compound = new_compound
                state.tire_age = 0
                state.tire_usage = 0.0
                state.tire_wear = 0.0
                blanket_temperature_c = tire_blanket_temperature_c(new_compound)
                state.tire_surface_temperature_c = blanket_temperature_c
                state.tire_core_temperature_c = blanket_temperature_c
                state.tire_thermal_grip = 1.0
                self._pit_phase[driver_id] = "out"
                self._pit_phase_duration[driver_id] = 0.0
                self._pit_route_speed_mps[driver_id] = 0.0
            self._sync_pit_race_progress(driver_id, state)
            return

        pit_lane = self.circuit.pit_lane
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or route_length_m <= 1e-6:
            return

        progress = self._pit_route_progress.get(driver_id, 0.0)
        speed_mps = self._pit_route_speed_mps.get(driver_id, state.speed_kph / 3.6)
        pit_speed_limit_kph = pit_lane.speed_limit_kph
        limit_mps = pit_speed_limit_kph / 3.6
        assigned_box_progress = self._pit_box_progress_for_driver(driver_id)
        acceleration_mps2 = 0.0
        landmark_progress: float
        landmark_speed_mps: float

        if phase == "in" and progress < pit_lane.speed_limit_start:
            self._pit_merge_state[driver_id] = "approach"
            landmark_progress = pit_lane.speed_limit_start
            landmark_speed_mps = limit_mps
            distance_m = max(1e-6, (landmark_progress - progress) * route_length_m)
            required_acceleration_mps2 = (
                landmark_speed_mps**2 - speed_mps**2
            ) / (2.0 * distance_m)
            # Use the kinematic acceleration required to arrive at the line at
            # exactly the event speed limit. The old bang-bang controller always applied
            # at least 7 m/s² of braking, reached the limit too early, then
            # oscillated around the configured limit before the line.
            acceleration_mps2 = max(
                -PIT_ENTRY_BRAKING_MPS2,
                min(PIT_LANE_ACCELERATION_MPS2, required_acceleration_mps2),
            )
        elif phase == "in":
            self._pit_merge_state[driver_id] = "limited"
            landmark_progress = assigned_box_progress
            landmark_speed_mps = 0.0
            distance_m = max(0.0, (landmark_progress - progress) * route_length_m)
            braking_envelope = sqrt(2.0 * PIT_BOX_BRAKING_MPS2 * distance_m)
            desired_speed_mps = min(limit_mps, braking_envelope)
            if speed_mps > desired_speed_mps + 1e-6:
                acceleration_mps2 = -PIT_BOX_BRAKING_MPS2
            elif speed_mps < desired_speed_mps - 1e-6:
                acceleration_mps2 = PIT_LANE_ACCELERATION_MPS2
        elif progress < pit_lane.speed_limit_end:
            self._pit_merge_state[driver_id] = "limited"
            landmark_progress = pit_lane.speed_limit_end
            landmark_speed_mps = limit_mps
            if speed_mps < limit_mps - 1e-6:
                acceleration_mps2 = PIT_LANE_ACCELERATION_MPS2
            elif speed_mps > limit_mps + 1e-6:
                acceleration_mps2 = -PIT_BOX_BRAKING_MPS2
        else:
            previous_merge_state = self._pit_merge_state.get(driver_id)
            merge_decision = self._pit_rejoin_decision(
                driver_id,
                state,
                progress,
                speed_mps,
            )
            merge_state = merge_decision.state
            conflict_driver_id = merge_decision.conflict_driver_id
            self._pit_merge_state[driver_id] = merge_state
            self._pit_merge_conflict_driver_id[driver_id] = conflict_driver_id
            self._pit_merge_conflict_group_id[driver_id] = (
                merge_decision.conflict_group_id
            )
            self._pit_merge_conflict_group_member_ids[driver_id] = (
                merge_decision.conflict_group_member_ids
            )
            if (
                merge_state in {"yield", "hold"}
                and merge_state != previous_merge_state
            ):
                conflict_name = (
                    self._driver_meta[conflict_driver_id]["abbreviation"]
                    if conflict_driver_id in self._driver_meta
                    else "traffic"
                )
                events.append(
                    RaceEvent(
                        type=f"pit_merge_{merge_state}",
                        driver=meta["abbreviation"],
                        message=(
                            f"{meta['full_name']} {merge_state}s for {conflict_name} "
                            "at pit exit"
                        ),
                        message_ko=(
                            f"{meta['full_name']}가 피트 출구에서 {conflict_name} 차량에 "
                            f"{('양보합니다' if merge_state == 'yield' else '대기합니다')}"
                        ),
                        payload={
                            "driver_id": driver_id,
                            "conflict_driver_id": conflict_driver_id or 0,
                            "conflict_group_id": (
                                merge_decision.conflict_group_id or ""
                            ),
                            "conflict_group_member_ids": ",".join(
                                str(item)
                                for item in merge_decision.conflict_group_member_ids
                            ),
                            "merge_state": merge_state,
                        },
                    )
                )
            if merge_state == "merge":
                landmark_progress = pit_lane.side_rejoin_progress
                landmark_speed_mps = speed_mps
                acceleration_mps2 = PIT_EXIT_ACCELERATION_MPS2
            else:
                hold_progress = max(
                    progress,
                    pit_lane.side_rejoin_progress
                    - PIT_MERGE_HOLD_DISTANCE_M / route_length_m,
                )
                landmark_progress = hold_progress
                landmark_speed_mps = 0.0
                distance_m = max(0.0, (hold_progress - progress) * route_length_m)
                braking_envelope = sqrt(
                    2.0 * PIT_MERGE_BRAKING_MPS2 * distance_m
                )
                if speed_mps > braking_envelope + 1e-6:
                    acceleration_mps2 = -PIT_MERGE_BRAKING_MPS2
                elif distance_m <= 1e-6:
                    acceleration_mps2 = -PIT_MERGE_BRAKING_MPS2

        next_speed_mps = max(0.0, speed_mps + acceleration_mps2 * delta)
        if phase == "in" and progress >= pit_lane.speed_limit_start:
            next_speed_mps = min(limit_mps, next_speed_mps)
        elif phase == "out" and progress < pit_lane.speed_limit_end:
            next_speed_mps = min(limit_mps, next_speed_mps)
        distance_step_m = max(0.0, 0.5 * (speed_mps + next_speed_mps) * delta)
        next_progress = progress + distance_step_m / route_length_m

        if next_progress >= landmark_progress - 1e-12:
            next_progress = landmark_progress
            next_speed_mps = landmark_speed_mps
            if phase == "in" and landmark_progress == pit_lane.speed_limit_start:
                next_speed_mps = limit_mps
            elif phase == "in" and landmark_progress == assigned_box_progress:
                next_speed_mps = 0.0
                self._pit_phase[driver_id] = "stop"
                duration = self._pit_tire_change_time[driver_id]
                self._pit_phase_duration[driver_id] = duration
                self._pit_phase_remaining[driver_id] = duration
            elif phase == "out" and landmark_progress == pit_lane.speed_limit_end:
                next_speed_mps = limit_mps

        self._pit_route_progress[driver_id] = next_progress
        self._pit_route_speed_mps[driver_id] = next_speed_mps
        state.speed_kph = next_speed_mps * 3.6
        state.acceleration_mps2 = acceleration_mps2
        state.target_speed_kph = (
            pit_speed_limit_kph
            if next_progress <= pit_lane.speed_limit_end
            else state.speed_kph
        )
        state.throttle = 1.0 if acceleration_mps2 > 0.0 else 0.0
        state.brake = min(1.0, max(0.0, -acceleration_mps2 / PIT_ENTRY_BRAKING_MPS2))
        self._sync_pit_race_progress(driver_id, state)

        if phase == "out" and next_progress >= pit_lane.side_rejoin_progress:
            self._finish_pit_rejoin(driver_id, state, meta, events)

    def _pit_rejoin_decision(
        self,
        driver_id: int,
        state: DriverRaceState,
        lane_progress: float,
        speed_mps: float,
    ) -> PitMergeDecision:
        """Choose MERGE, YIELD, or HOLD with main-track group priority."""
        pit_lane = self.circuit.pit_lane
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or entry is None or exit_ is None or route_length_m <= 1e-6:
            return PitMergeDecision("merge")

        rejoin_progress = pit_lane.side_rejoin_progress
        remaining_m = max(0.0, (rejoin_progress - lane_progress) * route_length_m)
        if speed_mps > 1.0:
            time_to_rejoin_s = remaining_m / speed_mps
        else:
            time_to_rejoin_s = sqrt(
                2.0 * remaining_m / max(0.1, PIT_EXIT_ACCELERATION_MPS2)
            )
        time_to_rejoin_s = min(5.0, max(PHYSICS_STEP_SECONDS, time_to_rejoin_s))

        mapped_total_progress = (
            state.current_lap
            + entry
            + self._progress_distance(entry, exit_) * rejoin_progress
        )
        track_profile = self._track_physics_for_driver(state)
        track_progress = mapped_total_progress % 1.0
        pit_x, pit_y, pit_heading = self._pit_lane_pose_at_progress_m(
            rejoin_progress,
            track_profile,
        )
        line_length_m = max(1.0, track_profile.length_for_line(DRIVING_LINE_RACING))
        for _ in range(4):
            line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                track_progress,
            )
            tangent_delta_m = (
                (pit_x - line_x) * cos(line_heading)
                + (pit_y - line_y) * sin(line_heading)
            )
            track_progress = (track_progress + tangent_delta_m / line_length_m) % 1.0
        mapped_fraction = mapped_total_progress % 1.0
        refinement_delta = (
            track_progress - mapped_fraction + 0.5
        ) % 1.0 - 0.5
        target_total_progress = mapped_total_progress + refinement_delta
        line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        lateral_offset_m = track_profile.line_offset_at_progress(
            DRIVING_LINE_RACING,
            track_progress,
        ) + (
            (pit_x - line_x) * -sin(line_heading)
            + (pit_y - line_y) * cos(line_heading)
        )
        pit_pose = BodyPose(
            longitudinal_m=target_total_progress * self.track_length_m,
            lateral_m=lateral_offset_m,
            heading_rad=(pit_heading - line_heading + pi) % (2.0 * pi) - pi,
            length_m=state.car_length_m + 2.0 * PIT_MERGE_LONGITUDINAL_MARGIN_M,
            width_m=state.car_width_m + 2.0 * PIT_MERGE_LATERAL_MARGIN_M,
            longitudinal_speed_mps=max(0.0, speed_mps),
        )

        conflicts: list[tuple[float, DriverRaceState, ManeuverGroup | None]] = []
        for other in self.driver_states.values():
            if (
                other.driver_id == driver_id
                or other.retired
                or other.finished
                or other.in_pit
            ):
                continue
            maneuver_group = self._maneuver_group_for_driver(other.driver_id)
            time_offsets = (
                (-PIT_MERGE_GROUP_CLEARANCE_SECONDS, 0.0, PIT_MERGE_GROUP_CLEARANCE_SECONDS)
                if maneuver_group is not None
                else (0.0,)
            )
            overlapping_poses: list[BodyPose] = []
            for time_offset in time_offsets:
                prediction_seconds = max(
                    PHYSICS_STEP_SECONDS,
                    time_to_rejoin_s + time_offset,
                )
                predicted_total_progress = (
                    other.total_progress
                    + max(0.0, other.speed_kph / 3.6)
                    * prediction_seconds
                    / self.track_length_m
                )
                predicted_lateral_m = (
                    other.lateral_offset_m
                    + other.lateral_speed_mps * prediction_seconds
                )
                other_pose = BodyPose(
                    longitudinal_m=predicted_total_progress * self.track_length_m,
                    lateral_m=predicted_lateral_m,
                    heading_rad=other.slip_angle_rad,
                    length_m=other.car_length_m,
                    width_m=other.car_width_m,
                    longitudinal_speed_mps=max(0.0, other.speed_kph / 3.6),
                    lateral_speed_mps=other.lateral_speed_mps,
                )
                if oriented_body_overlap(pit_pose, other_pose) is not None:
                    overlapping_poses.append(other_pose)
            if overlapping_poses:
                conflicts.append(
                    (
                        min(
                            abs(other_pose.longitudinal_m - pit_pose.longitudinal_m)
                            for other_pose in overlapping_poses
                        ),
                        other,
                        maneuver_group,
                    )
                )

        if not conflicts:
            return PitMergeDecision("merge")
        _, conflict, conflict_group = min(
            conflicts,
            key=lambda item: (item[0], item[1].driver_id),
        )
        hold_threshold_m = PIT_MERGE_HOLD_DISTANCE_M + 0.5
        decision = "hold" if remaining_m <= hold_threshold_m or speed_mps <= 1.0 else "yield"
        return PitMergeDecision(
            decision,
            conflict_driver_id=conflict.driver_id,
            conflict_group_id=(conflict_group.group_id if conflict_group else None),
            conflict_group_member_ids=(
                conflict_group.member_ids if conflict_group else (conflict.driver_id,)
            ),
        )

    def _finish_pit_rejoin(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        events: list[RaceEvent],
    ) -> None:
        """Project the side-road pose into track coordinates without moving it."""
        new_compound = self._pit_tire[driver_id]
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        pit_entry_stint = state.pit_count + 1
        lane_progress = self._pit_route_progress[driver_id]
        pit_x, pit_y, pit_heading = self._pit_lane_pose_at_progress_m(lane_progress)

        if entry is not None and exit_ is not None:
            mapped_total_progress = (
                state.current_lap
                + entry
                + self._progress_distance(entry, exit_) * lane_progress
            )
            if mapped_total_progress >= state.current_lap + 1.0:
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
            state.total_progress = mapped_total_progress
            state.progress = mapped_total_progress % 1.0

        track_profile = self._track_physics_for_driver(state)
        line_length_m = max(
            1.0,
            track_profile.length_for_line(DRIVING_LINE_RACING),
        )
        # The pit route and circuit progress use different arc-length frames.
        # Refine the mapped circuit progress along its tangent so converting
        # from pit-route coordinates does not create a longitudinal jump.
        for _ in range(4):
            candidate_x, candidate_y, candidate_heading = (
                track_profile.line_pose_at_progress_m(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
            )
            tangent_delta_m = (
                (pit_x - candidate_x) * cos(candidate_heading)
                + (pit_y - candidate_y) * sin(candidate_heading)
            )
            state.progress = (state.progress + tangent_delta_m / line_length_m) % 1.0
        state.total_progress = state.current_lap + state.progress
        line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            state.progress,
        )
        lateral_delta_m = (
            (pit_x - line_x) * -sin(line_heading)
            + (pit_y - line_y) * cos(line_heading)
        )
        racing_offset_m = track_profile.line_offset_at_progress(
            DRIVING_LINE_RACING,
            state.progress,
        )
        state.in_pit = False
        state.pit_count += 1
        state.racing_line = DRIVING_LINE_RACING
        state.lateral_offset_m = racing_offset_m + lateral_delta_m
        state.target_lateral_offset_m = state.lateral_offset_m
        state.slip_angle_rad = (pit_heading - line_heading + pi) % (2.0 * pi) - pi
        state.total_distance_m = state.total_progress * self.track_length_m
        self._begin_sc_pit_exit_ordering(state)
        self._refresh_lap_variation(driver_id)
        merge_state = self._pit_merge_state.get(driver_id, "merge")
        self._clear_pit_state(driver_id)
        events.append(
            RaceEvent(
                type="pit_exit",
                driver=meta["abbreviation"],
                message=f"{meta['full_name']} exits pits on {new_compound.value} tires",
                message_ko=f"{meta['full_name']}가 {new_compound.value} 타이어로 피트를 빠져나옵니다",
                payload={"driver_id": driver_id, "merge_state": merge_state},
            )
        )

    def _clear_pit_state(self, driver_id: int) -> None:
        for store in (
            self._pit_phase,
            self._pit_phase_remaining,
            self._pit_phase_duration,
            self._pit_lane_half_time,
            self._pit_route_progress,
            self._pit_route_speed_mps,
            self._pit_merge_state,
            self._pit_merge_conflict_driver_id,
            self._pit_merge_conflict_group_id,
            self._pit_merge_conflict_group_member_ids,
            self._pit_tire_change_time,
            self._pit_elapsed,
            self._pit_stop_elapsed,
            self._pit_tire,
            self._pit_entry_compound,
        ):
            store.pop(driver_id, None)

    def _pit_lane_progress(self, driver_id: int) -> float:
        """Position along the pit lane (0=entry, 1=exit) for an in-pit car."""
        return min(1.0, max(0.0, self._pit_route_progress.get(driver_id, 0.0)))

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
        state.total_distance_m = state.total_progress * self.track_length_m

    def _pit_lane_progress_rate(self, driver_id: int) -> float:
        """Pit-lane progress rate per game second for front-end prediction."""
        if self._pit_phase.get(driver_id) in {None, "stop"}:
            return 0.0
        route_length_m = self._pit_route_length_m()
        if route_length_m <= 1e-9:
            return 0.0
        return self._pit_route_speed_mps.get(driver_id, 0.0) / route_length_m

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
        sector_times, mini_sector_times = self._lap_split_times(driver_id, lap)
        self._lap_history.setdefault(driver_id, []).append(
            LapTimeInfo(
                lap=lap,
                lap_time=round(lap_time, 3),
                tire_compound=tire_compound.value,
                stint=stint,
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
            self._apply_pace_mode(state, next_mode)
            self._ai_pace_cooldown[driver_id] = self._ai_pace_decision_cooldown()

    def _update_positions(self) -> None:
        self._require_rules_phase("update race positions")
        active = [
            s for s in self.driver_states.values() if not s.retired and not s.finished
        ]
        finished = [s for s in self.driver_states.values() if s.finished and not s.retired]
        retired = [s for s in self.driver_states.values() if s.retired]

        if self.race_phase == "green":
            active.sort(key=lambda s: (-s.total_progress, s.total_time))
            active_indices = {
                state.driver_id: index for index, state in enumerate(active)
            }
            for battle in self._side_by_side_battles.values():
                attacker_index = active_indices.get(battle.attacker_id)
                defender_index = active_indices.get(battle.defender_id)
                if attacker_index is None or defender_index is None:
                    continue
                attacker_should_lead = battle.pass_confirmed
                order_is_confirmed = (
                    attacker_index < defender_index
                    if attacker_should_lead
                    else defender_index < attacker_index
                )
                if order_is_confirmed:
                    continue
                active[attacker_index], active[defender_index] = (
                    active[defender_index],
                    active[attacker_index],
                )
                active_indices[battle.attacker_id] = defender_index
                active_indices[battle.defender_id] = attacker_index
        elif self.race_phase == "sc":
            # The on-track queue remains frozen, except for a car emerging
            # from the pits. Its place is provisional up to SC2 and is then
            # committed exactly once from physical race distance.
            self._commit_ready_sc_pit_exit_orders()
            on_track = self._sc_ordered_on_track_states()
            in_pit = sorted(
                (s for s in active if s.in_pit),
                key=lambda s: (-s.total_progress, s.position),
            )
            active = on_track
            for pit_car in in_pit:
                self._insert_by_live_race_distance(active, pit_car)
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
                self._insert_by_live_race_distance(active, pit_car)
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

    def _check_race_finished(self) -> bool:
        active = [
            s for s in self.driver_states.values()
            if not s.retired and not s.finished
        ]
        return len(active) == 0

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

    def build_tick_state(
        self,
        events: list[RaceEvent] | None = None,
        trajectory_samples_by_driver: dict[int, list[VehicleTrajectorySample]] | None = None,
        runtime_metrics: dict[str, float | int] | None = None,
    ) -> RaceTickState:
        """Build WebSocket tick payload."""
        trajectory_samples_by_driver = trajectory_samples_by_driver or {}
        runtime_metrics = runtime_metrics or {}
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
            local_yellow_active = self._local_yellow_active_for(state)
            tire_factors = self._current_tire_physics(state)

            interval = "—"
            interval_seconds = self._live_interval_seconds.get(state.driver_id)
            interval_timing_gap_valid = self._interval_timing_gap_valid.get(
                state.driver_id,
                False,
            )
            if not state.retired and state.position > 1:
                if running_by_pos.get(state.position - 1) is not None:
                    interval = self._format_interval(interval_seconds)

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

            (
                current_sector,
                current_mini_sector,
                current_timing_loop,
            ) = self._timing_location(state.progress)
            timing_lap_index = max(
                0,
                state.current_lap - (1 if state.finished else 0),
            )
            mini_sector_splits = self._current_mini_sector_splits(
                state.driver_id,
                timing_lap_index,
            )
            mini_sector_statuses = self._mini_sector_statuses(
                state.driver_id,
                mini_sector_splits,
            )
            last_completed_mini_index = self._last_completed_mini_sector_index.get(
                state.driver_id
            )
            last_mini_delta_to_best: float | None = None
            if last_completed_mini_index is not None:
                session_best = self._session_best_mini_sector_times[
                    last_completed_mini_index
                ]
                if session_best is not None:
                    last_mini_delta_to_best = max(
                        0.0,
                        self._last_mini_sector_time.get(state.driver_id, 0.0)
                        - session_best,
                    )

            maneuver = self._side_by_side_battle_for_driver(state.driver_id)
            maneuver_group = self._maneuver_group_for_driver(state.driver_id)
            maneuver_active = maneuver is not None or maneuver_group is not None
            side_by_side_active = bool(
                (maneuver is not None and maneuver.phase in {"overlap", "clear"})
                or (maneuver_group is not None and maneuver_group.size >= 3)
            )
            transition_elapsed = self._pace_mode_transition_elapsed.get(
                state.driver_id,
                PACE_MODE_TRANSITION_SECONDS,
            )
            telemetry_pace_transition_progress = min(
                1.0,
                max(0.0, transition_elapsed / PACE_MODE_TRANSITION_SECONDS),
            )
            telemetry_pace_intensity = self._pace_mode_intensity.get(
                state.driver_id,
                PACE_MODE_INTENSITIES.get(state.pace_mode, 0.0),
            )
            positions.append(
                DriverPositionInfo(
                    driver_id=state.driver_id,
                    name=meta["abbreviation"],
                    full_name=meta["full_name"],
                    team=meta["team_name"],
                    team_color=meta["team_color"],
                    position=state.position,
                    progress=round(state.progress, 7),
                    progress_rate=round(
                        0.0
                        if self.paused or state.in_pit or state.retired or state.finished
                        else self._progress_rate.get(state.driver_id, 0.0),
                        8,
                    ),
                    speed_kph=round(
                        0.0
                        if state.retired or state.finished
                        else state.speed_kph,
                        1,
                    ),
                    acceleration_mps2=state.acceleration_mps2,
                    simulation_time_s=state.simulation_time_s,
                    physics_frame=state.physics_frame,
                    world_x_m=state.world_x_m,
                    world_y_m=state.world_y_m,
                    heading_rad=state.heading_rad,
                    yaw_rate_rad_s=state.yaw_rate_rad_s,
                    velocity_x_mps=state.velocity_x_mps,
                    velocity_y_mps=state.velocity_y_mps,
                    acceleration_x_mps2=state.acceleration_x_mps2,
                    acceleration_y_mps2=state.acceleration_y_mps2,
                    lateral_acceleration_mps2=state.lateral_acceleration_mps2,
                    steering_angle_rad=state.steering_angle_rad,
                    gear=state.gear,
                    engine_rpm=state.engine_rpm,
                    drive_force_n=state.drive_force_n,
                    track_elevation_m=state.track_elevation_m,
                    track_grade=state.track_grade,
                    telemetry_source=state.telemetry_source,
                    target_speed_kph=state.target_speed_kph,
                    throttle=state.throttle,
                    brake=state.brake,
                    racing_line=state.racing_line,
                    lateral_offset_m=state.lateral_offset_m,
                    lateral_speed_mps=state.lateral_speed_mps,
                    target_lateral_offset_m=state.target_lateral_offset_m,
                    grip_utilization=state.grip_utilization,
                    handling_state=state.handling_state,
                    slip_angle_rad=state.slip_angle_rad,
                    wheel_lock_ratio=state.wheel_lock_ratio,
                    traction_slip_ratio=state.traction_slip_ratio,
                    tire_slide_energy_j=state.tire_slide_energy_j,
                    car_width_m=state.car_width_m,
                    car_length_m=state.car_length_m,
                    wheelbase_m=state.wheelbase_m,
                    gap=gap,
                    interval=interval,
                    gap_seconds=(
                        None
                        if state.retired or state.finished
                        else round(state.gap_to_leader, 3)
                    ),
                    interval_seconds=(
                        round(interval_seconds, 3)
                        if interval_seconds is not None
                        else None
                    ),
                    timing_gap_valid=self._timing_gap_valid.get(
                        state.driver_id,
                        False,
                    ),
                    interval_timing_gap_valid=interval_timing_gap_valid,
                    timing_gap_source=(
                        "live"
                        if self._timing_gap_valid.get(state.driver_id, False)
                        else "estimated"
                    ),
                    interval_timing_gap_source=(
                        "live" if interval_timing_gap_valid else "estimated"
                    ),
                    current_sector=current_sector,
                    current_mini_sector=current_mini_sector,
                    current_timing_loop=current_timing_loop,
                    last_sector_time=round(
                        self._last_sector_time.get(state.driver_id, 0.0),
                        3,
                    ),
                    last_mini_sector_time=round(
                        self._last_mini_sector_time.get(state.driver_id, 0.0),
                        3,
                    ),
                    last_mini_sector_delta_to_best=(
                        round(last_mini_delta_to_best, 3)
                        if last_mini_delta_to_best is not None
                        else None
                    ),
                    mini_sector_splits=[
                        round(split, 3) if split is not None else None
                        for split in mini_sector_splits
                    ],
                    mini_sector_statuses=mini_sector_statuses,
                    tire_compound=state.tire_compound.value,
                    tire_age=state.tire_age,
                    tire_wear=round(tire_factors.wear, 3),
                    tire_lateral_grip=round(tire_factors.lateral_grip, 4),
                    tire_traction_grip=round(tire_factors.traction_grip, 4),
                    tire_braking_grip=round(tire_factors.braking_grip, 4),
                    tire_surface_temperature_c=round(
                        state.tire_surface_temperature_c,
                        2,
                    ),
                    tire_core_temperature_c=round(
                        state.tire_core_temperature_c,
                        2,
                    ),
                    tire_thermal_grip=round(tire_factors.thermal_grip, 4),
                    fuel_mass_kg=round(state.fuel_mass_kg, 3),
                    fuel_burned_kg=round(state.fuel_burned_kg, 3),
                    fuel_laps_remaining=round(state.fuel_laps_remaining, 2),
                    planner_tier_hz=state.planner_tier_hz,
                    planner_mode=state.planner_mode,
                    planner_generation_ms=state.planner_generation_ms,
                    planner_fallback_active=state.planner_fallback_active,
                    planner_nearby_vehicle_count=(
                        state.planner_nearby_vehicle_count
                    ),
                    planner_replan_count=state.planner_replan_count,
                    pace_mode=state.pace_mode.value,
                    pace_mode_from=self._pace_mode_transition_from.get(
                        state.driver_id,
                        state.pace_mode,
                    ).value,
                    pace_mode_transition_progress=round(
                        telemetry_pace_transition_progress,
                        4,
                    ),
                    pace_mode_effective_intensity=round(
                        telemetry_pace_intensity,
                        4,
                    ),
                    last_lap_time=round(state.last_lap_time, 3),
                    best_lap_time=round(state.best_lap_time, 3),
                    in_pit=state.in_pit,
                    pit_count=state.pit_count,
                    pit_phase=self._pit_phase.get(state.driver_id),
                    pit_merge_state=self._pit_merge_state.get(state.driver_id),
                    pit_merge_conflict_driver_id=(
                        self._pit_merge_conflict_driver_id.get(state.driver_id)
                    ),
                    pit_merge_conflict_group_id=(
                        self._pit_merge_conflict_group_id.get(state.driver_id)
                    ),
                    pit_merge_conflict_group_member_ids=list(
                        self._pit_merge_conflict_group_member_ids.get(
                            state.driver_id,
                            (),
                        )
                    ),
                    pit_lane_progress=round(self._pit_lane_progress(state.driver_id), 7),
                    pit_lane_progress_rate=round(
                        0.0 if self.paused else self._pit_lane_progress_rate(state.driver_id),
                        8,
                    ),
                    pit_box_progress=round(
                        self._pit_box_progress_for_driver(state.driver_id),
                        7,
                    ),
                    pit_elapsed=round(self._pit_elapsed.get(state.driver_id, 0.0), 1),
                    pit_stop_elapsed=round(self._pit_stop_elapsed.get(state.driver_id, 0.0), 1),
                    lap_history=self._lap_history.get(state.driver_id, []),
                    retired=state.retired,
                    finished=state.finished,
                    drs_active=state.drs_active,
                    dirty_air_active=state.dirty_air_active,
                    wake_strength=round(state.wake_strength, 4),
                    tow_strength=round(state.tow_strength, 4),
                    dirty_air_strength=round(state.dirty_air_strength, 4),
                    wake_source_driver_id=state.wake_source_driver_id,
                    wake_longitudinal_gap_m=round(
                        state.wake_longitudinal_gap_m,
                        3,
                    ),
                    wake_lateral_separation_m=round(
                        state.wake_lateral_separation_m,
                        3,
                    ),
                    wake_drag_multiplier=round(state.wake_drag_multiplier, 4),
                    wake_downforce_multiplier=round(
                        state.wake_downforce_multiplier,
                        4,
                    ),
                    maneuver_active=maneuver_active,
                    maneuver_phase=maneuver.phase if maneuver else None,
                    maneuver_role=(
                        "attacker"
                        if maneuver and state.driver_id == maneuver.attacker_id
                        else "defender" if maneuver else None
                    ),
                    maneuver_opponent_id=(
                        maneuver.defender_id
                        if maneuver and state.driver_id == maneuver.attacker_id
                        else maneuver.attacker_id if maneuver else None
                    ),
                    maneuver_line=(
                        maneuver.attacker_line
                        if maneuver and state.driver_id == maneuver.attacker_id
                        else maneuver.defender_line if maneuver else None
                    ),
                    maneuver_elapsed_seconds=(
                        round(maneuver.elapsed_seconds, 3) if maneuver else 0.0
                    ),
                    side_by_side_active=side_by_side_active,
                    maneuver_corner_active=(
                        maneuver.corner_active if maneuver else False
                    ),
                    maneuver_corner_authorized=(
                        maneuver.corner_authorized if maneuver else False
                    ),
                    maneuver_line_committed=(
                        maneuver.line_committed if maneuver else False
                    ),
                    maneuver_corridor=(
                        self._corner_corridor_for_driver(maneuver, state.driver_id)
                        if maneuver and maneuver.corner_authorized
                        else None
                    ),
                    maneuver_corner_turn_direction=(
                        maneuver.corner_turn_direction if maneuver else 0
                    ),
                    maneuver_corner_entry_advantage_m=(
                        round(maneuver.corner_entry_advantage_m, 3)
                        if maneuver
                        else 0.0
                    ),
                    maneuver_group_id=(
                        maneuver_group.group_id if maneuver_group else None
                    ),
                    maneuver_group_size=(
                        maneuver_group.size if maneuver_group else 0
                    ),
                    maneuver_group_member_ids=(
                        list(maneuver_group.member_ids) if maneuver_group else []
                    ),
                    maneuver_group_phase=(
                        maneuver_group.phase if maneuver_group else None
                    ),
                    maneuver_group_corridor_index=(
                        maneuver_group.member_ids.index(state.driver_id)
                        if maneuver_group
                        else None
                    ),
                    maneuver_group_corner_priority=(
                        maneuver_group.corner_priority_ids.index(state.driver_id)
                        if maneuver_group
                        and state.driver_id in maneuver_group.corner_priority_ids
                        else None
                    ),
                    maneuver_group_corner_turn_direction=(
                        maneuver_group.corner_turn_direction
                        if maneuver_group
                        else 0
                    ),
                    forced_wide_by_driver_id=self._forced_wide_by_driver.get(
                        state.driver_id
                    ),
                    surface_state=state.surface_state,
                    wheel_surfaces=state.wheel_surfaces,
                    kerb_contact=state.kerb_contact,
                    off_track=state.off_track,
                    off_track_cause=state.off_track_cause,
                    track_limits_active=state.track_limits_active,
                    surface_grip_multiplier=state.surface_grip_multiplier,
                    surface_drag_deceleration_mps2=(
                        state.surface_drag_deceleration_mps2
                    ),
                    contact_active=state.contact_active,
                    contact_opponent_id=state.contact_opponent_id,
                    contact_impact_speed_mps=state.contact_impact_speed_mps,
                    contact_type=state.contact_type,
                    contact_severity=state.contact_severity,
                    contact_progress=round(state.contact_progress, 7),
                    contact_lateral_offset_m=state.contact_lateral_offset_m,
                    contact_normal_longitudinal=state.contact_normal_longitudinal,
                    contact_normal_lateral=state.contact_normal_lateral,
                    collision_damage=round(state.collision_damage, 4),
                    vehicle_status=state.vehicle_status,
                    hazard_active=state.hazard_active,
                    hazard_cause=state.hazard_cause,
                    local_yellow_active=local_yellow_active,
                    avoidance_active=state.avoidance_active,
                    avoidance_hazard_driver_id=(
                        state.avoidance_hazard_driver_id
                    ),
                    avoidance_side=state.avoidance_side,
                    avoidance_ttc_seconds=round(
                        state.avoidance_ttc_seconds,
                        3,
                    ),
                    avoidance_target_lateral_offset_m=round(
                        state.avoidance_target_lateral_offset_m,
                        4,
                    ),
                    emergency_braking=state.emergency_braking,
                    trajectory_samples=trajectory_samples_by_driver.get(
                        state.driver_id,
                        [],
                    ),
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
                7,
            ),
            safety_car_progress_rate=round(self._safety_car_progress_rate, 8),
            safety_car_pit_lane_progress=round(self._safety_car_pit_lane_progress, 7),
            safety_car_queue_formed=self._safety_car_queue_formed,
            overtaking_allowed=self.race_started and self.race_phase == "green",
            race_started=self.race_started,
            start_sequence_phase=self.start_sequence_phase,
            start_light_count=self.start_light_count,
            restart_line_progress=0.0,
            pit_window_open=self.pit_window_open,
            race_elapsed=round(self.race_elapsed, 3),
            physics_frame=self._physics_frame,
            speed_multiplier=self.speed_multiplier,
            paused=self.paused,
            physics_hz=round(1.0 / PHYSICS_STEP_SECONDS),
            broadcast_hz=int(runtime_metrics.get("broadcast_hz", 30)),
            effective_speed_multiplier=round(
                float(runtime_metrics.get("effective_speed_multiplier", 0.0)),
                3,
            ),
            simulation_backlog_seconds=round(
                float(runtime_metrics.get("simulation_backlog_seconds", 0.0)),
                4,
            ),
            broadcast_jitter_ms=round(
                float(runtime_metrics.get("broadcast_jitter_ms", 0.0)),
                2,
            ),
            physics_steps_last_broadcast=int(
                runtime_metrics.get("physics_steps_last_broadcast", 0)
            ),
            positions=positions,
            events=events or [],
        )

    def build_dashboard_payload(
        self,
        runtime_metrics: dict[str, float | int] | None = None,
    ) -> dict:
        """Build the live UI payload without allocating full telemetry models."""
        runtime_metrics = runtime_metrics or {}
        sorted_states = sorted(
            self.driver_states.values(),
            key=lambda state: state.position,
        )
        running_by_pos = {
            state.position: state
            for state in sorted_states
            if not state.retired
        }
        positions: list[dict] = []
        for state in sorted_states:
            meta = self._driver_meta[state.driver_id]
            interval_seconds = self._live_interval_seconds.get(state.driver_id)
            interval_timing_gap_valid = self._interval_timing_gap_valid.get(
                state.driver_id,
                False,
            )
            interval = "—"
            if (
                not state.retired
                and state.position > 1
                and running_by_pos.get(state.position - 1) is not None
            ):
                interval = self._format_interval(interval_seconds)

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

            (
                current_sector,
                current_mini_sector,
                current_timing_loop,
            ) = self._timing_location(state.progress)
            transition_elapsed = self._pace_mode_transition_elapsed.get(
                state.driver_id,
                PACE_MODE_TRANSITION_SECONDS,
            )
            maneuver = self._side_by_side_battle_for_driver(state.driver_id)
            maneuver_group = self._maneuver_group_for_driver(state.driver_id)
            side_by_side_active = bool(
                (maneuver is not None and maneuver.phase in {"overlap", "clear"})
                or (maneuver_group is not None and maneuver_group.size >= 3)
            )
            position = {
                    "driver_id": state.driver_id,
                    "name": meta["abbreviation"],
                    "full_name": meta["full_name"],
                    "team": meta["team_name"],
                    "team_color": meta["team_color"],
                    "position": state.position,
                    "speed_kph": round(
                        0.0
                        if state.retired or state.finished
                        else state.speed_kph,
                        1,
                    ),
                    "gap": gap,
                    "interval": interval,
                    "timing_gap_valid": self._timing_gap_valid.get(
                        state.driver_id,
                        False,
                    ),
                    "interval_timing_gap_valid": interval_timing_gap_valid,
                    "timing_gap_source": (
                        "live"
                        if self._timing_gap_valid.get(state.driver_id, False)
                        else "estimated"
                    ),
                    "interval_timing_gap_source": (
                        "live" if interval_timing_gap_valid else "estimated"
                    ),
                    "current_sector": current_sector,
                    "current_mini_sector": current_mini_sector,
                    "current_timing_loop": current_timing_loop,
                    "tire_compound": state.tire_compound.value,
                    "tire_age": state.tire_age,
                    "tire_wear": round(
                        self._current_tire_physics(state).wear,
                        3,
                    ),
                    "pace_mode": state.pace_mode.value,
                    "pace_mode_from": self._pace_mode_transition_from.get(
                        state.driver_id,
                        state.pace_mode,
                    ).value,
                    "pace_mode_transition_progress": round(
                        min(
                            1.0,
                            max(
                                0.0,
                                transition_elapsed / PACE_MODE_TRANSITION_SECONDS,
                            ),
                        ),
                        4,
                    ),
                    "last_lap_time": round(state.last_lap_time, 3),
                    "best_lap_time": round(state.best_lap_time, 3),
                    "in_pit": state.in_pit,
                    "pit_count": state.pit_count,
                    "pit_phase": self._pit_phase.get(state.driver_id),
                    "pit_merge_state": self._pit_merge_state.get(state.driver_id),
                    "pit_box_progress": round(
                        self._pit_box_progress_for_driver(state.driver_id),
                        7,
                    ),
                    "pit_elapsed": round(
                        self._pit_elapsed.get(state.driver_id, 0.0),
                        1,
                    ),
                    "pit_stop_elapsed": round(
                        self._pit_stop_elapsed.get(state.driver_id, 0.0),
                        1,
                    ),
                    "retired": state.retired,
                    "finished": state.finished,
                    "drs_active": state.drs_active,
                    "dirty_air_active": state.dirty_air_active,
                    "side_by_side_active": side_by_side_active,
                    "maneuver_group_size": (
                        maneuver_group.size if maneuver_group else 0
                    ),
                    "hazard_active": state.hazard_active,
                    "local_yellow_active": self._local_yellow_active_for(state),
                }
            position_defaults = {
                "speed_kph": 0.0,
                "timing_gap_valid": False,
                "interval_timing_gap_valid": False,
                "timing_gap_source": "estimated",
                "interval_timing_gap_source": "estimated",
                "current_sector": 1,
                "current_mini_sector": 1,
                "current_timing_loop": 1,
                "pace_mode": PaceMode.STANDARD.value,
                "pace_mode_from": PaceMode.STANDARD.value,
                "pace_mode_transition_progress": 1.0,
                "pit_phase": None,
                "pit_merge_state": None,
                "pit_box_progress": 0.5,
                "pit_elapsed": 0.0,
                "pit_stop_elapsed": 0.0,
                "finished": False,
                "drs_active": False,
                "dirty_air_active": False,
                "side_by_side_active": False,
                "maneuver_group_size": 0,
                "hazard_active": False,
                "local_yellow_active": False,
            }
            for field, default in position_defaults.items():
                if position.get(field) == default:
                    position.pop(field, None)
            positions.append(position)

        payload = {
            "type": "race_state",
            "lap": self.current_lap,
            "total_laps": self.total_laps,
            "race_phase": self.race_phase,
            "race_phase_remaining_seconds": round(
                self._race_phase_remaining_seconds(),
                1,
            ),
            "race_phase_remaining_laps": self._race_phase_remaining_laps(),
            "safety_car_stage": self.safety_car_stage,
            "safety_car_visible": self._safety_car_visible,
            "safety_car_route": self._safety_car_route,
            "safety_car_progress": round(
                (self._safety_car_total_progress or 0.0) % 1.0,
                7,
            ),
            "safety_car_progress_rate": round(
                self._safety_car_progress_rate,
                8,
            ),
            "safety_car_pit_lane_progress": round(
                self._safety_car_pit_lane_progress,
                7,
            ),
            "pit_window_open": self.pit_window_open,
            "start_sequence_phase": self.start_sequence_phase,
            "start_light_count": self.start_light_count,
            "speed_multiplier": self.speed_multiplier,
            "paused": self.paused,
            "physics_hz": round(1.0 / PHYSICS_STEP_SECONDS),
            "broadcast_hz": int(runtime_metrics.get("broadcast_hz", 30)),
            "effective_speed_multiplier": round(
                float(runtime_metrics.get("effective_speed_multiplier", 0.0)),
                3,
            ),
            "simulation_backlog_seconds": round(
                float(runtime_metrics.get("simulation_backlog_seconds", 0.0)),
                4,
            ),
            "broadcast_jitter_ms": round(
                float(runtime_metrics.get("broadcast_jitter_ms", 0.0)),
                2,
            ),
            "positions": positions,
        }
        state_defaults = {
            "race_phase": "green",
            "race_phase_remaining_seconds": 0.0,
            "race_phase_remaining_laps": 0,
            "safety_car_stage": "inactive",
            "safety_car_visible": False,
            "safety_car_route": "track",
            "safety_car_progress": 0.0,
            "safety_car_progress_rate": 0.0,
            "safety_car_pit_lane_progress": 0.0,
            "pit_window_open": False,
            "start_sequence_phase": "racing",
            "start_light_count": 0,
            "speed_multiplier": 1,
            "paused": False,
            "physics_hz": 50,
            "broadcast_hz": 30,
            "effective_speed_multiplier": 0.0,
            "simulation_backlog_seconds": 0.0,
            "broadcast_jitter_ms": 0.0,
        }
        for field, default in state_defaults.items():
            if payload.get(field) == default:
                payload.pop(field, None)
        return payload

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

    def build_pose_state(
        self,
        trajectory_samples_by_driver: (
            dict[int, list[VehicleTrajectorySample]] | None
        ) = None,
    ) -> RacePoseState:
        """Build the compact high-frequency stream consumed by the canvas.

        Dashboard, timing, strategy and diagnostic fields deliberately stay in
        ``build_tick_state``. Sending them with every pose made Chrome parse
        several megabytes of short-lived JSON per second.
        """
        trajectory_samples_by_driver = trajectory_samples_by_driver or {}
        positions: list[DriverPoseInfo] = []
        for state in self.driver_states.values():
            trajectory_samples = trajectory_samples_by_driver.get(
                state.driver_id,
                [],
            )
            # A trajectory already contains the latest authoritative pose. Keep
            # the top-level pose only as the no-step/initial-snapshot fallback;
            # compact serialization can then omit five duplicate numbers.
            has_trajectory = bool(trajectory_samples)
            positions.append(
                DriverPoseInfo(
                    driver_id=state.driver_id,
                    simulation_time_s=(
                        0.0 if has_trajectory else state.simulation_time_s
                    ),
                    physics_frame=0 if has_trajectory else state.physics_frame,
                    world_x_m=0.0 if has_trajectory else state.world_x_m,
                    world_y_m=0.0 if has_trajectory else state.world_y_m,
                    heading_rad=0.0 if has_trajectory else state.heading_rad,
                    retired=state.retired,
                    hazard_active=state.hazard_active,
                    trajectory_samples=trajectory_samples,
                )
            )
        return RacePoseState(
            physics_frame=self._physics_frame,
            speed_multiplier=self.speed_multiplier,
            paused=self.paused,
            positions=positions,
        )

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
