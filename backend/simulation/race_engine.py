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
from simulation.car_performance import CarPerformanceFactors, car_performance_factors
from simulation.collision import (
    BodyPose,
    oriented_body_separation_m,
)
from simulation.events import roll_events
from simulation.incidents import (
    Incident,
    roll_solo_incident,
)
from simulation.local_trajectory_planner import (
    LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
    LocalTrajectoryPlan,
    LocalTrajectoryPlanner,
    LocalTrajectoryPlanningRequest,
    NearbyVehiclePredictionInput,
)
from simulation.physics import (
    GAME_TICK_SECONDS,
    driver_pace_multiplier,
)
from simulation.fixed_step import FixedStepAccumulator
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
    compute_tire_physics_factors,
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
from simulation.safety_car import (
    SC_ADDITIONAL_INCIDENT_SECONDS,
    SC_CAR_LENGTH_M,
    SC_CATCH_UP_FAST_LAP_TIME_FACTOR,
    SC_CATCH_UP_MAX_SPEED_KPH,
    SC_CATCH_UP_NEAR_LAP_TIME_FACTOR,
    SC_CAUGHT_MAX_SPEED_KPH,
    SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR,
    SC_CLEANUP_SECONDS,
    SC_COLLECTION_LAP_TIME_FACTOR,
    SC_DEPLOY_PIT_SECONDS,
    SC_LAP_TIME_FACTOR,
    SC_LEAD_PROGRESS_GAP,
    SC_LEADER_ACQUISITION_SPEED_KPH,
    SC_MAX_GAP_CAR_LENGTHS,
    SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS,
    SC_NEAR_QUEUE_MAX_SPEED_KPH,
    SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS,
    SC_ORDER_RESTORE_SPEED_DELTA_KPH,
    SC_ORDER_RESTORE_TRIGGER_M,
    SC_PIT_PROBABILITY,
    SC_PIT_WEAR_THRESHOLD,
    SC_QUEUE_APPROACH_DECELERATION_MPS2,
    SC_QUEUE_APPROACH_MAX_GAP_M,
    SC_QUEUE_APPROACH_REACTION_SECONDS,
    SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH,
    SC_QUEUE_PROPAGATION_HORIZON_M,
    SC_QUEUE_TARGET_CAR_LENGTHS,
    SC_RELEASE_GAP_CAR_LENGTHS,
    SC_UNLAP_LAP_TIME_FACTOR,
    SC_UNLAP_MAX_SPEED_KPH,
    SC_WITHDRAW_PIT_SECONDS,
    SafetyCarMixin,
    VSC_DURATION_SECONDS,
    VSC_LAP_TIME_FACTOR,
)
from simulation.pit_ops import (
    PIT_LANE_SPEED_LIMIT_KPH,
    PitMergeDecision,
    PitOpsMixin,
)
from simulation.racecraft_ops import (
    AI_ATTACK_EXIT_GAP_SECONDS,
    AI_ATTACK_GAP_SECONDS,
    AI_DEFEND_EXIT_GAP_SECONDS,
    AI_DEFEND_GAP_SECONDS,
    ATTACK_LINE_INSIDE,
    ATTACK_LINE_OUTSIDE,
    BATTLE_EVENT_GAP_SECONDS,
    BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA,
    CORNER_CORRIDOR_MARGIN_M,
    DEFENDER_LINE_DEFENSIVE,
    DEFENDER_LINE_RACING,
    DRS_MAX_BONUS,
    MANEUVER_ABORT_REJOIN_SECONDS,
    MANEUVER_ATTACK_DECISION_BUMPER_GAP_M,
    MANEUVER_CLEARANCE_MARGIN_M,
    MANEUVER_LIVE_CLEARANCE_BUFFER_M,
    MANEUVER_PASS_CLEARANCE_MARGIN_M,
    MANEUVER_PULL_OUT_BUMPER_GAP_M,
    MANEUVER_PULL_OUT_MIN_CLEARANCE_M,
    MANEUVER_TOW_PREP_BUMPER_GAP_M,
    TRAFFIC_GAP_SECONDS,
    BattleIntent,
    LocalPullOutDecision,
    ManeuverGroup,
    PendingOvertakeCommand,
    RacecraftMixin,
)
from simulation.timing_ops import (
    TIMING_CROSSING_LAPS_TO_RETAIN,
    TimingLoop,
    TimingOpsMixin,
)
from simulation.strategy_ops import (
    AI_CONSERVE_EXIT_TIRE_LIFE,
    AI_CRITICAL_TIRE_LIFE,
    AI_FRESH_TIRE_USAGE,
    AI_LOW_TIRE_LIFE,
    AI_PACE_INITIAL_STAGGER_MAX_SECONDS,
    AI_PACE_INITIAL_STAGGER_MIN_SECONDS,
    AI_PACE_MAX_COOLDOWN_SECONDS,
    AI_PACE_MIN_COOLDOWN_SECONDS,
    FUEL_CONSUMPTION_KG_PER_KM,
    FUEL_LOAD_RESERVE_FACTOR,
    IDLE_FUEL_FLOW_KG_PER_SECOND,
    MAX_FUEL_FLOW_KG_PER_SECOND,
    MAX_INITIAL_FUEL_MASS_KG,
    PACE_MODE_ATTACK_FACTORS,
    PACE_MODE_DRS_FACTORS,
    PACE_MODE_EFFECTS,
    PACE_MODE_INTENSITIES,
    PACE_MODE_PHYSICS_FACTORS,
    PACE_MODE_PLANNER_WEIGHTS,
    PACE_MODE_TRANSITION_SECONDS,
    StrategyOpsMixin,
    TIRE_SLIDE_WEAR_LAPS_PER_JOULE,
)
from simulation.incident_ops import (
    COLLISION_DISPLAY_SECONDS,
    COLLISION_ESCALATION_IMPACT_MPS,
    COLLISION_MIN_REPORT_IMPACT_MPS,
    COLLISION_PAIR_COOLDOWN_SECONDS,
    COLLISION_STATIC_PENETRATION_TOLERANCE_M,
    FOLLOWING_MIN_BUMPER_GAP_M,
    HAZARD_BLOCKED_CLEAR_SECONDS,
    HAZARD_BRAKING_DECELERATION_MPS2,
    HAZARD_CLEAR_RADIUS_M,
    HAZARD_CLUSTER_LONGITUDINAL_GAP_M,
    HAZARD_CORRIDOR_PREFERENCE_PENALTY,
    HAZARD_CORRIDOR_TARGET_MARGIN_M,
    HAZARD_DETECTION_DISTANCE_M,
    HAZARD_EVASIVE_LATERAL_ACCELERATION_MPS2,
    HAZARD_EVASIVE_TIME_BUFFER_SECONDS,
    HAZARD_LATERAL_MARGIN_M,
    HAZARD_MOVING_RELEASE_MARGIN_M,
    HAZARD_PASS_CLEARANCE_M,
    HAZARD_REACTION_SECONDS,
    HAZARD_TRAFFIC_WINDOW_M,
    INCIDENT_MINOR_TIME_PENALTY,
    INCIDENT_MINOR_TIRE_USAGE,
    DriverInputError,
    IncidentOpsMixin,
)
from simulation.runtime_constants import GRID_LAUNCH_MERGE_DISTANCE_M
from simulation.start_ops import (
    GRID_COLUMN_OFFSET_M,
    GRID_FIRST_LIGHT_SECONDS,
    GRID_LAUNCH_LANE_HOLD_M,
    GRID_LIGHTS_OUT_DISPLAY_SECONDS,
    GRID_LIGHTS_OUT_SECONDS,
    GRID_LIGHT_INTERVAL_SECONDS,
    GRID_OVERTAKE_LOCKOUT_SECONDS,
    GRID_POLE_DISTANCE_BEHIND_LINE_M,
    GRID_SLOT_PROGRESS_GAP,
    GRID_SLOT_SPACING_M,
    StartOpsMixin,
)

MAX_PHYSICS_DIAGNOSTIC_SAMPLES = 256

RACING_ACCELERATION_MPS2 = 12.0
RACING_BRAKING_MPS2 = 34.0
# Tactical intent refreshes at 10 Hz; path rebuild cadence comes from
# LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS (local_trajectory_planner).
AI_TACTICAL_DECISION_INTERVAL_SECONDS = 0.10
LOCAL_TRAJECTORY_OPPONENT_RADIUS_M = 150.0
LOCAL_TRAJECTORY_DENSE_TRAFFIC_RADIUS_M = 30.0
LOCAL_TRAJECTORY_MAX_OPPONENTS = 4
LOCAL_TRAJECTORY_MIN_SELECTED_CLEARANCE_M = 0.35
PROGRESS_EPSILON = 1e-9
DRS_DRAG_MULTIPLIER = 0.82
DRS_DOWNFORCE_MULTIPLIER = 0.92

# Supported public facade. Domain modules own definitions; race_engine only
# re-exports names listed in ``__all__`` for tests/tools. This is not a full
# replay of every pre-decomposition module-level name — consumers that need
# other symbols should import from the owning domain module.
__all__ = [
    "AI_ATTACK_EXIT_GAP_SECONDS",
    "AI_ATTACK_GAP_SECONDS",
    "AI_CONSERVE_EXIT_TIRE_LIFE",
    "AI_CRITICAL_TIRE_LIFE",
    "AI_DEFEND_EXIT_GAP_SECONDS",
    "AI_DEFEND_GAP_SECONDS",
    "AI_FRESH_TIRE_USAGE",
    "AI_LOW_TIRE_LIFE",
    "AI_PACE_INITIAL_STAGGER_MAX_SECONDS",
    "AI_PACE_INITIAL_STAGGER_MIN_SECONDS",
    "AI_PACE_MAX_COOLDOWN_SECONDS",
    "AI_PACE_MIN_COOLDOWN_SECONDS",
    "AI_TACTICAL_DECISION_INTERVAL_SECONDS",
    "ATTACK_LINE_INSIDE",
    "ATTACK_LINE_OUTSIDE",
    "BATTLE_EVENT_GAP_SECONDS",
    "BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA",
    "BattleIntent",
    "COLLISION_DISPLAY_SECONDS",
    "COLLISION_ESCALATION_IMPACT_MPS",
    "COLLISION_MIN_REPORT_IMPACT_MPS",
    "COLLISION_PAIR_COOLDOWN_SECONDS",
    "COLLISION_STATIC_PENETRATION_TOLERANCE_M",
    "CORNER_CORRIDOR_MARGIN_M",
    "DEFENDER_LINE_DEFENSIVE",
    "DEFENDER_LINE_RACING",
    "DRS_DOWNFORCE_MULTIPLIER",
    "DRS_DRAG_MULTIPLIER",
    "DRS_MAX_BONUS",
    "DriverInputError",
    "FOLLOWING_MIN_BUMPER_GAP_M",
    "FUEL_CONSUMPTION_KG_PER_KM",
    "FUEL_LOAD_RESERVE_FACTOR",
    "GRID_COLUMN_OFFSET_M",
    "GRID_FIRST_LIGHT_SECONDS",
    "GRID_LAUNCH_LANE_HOLD_M",
    "GRID_LAUNCH_MERGE_DISTANCE_M",
    "GRID_LIGHTS_OUT_DISPLAY_SECONDS",
    "GRID_LIGHTS_OUT_SECONDS",
    "GRID_LIGHT_INTERVAL_SECONDS",
    "GRID_OVERTAKE_LOCKOUT_SECONDS",
    "GRID_POLE_DISTANCE_BEHIND_LINE_M",
    "GRID_SLOT_PROGRESS_GAP",
    "GRID_SLOT_SPACING_M",
    "HAZARD_BLOCKED_CLEAR_SECONDS",
    "HAZARD_BRAKING_DECELERATION_MPS2",
    "HAZARD_CLEAR_RADIUS_M",
    "HAZARD_CLUSTER_LONGITUDINAL_GAP_M",
    "HAZARD_CORRIDOR_PREFERENCE_PENALTY",
    "HAZARD_CORRIDOR_TARGET_MARGIN_M",
    "HAZARD_DETECTION_DISTANCE_M",
    "HAZARD_EVASIVE_LATERAL_ACCELERATION_MPS2",
    "HAZARD_EVASIVE_TIME_BUFFER_SECONDS",
    "HAZARD_LATERAL_MARGIN_M",
    "HAZARD_MOVING_RELEASE_MARGIN_M",
    "HAZARD_PASS_CLEARANCE_M",
    "HAZARD_REACTION_SECONDS",
    "HAZARD_TRAFFIC_WINDOW_M",
    "IDLE_FUEL_FLOW_KG_PER_SECOND",
    "INCIDENT_MINOR_TIME_PENALTY",
    "INCIDENT_MINOR_TIRE_USAGE",
    "LOCAL_TRAJECTORY_DENSE_TRAFFIC_RADIUS_M",
    "LOCAL_TRAJECTORY_MAX_OPPONENTS",
    "LOCAL_TRAJECTORY_MIN_SELECTED_CLEARANCE_M",
    "LOCAL_TRAJECTORY_OPPONENT_RADIUS_M",
    "LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS",
    "LocalPullOutDecision",
    "MANEUVER_ABORT_REJOIN_SECONDS",
    "MANEUVER_ATTACK_DECISION_BUMPER_GAP_M",
    "MANEUVER_CLEARANCE_MARGIN_M",
    "MANEUVER_LIVE_CLEARANCE_BUFFER_M",
    "MANEUVER_PASS_CLEARANCE_MARGIN_M",
    "MANEUVER_PULL_OUT_BUMPER_GAP_M",
    "MANEUVER_PULL_OUT_MIN_CLEARANCE_M",
    "MANEUVER_TOW_PREP_BUMPER_GAP_M",
    "MAX_FUEL_FLOW_KG_PER_SECOND",
    "MAX_INITIAL_FUEL_MASS_KG",
    "MAX_PHYSICS_DIAGNOSTIC_SAMPLES",
    "ManeuverGroup",
    "PACE_MODE_ATTACK_FACTORS",
    "PACE_MODE_DRS_FACTORS",
    "PACE_MODE_EFFECTS",
    "PACE_MODE_INTENSITIES",
    "PACE_MODE_PHYSICS_FACTORS",
    "PACE_MODE_PLANNER_WEIGHTS",
    "PACE_MODE_TRANSITION_SECONDS",
    "PIT_LANE_SPEED_LIMIT_KPH",
    "PROGRESS_EPSILON",
    "PendingOvertakeCommand",
    "PitMergeDecision",
    "RACING_ACCELERATION_MPS2",
    "RACING_BRAKING_MPS2",
    "RaceEngine",
    "SC_ADDITIONAL_INCIDENT_SECONDS",
    "SC_CAR_LENGTH_M",
    "SC_CATCH_UP_FAST_LAP_TIME_FACTOR",
    "SC_CATCH_UP_MAX_SPEED_KPH",
    "SC_CATCH_UP_NEAR_LAP_TIME_FACTOR",
    "SC_CAUGHT_MAX_SPEED_KPH",
    "SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR",
    "SC_CLEANUP_SECONDS",
    "SC_COLLECTION_LAP_TIME_FACTOR",
    "SC_DEPLOY_PIT_SECONDS",
    "SC_LAP_TIME_FACTOR",
    "SC_LEADER_ACQUISITION_SPEED_KPH",
    "SC_LEAD_PROGRESS_GAP",
    "SC_MAX_GAP_CAR_LENGTHS",
    "SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS",
    "SC_NEAR_QUEUE_MAX_SPEED_KPH",
    "SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS",
    "SC_ORDER_RESTORE_SPEED_DELTA_KPH",
    "SC_ORDER_RESTORE_TRIGGER_M",
    "SC_PIT_PROBABILITY",
    "SC_PIT_WEAR_THRESHOLD",
    "SC_QUEUE_APPROACH_DECELERATION_MPS2",
    "SC_QUEUE_APPROACH_MAX_GAP_M",
    "SC_QUEUE_APPROACH_REACTION_SECONDS",
    "SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH",
    "SC_QUEUE_PROPAGATION_HORIZON_M",
    "SC_QUEUE_TARGET_CAR_LENGTHS",
    "SC_RELEASE_GAP_CAR_LENGTHS",
    "SC_UNLAP_LAP_TIME_FACTOR",
    "SC_UNLAP_MAX_SPEED_KPH",
    "SC_WITHDRAW_PIT_SECONDS",
    "TIMING_CROSSING_LAPS_TO_RETAIN",
    "TIRE_SLIDE_WEAR_LAPS_PER_JOULE",
    "TRAFFIC_GAP_SECONDS",
    "TimingLoop",
    "VSC_DURATION_SECONDS",
    "VSC_LAP_TIME_FACTOR",
]


class RaceEngine(
    SafetyCarMixin,
    PitOpsMixin,
    RacecraftMixin,
    TimingOpsMixin,
    StrategyOpsMixin,
    IncidentOpsMixin,
    StartOpsMixin,
):
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
        self._init_safety_car_state()
        self.pit_window_open = False
        self.finished = False
        self.speed_multiplier = 1
        self.paused = False
        self.track_length_m = max(1.0, float(circuit.track_length_m))
        self._init_timing_state()
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
        self._init_racecraft_state()
        self._local_trajectory_planners: dict[int, LocalTrajectoryPlanner] = {}
        self._local_trajectory_plans: dict[int, LocalTrajectoryPlan] = {}
        self._ai_tactical_decision_elapsed = 0.0
        self._local_trajectory_plan_ages: dict[int, float] = {}
        self._local_trajectory_selected_ages: dict[int, float] = {}
        self._local_trajectory_validation_ages: dict[int, float] = {}
        self._local_trajectory_last_safe_plans: dict[int, LocalTrajectoryPlan] = {}
        self._local_trajectory_replan_counts: dict[int, int] = {}
        self._adaptive_planner_scheduler = AdaptivePlannerScheduler()
        self._planner_spatial_index = TrackSpatialIndex(self.track_length_m)

        self._init_strategy_state()
        self.driver_states: dict[int, DriverRaceState] = {}
        self._progress_rate: dict[int, float] = {}
        # Distance-integrated pit stop state (keyed by driver_id while in_pit).
        self._init_pit_ops_state()
        self._driver_meta: dict[int, dict] = {}
        self._finish_order: list[int] = []
        self._attack_line_choice_cache_bucket = -1
        self._attack_line_choice_cache: dict[tuple[int, int], str] = {}
        self._forced_wide_by_driver: dict[int, int] = {}
        self._pending_forced_wide_events: list[RaceEvent] = []
        self._pending_physical_handling_events: list[RaceEvent] = []
        self._physical_handling_event_cooldown: dict[int, float] = {}

        self._init_start_ops_state(start_sequence_enabled)
        self._init_incident_ops_state()
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

    def _center_gap_m(
        self,
        follower: DriverRaceState,
        leader: DriverRaceState,
    ) -> float:
        return (leader.total_progress - follower.total_progress) * self.track_length_m

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
        if (
            self.race_phase == "sc"
            and not state.in_pit
            and state.driver_id not in self._sc_unlap_driver_ids
        ):
            # Under Safety Car, lateral separation is not permission to pass.
            # Always constrain a car to the predecessor in the frozen sporting
            # order.  A physical inversion is handled by the smooth give-back
            # speed cap until this predecessor is ahead again.
            car_ahead = self._sc_queue_predecessor(state)
        else:
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

    def _total_progress_at_or_after(self, reference: float, progress: float) -> float:
        target = int(reference // 1.0) + (progress % 1.0)
        if target <= reference + PROGRESS_EPSILON:
            target += 1.0
        return target

    def _set_state_total_progress(self, state: DriverRaceState, total: float) -> None:
        lap = int(total // 1.0)
        state.current_lap = lap
        state.progress = total - lap
        state.total_progress = total
        state.total_distance_m = total * self.track_length_m

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

    def set_speed(self, multiplier: int) -> bool:
        if multiplier not in (1, 2):
            return False
        self.speed_multiplier = multiplier
        return True

    def pause_race(self) -> None:
        self.paused = True

    def resume_race(self) -> None:
        self.paused = False

    def _progress_distance(self, start: float, target: float) -> float:
        return (target - start) % 1.0

    def _crossed_progress(self, start: float, delta: float, target: float) -> bool:
        distance = self._progress_distance(start % 1.0, target % 1.0)
        return PROGRESS_EPSILON < distance <= delta + PROGRESS_EPSILON

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

    def _check_race_finished(self) -> bool:
        active = [
            s for s in self.driver_states.values()
            if not s.retired and not s.finished
        ]
        return len(active) == 0

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
