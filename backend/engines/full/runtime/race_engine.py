"""Core race simulation engine owned by the FULL engine family."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, floor, hypot, pi, sin, sqrt
import random
from typing import Any

from models.schemas import (
    Circuit,
    DryTireRole,
    Driver,
    DriverPoseInfo,
    DriverPositionInfo,
    DriverRaceHistoryInfo,
    DriverRaceState,
    LapTimeInfo,
    PaceMode,
    PhysicalTireCompound,
    RaceEvent,
    RaceHistoryState,
    RacePoseState,
    RaceTickState,
    Team,
    ThermalPresetName,
    TireCompound,
    TrackConditions,
    TrackSegmentType,
    VehicleTrajectorySample,
)
from simulation.car_performance import CarPerformanceFactors, car_performance_factors
from .brake_model import brake_temperature_force_factor
from .collision import (
    BodyPose,
    oriented_body_separation_m,
)
from .events import roll_events
from .incidents import (
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
from .fixed_step import FixedStepAccumulator
from .speed_profile import SpeedProfile, build_speed_profile
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
    track_physics_cache_counts,
)
from simulation.track_surface import TrackSurfaceProfile, VehicleSurfaceState
from .vehicle_physics import (
    FOLLOWING_PREDICTIVE_DECELERATION_MPS2,
    LOCKUP_SLIP_RATIO_THRESHOLD,
    PHYSICS_MAX_LATERAL_SPEED_MPS,
    PHYSICS_STEP_SECONDS,
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
    TireThermalBudget,
    TireThermalState,
    compound_spec_for,
    compute_tire_physics_factors,
    physical_compound_for,
    physical_compound_for_state,
    tire_grip_indices_c3,
)
from simulation.trajectory_physics import (
    TireTrajectorySpec,
    VehicleTrajectorySpec,
)
from .state_contract import (
    CollisionFact,
    TELEMETRY_SOURCE_DERIVED,
    PhysicsStepResult,
    ProgressCrossingFact,
    TickPhase,
)

from .wake_model import (
    WAKE_MAX_GAP_M,
    WAKE_MIN_GAP_M,
    WakeEffects,
    compute_wake_effects,
)
from .safety_car import (
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
    SC_QUEUE_RELATIVE_SPEED_FILTER_SECONDS,
    SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH,
    SC_QUEUE_STABLE_RELATIVE_SPEED_GRACE_SECONDS,
    SC_QUEUE_STABLE_RELATIVE_SPEED_KPH,
    SC_QUEUE_TARGET_CAR_LENGTHS,
    SC_RELEASE_GAP_CAR_LENGTHS,
    SC_RESTART_CONFIRMATION_TOLERANCE_PROGRESS,
    SC_UNLAP_LAP_TIME_FACTOR,
    SC_UNLAP_MAX_SPEED_KPH,
    SC_WITHDRAW_PIT_SECONDS,
    SafetyCarMixin,
    VSC_DURATION_SECONDS,
    VSC_LAP_TIME_FACTOR,
)

from .pit_ops import (
    PIT_LANE_SPEED_LIMIT_KPH,
    PitMergeDecision,
    PitOpsMixin,
)
from .racecraft_ops import (
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
from .timing_ops import (
    TIMING_CROSSING_LAPS_TO_RETAIN,
    TimingLoop,
    TimingOpsMixin,
)
from .strategy_ops import (
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
from .incident_ops import (
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
from .runtime_constants import GRID_LAUNCH_MERGE_DISTANCE_M
from .start_ops import (
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
TIRE_OVERHEAT_SURFACE_THRESHOLD_C = 130.0
TRACTION_LOSS_EVENT_SUSTAINED_SLIP_RATIO = 0.06
TRACTION_LOSS_EVENT_PEAK_SLIP_RATIO = 0.10
TRACTION_LOSS_EVENT_CONFIRM_SECONDS = 0.12

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
DRS_DETECTION_FALLBACK_DISTANCE_M = 200.0
DRS_INITIAL_ENABLE_AFTER_LEADER_LAPS = 1.0
DRS_SC_RESTART_DISABLED_LAPS = 1.0


def empty_tire_temperature_diagnostic_snapshot(
    track_conditions: TrackConditions | None = None,
    thermal_preset: ThermalPresetName | None = None,
    track_conditions_source: str = "explicit_override",
) -> dict[str, Any]:
    """Return the bounded tire-temperature diagnostic shape.

    This deliberately stores aggregate ranges and threshold evidence instead
    of a per-tick/per-driver telemetry history.  The desktop sampler can then
    identify sustained overheating without turning the diagnostic log into a
    second dashboard stream.
    """
    conditions = track_conditions or TrackConditions()
    source_names = ("baseline", "lateral", "braking", "traction", "slide")

    def cumulative_axle() -> dict[str, float]:
        return {
            **{f"{name}_heat_j": 0.0 for name in source_names},
            "surface_air_track_cooling_j": 0.0,
            "surface_to_core_transfer_j": 0.0,
            "core_ambient_cooling_j": 0.0,
            "surface_net_energy_j": 0.0,
            "core_net_energy_j": 0.0,
        }

    def current_axle() -> dict[str, Any]:
        return {
            "heat_input_w": 0.0,
            "cooling_w": 0.0,
            "net_w": 0.0,
            "source_heat_w": {
                f"{name}_heat_w": 0.0 for name in source_names
            },
        }

    def compound_entry(code: PhysicalTireCompound) -> dict[str, Any]:
        spec = compound_spec_for(code)
        return {
            "active_count": 0,
            "surface_min_c": None,
            "surface_max_c": None,
            "core_min_c": None,
            "core_max_c": None,
            "peak_surface_max_c": None,
            "peak_core_max_c": None,
            "operating_lower_c": round(
                spec.optimal_surface_temperature_c - spec.surface_operating_window_c,
                3,
            ),
            "operating_upper_c": round(
                spec.optimal_surface_temperature_c + spec.surface_operating_window_c,
                3,
            ),
            "hot_threshold_c": spec.hot_diagnostic_threshold_c,
            "current_overheat_count": 0,
            "current_overheat_driver_ids": [],
            "current_max_continuous_overheat_seconds": 0.0,
            "current_max_continuous_overheat_driver_id": None,
            "peak_overheat_count": 0,
            "peak_overheat_driver_ids": [],
            "max_continuous_overheat_seconds": 0.0,
            "max_continuous_overheat_driver_id": None,
        }

    return {
        "schema_version": 2,
        "track_conditions": {
            "ambient_temperature_c": conditions.ambient_temperature_c,
            "track_temperature_c": conditions.track_temperature_c,
        },
        "thermal_preset": (
            thermal_preset.value if isinstance(thermal_preset, ThermalPresetName)
            else thermal_preset
        ),
        "track_conditions_source": track_conditions_source,
        "sample_count": 0,
        "thresholds": {
            "rear_surface_overheat_c": TIRE_OVERHEAT_SURFACE_THRESHOLD_C,
            "legacy_130_c": TIRE_OVERHEAT_SURFACE_THRESHOLD_C,
        },
        "per_compound": {
            code.value: compound_entry(code)
            for code in PhysicalTireCompound
            if code.value.startswith("C")
        },
        "compound_distribution": {
            code.value: 0
            for code in PhysicalTireCompound
            if code.value.startswith("C")
        },
        "current": {
            "active_driver_count": 0,
            "front_surface_min_c": None,
            "front_surface_max_c": None,
            "rear_surface_min_c": None,
            "rear_surface_max_c": None,
            "front_core_min_c": None,
            "front_core_max_c": None,
            "rear_core_min_c": None,
            "rear_core_max_c": None,
            "rear_overheat_driver_count": 0,
            "rear_overheat_driver_ids": [],
            "compound_overheat_driver_count": 0,
            "compound_overheat_driver_ids": [],
            "max_compound_overheat_seconds": 0.0,
            "max_compound_overheat_driver_id": None,
            "max_current_overheat_seconds": 0.0,
            "max_current_overheat_driver_id": None,
            "front": current_axle(),
            "rear": current_axle(),
            "front_heat_input_w": 0.0,
            "rear_heat_input_w": 0.0,
            "front_cooling_w": 0.0,
            "rear_cooling_w": 0.0,
            "front_net_w": 0.0,
            "rear_net_w": 0.0,
            "front_source_heat_w": current_axle()["source_heat_w"].copy(),
            "rear_source_heat_w": current_axle()["source_heat_w"].copy(),
        },
        "peak": {
            "front_surface_max_c": None,
            "rear_surface_max_c": None,
            "front_core_max_c": None,
            "rear_core_max_c": None,
            "max_rear_overheat_driver_count": 0,
            "max_rear_overheat_driver_ids": [],
            "max_compound_overheat_driver_count": 0,
            "max_compound_overheat_driver_ids": [],
            "max_compound_overheat_seconds": 0.0,
            "max_compound_overheat_driver_id": None,
            "max_continuous_overheat_seconds": 0.0,
            "max_continuous_overheat_driver_id": None,
            "front": current_axle(),
            "rear": current_axle(),
            "front_heat_input_w": 0.0,
            "rear_heat_input_w": 0.0,
            "front_cooling_w": 0.0,
            "rear_cooling_w": 0.0,
            "front_net_w": 0.0,
            "rear_net_w": 0.0,
            "front_source_heat_w": current_axle()["source_heat_w"].copy(),
            "rear_source_heat_w": current_axle()["source_heat_w"].copy(),
        },
        "cumulative": {
            "front": cumulative_axle(),
            "rear": cumulative_axle(),
        },
        "clamp": {
            "current": {
                "surface_driver_ids": [],
                "core_driver_ids": [],
                "surface_active_seconds": 0.0,
                "core_active_seconds": 0.0,
                "max_continuous_seconds": 0.0,
                "max_continuous_driver_id": None,
            },
            "cumulative": {
                "surface_hit_count": 0,
                "core_hit_count": 0,
                "surface_active_seconds": 0.0,
                "core_active_seconds": 0.0,
            },
            "first": {
                "surface": None,
                "core": None,
            },
            "peak": {
                "surface_unclamped_temperature_c": None,
                "core_unclamped_temperature_c": None,
                "surface_overshoot_c": 0.0,
                "core_overshoot_c": 0.0,
                "max_continuous_seconds": 0.0,
            },
        },
    }

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
    "TIRE_OVERHEAT_SURFACE_THRESHOLD_C",
    "empty_tire_temperature_diagnostic_snapshot",
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
    "SC_QUEUE_RELATIVE_SPEED_FILTER_SECONDS",
    "SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH",
    "SC_QUEUE_STABLE_RELATIVE_SPEED_GRACE_SECONDS",
    "SC_QUEUE_STABLE_RELATIVE_SPEED_KPH",
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
        starting_physical_tires: dict[int, PhysicalTireCompound] | None = None,
        grid_order: list[int] | None = None,
        seed: int | None = None,
        start_sequence_enabled: bool = True,
        track_conditions: TrackConditions | None = None,
        thermal_preset: ThermalPresetName | None = None,
        track_conditions_source: str = "explicit_override",
        clean_line_lateral_speed_feedforward: float = 1.0,
        clean_line_lateral_speed_limit_mps: float = 1.25,
        solo_incidents_enabled: bool = True,
    ):
        self.circuit = circuit
        self.drivers = {d.id: d for d in drivers}
        self.teams = teams
        self.player_team_id = player_team_id
        self.player_driver_ids = set(player_driver_ids)
        self.starting_tires = starting_tires or {}
        self.starting_physical_tires = starting_physical_tires or {}
        self.grid_order = grid_order or []
        self.rng = random.Random(seed)
        self.track_conditions = track_conditions or TrackConditions()
        self.thermal_preset = thermal_preset
        self.track_conditions_source = track_conditions_source
        self.solo_incidents_enabled = bool(solo_incidents_enabled)
        # ``DriverRaceState.lateral_*`` is expressed from the compiled
        # centerline while the bicycle model integrates error from the active
        # path.  A clean car therefore needs the path offset derivative on
        # both sides of that frame conversion. The scalar remains injectable
        # for controlled calibration while production uses the complete paired
        # transport contract.
        self.clean_line_lateral_speed_feedforward = max(
            0.0,
            min(1.0, float(clean_line_lateral_speed_feedforward)),
        )
        self.clean_line_lateral_speed_limit_mps = max(
            0.5,
            min(PHYSICS_MAX_LATERAL_SPEED_MPS, float(clean_line_lateral_speed_limit_mps)),
        )

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
        self._tire_temperature_diagnostics = (
            empty_tire_temperature_diagnostic_snapshot(
                self.track_conditions,
                self.thermal_preset,
                self.track_conditions_source,
            )
        )
        self._rear_overheat_current_seconds: dict[int, float] = {}
        self._compound_overheat_current_seconds: dict[
            tuple[int, PhysicalTireCompound],
            float,
        ] = {}
        self._last_tire_thermal_states: dict[
            int,
            tuple[TireThermalState, TireThermalState],
        ] = {}
        self._tire_thermal_budget_by_driver: dict[
            int,
            tuple[TireThermalBudget, TireThermalBudget],
        ] = {}
        self._tire_clamp_state_by_driver: dict[
            int,
            tuple[bool, bool, bool, bool],
        ] = {}
        self._tire_clamp_current_seconds: dict[
            tuple[int, str],
            float,
        ] = {}
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
        self._drs_eligibility: dict[tuple[int, int], bool] = {}
        self._drs_enable_after_leader_total_progress = (
            DRS_INITIAL_ENABLE_AFTER_LEADER_LAPS
        )
        self._attack_line_choice_cache_bucket = -1
        self._attack_line_choice_cache: dict[tuple[int, int], str] = {}
        self._forced_wide_by_driver: dict[int, int] = {}
        self._pending_forced_wide_events: list[RaceEvent] = []
        self._pending_physical_handling_events: list[RaceEvent] = []
        self._physical_handling_event_cooldown: dict[int, float] = {}
        self._traction_loss_event_duration_s: dict[int, float] = {}

        self._init_start_ops_state(start_sequence_enabled)
        self._init_incident_ops_state()
        self._init_grid(drivers)
        self._initialize_driver_trajectory_physics()
        self._initialize_authoritative_vehicle_telemetry()

    @property
    def track_physics_profile(self) -> TrackPhysicsProfile:
        """Public compiled track profile shared with display serializers."""
        return self._track_physics

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
            "release_inward_recovery_speed_cap": (
                calibration.release_inward_recovery_speed_cap
                if calibration
                else False
            ),
        }
        telemetry_reference = (
            sorted(calibration.telemetry_reference, key=lambda item: item.progress)
            if calibration
            else []
        )
        telemetry_progress_offset = (
            calibration.telemetry_progress_offset if calibration else 0.0
        )

        def telemetry_speed_mps(progress: float) -> float | None:
            if not telemetry_reference:
                return None
            normalized = (progress + telemetry_progress_offset) % 1.0
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
            normalized = (progress + telemetry_progress_offset) % 1.0
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
                physical_compound_for_state(state),
                compute_tire_physics_factors(physical_compound_for_state(state), 0.0),
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
                    physical_compound_for_state(state),
                    track_profile=track_profile,
                    physics_by_line=physics_by_line,
                )

    def _initial_speed_kph(
        self,
        progress: float,
        car_factors: CarPerformanceFactors,
        driver_pace: float,
        tire_compound: PhysicalTireCompound | TireCompound,
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

    def physical_compound_for_role(
        self,
        role: DryTireRole | TireCompound | str,
    ) -> PhysicalTireCompound:
        """Resolve a weekend role through the session's circuit nomination."""
        from data_loader import resolve_tire_compound

        nomination = self.circuit.tire_compound_nomination
        if nomination is not None:
            return resolve_tire_compound(self.circuit, role)[0]
        return physical_compound_for(role)

    def set_driver_tire_compound(
        self,
        state: DriverRaceState,
        role: DryTireRole | TireCompound | str,
    ) -> PhysicalTireCompound:
        """Atomically update the legacy role, weekend role, and physical code."""
        from data_loader import resolve_tire_role

        resolved_role = resolve_tire_role(role)
        physical = self.physical_compound_for_role(resolved_role)
        state.tire_compound = TireCompound(resolved_role.value)
        state.tire_role = resolved_role
        state.physical_tire_compound = physical
        self._rear_overheat_current_seconds[state.driver_id] = 0.0
        for key in tuple(self._compound_overheat_current_seconds):
            if key[0] == state.driver_id:
                self._compound_overheat_current_seconds[key] = 0.0
        return physical

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
            brake_force_n=(
                car_factors.brake_force_n
                * brake_temperature_force_factor(
                    state.front_brake_temperature_c,
                    state.rear_brake_temperature_c,
                )
            ),
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
            center_of_gravity_height_m=0.30,
            wheel_radius_m=0.36,
            front_brake_bias=0.58,
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

    def _traction_loss_event_ready(
        self,
        driver_id: int,
        traction_slip_ratio: float,
        previous_traction_slip_ratio: float,
        delta: float,
    ) -> bool:
        previous_duration = self._traction_loss_event_duration_s.get(
            driver_id,
            0.0,
        )
        if traction_slip_ratio >= TRACTION_LOSS_EVENT_SUSTAINED_SLIP_RATIO:
            duration = previous_duration + max(0.0, delta)
        else:
            duration = 0.0
        self._traction_loss_event_duration_s[driver_id] = duration
        peak_crossed = (
            traction_slip_ratio >= TRACTION_LOSS_EVENT_PEAK_SLIP_RATIO
            and previous_traction_slip_ratio
            < TRACTION_LOSS_EVENT_PEAK_SLIP_RATIO
        )
        sustained_confirmed = (
            duration >= TRACTION_LOSS_EVENT_CONFIRM_SECONDS
            and previous_duration < TRACTION_LOSS_EVENT_CONFIRM_SECONDS
        )
        return peak_crossed or sustained_confirmed

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
        physical_car_ahead = car_ahead
        if (
            self.race_phase == "green"
            and not self._active_stopped_hazards()
        ):
            physical_car_ahead = self._nearest_physical_following_leader(
                state,
                start_snapshot,
                car_ahead,
            )
        following = self._physics_v2_following_constraint(
            state,
            physical_car_ahead,
            start_snapshot,
            active_line,
            delta,
        )
        if self._local_trajectory_basic_planning_allowed(state, active_line):
            self._update_local_trajectory_plan(
                state,
                active_line,
                None,
                delta,
                surface=surface,
                following=following,
                following_driver_id=(
                    physical_car_ahead.driver_id
                    if physical_car_ahead is not None
                    else None
                ),
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
        if self.race_phase == "green":
            state.target_lateral_offset_m = (
                self._guard_tactical_target_against_third_party(
                    state,
                    state.target_lateral_offset_m,
                )
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
            else self.clean_line_lateral_speed_limit_mps
        )
        minimum_lateral, maximum_lateral = self._track_surface.safety_lateral_bounds(
            state.progress
        )
        nominal_lateral_bounds = self._physics_v2_nominal_lateral_bounds(state)
        if (
            nominal_lateral_bounds is not None
            and self._nominal_line_edge_buffer_m() > 0.0
        ):
            state.target_lateral_offset_m = max(
                nominal_lateral_bounds[0],
                min(
                    nominal_lateral_bounds[1],
                    state.target_lateral_offset_m,
                ),
            )
        previous_wheel_lock_ratio = state.wheel_lock_ratio
        previous_traction_slip_ratio = state.traction_slip_ratio
        physics_modifiers = self._physics_v2_modifiers(state, surface)
        (
            target_lateral_speed_mps,
            reference_lateral_speed_mps,
        ) = self._physics_v2_lateral_speed_inputs(
            state,
            active_line,
            track_profile,
            target_lateral_speed_mps,
            tactical_lateral_motion=tactical_lateral_motion,
        )
        result = line_physics.advance(
            distance_m=start_distance_m,
            speed_mps=max(0.0, state.speed_kph / 3.6),
            delta_seconds=delta,
            modifiers=physics_modifiers,
            following=following,
            lateral_offset_m=state.lateral_offset_m,
            lateral_speed_mps=state.lateral_speed_mps,
            target_lateral_offset_m=state.target_lateral_offset_m,
            target_lateral_speed_mps=target_lateral_speed_mps,
            reference_lateral_offset_m=track_profile.line_offset_at_progress(
                active_line,
                state.progress,
            ),
            # The bicycle owns path curvature.  Matching target/reference
            # speeds only transports the centerline-relative state origin; it
            # does not add a second steering or lateral-force command.
            reference_lateral_speed_mps=reference_lateral_speed_mps,
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
        self._update_tire_thermal_state(
            state,
            result,
            delta,
            front_brake_bias=physics_modifiers.front_brake_bias,
        )
        self._update_brake_thermal_state(
            state,
            result,
            delta,
            front_brake_bias=physics_modifiers.front_brake_bias,
        )
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
        state.front_tire_slide_energy_j = round(
            result.front_tire_slide_energy_j,
            2,
        )
        state.rear_tire_slide_energy_j = round(
            result.rear_tire_slide_energy_j,
            2,
        )
        state.tire_slide_energy_j = round(result.tire_slide_energy_j, 2)
        state.rear_applied_drive_energy_j = round(
            result.rear_applied_drive_energy_j,
            2,
        )
        state.front_applied_brake_energy_j = round(
            result.front_applied_brake_energy_j,
            2,
        )
        state.rear_applied_brake_energy_j = round(
            result.rear_applied_brake_energy_j,
            2,
        )
        state.vehicle_mass_kg = round(result.vehicle_mass_kg, 3)
        state.front_normal_load_n = round(result.front_normal_load_n, 2)
        state.rear_normal_load_n = round(result.rear_normal_load_n, 2)
        state.longitudinal_load_transfer_n = round(
            result.longitudinal_load_transfer_n,
            2,
        )
        state.front_wheel_speed_rad_s = round(
            result.front_wheel_speed_rad_s,
            4,
        )
        state.rear_wheel_speed_rad_s = round(
            result.rear_wheel_speed_rad_s,
            4,
        )
        state.front_axle_slip_ratio = round(result.front_axle_slip_ratio, 5)
        state.rear_axle_slip_ratio = round(result.rear_axle_slip_ratio, 5)
        state.applied_brake_force_n = round(result.applied_brake_force_n, 2)
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
        traction_loss_event_ready = self._traction_loss_event_ready(
            state.driver_id,
            result.traction_slip_ratio,
            previous_traction_slip_ratio,
            delta,
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
            and traction_loss_event_ready
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

    def _nominal_line_edge_buffer_m(self) -> float:
        calibration = self.circuit.physics_calibration
        return calibration.nominal_line_edge_buffer_m if calibration else 0.0

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
        nominal_edge_buffer_m = self._nominal_line_edge_buffer_m()
        rear_minimum, rear_maximum = (
            self._track_surface.trajectory_body_lateral_bounds(
                state.progress - half_length_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=TRACK_EDGE_MARGIN_M + nominal_edge_buffer_m,
            )
        )
        front_minimum, front_maximum = (
            self._track_surface.trajectory_body_lateral_bounds(
                state.progress + half_length_progress,
                body_width_m=state.car_width_m,
                edge_margin_m=TRACK_EDGE_MARGIN_M + nominal_edge_buffer_m,
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
                # The optimizer may legally use a low kerb, while the live
                # four-wheel contact model also needs room for interpolation,
                # yaw and a narrowing next sample. Keep a small runtime buffer
                # so one wheel does not flicker onto runoff at T6.
                edge_margin_m=TRACK_EDGE_MARGIN_M + nominal_edge_buffer_m,
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
            or state.contact_active
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
        following: VehicleFollowingConstraint | None = None,
        following_driver_id: int | None = None,
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
                following_driver_id=(
                    following_driver_id if following is not None else None
                ),
                following_desired_gap_m=(
                    following.desired_gap_m if following is not None else 0.0
                ),
                following_minimum_gap_m=(
                    following.minimum_gap_m if following is not None else 0.0
                ),
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
            # Cached lateral geometry describes old traffic occupancy. Reusing
            # it after every current candidate becomes unsafe can prolong a
            # conflict for seconds. The 50 Hz following/collision controller
            # is the safe fallback for this bounded no-plan interval.
            self._local_trajectory_plans.pop(state.driver_id, None)
            expected_traffic_hold = (
                bool(nearby_vehicles)
                and not state.avoidance_active
                and not state.emergency_braking
            )
            state.planner_fallback_active = not expected_traffic_hold
            if expected_traffic_hold:
                state.planner_mode = "traffic_hold"
        if next_plan.selected.viable:
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
        correction = self._sc_order_correction
        if (
            self.race_phase == "sc"
            and correction is not None
            and correction.get("yielding_driver_id") == state.driver_id
        ):
            phase = correction.get("phase")
            if phase in {"MOVE_ASIDE", "YIELDING"}:
                target = correction.get("lateral_target_m")
                predecessor_id = correction.get("predecessor_driver_id")
                predecessor = (
                    self.driver_states.get(predecessor_id)
                    if isinstance(predecessor_id, int)
                    else None
                )
                if predecessor is not None:
                    refreshed_target = self._sc_order_correction_lateral_target(
                        state,
                        predecessor,
                    )
                    if refreshed_target is not None:
                        target = refreshed_target
                        correction["lateral_target_m"] = refreshed_target
                if isinstance(target, (int, float)):
                    return float(target)
            if phase == "MERGE_BACK":
                return track_profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
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
        correction = self._sc_order_correction
        if (
            correction is not None
            and correction.get("yielding_driver_id") == state.driver_id
            and correction.get("phase") in {"MOVE_ASIDE", "YIELDING"}
        ):
            response_seconds = 0.70
            return max(
                -PHYSICS_MAX_LATERAL_SPEED_MPS,
                min(
                    PHYSICS_MAX_LATERAL_SPEED_MPS,
                    (target_lateral_offset_m - state.lateral_offset_m)
                    / response_seconds,
                ),
            )
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

        # Clean-line reference transport is paired with the reference speed in
        # ``_physics_v2_lateral_speed_inputs``.  This method owns manoeuvre
        # motion only.
        return 0.0

    def _guard_tactical_target_against_third_party(
        self,
        state: DriverRaceState,
        target_lateral_offset_m: float,
    ) -> float:
        """Keep a validated pair maneuver from crossing an uninvolved car."""
        excluded_driver_ids = {state.driver_id}
        battle = self._side_by_side_battle_for_driver(state.driver_id)
        if battle is not None:
            excluded_driver_ids.update(
                (battle.attacker_id, battle.defender_id)
            )
        pending = self._pending_overtake_commands.get(state.driver_id)
        if pending is not None:
            excluded_driver_ids.add(pending.defender_id)

        guarded_target_m = target_lateral_offset_m
        nearby = sorted(
            (
                other
                for other in self.driver_states.values()
                if other.driver_id not in excluded_driver_ids
                and not other.in_pit
                and not other.retired
                and not other.finished
                and abs(other.total_progress - state.total_progress)
                * self.track_length_m
                <= 35.0
            ),
            key=lambda other: abs(
                other.total_progress - state.total_progress
            ),
        )
        for other in nearby:
            other_target_m = other.target_lateral_offset_m
            required_clearance_m = (
                self._physical_lateral_clearance_required_m(state, other)
                + 0.10
            )
            current_delta_m = state.lateral_offset_m - other.lateral_offset_m
            target_delta_m = guarded_target_m - other_target_m
            longitudinal_separation_m = abs(
                other.total_progress - state.total_progress
            ) * self.track_length_m
            already_side_by_side = (
                longitudinal_separation_m
                <= self._physical_longitudinal_half_extents_m(state, other) + 1.0
                and abs(current_delta_m) >= required_clearance_m * 0.65
            )
            if (
                abs(current_delta_m) < required_clearance_m
                and not already_side_by_side
            ):
                # Cars already sharing one corridor are handled
                # longitudinally. Near-width lateral overlap while the bodies
                # are longitudinally alongside is instead separated back into
                # the two established corridors.
                continue
            crosses_corridor = current_delta_m * target_delta_m <= 0.0
            finishes_too_close = abs(target_delta_m) < required_clearance_m
            if not crosses_corridor and not finishes_too_close:
                continue
            if abs(current_delta_m) > 0.10:
                side = 1.0 if current_delta_m > 0.0 else -1.0
            elif abs(target_delta_m) > 0.10:
                side = 1.0 if target_delta_m > 0.0 else -1.0
            else:
                side = 1.0 if state.driver_id < other.driver_id else -1.0
            guarded_target_m = other_target_m + side * required_clearance_m

        minimum_m, maximum_m = self._track_surface.trajectory_body_lateral_bounds(
            state.progress,
            body_width_m=state.car_width_m,
            edge_margin_m=TRACK_EDGE_MARGIN_M,
        )
        if guarded_target_m < minimum_m or guarded_target_m > maximum_m:
            # If the reserved side has no legal body corridor, hold position;
            # longitudinal following will create room before the rejoin.
            guarded_target_m = state.lateral_offset_m
        return max(minimum_m, min(maximum_m, guarded_target_m))

    def _physics_v2_lateral_speed_inputs(
        self,
        state: DriverRaceState,
        active_line: str,
        track_profile: TrackPhysicsProfile,
        target_lateral_speed_mps: float,
        *,
        tactical_lateral_motion: bool,
    ) -> tuple[float, float]:
        """Return centerline target speed and matching frame-reference speed.

        The dynamic bicycle subtracts the reference values before integration
        and adds them back afterwards.  Supplying only the reference derivative
        would turn a zero relative-speed target into ``-reference_speed`` and
        cancel the intended path transport.  A clean car therefore receives
        the same derivative as both target and reference speed.
        """
        if (
            active_line != DRIVING_LINE_RACING
            or tactical_lateral_motion
            or self.race_phase != "green"
            or self.clean_line_lateral_speed_feedforward <= 0.0
        ):
            return target_lateral_speed_mps, 0.0

        sample_distance_m = 1.0
        progress_delta = sample_distance_m / max(1.0, self.track_length_m)
        forward_total_progress = state.total_progress + progress_delta
        backward_total_progress = state.total_progress - progress_delta
        forward_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            forward_total_progress,
        )
        backward_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            backward_total_progress,
        )
        distance_span_m = forward_distance_m - backward_distance_m
        if distance_span_m <= 1e-9:
            return target_lateral_speed_mps, 0.0

        forward_offset_m = track_profile.line_offset_at_progress(
            active_line,
            forward_total_progress,
        )
        backward_offset_m = track_profile.line_offset_at_progress(
            active_line,
            backward_total_progress,
        )
        reference_lateral_speed_mps = (
            (forward_offset_m - backward_offset_m)
            / distance_span_m
            * max(0.0, state.speed_kph / 3.6)
            * self.clean_line_lateral_speed_feedforward
        )
        return reference_lateral_speed_mps, reference_lateral_speed_mps

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
        candidates: list[tuple[float, float, DriverRaceState]] = []
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
                    + candidate.lateral_speed_mps * 1.5
                )
                - (
                    state.lateral_offset_m
                    + state.lateral_speed_mps * 1.5
                )
            )
            target_lateral_gap_m = abs(
                candidate.target_lateral_offset_m
                - state.target_lateral_offset_m
            )
            required_lateral_clearance_m = (
                self._physical_lateral_clearance_required_m(
                    state,
                    candidate,
                )
            )
            longitudinally_overlapping = (
                longitudinal_gap_m
                <= self._physical_longitudinal_half_extents_m(
                    state,
                    candidate,
                )
                + FOLLOWING_MIN_BUMPER_GAP_M
            )
            separating_into_distinct_corridors = (
                projected_lateral_gap_m >= required_lateral_clearance_m
                and target_lateral_gap_m >= required_lateral_clearance_m
            )
            if longitudinally_overlapping and separating_into_distinct_corridors:
                continue
            shares_projected_lane = (
                min(
                    current_lateral_gap_m,
                    projected_lateral_gap_m,
                    target_lateral_gap_m,
                )
                < required_lateral_clearance_m
            )
            if not shares_projected_lane:
                continue
            correction = self._sc_order_correction
            if (
                correction is not None
                and correction.get("yielding_driver_id") == candidate.driver_id
                and correction.get("phase") in {"MOVE_ASIDE", "YIELDING"}
            ):
                lateral_target = correction.get("lateral_target_m")
                if isinstance(lateral_target, (int, float)):
                    pass_clearance = 0.5 * (
                        state.car_width_m + candidate.car_width_m
                    ) + 0.25
                    if abs(float(lateral_target) - state.lateral_offset_m) >= pass_clearance:
                        # The order controller has reserved a second legal
                        # corridor for this yielding car.  A physical follower
                        # may pass that car without treating the sporting
                        # predecessor as its leader.
                        continue
            if (
                self._physics_v2_passing_authorized(state, candidate)
                and projected_lateral_gap_m >= required_lateral_clearance_m
                and target_lateral_gap_m >= required_lateral_clearance_m
            ):
                continue
            candidate_speed_mps = leader_snapshot[1]
            minimum_longitudinal_gap_m = (
                self._physical_longitudinal_half_extents_m(
                    state,
                    candidate,
                )
                + FOLLOWING_MIN_BUMPER_GAP_M
            )
            braking_gap_m = max(
                0.0,
                longitudinal_gap_m - minimum_longitudinal_gap_m,
            )
            kinematic_safe_speed_mps = sqrt(
                max(
                    0.0,
                    candidate_speed_mps * candidate_speed_mps
                    + 2.0
                    * FOLLOWING_PREDICTIVE_DECELERATION_MPS2
                    * braking_gap_m,
                )
            )
            candidates.append(
                (
                    kinematic_safe_speed_mps,
                    longitudinal_gap_m,
                    candidate,
                )
            )
        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], item[1], item[2].driver_id))[2]

    def _nearest_physical_order_leader(
        self,
        state: DriverRaceState,
        start_snapshot: dict[int, tuple[float, float]],
    ) -> DriverRaceState | None:
        """Return the nearest actual body ahead when SC has no active pass."""
        follower_snapshot = start_snapshot.get(state.driver_id)
        if follower_snapshot is None:
            return None
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
            candidate_snapshot = start_snapshot.get(candidate.driver_id)
            if candidate_snapshot is None:
                continue
            gap_m = (candidate_snapshot[0] - follower_progress) * self.track_length_m
            if gap_m <= PROGRESS_EPSILON or gap_m > HAZARD_DETECTION_DISTANCE_M:
                continue
            candidates.append((gap_m, candidate))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def _sc_order_correction_allows_pass(
        self,
        follower: DriverRaceState,
        leader: DriverRaceState,
    ) -> bool:
        correction = self._sc_order_correction
        if correction is None:
            return False
        if (
            correction.get("phase") not in {"MOVE_ASIDE", "YIELDING"}
            or correction.get("yielding_driver_id") != leader.driver_id
            or correction.get("predecessor_driver_id") != follower.driver_id
        ):
            return False
        lateral_target = correction.get("lateral_target_m")
        if not isinstance(lateral_target, (int, float)):
            return False
        required_clearance = 0.5 * (
            follower.car_width_m + leader.car_width_m
        ) + 0.25
        actual_clearance = (
            abs(float(lateral_target) - follower.lateral_offset_m) >= required_clearance
            and abs(leader.lateral_offset_m - follower.lateral_offset_m)
            >= required_clearance
        )
        if actual_clearance:
            return True
        # The reserved corridor may be reached a fraction before the cars are
        # side-by-side.  Do not let a predecessor/leader feedback loop brake
        # both cars to zero while that longitudinal clearance still exists;
        # swept-body collision handling remains the final authority.
        longitudinal_gap_m = (
            leader.total_progress - follower.total_progress
        ) * self.track_length_m
        return (
            longitudinal_gap_m
            > max(
                0.75 * max(follower.car_length_m, leader.car_length_m),
                FOLLOWING_MIN_BUMPER_GAP_M + 0.25,
            )
            and abs(float(lateral_target) - follower.lateral_offset_m) >= 0.75
        )

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
            # QueuePlan owns the sporting predecessor and gap target.  This
            # constraint is only for collision safety, so it must always use
            # the nearest physical body in the active corridor.  A sporting
            # predecessor that is physically behind is never used as a
            # collision leader.
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
        ):
            return None
        projected_lateral_gap_m = abs(
            (
                car_ahead.lateral_offset_m
                + car_ahead.lateral_speed_mps * 1.5
            )
            - (state.lateral_offset_m + state.lateral_speed_mps * 1.5)
        )
        target_lateral_gap_m = abs(
            car_ahead.target_lateral_offset_m
            - state.target_lateral_offset_m
        )
        required_lateral_clearance_m = (
            self._physical_lateral_clearance_required_m(
                state,
                car_ahead,
            )
        )
        if (
            self._physics_v2_passing_authorized(state, car_ahead)
            and projected_lateral_gap_m >= required_lateral_clearance_m
            and target_lateral_gap_m >= required_lateral_clearance_m
        ):
            return None
        hazard_recovery_active = getattr(self, "_hazard_recovery_active", None)
        if (
            self.race_phase == "sc"
            and self.safety_car_stage == "collecting"
            and callable(hazard_recovery_active)
            and hazard_recovery_active(state)
            and state.handling_state == "recovering"
            and car_ahead.handling_state == "recovering"
            and state.speed_kph < 5.0
            and 0.75 * state.car_length_m
            <= (car_ahead.total_progress - state.total_progress)
            * self.track_length_m
        ):
            # A blocked incident can leave several cars compressed in the same
            # temporary escape corridor.  Release this bounded recovery pair
            # while a body-length of longitudinal room remains; swept-body
            # collision resolution still owns the actual safety decision.
            return None
        if self.race_phase == "sc" and self._sc_order_correction_allows_pass(
            state,
            car_ahead,
        ):
            # The active sporting predecessor has a reserved, clear corridor.
            # Let that one physical pass proceed; swept-body collision checks
            # remain authoritative for the actual clearance.
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
            self._physical_longitudinal_half_extents_m(state, car_ahead)
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

    def _drs_zone_index(self, progress: float) -> int | None:
        """Return the activation-zone index containing ``progress``."""
        normalized = progress % 1.0
        for index, zone in enumerate(self.circuit.drs_zones):
            start = float(zone.start) % 1.0
            end = float(zone.end) % 1.0
            if start <= end and start <= normalized <= end:
                return index
            if start > end and (normalized >= start or normalized <= end):
                return index
        return None

    def _is_drs_zone(self, progress: float) -> bool:
        """Return whether current track progress is inside a DRS activation zone."""
        return self._drs_zone_index(progress) is not None

    def _drs_detection_progress(self, zone_index: int) -> float:
        zone = self.circuit.drs_zones[zone_index]
        if zone.detection is not None:
            return float(zone.detection) % 1.0
        fallback_progress = DRS_DETECTION_FALLBACK_DISTANCE_M / self.track_length_m
        return (float(zone.start) - fallback_progress) % 1.0

    @staticmethod
    def _crossed_progress_anchor(
        start_total_progress: float,
        end_total_progress: float,
        anchor_progress: float,
    ) -> bool:
        if end_total_progress <= start_total_progress:
            return False
        return floor(end_total_progress - anchor_progress + PROGRESS_EPSILON) > floor(
            start_total_progress - anchor_progress + PROGRESS_EPSILON
        )

    def _drs_race_control_enabled(self) -> bool:
        if not self.race_started or self.race_phase != "green":
            return False
        leader = self._on_track_leader()
        return bool(
            leader is not None
            and leader.total_progress + PROGRESS_EPSILON
            >= self._drs_enable_after_leader_total_progress
        )

    def _clear_drs_eligibility(self) -> None:
        self._drs_eligibility.clear()
        for state in self.driver_states.values():
            state.drs_active = False

    def _set_drs_restart_lockout(self) -> None:
        leader = self._on_track_leader()
        leader_progress = leader.total_progress if leader is not None else 0.0
        self._drs_enable_after_leader_total_progress = (
            leader_progress + DRS_SC_RESTART_DISABLED_LAPS
        )
        self._clear_drs_eligibility()

    def _update_drs_detection_eligibility(
        self,
        start_snapshot: dict[int, tuple[float, float]],
    ) -> None:
        """Latch DRS qualification only when a car crosses a detection line."""
        if not self._drs_race_control_enabled():
            self._clear_drs_eligibility()
            return

        running = sorted(
            (
                state
                for state in self.driver_states.values()
                if not state.retired and not state.finished and not state.in_pit
            ),
            key=lambda state: (-state.total_progress, state.position),
        )
        ahead_by_driver = {
            state.driver_id: (running[index - 1] if index > 0 else None)
            for index, state in enumerate(running)
        }
        for state in running:
            start = start_snapshot.get(state.driver_id, (state.total_progress, 0.0))[0]
            for zone_index in range(len(self.circuit.drs_zones)):
                detection = self._drs_detection_progress(zone_index)
                if not self._crossed_progress_anchor(
                    start,
                    state.total_progress,
                    detection,
                ):
                    continue
                car_ahead = ahead_by_driver[state.driver_id]
                eligible = False
                if car_ahead is not None:
                    progress_gap = car_ahead.total_progress - state.total_progress
                    gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
                    eligible = (
                        0.0 < gap_seconds <= TRAFFIC_GAP_SECONDS
                        and self._overtaking_candidate_allowed(state, car_ahead)
                    )
                self._drs_eligibility[(state.driver_id, zone_index)] = eligible

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

        zone_index = self._drs_zone_index(state.progress)
        if car_ahead is not None and zone_index is not None:
            progress_gap = car_ahead.total_progress - state.total_progress
            gap_seconds = self._progress_gap_to_seconds(progress_gap, state)
            state.drs_active = (
                self._drs_race_control_enabled()
                and self._drs_eligibility.get((state.driver_id, zone_index), False)
                and 0.0 < gap_seconds
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
            if self._sc_restart_hold_ticks > 0 and self._physics_frame > 0:
                self._sc_restart_hold_ticks -= 1
                return
            leader = self._on_track_leader()
            restart_entered_this_tick = (
                self._tick_phase is TickPhase.RULES
                and abs(self.race_elapsed - self._sc_restart_entered_at) <= 1e-9
            )
            if (
                leader is not None
                and not restart_entered_this_tick
                and (
                    leader.total_progress >= self._sc_restart_target
                    or (
                        self.race_elapsed > self._sc_restart_entered_at
                        and leader.total_progress >= (
                            self._sc_restart_target
                            - SC_RESTART_CONFIRMATION_TOLERANCE_PROGRESS
                        )
                    )
                )
            ):
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
                    continue
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
                else:
                    # The first command pass above already installed every
                    # safe straight-line pull-out.  A missing battle intent is
                    # not permission to synthesize an unvalidated tactical
                    # line, especially under braking.
                    continue
                if decision is None:
                    continue
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
                * self._circuit_tire_usage_per_lap()
                * self._tire_thermal_usage_multiplier(state)
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
        if self.race_phase == "green":
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

            if (
                self.solo_incidents_enabled
                and self.race_phase == "green"
                and not state.retired
                and not state.finished
            ):
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

        self._update_drs_detection_eligibility(physics_start_snapshot)
        if self.race_phase == "green":
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
        if self.race_phase == "green":
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
        self._record_tire_temperature_diagnostics(delta)

        return events

    @staticmethod
    def _add_tire_thermal_budgets(
        first: TireThermalBudget,
        second: TireThermalBudget,
    ) -> TireThermalBudget:
        fields = (
            "baseline_heat_j",
            "lateral_heat_j",
            "braking_heat_j",
            "traction_heat_j",
            "slide_heat_j",
            "surface_air_track_cooling_j",
            "surface_to_core_transfer_j",
            "core_ambient_cooling_j",
            "surface_net_energy_j",
            "core_net_energy_j",
        )
        return TireThermalBudget(
            **{
                field: getattr(first, field) + getattr(second, field)
                for field in fields
            }
        )

    @staticmethod
    def _tire_budget_as_dict(budget: TireThermalBudget) -> dict[str, float]:
        return {
            field: round(float(getattr(budget, field)), 6)
            for field in (
                "baseline_heat_j",
                "lateral_heat_j",
                "braking_heat_j",
                "traction_heat_j",
                "slide_heat_j",
                "surface_air_track_cooling_j",
                "surface_to_core_transfer_j",
                "core_ambient_cooling_j",
                "surface_net_energy_j",
                "core_net_energy_j",
            )
        }

    def _accumulate_tire_thermal_diagnostics(
        self,
        state: DriverRaceState,
        front_thermal: TireThermalState,
        rear_thermal: TireThermalState,
        delta_seconds: float,
    ) -> None:
        """Accumulate bounded per-step thermal facts before the next snapshot."""
        driver_id = state.driver_id
        previous = self._tire_thermal_budget_by_driver.get(
            driver_id,
            (TireThermalBudget.zero(), TireThermalBudget.zero()),
        )
        self._tire_thermal_budget_by_driver[driver_id] = (
            self._add_tire_thermal_budgets(previous[0], front_thermal.budget),
            self._add_tire_thermal_budgets(previous[1], rear_thermal.budget),
        )
        flags = (
            front_thermal.surface_clamp_hit,
            front_thermal.core_clamp_hit,
            rear_thermal.surface_clamp_hit,
            rear_thermal.core_clamp_hit,
        )
        self._tire_clamp_state_by_driver[driver_id] = flags
        step = max(0.0, float(delta_seconds))
        diagnostics = self._tire_temperature_diagnostics
        clamp = diagnostics["clamp"]
        cumulative = clamp["cumulative"]
        for axle, thermal in (
            ("front", front_thermal),
            ("rear", rear_thermal),
        ):
            for node, hit, unclamped, overshoot in (
                (
                    "surface",
                    thermal.surface_clamp_hit,
                    thermal.surface_unclamped_temperature_c,
                    thermal.surface_clamp_overshoot_c,
                ),
                (
                    "core",
                    thermal.core_clamp_hit,
                    thermal.core_unclamped_temperature_c,
                    thermal.core_clamp_overshoot_c,
                ),
            ):
                key = (driver_id, f"{axle}_{node}")
                if hit:
                    current_seconds = (
                        self._tire_clamp_current_seconds.get(key, 0.0)
                        + step
                    )
                    self._tire_clamp_current_seconds[key] = current_seconds
                    cumulative[f"{node}_hit_count"] += 1
                    cumulative[f"{node}_active_seconds"] += step
                    first = clamp["first"][node]
                    if first is None:
                        clamp["first"][node] = {
                            "timestamp_seconds": round(self.race_elapsed, 3),
                            "lap": state.current_lap,
                            "driver_id": driver_id,
                            "compound": state.tire_compound.value,
                            "tire_role": state.tire_role.value,
                            "physical_tire_compound": physical_compound_for_state(state).value,
                            "axle": axle,
                            "node": node,
                            "unclamped_temperature_c": round(unclamped, 6),
                            "overshoot_c": round(overshoot, 6),
                            "track_conditions": {
                                "ambient_temperature_c": self.track_conditions.ambient_temperature_c,
                                "track_temperature_c": self.track_conditions.track_temperature_c,
                            },
                            "heat_budget": self._tire_budget_as_dict(
                                thermal.budget
                            ),
                        }
                    peak = clamp["peak"]
                    temperature_key = f"{node}_unclamped_temperature_c"
                    overshoot_key = f"{node}_overshoot_c"
                    peak[temperature_key] = round(
                        max(peak[temperature_key] or unclamped, unclamped),
                        6,
                    )
                    peak[overshoot_key] = round(
                        max(peak[overshoot_key], overshoot),
                        6,
                    )
                    peak["max_continuous_seconds"] = round(
                        max(peak["max_continuous_seconds"], current_seconds),
                        6,
                    )
                else:
                    self._tire_clamp_current_seconds[key] = 0.0

    def _record_compound_temperature_diagnostics(self, active_states: list[DriverRaceState]) -> None:
        """Update bounded current and historical C1-C5 aggregates."""
        diagnostics = self._tire_temperature_diagnostics
        per_compound = diagnostics["per_compound"]
        distribution = diagnostics["compound_distribution"]
        aggregate_overheat_ids: set[int] = set()
        observed_states = [
            state for state in self.driver_states.values() if not state.retired
        ]
        for code in PhysicalTireCompound:
            if not code.value.startswith("C"):
                continue
            entry = per_compound[code.value]
            states = [
                state for state in active_states
                if physical_compound_for_state(state) == code
            ]
            observed = [
                state for state in observed_states
                if physical_compound_for_state(state) == code
            ]
            entry["active_count"] = len(states)
            distribution[code.value] = len(states)
            if observed:
                entry["peak_surface_max_c"] = round(
                    max(
                        entry["peak_surface_max_c"] or observed[0].rear_tire_surface_temperature_c,
                        max(state.rear_tire_surface_temperature_c for state in observed),
                    ),
                    3,
                )
                entry["peak_core_max_c"] = round(
                    max(
                        entry["peak_core_max_c"] or observed[0].rear_tire_core_temperature_c,
                        max(state.rear_tire_core_temperature_c for state in observed),
                    ),
                    3,
                )
            if not states:
                entry["surface_min_c"] = None
                entry["surface_max_c"] = None
                entry["core_min_c"] = None
                entry["core_max_c"] = None
                entry["current_overheat_count"] = 0
                entry["current_overheat_driver_ids"] = []
                entry["current_max_continuous_overheat_seconds"] = 0.0
                entry["current_max_continuous_overheat_driver_id"] = None
                continue
            surfaces = [state.rear_tire_surface_temperature_c for state in states]
            cores = [state.rear_tire_core_temperature_c for state in states]
            entry["surface_min_c"] = round(min(surfaces), 3)
            entry["surface_max_c"] = round(max(surfaces), 3)
            entry["core_min_c"] = round(min(cores), 3)
            entry["core_max_c"] = round(max(cores), 3)
            overheat_ids = sorted(
                state.driver_id
                for state in states
                if state.rear_tire_surface_temperature_c >= entry["hot_threshold_c"]
            )
            entry["current_overheat_count"] = len(overheat_ids)
            entry["current_overheat_driver_ids"] = overheat_ids
            aggregate_overheat_ids.update(overheat_ids)
            current_seconds = max(
                (
                    self._compound_overheat_current_seconds.get(
                        (state.driver_id, code),
                        0.0,
                    ),
                    state.driver_id,
                )
                for state in states
            )
            entry["current_max_continuous_overheat_seconds"] = round(
                current_seconds[0],
                3,
            )
            entry["current_max_continuous_overheat_driver_id"] = (
                current_seconds[1] if current_seconds[0] > 0.0 else None
            )
            if current_seconds[0] > entry["max_continuous_overheat_seconds"]:
                entry["max_continuous_overheat_seconds"] = round(
                    current_seconds[0],
                    3,
                )
                entry["max_continuous_overheat_driver_id"] = current_seconds[1]
            if len(overheat_ids) > entry["peak_overheat_count"]:
                entry["peak_overheat_count"] = len(overheat_ids)
                entry["peak_overheat_driver_ids"] = overheat_ids.copy()
        current = diagnostics["current"]
        peak = diagnostics["peak"]
        current["compound_overheat_driver_ids"] = sorted(aggregate_overheat_ids)
        current["compound_overheat_driver_count"] = len(aggregate_overheat_ids)
        current_compound_seconds = max(
            (
                entry["current_max_continuous_overheat_seconds"],
                code,
                entry["current_max_continuous_overheat_driver_id"],
            )
            for code, entry in per_compound.items()
        )
        current["max_compound_overheat_seconds"] = round(
            current_compound_seconds[0],
            3,
        )
        current["max_compound_overheat_driver_id"] = (
            current_compound_seconds[2]
            if current_compound_seconds[0] > 0.0
            else None
        )
        if len(aggregate_overheat_ids) > peak["max_compound_overheat_driver_count"]:
            peak["max_compound_overheat_driver_count"] = len(aggregate_overheat_ids)
            peak["max_compound_overheat_driver_ids"] = sorted(aggregate_overheat_ids)
        if current_compound_seconds[0] > peak["max_compound_overheat_seconds"]:
            peak["max_compound_overheat_seconds"] = round(
                current_compound_seconds[0],
                3,
            )
            peak["max_compound_overheat_driver_id"] = current_compound_seconds[2]

    def _record_tire_temperature_diagnostics(self, delta_seconds: float) -> None:
        """Update bounded aggregate evidence for sustained rear overheating."""
        diagnostics = self._tire_temperature_diagnostics
        active_states = [
            state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        ]
        active_driver_ids = {state.driver_id for state in active_states}
        step = max(0.0, float(delta_seconds))

        pending_budgets = self._tire_thermal_budget_by_driver
        cumulative = diagnostics["cumulative"]
        for front_budget, rear_budget in pending_budgets.values():
            for axle, budget in (
                ("front", front_budget),
                ("rear", rear_budget),
            ):
                cumulative_axle = cumulative[axle]
                for field in cumulative_axle:
                    cumulative_axle[field] += getattr(budget, field)

        def budget_snapshot(budget: TireThermalBudget) -> dict[str, Any]:
            heat_fields = (
                "baseline_heat_j",
                "lateral_heat_j",
                "braking_heat_j",
                "traction_heat_j",
                "slide_heat_j",
            )
            heat_input_j = sum(getattr(budget, field) for field in heat_fields)
            cooling_j = (
                budget.surface_air_track_cooling_j
                + budget.core_ambient_cooling_j
            )
            return {
                "heat_input_w": round(heat_input_j / step, 6) if step else 0.0,
                "cooling_w": round(cooling_j / step, 6) if step else 0.0,
                "net_w": round(
                    (budget.surface_net_energy_j + budget.core_net_energy_j)
                    / step,
                    6,
                )
                if step
                else 0.0,
                "source_heat_w": {
                    f"{name}_heat_w": round(
                        getattr(budget, f"{name}_heat_j") / step,
                        6,
                    )
                    if step
                    else 0.0
                    for name in (
                        "baseline",
                        "lateral",
                        "braking",
                        "traction",
                        "slide",
                    )
                },
            }

        current_budgets = {
            "front": TireThermalBudget.zero(),
            "rear": TireThermalBudget.zero(),
        }
        for state in active_states:
            pending = pending_budgets.get(
                state.driver_id,
                (TireThermalBudget.zero(), TireThermalBudget.zero()),
            )
            current_budgets["front"] = self._add_tire_thermal_budgets(
                current_budgets["front"],
                pending[0],
            )
            current_budgets["rear"] = self._add_tire_thermal_budgets(
                current_budgets["rear"],
                pending[1],
            )
        for axle in ("front", "rear"):
            snapshot = budget_snapshot(current_budgets[axle])
            diagnostics["current"][axle] = snapshot
            diagnostics["current"][f"{axle}_heat_input_w"] = snapshot[
                "heat_input_w"
            ]
            diagnostics["current"][f"{axle}_cooling_w"] = snapshot[
                "cooling_w"
            ]
            diagnostics["current"][f"{axle}_net_w"] = snapshot["net_w"]
            diagnostics["current"][f"{axle}_source_heat_w"] = snapshot[
                "source_heat_w"
            ]
            previous_peak = diagnostics["peak"][axle]
            previous_peak["heat_input_w"] = max(
                previous_peak["heat_input_w"],
                snapshot["heat_input_w"],
            )
            previous_peak["cooling_w"] = max(
                previous_peak["cooling_w"],
                snapshot["cooling_w"],
            )
            previous_peak["net_w"] = max(
                previous_peak["net_w"],
                snapshot["net_w"],
            )
            for source, value in snapshot["source_heat_w"].items():
                previous_peak["source_heat_w"][source] = max(
                    previous_peak["source_heat_w"][source],
                    value,
                )
            diagnostics["peak"][f"{axle}_heat_input_w"] = previous_peak[
                "heat_input_w"
            ]
            diagnostics["peak"][f"{axle}_cooling_w"] = previous_peak[
                "cooling_w"
            ]
            diagnostics["peak"][f"{axle}_net_w"] = previous_peak["net_w"]
            diagnostics["peak"][f"{axle}_source_heat_w"] = previous_peak[
                "source_heat_w"
            ]
        self._tire_thermal_budget_by_driver = {}

        for driver_id in self.driver_states:
            if driver_id not in active_driver_ids:
                self._rear_overheat_current_seconds[driver_id] = 0.0
                for code in PhysicalTireCompound:
                    if code.value.startswith("C"):
                        self._compound_overheat_current_seconds[
                            (driver_id, code)
                        ] = 0.0
                continue
            state = self.driver_states[driver_id]
            if state.rear_tire_surface_temperature_c >= TIRE_OVERHEAT_SURFACE_THRESHOLD_C:
                self._rear_overheat_current_seconds[driver_id] = (
                    self._rear_overheat_current_seconds.get(driver_id, 0.0)
                    + step
                )
            else:
                self._rear_overheat_current_seconds[driver_id] = 0.0
            physical_code = physical_compound_for_state(state)
            for code in PhysicalTireCompound:
                if not code.value.startswith("C"):
                    continue
                key = (driver_id, code)
                if (
                    code == physical_code
                    and state.rear_tire_surface_temperature_c
                    >= compound_spec_for(code).hot_diagnostic_threshold_c
                ):
                    self._compound_overheat_current_seconds[key] = (
                        self._compound_overheat_current_seconds.get(key, 0.0)
                        + step
                    )
                else:
                    self._compound_overheat_current_seconds[key] = 0.0

        diagnostics["sample_count"] += 1
        current = diagnostics["current"]
        peak = diagnostics["peak"]
        current["active_driver_count"] = len(active_states)
        self._record_compound_temperature_diagnostics(active_states)

        if not active_states:
            current["rear_overheat_driver_count"] = 0
            current["rear_overheat_driver_ids"] = []
            current["max_current_overheat_seconds"] = 0.0
            current["max_current_overheat_driver_id"] = None
            diagnostics["clamp"]["current"] = {
                "surface_driver_ids": [],
                "core_driver_ids": [],
                "surface_active_seconds": 0.0,
                "core_active_seconds": 0.0,
                "max_continuous_seconds": 0.0,
                "max_continuous_driver_id": None,
            }
            return

        fields = {
            "front_surface": [
                state.front_tire_surface_temperature_c for state in active_states
            ],
            "rear_surface": [
                state.rear_tire_surface_temperature_c for state in active_states
            ],
            "front_core": [
                state.front_tire_core_temperature_c for state in active_states
            ],
            "rear_core": [
                state.rear_tire_core_temperature_c for state in active_states
            ],
        }
        for name, values in fields.items():
            current[f"{name}_min_c"] = round(min(values), 3)
            current[f"{name}_max_c"] = round(max(values), 3)
            peak[f"{name}_max_c"] = round(
                max(peak[f"{name}_max_c"] or values[0], max(values)),
                3,
            )

        overheat_ids = sorted(
            state.driver_id
            for state in active_states
            if state.rear_tire_surface_temperature_c
            >= TIRE_OVERHEAT_SURFACE_THRESHOLD_C
        )
        current["rear_overheat_driver_count"] = len(overheat_ids)
        current["rear_overheat_driver_ids"] = overheat_ids
        if len(overheat_ids) > peak["max_rear_overheat_driver_count"]:
            peak["max_rear_overheat_driver_count"] = len(overheat_ids)
            peak["max_rear_overheat_driver_ids"] = overheat_ids.copy()
        max_current_seconds = max(
            (
                self._rear_overheat_current_seconds.get(state.driver_id, 0.0),
                state.driver_id,
            )
            for state in active_states
        )
        current["max_current_overheat_seconds"] = round(max_current_seconds[0], 3)
        current["max_current_overheat_driver_id"] = (
            max_current_seconds[1] if max_current_seconds[0] > 0.0 else None
        )
        if max_current_seconds[0] > peak["max_continuous_overheat_seconds"]:
            peak["max_continuous_overheat_seconds"] = round(
                max_current_seconds[0],
                3,
            )
            peak["max_continuous_overheat_driver_id"] = max_current_seconds[1]

        clamp_current = diagnostics["clamp"]["current"]
        surface_driver_ids = sorted(
            state.driver_id
            for state in active_states
            if self._tire_clamp_state_by_driver.get(state.driver_id, (False,) * 4)[0]
            or self._tire_clamp_state_by_driver.get(state.driver_id, (False,) * 4)[2]
        )
        core_driver_ids = sorted(
            state.driver_id
            for state in active_states
            if self._tire_clamp_state_by_driver.get(state.driver_id, (False,) * 4)[1]
            or self._tire_clamp_state_by_driver.get(state.driver_id, (False,) * 4)[3]
        )
        surface_seconds = max(
            (
                max(
                    self._tire_clamp_current_seconds.get(
                        (state.driver_id, "front_surface"),
                        0.0,
                    ),
                    self._tire_clamp_current_seconds.get(
                        (state.driver_id, "rear_surface"),
                        0.0,
                    ),
                ),
                state.driver_id,
            )
            for state in active_states
        )
        core_seconds = max(
            (
                max(
                    self._tire_clamp_current_seconds.get(
                        (state.driver_id, "front_core"),
                        0.0,
                    ),
                    self._tire_clamp_current_seconds.get(
                        (state.driver_id, "rear_core"),
                        0.0,
                    ),
                ),
                state.driver_id,
            )
            for state in active_states
        )
        max_clamp = max(surface_seconds, core_seconds)
        clamp_current.update(
            {
                "surface_driver_ids": surface_driver_ids,
                "core_driver_ids": core_driver_ids,
                "surface_active_seconds": round(surface_seconds[0], 6),
                "core_active_seconds": round(core_seconds[0], 6),
                "max_continuous_seconds": round(max_clamp[0], 6),
                "max_continuous_driver_id": (
                    max_clamp[1] if max_clamp[0] > 0.0 else None
                ),
            }
        )

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
        retired.sort(
            key=lambda state: (
                -state.total_progress,
                state.total_time,
                state.driver_id,
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
            (
                tire_lateral_grip_index,
                tire_traction_grip_index,
                tire_braking_grip_index,
            ) = tire_grip_indices_c3(tire_factors)

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
                    front_tire_slide_energy_j=round(
                        state.front_tire_slide_energy_j,
                        2,
                    ),
                    rear_tire_slide_energy_j=round(
                        state.rear_tire_slide_energy_j,
                        2,
                    ),
                    tire_slide_energy_j=state.tire_slide_energy_j,
                    rear_applied_drive_energy_j=round(
                        state.rear_applied_drive_energy_j,
                        2,
                    ),
                    front_applied_brake_energy_j=round(
                        state.front_applied_brake_energy_j,
                        2,
                    ),
                    rear_applied_brake_energy_j=round(
                        state.rear_applied_brake_energy_j,
                        2,
                    ),
                    vehicle_mass_kg=state.vehicle_mass_kg,
                    front_normal_load_n=state.front_normal_load_n,
                    rear_normal_load_n=state.rear_normal_load_n,
                    longitudinal_load_transfer_n=(
                        state.longitudinal_load_transfer_n
                    ),
                    front_wheel_speed_rad_s=state.front_wheel_speed_rad_s,
                    rear_wheel_speed_rad_s=state.rear_wheel_speed_rad_s,
                    front_axle_slip_ratio=state.front_axle_slip_ratio,
                    rear_axle_slip_ratio=state.rear_axle_slip_ratio,
                    applied_brake_force_n=state.applied_brake_force_n,
                    front_brake_temperature_c=(
                        state.front_brake_temperature_c
                    ),
                    rear_brake_temperature_c=state.rear_brake_temperature_c,
                    brake_fade_factor=state.brake_fade_factor,
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
                    tire_role=state.tire_role.value,
                    physical_tire_compound=physical_compound_for_state(state).value,
                    tire_age=state.tire_age,
                    tire_wear=round(tire_factors.wear, 3),
                    tire_lateral_grip=round(tire_factors.lateral_grip, 4),
                    tire_traction_grip=round(tire_factors.traction_grip, 4),
                    tire_braking_grip=round(tire_factors.braking_grip, 4),
                    tire_lateral_grip_index=round(tire_lateral_grip_index, 4),
                    tire_traction_grip_index=round(tire_traction_grip_index, 4),
                    tire_braking_grip_index=round(tire_braking_grip_index, 4),
                    tire_surface_temperature_c=round(
                        state.tire_surface_temperature_c,
                        2,
                    ),
                    tire_core_temperature_c=round(
                        state.tire_core_temperature_c,
                        2,
                    ),
                    tire_thermal_grip=round(tire_factors.thermal_grip, 4),
                    front_tire_surface_temperature_c=round(
                        state.front_tire_surface_temperature_c,
                        2,
                    ),
                    front_tire_core_temperature_c=round(
                        state.front_tire_core_temperature_c,
                        2,
                    ),
                    front_tire_thermal_grip=round(
                        state.front_tire_thermal_grip,
                        4,
                    ),
                    rear_tire_surface_temperature_c=round(
                        state.rear_tire_surface_temperature_c,
                        2,
                    ),
                    rear_tire_core_temperature_c=round(
                        state.rear_tire_core_temperature_c,
                        2,
                    ),
                    rear_tire_thermal_grip=round(
                        state.rear_tire_thermal_grip,
                        4,
                    ),
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
                    pit_main_route_progress=round(
                        self._pit_main_route_progress(state.driver_id),
                        7,
                    ),
                    pit_exit_lane_progress=round(
                        self._pit_exit_lane_progress_value(state.driver_id),
                        7,
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
            track_conditions=self.track_conditions,
            thermal_preset=self.thermal_preset,
            track_conditions_source=self.track_conditions_source,
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
            tire_factors = self._current_tire_physics(state)
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
                    "tire_role": state.tire_role.value,
                    "physical_tire_compound": physical_compound_for_state(state).value,
                    "tire_age": state.tire_age,
                    "tire_wear": round(tire_factors.wear, 3),
                    "tire_lateral_grip": round(
                        tire_factors.lateral_grip,
                        4,
                    ),
                    "tire_traction_grip": round(
                        tire_factors.traction_grip,
                        4,
                    ),
                    "tire_braking_grip": round(
                        tire_factors.braking_grip,
                        4,
                    ),
                    "tire_surface_temperature_c": round(
                        state.tire_surface_temperature_c,
                        2,
                    ),
                    "tire_core_temperature_c": round(
                        state.tire_core_temperature_c,
                        2,
                    ),
                    "tire_thermal_grip": round(
                        tire_factors.thermal_grip,
                        4,
                    ),
                    "front_tire_surface_temperature_c": round(
                        state.front_tire_surface_temperature_c,
                        2,
                    ),
                    "front_tire_core_temperature_c": round(
                        state.front_tire_core_temperature_c,
                        2,
                    ),
                    "front_tire_thermal_grip": round(
                        state.front_tire_thermal_grip,
                        4,
                    ),
                    "rear_tire_surface_temperature_c": round(
                        state.rear_tire_surface_temperature_c,
                        2,
                    ),
                    "rear_tire_core_temperature_c": round(
                        state.rear_tire_core_temperature_c,
                        2,
                    ),
                    "rear_tire_thermal_grip": round(
                        state.rear_tire_thermal_grip,
                        4,
                    ),
                    "fuel_mass_kg": round(state.fuel_mass_kg, 3),
                    "fuel_burned_kg": round(state.fuel_burned_kg, 3),
                    "fuel_laps_remaining": round(
                        state.fuel_laps_remaining,
                        2,
                    ),
                    "vehicle_mass_kg": round(state.vehicle_mass_kg, 3),
                    "front_normal_load_n": round(
                        state.front_normal_load_n,
                        2,
                    ),
                    "rear_normal_load_n": round(
                        state.rear_normal_load_n,
                        2,
                    ),
                    "longitudinal_load_transfer_n": round(
                        state.longitudinal_load_transfer_n,
                        2,
                    ),
                    "front_wheel_speed_rad_s": round(
                        state.front_wheel_speed_rad_s,
                        4,
                    ),
                    "rear_wheel_speed_rad_s": round(
                        state.rear_wheel_speed_rad_s,
                        4,
                    ),
                    "front_axle_slip_ratio": round(
                        state.front_axle_slip_ratio,
                        5,
                    ),
                    "rear_axle_slip_ratio": round(
                        state.rear_axle_slip_ratio,
                        5,
                    ),
                    "applied_brake_force_n": round(
                        state.applied_brake_force_n,
                        2,
                    ),
                    "front_brake_temperature_c": round(
                        state.front_brake_temperature_c,
                        2,
                    ),
                    "rear_brake_temperature_c": round(
                        state.rear_brake_temperature_c,
                        2,
                    ),
                    "brake_fade_factor": round(
                        state.brake_fade_factor,
                        5,
                    ),
                    "grip_utilization": round(
                        state.grip_utilization,
                        4,
                    ),
                    "handling_state": state.handling_state,
                    "wheel_lock_ratio": round(
                        state.wheel_lock_ratio,
                        5,
                    ),
                    "traction_slip_ratio": round(
                        state.traction_slip_ratio,
                        5,
                    ),
                    "front_tire_slide_energy_j": round(
                        state.front_tire_slide_energy_j,
                        2,
                    ),
                    "rear_tire_slide_energy_j": round(
                        state.rear_tire_slide_energy_j,
                        2,
                    ),
                    "tire_slide_energy_j": round(
                        state.tire_slide_energy_j,
                        2,
                    ),
                    "rear_applied_drive_energy_j": round(
                        state.rear_applied_drive_energy_j,
                        2,
                    ),
                    "front_applied_brake_energy_j": round(
                        state.front_applied_brake_energy_j,
                        2,
                    ),
                    "rear_applied_brake_energy_j": round(
                        state.rear_applied_brake_energy_j,
                        2,
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
            "track_conditions": {
                "ambient_temperature_c": self.track_conditions.ambient_temperature_c,
                "track_temperature_c": self.track_conditions.track_temperature_c,
            },
            "thermal_preset": (
                self.thermal_preset.value
                if isinstance(self.thermal_preset, ThermalPresetName)
                else self.thermal_preset
            ),
            "track_conditions_source": self.track_conditions_source,
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

    def diagnostic_counts(self) -> dict[str, Any]:
        """Return bounded ownership counts, never live simulation payloads."""
        physics_instances: dict[int, LongitudinalVehiclePhysics] = {}
        for physics_by_line in self._vehicle_physics_by_driver_line.values():
            for physics in physics_by_line.values():
                physics_instances[id(physics)] = physics
        for physics in self._vehicle_physics_by_line.values():
            physics_instances[id(physics)] = physics

        sc_set_names = (
            "_sc_caught_driver_ids",
            "_sc_unlap_driver_ids",
            "_sc_order_yield_targets",
            "_sc_pit_exit_order_targets",
            "_sc_unlap_targets",
        )
        sc_set_count = 0
        for name in sc_set_names:
            value = getattr(self, name, None)
            sc_set_count += len(value) if hasattr(value, "__len__") else 0

        timing_crossing_count = sum(
            len(crossings)
            for crossings in getattr(self, "_timing_crossings", {}).values()
        )
        lap_history_count = sum(
            len(history)
            for history in getattr(self, "_lap_history", {}).values()
        )
        queue_plan = getattr(self, "_sc_queue_plan", None)
        nomination = self.circuit.tire_compound_nomination
        return {
            **track_physics_cache_counts(),
            "driver_count": len(self.driver_states),
            "predictive_speed_cache_entries": sum(
                len(getattr(physics, "_predictive_speed_cache", {}))
                for physics in physics_instances.values()
            ),
            "predictive_track_sample_cache_entries": sum(
                len(getattr(physics, "_predictive_track_sample_cache", {}))
                for physics in physics_instances.values()
            ),
            "local_trajectory_planner_entries": len(
                getattr(self, "_local_trajectory_planners", {})
            ),
            "local_trajectory_plan_entries": len(
                getattr(self, "_local_trajectory_plans", {})
            ),
            "local_trajectory_last_safe_plan_entries": len(
                getattr(self, "_local_trajectory_last_safe_plans", {})
            ),
            "attack_line_cache_entries": len(
                getattr(self, "_attack_line_choice_cache", {})
            ),
            "timing_crossing_entries": timing_crossing_count,
            "lap_history_entries": lap_history_count,
            "physics_diagnostic_samples": len(
                getattr(self, "_physics_step_deltas", ())
            ) + len(getattr(self, "_consumed_physics_frame_ids", ())),
            "safety_car_collection_entries": sc_set_count,
            "safety_car_queue_plan_slots": len(queue_plan.slots) if queue_plan else 0,
            "safety_car_queue_plan_revision": queue_plan.revision if queue_plan else 0,
            "safety_car_driver_state_entries": len(
                getattr(self, "_sc_driver_states", {})
            ),
            "safety_car_queue_stable_seconds": round(
                float(getattr(self, "_sc_queue_stable_seconds", 0.0)),
                3,
            ),
            "tire_temperature": self._tire_temperature_diagnostics,
            "thermal_preset": (
                self.thermal_preset.value
                if isinstance(self.thermal_preset, ThermalPresetName)
                else self.thermal_preset
            ),
            "track_conditions_source": self.track_conditions_source,
            "tire_compound_ruleset": nomination.ruleset if nomination else None,
            "tire_compound_nomination": (
                nomination.model_dump(mode="json") if nomination else None
            ),
            "finished": bool(self.finished),
        }
