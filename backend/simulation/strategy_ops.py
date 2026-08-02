"""Pace modes, fuel/tire wear and AI race strategy extracted from RaceEngine.

RaceEngine inherits StrategyOpsMixin so existing call sites and tests keep the
same method names.  Behavior is unchanged.

Pit compound helpers remain in ``ai_strategy.py``; this module owns runtime
pace/fuel/tire strategy on the engine.
"""

from __future__ import annotations

from models.schemas import DriverRaceState, PaceMode, TrackSegmentType
from simulation.ai_strategy import choose_pit_tire, should_pit
from simulation.brake_model import advance_brake_thermal_state
from simulation.local_trajectory_planner import (
    LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
    LocalTrajectoryPlannerWeights,
)
from simulation.physics import compute_effective_lap_time
from simulation.racecraft_ops import (
    AI_ATTACK_EXIT_GAP_SECONDS,
    AI_ATTACK_GAP_SECONDS,
    AI_DEFEND_EXIT_GAP_SECONDS,
    AI_DEFEND_GAP_SECONDS,
)
from simulation.tire_model import (
    compound_spec_for,
    TirePhysicsFactors,
    advance_tire_thermal_state,
    compute_managed_tire_age,
    compute_tire_performance,
    compute_tire_physics_factors,
    compute_wear,
    circuit_tire_usage_per_lap,
    physical_compound_for_state,
    tire_thermal_wear_multiplier,
)
from simulation.track_geometry import segment_at_progress

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
AI_PACE_MIN_COOLDOWN_SECONDS = 8.0
AI_PACE_MAX_COOLDOWN_SECONDS = 12.0
AI_PACE_INITIAL_STAGGER_MIN_SECONDS = 0.5
AI_PACE_INITIAL_STAGGER_MAX_SECONDS = 5.0
AI_LOW_TIRE_LIFE = 0.25
AI_CONSERVE_EXIT_TIRE_LIFE = 0.32
AI_CRITICAL_TIRE_LIFE = 0.12
AI_FRESH_TIRE_USAGE = 2.0


class StrategyOpsMixin:
    """Pace commands, fuel/tire state and AI strategy decisions."""

    def _init_strategy_state(self) -> None:
        """Initialize pace-mode transitions, lap variance and AI pace cooldowns."""
        self._pace_mode_replan_required: set[int] = set()
        self._pace_mode_intensity: dict[int, float] = {}
        self._pace_mode_transition_source: dict[int, float] = {}
        self._pace_mode_transition_elapsed: dict[int, float] = {}
        self._pace_mode_transition_target: dict[int, PaceMode] = {}
        self._pace_mode_transition_from: dict[int, PaceMode] = {}
        self._lap_random: dict[int, float] = {}
        self._tire_random: dict[int, float] = {}
        self._ai_pace_cooldown: dict[int, float] = {}

    def _consistency_variation_spread(self, consistency: float) -> float:
        """Return per-lap random swing from driver consistency."""
        return 0.10 + (1.0 - consistency) * 1.35

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
        return compute_wear(physical_compound_for_state(state), self._effective_tire_age(state))

    def _circuit_tire_usage_per_lap(self) -> float:
        """Return circuit distance/abrasion as Bahrain-equivalent lap usage."""
        return circuit_tire_usage_per_lap(
            self.circuit.track_length_m,
            self.circuit.tire_wear_profile.abrasion_multiplier,
        )

    def _tire_thermal_usage_multiplier(self, state: DriverRaceState) -> float:
        return tire_thermal_wear_multiplier(
            physical_compound_for_state(state),
            state.tire_surface_temperature_c,
        )

    def _current_tire_physics(self, state: DriverRaceState) -> TirePhysicsFactors:
        return compute_tire_physics_factors(
            physical_compound_for_state(state),
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
        *,
        front_brake_bias: float,
    ) -> None:
        # Kept in the helper contract because callers already provide the
        # physical brake bias. Axle brake work below is already bias-split.
        _ = front_brake_bias
        normal_load = max(
            1.0,
            result.front_normal_load_n + result.rear_normal_load_n,
        )
        front_load_share = max(
            0.35,
            min(0.65, result.front_normal_load_n / normal_load),
        )
        rear_load_share = 1.0 - front_load_share
        front_thermal = advance_tire_thermal_state(
            physical_compound_for_state(state),
            surface_temperature_c=state.front_tire_surface_temperature_c,
            core_temperature_c=state.front_tire_core_temperature_c,
            delta_seconds=delta_seconds,
            speed_mps=result.speed_mps,
            lateral_acceleration_mps2=result.lateral_acceleration_mps2,
            throttle=result.throttle,
            brake=result.brake,
            slide_energy_j=result.front_tire_slide_energy_j,
            ambient_temperature_c=self.track_conditions.ambient_temperature_c,
            track_temperature_c=self.track_conditions.track_temperature_c,
            baseline_heat_share=0.5,
            lateral_heat_share=front_load_share,
            # The physical result already contains the front axle's applied
            # brake work. Do not apply the bias a second time in the thermal
            # conversion.
            brake_heat_share=1.0,
            traction_heat_share=0.0,
            slide_heat_share=1.0,
            thermal_mass_share=0.5,
            applied_drive_energy_j=0.0,
            applied_brake_work_energy_j=result.front_applied_brake_energy_j,
        )
        rear_thermal = advance_tire_thermal_state(
            physical_compound_for_state(state),
            surface_temperature_c=state.rear_tire_surface_temperature_c,
            core_temperature_c=state.rear_tire_core_temperature_c,
            delta_seconds=delta_seconds,
            speed_mps=result.speed_mps,
            lateral_acceleration_mps2=result.lateral_acceleration_mps2,
            throttle=result.throttle,
            brake=result.brake,
            slide_energy_j=result.rear_tire_slide_energy_j,
            ambient_temperature_c=self.track_conditions.ambient_temperature_c,
            track_temperature_c=self.track_conditions.track_temperature_c,
            baseline_heat_share=0.5,
            lateral_heat_share=rear_load_share,
            # The physical result already contains the rear axle's applied
            # brake work. Do not apply the bias a second time in the thermal
            # conversion.
            brake_heat_share=1.0,
            traction_heat_share=1.0,
            slide_heat_share=1.0,
            thermal_mass_share=0.5,
            applied_drive_energy_j=result.rear_applied_drive_energy_j,
            applied_brake_work_energy_j=result.rear_applied_brake_energy_j,
        )
        self._last_tire_thermal_states[state.driver_id] = (
            front_thermal,
            rear_thermal,
        )
        self._accumulate_tire_thermal_diagnostics(
            state,
            front_thermal,
            rear_thermal,
            delta_seconds,
        )
        state.front_tire_surface_temperature_c = round(
            front_thermal.surface_temperature_c,
            4,
        )
        state.front_tire_core_temperature_c = round(
            front_thermal.core_temperature_c,
            4,
        )
        state.front_tire_thermal_grip = round(
            front_thermal.thermal_grip,
            5,
        )
        state.rear_tire_surface_temperature_c = round(
            rear_thermal.surface_temperature_c,
            4,
        )
        state.rear_tire_core_temperature_c = round(
            rear_thermal.core_temperature_c,
            4,
        )
        state.rear_tire_thermal_grip = round(
            rear_thermal.thermal_grip,
            5,
        )
        # Keep the original aggregate fields as a compatibility surface for
        # physics consumers and older clients.
        state.tire_surface_temperature_c = round(
            (
                front_thermal.surface_temperature_c
                + rear_thermal.surface_temperature_c
            )
            * 0.5,
            4,
        )
        state.tire_core_temperature_c = round(
            (
                front_thermal.core_temperature_c
                + rear_thermal.core_temperature_c
            )
            * 0.5,
            4,
        )
        state.tire_thermal_grip = round(
            (front_thermal.thermal_grip + rear_thermal.thermal_grip) * 0.5,
            5,
        )

    def _update_brake_thermal_state(
        self,
        state: DriverRaceState,
        result,
        delta_seconds: float,
        *,
        front_brake_bias: float,
    ) -> None:
        thermal = advance_brake_thermal_state(
            front_temperature_c=state.front_brake_temperature_c,
            rear_temperature_c=state.rear_brake_temperature_c,
            delta_seconds=delta_seconds,
            speed_mps=result.speed_mps,
            applied_brake_force_n=result.applied_brake_force_n,
            front_brake_bias=front_brake_bias,
            ambient_temperature_c=self.track_conditions.ambient_temperature_c,
        )
        state.front_brake_temperature_c = round(
            thermal.front_temperature_c,
            4,
        )
        state.rear_brake_temperature_c = round(
            thermal.rear_temperature_c,
            4,
        )
        state.brake_fade_factor = round(thermal.fade_factor, 5)

    def _base_lap_time_for_state(self, state: DriverRaceState) -> float:
        """Estimate lap time before traffic effects."""
        meta = self._driver_meta[state.driver_id]
        tire_perf = compute_tire_performance(
            physical_compound_for_state(state),
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
        compound_spec = compound_spec_for(physical_compound_for_state(state))
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

    def _run_ai_strategy(self) -> None:
        for driver_id, state in self.driver_states.items():
            if state.retired or state.finished:
                continue
            if driver_id in self.player_driver_ids:
                continue
            remaining = self.total_laps - state.current_lap
            tire_management = self._driver_meta[driver_id]["tire_management"]
            if should_pit(
                state,
                remaining,
                is_player=False,
                tire_management=tire_management,
                next_lap_usage=self._circuit_tire_usage_per_lap(),
            ):
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
