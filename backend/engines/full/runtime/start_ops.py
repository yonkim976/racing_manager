"""FULL formation grid, start lights and launch-lane merge operations.

RaceEngine inherits StartOpsMixin so existing call sites and tests keep the
same method names.  Behavior is unchanged.
"""

from __future__ import annotations

from models.schemas import (
    Driver,
    DriverRaceState,
    DryTireRole,
    PaceMode,
    RaceEvent,
    TireCompound,
    TrackSegmentType,
)
from simulation.car_performance import car_performance_factors
from simulation.physics import driver_pace_multiplier
from .racecraft_ops import MANEUVER_CLEARANCE_MARGIN_M
from .runtime_constants import GRID_LAUNCH_MERGE_DISTANCE_M
from .strategy_ops import (
    AI_PACE_INITIAL_STAGGER_MAX_SECONDS,
    AI_PACE_INITIAL_STAGGER_MIN_SECONDS,
    FUEL_LOAD_RESERVE_FACTOR,
    MAX_INITIAL_FUEL_MASS_KG,
    PACE_MODE_TRANSITION_SECONDS,
)
from simulation.tire_model import tire_blanket_temperature_c
from simulation.start_grid_geometry import (
    GRID_COLUMN_OFFSET_M,
    GRID_POLE_DISTANCE_BEHIND_LINE_M,
    GRID_SLOT_PROGRESS_GAP,
    GRID_SLOT_SPACING_M,
    build_grid_slots,
)
from simulation.track_physics import PHYSICAL_CAR_LENGTH_M, PHYSICAL_CAR_WIDTH_M
from simulation.vehicle_dimensions import PHYSICAL_CAR_WHEELBASE_M

GRID_FIRST_LIGHT_SECONDS = 0.25
GRID_LIGHT_INTERVAL_SECONDS = 0.62
GRID_LIGHTS_OUT_SECONDS = GRID_FIRST_LIGHT_SECONDS + GRID_LIGHT_INTERVAL_SECONDS * 5
GRID_LIGHTS_OUT_DISPLAY_SECONDS = 0.70
GRID_LAUNCH_LANE_HOLD_M = 35.0
GRID_OVERTAKE_LOCKOUT_SECONDS = 0.50

class StartOpsMixin:
    """Grid slots, start-light sequence and post-lights launch lanes."""

    def _init_start_ops_state(self, start_sequence_enabled: bool) -> None:
        """Initialize start lights, lockout and staggered-grid anchors."""
        self.start_sequence_enabled = start_sequence_enabled
        self.race_started = not start_sequence_enabled
        self.start_sequence_phase = "grid" if start_sequence_enabled else "racing"
        self.start_light_count = 0
        self._start_sequence_elapsed = 0.0
        self._lights_out_display_remaining = 0.0
        self._start_overtake_lockout_remaining = 0.0
        self._grid_start_progress: dict[int, float] = {}
        self._grid_lateral_offsets: dict[int, float] = {}

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
            physical_compound = self.physical_compound_for_role(starting_compound)
            try:
                tire_role = DryTireRole(starting_compound.value)
            except ValueError:
                tire_role = DryTireRole.MEDIUM
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
                    physical_compound,
                )
            self._grid_start_progress[driver.id] = grid_progress
            self._grid_lateral_offsets[driver.id] = grid_lateral_offset_m
            initial_fuel_mass_kg = min(
                MAX_INITIAL_FUEL_MASS_KG,
                self._expected_fuel_per_lap_kg()
                * self.total_laps
                * FUEL_LOAD_RESERVE_FACTOR,
            )
            blanket_temperature_c = tire_blanket_temperature_c(physical_compound)
            self.driver_states[driver.id] = DriverRaceState(
                driver_id=driver.id,
                position=position,
                progress=grid_progress,
                current_lap=0,
                total_progress=grid_progress,
                tire_compound=starting_compound,
                tire_role=tire_role,
                physical_tire_compound=physical_compound,
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
                front_tire_surface_temperature_c=blanket_temperature_c,
                front_tire_core_temperature_c=blanket_temperature_c,
                rear_tire_surface_temperature_c=blanket_temperature_c,
                rear_tire_core_temperature_c=blanket_temperature_c,
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

    def _grid_launch_distance_m(self, state: DriverRaceState) -> float:
        start_progress = self._grid_start_progress.get(state.driver_id)
        if start_progress is None:
            return float("inf")
        return max(0.0, (state.total_progress - start_progress) * self.track_length_m)

    def _grid_launch_lane_hold_distance_m(self) -> float:
        first_corner = next(
            (
                segment
                for segment in sorted(
                    self.circuit.segments,
                    key=lambda item: item.start,
                )
                if segment.type != TrackSegmentType.STRAIGHT
                and segment.end > 0.0
            ),
            None,
        )
        if first_corner is None:
            return GRID_LAUNCH_LANE_HOLD_M
        return max(
            GRID_LAUNCH_LANE_HOLD_M,
            min(700.0, first_corner.end * self.track_length_m + 30.0),
        )

    def _grid_launch_merge_distance_m(self) -> float:
        return max(
            GRID_LAUNCH_MERGE_DISTANCE_M,
            self._grid_launch_lane_hold_distance_m() + 120.0,
        )

    def _grid_launch_target_lateral_offset(
        self,
        state: DriverRaceState,
        racing_line_offset_m: float,
    ) -> float | None:
        if not self.start_sequence_enabled or not self.race_started:
            return None
        launch_distance_m = self._grid_launch_distance_m(state)
        merge_distance_m = self._grid_launch_merge_distance_m()
        hold_distance_m = self._grid_launch_lane_hold_distance_m()
        if launch_distance_m >= merge_distance_m:
            return None
        grid_offset_m = self._grid_lateral_offsets.get(state.driver_id)
        if grid_offset_m is None:
            return None
        if launch_distance_m <= hold_distance_m:
            return grid_offset_m
        merge_span_m = max(
            1.0,
            merge_distance_m - hold_distance_m,
        )
        ratio = min(
            1.0,
            max(0.0, (launch_distance_m - hold_distance_m) / merge_span_m),
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
            self._grid_launch_distance_m(first) >= self._grid_launch_merge_distance_m()
            or self._grid_launch_distance_m(second) >= self._grid_launch_merge_distance_m()
        ):
            return False
        return abs(first.lateral_offset_m - second.lateral_offset_m) >= (
            0.5 * (first.car_width_m + second.car_width_m)
            + MANEUVER_CLEARANCE_MARGIN_M
        )

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
        slots = build_grid_slots(
            (
                state.driver_id
                for state in sorted(self.driver_states.values(), key=lambda item: item.position)
            ),
            self.track_length_m,
            start_sequence_enabled=self.start_sequence_enabled,
            track_profile=self._track_physics,
        )
        return [slot.to_dict() for slot in slots]

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
