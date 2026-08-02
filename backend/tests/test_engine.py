"""Unit tests for simulation engine logic."""

from __future__ import annotations

import asyncio
import unittest
from math import atan2, cos, degrees, hypot, pi, sin, sqrt
from types import SimpleNamespace

from data_loader import load_circuits, load_drivers, load_teams
from pydantic import ValidationError

from models.schemas import (
    Circuit,
    CircuitEditorState,
    CircuitThermalProfile,
    EditorPoint,
    PaceMode,
    PitLaneConfig,
    RaceSetupRequest,
    TireCompound,
    ThermalPresetName,
    TrackConditions,
    TrackLayoutSegment,
    TrackLayoutSegmentType,
    TrackSegmentType,
)
from session import SessionManager
from simulation.ai_strategy import choose_tire_for_remaining_laps, should_pit
from simulation.car_performance import car_performance_factors
from simulation.collision import oriented_body_overlap
from simulation.incidents import (
    Incident,
    IncidentCause,
    IncidentSeverity,
    escalate_collision,
)
from simulation.physics import GAME_TICK_SECONDS, driver_pace_multiplier
from simulation.qualifying import run_qualifying
from simulation.race_engine import (
    AI_PACE_INITIAL_STAGGER_MAX_SECONDS,
    AI_PACE_INITIAL_STAGGER_MIN_SECONDS,
    ATTACK_LINE_INSIDE,
    ATTACK_LINE_OUTSIDE,
    BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA,
    DEFENDER_LINE_DEFENSIVE,
    DEFENDER_LINE_RACING,
    GRID_COLUMN_OFFSET_M,
    GRID_LIGHTS_OUT_SECONDS,
    GRID_SLOT_SPACING_M,
    FOLLOWING_MIN_BUMPER_GAP_M,
    LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
    ManeuverGroup,
    PIT_LANE_SPEED_LIMIT_KPH,
    SC_CAR_LENGTH_M,
    SC_CATCH_UP_FAST_LAP_TIME_FACTOR,
    SC_CATCH_UP_MAX_SPEED_KPH,
    SC_CATCH_UP_NEAR_LAP_TIME_FACTOR,
    SC_CAUGHT_MAX_SPEED_KPH,
    SC_CLEANUP_SECONDS,
    SC_MAX_GAP_CAR_LENGTHS,
    SC_NEAR_QUEUE_MAX_SPEED_KPH,
    SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS,
    SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH,
    SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH,
    SC_QUEUE_STABLE_RELATIVE_SPEED_GRACE_SECONDS,
    SC_QUEUE_STABLE_RELATIVE_SPEED_KPH,
    SC_QUEUE_TARGET_CAR_LENGTHS,
    TIMING_CROSSING_LAPS_TO_RETAIN,
    VSC_DURATION_SECONDS,
    RaceEngine,
)
from simulation.track_geometry import (
    segment_at_progress,
    segment_lengths,
    self_intersections,
    track_length,
    validate_circuit_geometry,
)
from simulation.track_physics import (
    DRIVING_LINE_RACING,
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
)


def _test_thermal_profile() -> CircuitThermalProfile:
    return CircuitThermalProfile(
        presets={
            ThermalPresetName.COOL: TrackConditions(
                ambient_temperature_c=24.0,
                track_temperature_c=32.0,
            ),
            ThermalPresetName.NORMAL: TrackConditions(
                ambient_temperature_c=30.0,
                track_temperature_c=40.0,
            ),
            ThermalPresetName.HOT: TrackConditions(
                ambient_temperature_c=36.0,
                track_temperature_c=52.0,
            ),
        }
    )
from simulation.track_compiler import _point_and_tangent_at_progress, compile_circuit_layout, compile_layout_segments
from simulation.tire_model import (
    compute_managed_tire_age,
    compute_tire_performance,
    compute_tire_physics_factors,
    compute_wear,
    tire_management_age_multiplier,
)
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS
from simulation.state_contract import TickPhase


def _make_engine(seed: int = 42) -> RaceEngine:
    drivers = load_drivers()
    teams = {t.id: t for t in load_teams()}
    circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
    return RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=1,
        player_driver_ids=[1, 2],
        seed=seed,
        start_sequence_enabled=False,
    )


def _prepare_two_car_sc_queue(engine: RaceEngine):
    """Create a two-car SC queue with both cars at the target gap."""
    engine._trigger_safety_car([])
    running = sorted(engine.driver_states.values(), key=lambda state: state.position)
    leader, follower = running[:2]
    for state in running[2:]:
        state.retired = True

    engine.safety_car_stage = "collecting"
    target_gap_m = SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
    engine._set_state_total_progress(leader, 1.2)
    engine._set_state_total_progress(
        follower,
        leader.total_progress - target_gap_m / engine.track_length_m,
    )
    leader.speed_kph = 80.0
    follower.speed_kph = 80.0
    engine._safety_car_speed_mps = 80.0 / 3.6
    engine._safety_car_total_progress = (
        leader.total_progress + target_gap_m / engine.track_length_m
    )

    # The first sync initializes the filtered signal; the second one starts
    # accumulating the stable interval with a non-zero elapsed time.
    engine._sync_safety_car_queue([])
    engine.race_elapsed += 0.1
    engine._sync_safety_car_queue([])
    return leader, follower


def _real_circuit_ids() -> tuple[int, ...]:
    """All seed circuits backed by a real-world geo source."""
    return tuple(c.id for c in load_circuits() if c.geo is not None)


def _make_engine_for_circuit(circuit_id: int, seed: int = 42) -> RaceEngine:
    drivers = load_drivers()
    teams = {t.id: t for t in load_teams()}
    circuit = next(c for c in load_circuits() if c.id == circuit_id)
    return RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=1,
        player_driver_ids=[1, 2],
        seed=seed,
        start_sequence_enabled=False,
    )


def _make_starting_engine(circuit_id: int = 4, seed: int = 42) -> RaceEngine:
    drivers = load_drivers()
    teams = {team.id: team for team in load_teams()}
    circuit = next(item for item in load_circuits() if item.id == circuit_id)
    return RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=1,
        player_driver_ids=[1, 2],
        seed=seed,
        start_sequence_enabled=True,
    )


def _place_close_pair(
    engine: RaceEngine,
    car_ahead,
    attacker,
    ahead_total_progress: float,
    bumper_gap_m: float = 1.5,
) -> None:
    engine._set_state_total_progress(car_ahead, ahead_total_progress)
    engine._set_state_total_progress(
        attacker,
        ahead_total_progress
        - (PHYSICAL_CAR_LENGTH_M + bumper_gap_m) / engine.track_length_m,
    )


def _set_test_tire(engine: RaceEngine, state, role: TireCompound) -> None:
    """Use the same atomic role/physical/legacy transition as runtime pit ops."""
    engine.set_driver_tire_compound(state, role)


def _prepare_local_pull_out_plan(engine: RaceEngine, attacker) -> None:
    engine._update_local_trajectory_plan(
        attacker,
        DRIVING_LINE_RACING,
        engine._physics_v2_modifiers(attacker),
        0.20,
    )


def _distance_to_polyline(point: list[float], coords: list[list[float]]) -> float:
    best_distance = float("inf")
    for start, end in zip(coords, coords[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 0:
            continue
        t = min(
            1.0,
            max(
                0.0,
                ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared,
            ),
        )
        projected_x = start[0] + dx * t
        projected_y = start[1] + dy * t
        best_distance = min(
            best_distance,
            hypot(point[0] - projected_x, point[1] - projected_y),
        )
    return best_distance


def _make_engine_with_starting_tires(starting_tires: dict[int, TireCompound]) -> RaceEngine:
    drivers = load_drivers()
    teams = {t.id: t for t in load_teams()}
    circuit = load_circuits()[0]
    return RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=1,
        player_driver_ids=[1, 2],
        starting_tires=starting_tires,
        seed=42,
        start_sequence_enabled=False,
    )


def _physics_target_speed_at(
    engine: RaceEngine,
    state,
    progress: float,
) -> float:
    state.progress = progress
    track_profile = engine._track_physics_for_driver(state)
    physics_by_line = engine._vehicle_physics_for_driver(state)
    distance_m = track_profile.line_distance_at_total_progress(
        DRIVING_LINE_RACING,
        progress,
    )
    return physics_by_line[DRIVING_LINE_RACING].target_speed_mps(
        distance_m,
        engine._physics_v2_modifiers(state),
    )


class RaceSetupTests(unittest.TestCase):
    def test_setup_request_accepts_lap_count_between_5_and_100(self) -> None:
        request = RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=5)
        self.assertEqual(request.total_laps, 5)

        request = RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=100)
        self.assertEqual(request.total_laps, 100)

    def test_setup_request_rejects_lap_count_outside_range(self) -> None:
        with self.assertRaises(ValidationError):
            RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=4)

        with self.assertRaises(ValidationError):
            RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=101)

    def test_session_uses_requested_lap_count_without_mutating_circuit_data(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()

        session = manager.create_session(
            RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=12),
            drivers,
            teams,
            circuits,
        )

        self.assertEqual(session.circuit.total_laps, 12)
        self.assertEqual(session.engine.total_laps, 12)
        self.assertEqual(circuits[0].total_laps, 57)

    def test_session_uses_supplied_qualifying_grid_order(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()
        grid_order = [drivers[-1].id, drivers[0].id, drivers[1].id]

        session = manager.create_session(
            RaceSetupRequest(
                circuit_id=3,
                player_team_id=1,
                total_laps=12,
                grid_order=grid_order,
            ),
            drivers,
            teams,
            circuits,
        )

        self.assertEqual(session.engine.driver_states[grid_order[0]].position, 1)
        self.assertEqual(session.engine.driver_states[grid_order[1]].position, 2)
        self.assertEqual(session.engine.driver_states[grid_order[2]].position, 3)

    def test_can_create_session_with_bahrain_circuit(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()

        session = manager.create_session(
            RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=57),
            drivers,
            teams,
            circuits,
        )

        self.assertEqual(session.circuit.id, 3)
        self.assertEqual(session.circuit.name, "Bahrain International Circuit")
        self.assertEqual(session.engine.total_laps, 57)
        self.assertGreater(len(session.circuit.track_coords), 30)
        self.assertEqual(len(session.circuit.drs_zones), 4)
        self.assertGreaterEqual(len(session.circuit.segments), 9)

    def test_can_create_session_with_red_bull_ring_circuit(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()

        session = manager.create_session(
            RaceSetupRequest(circuit_id=4, player_team_id=1, total_laps=71),
            drivers,
            teams,
            circuits,
        )

        self.assertEqual(session.circuit.id, 4)
        self.assertEqual(session.circuit.name, "Red Bull Ring")
        self.assertEqual(next(c.total_laps for c in circuits if c.id == 4), 71)
        self.assertEqual(session.engine.total_laps, 71)
        self.assertGreater(len(session.circuit.track_coords), 40)
        self.assertEqual(len(session.circuit.drs_zones), 3)
        self.assertGreaterEqual(len(session.circuit.segments), 10)

    def test_session_pause_and_resume_commands_keep_connection_state_valid(self) -> None:
        manager = SessionManager()
        session = manager.create_session(
            RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=12),
            load_drivers(),
            load_teams(),
            load_circuits(),
        )

        pause_response = asyncio.run(session.handle_command({"type": "pause"}))
        self.assertEqual(pause_response["type"], "command_ack")
        self.assertTrue(session.engine.paused)
        self.assertTrue(callable(session.engine.pause_race))
        self.assertEqual(session.engine.build_tick_state().physics_hz, 50)
        self.assertEqual(session.race_info.car_width_m, 1.9)
        self.assertEqual(session.race_info.car_length_m, 5.0)
        self.assertEqual(
            session.race_info.pit_lane_width_m,
            session.circuit.pit_lane.lane_width_m,
        )
        self.assertEqual(
            session.race_info.pit_side_entry_progress,
            session.circuit.pit_lane.side_entry_progress,
        )
        self.assertEqual(
            session.race_info.pit_speed_limit_start,
            session.circuit.pit_lane.speed_limit_start,
        )
        self.assertEqual(
            session.race_info.pit_box_progress,
            session.circuit.pit_lane.box_progress,
        )
        self.assertEqual(
            session.race_info.pit_speed_limit_end,
            session.circuit.pit_lane.speed_limit_end,
        )
        self.assertEqual(
            session.race_info.pit_side_rejoin_progress,
            session.circuit.pit_lane.side_rejoin_progress,
        )
        self.assertLess(
            session.race_info.pit_side_entry_progress,
            session.race_info.pit_speed_limit_start,
        )
        self.assertLess(
            session.race_info.pit_speed_limit_start,
            session.race_info.pit_box_progress,
        )
        self.assertLess(
            session.race_info.pit_box_progress,
            session.race_info.pit_speed_limit_end,
        )
        self.assertLess(
            session.race_info.pit_speed_limit_end,
            session.race_info.pit_side_rejoin_progress,
        )
        self.assertGreater(len(session.race_info.racing_line_profile), 10)
        self.assertGreater(len(session.race_info.racing_line_coords), 10)
        self.assertGreater(session.race_info.racing_line_length_m, 0.0)
        self.assertGreater(session.race_info.predicted_racing_lap_time, 0.0)
        self.assertEqual(
            set(session.race_info.driving_line_coords),
            {"racing_line", "inside", "outside", "defensive_line"},
        )
        self.assertEqual(
            set(session.race_info.driving_line_lengths_m),
            {"racing_line", "inside", "outside", "defensive_line"},
        )
        self.assertEqual(
            set(session.race_info.predicted_line_lap_times),
            {"racing_line", "inside", "outside", "defensive_line"},
        )

        resume_response = asyncio.run(session.handle_command({"type": "resume"}))
        self.assertEqual(resume_response["type"], "command_ack")
        self.assertFalse(session.engine.paused)

    def test_race_info_pit_route_uses_authoritative_track_anchors(self) -> None:
        manager = SessionManager()
        session = manager.create_session(
            RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=5),
            load_drivers(),
            load_teams(),
            load_circuits(),
        )
        pit_lane = session.circuit.pit_lane
        assert pit_lane is not None
        entry_pose = session.engine._track_physics.line_pose_at_progress(
            DRIVING_LINE_RACING,
            pit_lane.entry_progress,
        )
        exit_pose = session.engine._track_physics.line_pose_at_progress(
            DRIVING_LINE_RACING,
            pit_lane.exit_progress,
        )

        self.assertAlmostEqual(session.race_info.pit_lane_coords[0][0], entry_pose[0])
        self.assertAlmostEqual(session.race_info.pit_lane_coords[0][1], entry_pose[1])
        self.assertAlmostEqual(session.race_info.pit_lane_coords[-1][0], exit_pose[0])
        self.assertAlmostEqual(session.race_info.pit_lane_coords[-1][1], exit_pose[1])
        self.assertGreater(len(session.race_info.pit_exit_lane_coords), 2)
        self.assertEqual(
            session.race_info.pit_exit_lane_coords,
            session.engine.get_pit_exit_lane_coords(),
        )

    def test_pit_operational_landmarks_must_follow_route_order(self) -> None:
        with self.assertRaises(ValidationError):
            PitLaneConfig(
                side_entry_progress=0.13,
                speed_limit_start=0.12,
            )

        with self.assertRaises(ValidationError):
            PitLaneConfig(
                speed_limit_start=0.55,
                box_progress=0.50,
                speed_limit_end=0.88,
            )

        with self.assertRaises(ValidationError):
            PitLaneConfig(
                speed_limit_start=0.12,
                box_progress=0.90,
                speed_limit_end=0.88,
            )

        with self.assertRaises(ValidationError):
            PitLaneConfig(
                speed_limit_start=0.12,
                box_progress=0.50,
                speed_limit_end=0.95,
                side_rejoin_progress=0.94,
            )

    def test_session_pace_command_updates_next_tick_state(self) -> None:
        manager = SessionManager()
        session = manager.create_session(
            RaceSetupRequest(circuit_id=4, player_team_id=1, total_laps=12),
            load_drivers(),
            load_teams(),
            load_circuits(),
        )

        response = asyncio.run(
            session.handle_command(
                {
                    "type": "set_pace_mode",
                    "driver_id": 1,
                    "pace_mode": "ATTACK",
                }
            )
        )
        tick_state = session.engine.build_tick_state()
        driver_state = next(
            driver for driver in tick_state.positions if driver.driver_id == 1
        )

        self.assertEqual(response["type"], "command_ack")
        self.assertEqual(response["pace_mode"], PaceMode.ATTACK.value)
        self.assertEqual(driver_state.pace_mode, PaceMode.ATTACK)

    def test_session_reconnect_snapshot_preserves_pace_transition(self) -> None:
        manager = SessionManager()
        session = manager.create_session(
            RaceSetupRequest(circuit_id=4, player_team_id=1, total_laps=12),
            load_drivers(),
            load_teams(),
            load_circuits(),
        )
        asyncio.run(
            session.handle_command(
                {
                    "type": "set_pace_mode",
                    "driver_id": 1,
                    "pace_mode": "ATTACK",
                }
            )
        )
        session.engine._tick_pace_mode_transitions(0.5)

        before_reconnect = next(
            position
            for position in session.engine.build_tick_state().positions
            if position.driver_id == 1
        )
        reconnect_snapshot = next(
            position
            for position in session.engine.build_tick_state().positions
            if position.driver_id == 1
        )

        self.assertEqual(before_reconnect, reconnect_snapshot)
        self.assertEqual(reconnect_snapshot.pace_mode, PaceMode.ATTACK.value)
        self.assertEqual(
            reconnect_snapshot.pace_mode_from,
            PaceMode.STANDARD.value,
        )
        self.assertAlmostEqual(
            reconnect_snapshot.pace_mode_transition_progress,
            1.0 / 3.0,
            places=4,
        )
        self.assertGreater(reconnect_snapshot.pace_mode_effective_intensity, 0.0)
        self.assertLess(reconnect_snapshot.pace_mode_effective_intensity, 1.0)

    def test_circuit_track_coords_do_not_self_intersect(self) -> None:
        for circuit in load_circuits():
            intersections = self_intersections(circuit.track_coords)
            if circuit.allows_self_intersection:
                self.assertEqual(len(intersections), 1, circuit.name)
            else:
                self.assertEqual(intersections, [], circuit.name)

    def test_circuit_geometry_validation_passes_for_seed_data(self) -> None:
        for circuit in load_circuits():
            self.assertEqual(validate_circuit_geometry(circuit), [], circuit.name)

    def test_drs_zones_are_inside_straight_segments(self) -> None:
        for circuit in load_circuits():
            for zone in circuit.drs_zones:
                segment = segment_at_progress(circuit, zone.start + 0.001)
                self.assertIsNotNone(segment, f"{circuit.name} {zone.name}")
                self.assertEqual(segment.type, TrackSegmentType.STRAIGHT, f"{circuit.name} {zone.name}")

                segment = segment_at_progress(circuit, zone.end - 0.001)
                self.assertIsNotNone(segment, f"{circuit.name} {zone.name}")
                self.assertEqual(segment.type, TrackSegmentType.STRAIGHT, f"{circuit.name} {zone.name}")

    def test_circuit_segments_are_available_for_driving_model(self) -> None:
        for circuit in load_circuits():
            self.assertGreater(track_length(circuit.track_coords), 1000.0, circuit.name)
            self.assertIsNotNone(segment_at_progress(circuit, 0.0), circuit.name)
            self.assertIsNotNone(segment_at_progress(circuit, 0.5), circuit.name)
            self.assertIsNotNone(segment_at_progress(circuit, 0.999), circuit.name)

    def test_seed_circuits_have_real_track_lengths_for_speed_model(self) -> None:
        expected_lengths = {
            3: 5412.0,
            4: 4318.0,
        }
        for circuit in load_circuits():
            self.assertGreater(circuit.track_length_m, 4000.0, circuit.name)
            if circuit.id in expected_lengths:
                self.assertAlmostEqual(circuit.track_length_m, expected_lengths[circuit.id])

    def test_layout_segments_compile_to_closed_smooth_track(self) -> None:
        segments = [
            TrackLayoutSegment(
                type=TrackLayoutSegmentType.BEZIER,
                start=[0, 0],
                cp1=[100, 0],
                cp2=[100, 100],
                end=[0, 100],
                samples=24,
            ),
            TrackLayoutSegment(
                type=TrackLayoutSegmentType.STRAIGHT,
                start=[0, 100],
                end=[0, 0],
            ),
        ]

        coords = compile_layout_segments(segments, spacing=8.0)

        self.assertGreater(len(coords), 30)
        self.assertEqual(coords[0], coords[-1])
        self.assertGreater(max(point[0] for point in coords), 70.0)

    def test_editor_centerline_compiles_to_legacy_track_coords(self) -> None:
        circuit = Circuit(
            id="custom_test",
            name="Custom Test",
            country="Virtual",
            base_lap_time=70.0,
            total_laps=20,
            pit_loss_time=18.0,
            track_length_m=4300.0,
            thermal_profile=_test_thermal_profile(),
            overtaking_difficulty=0.4,
            pit_lane=PitLaneConfig(),
            editor=CircuitEditorState(
                centerlineControlPoints=[
                    EditorPoint(x=100, y=100),
                    EditorPoint(x=300, y=90),
                    EditorPoint(x=420, y=260),
                    EditorPoint(x=290, y=420),
                    EditorPoint(x=110, y=360),
                ],
                pitLaneControlPoints=[
                    EditorPoint(x=110, y=120),
                    EditorPoint(x=170, y=170),
                    EditorPoint(x=250, y=150),
                ],
                trackWidth=20,
                sampleSpacing=16,
            ),
            drs_zones=[
                {"name": "DRS", "start": 0.02, "end": 0.08},
            ],
            landmarks=[
                {"type": "corner", "label": "T1", "progress": 0.1},
            ],
            segments=[
                {"name": "Main", "type": "straight", "start": 0.0, "end": 1.0},
            ],
        )

        compiled = compile_circuit_layout(circuit)

        self.assertGreaterEqual(len(compiled.track_coords), 50)
        self.assertIsInstance(compiled.track_coords[0], list)
        self.assertEqual(len(compiled.track_coords[0]), 2)
        self.assertGreaterEqual(len(compiled.pit_lane_coords), 2)
        self.assertIsNotNone(compiled.pit_lane.entry_progress)
        self.assertIsNotNone(compiled.pit_lane.exit_progress)
        self.assertGreater(len(compiled.track_points), 0)
        self.assertIsNotNone(compiled.track_boundaries)
        self.assertGreater(compiled.landmarks[0].track_index, 0)

    def test_geo_centerline_compiles_to_track_and_pit_lane(self) -> None:
        circuit = Circuit(
            id="geo_test",
            name="Geo Test",
            country="Virtual",
            base_lap_time=70.0,
            total_laps=20,
            pit_loss_time=18.0,
            track_length_m=3000.0,
            thermal_profile=_test_thermal_profile(),
            pit_lane=PitLaneConfig(),
            geo={
                "source": "test",
                "centerlineLonLat": [
                    {"lat": 52.000, "lon": -1.000},
                    {"lat": 52.000, "lon": -0.990},
                    {"lat": 51.995, "lon": -0.990},
                    {"lat": 51.995, "lon": -1.000},
                ],
                "pitLaneLonLat": [
                    {"lat": 51.9998, "lon": -0.9990},
                    {"lat": 51.9993, "lon": -0.9960},
                    {"lat": 51.9998, "lon": -0.9930},
                ],
                "sampleSpacing": 14,
                "pitSampleSpacing": 10,
                "fit": {"x": 0, "y": 0, "width": 500, "height": 380, "padding": 24},
            },
        )

        compiled = compile_circuit_layout(circuit)

        self.assertGreater(len(compiled.track_coords), 50)
        self.assertEqual(compiled.track_coords[0], compiled.track_coords[-1])
        self.assertGreater(len(compiled.pit_lane_coords), 6)
        self.assertIsNotNone(compiled.pit_lane.entry_progress)
        self.assertIsNotNone(compiled.pit_lane.exit_progress)
        self.assertIsNotNone(compiled.geo.source_length_m)
        self.assertAlmostEqual(compiled.geo.computed_length_m, 3000.0, delta=0.5)
        self.assertIsNotNone(compiled.track_boundaries)

    def test_metric_centerline_compiles_with_variable_width_boundaries(self) -> None:
        circuit = Circuit(
            id="metric_test",
            name="Metric Test",
            country="Virtual",
            base_lap_time=70.0,
            total_laps=20,
            pit_loss_time=18.0,
            track_length_m=3000.0,
            thermal_profile=_test_thermal_profile(),
            pit_lane=PitLaneConfig(),
            metric={
                "source": "test",
                "centerline": [
                    {"x_m": 0, "y_m": 0, "w_tr_right_m": 5, "w_tr_left_m": 7},
                    {"x_m": 200, "y_m": 0, "w_tr_right_m": 6, "w_tr_left_m": 8},
                    {"x_m": 200, "y_m": 100, "w_tr_right_m": 7, "w_tr_left_m": 9},
                    {"x_m": 0, "y_m": 100, "w_tr_right_m": 8, "w_tr_left_m": 10},
                ],
                "pitLane": [
                    {"x_m": 20, "y_m": 8},
                    {"x_m": 90, "y_m": 18},
                    {"x_m": 175, "y_m": 8},
                ],
                "sampleSpacing": 14,
                "pitSampleSpacing": 10,
                "fit": {"x": 0, "y": 0, "width": 500, "height": 380, "padding": 24},
            },
        )

        compiled = compile_circuit_layout(circuit)

        self.assertGreater(len(compiled.track_coords), 50)
        self.assertEqual(compiled.track_coords[0], compiled.track_coords[-1])
        self.assertGreater(len(compiled.pit_lane_coords), 6)
        self.assertIsNotNone(compiled.pit_lane.entry_progress)
        self.assertIsNotNone(compiled.pit_lane.exit_progress)
        self.assertIsNotNone(compiled.metric.source_length_m)
        self.assertAlmostEqual(compiled.metric.computed_length_m, 3000.0, delta=0.5)
        self.assertIsNotNone(compiled.track_boundaries)
        self.assertEqual(len(compiled.track_boundaries.left), len(compiled.track_coords))

    def test_seed_circuits_are_compiled_from_layout_segments(self) -> None:
        for circuit in load_circuits():
            if circuit.geo or circuit.metric:
                self.assertGreater(len(circuit.track_coords), 80, circuit.name)
                source_state = circuit.metric or circuit.geo
                self.assertIsNotNone(source_state.source_length_m, circuit.name)
                self.assertIsNotNone(source_state.computed_length_m, circuit.name)
            else:
                self.assertGreater(len(circuit.layout_segments), 0, circuit.name)
                self.assertGreater(len(circuit.track_coords), len(circuit.layout_segments) * 8, circuit.name)
            self.assertLess(max(segment_lengths(circuit.track_coords)), 24.0, circuit.name)
            self.assertGreater(len(circuit.pit_lane_coords), 8, circuit.name)
            if circuit.pit_lane and circuit.pit_lane.wall_offset is not None:
                self.assertGreater(len(circuit.pit_wall_coords), 8, circuit.name)

    def test_source_pit_lanes_are_anchored_to_rendered_track(self) -> None:
        circuits = {circuit.id: circuit for circuit in load_circuits()}

        for circuit_id in _real_circuit_ids():
            circuit = circuits[circuit_id]
            first = circuit.pit_lane_coords[0]
            last = circuit.pit_lane_coords[-1]
            first_distance = _distance_to_polyline(first, circuit.track_coords)
            last_distance = _distance_to_polyline(last, circuit.track_coords)

            self.assertLess(first_distance, 0.01, circuit.name)
            self.assertLess(last_distance, 0.01, circuit.name)

    def test_landmark_progress_is_resolved_to_compiled_track_index(self) -> None:
        for circuit in load_circuits():
            progress_landmarks = [
                landmark for landmark in circuit.landmarks if landmark.progress is not None
            ]
            self.assertGreater(len(progress_landmarks), 0, circuit.name)
            for landmark in progress_landmarks:
                self.assertLess(landmark.track_index, len(circuit.track_coords) - 1, landmark.label)

    def test_red_bull_ring_pit_lane_is_anchored_to_track(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)
        first = circuit.pit_lane_coords[0]
        last = circuit.pit_lane_coords[-1]

        first_distance = min(hypot(first[0] - point[0], first[1] - point[1]) for point in circuit.track_coords)
        last_distance = min(hypot(last[0] - point[0], last[1] - point[1]) for point in circuit.track_coords)

        self.assertLess(first_distance, 12.0)
        self.assertLess(last_distance, 12.0)
        self.assertGreater(len(circuit.pit_wall_coords), 8)

    def test_red_bull_ring_pit_lane_has_no_angular_entry_or_exit_kink(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)
        points = [
            (float(point[0]), float(point[1]))
            for point in circuit.pit_lane_coords
        ]
        heading_changes = []
        for previous, point, following in zip(points, points[1:], points[2:]):
            incoming = atan2(point[1] - previous[1], point[0] - previous[0])
            outgoing = atan2(following[1] - point[1], following[0] - point[0])
            delta = (outgoing - incoming + pi) % (2 * pi) - pi
            heading_changes.append(abs(degrees(delta)))

        self.assertLessEqual(max(heading_changes), 25.0)
        self.assertLessEqual(heading_changes[0], 12.0)
        self.assertLessEqual(heading_changes[-1], 12.0)

        _, entry_tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.entry_progress,
        )
        _, exit_tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )
        entry_heading = atan2(
            points[1][1] - points[0][1],
            points[1][0] - points[0][0],
        )
        exit_heading = atan2(
            points[-1][1] - points[-2][1],
            points[-1][0] - points[-2][0],
        )
        entry_track_heading = atan2(entry_tangent[1], entry_tangent[0])
        exit_track_heading = atan2(exit_tangent[1], exit_tangent[0])
        self.assertLessEqual(
            abs(degrees((entry_heading - entry_track_heading + pi) % (2 * pi) - pi)),
            7.0,
        )
        self.assertLessEqual(
            abs(degrees((exit_heading - exit_track_heading + pi) % (2 * pi) - pi)),
            7.0,
        )

    def test_red_bull_ring_pit_exit_continuation_stays_separate_then_rejoins(
        self,
    ) -> None:
        engine = _make_engine_for_circuit(4)
        pit_lane = engine.circuit.pit_lane
        assert pit_lane is not None
        assert pit_lane.exit_lane_rejoin_progress is not None

        points = engine._pit_exit_lane_points_m()
        self.assertGreater(len(points), 30)
        self.assertAlmostEqual(engine._pit_exit_lane_length_m(), 192.7, delta=1.0)
        start_x, start_y, _ = engine._pit_lane_pose_at_progress_m(
            pit_lane.side_rejoin_progress
        )
        self.assertAlmostEqual(points[0][0], start_x, delta=0.02)
        self.assertAlmostEqual(points[0][1], start_y, delta=0.02)

        merge_route_progress = pit_lane.exit_lane_merge_start * 0.8
        lane_x, lane_y, _ = engine._pit_exit_lane_pose_at_progress_m(
            merge_route_progress
        )
        start_track_progress = engine._pit_exit_lane_start_track_progress()
        assert start_track_progress is not None
        track_progress = (
            start_track_progress
            + engine._progress_distance(
                start_track_progress,
                pit_lane.exit_lane_rejoin_progress,
            )
            * merge_route_progress
        ) % 1.0
        line_x, line_y, line_heading = engine._track_physics.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        separated_m = abs(
            (lane_x - line_x) * -sin(line_heading)
            + (lane_y - line_y) * cos(line_heading)
        )
        self.assertGreater(separated_m, 7.5)

        rejoin_x, rejoin_y, _ = engine._track_physics.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            pit_lane.exit_lane_rejoin_progress,
        )
        self.assertAlmostEqual(points[-1][0], rejoin_x, delta=0.02)
        self.assertAlmostEqual(points[-1][1], rejoin_y, delta=0.02)

    def test_red_bull_ring_pit_out_uses_dedicated_lane_until_progress_015(
        self,
    ) -> None:
        engine = _make_engine_for_circuit(4)
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        pit_lane = engine.circuit.pit_lane
        assert pit_lane is not None and pit_lane.entry_progress is not None
        starting_lap = state.current_lap

        state.pit_request = TireCompound.HARD
        state.progress = pit_lane.entry_progress - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        phases_seen: set[str] = set()
        observed_total_progress = [state.total_progress]
        while state.in_pit:
            phases_seen.add(engine._pit_phase[driver_id])
            engine.tick(PHYSICS_STEP_SECONDS)
            observed_total_progress.append(state.total_progress)

        self.assertIn("exit_lane", phases_seen)
        self.assertEqual(observed_total_progress, sorted(observed_total_progress))
        self.assertEqual(state.pit_count, 1)
        self.assertAlmostEqual(
            state.total_progress,
            starting_lap + 1.0 + pit_lane.exit_lane_rejoin_progress,
            delta=0.001,
        )

    def test_bahrain_pit_exit_follows_race_direction(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 3)
        previous_point = circuit.pit_lane_coords[-2]
        exit_point = circuit.pit_lane_coords[-1]
        _, tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )

        exit_vector = (
            exit_point[0] - previous_point[0],
            exit_point[1] - previous_point[1],
        )
        dot = exit_vector[0] * tangent[0] + exit_vector[1] * tangent[1]

        self.assertGreater(dot, 0.0)
        self.assertAlmostEqual(
            circuit.pit_lane.safety_car_line_2_progress,
            circuit.pit_lane.exit_progress,
            delta=0.002,
        )

    def test_bahrain_pit_exit_continuation_stays_separate_then_rejoins(self) -> None:
        engine = _make_engine_for_circuit(3)
        pit_lane = engine.circuit.pit_lane
        assert pit_lane is not None
        assert pit_lane.exit_lane_rejoin_progress is not None

        points = engine._pit_exit_lane_points_m()
        self.assertGreater(len(points), 12)
        start_x, start_y, _ = engine._pit_lane_pose_at_progress_m(
            pit_lane.side_rejoin_progress
        )
        self.assertAlmostEqual(points[0][0], start_x, delta=0.02)
        self.assertAlmostEqual(points[0][1], start_y, delta=0.02)

        merge_index = int(len(points) * pit_lane.exit_lane_merge_start * 0.8)
        merge_route_progress = merge_index / (len(points) - 1)
        start_track_progress = engine._pit_exit_lane_start_track_progress()
        assert start_track_progress is not None
        track_progress = (
            start_track_progress
            + engine._progress_distance(
                start_track_progress,
                pit_lane.exit_lane_rejoin_progress,
            )
            * merge_route_progress
        ) % 1.0
        line_x, line_y, line_heading = engine._track_physics.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        separated_m = abs(
            (points[merge_index][0] - line_x) * -sin(line_heading)
            + (points[merge_index][1] - line_y) * cos(line_heading)
        )
        self.assertGreater(separated_m, 2.5)

        rejoin_x, rejoin_y, _ = engine._track_physics.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            pit_lane.exit_lane_rejoin_progress,
        )
        self.assertAlmostEqual(points[-1][0], rejoin_x, delta=0.02)
        self.assertAlmostEqual(points[-1][1], rejoin_y, delta=0.02)

    def test_bahrain_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 3)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:bahrain-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 5412.0, delta=2.0)
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.9595, delta=0.02)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.1014, delta=0.02)

    def test_bahrain_segment_lookup_matches_expected_driving_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 3)

        self.assertEqual(segment_at_progress(circuit, 0.05).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(
            segment_at_progress(circuit, 0.12).type,
            TrackSegmentType.HEAVY_BRAKING,
        )
        self.assertEqual(segment_at_progress(circuit, 0.20).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.45).type, TrackSegmentType.TECHNICAL)
        self.assertEqual(segment_at_progress(circuit, 0.52).type, TrackSegmentType.HEAVY_BRAKING)
        self.assertEqual(segment_at_progress(circuit, 0.57).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.70).type, TrackSegmentType.TRACTION)
        self.assertEqual(segment_at_progress(circuit, 0.78).type, TrackSegmentType.TRACTION)
        self.assertEqual(segment_at_progress(circuit, 0.82).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.88).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.89).type, TrackSegmentType.HEAVY_BRAKING)
        self.assertEqual(segment_at_progress(circuit, 0.92).type, TrackSegmentType.TRACTION)
        self.assertEqual(segment_at_progress(circuit, 0.97).type, TrackSegmentType.STRAIGHT)

    def test_bahrain_uses_major_sector_lines_and_twenty_seven_timing_loops(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 3)

        self.assertEqual(len(circuit.sectors), 3)
        self.assertEqual(
            [(sector.start, sector.end) for sector in circuit.sectors],
            [(0.0, 0.345), (0.345, 0.7821308), (0.7821308, 1.0)],
        )
        self.assertEqual(
            [sector.mini_sector_count for sector in circuit.sectors],
            [9, 12, 6],
        )

    def test_bahrain_drs_zones_avoid_corner_landmarks(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 3)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        self.assertEqual(segment_at_progress(circuit, 0.97).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.05).type, TrackSegmentType.STRAIGHT)
        self.assertLess(zones["Back Straight"].end, landmarks["T11"])
        self.assertGreaterEqual(zones["Back Straight"].start, 0.55)
        self.assertEqual(segment_at_progress(circuit, zones["Back Straight"].start).type, TrackSegmentType.STRAIGHT)
        self.assertGreater(zones["T13-T14 Straight"].start, landmarks["T13"])
        self.assertAlmostEqual(landmarks["T13"], 0.791, delta=0.002)
        self.assertGreaterEqual(zones["T13-T14 Straight"].start, 0.800)
        self.assertAlmostEqual(zones["T13-T14 Straight"].end, 0.884, delta=0.001)
        self.assertLess(zones["T13-T14 Straight"].end, landmarks["T14"])

    def test_red_bull_ring_segment_lookup_matches_expected_driving_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)

        self.assertEqual(segment_at_progress(circuit, 0.05).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.08).type, TrackSegmentType.HEAVY_BRAKING)
        self.assertEqual(segment_at_progress(circuit, 0.15).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.30).type, TrackSegmentType.HEAVY_BRAKING)
        self.assertEqual(segment_at_progress(circuit, 0.36).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.49).type, TrackSegmentType.HEAVY_BRAKING)
        self.assertEqual(segment_at_progress(circuit, 0.55).type, TrackSegmentType.TRACTION)
        self.assertEqual(segment_at_progress(circuit, 0.68).type, TrackSegmentType.TECHNICAL)
        self.assertEqual(segment_at_progress(circuit, 0.80).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.87).type, TrackSegmentType.SWEEPING)
        self.assertEqual(segment_at_progress(circuit, 0.91).type, TrackSegmentType.SWEEPING)
        self.assertEqual(segment_at_progress(circuit, 0.94).type, TrackSegmentType.TRACTION)
        self.assertEqual(segment_at_progress(circuit, 0.95).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, 0.98).type, TrackSegmentType.STRAIGHT)

    def test_red_bull_ring_sectors_use_official_timing_ranges_and_27_loops(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)

        self.assertEqual(
            [(sector.start, sector.end) for sector in circuit.sectors],
            [(0.0, 0.28086), (0.28086, 0.673139), (0.673139, 1.0)],
        )
        self.assertEqual(
            [sector.mini_sector_count for sector in circuit.sectors],
            [9, 12, 6],
        )
        self.assertEqual(sum(sector.mini_sector_count for sector in circuit.sectors), 27)
        self.assertIsNotNone(circuit.sector_timing_source)
        assert circuit.sector_timing_source is not None
        self.assertIn("fia.com", circuit.sector_timing_source.source_url)
        self.assertEqual(circuit.sector_timing_source.acquired_at, "2026-07-31")
        self.assertAlmostEqual(
            sum(circuit.sector_timing_source.sector_lengths_m),
            circuit.sector_timing_source.source_centerline_length_m,
            places=6,
        )

        engine = _make_engine()
        self.assertEqual(len(engine._timing_loops), 27)
        self.assertEqual(engine._timing_loops[0].progress, 0.0)
        self.assertAlmostEqual(engine._timing_loops[9].progress, 0.28086, places=6)
        self.assertAlmostEqual(engine._timing_loops[21].progress, 0.673139, places=6)

    def test_red_bull_ring_segments_have_corner_specific_speed_factors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)
        rindt_sweep = segment_at_progress(circuit, 0.87)
        final_complex = segment_at_progress(circuit, 0.90)
        downhill_sweep = segment_at_progress(circuit, 0.55)
        t10_traction = segment_at_progress(circuit, 0.94)

        self.assertEqual(rindt_sweep.type, TrackSegmentType.SWEEPING)
        self.assertEqual(final_complex.type, TrackSegmentType.SWEEPING)
        self.assertGreater(rindt_sweep.speed_factor, final_complex.speed_factor)
        self.assertEqual(downhill_sweep.type, TrackSegmentType.TRACTION)
        self.assertEqual(t10_traction.type, TrackSegmentType.TRACTION)
        self.assertGreater(downhill_sweep.speed_factor, t10_traction.speed_factor)

    def test_red_bull_ring_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:red-bull-ring-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 4318.0, delta=12.0)
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.8718, delta=0.02)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.1218, delta=0.02)

    def test_red_bull_ring_drs_zones_stay_on_straights(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 4)
        zones = {zone.name: zone for zone in circuit.drs_zones}

        self.assertEqual(set(zones), {"Main Straight", "Uphill DRS Climb", "Top Straight"})
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        self.assertAlmostEqual(zones["Main Straight"].start, 0.946, delta=0.001)
        for zone in zones.values():
            self.assertEqual(segment_at_progress(circuit, zone.start).type, TrackSegmentType.STRAIGHT)
            self.assertEqual(segment_at_progress(circuit, zone.end - 0.001).type, TrackSegmentType.STRAIGHT)

    def test_spa_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 6)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:spa-francorchamps-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 7004.0, delta=2.0)
        self.assertAlmostEqual(circuit.geo.computed_length_m, 7004.0, delta=0.5)
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.946, delta=0.015)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.063, delta=0.015)

    def test_spa_drs_zones_and_landmarks_match_fia_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 6)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        self.assertEqual(set(zones), {"Kemmel Straight", "Main Straight"})
        self.assertEqual(segment_at_progress(circuit, zones["Kemmel Straight"].start).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, zones["Kemmel Straight"].end - 0.001).type, TrackSegmentType.STRAIGHT)
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        self.assertEqual(segment_at_progress(circuit, zones["Main Straight"].start).type, TrackSegmentType.STRAIGHT)
        self.assertEqual(segment_at_progress(circuit, zones["Main Straight"].end - 0.001).type, TrackSegmentType.STRAIGHT)
        self.assertAlmostEqual(landmarks["T1 La Source"], 0.039, delta=0.001)
        self.assertAlmostEqual(landmarks["T4 Raidillon"], 0.156, delta=0.001)
        self.assertAlmostEqual(landmarks["T18 Bus Stop"], 0.946, delta=0.001)

    def test_hungaroring_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 7)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:hungaroring-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 4372.1, delta=12.0)
        self.assertAlmostEqual(circuit.geo.computed_length_m, 4381.0, delta=0.5)
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.882, delta=0.015)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.138, delta=0.015)

    def test_hungaroring_drs_zones_and_landmarks_match_fia_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 7)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        self.assertEqual(set(zones), {"Main Straight", "T1-T2 Straight"})
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        for zone in zones.values():
            self.assertEqual(segment_at_progress(circuit, zone.start).type, TrackSegmentType.STRAIGHT)
            self.assertEqual(segment_at_progress(circuit, zone.end - 0.001).type, TrackSegmentType.STRAIGHT)
        self.assertAlmostEqual(
            (zones["Main Straight"].start - landmarks["T14"]) * circuit.track_length_m,
            40.0,
            delta=2.0,
        )
        self.assertAlmostEqual(
            (zones["T1-T2 Straight"].start - landmarks["T1"]) * circuit.track_length_m,
            6.0,
            delta=1.0,
        )
        self.assertAlmostEqual(landmarks["T1"], 0.149, delta=0.001)
        self.assertAlmostEqual(landmarks["T14"], 0.946, delta=0.001)

    def test_hungaroring_pit_exit_follows_race_direction(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 7)
        previous_point = circuit.pit_lane_coords[-2]
        exit_point = circuit.pit_lane_coords[-1]
        _, tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )
        exit_vector = (
            exit_point[0] - previous_point[0],
            exit_point[1] - previous_point[1],
        )

        self.assertGreater(exit_vector[0] * tangent[0] + exit_vector[1] * tangent[1], 0.0)

    def test_monza_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 8)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:monza-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 5798.0, delta=12.0)
        self.assertAlmostEqual(circuit.geo.computed_length_m, 5793.0, delta=0.5)
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.903, delta=0.015)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.032, delta=0.015)

    def test_monza_drs_zones_and_landmarks_match_fia_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 8)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        self.assertEqual(set(zones), {"Main Straight", "Serraglio Straight"})
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        for zone in zones.values():
            self.assertEqual(segment_at_progress(circuit, zone.start).type, TrackSegmentType.STRAIGHT)
            self.assertEqual(segment_at_progress(circuit, zone.end - 0.001).type, TrackSegmentType.STRAIGHT)
        # FIA 2025: DRS activation 1 is 170m after Turn 7, activation 2 is 20m
        # after Turn 11 (the map lists activation 2 relative to the corner exit).
        self.assertAlmostEqual(
            (zones["Serraglio Straight"].start - landmarks["T7"]) * circuit.track_length_m,
            190.0,
            delta=25.0,
        )
        self.assertAlmostEqual(landmarks["T1"], 0.107, delta=0.001)
        self.assertAlmostEqual(landmarks["T8"], 0.628, delta=0.001)
        self.assertAlmostEqual(landmarks["T11"], 0.845, delta=0.001)

    def test_monza_pit_exit_follows_race_direction(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 8)
        previous_point = circuit.pit_lane_coords[-2]
        exit_point = circuit.pit_lane_coords[-1]
        _, tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )
        exit_vector = (
            exit_point[0] - previous_point[0],
            exit_point[1] - previous_point[1],
        )

        self.assertGreater(exit_vector[0] * tangent[0] + exit_vector[1] * tangent[1], 0.0)

    def test_zandvoort_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 9)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:zandvoort-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 4261.9, delta=12.0)
        self.assertAlmostEqual(circuit.geo.computed_length_m, 4259.0, delta=0.5)
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.907, delta=0.015)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.093, delta=0.015)

    def test_zandvoort_drs_zones_and_landmarks_match_fia_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 9)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        self.assertEqual(set(zones), {"Main Straight", "Back Straight"})
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        for zone in zones.values():
            self.assertEqual(segment_at_progress(circuit, zone.start).type, TrackSegmentType.STRAIGHT)
            self.assertEqual(segment_at_progress(circuit, zone.end - 0.001).type, TrackSegmentType.STRAIGHT)
        # FIA 2025: back-straight activation is 50m after Turn 10's exit.
        self.assertGreater(zones["Back Straight"].start, landmarks["T10"])
        self.assertLess(zones["Back Straight"].end, landmarks["T11"])
        # T1 Tarzan sits 164m from the grid (RaceFans/FIA event data).
        self.assertAlmostEqual(landmarks["T1"], 0.054, delta=0.002)
        self.assertAlmostEqual(landmarks["T14"], 0.849, delta=0.002)

    def test_zandvoort_pit_exit_follows_race_direction(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 9)
        previous_point = circuit.pit_lane_coords[-2]
        exit_point = circuit.pit_lane_coords[-1]
        _, tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )
        exit_vector = (
            exit_point[0] - previous_point[0],
            exit_point[1] - previous_point[1],
        )

        self.assertGreater(exit_vector[0] * tangent[0] + exit_vector[1] * tangent[1], 0.0)

    def test_barcelona_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 10)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:barcelona-catalunya-gp")
        # 2025 layout without the T14-15 chicane (OSM moto loop rerouted via the
        # fast final-corner bypass ways).
        self.assertAlmostEqual(circuit.geo.source_length_m, 4664.0, delta=12.0)
        self.assertAlmostEqual(circuit.geo.computed_length_m, 4657.0, delta=0.5)
        # Pit entry branches between T13 and T14 and cuts inside the final corner.
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.88, delta=0.015)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.107, delta=0.015)

    def test_barcelona_drs_zones_and_landmarks_match_fia_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 10)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        self.assertEqual(set(zones), {"Main Straight", "Back Straight"})
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        for zone in zones.values():
            self.assertEqual(segment_at_progress(circuit, zone.start).type, TrackSegmentType.STRAIGHT)
            self.assertEqual(segment_at_progress(circuit, zone.end - 0.001).type, TrackSegmentType.STRAIGHT)
        # FIA event data: back-straight activation sits 40m after Turn 9's exit.
        self.assertGreater(zones["Back Straight"].start, landmarks["T9"])
        self.assertLess(zones["Back Straight"].end, landmarks["T10"])
        # Pole to the first braking zone is 565m with a 125m braking distance,
        # so T1's apex lands 690m plus the corner arc into the lap.
        self.assertAlmostEqual(landmarks["T1"], 0.154, delta=0.002)
        self.assertAlmostEqual(landmarks["T10"], 0.723, delta=0.002)
        self.assertAlmostEqual(landmarks["T14"], 0.899, delta=0.002)
        # 14 corners on the 2025 layout: no chicane landmarks remain.
        corner_labels = [lm.label for lm in circuit.landmarks if lm.type == "corner"]
        self.assertEqual(len(corner_labels), 14)

    def test_barcelona_pit_exit_follows_race_direction(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 10)
        previous_point = circuit.pit_lane_coords[-2]
        exit_point = circuit.pit_lane_coords[-1]
        _, tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )
        exit_vector = (
            exit_point[0] - previous_point[0],
            exit_point[1] - previous_point[1],
        )

        self.assertGreater(exit_vector[0] * tangent[0] + exit_vector[1] * tangent[1], 0.0)

    def test_suzuka_geo_source_matches_official_layout_anchors(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 11)

        self.assertIsNotNone(circuit.geo)
        self.assertEqual(circuit.geo.source_id, "osm:suzuka-gp")
        self.assertAlmostEqual(circuit.geo.source_length_m, 5811.4, delta=12.0)
        self.assertAlmostEqual(circuit.geo.computed_length_m, 5807.0, delta=0.5)
        # Pit lane parallels the main straight from Casio exit to Turn 1.
        self.assertAlmostEqual(circuit.pit_lane.entry_progress, 0.909, delta=0.015)
        self.assertAlmostEqual(circuit.pit_lane.exit_progress, 0.063, delta=0.015)

    def test_suzuka_drs_zones_and_landmarks_match_fia_sections(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 11)
        zones = {zone.name: zone for zone in circuit.drs_zones}
        landmarks = {landmark.label: landmark.progress for landmark in circuit.landmarks}

        # 2025 Japanese GP: single DRS zone on the main straight.
        self.assertEqual(set(zones), {"Main Straight"})
        self.assertGreater(zones["Main Straight"].start, zones["Main Straight"].end)
        self.assertEqual(
            segment_at_progress(circuit, zones["Main Straight"].start).type,
            TrackSegmentType.STRAIGHT,
        )
        self.assertEqual(
            segment_at_progress(circuit, zones["Main Straight"].end - 0.001).type,
            TrackSegmentType.STRAIGHT,
        )
        # Pole to first braking zone is 277m; T1 apex follows shortly after.
        self.assertAlmostEqual(landmarks["T1"], 0.076, delta=0.003)
        self.assertAlmostEqual(landmarks["T8"], 0.268, delta=0.003)
        self.assertAlmostEqual(landmarks["T15"], 0.828, delta=0.003)
        corner_labels = [lm.label for lm in circuit.landmarks if lm.type == "corner"]
        self.assertEqual(len(corner_labels), 18)

    def test_suzuka_pit_exit_follows_race_direction(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 11)
        previous_point = circuit.pit_lane_coords[-2]
        exit_point = circuit.pit_lane_coords[-1]
        _, tangent = _point_and_tangent_at_progress(
            circuit.track_coords,
            circuit.pit_lane.exit_progress,
        )
        exit_vector = (
            exit_point[0] - previous_point[0],
            exit_point[1] - previous_point[1],
        )

        self.assertGreater(exit_vector[0] * tangent[0] + exit_vector[1] * tangent[1], 0.0)

    def test_real_circuits_have_complete_pit_lane_operational_config(self) -> None:
        """Tier A: every geo circuit pins its pit operational landmarks."""
        for circuit in load_circuits():
            if circuit.geo is None:
                continue
            pit = circuit.pit_lane
            self.assertIsNotNone(pit, circuit.name)
            self.assertIsNotNone(pit.entry_progress, circuit.name)
            self.assertIsNotNone(pit.exit_progress, circuit.name)
            self.assertIsNotNone(pit.safety_car_line_2_progress, circuit.name)
            self.assertAlmostEqual(
                pit.safety_car_line_2_progress,
                pit.exit_progress,
                delta=0.002,
                msg=circuit.name,
            )
            self.assertEqual(pit.speed_limit_kph, 80.0, circuit.name)
            self.assertLess(pit.side_entry_progress, pit.speed_limit_start, circuit.name)
            self.assertLess(pit.speed_limit_start, pit.box_progress, circuit.name)
            self.assertLess(pit.box_progress, pit.speed_limit_end, circuit.name)
            self.assertLess(pit.speed_limit_end, pit.side_rejoin_progress, circuit.name)
            self.assertGreaterEqual(pit.lane_width_m, 3.0, circuit.name)


class TireModelTests(unittest.TestCase):
    def test_wear_increases_with_age(self) -> None:
        self.assertLess(compute_wear(TireCompound.MEDIUM, 5), compute_wear(TireCompound.MEDIUM, 15))

    def test_wear_accepts_fractional_age(self) -> None:
        self.assertLess(
            compute_wear(TireCompound.MEDIUM, 5),
            compute_wear(TireCompound.MEDIUM, 5.5),
        )

    def test_performance_decreases_with_age(self) -> None:
        self.assertGreater(
            compute_tire_performance(TireCompound.SOFT, 2),
            compute_tire_performance(TireCompound.SOFT, 20),
        )

    def test_performance_has_no_085_floor(self) -> None:
        self.assertLess(compute_tire_performance(TireCompound.SOFT, 30), 0.85)

    def test_performance_loss_is_curved(self) -> None:
        fresh = compute_tire_performance(TireCompound.SOFT, 0)
        mid = compute_tire_performance(TireCompound.SOFT, 10)
        old = compute_tire_performance(TireCompound.SOFT, 20)

        early_loss = fresh - mid
        later_loss = mid - old
        self.assertGreater(later_loss, early_loss)

    def test_performance_variation_adjusts_tire_output(self) -> None:
        base = compute_tire_performance(TireCompound.MEDIUM, 12)
        high = compute_tire_performance(TireCompound.MEDIUM, 12, 0.004)
        low = compute_tire_performance(TireCompound.MEDIUM, 12, -0.004)

        self.assertGreater(high, base)
        self.assertLess(low, base)

    def test_fresh_dry_compounds_have_distinct_physical_grip(self) -> None:
        soft = compute_tire_physics_factors(TireCompound.SOFT, 0.0)
        medium = compute_tire_physics_factors(TireCompound.MEDIUM, 0.0)
        hard = compute_tire_physics_factors(TireCompound.HARD, 0.0)

        self.assertGreater(soft.lateral_grip, medium.lateral_grip)
        self.assertGreater(medium.lateral_grip, hard.lateral_grip)
        self.assertGreater(soft.traction_grip, medium.traction_grip)
        self.assertGreater(medium.braking_grip, hard.braking_grip)

    def test_wear_reduces_each_physical_grip_channel(self) -> None:
        fresh = compute_tire_physics_factors(TireCompound.SOFT, 0.0)
        worn = compute_tire_physics_factors(TireCompound.SOFT, 20.0)

        self.assertGreater(fresh.lateral_grip, worn.lateral_grip)
        self.assertGreater(fresh.traction_grip, worn.traction_grip)
        self.assertGreater(fresh.braking_grip, worn.braking_grip)
        self.assertGreater(
            fresh.traction_grip - worn.traction_grip,
            fresh.braking_grip - worn.braking_grip,
        )

    def test_tire_management_changes_effective_tire_age(self) -> None:
        self.assertLess(
            tire_management_age_multiplier(0.95),
            tire_management_age_multiplier(0.76),
        )
        self.assertLess(
            compute_managed_tire_age(20, 0.95),
            compute_managed_tire_age(20, 0.76),
        )

    def test_compound_balance_has_distinct_stint_windows(self) -> None:
        def stint_total(compound: TireCompound, stint_laps: int) -> float:
            return sum(64.0 / compute_tire_performance(compound, age) for age in range(stint_laps))

        short_stint = {
            compound: stint_total(compound, 10)
            for compound in (TireCompound.SOFT, TireCompound.MEDIUM, TireCompound.HARD)
        }
        medium_stint = {
            compound: stint_total(compound, 20)
            for compound in (TireCompound.SOFT, TireCompound.MEDIUM, TireCompound.HARD)
        }
        long_stint = {
            compound: stint_total(compound, 35)
            for compound in (TireCompound.SOFT, TireCompound.MEDIUM, TireCompound.HARD)
        }

        self.assertEqual(min(short_stint, key=short_stint.get), TireCompound.SOFT)
        self.assertEqual(min(medium_stint, key=medium_stint.get), TireCompound.MEDIUM)
        self.assertEqual(min(long_stint, key=long_stint.get), TireCompound.HARD)


class AIStrategyTests(unittest.TestCase):
    def test_ai_tire_choice_matches_balanced_stint_windows(self) -> None:
        self.assertEqual(choose_tire_for_remaining_laps(10), TireCompound.SOFT)
        self.assertEqual(choose_tire_for_remaining_laps(20), TireCompound.MEDIUM)
        self.assertEqual(choose_tire_for_remaining_laps(35), TireCompound.HARD)

    def test_ai_does_not_pit_fresh_tires_just_because_race_is_long(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[3]

        self.assertFalse(
            should_pit(
                state,
                remaining_laps=57,
                is_player=False,
                tire_management=engine._driver_meta[3]["tire_management"],
            )
        )

    def test_ai_pits_when_tires_are_worn(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[3]
        state.tire_usage = 24.0

        self.assertTrue(
            should_pit(
                state,
                remaining_laps=33,
                is_player=False,
                tire_management=engine._driver_meta[3]["tire_management"],
            )
        )


class PerformanceBalanceTests(unittest.TestCase):
    def test_driver_pace_mapping_is_compressed_for_closer_racing(self) -> None:
        self.assertLess(
            driver_pace_multiplier(0.98) - driver_pace_multiplier(0.78),
            0.06,
        )

    def test_constructor_performance_has_independent_strengths(self) -> None:
        teams = {team.id: team for team in load_teams()}
        red_bull = car_performance_factors(teams[1])
        ferrari = car_performance_factors(teams[2])
        mclaren = car_performance_factors(teams[3])

        self.assertGreater(ferrari.braking, red_bull.braking)
        self.assertGreater(red_bull.top_speed, ferrari.top_speed)
        self.assertGreater(mclaren.high_speed_grip, ferrari.high_speed_grip)

    def test_grid_theoretical_lap_range_stays_close(self) -> None:
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        base_lap_time = load_circuits()[0].base_lap_time
        tire_performance = compute_tire_performance(TireCompound.MEDIUM, 0)
        lap_times = []

        for driver in drivers:
            team = teams[driver.team_id]
            lap_times.append(
                base_lap_time / (
                    car_performance_factors(team).qualifying
                    * driver_pace_multiplier(driver.stats.pace)
                    * tire_performance
                )
            )

        self.assertLess(max(lap_times) - min(lap_times), base_lap_time * 0.07)


class QualifyingTests(unittest.TestCase):
    def test_bahrain_qualifying_clock_is_anchored_to_telemetry_reference(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuit = load_circuits()[0]
        result = run_qualifying(
            circuit=circuit,
            drivers=drivers,
            teams={team.id: team for team in teams},
            player_team=teams[0],
            attempt_laps=3,
            seed=7,
        )

        self.assertIsNotNone(circuit.physics_calibration)
        self.assertEqual(
            result.results[0].q3_time,
            circuit.physics_calibration.reference_lap_time_seconds,
        )

    def test_qualifying_knockout_format_builds_grid(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        team_map = {team.id: team for team in teams}
        circuit = load_circuits()[0]

        result = run_qualifying(
            circuit=circuit,
            drivers=drivers,
            teams=team_map,
            player_team=teams[0],
            attempt_laps=3,
            seed=7,
        )

        self.assertEqual(len(result.results), len(drivers))
        self.assertEqual(result.grid_order, [entry.driver_id for entry in result.results])
        self.assertEqual(result.results[0].gap, "POLE")
        self.assertEqual(result.results[0].knockout, "Q3")
        self.assertTrue(all(len(entry.laps) == 3 for entry in result.results))

        # 20명 필드 기준 표준 F1 녹아웃 구간 (Q3 10 / Q2 5 / Q1 5)
        knockouts = [entry.knockout for entry in result.results]
        self.assertEqual(knockouts[:10], ["Q3"] * 10)
        self.assertEqual(knockouts[10:15], ["Q2"] * 5)
        self.assertEqual(knockouts[15:20], ["Q1"] * 5)

    def test_qualifying_session_times_match_knockout_stage(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        team_map = {team.id: team for team in teams}
        circuit = load_circuits()[0]

        result = run_qualifying(
            circuit=circuit,
            drivers=drivers,
            teams=team_map,
            player_team=teams[0],
            attempt_laps=3,
            seed=7,
        )

        # Q3 진출자는 세 세션 기록을 모두 보유한다.
        for entry in result.results[:10]:
            self.assertIsNotNone(entry.q1_time)
            self.assertIsNotNone(entry.q2_time)
            self.assertIsNotNone(entry.q3_time)
        # Q2 탈락자는 Q3 기록이 없다.
        for entry in result.results[10:15]:
            self.assertIsNotNone(entry.q1_time)
            self.assertIsNotNone(entry.q2_time)
            self.assertIsNone(entry.q3_time)
        # Q1 탈락자는 Q1 기록만 보유한다.
        for entry in result.results[15:20]:
            self.assertIsNotNone(entry.q1_time)
            self.assertIsNone(entry.q2_time)
            self.assertIsNone(entry.q3_time)

        # 각 녹아웃 블록 내부는 해당 세션 베스트랩 오름차순으로 정렬된다.
        q3_times = [entry.q3_time for entry in result.results[:10]]
        q2_block = [entry.q2_time for entry in result.results[10:15]]
        q1_block = [entry.q1_time for entry in result.results[15:20]]
        self.assertEqual(q3_times, sorted(q3_times))
        self.assertEqual(q2_block, sorted(q2_block))
        self.assertEqual(q1_block, sorted(q1_block))


class RaceEngineTests(unittest.TestCase):
    def test_traction_loss_event_requires_sustained_or_peak_slip(self) -> None:
        engine = _make_engine()
        driver_id = next(iter(engine.driver_states))

        previous_slip = 0.0
        for _ in range(5):
            self.assertFalse(
                engine._traction_loss_event_ready(
                    driver_id,
                    0.065,
                    previous_slip,
                    PHYSICS_STEP_SECONDS,
                )
            )
            previous_slip = 0.065
        self.assertTrue(
            engine._traction_loss_event_ready(
                driver_id,
                0.065,
                previous_slip,
                PHYSICS_STEP_SECONDS,
            )
        )
        self.assertFalse(
            engine._traction_loss_event_ready(
                driver_id,
                0.065,
                previous_slip,
                PHYSICS_STEP_SECONDS,
            )
        )

        self.assertFalse(
            engine._traction_loss_event_ready(
                driver_id,
                0.0,
                0.065,
                PHYSICS_STEP_SECONDS,
            )
        )
        self.assertTrue(
            engine._traction_loss_event_ready(
                driver_id,
                0.11,
                0.0,
                PHYSICS_STEP_SECONDS,
            )
        )

    def test_clean_standard_car_avoids_sustained_track_exit_on_all_circuits(self) -> None:
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        for circuit in load_circuits():
            engine = RaceEngine(
                circuit=circuit,
                drivers=[driver],
                teams=teams,
                player_team_id=driver.team_id,
                player_driver_ids=[driver.id],
                seed=2026,
                start_sequence_enabled=False,
            )
            state = engine.driver_states[driver.id]
            consecutive_off_track_ticks = 0
            maximum_off_track_ticks = 0
            while state.current_lap < 2:
                engine.tick(GAME_TICK_SECONDS)
                if state.off_track:
                    consecutive_off_track_ticks += 1
                    maximum_off_track_ticks = max(
                        maximum_off_track_ticks,
                        consecutive_off_track_ticks,
                    )
                else:
                    consecutive_off_track_ticks = 0
                self.assertFalse(
                    state.track_limits_active,
                    f"{circuit.name} at progress {state.progress:.4f}",
                )
            self.assertLessEqual(
                maximum_off_track_ticks,
                10,
                circuit.name,
            )

    def test_spa_telemetry_calibration_keeps_clean_car_on_track_through_la_source(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 6)
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams=teams,
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=42,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[driver.id]
        core_speeds = []
        left_track = False

        for _ in range(400):
            engine.tick(0.05)
            distance_m = state.progress * engine.track_length_m
            if 220.0 <= distance_m <= 310.0:
                core_speeds.append(state.speed_kph)
            left_track = left_track or state.off_track
            if distance_m >= 800.0:
                break

        self.assertIsNotNone(circuit.physics_calibration)
        self.assertEqual(engine._vehicle_physics.planner_braking_utilization, 0.55)
        self.assertEqual(engine._vehicle_physics.controller_sample_distance_m, 10.0)
        self.assertFalse(left_track)
        self.assertTrue(core_speeds)
        self.assertLess(min(core_speeds), 115.0)

    def test_bahrain_reference_controller_keeps_clean_car_inside_track(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams=teams,
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=2026,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[driver.id]

        while state.current_lap < 2:
            engine.tick(GAME_TICK_SECONDS)
            self.assertFalse(
                state.off_track,
                f"Bahrain at progress {state.progress:.4f}",
            )

        self.assertAlmostEqual(
            engine._vehicle_physics.reference_lap_time,
            89.841,
        )

    def test_collision_severity_probability_increases_with_impact_speed(self) -> None:
        class FixedRng:
            @staticmethod
            def random() -> float:
                return 0.5

        self.assertEqual(
            escalate_collision(
                FixedRng(),
                segment_type="straight",
                impact_speed_mps=8.0,
            ),
            IncidentSeverity.MINOR,
        )
        self.assertEqual(
            escalate_collision(
                FixedRng(),
                segment_type="straight",
                impact_speed_mps=30.0,
            ),
            IncidentSeverity.CRASH,
        )

    def test_engine_uses_fixed_step_physics(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]
        progress_before = state.total_progress

        engine.tick(GAME_TICK_SECONDS)
        tick = engine.build_tick_state()
        position = next(item for item in tick.positions if item.driver_id == state.driver_id)
        self.assertEqual(tick.physics_hz, 50)
        self.assertGreater(position.target_speed_kph, 0.0)
        self.assertGreaterEqual(position.grip_utilization, 0.0)
        self.assertIn(
            position.handling_state,
            {"stable", "understeer", "oversteer", "run_wide"},
        )
        self.assertGreater(state.total_progress, progress_before)

    def test_live_race_uses_cached_vehicle_specific_trajectory_profiles(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)

        self.assertEqual(
            set(engine._track_physics_by_driver),
            set(engine.driver_states),
        )
        self.assertEqual(
            set(engine._vehicle_physics_by_driver_line),
            set(engine.driver_states),
        )

        # Team-mates on the same compound share one immutable cached profile.
        self.assertIs(
            engine._track_physics_by_driver[1],
            engine._track_physics_by_driver[2],
        )

        red_bull_profile = engine._track_physics_by_driver[1]
        ferrari_profile = engine._track_physics_by_driver[3]
        self.assertIsNot(red_bull_profile, ferrari_profile)
        red_bull_offsets = red_bull_profile.driving_line_samples[
            DRIVING_LINE_RACING
        ]
        ferrari_offsets = ferrari_profile.driving_line_samples[
            DRIVING_LINE_RACING
        ]
        # Different vehicle specs must get independent profiles, but they can
        # legitimately converge on the same geometric optimum.  Their speed
        # profiles still differ through aero, grip, braking and traction data.
        self.assertEqual(len(red_bull_offsets), len(ferrari_offsets))
        self.assertNotAlmostEqual(
            red_bull_profile.predicted_line_lap_times[DRIVING_LINE_RACING],
            ferrari_profile.predicted_line_lap_times[DRIVING_LINE_RACING],
            places=3,
        )

    def test_live_vehicle_targets_its_own_line_and_distance_mapping(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        for driver_id in (1, 3):
            state = engine.driver_states[driver_id]
            profile = engine._track_physics_for_driver(state)
            state.progress = 0.371
            state.total_progress = 1.371
            line_distance_m = profile.line_distance_at_total_progress(
                DRIVING_LINE_RACING,
                state.total_progress,
            )
            restored_progress = profile.total_progress_at_line_distance(
                DRIVING_LINE_RACING,
                line_distance_m,
            )
            self.assertAlmostEqual(restored_progress, state.total_progress, places=6)

            expected_offset_m = profile.line_offset_at_progress(
                DRIVING_LINE_RACING,
                state.progress,
            )
            target_offset_m = engine._physics_v2_target_lateral_offset(
                state,
                profile.at_progress(state.progress),
                DRIVING_LINE_RACING,
                None,
                None,
            )
            self.assertAlmostEqual(target_offset_m, expected_offset_m, places=6)
            self.assertAlmostEqual(
                state.target_lateral_offset_m,
                profile.line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    engine._grid_start_progress[driver_id],
                ),
                places=6,
            )

    def test_clean_live_car_replans_local_trajectory_at_regular_cadence(self) -> None:
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams=teams,
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=99,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[driver.id]

        ticks_per_plan = round(
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS / GAME_TICK_SECONDS
        )
        engine.tick(GAME_TICK_SECONDS)
        self.assertNotIn(driver.id, engine._local_trajectory_plans)
        engine.tick(GAME_TICK_SECONDS)
        first_plan = engine._local_trajectory_plans[driver.id]
        self.assertEqual(first_plan.selected_candidate_id, "center")
        self.assertEqual(len(first_plan.candidates), 5)
        self.assertTrue(first_plan.selected.viable)
        self.assertLess(
            abs(
                state.target_lateral_offset_m
                - engine._track_physics_for_driver(state).line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
            ),
            0.02,
        )

        for _ in range(ticks_per_plan - 1):
            engine.tick(GAME_TICK_SECONDS)
            self.assertIs(engine._local_trajectory_plans[driver.id], first_plan)
            self.assertGreater(
                engine._local_trajectory_selected_ages[driver.id],
                0.0,
            )
        engine.tick(GAME_TICK_SECONDS)
        next_plan = engine._local_trajectory_plans[driver.id]
        self.assertIsNot(next_plan, first_plan)
        self.assertEqual(next_plan.selection_reason, "continued")
        self.assertGreaterEqual(
            engine._local_trajectory_selected_ages[driver.id],
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
        )

    def test_local_trajectory_plan_yields_to_race_control(self) -> None:
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams=teams,
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=99,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[driver.id]
        engine.tick(GAME_TICK_SECONDS)
        engine.tick(GAME_TICK_SECONDS)
        self.assertIn(driver.id, engine._local_trajectory_plans)

        engine._trigger_vsc([])
        progress_before_tick = state.progress
        engine.tick(GAME_TICK_SECONDS)

        self.assertNotIn(driver.id, engine._local_trajectory_plans)
        expected = engine._track_physics_for_driver(state).line_offset_at_progress(
            DRIVING_LINE_RACING,
            progress_before_tick,
        )
        self.assertAlmostEqual(state.target_lateral_offset_m, expected, places=4)

    def test_live_local_plan_receives_current_tire_wear_and_grip(self) -> None:
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams=teams,
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=99,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[driver.id]
        engine._set_state_total_progress(state, 0.92)
        profile = engine._track_physics_for_driver(state)
        bounds = engine._track_surface.trajectory_body_lateral_bounds(
            state.progress,
            body_width_m=state.car_width_m,
            edge_margin_m=0.35,
        )
        state.lateral_offset_m = bounds[0] + 0.02
        state.speed_kph = 65.0 * 3.6
        state.tire_usage = 30.0
        modifiers = engine._physics_v2_modifiers(state)

        engine._update_local_trajectory_plan(
            state,
            DRIVING_LINE_RACING,
            modifiers,
            0.20,
        )
        plan = engine._local_trajectory_plans[state.driver_id]
        kerb_candidate = next(
            candidate
            for candidate in plan.candidates
            if candidate.candidate_id == "right_2"
        )

        self.assertGreater(state.tire_wear, 0.8)
        self.assertGreater(kerb_candidate.surface_cost_seconds, 0.0)
        self.assertGreater(kerb_candidate.tire_surface_cost_seconds, 0.0)

    def test_sparse_traffic_is_connected_to_local_trajectory_prediction(self) -> None:
        drivers = load_drivers()[:3]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=99,
            start_sequence_enabled=False,
        )
        follower = engine.driver_states[drivers[0].id]
        leader = engine.driver_states[drivers[1].id]
        follower_progress = 0.12
        engine._set_state_total_progress(follower, follower_progress)
        engine._set_state_total_progress(
            leader,
            follower_progress + 40.0 / engine.track_length_m,
        )
        profile = engine._track_physics_for_driver(follower)
        corridor_bias_m = 0.5
        for state in (follower, leader):
            offset = profile.line_offset_at_progress(
                DRIVING_LINE_RACING,
                state.progress,
            )
            state.lateral_offset_m = offset + corridor_bias_m
            state.target_lateral_offset_m = state.lateral_offset_m
        follower.speed_kph = 70.0 * 3.6
        leader.speed_kph = 35.0 * 3.6
        follower.acceleration_mps2 = 0.0
        leader.acceleration_mps2 = 0.0

        self.assertTrue(
            engine._local_trajectory_planning_allowed(
                follower,
                DRIVING_LINE_RACING,
            )
        )
        engine._update_local_trajectory_plan(
            follower,
            DRIVING_LINE_RACING,
            engine._physics_v2_modifiers(follower),
            0.20,
        )
        plan = engine._local_trajectory_plans[follower.driver_id]

        self.assertEqual(len(plan.candidates), 7)
        self.assertAlmostEqual(
            0.5
            * (
                min(candidate.lateral_bias_m for candidate in plan.candidates)
                + max(candidate.lateral_bias_m for candidate in plan.candidates)
            ),
            corridor_bias_m,
            places=6,
        )
        self.assertEqual(
            tuple(item.driver_id for item in plan.opponent_occupancies),
            (leader.driver_id,),
        )
        self.assertTrue(plan.selected.viable)
        self.assertNotEqual(plan.selected_candidate_id, "center")

    def test_close_two_car_straight_chase_enables_local_pull_out_plan(self) -> None:
        drivers = load_drivers()[:2]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=99,
            start_sequence_enabled=False,
        )
        follower = engine.driver_states[drivers[0].id]
        leader = engine.driver_states[drivers[1].id]
        engine._set_state_total_progress(follower, 0.12)
        engine._set_state_total_progress(
            leader,
            0.12 + 20.0 / engine.track_length_m,
        )
        follower.drs_active = True
        follower.tow_strength = 1.0

        self.assertTrue(
            engine._local_trajectory_planning_allowed(
                follower,
                DRIVING_LINE_RACING,
            )
        )
        self.assertTrue(
            engine._local_trajectory_planning_allowed(
                leader,
                DRIVING_LINE_RACING,
            )
        )

    def test_dense_three_car_pack_uses_live_occupancy_planner(self) -> None:
        drivers = load_drivers()[:3]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=99,
            start_sequence_enabled=False,
        )
        follower = engine.driver_states[drivers[0].id]
        leader = engine.driver_states[drivers[1].id]
        trailing = engine.driver_states[drivers[2].id]
        engine._set_state_total_progress(follower, 0.12)
        engine._set_state_total_progress(
            leader,
            0.12 + 20.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            trailing,
            0.12 - 15.0 / engine.track_length_m,
        )

        self.assertTrue(
            engine._local_trajectory_planning_allowed(
                follower,
                DRIVING_LINE_RACING,
            )
        )

    def test_imminent_collision_rejects_every_straight_pull_out_candidate(self) -> None:
        drivers = load_drivers()[:2]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=99,
            start_sequence_enabled=False,
        )
        attacker = engine.driver_states[drivers[0].id]
        defender = engine.driver_states[drivers[1].id]
        engine._set_state_total_progress(attacker, 0.12)
        engine._set_state_total_progress(
            defender,
            0.12 + 20.0 / engine.track_length_m,
        )
        profile = engine._track_physics_for_driver(attacker)
        for state in (attacker, defender):
            state.lateral_offset_m = profile.line_offset_at_progress(
                DRIVING_LINE_RACING,
                state.progress,
            )
            state.target_lateral_offset_m = state.lateral_offset_m
        attacker.speed_kph = 70.0 * 3.6
        defender.speed_kph = 35.0 * 3.6

        _prepare_local_pull_out_plan(engine, attacker)
        plan = engine._local_trajectory_plans[attacker.driver_id]

        self.assertFalse(any(candidate.viable for candidate in plan.candidates))
        self.assertIsNone(
            engine._local_pull_out_decision(attacker, defender)
        )

    def test_physics_v2_same_line_following_preserves_car_length_gap(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        leader, follower = ordered[:2]
        engine._set_state_total_progress(leader, 1.2)
        engine._set_state_total_progress(
            follower,
            1.2 - 8.0 / engine.track_length_m,
        )
        leader.speed_kph = 120.0
        follower.speed_kph = 300.0
        engine._trigger_vsc([])

        engine.tick(GAME_TICK_SECONDS)

        gap_m = (leader.total_progress - follower.total_progress) * engine.track_length_m
        self.assertGreaterEqual(gap_m, SC_CAR_LENGTH_M - 1e-6)
        self.assertLessEqual(follower.speed_kph, leader.speed_kph + 1e-6)
        self.assertLess(leader.position, follower.position)

    def test_close_gap_resolution_never_copies_speed_after_physics(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        leader, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._set_state_total_progress(leader, 1.12)
        engine._set_state_total_progress(
            follower,
            leader.total_progress
            - (PHYSICAL_CAR_LENGTH_M + 0.5) / engine.track_length_m,
        )
        leader.lateral_offset_m = follower.lateral_offset_m = 0.0
        leader.speed_kph = 120.0
        follower.speed_kph = 300.0

        engine._resolve_physics_v2_same_line_gaps()

        self.assertEqual(leader.speed_kph, 120.0)
        self.assertEqual(follower.speed_kph, 300.0)

    def test_green_close_chase_targets_a_small_bumper_gap(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        leader, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        _place_close_pair(engine, leader, follower, 1.2, bumper_gap_m=12.0)
        leader.speed_kph = 260.0
        follower.speed_kph = 280.0
        snapshot = {
            leader.driver_id: (leader.total_progress, leader.speed_kph / 3.6),
            follower.driver_id: (follower.total_progress, follower.speed_kph / 3.6),
        }

        following = engine._physics_v2_following_constraint(
            follower,
            leader,
            snapshot,
            DRIVING_LINE_RACING,
            GAME_TICK_SECONDS,
        )

        self.assertIsNotNone(following)
        assert following is not None
        self.assertAlmostEqual(
            following.desired_gap_m,
            PHYSICAL_CAR_LENGTH_M + 2.50,
        )
        self.assertEqual(
            following.minimum_gap_m,
            PHYSICAL_CAR_LENGTH_M + FOLLOWING_MIN_BUMPER_GAP_M,
        )

    def test_pull_out_progressively_releases_longitudinal_gap(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="pull_out",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None

        attacker.lateral_offset_m = defender.lateral_offset_m
        aligned = engine._maneuver_following_gaps_m(attacker, defender, battle)
        attacker.lateral_offset_m = defender.lateral_offset_m + 2.0
        almost_clear = engine._maneuver_following_gaps_m(attacker, defender, battle)
        attacker.lateral_offset_m = defender.lateral_offset_m + 2.15
        clear = engine._maneuver_following_gaps_m(attacker, defender, battle)

        self.assertEqual(aligned[1], PHYSICAL_CAR_LENGTH_M + 0.25)
        self.assertGreater(aligned[0], almost_clear[0])
        self.assertGreater(almost_clear[0], clear[0])
        self.assertAlmostEqual(clear[1], 0.0, places=6)

    def test_corner_overlap_is_not_limited_as_same_line_following(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_active = True
        attacker.lateral_offset_m = (
            defender.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )

        self.assertEqual(
            engine._physics_v2_virtual_line(attacker),
            DRIVING_LINE_RACING,
        )
        self.assertTrue(
            engine._physics_v2_passing_authorized(attacker, defender)
        )

    def test_aborted_maneuver_rebuilds_gap_before_lateral_rejoin(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._set_state_total_progress(
            attacker,
            defender.total_progress - 2.0 / engine.track_length_m,
        )
        attacker.lateral_offset_m = defender.lateral_offset_m + 2.3
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="abort",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None

        desired_gap_m, minimum_gap_m = engine._maneuver_following_gaps_m(
            attacker,
            defender,
            battle,
        )
        hold_target_m = engine._physics_v2_target_lateral_offset(
            attacker,
            engine._track_physics.at_progress(attacker.progress),
            DRIVING_LINE_RACING,
            defender,
            None,
        )

        self.assertEqual(desired_gap_m, PHYSICAL_CAR_LENGTH_M + 1.25)
        self.assertEqual(minimum_gap_m, PHYSICAL_CAR_LENGTH_M + 0.25)
        self.assertGreaterEqual(
            abs(hold_target_m - defender.lateral_offset_m),
            PHYSICAL_CAR_WIDTH_M + 0.25,
        )

    def test_late_straight_attack_is_rejected(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        defender = engine.driver_states[14]
        attacker = engine.driver_states[1]
        defender.position = 1
        attacker.position = 2
        _place_close_pair(engine, defender, attacker, 1.098, bumper_gap_m=1.5)
        attacker.drs_active = True
        attacker.tow_strength = 1.0

        self.assertFalse(engine._overtake_opportunity_is_viable(attacker, defender))

        _place_close_pair(engine, defender, attacker, 1.058, bumper_gap_m=1.5)
        self.assertTrue(engine._overtake_opportunity_is_viable(attacker, defender))

    def test_physics_v2_virtual_lines_only_authorize_green_flag_passes(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        defender, attacker = ordered[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            "test corner",
            phase="pull_out",
        )

        self.assertEqual(engine._physics_v2_virtual_line(attacker), ATTACK_LINE_INSIDE)
        self.assertEqual(engine._physics_v2_virtual_line(defender), DEFENDER_LINE_RACING)
        self.assertFalse(engine._physics_v2_passing_authorized(attacker, defender))
        attacker.lateral_offset_m = defender.lateral_offset_m + 2.2
        self.assertTrue(engine._physics_v2_passing_authorized(attacker, defender))

        engine._trigger_vsc([])
        self.assertFalse(engine._physics_v2_passing_authorized(attacker, defender))

        engine.race_phase = "sc"
        engine._sc_unlap_driver_ids.add(attacker.driver_id)
        self.assertTrue(engine._physics_v2_passing_authorized(attacker, defender))

    def test_active_battle_line_uses_its_own_physical_profile(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        defender, attacker = ordered[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            "test corner",
            phase="pull_out",
        )

        engine.tick(GAME_TICK_SECONDS)

        self.assertEqual(attacker.racing_line, ATTACK_LINE_INSIDE)
        self.assertIsNot(
            engine._vehicle_physics_by_line[ATTACK_LINE_INSIDE],
            engine._vehicle_physics_by_line[DEFENDER_LINE_RACING],
        )
        self.assertNotEqual(
            engine._vehicle_physics_by_line[ATTACK_LINE_INSIDE].track_length_m,
            engine._vehicle_physics_by_line[DEFENDER_LINE_RACING].track_length_m,
        )

    def test_ai_refines_attack_template_to_create_lateral_clearance(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        defender, attacker = ordered[:2]
        engine._set_state_total_progress(defender, 0.20)
        engine._set_state_total_progress(
            attacker,
            0.20 - 30.0 / engine.track_length_m,
        )
        defender.lateral_offset_m = 0.0
        defender.lateral_speed_mps = 0.0
        attacker.lateral_offset_m = 0.0
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            "test corner",
        )
        track_sample = engine._track_physics.at_progress(attacker.progress)

        target = engine._physics_v2_target_lateral_offset(
            attacker,
            track_sample,
            ATTACK_LINE_INSIDE,
            defender,
            None,
        )

        self.assertGreaterEqual(
            abs(target - defender.lateral_offset_m),
            PHYSICAL_CAR_WIDTH_M + 0.35 - 1e-6,
        )

    def test_physics_v2_green_battles_never_overlap_physical_car_bodies(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)

        for _ in range(800):
            engine.tick(GAME_TICK_SECONDS)
            ordered = sorted(
                (
                    state
                    for state in engine.driver_states.values()
                    if not state.retired and not state.finished and not state.in_pit
                ),
                key=lambda state: state.position,
            )
            for state in ordered:
                minimum_lateral_m, maximum_lateral_m = (
                    engine._track_surface.safety_lateral_bounds(state.progress)
                )
                self.assertLessEqual(
                    state.lateral_offset_m,
                    maximum_lateral_m + 1e-6,
                )
                self.assertGreaterEqual(
                    state.lateral_offset_m,
                    minimum_lateral_m - 1e-6,
                )
            for ahead, behind in zip(ordered, ordered[1:]):
                overlap = oriented_body_overlap(
                    engine._body_pose(ahead),
                    engine._body_pose(behind),
                )
                if overlap is not None:
                    self.assertLessEqual(overlap[0], 0.003)

    def test_physics_v2_completes_sc_lifecycle_without_track_overtakes(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        order_at_deploy = [
            state.driver_id
            for state in sorted(engine.driver_states.values(), key=lambda state: state.position)
        ]
        engine._trigger_safety_car([])
        engine._sc_cleanup_until = 0.0
        event_types: set[str] = set()

        for _ in range(3000):
            events = engine.tick(GAME_TICK_SECONDS)
            event_types.update(event.type for event in events)
            if engine.race_phase == "green":
                break

        order_at_green = [
            state.driver_id
            for state in sorted(engine.driver_states.values(), key=lambda state: state.position)
        ]
        self.assertEqual(engine.race_phase, "green")
        self.assertEqual(order_at_deploy, order_at_green)
        self.assertTrue(
            {"sc_track_join", "sc_queue", "sc_in_this_lap", "sc_pit", "sc_end"}
            .issubset(event_types)
        )

    def test_sc_physical_order_inversion_activates_smooth_place_give_back(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        engine._trigger_safety_car([])
        engine.safety_car_stage = "collecting"
        running = engine._sc_ordered_on_track_states()
        predecessor, yielding = running[:2]
        engine._safety_car_total_progress = (
            predecessor.total_progress + engine._sc_target_gap_progress()
        )
        engine._set_state_total_progress(
            yielding,
            predecessor.total_progress + 15.0 / engine.track_length_m,
        )
        predecessor.speed_kph = 120.0
        yielding.speed_kph = 160.0

        engine._sync_safety_car_queue([])

        self.assertEqual(
            engine._sc_order_yield_targets.get(yielding.driver_id),
            predecessor.driver_id,
        )
        self.assertEqual(engine._sc_order_correction.get("phase"), "MOVE_ASIDE")
        self.assertGreater(
            engine._race_control_speed_cap_mps(yielding),
            predecessor.speed_kph / 3.6,
        )
        lateral_target = engine._sc_order_correction.get("lateral_target_m")
        self.assertIsInstance(lateral_target, (int, float))
        yielding.lateral_offset_m = float(lateral_target)
        engine._sync_safety_car_queue([])
        self.assertEqual(engine._sc_order_correction.get("phase"), "YIELDING")
        self.assertLess(
            engine._race_control_speed_cap_mps(yielding),
            predecessor.speed_kph / 3.6,
        )
        self.assertLess(predecessor.position, yielding.position)

        engine._set_state_total_progress(
            predecessor,
            yielding.total_progress
            + (
                SC_CAR_LENGTH_M
                * SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS
                / engine.track_length_m
            ),
        )
        engine._sync_safety_car_queue([])
        self.assertEqual(engine._sc_order_correction.get("phase"), "CONFIRM_ORDER")
        engine._sync_safety_car_queue([])
        self.assertEqual(engine._sc_order_correction.get("phase"), "MERGE_BACK")
        yielding.lateral_offset_m = engine._track_physics_for_driver(
            yielding
        ).line_offset_at_progress(
            DRIVING_LINE_RACING,
            yielding.progress,
        )
        engine._sync_safety_car_queue([])
        engine._sync_safety_car_queue([])
        self.assertNotIn(yielding.driver_id, engine._sc_order_yield_targets)

    def test_sc_order_correction_releases_reserved_corridor_before_full_car_gap(
        self,
    ) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        engine._trigger_safety_car([])
        engine.safety_car_stage = "collecting"
        predecessor, yielding = engine._sc_ordered_on_track_states()[:2]
        engine._set_state_total_progress(predecessor, 1.20)
        engine._set_state_total_progress(
            yielding,
            predecessor.total_progress + 5.0 / engine.track_length_m,
        )
        engine._sc_order_correction = {
            "phase": "MOVE_ASIDE",
            "yielding_driver_id": yielding.driver_id,
            "predecessor_driver_id": predecessor.driver_id,
            "lateral_target_m": predecessor.lateral_offset_m - 1.0,
        }

        self.assertTrue(
            engine._sc_order_correction_allows_pass(predecessor, yielding)
        )

    def test_sc_reconciles_a_six_car_physical_inversion_with_engine_ticks(self) -> None:
        """A frozen sporting queue must not deadlock the physical queue at SC pace."""
        engine = _make_engine_for_circuit(4, seed=7)
        engine._trigger_safety_car([])
        sporting = sorted(engine.driver_states.values(), key=lambda state: state.position)[:6]
        for state in engine.driver_states.values():
            if state not in sporting:
                state.retired = True

        engine._sc_running_order = [state.driver_id for state in sporting]
        engine.safety_car_stage = "collecting"
        engine._safety_car_speed_mps = 110.0 / 3.6
        engine._safety_car_progress_rate = engine._safety_car_speed_mps / engine.track_length_m
        for index, state in enumerate(sporting):
            # Sporting order is A-B-C-D-E-F, while physical progress is F-E-D-C-B-A.
            engine._set_state_total_progress(state, 0.15 + index * 0.012)
            state.speed_kph = 110.0
        engine._safety_car_total_progress = (
            sporting[0].total_progress + engine._sc_target_gap_progress()
        )
        engine._initialize_authoritative_vehicle_telemetry()
        engine._sync_safety_car_queue([])

        initial_physical_order = [
            state.driver_id for state in sorted(sporting, key=lambda state: -state.total_progress)
        ]
        self.assertEqual(initial_physical_order, [state.driver_id for state in reversed(sporting)])

        previous_progress = {state.driver_id: state.total_progress for state in sporting}
        simultaneous_stop_seconds = 0.0
        maximum_simultaneous_stop_seconds = 0.0
        for _ in range(round(90.0 / GAME_TICK_SECONDS)):
            engine.tick(GAME_TICK_SECONDS)
            physical_order = sorted(
                sporting,
                key=lambda state: -state.total_progress,
            )
            correction = engine._sc_order_correction
            if (
                correction is not None
                and correction.get("phase")
                in {"WAIT_SAFE_ZONE", "MOVE_ASIDE", "YIELDING"}
            ):
                yielding_id = correction.get("yielding_driver_id")
                predecessor_id = correction.get("predecessor_driver_id")
                physical_ids = [state.driver_id for state in physical_order]
                self.assertEqual(
                    abs(
                        physical_ids.index(yielding_id)
                        - physical_ids.index(predecessor_id)
                    ),
                    1,
                )
            if sum(state.speed_kph < 5.0 for state in sporting) >= len(sporting) - 1:
                simultaneous_stop_seconds += GAME_TICK_SECONDS
                maximum_simultaneous_stop_seconds = max(
                    maximum_simultaneous_stop_seconds,
                    simultaneous_stop_seconds,
                )
            else:
                simultaneous_stop_seconds = 0.0
            for state in sporting:
                self.assertLess(
                    state.total_progress - previous_progress[state.driver_id],
                    0.003,
                )
                previous_progress[state.driver_id] = state.total_progress

        final_physical_order = [
            state.driver_id for state in sorted(sporting, key=lambda state: -state.total_progress)
        ]
        # The real 50 Hz controller must make progress on the inversion while
        # waiting for safe straight/corridor windows; it must not leave the
        # entire reversed field at the nominal 64.8 km/h floor.
        self.assertNotEqual(final_physical_order, initial_physical_order)
        self.assertLessEqual(maximum_simultaneous_stop_seconds, 8.0)
        self.assertFalse(all(state.speed_kph <= 64.8 + 0.5 for state in sporting))
        self.assertTrue(
            any(
                predecessor.total_progress - yielding.total_progress
                >= SC_CAR_LENGTH_M
                * SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS
                / engine.track_length_m
                for predecessor, yielding in zip(sporting, sporting[1:])
            )
        )

    def test_p1_is_always_leader(self) -> None:
        engine = _make_engine()
        for _ in range(30):
            engine.tick(GAME_TICK_SECONDS)
            tick = engine.build_tick_state()
            p1 = next(p for p in tick.positions if p.position == 1)
            self.assertEqual(p1.gap, "LEADER")
            self.assertEqual(
                sum(1 for p in tick.positions if p.gap == "LEADER"),
                1,
            )

    def test_gaps_and_intervals_are_non_negative(self) -> None:
        engine = _make_engine()
        for _ in range(20):
            engine.tick(GAME_TICK_SECONDS)
            tick = engine.build_tick_state()
            for pos in tick.positions:
                if pos.retired:
                    continue
                if pos.gap != "LEADER":
                    self.assertTrue(pos.gap.startswith("+"))
                if pos.interval != "—":
                    self.assertTrue(pos.interval.startswith("+"))

    def test_live_gap_updates_after_first_lap(self) -> None:
        engine = _make_engine()
        observed = []
        for i in range(1, 70):
            engine.tick(GAME_TICK_SECONDS * 5)
            if i in (25, 40, 55):
                tick = engine.build_tick_state()
                observed.append(tick.positions[1].gap)

        self.assertGreater(len(set(observed)), 1)

    def test_gap_uses_latest_common_timing_loop_crossing_times(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        leader = engine.driver_states[1]
        follower = engine.driver_states[5]
        loop = engine._timing_loops[1]
        leader_start = 10.0
        follower_start = 11.234
        previous = loop.progress - 0.005
        current = loop.progress + 0.005

        engine._record_timing_loop_crossings(
            leader,
            previous,
            current,
            leader_start,
            0.02,
        )
        engine._record_timing_loop_crossings(
            follower,
            previous,
            current,
            follower_start,
            0.02,
        )

        self.assertAlmostEqual(
            engine._timing_gap_seconds_between(leader, follower),
            follower_start - leader_start,
            places=6,
        )
        for state in engine.driver_states.values():
            if state.driver_id not in {leader.driver_id, follower.driver_id}:
                state.retired = True
        leader.position = 1
        follower.position = 2
        engine._tick_phase = TickPhase.RULES
        expected_live_gap, anchored = engine._live_timing_gap_seconds_between(
            leader,
            follower,
        )
        engine._update_gaps()
        tick = engine.build_tick_state()
        follower_tick = next(
            position for position in tick.positions
            if position.driver_id == follower.driver_id
        )
        self.assertTrue(follower_tick.timing_gap_valid)
        self.assertTrue(follower_tick.interval_timing_gap_valid)
        self.assertEqual(follower_tick.timing_gap_source, "live")
        self.assertEqual(follower_tick.interval_timing_gap_source, "live")
        self.assertTrue(anchored)
        self.assertIsNotNone(expected_live_gap)
        self.assertAlmostEqual(
            follower_tick.gap_seconds or 0.0,
            expected_live_gap or 0.0,
            places=3,
        )

    def test_reversed_latest_timing_crossing_does_not_collapse_gap_to_zero(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        leader = engine.driver_states[1]
        follower = engine.driver_states[5]
        for state in engine.driver_states.values():
            if state.driver_id not in {leader.driver_id, follower.driver_id}:
                state.retired = True
        leader.position = 1
        follower.position = 2
        engine._set_state_total_progress(leader, 1.20)
        engine._set_state_total_progress(follower, 1.18)
        leader.speed_kph = 190.0
        follower.speed_kph = 205.0
        loop_index = engine._timing_loops[2].index
        # This is a stale pre-pass ordering: the current leader crossed later.
        engine._timing_crossings[leader.driver_id][(1, loop_index)] = 11.0
        engine._timing_crossings[follower.driver_id][(1, loop_index)] = 10.0

        self.assertIsNone(engine._timing_gap_seconds_between(leader, follower))
        engine._tick_phase = TickPhase.RULES
        engine._update_gaps()
        follower_tick = next(
            item for item in engine.build_tick_state().positions
            if item.driver_id == follower.driver_id
        )

        self.assertGreater(follower_tick.gap_seconds or 0.0, 0.001)
        self.assertGreater(follower_tick.interval_seconds or 0.0, 0.001)
        self.assertNotEqual(follower_tick.gap, "+0.000")
        self.assertNotEqual(follower_tick.interval, "+0.000")
        self.assertFalse(follower_tick.timing_gap_valid)
        self.assertFalse(follower_tick.interval_timing_gap_valid)
        self.assertEqual(follower_tick.timing_gap_source, "estimated")
        self.assertEqual(follower_tick.interval_timing_gap_source, "estimated")

    def test_live_gap_changes_between_timing_loop_crossings(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        leader = engine.driver_states[1]
        follower = engine.driver_states[5]
        for state in engine.driver_states.values():
            if state.driver_id not in {leader.driver_id, follower.driver_id}:
                state.retired = True
        leader.position = 1
        follower.position = 2
        loop_index = engine._timing_loops[2].index
        engine._timing_crossings[leader.driver_id][(1, loop_index)] = 10.0
        engine._timing_crossings[follower.driver_id][(1, loop_index)] = 11.5
        engine._set_state_total_progress(leader, 1.20)
        engine._set_state_total_progress(follower, 1.18)
        leader.speed_kph = 190.0
        follower.speed_kph = 210.0
        engine._tick_phase = TickPhase.RULES

        engine._update_gaps()
        first_gap = follower.gap_to_leader
        engine._set_state_total_progress(follower, 1.19)
        follower.speed_kph = 230.0
        engine._update_gaps()
        second_gap = follower.gap_to_leader
        follower_tick = next(
            item for item in engine.build_tick_state().positions
            if item.driver_id == follower.driver_id
        )

        self.assertNotAlmostEqual(first_gap, second_gap, places=3)
        self.assertTrue(follower_tick.timing_gap_valid)
        self.assertEqual(follower_tick.timing_gap_source, "live")

    def test_live_mini_sector_splits_compare_overall_and_personal_best(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        leader = engine.driver_states[1]
        follower = engine.driver_states[5]
        for state in (leader, follower):
            engine._timing_crossings[state.driver_id][(0, 0)] = 0.0
        loop = engine._timing_loops[1]
        previous = loop.progress - 0.005
        current = loop.progress + 0.005
        engine._record_timing_loop_crossings(
            leader,
            previous,
            current,
            10.0,
            0.02,
        )
        engine._record_timing_loop_crossings(
            follower,
            previous,
            current,
            10.5,
            0.02,
        )

        tick = engine.build_tick_state()
        leader_tick = next(
            position for position in tick.positions
            if position.driver_id == leader.driver_id
        )
        follower_tick = next(
            position for position in tick.positions
            if position.driver_id == follower.driver_id
        )

        self.assertEqual(leader_tick.mini_sector_statuses[0], "overall_best")
        self.assertEqual(follower_tick.mini_sector_statuses[0], "personal_best")
        self.assertAlmostEqual(
            follower_tick.last_mini_sector_delta_to_best or 0.0,
            0.5,
            places=3,
        )

    def test_completed_lap_contains_three_sector_and_twenty_seven_mini_times(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        driver_id = 1
        crossings = engine._timing_crossings[driver_id]
        for loop in engine._timing_loops:
            crossings[(0, loop.index)] = loop.index * 2.0
        crossings[(1, 0)] = len(engine._timing_loops) * 2.0

        sectors, minis = engine._lap_split_times(driver_id, lap=1)

        self.assertEqual(sectors, [18.0, 24.0, 12.0])
        self.assertEqual(minis, [2.0] * 27)

    def test_compact_dashboard_and_timing_match_full_tick_fields(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        runtime_metrics = {
            "broadcast_hz": 30,
            "effective_speed_multiplier": 1.75,
            "simulation_backlog_seconds": 0.0123,
            "broadcast_jitter_ms": 4.56,
        }
        full = engine.build_tick_state(
            runtime_metrics=runtime_metrics,
        ).model_dump()
        dashboard = engine.build_dashboard_payload(runtime_metrics)
        timing = engine.build_timing_payload()

        for field, value in dashboard.items():
            if field in {"type", "positions"}:
                continue
            self.assertEqual(value, full[field], field)

        full_by_driver = {
            position["driver_id"]: position
            for position in full["positions"]
        }
        for position in dashboard["positions"]:
            expected = full_by_driver[position["driver_id"]]
            for field, value in position.items():
                self.assertEqual(value, expected[field], field)
        for position in timing["positions"]:
            expected = full_by_driver[position["driver_id"]]
            for field, value in position.items():
                self.assertEqual(value, expected[field], field)

    def test_raw_timing_crossings_are_bounded_after_lap_summary(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        driver_id = 1
        crossings = engine._timing_crossings[driver_id]
        for lap_index in range(12):
            for loop in engine._timing_loops:
                crossings[(lap_index, loop.index)] = float(
                    lap_index * 100 + loop.index,
                )

        engine._prune_timing_crossings(driver_id, completed_lap=11)

        self.assertEqual(
            min(lap_index for lap_index, _ in crossings),
            11 - TIMING_CROSSING_LAPS_TO_RETAIN,
        )
        self.assertLessEqual(
            len(crossings),
            (TIMING_CROSSING_LAPS_TO_RETAIN + 1) * len(engine._timing_loops),
        )

    def test_grid_uses_distance_offset_at_lights_out(self) -> None:
        engine = _make_engine()
        pole_id = next(s.driver_id for s in engine.driver_states.values() if s.position == 1)
        back_id = next(s.driver_id for s in engine.driver_states.values() if s.position == 20)

        # 그리드 정렬: 폴은 스타트라인, 뒤 차량은 거리만큼 라인 뒤에 위치한다.
        self.assertEqual(engine.driver_states[pole_id].total_progress, 0.0)
        self.assertLess(engine.driver_states[back_id].total_progress, 0.0)

        pole_before = engine.driver_states[pole_id].total_progress
        back_before = engine.driver_states[back_id].total_progress

        engine.tick(GAME_TICK_SECONDS)

        # 라이트아웃 시 전원이 동시에 전진한다(순차 출발 아님).
        self.assertGreater(engine.driver_states[pole_id].total_progress, pole_before)
        self.assertGreater(engine.driver_states[back_id].total_progress, back_before)
        # 그리드 간격 때문에 폴이 여전히 앞선다.
        self.assertGreater(
            engine.driver_states[pole_id].total_progress,
            engine.driver_states[back_id].total_progress,
        )

    def test_start_sequence_holds_staggered_grid_until_lights_out(self) -> None:
        engine = _make_starting_engine()
        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        initial_progress = {
            state.driver_id: state.total_progress
            for state in ordered
        }

        self.assertFalse(engine.race_started)
        self.assertEqual(engine.start_sequence_phase, "grid")
        self.assertTrue(all(state.speed_kph == 0.0 for state in ordered))
        self.assertTrue(all(state.brake == 1.0 for state in ordered))
        for index, state in enumerate(ordered):
            expected_lateral = GRID_COLUMN_OFFSET_M if index % 2 == 0 else -GRID_COLUMN_OFFSET_M
            self.assertAlmostEqual(state.lateral_offset_m, expected_lateral)
        for ahead, behind in zip(ordered, ordered[1:]):
            longitudinal_gap_m = (
                ahead.total_progress - behind.total_progress
            ) * engine.track_length_m
            self.assertAlmostEqual(longitudinal_gap_m, GRID_SLOT_SPACING_M, places=5)

        events = []
        while not engine.race_started:
            events.extend(engine.tick(GAME_TICK_SECONDS))

        self.assertGreaterEqual(engine._start_sequence_elapsed, GRID_LIGHTS_OUT_SECONDS)
        self.assertEqual(engine.start_sequence_phase, "lights_out")
        self.assertEqual(engine.race_elapsed, 0.0)
        self.assertIn("race_start", [event.type for event in events])
        for state in ordered:
            self.assertEqual(state.total_progress, initial_progress[state.driver_id])

        engine.tick(GAME_TICK_SECONDS)

        self.assertGreater(engine.race_elapsed, 0.0)
        self.assertTrue(all(state.speed_kph > 0.0 for state in ordered))
        self.assertTrue(
            all(
                state.total_progress > initial_progress[state.driver_id]
                for state in ordered
            )
        )

    def test_start_tick_and_race_info_share_physical_grid_slots(self) -> None:
        engine = _make_starting_engine(circuit_id=6)
        tick = engine.build_tick_state()
        slots = engine.get_grid_slots()

        self.assertFalse(tick.race_started)
        self.assertEqual(tick.start_sequence_phase, "grid")
        self.assertEqual(tick.start_light_count, 0)
        self.assertFalse(tick.overtaking_allowed)
        self.assertEqual(len(slots), len(engine.driver_states))
        for slot in slots:
            state = next(
                item
                for item in engine.driver_states.values()
                if item.position == slot["position"]
            )
            self.assertAlmostEqual(
                state.progress % 1.0,
                slot["progress"],
                places=7,
            )
            self.assertAlmostEqual(
                state.lateral_offset_m,
                slot["lateral_offset_m"],
            )

        world_poses = {
            (round(state.world_x_m, 3), round(state.world_y_m, 3))
            for state in engine.driver_states.values()
        }
        self.assertEqual(len(world_poses), len(engine.driver_states))
        self.assertNotIn((0.0, 0.0), world_poses)
        self.assertTrue(all(state.physics_frame == 0 for state in engine.driver_states.values()))

    def test_team_mates_reuse_identical_longitudinal_physics_profiles(self) -> None:
        engine = _make_starting_engine(circuit_id=3)

        self.assertIs(
            engine._track_physics_by_driver[1],
            engine._track_physics_by_driver[2],
        )
        self.assertIs(
            engine._vehicle_physics_by_driver_line[1],
            engine._vehicle_physics_by_driver_line[2],
        )

    def test_grid_launch_emits_a_visible_contest_without_battle_carryover(self) -> None:
        engine = _make_starting_engine(circuit_id=3, seed=42)
        launch_events = []

        for _ in range(100):
            launch_events.extend(engine.tick(GAME_TICK_SECONDS))

        self.assertTrue(
            any(event.type in {"attack", "defend"} for event in launch_events)
        )
        self.assertFalse(engine._side_by_side_battles)

    def test_grid_initial_gaps_match_distance_offset(self) -> None:
        engine = _make_engine()
        engine.tick(GAME_TICK_SECONDS)
        engine.tick(GAME_TICK_SECONDS)

        running = sorted(
            (s for s in engine.driver_states.values() if not s.retired),
            key=lambda s: s.position,
        )
        # 그리드 거리 간격이 자연스러운 양(+)의 초기 gap으로 변환된다.
        gaps = [s.gap_to_leader for s in running]
        self.assertEqual(gaps[0], 0.0)
        for gap in gaps[1:]:
            self.assertGreater(gap, 0.0)
        # 뒤쪽 그리드는 앞쪽 그리드보다 리더와의 gap이 크다.
        self.assertGreater(gaps[-1], gaps[1])

    def test_trigger_vsc_sets_phase_and_factor(self) -> None:
        engine = _make_engine()
        events: list = []
        engine._trigger_vsc(events)
        self.assertEqual(engine.race_phase, "vsc")
        self.assertAlmostEqual(engine._phase_lap_time_factor(), 1.4)
        self.assertTrue(any(e.type == "vsc_start" for e in events))
        tick = engine.build_tick_state()
        self.assertEqual(tick.race_phase, "vsc")
        self.assertEqual(tick.race_phase_remaining_seconds, VSC_DURATION_SECONDS)
        self.assertEqual(tick.race_phase_remaining_laps, 0)

    def test_trigger_safety_car_sets_phase_and_factor(self) -> None:
        engine = _make_engine_for_circuit(3)
        events: list = []
        engine._trigger_safety_car(events)
        self.assertEqual(engine.race_phase, "sc")
        self.assertTrue(engine.safety_car)
        self.assertAlmostEqual(
            engine._phase_lap_time_factor(),
            SC_CATCH_UP_FAST_LAP_TIME_FACTOR,
        )
        self.assertTrue(any(e.type == "sc_start" for e in events))
        tick = engine.build_tick_state()
        self.assertEqual(tick.race_phase_remaining_seconds, 0.0)
        self.assertEqual(tick.race_phase_remaining_laps, 0)
        self.assertEqual(tick.safety_car_stage, "deploying")
        self.assertTrue(tick.safety_car_visible)
        self.assertEqual(tick.safety_car_route, "pit")
        self.assertAlmostEqual(tick.safety_car_pit_lane_progress, 0.82)
        self.assertGreaterEqual(engine._sc_cleanup_until, SC_CLEANUP_SECONDS)

    def test_sc_and_vsc_immediately_remove_competitive_maneuvers(self) -> None:
        for phase in ("vsc", "sc"):
            with self.subTest(phase=phase):
                engine = _make_engine_for_circuit(3)
                defender, attacker = sorted(
                    engine.driver_states.values(),
                    key=lambda state: state.position,
                )[:2]
                engine._start_side_by_side_battle(
                    attacker.driver_id,
                    defender.driver_id,
                    ATTACK_LINE_INSIDE,
                    DEFENDER_LINE_RACING,
                    phase="overlap",
                )
                engine._local_trajectory_plans[attacker.driver_id] = (
                    SimpleNamespace()
                )
                engine._local_trajectory_plans[defender.driver_id] = (
                    SimpleNamespace()
                )

                if phase == "vsc":
                    engine._trigger_vsc([])
                else:
                    engine._trigger_safety_car([])

                self.assertEqual(engine.race_phase, phase)
                self.assertEqual(engine._side_by_side_battles, {})
                self.assertNotIn(
                    attacker.driver_id,
                    engine._local_trajectory_plans,
                )
                self.assertNotIn(
                    defender.driver_id,
                    engine._local_trajectory_plans,
                )
                self.assertEqual(
                    engine._battle_event_probability(attacker, defender, 0.4),
                    0.0,
                )
                self.assertFalse(
                    engine._overtake_opportunity_is_viable(attacker, defender)
                )

    def test_sc_unlapping_is_not_a_competitive_overtake_candidate(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, unlapping = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        unlapping.lateral_offset_m = (
            defender.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )
        engine._trigger_safety_car([])
        engine._sc_unlap_driver_ids.add(unlapping.driver_id)

        self.assertFalse(
            engine._overtaking_candidate_allowed(unlapping, defender)
        )
        self.assertTrue(
            engine._physics_v2_passing_authorized(unlapping, defender)
        )
        self.assertEqual(
            engine._battle_event_probability(unlapping, defender, 0.4),
            0.0,
        )

    def test_development_race_control_can_force_and_clear_phase(self) -> None:
        engine = _make_engine()

        vsc_events = engine.set_race_control_phase_for_testing("vsc")
        self.assertEqual(engine.race_phase, "vsc")
        self.assertTrue(any(e.type == "vsc_start" for e in vsc_events))

        sc_events = engine.set_race_control_phase_for_testing("sc")
        self.assertEqual(engine.race_phase, "sc")
        self.assertTrue(any(e.type == "sc_start" for e in sc_events))

        green_events = engine.set_race_control_phase_for_testing("green")
        self.assertEqual(engine.race_phase, "sc")
        self.assertTrue(engine.safety_car)
        self.assertEqual(engine.safety_car_stage, "in_this_lap")
        self.assertTrue(any(e.type == "sc_in_this_lap" for e in green_events))

        with self.assertRaises(ValueError):
            engine.set_race_control_phase_for_testing("red")

    def test_safety_car_upgrades_and_outranks_vsc(self) -> None:
        engine = _make_engine()
        events: list = []
        engine._trigger_vsc(events)
        engine._trigger_safety_car(events)
        self.assertEqual(engine.race_phase, "sc")
        # SC가 활성일 때 VSC 트리거는 무시된다.
        engine._trigger_vsc(events)
        self.assertEqual(engine.race_phase, "sc")

    def test_race_phase_expires_back_to_green(self) -> None:
        engine = _make_engine()
        events: list = []
        engine._trigger_vsc(events)
        engine.race_elapsed = engine._phase_until + 1.0
        end_events: list = []
        engine._tick_race_phase(GAME_TICK_SECONDS, end_events)
        self.assertEqual(engine.race_phase, "green")
        self.assertFalse(engine.safety_car)
        self.assertTrue(any(e.type == "vsc_end" for e in end_events))

    def test_car_stopped_incident_creates_hazard_and_triggers_sc(self) -> None:
        engine = _make_engine()
        target = next(iter(engine.driver_states))
        events: list = []
        engine._apply_incident(
            Incident(
                cause=IncidentCause.MECHANICAL,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=target,
            ),
            events,
        )
        self.assertTrue(engine.driver_states[target].retired)
        self.assertTrue(engine.driver_states[target].hazard_active)
        self.assertEqual(
            engine.driver_states[target].vehicle_status,
            "stopped_on_track",
        )
        self.assertEqual(engine.race_phase, "sc")
        self.assertTrue(any(e.type == "retirement" for e in events))

    def test_development_retirement_uses_real_hazard_and_sc_lifecycle(self) -> None:
        engine = _make_engine()
        target = next(iter(engine.driver_states))

        events = engine.retire_driver_for_testing(target)

        state = engine.driver_states[target]
        self.assertTrue(state.retired)
        self.assertTrue(state.hazard_active)
        self.assertEqual(state.vehicle_status, "stopped_on_track")
        self.assertEqual(engine.race_phase, "sc")
        self.assertTrue(any(event.type == "retirement" for event in events))

    def test_driver_selects_clear_side_and_brakes_for_stopped_hazard(self) -> None:
        engine = _make_engine_for_circuit(3)
        hazard, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._set_state_total_progress(hazard, 1.20)
        engine._set_state_total_progress(
            follower,
            1.20 - 120.0 / engine.track_length_m,
        )
        hazard.lateral_offset_m = 0.0
        follower.lateral_offset_m = 0.0
        follower.speed_kph = 270.0
        for state in engine.driver_states.values():
            if state.driver_id not in {hazard.driver_id, follower.driver_id}:
                engine._set_state_total_progress(state, 0.5)
        engine._apply_incident(
            Incident(
                cause=IncidentCause.MECHANICAL,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=hazard.driver_id,
            ),
            [],
        )
        engine._reset_avoidance_state(follower)
        track_sample = engine._track_physics.at_progress(follower.progress)
        target = engine._physics_v2_target_lateral_offset(
            follower,
            track_sample,
            DEFENDER_LINE_RACING,
            None,
            None,
        )
        required = engine._hazard_required_lateral_clearance_m(
            follower,
            hazard,
        )

        self.assertTrue(follower.avoidance_active)
        self.assertIn(follower.avoidance_side, {"left", "right"})
        self.assertGreaterEqual(
            abs(target - hazard.lateral_offset_m),
            required - 1e-6,
        )
        following = engine._hazard_following_constraint(
            follower,
            DEFENDER_LINE_RACING,
        )
        self.assertIsNotNone(following)
        self.assertTrue(follower.emergency_braking)

    def test_narrow_track_without_escape_corridor_uses_emergency_braking(self) -> None:
        engine = _make_engine_for_circuit(3)
        hazard, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._set_state_total_progress(hazard, 1.20)
        engine._set_state_total_progress(
            follower,
            1.20 - 80.0 / engine.track_length_m,
        )
        hazard.lateral_offset_m = follower.lateral_offset_m = 0.0
        engine._apply_incident(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=hazard.driver_id,
            ),
            [],
        )
        engine._reset_avoidance_state(follower)

        target = engine._hazard_avoidance_target(
            follower,
            SimpleNamespace(left_width_m=1.2, right_width_m=1.2),
            0.0,
        )

        self.assertAlmostEqual(target or 0.0, 0.0, places=6)
        self.assertEqual(follower.avoidance_side, "blocked")
        self.assertTrue(follower.emergency_braking)

    def test_sc_removes_stopped_hazard_when_no_car_is_within_300m(self) -> None:
        engine = _make_engine_for_circuit(3)
        hazard = min(engine.driver_states.values(), key=lambda state: state.position)
        engine._set_state_total_progress(hazard, 1.2)
        engine._apply_incident(
            Incident(
                cause=IncidentCause.MECHANICAL,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=hazard.driver_id,
            ),
            [],
        )
        for state in engine.driver_states.values():
            if not state.retired:
                engine._set_state_total_progress(state, 1.4)
        events: list = []

        engine._tick_stopped_hazard_clearance(events)

        self.assertFalse(hazard.hazard_active)
        self.assertEqual(hazard.vehicle_status, "cleared")
        self.assertTrue(any(event.type == "hazard_cleared" for event in events))

    def test_sc_removes_stopped_hazard_after_original_tail_passes(self) -> None:
        engine = _make_engine_for_circuit(3)
        hazard = min(engine.driver_states.values(), key=lambda state: state.position)
        engine._set_state_total_progress(hazard, 1.2)
        engine._apply_incident(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=hazard.driver_id,
            ),
            [],
        )
        tail_driver_id, target = engine._hazard_tail_targets[hazard.driver_id]
        tail = engine.driver_states[tail_driver_id]
        engine._set_state_total_progress(tail, target)
        events: list = []

        engine._tick_stopped_hazard_clearance(events)

        self.assertFalse(hazard.hazard_active)
        self.assertTrue(any(event.type == "hazard_cleared" for event in events))

    def test_field_avoids_stopped_hazard_without_weaving_or_body_overlap(self) -> None:
        engine = _make_engine_for_circuit(3)
        active = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:6]
        active_ids = {state.driver_id for state in active}
        hazard = active[0]
        for state in engine.driver_states.values():
            if state.driver_id not in active_ids:
                state.retired = True
        engine._set_state_total_progress(hazard, 1.2)
        hazard.lateral_offset_m = 0.0
        for index, state in enumerate(active[1:], start=1):
            engine._set_state_total_progress(
                state,
                1.2 - (100.0 + 35.0 * (index - 1)) / engine.track_length_m,
            )
            state.lateral_offset_m = 0.0
            state.speed_kph = 240.0
        engine._apply_incident(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=hazard.driver_id,
            ),
            [],
        )
        hazard_tick = next(
            position
            for position in engine.build_tick_state().positions
            if position.driver_id == hazard.driver_id
        )
        self.assertTrue(hazard_tick.hazard_active)
        last_side: dict[int, str] = {}
        side_changes = 0
        observed_sides: set[str] = set()

        for _ in range(500):
            engine.tick(GAME_TICK_SECONDS)
            for state in active[1:]:
                if hazard.hazard_active:
                    self.assertIsNone(
                        oriented_body_overlap(
                            engine._body_pose(hazard),
                            engine._body_pose(state),
                        )
                    )
                if not state.avoidance_active or state.avoidance_side not in {
                    "left",
                    "right",
                }:
                    continue
                observed_sides.add(state.avoidance_side)
                previous = last_side.get(state.driver_id)
                if previous is not None and previous != state.avoidance_side:
                    side_changes += 1
                last_side[state.driver_id] = state.avoidance_side
            if not hazard.hazard_active:
                break

        self.assertFalse(hazard.hazard_active)
        self.assertEqual(side_changes, 0)
        self.assertEqual(observed_sides, {"left", "right"})

    def test_dense_field_uses_two_corridors_without_full_stop_wave(self) -> None:
        engine = _make_engine_for_circuit(6)
        active = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:10]
        active_ids = {state.driver_id for state in active}
        hazard = active[0]
        followers = active[1:]
        for state in engine.driver_states.values():
            if state.driver_id not in active_ids:
                state.retired = True
        engine._set_state_total_progress(hazard, 1.2)
        hazard.lateral_offset_m = 0.0
        for index, state in enumerate(followers):
            engine._set_state_total_progress(
                state,
                1.2 - (75.0 + 12.0 * index) / engine.track_length_m,
            )
            state.lateral_offset_m = 0.0
            state.speed_kph = 240.0
        engine._apply_incident(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=hazard.driver_id,
            ),
            [],
        )
        stopped_ids: set[int] = set()
        stop_details: dict[int, tuple] = {}
        observed_sides: set[str] = set()

        for tick_index in range(500):
            engine.tick(GAME_TICK_SECONDS)
            for state in followers:
                if state.speed_kph < 5.0:
                    stopped_ids.add(state.driver_id)
                    stop_details.setdefault(
                        state.driver_id,
                        (
                            tick_index,
                            round(
                                (hazard.total_progress - state.total_progress)
                                * engine.track_length_m,
                                1,
                            ),
                            round(state.lateral_offset_m, 2),
                            state.avoidance_side,
                            state.contact_active,
                            round(state.target_lateral_offset_m, 2),
                            state.handling_state,
                            round(state.lateral_speed_mps, 2),
                        ),
                    )
                if state.avoidance_side in {"left", "right"}:
                    observed_sides.add(state.avoidance_side)
            if not hazard.hazard_active:
                break

        self.assertFalse(hazard.hazard_active)
        self.assertEqual(observed_sides, {"left", "right"})
        self.assertEqual(
            len(stopped_ids),
            0,
            msg=str(
                {
                    "stops": stop_details,
                    "final": [
                        (
                            state.driver_id,
                            round(state.total_progress, 4),
                            round(state.speed_kph, 1),
                            round(state.lateral_offset_m, 2),
                            state.avoidance_side,
                        )
                        for state in followers
                    ],
                }
            ),
        )

    def test_multi_car_crash_cluster_does_not_trap_the_tail_at_spa(self) -> None:
        engine = _make_engine_for_circuit(6)
        active = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )
        first_hazard, second_hazard = active[:2]
        followers = active[2:]
        narrowest = min(
            engine._track_physics.samples,
            key=lambda sample: sample.left_width_m + sample.right_width_m,
        )
        hazard_progress = 1.0 + narrowest.progress
        engine._set_state_total_progress(first_hazard, hazard_progress)
        engine._set_state_total_progress(
            second_hazard,
            hazard_progress - 3.0 / engine.track_length_m,
        )
        first_hazard.lateral_offset_m = -0.9
        second_hazard.lateral_offset_m = 0.9
        for index, state in enumerate(followers):
            engine._set_state_total_progress(
                state,
                hazard_progress
                - (75.0 + 12.0 * index) / engine.track_length_m,
            )
            state.lateral_offset_m = 0.0
            state.speed_kph = 240.0

        engine._apply_incident(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=IncidentSeverity.CRASH,
                primary_driver_id=first_hazard.driver_id,
                secondary_driver_id=second_hazard.driver_id,
            ),
            [],
        )
        maximum_stationary_ticks = 0
        stationary_ticks = {state.driver_id: 0 for state in followers}

        for _ in range(500):
            engine.tick(GAME_TICK_SECONDS)
            for state in followers:
                stationary_ticks[state.driver_id] = (
                    stationary_ticks[state.driver_id] + 1
                    if state.speed_kph < 5.0
                    else 0
                )
                maximum_stationary_ticks = max(
                    maximum_stationary_ticks,
                    stationary_ticks[state.driver_id],
                )
                for hazard in (first_hazard, second_hazard):
                    if hazard.hazard_active:
                        self.assertIsNone(
                            oriented_body_overlap(
                                engine._body_pose(hazard),
                                engine._body_pose(state),
                            )
                        )
            if not engine._active_stopped_hazards():
                break

        self.assertFalse(engine._active_stopped_hazards())
        self.assertLessEqual(maximum_stationary_ticks * GAME_TICK_SECONDS, 1.5)
        self.assertTrue(all(state.speed_kph >= 5.0 for state in followers))

    def test_physically_blocked_hazard_cluster_is_cleared_under_sc(self) -> None:
        engine = _make_engine_for_circuit(6)
        active = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:4]
        hazards = active[:3]
        follower = active[3]
        for state in engine.driver_states.values():
            if state.driver_id not in {item.driver_id for item in active}:
                state.retired = True
        hazard_progress = 1.2
        for hazard, lateral_offset_m in zip(hazards, (-3.0, 0.0, 3.0)):
            engine._set_state_total_progress(hazard, hazard_progress)
            hazard.lateral_offset_m = lateral_offset_m
            hazard.retired = True
            engine._activate_stopped_hazard(
                hazard,
                IncidentCause.COLLISION,
            )
        engine._set_state_total_progress(
            follower,
            hazard_progress - 100.0 / engine.track_length_m,
        )
        engine._trigger_safety_car([])
        events = []

        for _ in range(80):
            events.extend(engine.tick(GAME_TICK_SECONDS))
            if not engine._active_stopped_hazards():
                break

        self.assertFalse(engine._active_stopped_hazards())
        self.assertTrue(any(event.type == "hazard_cleared" for event in events))

    def test_crash_incident_triggers_safety_car_and_retires_both(self) -> None:
        engine = _make_engine()
        ids = list(engine.driver_states)
        primary, secondary = ids[3], ids[4]
        events: list = []
        engine._apply_incident(
            Incident(
                cause=IncidentCause.COLLISION,
                severity=IncidentSeverity.CRASH,
                primary_driver_id=primary,
                secondary_driver_id=secondary,
            ),
            events,
        )
        self.assertTrue(engine.driver_states[primary].retired)
        self.assertTrue(engine.driver_states[secondary].retired)
        self.assertEqual(engine.race_phase, "sc")
        self.assertTrue(engine.safety_car)

    def test_minor_incident_costs_time_without_retirement(self) -> None:
        engine = _make_engine()
        target = next(iter(engine.driver_states))
        before = engine.driver_states[target].total_time
        events: list = []
        engine._apply_incident(
            Incident(
                cause=IncidentCause.DRIVER_ERROR,
                severity=IncidentSeverity.MINOR,
                primary_driver_id=target,
            ),
            events,
        )
        self.assertFalse(engine.driver_states[target].retired)
        self.assertEqual(engine.race_phase, "green")
        self.assertGreater(engine.driver_states[target].total_time, before)
        self.assertTrue(any(e.type == "incident" for e in events))

    def test_safety_car_forms_queue_without_instantly_moving_field(self) -> None:
        engine = _make_engine()
        for _ in range(120):
            engine.tick(GAME_TICK_SECONDS)

        # Exercise a field spread over roughly three quarters of a lap.
        spread_field = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )
        leader_total = spread_field[0].total_progress
        for index, state in enumerate(spread_field):
            engine._set_state_total_progress(state, leader_total - index * 0.04)

        progress_before = {
            state.driver_id: state.total_progress for state in engine.driver_states.values()
        }
        order_before = [
            s.driver_id for s in sorted(engine.driver_states.values(), key=lambda s: s.position)
        ]
        engine._trigger_safety_car([])
        deployed_at = engine.race_elapsed
        maximum_catch_up_speed_kph = 0.0

        self.assertEqual(
            progress_before,
            {state.driver_id: state.total_progress for state in engine.driver_states.values()},
        )

        engine._sc_cleanup_until = float("inf")
        # Slot-based catch-up preserves the green-physics envelope, so a field
        # deliberately spread over most of a lap needs more than three nominal
        # laps to converge without teleporting or forcing a low global cap.
        for _ in range(6000):
            engine.tick(GAME_TICK_SECONDS)
            if engine.race_elapsed - deployed_at >= 3.0:
                maximum_catch_up_speed_kph = max(
                    maximum_catch_up_speed_kph,
                    *(state.speed_kph for state in spread_field),
                )
            if engine._safety_car_queue_formed:
                break
        self.assertTrue(engine._safety_car_queue_formed)
        self.assertLessEqual(
            engine.race_elapsed - deployed_at,
            engine.circuit.base_lap_time * 8.0,
        )
        self.assertLessEqual(maximum_catch_up_speed_kph, 300.1)

        running = sorted(
            (
                s
                for s in engine.driver_states.values()
                if not s.retired and not s.finished and not s.in_pit
            ),
            key=lambda s: s.position,
        )
        max_gap = engine._sc_max_gap_progress()
        for ahead, behind in zip(running, running[1:]):
            gap = ahead.total_progress - behind.total_progress
            self.assertLessEqual(gap, max_gap + 1e-6)
        for first_index, first in enumerate(running):
            for second in running[first_index + 1:]:
                self.assertIsNone(
                    oriented_body_overlap(
                        engine._body_pose(first),
                        engine._body_pose(second),
                    )
                )

        # Once joined, cars continue closing gently toward the seven-car target.
        for _ in range(240):
            engine.tick(GAME_TICK_SECONDS)
        running = sorted(
            (
                s
                for s in engine.driver_states.values()
                if not s.retired and not s.finished and not s.in_pit
            ),
            key=lambda s: s.position,
        )
        target_gap_m = SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
        for ahead, behind in zip(running, running[1:]):
            gap_m = (ahead.total_progress - behind.total_progress) * engine.track_length_m
            self.assertLessEqual(
                gap_m,
                max(
                    target_gap_m + SC_CAR_LENGTH_M,
                    SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS * 2.0,
                ),
            )

        order_after = [
            s.driver_id for s in sorted(engine.driver_states.values(), key=lambda s: s.position)
        ]
        self.assertEqual(order_before, order_after)

    def test_safety_car_queue_gap_distances_and_variable_catch_up_pace(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        running = sorted(engine.driver_states.values(), key=lambda state: state.position)
        leader = running[0]

        self.assertAlmostEqual(
            engine._sc_target_gap_progress() * engine.track_length_m,
            SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS,
        )
        self.assertAlmostEqual(
            engine._sc_max_gap_progress() * engine.track_length_m,
            SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS,
        )

        engine.safety_car_stage = "collecting"
        engine._safety_car_total_progress = leader.total_progress + 0.5
        far_factor = engine._phase_lap_time_factor(leader)
        engine._safety_car_total_progress = (
            leader.total_progress + engine._sc_max_gap_progress()
        )
        near_factor = engine._phase_lap_time_factor(leader)

        self.assertAlmostEqual(far_factor, SC_CATCH_UP_FAST_LAP_TIME_FACTOR)
        self.assertAlmostEqual(near_factor, SC_CATCH_UP_NEAR_LAP_TIME_FACTOR)
        self.assertLess(far_factor, near_factor)

        engine._sc_caught_driver_ids.add(leader.driver_id)
        engine._safety_car_total_progress = (
            leader.total_progress + engine._sc_max_gap_progress() + 0.01
        )
        engine._sync_safety_car_queue([])
        self.assertNotIn(leader.driver_id, engine._sc_caught_driver_ids)

    def test_safety_car_queue_sync_does_not_snap_caught_car_position(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader = min(engine.driver_states.values(), key=lambda state: state.position)
        engine.safety_car_stage = "queued"
        engine._sc_caught_driver_ids.add(leader.driver_id)
        engine._safety_car_total_progress = (
            leader.total_progress + engine._sc_target_gap_progress() * 0.5
        )
        before = leader.total_progress

        engine._sync_safety_car_queue([])

        self.assertEqual(leader.total_progress, before)

    def test_caught_leader_physically_follows_safety_car(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader = min(engine.driver_states.values(), key=lambda state: state.position)
        engine.safety_car_stage = "queued"
        engine._sc_caught_driver_ids.add(leader.driver_id)
        engine._safety_car_total_progress = (
            leader.total_progress + engine._sc_target_gap_progress()
        )
        engine._safety_car_progress_rate = 1.0 / (
            engine.circuit.base_lap_time * 1.8
        )
        engine._safety_car_speed_mps = leader.speed_kph / 3.6

        following = engine._safety_car_following_constraint(
            leader,
            None,
            DRIVING_LINE_RACING,
            GAME_TICK_SECONDS,
        )

        self.assertIsNotNone(following)
        assert following is not None
        self.assertAlmostEqual(
            following.desired_gap_m,
            SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS,
        )
        self.assertGreater(
            following.leader_end_distance_m,
            following.leader_distance_m,
        )
        modifiers = engine._physics_v2_modifiers(leader)
        self.assertIsNotNone(modifiers.maximum_speed_mps)
        assert modifiers.maximum_speed_mps is not None
        self.assertGreater(modifiers.maximum_speed_mps, 0.0)
        self.assertLessEqual(
            modifiers.maximum_speed_mps,
            SC_CAUGHT_MAX_SPEED_KPH / 3.6,
        )

    def test_sc_leader_acquisition_wait_does_not_lock_leader_to_five_kph(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader = min(engine.driver_states.values(), key=lambda state: state.position)
        engine.safety_car_stage = "collecting"
        engine._safety_car_route = "track"
        engine._safety_car_speed_mps = 80.0 / 3.6
        engine._safety_car_progress_rate = (
            engine._safety_car_speed_mps / engine.track_length_m
        )
        engine._safety_car_total_progress = (
            leader.total_progress + 700.0 / engine.track_length_m
        )
        leader.speed_kph = 300.0

        following = engine._safety_car_following_constraint(
            leader,
            None,
            DRIVING_LINE_RACING,
            GAME_TICK_SECONDS,
        )
        speed_cap = engine._race_control_speed_cap_mps(leader)

        self.assertNotIn(leader.driver_id, engine._sc_caught_driver_ids)
        self.assertTrue(engine._sc_leader_acquisition_wait_active())
        self.assertIsNone(following)
        self.assertIsNotNone(speed_cap)
        assert speed_cap is not None
        self.assertGreater(speed_cap, 5.0 / 3.6)
        self.assertLessEqual(speed_cap, leader.speed_kph / 3.6)

    def test_sc_queue_join_waits_for_safe_relative_speed(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader = min(engine.driver_states.values(), key=lambda state: state.position)
        engine.safety_car_stage = "collecting"
        engine._safety_car_speed_mps = 80.0 / 3.6
        engine._safety_car_total_progress = (
            leader.total_progress + 40.0 / engine.track_length_m
        )
        leader.speed_kph = 200.0

        engine._sync_safety_car_queue([])
        self.assertNotIn(leader.driver_id, engine._sc_caught_driver_ids)

        leader.speed_kph = 79.5 + SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH
        engine._sync_safety_car_queue([])
        self.assertIn(leader.driver_id, engine._sc_caught_driver_ids)

    def test_sc_queue_stability_debounces_corner_speed_spike(self) -> None:
        engine = _make_engine()
        leader, follower = _prepare_two_car_sc_queue(engine)
        control = engine._sc_driver_states[follower.driver_id]

        self.assertEqual(control.mode, "STABLE")
        stable_before_spike = control.stable_seconds

        # A short corner-like speed excursion is larger than the entry
        # threshold, but must not reset an already stable pair immediately.
        follower.speed_kph = (
            leader.speed_kph + SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH + 20.0
        )
        for _ in range(5):
            engine.race_elapsed += 0.1
            engine._sync_safety_car_queue([])

        self.assertEqual(control.mode, "STABLE")
        self.assertGreater(
            abs(control.relative_speed_filtered_kph or 0.0),
            SC_QUEUE_STABLE_RELATIVE_SPEED_KPH,
        )
        self.assertLess(
            control.relative_speed_violation_seconds,
            SC_QUEUE_STABLE_RELATIVE_SPEED_GRACE_SECONDS,
        )
        self.assertGreater(control.stable_seconds, stable_before_spike)
        diagnostic_driver = next(
            item
            for item in engine.safety_car_diagnostic_snapshot()["drivers"]
            if item["driver_id"] == follower.driver_id
        )
        self.assertIn("relative_speed_kph", diagnostic_driver)
        self.assertIn("relative_speed_filtered_kph", diagnostic_driver)
        self.assertIn("relative_speed_violation_seconds", diagnostic_driver)

    def test_sc_queue_stability_releases_after_sustained_speed_separation(self) -> None:
        engine = _make_engine()
        leader, follower = _prepare_two_car_sc_queue(engine)
        control = engine._sc_driver_states[follower.driver_id]
        follower.speed_kph = (
            leader.speed_kph + SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH + 20.0
        )

        for _ in range(20):
            engine.race_elapsed += 0.1
            engine._sync_safety_car_queue([])

        self.assertEqual(control.mode, "FOLLOWING")
        self.assertEqual(control.stable_seconds, 0.0)

    def test_sc_queue_stability_still_breaks_on_real_gap_excursion(self) -> None:
        engine = _make_engine()
        leader, follower = _prepare_two_car_sc_queue(engine)
        follower_gap_m = SC_CAR_LENGTH_M * 12.0
        engine._set_state_total_progress(
            follower,
            leader.total_progress - follower_gap_m / engine.track_length_m,
        )
        engine.race_elapsed += 0.1
        engine._sync_safety_car_queue([])

        self.assertEqual(
            engine._sc_driver_states[follower.driver_id].mode,
            "APPROACHING",
        )

    def test_uncaught_sc_car_outside_join_gap_is_not_clamped_to_queue_speed(
        self,
    ) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {leader.driver_id, follower.driver_id}:
                state.retired = True

        engine.safety_car_stage = "collecting"
        queue_join_gap_m = (
            SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS
        )
        engine._set_state_total_progress(leader, 1.2)
        engine._set_state_total_progress(
            follower,
            leader.total_progress
            - (queue_join_gap_m + 1.0) / engine.track_length_m,
        )
        engine._safety_car_total_progress = (
            leader.total_progress
            + SC_CAR_LENGTH_M
            * SC_QUEUE_TARGET_CAR_LENGTHS
            / engine.track_length_m
        )
        engine._sc_caught_driver_ids.add(leader.driver_id)
        leader.speed_kph = 65.0
        follower.speed_kph = 250.0

        speed_cap = engine._race_control_speed_cap_mps(follower)

        self.assertNotIn(follower.driver_id, engine._sc_caught_driver_ids)
        self.assertIsNotNone(speed_cap)
        assert speed_cap is not None
        self.assertGreater(speed_cap, leader.speed_kph / 3.6)

    def test_caught_sc_queue_car_matches_predecessor_at_target_gap(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {leader.driver_id, follower.driver_id}:
                state.retired = True
        engine.safety_car_stage = "queued"
        engine._sc_caught_driver_ids.update({leader.driver_id, follower.driver_id})
        engine._set_state_total_progress(leader, 1.2)
        engine._set_state_total_progress(
            follower,
            leader.total_progress
            - SC_CAR_LENGTH_M
            * SC_QUEUE_TARGET_CAR_LENGTHS
            / engine.track_length_m,
        )
        leader.speed_kph = 100.0
        follower.speed_kph = 160.0
        snapshot = {
            leader.driver_id: (leader.total_progress, leader.speed_kph / 3.6),
            follower.driver_id: (follower.total_progress, follower.speed_kph / 3.6),
        }

        following = engine._physics_v2_following_constraint(
            follower,
            leader,
            snapshot,
            DRIVING_LINE_RACING,
            GAME_TICK_SECONDS,
        )
        speed_cap = engine._race_control_speed_cap_mps(follower)

        self.assertIsNotNone(following)
        assert following is not None
        self.assertAlmostEqual(
            following.desired_gap_m,
            SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS,
        )
        self.assertIsNotNone(speed_cap)
        assert speed_cap is not None
        self.assertLessEqual(speed_cap, leader.speed_kph / 3.6 + 1e-6)

    def test_sc_queue_speed_control_propagates_to_rear_field(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        leader, middle, rear, distant = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:4]
        for state in engine.driver_states.values():
            if state.driver_id not in {
                leader.driver_id,
                middle.driver_id,
                rear.driver_id,
                distant.driver_id,
            }:
                state.retired = True
        engine.safety_car_stage = "collecting"
        engine._safety_car_speed_mps = 80.0 / 3.6
        engine._set_state_total_progress(leader, 1.2)
        engine._set_state_total_progress(
            middle,
            leader.total_progress - 100.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            rear,
            middle.total_progress - 100.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            distant,
            rear.total_progress - 500.0 / engine.track_length_m,
        )
        engine._safety_car_total_progress = (
            leader.total_progress + 35.0 / engine.track_length_m
        )
        engine._sc_caught_driver_ids.add(leader.driver_id)
        leader.speed_kph = 80.0
        middle.speed_kph = 180.0
        rear.speed_kph = 300.0
        distant.speed_kph = 300.0

        self.assertTrue(engine._sc_queue_control_reaches(middle))
        self.assertTrue(engine._sc_queue_control_reaches(rear))
        self.assertFalse(engine._sc_queue_control_reaches(distant))
        rear_cap = engine._race_control_speed_cap_mps(rear)
        self.assertIsNotNone(rear_cap)
        assert rear_cap is not None
        self.assertLess(rear_cap, rear.speed_kph / 3.6)

    def test_sc_uncaught_rear_train_receives_catch_up_acceleration_same_tick(
        self,
    ) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        running = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )
        caught = running[:13]
        rear_train = running[13:]

        engine.safety_car_stage = "collecting"
        engine._safety_car_speed_mps = 65.0 / 3.6
        target_gap_m = SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
        engine._set_state_total_progress(running[0], 1.2)
        for index, state in enumerate(running[1:], start=1):
            gap_m = 100.0 if index == len(caught) else target_gap_m
            engine._set_state_total_progress(
                state,
                running[index - 1].total_progress
                - gap_m / engine.track_length_m,
            )
        engine._safety_car_total_progress = (
            running[0].total_progress + target_gap_m / engine.track_length_m
        )
        engine._sc_caught_driver_ids.update(
            state.driver_id for state in caught
        )
        for state in running:
            state.speed_kph = 65.0

        rear_caps = [
            engine._race_control_speed_cap_mps(state)
            for state in rear_train
        ]

        self.assertTrue(all(cap is not None for cap in rear_caps))
        self.assertTrue(
            all(
                cap is not None and cap > 65.0 / 3.6
                for cap in rear_caps
            )
        )
        self.assertTrue(
            all(
                following_cap is not None
                and leading_cap is not None
                and following_cap >= leading_cap - 1e-9
                for leading_cap, following_cap
                in zip(rear_caps, rear_caps[1:])
            )
        )

    def test_safety_car_queue_does_not_catch_car_ahead_of_sc(self) -> None:
        engine = _make_engine()
        engine._trigger_safety_car([])
        engine.safety_car_stage = "collecting"
        leader = min(engine.driver_states.values(), key=lambda state: state.position)
        engine._safety_car_total_progress = leader.total_progress - 0.01

        engine._sync_safety_car_queue([])

        self.assertNotIn(leader.driver_id, engine._sc_caught_driver_ids)

    def test_final_gap_resolution_reconciles_visual_progress_rate(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        start = state.total_progress
        state.total_progress = start + 0.001
        state.progress = state.total_progress % 1.0
        state.speed_kph = 250.0
        engine._progress_rate[state.driver_id] = 0.03

        engine._reconcile_progress_rates(
            {state.driver_id: (start, state.speed_kph / 3.6)},
            0.1,
        )

        self.assertAlmostEqual(engine._progress_rate[state.driver_id], 0.01)
        self.assertLess(state.speed_kph, 250.0)

    def test_safety_car_keeps_order_frozen(self) -> None:
        engine = _make_engine()
        for _ in range(120):
            engine.tick(GAME_TICK_SECONDS)
        engine._trigger_safety_car([])
        order_at_deploy = [
            s.driver_id for s in sorted(engine.driver_states.values(), key=lambda s: s.position)
        ]
        for _ in range(60):
            engine.tick(GAME_TICK_SECONDS)
        order_under_sc = [
            s.driver_id for s in sorted(engine.driver_states.values(), key=lambda s: s.position)
        ]
        self.assertEqual(order_at_deploy, order_under_sc)

    def test_safety_car_queue_plan_is_not_reordered_by_live_physical_order(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])
        planned_ids = list(engine._sc_running_order)
        first = engine.driver_states[planned_ids[0]]
        second = engine.driver_states[planned_ids[1]]
        engine._set_state_total_progress(first, 1.10)
        engine._set_state_total_progress(second, 1.12)

        engine._sync_safety_car_queue([])

        self.assertEqual(engine._sc_running_order, planned_ids)
        self.assertEqual(
            [slot.driver_id for slot in engine._sc_queue_plan.slots],
            planned_ids,
        )
        self.assertEqual(engine._sc_queue_predecessor(second), first)

    def test_safety_car_slot_cap_uses_gap_error_for_catch_up(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])
        leader, follower = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine.safety_car_stage = "collecting"
        engine._set_state_total_progress(leader, 1.2)
        engine._set_state_total_progress(
            follower,
            leader.total_progress
            - (SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS + 100.0)
            / engine.track_length_m,
        )
        engine._safety_car_total_progress = (
            leader.total_progress
            + SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS / engine.track_length_m
        )
        leader.speed_kph = 65.0
        follower.speed_kph = 65.0

        cap = engine._race_control_speed_cap_mps(follower)

        self.assertIsNotNone(cap)
        assert cap is not None
        self.assertGreater(cap, leader.speed_kph / 3.6)
        self.assertLess(cap, SC_CATCH_UP_MAX_SPEED_KPH / 3.6)

    def test_safety_car_pit_exit_order_is_provisional_until_sc2_then_committed(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])

        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        pitting_car = ordered[0]
        next_on_track = ordered[1]
        third_on_track = ordered[2]
        for state in ordered[3:]:
            state.retired = True
        sc2 = engine._safety_car_line_2_progress()
        self.assertIsNotNone(sc2)
        assert sc2 is not None

        engine._remove_from_sc_running_order(pitting_car.driver_id)
        pitting_car.in_pit = True
        engine._set_state_total_progress(next_on_track, 1.0 + sc2 - 0.004)
        engine._set_state_total_progress(third_on_track, 1.0 + sc2 - 0.008)
        engine._set_state_total_progress(pitting_car, 1.0 + sc2 - 0.006)
        engine._update_positions()

        self.assertLess(next_on_track.position, pitting_car.position)
        self.assertLess(pitting_car.position, third_on_track.position)

        pitting_car.in_pit = False
        engine._set_state_total_progress(pitting_car, 1.0 + sc2 - 0.001)
        engine._begin_sc_pit_exit_ordering(pitting_car)
        engine._update_positions()

        self.assertEqual(pitting_car.position, 1)
        self.assertIn(pitting_car.driver_id, engine._sc_pit_exit_order_targets)
        self.assertNotIn(pitting_car.driver_id, engine._sc_running_order)

        engine._set_state_total_progress(pitting_car, 1.0 + sc2 + 0.0001)
        engine._update_positions()

        self.assertNotIn(pitting_car.driver_id, engine._sc_pit_exit_order_targets)
        self.assertEqual(
            engine._sc_running_order,
            [
                pitting_car.driver_id,
                next_on_track.driver_id,
                third_on_track.driver_id,
            ],
        )
        self.assertEqual(
            engine._sc_queue_predecessor(next_on_track),
            pitting_car,
        )

    def test_safety_car_pit_exit_behind_at_sc2_loses_place_and_order_stays_frozen(
        self,
    ) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])

        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        pitting_car = ordered[0]
        next_on_track = ordered[1]
        third_on_track = ordered[2]
        for state in ordered[3:]:
            state.retired = True
        sc2 = engine._safety_car_line_2_progress()
        self.assertIsNotNone(sc2)
        assert sc2 is not None

        engine._remove_from_sc_running_order(pitting_car.driver_id)
        pitting_car.in_pit = False
        engine._set_state_total_progress(next_on_track, 1.0 + sc2 + 0.003)
        engine._set_state_total_progress(pitting_car, 1.0 + sc2 + 0.001)
        engine._set_state_total_progress(third_on_track, 1.0 + sc2 - 0.003)
        engine._begin_sc_pit_exit_ordering(pitting_car)
        engine._update_positions()

        committed_order = [
            next_on_track.driver_id,
            pitting_car.driver_id,
            third_on_track.driver_id,
        ]
        self.assertEqual(engine._sc_running_order, committed_order)
        self.assertEqual(pitting_car.position, 2)

        # A later numerical crossing cannot become an on-track pass under SC.
        engine._set_state_total_progress(
            pitting_car,
            next_on_track.total_progress + 0.002,
        )
        engine._update_positions()

        self.assertEqual(engine._sc_running_order, committed_order)
        self.assertEqual(next_on_track.position, 1)
        self.assertEqual(pitting_car.position, 2)

    def test_safety_car_commits_two_pit_out_cars_in_physical_sc2_order(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])

        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        pit_out_a, pit_out_b, track_a, track_b = ordered[:4]
        for state in ordered[4:]:
            state.retired = True
        sc2 = engine._safety_car_line_2_progress()
        self.assertIsNotNone(sc2)
        assert sc2 is not None

        for state in (pit_out_a, pit_out_b):
            engine._remove_from_sc_running_order(state.driver_id)
            state.in_pit = False
        engine._set_state_total_progress(track_a, 1.0 + sc2 + 0.004)
        engine._set_state_total_progress(pit_out_a, 1.0 + sc2 + 0.003)
        engine._set_state_total_progress(pit_out_b, 1.0 + sc2 + 0.002)
        engine._set_state_total_progress(track_b, 1.0 + sc2 + 0.001)
        engine._begin_sc_pit_exit_ordering(pit_out_a)
        engine._begin_sc_pit_exit_ordering(pit_out_b)

        engine._update_positions()

        expected = [
            track_a.driver_id,
            pit_out_a.driver_id,
            pit_out_b.driver_id,
            track_b.driver_id,
        ]
        self.assertEqual(engine._sc_running_order, expected)
        self.assertEqual(
            [
                state.driver_id
                for state in sorted(
                    (track_a, pit_out_a, pit_out_b, track_b),
                    key=lambda state: state.position,
                )
            ],
            expected,
        )
        self.assertEqual(engine._sc_queue_predecessor(pit_out_b), pit_out_a)
        self.assertEqual(engine._sc_queue_predecessor(track_b), pit_out_b)

    def test_safety_car_progress_does_not_jump_back_when_leader_pits(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])
        engine._tick_race_phase(3.0, [])
        progress_before = engine._safety_car_total_progress
        self.assertIsNotNone(progress_before)
        self.assertEqual(engine.safety_car_stage, "collecting")

        leader = engine._on_track_leader()
        self.assertIsNotNone(leader)
        leader.in_pit = True
        engine._update_positions()
        replacement_leader = engine._on_track_leader()
        self.assertIsNotNone(replacement_leader)
        self.assertIsNot(leader, replacement_leader)

        engine._tick_race_phase(GAME_TICK_SECONDS, [])
        self.assertGreater(engine._safety_car_total_progress, progress_before)
        self.assertGreater(engine._safety_car_progress_rate, 0.0)
        self.assertAlmostEqual(
            engine.build_tick_state().safety_car_progress,
            engine._safety_car_total_progress % 1.0,
            places=6,
        )

    def test_safety_car_deploys_from_configured_pit_exit_on_real_circuits(self) -> None:
        for circuit_id in _real_circuit_ids():
            engine = _make_engine_for_circuit(circuit_id)
            leader = engine._on_track_leader()
            exit_progress = engine.circuit.pit_lane.exit_progress
            self.assertIsNotNone(leader)
            self.assertIsNotNone(exit_progress)

            engine._trigger_safety_car([])
            expected_total = engine._total_progress_at_or_after(
                leader.total_progress,
                exit_progress,
            )
            self.assertAlmostEqual(engine._safety_car_total_progress, expected_total)
            self.assertEqual(engine._safety_car_route, "pit")
            self.assertEqual(engine.safety_car_stage, "deploying")

            engine._tick_race_phase(3.0, [])
            self.assertEqual(engine._safety_car_route, "track")
            self.assertEqual(engine.safety_car_stage, "collecting")
            self.assertAlmostEqual(
                engine.build_tick_state().safety_car_progress,
                exit_progress,
                places=4,
            )

    def test_safety_car_leader_loses_places_during_complete_pit_cycle(self) -> None:
        engine = _make_engine_for_circuit(3)
        leader = min(engine.driver_states.values(), key=lambda state: state.position)
        entry = engine.circuit.pit_lane.entry_progress
        self.assertIsNotNone(entry)
        for index, state in enumerate(
            sorted(engine.driver_states.values(), key=lambda item: item.position)
        ):
            engine._set_state_total_progress(state, entry - 0.001 - index * 0.004)
        leader.pit_request = TireCompound.HARD
        engine._trigger_safety_car([])

        entered_pit = False
        positions_while_in_pit: list[int] = []
        for _ in range(2000):
            engine.tick(GAME_TICK_SECONDS)
            if leader.in_pit:
                entered_pit = True
                positions_while_in_pit.append(leader.position)
            if entered_pit and not leader.in_pit:
                break

        self.assertTrue(entered_pit)
        self.assertEqual(leader.pit_count, 1)
        self.assertGreater(max(positions_while_in_pit), 1)
        self.assertGreater(leader.position, 1)
        self.assertEqual(engine.race_phase, "sc")

    def test_safety_car_pits_then_waits_for_restart_line_before_green(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])
        engine._tick_race_phase(3.0, [])
        events: list = []
        engine._begin_sc_in_this_lap(events)
        self.assertEqual(engine.safety_car_stage, "in_this_lap")
        self.assertTrue(any(event.type == "sc_in_this_lap" for event in events))

        engine._safety_car_total_progress = engine._sc_withdraw_target - 0.001
        engine._tick_race_phase(1.0, events)
        self.assertEqual(engine.safety_car_stage, "restart")
        self.assertEqual(engine.race_phase, "sc")
        self.assertEqual(engine._safety_car_route, "pit")

        leader = engine._on_track_leader()
        self.assertIsNotNone(leader)
        leader.total_progress = engine._sc_restart_target
        engine._tick_safety_car_after_cars(events)
        self.assertEqual(engine.race_phase, "green")
        self.assertFalse(engine.safety_car)
        self.assertTrue(any(event.type == "sc_end" for event in events))

    def test_safety_car_restart_moves_after_queue_stability_is_lost(self) -> None:
        """A withdrawn SC must not become a stationary speed reference."""
        engine = _make_engine_for_circuit(4)
        for state in sorted(
            engine.driver_states.values(),
            key=lambda item: item.position,
        )[-2:]:
            state.retired = True
        engine._trigger_safety_car([])
        engine._tick_race_phase(3.0, [])
        events: list = []
        engine._begin_sc_in_this_lap(events)

        engine._safety_car_total_progress = engine._sc_withdraw_target - 0.001
        engine._tick_race_phase(1.0, events)
        self.assertEqual(engine.safety_car_stage, "restart")

        running = engine._sc_ordered_on_track_states(include_pending=False)
        self.assertEqual(len(running), 18)
        leader = running[0]
        engine._safety_car_queue_formed = False
        engine._sc_queue_stable_seconds = 0.0
        engine._sc_caught_driver_ids = {
            state.driver_id for state in running[:4]
        }
        for state in running:
            state.speed_kph = 0.0

        # Reproduce the product failure away from the confirmation tolerance:
        # the leader must physically travel to the restart line instead of
        # being teleported there by the test.
        initial_leader_progress = leader.total_progress
        engine._sc_restart_target = initial_leader_progress + 0.12
        engine._sc_restart_accel_progress = initial_leader_progress + 0.06

        cap_mps = engine._race_control_speed_cap_mps(leader)
        self.assertIsNotNone(cap_mps)
        self.assertGreater(cap_mps, 0.0)

        maximum_ticks = round(
            engine.circuit.base_lap_time * 2.0 / GAME_TICK_SECONDS
        )
        maximum_simultaneously_stopped_seconds = 0.0
        current_simultaneously_stopped_seconds = 0.0
        for _ in range(maximum_ticks):
            engine.tick(GAME_TICK_SECONDS)
            if all(state.speed_kph < 1.0 for state in running):
                current_simultaneously_stopped_seconds += GAME_TICK_SECONDS
                maximum_simultaneously_stopped_seconds = max(
                    maximum_simultaneously_stopped_seconds,
                    current_simultaneously_stopped_seconds,
                )
            else:
                current_simultaneously_stopped_seconds = 0.0
            if engine.race_phase == "green":
                break

        self.assertGreater(leader.total_progress, initial_leader_progress)
        self.assertLess(maximum_simultaneously_stopped_seconds, 2.0)
        self.assertEqual(engine.race_phase, "green")
        self.assertTrue(any(event.type == "sc_pit" for event in events))

    def test_safety_car_completes_full_physical_lifecycle(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])
        engine._sc_cleanup_until = 0.0
        event_types: set[str] = set()
        stages = {engine.safety_car_stage}

        for _ in range(5000):
            events = engine.tick(GAME_TICK_SECONDS)
            event_types.update(event.type for event in events)
            stages.add(engine.safety_car_stage)
            if engine.race_phase == "green":
                break

        self.assertEqual(engine.race_phase, "green")
        self.assertIn("deploying", stages)
        self.assertIn("collecting", stages)
        self.assertIn("restart", stages)
        self.assertIn("sc_track_join", event_types)
        self.assertIn("sc_queue", event_types)
        self.assertIn("sc_in_this_lap", event_types)
        self.assertIn("sc_pit", event_types)
        self.assertIn("sc_end", event_types)

    def test_safety_car_opens_and_closes_pit_window(self) -> None:
        engine = _make_engine()
        self.assertFalse(engine.pit_window_open)
        events: list = []
        engine._trigger_safety_car(events)
        self.assertTrue(engine.pit_window_open)
        self.assertTrue(engine.build_tick_state().pit_window_open)
        self.assertTrue(any(e.type == "pit_window" for e in events))

        engine._finish_race_phase("sc", [])
        self.assertFalse(engine.pit_window_open)
        self.assertFalse(engine.build_tick_state().pit_window_open)

    def test_vsc_does_not_open_pit_window(self) -> None:
        engine = _make_engine()
        engine._trigger_vsc([])
        self.assertFalse(engine.pit_window_open)

    def test_safety_car_lets_ai_take_free_pit(self) -> None:
        engine = _make_engine()
        for _ in range(120):
            engine.tick(GAME_TICK_SECONDS)
        if engine.race_phase != "green":
            engine._finish_race_phase(engine.race_phase, [])
        # 모든 AI 타이어를 충분히 닳게 만들어 SC 피트 자격을 부여한다.
        for driver_id, state in engine.driver_states.items():
            if driver_id in engine.player_driver_ids:
                continue
            state.tire_usage = 30.0
        # 확정적으로 피트를 굴리도록 확률을 1.0으로.
        import simulation.safety_car as safety_car_mod

        original = safety_car_mod.SC_PIT_PROBABILITY
        safety_car_mod.SC_PIT_PROBABILITY = 1.0
        try:
            engine._trigger_safety_car([])
        finally:
            safety_car_mod.SC_PIT_PROBABILITY = original

        pit_requests = [
            s
            for did, s in engine.driver_states.items()
            if did not in engine.player_driver_ids
            and not s.retired
            and not s.finished
            and s.pit_request is not None
        ]
        self.assertGreater(len(pit_requests), 0)

    def test_safety_car_unlapping_driver_physically_completes_extra_lap(self) -> None:
        engine = _make_engine_for_circuit(3)
        for _ in range(500):
            engine.tick(GAME_TICK_SECONDS)
        for state in engine.driver_states.values():
            if state.hazard_active:
                state.hazard_active = False
                state.vehicle_status = "cleared"
        engine._hazard_tail_targets.clear()
        if engine.race_phase != "green":
            engine._finish_race_phase(engine.race_phase, [])
        leader = engine._on_track_leader()
        backmarker = max(
            (
                state
                for state in engine.driver_states.values()
                if not state.retired and not state.finished and not state.in_pit
            ),
            key=lambda state: state.position,
        )
        self.assertIsNotNone(leader)
        engine._set_state_total_progress(backmarker, leader.total_progress - 1.0)

        engine._trigger_safety_car([])
        engine._tick_race_phase(3.0, [])
        events: list = []
        engine._start_sc_unlapping([backmarker], events)
        unlap_target = engine._sc_unlap_targets[backmarker.driver_id]
        max_observed = backmarker.total_progress

        for _ in range(3000):
            tick_events = engine.tick(GAME_TICK_SECONDS)
            events.extend(tick_events)
            max_observed = max(max_observed, backmarker.total_progress)
            if any(event.type == "unlap" for event in tick_events):
                break

        self.assertGreaterEqual(max_observed, unlap_target - 1e-6)
        self.assertTrue(any(event.type == "unlap" for event in events))
        self.assertEqual(engine.safety_car_stage, "in_this_lap")

    def test_safety_car_slows_field(self) -> None:
        engine = _make_engine()
        for _ in range(10):
            engine.tick(GAME_TICK_SECONDS)
        green_leader = max(s.total_progress for s in engine.driver_states.values())
        engine._trigger_safety_car([])
        before = max(s.total_progress for s in engine.driver_states.values())
        for _ in range(10):
            engine.tick(GAME_TICK_SECONDS)
        sc_advance = max(s.total_progress for s in engine.driver_states.values()) - before
        # 세이프티카 구간에서는 같은 시간 동안 전진량이 평상시보다 작다.
        self.assertLess(sc_advance, green_leader)

    def test_sc_catch_up_never_raises_physical_corner_target_above_green(self) -> None:
        engine = _make_engine_for_circuit(3)
        state = engine.driver_states[1]
        for other in engine.driver_states.values():
            if other.driver_id != state.driver_id:
                other.retired = True
        engine._set_state_total_progress(state, 1.128)
        track_profile = engine._track_physics_for_driver(state)
        physics = engine._vehicle_physics_for_driver(state)[DRIVING_LINE_RACING]
        distance_m = track_profile.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            state.total_progress,
        )
        green_target_mps = physics.target_speed_mps(
            distance_m,
            engine._physics_v2_modifiers(state),
        )

        engine._trigger_safety_car([])
        engine.safety_car_stage = "collecting"
        engine._safety_car_total_progress = state.total_progress + 0.5
        engine._sc_caught_driver_ids.clear()
        self.assertLess(engine._phase_lap_time_factor(state), 1.0)
        sc_modifiers = engine._physics_v2_modifiers(state)
        sc_target_mps = physics.target_speed_mps(distance_m, sc_modifiers)

        self.assertEqual(sc_modifiers.speed_limit_factor, 1.0)
        self.assertLessEqual(sc_target_mps, green_target_mps + 1e-9)

    def test_safety_car_activation_clears_drs_and_wake_physics(self) -> None:
        engine = _make_engine_for_circuit(3)
        leader = engine.driver_states[1]
        chaser = engine.driver_states[2]
        corner_progress = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        ).progress
        engine._set_state_total_progress(chaser, 1.0 + corner_progress)
        engine._set_state_total_progress(
            leader,
            chaser.total_progress + 30.0 / engine.track_length_m,
        )
        chaser.speed_kph = 280.0
        leader.speed_kph = 280.0
        leader.lateral_offset_m = chaser.lateral_offset_m

        engine._update_wake_state(chaser, leader)
        self.assertTrue(chaser.dirty_air_active)
        self.assertLess(chaser.wake_downforce_multiplier, 1.0)

        engine._trigger_safety_car([])

        for state in engine.driver_states.values():
            self.assertFalse(state.drs_active)
            self.assertFalse(state.dirty_air_active)
            self.assertEqual(state.wake_strength, 0.0)
            self.assertEqual(state.wake_drag_multiplier, 1.0)
            self.assertEqual(state.wake_downforce_multiplier, 1.0)
            self.assertEqual(state.wake_braking_grip_multiplier, 1.0)
            self.assertEqual(state.wake_lateral_grip_multiplier, 1.0)

    def test_bahrain_sc_catch_up_does_not_run_wide_or_leave_track(self) -> None:
        engine = _make_engine_for_circuit(3)
        state = engine.driver_states[1]
        for other in engine.driver_states.values():
            if other.driver_id != state.driver_id:
                other.retired = True
        engine._update_positions()
        engine._trigger_safety_car([])
        engine.safety_car_stage = "collecting"
        engine._safety_car_visible = True
        engine._safety_car_route = "track"
        engine._sc_cleanup_until = 1.0e9

        run_wide_samples = 0
        off_track_samples = 0
        lockup_events = 0
        for _ in range(600):
            engine.safety_car_stage = "collecting"
            engine._sc_caught_driver_ids.clear()
            engine._safety_car_total_progress = state.total_progress + 0.5
            engine._safety_car_speed_mps = 100.0 / 3.6
            engine._safety_car_progress_rate = (
                engine._safety_car_speed_mps / engine.track_length_m
            )
            events = engine.tick(GAME_TICK_SECONDS)
            run_wide_samples += int(state.handling_state == "run_wide")
            off_track_samples += int(state.off_track)
            lockup_events += sum(event.type == "lockup" for event in events)

        self.assertEqual(run_wide_samples, 0)
        self.assertEqual(off_track_samples, 0)
        self.assertEqual(lockup_events, 0)

    def test_grid_start_delay_eventually_launches_full_field(self) -> None:
        engine = _make_engine()

        for _ in range(200):
            engine.tick(GAME_TICK_SECONDS)

        for state in engine.driver_states.values():
            self.assertGreater(state.total_progress, 0.0)

    def test_grid_start_progress_follows_grid_order(self) -> None:
        engine = _make_engine()

        for _ in range(10):
            engine.tick(GAME_TICK_SECONDS)

        running = sorted(
            (s for s in engine.driver_states.values() if not s.retired),
            key=lambda s: s.position,
        )
        self.assertEqual(running[0].gap_to_leader, 0.0)
        progresses = [s.total_progress for s in running]
        self.assertEqual(progresses, sorted(progresses, reverse=True))

    def test_lap_time_uses_finish_line_interpolation(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        for _ in range(3000):
            engine.tick(GAME_TICK_SECONDS)
            if state.last_lap_time > 0:
                break

        self.assertGreater(state.last_lap_time, 0)
        self.assertGreater(
            abs((state.last_lap_time * 10) - round(state.last_lap_time * 10)),
            0.001,
        )
        completed_lap = engine._lap_history[state.driver_id][0]
        self.assertEqual(len(completed_lap.sector_times), 3)
        self.assertEqual(
            len(completed_lap.mini_sector_times),
            len(engine._timing_loops),
        )
        self.assertAlmostEqual(
            sum(completed_lap.sector_times),
            completed_lap.lap_time,
            delta=0.01,
        )

    def test_tick_state_exposes_driver_lap_history(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        # The fixed-step controller may need a little over one nominal lap
        # after a long SC test suite has warmed/rotated its bounded caches.
        for _ in range(3000):
            engine.tick(GAME_TICK_SECONDS)
            if state.last_lap_time > 0:
                break

        tick = engine.build_tick_state()
        position = next(p for p in tick.positions if p.driver_id == 1)

        self.assertEqual(len(position.lap_history), 1)
        self.assertEqual(position.lap_history[0].lap, 1)
        self.assertEqual(position.lap_history[0].stint, 1)
        self.assertEqual(position.lap_history[0].tire_compound, state.tire_compound.value)
        self.assertAlmostEqual(position.lap_history[0].lap_time, state.last_lap_time, places=3)
        self.assertEqual(len(position.mini_sector_splits), len(engine._timing_loops))
        self.assertEqual(len(position.mini_sector_statuses), len(engine._timing_loops))

    def test_pit_resets_tire_age(self) -> None:
        engine = _make_engine()
        driver_id = 1
        state = engine.driver_states[driver_id]
        state.tire_age = 10
        state.tire_usage = 10.0
        state.pit_request = TireCompound.HARD
        state.progress = 0.99
        engine.tick(GAME_TICK_SECONDS)
        while not state.in_pit and not state.finished:
            engine.tick(GAME_TICK_SECONDS)
        tick = engine.build_tick_state()
        position = next(p for p in tick.positions if p.driver_id == driver_id)
        self.assertTrue(position.in_pit)
        self.assertEqual(position.pit_phase, "in")
        while state.in_pit:
            engine.tick(GAME_TICK_SECONDS)
        self.assertEqual(state.tire_age, 0)
        # The final 0.1s batch may contain track frames after pit exit.
        self.assertLess(state.tire_usage, 0.01)
        self.assertEqual(state.pit_count, 1)

    def test_pit_stop_starts_new_lap_history_stint(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        entry = engine.circuit.pit_lane.entry_progress
        self.assertIsNotNone(entry)

        old_compound = state.tire_compound.value
        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)

        while state.in_pit:
            engine.tick(GAME_TICK_SECONDS)

        history = engine._lap_history[driver_id]
        self.assertGreaterEqual(len(history), 1)
        self.assertTrue(history[-1].pit_stop)
        self.assertEqual(history[-1].stint, 1)
        self.assertEqual(history[-1].tire_compound, old_compound)

        pit_history_count = len(history)
        # The new fixed-slot SC/physics cadence can leave the car just short
        # of the next timing line at the nominal bound.
        for _ in range(3000):
            engine.tick(GAME_TICK_SECONDS)
            if len(engine._lap_history[driver_id]) > pit_history_count:
                break

        next_lap = engine._lap_history[driver_id][-1]
        self.assertFalse(next_lap.pit_stop)
        self.assertEqual(next_lap.stint, 2)
        self.assertEqual(next_lap.tire_compound, TireCompound.HARD.value)

    def test_pit_progresses_through_lane_phases(self) -> None:
        engine = _make_engine()
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        state.pit_request = TireCompound.HARD
        state.progress = 0.99
        engine.tick(GAME_TICK_SECONDS)
        while not state.in_pit and not state.finished:
            engine.tick(GAME_TICK_SECONDS)
        self.assertTrue(state.in_pit)

        phases_seen = []
        lane_progress_seen = []
        elapsed_seen = []
        stop_elapsed_seen = []
        while state.in_pit:
            tick = engine.build_tick_state()
            pos = next(p for p in tick.positions if p.driver_id == driver_id)
            phases_seen.append(pos.pit_phase)
            lane_progress_seen.append(pos.pit_lane_progress)
            elapsed_seen.append(pos.pit_elapsed)
            stop_elapsed_seen.append(pos.pit_stop_elapsed)
            engine.tick(GAME_TICK_SECONDS)

        # 전용 출구가 있는 서킷은 피트 본선 뒤의 분리 차선까지 거친다.
        expected_phases = {"in", "stop", "out"}
        if engine.circuit.pit_lane.exit_lane_rejoin_progress is not None:
            expected_phases.add("exit_lane")
        self.assertEqual(expected_phases, set(phases_seen))
        # 핏레인 진행도는 0 부근에서 시작해 출구(1.0 부근)로 단조 증가에 가깝게 진행.
        self.assertLess(lane_progress_seen[0], 0.2)
        self.assertGreater(max(lane_progress_seen), 0.9)
        # 전체 경과/정지 경과 시간은 감소가 아니라 증가한다.
        self.assertEqual(elapsed_seen, sorted(elapsed_seen))
        self.assertGreater(elapsed_seen[-1], 0)
        self.assertEqual(stop_elapsed_seen, sorted(stop_elapsed_seen))
        self.assertGreater(max(stop_elapsed_seen), 0)
        # 정지 단계에서만 타이어 교체 시간이 쌓이므로 전체 경과보다 작다.
        self.assertLess(max(stop_elapsed_seen), max(elapsed_seen) + 1e-6)

    def test_pit_lane_progress_updates_live_race_order_distance(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        rival_id = 2
        state = engine.driver_states[driver_id]
        rival = engine.driver_states[rival_id]
        for other in engine.driver_states.values():
            if other.driver_id not in {driver_id, rival_id}:
                other.retired = True
        entry = engine.circuit.pit_lane.entry_progress
        exit_ = engine.circuit.pit_lane.exit_progress
        self.assertIsNotNone(entry)
        self.assertIsNotNone(exit_)

        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        self.assertTrue(state.in_pit)

        entry_total_progress = state.total_progress
        engine.tick(engine._pit_lane_half_time[driver_id] / 2.0)

        lane_progress = engine._pit_lane_progress(driver_id)
        expected_total_progress = (
            state.current_lap
            + entry
            + engine._progress_distance(entry, exit_) * lane_progress
        )
        self.assertGreater(state.total_progress, entry_total_progress)
        self.assertAlmostEqual(state.total_progress, expected_total_progress, places=6)

        rival.total_progress = expected_total_progress - 0.001
        engine._tick_phase = TickPhase.RULES
        engine._update_positions()
        self.assertLess(state.position, rival.position)

        rival.total_progress = expected_total_progress + 0.001
        engine._update_positions()
        self.assertLess(rival.position, state.position)
        engine._tick_phase = TickPhase.TELEMETRY

    def test_real_circuit_pit_order_progress_is_monotonic(self) -> None:
        for circuit_id in _real_circuit_ids():
            engine = _make_engine_for_circuit(circuit_id)
            driver_id = 1
            state = engine.driver_states[driver_id]
            for other in engine.driver_states.values():
                if other.driver_id != driver_id:
                    other.retired = True
            entry = engine.circuit.pit_lane.entry_progress
            exit_ = engine.circuit.pit_lane.exit_progress
            self.assertIsNotNone(entry)
            self.assertIsNotNone(exit_)

            state.pit_request = TireCompound.HARD
            state.progress = entry - 0.001
            state.total_progress = state.current_lap + state.progress
            engine.tick(GAME_TICK_SECONDS)
            self.assertTrue(state.in_pit, engine.circuit.name)

            pit_entry_lap = state.current_lap
            observed = [state.total_progress]
            while state.in_pit:
                engine.tick(GAME_TICK_SECONDS)
                observed.append(state.total_progress)

            self.assertEqual(observed, sorted(observed), engine.circuit.name)
            self.assertGreater(observed[-1] - observed[0], 0.02, engine.circuit.name)
            if engine.circuit.pit_lane.exit_lane_rejoin_progress is not None:
                rejoin_total = (
                    pit_entry_lap
                    + 1.0
                    + engine.circuit.pit_lane.exit_lane_rejoin_progress
                )
            else:
                rejoin_total = (
                    pit_entry_lap
                    + entry
                    + engine._progress_distance(entry, exit_)
                    * engine.circuit.pit_lane.side_rejoin_progress
                )
            # A large external tick can include a few 20ms on-track frames
            # after the exact side-rejoin frame. The final state must be at or
            # just beyond the merge point, never behind or discontinuously far.
            self.assertGreaterEqual(observed[-1], rejoin_total)
            self.assertLessEqual(
                observed[-1] - rejoin_total,
                16.0 / engine.track_length_m,
                engine.circuit.name,
            )

    def test_pit_lane_drive_and_change_times_are_separate(self) -> None:
        engine = _make_engine()
        driver_id = 1
        state = engine.driver_states[driver_id]
        team = engine.teams[engine.drivers[driver_id].team_id]
        state.pit_request = TireCompound.SOFT
        state.progress = 0.99
        engine.tick(GAME_TICK_SECONDS)
        while not state.in_pit and not state.finished:
            engine.tick(GAME_TICK_SECONDS)

        half_lane = engine._pit_lane_half_time[driver_id]
        tire_change = engine._pit_tire_change_time[driver_id]
        # 레인 절반 시간은 pit_loss_time의 절반과 일치하고, 타이어 교체는 별도 값.
        self.assertAlmostEqual(half_lane, engine.circuit.pit_loss_time / 2.0, places=6)
        self.assertGreater(tire_change, 0)
        self.assertNotAlmostEqual(tire_change, engine.circuit.pit_loss_time, places=3)

    def test_pit_enters_at_configured_entry_progress(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        entry = engine.circuit.pit_lane.entry_progress
        self.assertIsNotNone(entry)

        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress

        engine.tick(GAME_TICK_SECONDS)

        self.assertTrue(state.in_pit)
        self.assertEqual(engine._pit_phase[driver_id], "in")
        self.assertAlmostEqual(state.progress, entry, places=6)
        self.assertLess(state.progress, 1.0)

    def test_bahrain_pit_out_uses_side_lane_before_rejoining_racing_line(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        entry = engine.circuit.pit_lane.entry_progress
        exit_ = engine.circuit.pit_lane.exit_progress
        self.assertIsNotNone(entry)
        self.assertIsNotNone(exit_)

        starting_lap = state.current_lap
        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        self.assertTrue(state.in_pit)

        while state.in_pit:
            engine.tick(GAME_TICK_SECONDS)

        self.assertEqual(state.pit_count, 1)
        self.assertIsNotNone(engine.circuit.pit_lane.exit_lane_rejoin_progress)
        expected_total = (
            starting_lap
            + 1.0
            + engine.circuit.pit_lane.exit_lane_rejoin_progress
        )
        self.assertAlmostEqual(state.total_progress, expected_total, delta=0.001)
        self.assertGreater(state.progress, exit_)

    def test_bahrain_pit_out_remains_in_dedicated_lane_until_final_merge(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        pit_lane = engine.circuit.pit_lane
        assert pit_lane is not None and pit_lane.entry_progress is not None

        state.pit_request = TireCompound.HARD
        state.progress = pit_lane.entry_progress - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        previous_total_progress = state.total_progress
        while state.in_pit and engine._pit_phase[driver_id] != "exit_lane":
            engine.tick(PHYSICS_STEP_SECONDS)
            self.assertGreaterEqual(state.total_progress, previous_total_progress)
            previous_total_progress = state.total_progress

        self.assertTrue(state.in_pit)
        self.assertEqual(engine._pit_phase[driver_id], "exit_lane")
        self.assertEqual(state.pit_count, 0)
        start_world = (state.world_x_m, state.world_y_m)
        engine.tick(PHYSICS_STEP_SECONDS)
        moved_m = hypot(
            state.world_x_m - start_world[0],
            state.world_y_m - start_world[1],
        )
        self.assertLess(moved_m, 2.0)
        self.assertGreater(moved_m, 0.0)

        while (
            state.in_pit
            and engine._pit_exit_lane_progress[driver_id]
            < pit_lane.exit_lane_merge_start * 0.8
        ):
            engine.tick(PHYSICS_STEP_SECONDS)
        profile = engine._track_physics_for_driver(state)
        line_x, line_y, line_heading = profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            state.progress,
        )
        lateral_separation_m = abs(
            (state.world_x_m - line_x) * -sin(line_heading)
            + (state.world_y_m - line_y) * cos(line_heading)
        )
        self.assertGreater(lateral_separation_m, 2.0)

    def test_tick_state_exposes_progress_rates_for_frontend_prediction(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine.tick(GAME_TICK_SECONDS)

        tick = engine.build_tick_state()
        driver = next(p for p in tick.positions if p.driver_id == 1)

        self.assertGreater(tick.race_elapsed, 0.0)
        self.assertGreater(driver.progress_rate, 0.0)
        self.assertEqual(driver.pit_lane_progress_rate, 0.0)

    def test_tick_state_exposes_pit_lane_progress_rate(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        entry = engine.circuit.pit_lane.entry_progress
        self.assertIsNotNone(entry)

        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)

        tick = engine.build_tick_state()
        driver = next(p for p in tick.positions if p.driver_id == driver_id)

        self.assertTrue(driver.in_pit)
        self.assertEqual(driver.progress_rate, 0.0)
        self.assertGreater(driver.pit_lane_progress_rate, 0.0)

    def test_pit_brakes_to_configured_limit_then_accelerates_after_release(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        entry = engine.circuit.pit_lane.entry_progress
        self.assertIsNotNone(entry)

        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)

        tick = engine.build_tick_state()
        driver = next(p for p in tick.positions if p.driver_id == driver_id)
        self.assertEqual(driver.pit_phase, "in")
        self.assertGreater(driver.speed_kph, PIT_LANE_SPEED_LIMIT_KPH)

        limit_start = engine.circuit.pit_lane.speed_limit_start
        speeds_before_limit = [driver.speed_kph]
        for _ in range(10000):
            engine.tick(PHYSICS_STEP_SECONDS)
            speeds_before_limit.append(state.speed_kph)
            if engine._pit_lane_progress(driver_id) >= limit_start:
                break
        self.assertAlmostEqual(
            engine._pit_lane_progress(driver_id),
            limit_start,
            places=9,
        )
        self.assertAlmostEqual(state.speed_kph, PIT_LANE_SPEED_LIMIT_KPH, places=6)
        self.assertLess(speeds_before_limit[-1], speeds_before_limit[0])

        while state.in_pit and engine._pit_phase[driver_id] != "stop":
            engine.tick(GAME_TICK_SECONDS)
        tick = engine.build_tick_state()
        driver = next(p for p in tick.positions if p.driver_id == driver_id)
        self.assertEqual(driver.pit_phase, "stop")
        self.assertEqual(driver.speed_kph, 0.0)

        while state.in_pit and engine._pit_phase[driver_id] != "out":
            engine.tick(GAME_TICK_SECONDS)
        tick = engine.build_tick_state()
        driver = next(p for p in tick.positions if p.driver_id == driver_id)
        self.assertEqual(driver.pit_phase, "out")
        self.assertLessEqual(driver.speed_kph, 2.0)

        limit_end = engine.circuit.pit_lane.speed_limit_end
        for _ in range(3000):
            engine.tick(PHYSICS_STEP_SECONDS)
            if engine._pit_lane_progress(driver_id) >= limit_end:
                break
        self.assertAlmostEqual(state.speed_kph, PIT_LANE_SPEED_LIMIT_KPH, places=6)
        engine.tick(GAME_TICK_SECONDS)
        self.assertGreater(state.speed_kph, PIT_LANE_SPEED_LIMIT_KPH)

    def test_bahrain_assigns_one_ordered_pit_box_per_team(self) -> None:
        engine = _make_engine_for_circuit(3)
        route_length_m = engine._pit_route_length_m()
        boxes_by_team: dict[int, set[float]] = {}
        for driver in engine.drivers.values():
            boxes_by_team.setdefault(driver.team_id, set()).add(
                engine._pit_box_progress_for_driver(driver.id)
            )

        self.assertEqual(len(boxes_by_team), 10)
        self.assertTrue(all(len(progresses) == 1 for progresses in boxes_by_team.values()))
        ordered = [next(iter(boxes_by_team[team_id])) for team_id in sorted(boxes_by_team)]
        self.assertEqual(ordered, sorted(ordered))
        for first, second in zip(ordered, ordered[1:]):
            self.assertAlmostEqual(
                (second - first) * route_length_m,
                16.0,
                places=6,
            )

    def test_pit_car_stops_at_its_team_garage_front(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        pit_lane = engine.circuit.pit_lane
        self.assertIsNotNone(pit_lane)
        assert pit_lane is not None
        entry = pit_lane.entry_progress
        self.assertIsNotNone(entry)
        assert entry is not None

        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        while state.in_pit and engine._pit_phase[driver_id] != "stop":
            engine.tick(PHYSICS_STEP_SECONDS)

        assigned = engine._pit_box_progress_for_driver(driver_id)
        self.assertAlmostEqual(engine._pit_lane_progress(driver_id), assigned, places=9)
        base_x, base_y, base_heading = engine._pit_lane_pose_at_progress_m(assigned)
        lateral_displacement_m = (
            (state.world_x_m - base_x) * -sin(base_heading)
            + (state.world_y_m - base_y) * cos(base_heading)
        )
        self.assertAlmostEqual(
            lateral_displacement_m,
            pit_lane.box_offset,
            delta=0.05,
        )
        tick_driver = next(
            item
            for item in engine.build_tick_state().positions
            if item.driver_id == driver_id
        )
        self.assertAlmostEqual(tick_driver.pit_box_progress, assigned, places=7)

    def test_bahrain_compact_eighty_kph_pit_zone_keeps_continuous_transit(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        for other in engine.driver_states.values():
            if other.driver_id != driver_id:
                other.retired = True
        pit_lane = engine.circuit.pit_lane
        self.assertIsNotNone(pit_lane)
        assert pit_lane is not None and pit_lane.entry_progress is not None
        self.assertEqual(pit_lane.speed_limit_kph, 80.0)
        self.assertEqual(pit_lane.speed_limit_start, 0.30)
        self.assertEqual(pit_lane.speed_limit_end, 0.70)

        state.pit_request = TireCompound.HARD
        state.progress = pit_lane.entry_progress - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        elapsed = 0.0
        while state.in_pit:
            engine.tick(PHYSICS_STEP_SECONDS)
            elapsed += PHYSICS_STEP_SECONDS

        self.assertAlmostEqual(elapsed, 32.84, delta=0.12)

    def test_pit_rejoin_rule_yields_holds_then_merges_when_occupancy_clears(self) -> None:
        drivers = load_drivers()[:3]
        engine = RaceEngine(
            circuit=next(c for c in load_circuits() if c.id == 3),
            drivers=drivers,
            teams={team.id: team for team in load_teams()},
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=99,
            start_sequence_enabled=False,
        )
        pit_car = engine.driver_states[drivers[0].id]
        traffic = engine.driver_states[drivers[1].id]
        group_mate = engine.driver_states[drivers[2].id]
        pit_lane = engine.circuit.pit_lane
        assert pit_lane is not None
        lane_progress = pit_lane.speed_limit_end + 0.01
        speed_mps = 30.0
        route_length_m = engine._pit_route_length_m()
        remaining_m = (
            pit_lane.side_rejoin_progress - lane_progress
        ) * route_length_m
        # Match _pit_rejoin_decision: long pit-exit legs are capped at 5s.
        time_to_rejoin_s = min(
            5.0,
            max(PHYSICS_STEP_SECONDS, remaining_m / speed_mps),
        )
        entry = engine._pit_entry_progress()
        exit_ = engine._pit_exit_progress()
        assert entry is not None and exit_ is not None
        mapped_total = (
            pit_car.current_lap
            + entry
            + engine._progress_distance(entry, exit_)
            * pit_lane.side_rejoin_progress
        )
        track_progress = mapped_total % 1.0
        pit_x, pit_y, _ = engine._pit_lane_pose_at_progress_m(
            pit_lane.side_rejoin_progress
        )
        profile = engine._track_physics_for_driver(pit_car)
        line_length_m = profile.length_for_line(DRIVING_LINE_RACING)
        for _ in range(4):
            line_x, line_y, heading = profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                track_progress,
            )
            tangent_m = (
                (pit_x - line_x) * cos(heading)
                + (pit_y - line_y) * sin(heading)
            )
            track_progress = (track_progress + tangent_m / line_length_m) % 1.0
        target_total = int(mapped_total) + track_progress
        line_x, line_y, heading = profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        traffic.lateral_offset_m = profile.line_offset_at_progress(
            DRIVING_LINE_RACING,
            track_progress,
        ) + (
            (pit_x - line_x) * -sin(heading)
            + (pit_y - line_y) * cos(heading)
        )
        traffic.speed_kph = 80.0 * 3.6
        traffic.total_progress = (
            target_total
            - 80.0 * time_to_rejoin_s / engine.track_length_m
        )
        group_mate.lateral_offset_m = traffic.lateral_offset_m + 3.0
        group_mate.speed_kph = traffic.speed_kph
        group_mate.total_progress = target_total + 200.0 / engine.track_length_m
        engine._maneuver_groups = {
            "mg-pit-traffic": ManeuverGroup(
                group_id="mg-pit-traffic",
                member_ids=(traffic.driver_id, group_mate.driver_id),
                pair_keys=(),
                phase="overlap",
                minimum_lateral_m=traffic.lateral_offset_m,
                maximum_lateral_m=group_mate.lateral_offset_m,
            )
        }

        decision = engine._pit_rejoin_decision(
            pit_car.driver_id,
            pit_car,
            lane_progress,
            speed_mps,
        )
        self.assertEqual(decision.state, "yield")
        self.assertEqual(decision.conflict_driver_id, traffic.driver_id)
        self.assertEqual(decision.conflict_group_id, "mg-pit-traffic")
        self.assertEqual(
            decision.conflict_group_member_ids,
            (traffic.driver_id, group_mate.driver_id),
        )

        hold_progress = (
            pit_lane.side_rejoin_progress
            - 4.0 / route_length_m
        )
        hold_time_s = sqrt(
            2.0
            * (pit_lane.side_rejoin_progress - hold_progress)
            * route_length_m
            / 9.0
        )
        traffic.total_progress = target_total - 80.0 * hold_time_s / engine.track_length_m
        decision = engine._pit_rejoin_decision(
            pit_car.driver_id,
            pit_car,
            hold_progress,
            0.0,
        )
        self.assertEqual(decision.state, "hold")

        traffic.total_progress = target_total + 100.0 / engine.track_length_m
        decision = engine._pit_rejoin_decision(
            pit_car.driver_id,
            pit_car,
            hold_progress,
            0.0,
        )
        self.assertEqual(decision.state, "merge")
        self.assertIsNone(decision.conflict_driver_id)
        self.assertIsNone(decision.conflict_group_id)

    def test_pause_stops_tick_progress(self) -> None:
        engine = _make_engine()
        engine.tick(GAME_TICK_SECONDS)
        before = engine.driver_states[1].total_progress
        engine.pause_race()
        engine.tick(GAME_TICK_SECONDS)
        after = engine.driver_states[1].total_progress
        self.assertEqual(before, after)
        engine.resume_race()
        engine.tick(GAME_TICK_SECONDS)
        self.assertGreater(engine.driver_states[1].total_progress, after)

    def test_speed_multiplier_affects_progress(self) -> None:
        engine_slow = _make_engine(seed=99)
        engine_fast = _make_engine(seed=99)
        engine_fast.set_speed(2)
        engine_slow.tick(GAME_TICK_SECONDS)
        engine_fast.tick(GAME_TICK_SECONDS * 2)
        slow_progress = engine_slow.driver_states[1].total_progress
        fast_progress = engine_fast.driver_states[1].total_progress
        self.assertGreater(fast_progress, slow_progress)

    def test_speed_multiplier_does_not_change_same_fixed_step_results(self) -> None:
        engine_one_x = _make_engine(seed=2026)
        engine_two_x = _make_engine(seed=2026)
        engine_two_x.set_speed(2)

        # The session uses the same 20 ms authoritative tick at both speeds;
        # 2x only schedules twice as many of those ticks per wall-clock second.
        for _ in range(300):
            engine_one_x.tick(PHYSICS_STEP_SECONDS)
            engine_two_x.tick(PHYSICS_STEP_SECONDS)

        self.assertEqual(engine_one_x.race_elapsed, engine_two_x.race_elapsed)
        self.assertEqual(engine_one_x.race_phase, engine_two_x.race_phase)
        self.assertEqual(engine_one_x.rng.getstate(), engine_two_x.rng.getstate())
        for driver_id in engine_one_x.driver_states:
            one_x = engine_one_x.driver_states[driver_id]
            two_x = engine_two_x.driver_states[driver_id]
            self.assertAlmostEqual(one_x.total_progress, two_x.total_progress)
            self.assertEqual(one_x.current_lap, two_x.current_lap)
            self.assertEqual(one_x.position, two_x.position)
            self.assertEqual(one_x.retired, two_x.retired)
            self.assertEqual(one_x.finished, two_x.finished)

    def test_tire_temperature_diagnostics_track_bounded_rear_overheat_duration(self) -> None:
        engine = _make_engine(seed=2026)
        engine.driver_states[1].rear_tire_surface_temperature_c = 131.0
        engine.driver_states[1].front_tire_surface_temperature_c = 112.0
        engine._record_tire_temperature_diagnostics(PHYSICS_STEP_SECONDS)

        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["current"]["rear_overheat_driver_ids"], [1])
        self.assertAlmostEqual(
            snapshot["current"]["max_current_overheat_seconds"],
            PHYSICS_STEP_SECONDS,
        )
        self.assertEqual(snapshot["peak"]["rear_surface_max_c"], 131.0)

        for _ in range(4):
            engine._record_tire_temperature_diagnostics(PHYSICS_STEP_SECONDS)
        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertAlmostEqual(
            snapshot["peak"]["max_continuous_overheat_seconds"],
            5 * PHYSICS_STEP_SECONDS,
        )
        self.assertEqual(snapshot["peak"]["max_continuous_overheat_driver_id"], 1)

        engine.driver_states[1].rear_tire_surface_temperature_c = 120.0
        engine._record_tire_temperature_diagnostics(PHYSICS_STEP_SECONDS)
        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["current"]["rear_overheat_driver_ids"], [])
        self.assertEqual(snapshot["current"]["max_current_overheat_seconds"], 0.0)

    def test_speed_multiplier_rejects_unsupported_values(self) -> None:
        engine = _make_engine(seed=99)
        self.assertFalse(engine.set_speed(3))
        self.assertEqual(engine.speed_multiplier, 1)
        self.assertTrue(engine.set_speed(2))
        self.assertEqual(engine.speed_multiplier, 2)

    def test_player_starting_tires_are_applied(self) -> None:
        engine = _make_engine_with_starting_tires({
            1: TireCompound.SOFT,
            2: TireCompound.HARD,
        })

        self.assertEqual(engine.driver_states[1].tire_compound, TireCompound.SOFT)
        self.assertEqual(engine.driver_states[2].tire_compound, TireCompound.HARD)

    def test_player_can_set_pace_mode(self) -> None:
        engine = _make_engine()

        error = engine.set_pace_mode(1, PaceMode.ATTACK)

        self.assertIsNone(error)
        self.assertEqual(engine.driver_states[1].pace_mode, PaceMode.ATTACK)
        self.assertIn(1, engine._pace_mode_replan_required)
        self.assertEqual(
            engine._local_trajectory_plan_ages[1],
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
        )

    def test_pace_mode_changes_physical_limit_utilization(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]

        state.pace_mode = PaceMode.CONSERVE
        conserve = engine._physics_v2_modifiers(state)
        state.pace_mode = PaceMode.STANDARD
        standard = engine._physics_v2_modifiers(state)
        state.pace_mode = PaceMode.ATTACK
        attack = engine._physics_v2_modifiers(state)

        for attribute in ("pace", "grip", "braking", "traction"):
            self.assertLess(
                getattr(conserve, attribute),
                getattr(standard, attribute),
            )
            self.assertLess(
                getattr(standard, attribute),
                getattr(attack, attribute),
            )

    def test_pace_mode_scales_drs_target_speed_commitment(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        state.drs_active = True

        state.pace_mode = PaceMode.CONSERVE
        conserve = engine._physics_v2_modifiers(state).speed_limit_factor
        state.pace_mode = PaceMode.STANDARD
        standard = engine._physics_v2_modifiers(state).speed_limit_factor
        state.pace_mode = PaceMode.ATTACK
        attack = engine._physics_v2_modifiers(state).speed_limit_factor

        self.assertLess(conserve, standard)
        self.assertLess(standard, attack)

    def test_pace_mode_physics_and_planner_weights_transition_smoothly(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        standard_physics = engine._physics_v2_modifiers(state)
        standard_weights = engine._interpolated_pace_mode_planner_weights(state)

        engine.set_pace_mode(state.driver_id, PaceMode.ATTACK)
        transition_start = engine._physics_v2_modifiers(state)
        engine._tick_pace_mode_transitions(0.75)
        transition_mid = engine._physics_v2_modifiers(state)
        mid_weights = engine._interpolated_pace_mode_planner_weights(state)
        engine._tick_pace_mode_transitions(0.75)
        transition_end = engine._physics_v2_modifiers(state)
        attack_weights = engine._interpolated_pace_mode_planner_weights(state)

        self.assertAlmostEqual(transition_start.pace, standard_physics.pace)
        self.assertLess(standard_physics.pace, transition_mid.pace)
        self.assertLess(transition_mid.pace, transition_end.pace)
        self.assertGreater(
            standard_weights.extra_path_distance,
            mid_weights.extra_path_distance,
        )
        self.assertGreater(
            mid_weights.extra_path_distance,
            attack_weights.extra_path_distance,
        )
        self.assertEqual(engine._pace_mode_transition_progress(state), 1.0)
        self.assertEqual(
            engine._pace_mode_transition_from[state.driver_id],
            PaceMode.ATTACK,
        )

    def test_retargeting_pace_mode_continues_from_current_effective_value(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        engine.set_pace_mode(state.driver_id, PaceMode.ATTACK)
        engine._tick_pace_mode_transitions(0.6)
        before_retarget = engine._effective_pace_mode_intensity(state)

        engine.set_pace_mode(state.driver_id, PaceMode.CONSERVE)
        immediately_after = engine._effective_pace_mode_intensity(state)
        engine._tick_pace_mode_transitions(0.1)
        after_next_step = engine._effective_pace_mode_intensity(state)

        self.assertAlmostEqual(immediately_after, before_retarget)
        self.assertLess(after_next_step, immediately_after)
        self.assertGreater(after_next_step, -1.0)
        self.assertEqual(
            engine._pace_mode_transition_target[state.driver_id],
            PaceMode.CONSERVE,
        )

    def test_pace_mode_change_forces_next_local_trajectory_replan(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        for other in engine.driver_states.values():
            if other.driver_id != state.driver_id:
                other.retired = True
        engine._update_local_trajectory_plan(
            state,
            DRIVING_LINE_RACING,
            engine._physics_v2_modifiers(state),
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
        )
        standard_plan = engine._local_trajectory_plans[state.driver_id]

        engine.set_pace_mode(state.driver_id, PaceMode.ATTACK)

        self.assertIs(
            engine._local_trajectory_plans[state.driver_id],
            standard_plan,
        )
        engine._tick_pace_mode_transitions(0.75)
        engine._update_local_trajectory_plan(
            state,
            DRIVING_LINE_RACING,
            engine._physics_v2_modifiers(state),
            0.0,
        )
        attack_plan = engine._local_trajectory_plans[state.driver_id]

        self.assertIsNot(attack_plan, standard_plan)
        self.assertNotEqual(
            attack_plan.selected.objective_cost,
            standard_plan.selected.objective_cost,
        )
        self.assertNotIn(state.driver_id, engine._pace_mode_replan_required)
        self.assertEqual(engine._local_trajectory_selected_ages[state.driver_id], 0.0)

    def test_non_player_cannot_set_pace_mode(self) -> None:
        engine = _make_engine()

        error = engine.set_pace_mode(3, PaceMode.ATTACK)

        self.assertEqual(error, "Not a player driver")
        self.assertEqual(engine.driver_states[3].pace_mode, PaceMode.STANDARD)

    def test_ai_pace_attacks_when_close_to_car_ahead(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        ai_driver = engine.driver_states[3]
        car_ahead.current_lap = 1
        ai_driver.current_lap = 1
        car_ahead.progress = 0.500
        ai_driver.progress = 0.492
        car_ahead.total_progress = 1.500
        ai_driver.total_progress = 1.492

        mode = engine._choose_ai_pace_mode(ai_driver, car_ahead, None)

        self.assertEqual(mode, PaceMode.ATTACK)

    def test_ai_pace_attack_gap_uses_exit_hysteresis(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        ai_driver = engine.driver_states[3]
        ai_driver.current_lap = 1
        car_ahead.current_lap = 1
        ai_driver.total_progress = 1.40
        gap_progress = 1.15 / engine._base_lap_time_for_state(ai_driver)
        car_ahead.total_progress = ai_driver.total_progress + gap_progress

        ai_driver.pace_mode = PaceMode.STANDARD
        standard_decision = engine._choose_ai_pace_mode(
            ai_driver,
            car_ahead,
            None,
        )
        ai_driver.pace_mode = PaceMode.ATTACK
        attack_decision = engine._choose_ai_pace_mode(
            ai_driver,
            car_ahead,
            None,
        )

        self.assertEqual(standard_decision, PaceMode.STANDARD)
        self.assertEqual(attack_decision, PaceMode.ATTACK)

    def test_ai_pace_conserve_tire_threshold_uses_exit_hysteresis(self) -> None:
        engine = _make_engine()
        ai_driver = engine.driver_states[3]
        _set_test_tire(engine, ai_driver, TireCompound.SOFT)
        ai_driver.tire_usage = next(
            usage / 10.0
            for usage in range(1, 300)
            if 0.27
            <= 1.0 - engine._current_tire_wear(
                ai_driver.model_copy(update={"tire_usage": usage / 10.0})
            )
            <= 0.30
        )

        ai_driver.pace_mode = PaceMode.STANDARD
        standard_decision = engine._choose_ai_pace_mode(ai_driver, None, None)
        ai_driver.pace_mode = PaceMode.CONSERVE
        conserve_decision = engine._choose_ai_pace_mode(ai_driver, None, None)

        self.assertEqual(standard_decision, PaceMode.STANDARD)
        self.assertEqual(conserve_decision, PaceMode.CONSERVE)

    def test_ai_pace_neutralizes_attack_during_sc(self) -> None:
        engine = _make_engine()
        ai_driver = engine.driver_states[3]
        ai_driver.pace_mode = PaceMode.ATTACK
        engine.race_phase = "sc"

        mode = engine._choose_ai_pace_mode(ai_driver, None, None)

        self.assertEqual(mode, PaceMode.CONSERVE)

    def test_ai_pace_attacks_in_endgame_with_usable_tires(self) -> None:
        engine = _make_engine()
        ai_driver = engine.driver_states[3]
        ai_driver.current_lap = engine.total_laps - 2

        mode = engine._choose_ai_pace_mode(ai_driver, None, None)

        self.assertEqual(mode, PaceMode.ATTACK)

    def test_ai_initial_decisions_are_staggered_per_driver(self) -> None:
        engine = _make_engine()
        cooldowns = {
            driver_id: cooldown
            for driver_id, cooldown in engine._ai_pace_cooldown.items()
            if driver_id not in engine.player_driver_ids
        }

        self.assertEqual(len(cooldowns), len(engine.driver_states) - 2)
        self.assertEqual(len(set(cooldowns.values())), len(cooldowns))
        self.assertGreaterEqual(
            min(cooldowns.values()),
            AI_PACE_INITIAL_STAGGER_MIN_SECONDS,
        )
        self.assertLessEqual(
            max(cooldowns.values()),
            AI_PACE_INITIAL_STAGGER_MAX_SECONDS,
        )

    def test_ai_pace_conserves_when_tire_life_is_critical(self) -> None:
        engine = _make_engine()
        ai_driver = engine.driver_states[3]
        _set_test_tire(engine, ai_driver, TireCompound.SOFT)
        ai_driver.tire_usage = 80.0

        mode = engine._choose_ai_pace_mode(ai_driver, None, None)

        self.assertEqual(mode, PaceMode.CONSERVE)

    def test_ai_pace_attacks_on_fresh_tires_after_stop(self) -> None:
        engine = _make_engine()
        ai_driver = engine.driver_states[3]
        ai_driver.pit_count = 1
        ai_driver.tire_usage = 0.5

        mode = engine._choose_ai_pace_mode(ai_driver, None, None)

        self.assertEqual(mode, PaceMode.ATTACK)

    def test_ai_pace_cooldown_keeps_current_mode(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        ai_driver = engine.driver_states[3]
        car_ahead.position = 1
        ai_driver.position = 2
        car_ahead.current_lap = 1
        ai_driver.current_lap = 1
        car_ahead.progress = 0.500
        ai_driver.progress = 0.492
        car_ahead.total_progress = 1.500
        ai_driver.total_progress = 1.492
        ai_driver.pace_mode = PaceMode.CONSERVE
        engine._ai_pace_cooldown[ai_driver.driver_id] = 4.0

        engine._run_ai_pace_modes({1: car_ahead, 2: ai_driver})

        self.assertEqual(ai_driver.pace_mode, PaceMode.CONSERVE)

    def test_ai_pace_decision_uses_shared_transition_and_replan_path(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        ai_driver = engine.driver_states[3]
        car_ahead.position = 1
        ai_driver.position = 2
        car_ahead.current_lap = 1
        ai_driver.current_lap = 1
        car_ahead.total_progress = 1.500
        ai_driver.total_progress = 1.492
        engine._ai_pace_cooldown.pop(ai_driver.driver_id, None)

        engine._run_ai_pace_modes({1: car_ahead, 2: ai_driver})

        self.assertEqual(ai_driver.pace_mode, PaceMode.ATTACK)
        self.assertEqual(
            engine._pace_mode_transition_target[ai_driver.driver_id],
            PaceMode.ATTACK,
        )
        self.assertIn(ai_driver.driver_id, engine._pace_mode_replan_required)
        self.assertGreater(engine._ai_pace_cooldown[ai_driver.driver_id], 0.0)

    def test_ai_pace_modes_do_not_change_player_drivers(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[3]
        player = engine.driver_states[1]
        car_ahead.position = 1
        player.position = 2
        car_ahead.current_lap = 1
        player.current_lap = 1
        car_ahead.progress = 0.500
        player.progress = 0.492
        car_ahead.total_progress = 1.500
        player.total_progress = 1.492
        player.pace_mode = PaceMode.CONSERVE

        engine._run_ai_pace_modes({1: car_ahead, 2: player})

        self.assertEqual(player.pace_mode, PaceMode.CONSERVE)

    def test_attack_mode_is_faster_but_uses_more_tire(self) -> None:
        attack_engine = _make_engine(seed=99)
        conserve_engine = _make_engine(seed=99)
        attack = attack_engine.driver_states[1]
        conserve = conserve_engine.driver_states[1]
        attack_engine.set_pace_mode(1, PaceMode.ATTACK)
        conserve_engine.set_pace_mode(1, PaceMode.CONSERVE)

        for _ in range(20):
            attack_engine.tick(GAME_TICK_SECONDS)
            conserve_engine.tick(GAME_TICK_SECONDS)

        self.assertGreater(attack.total_progress, conserve.total_progress)
        self.assertGreater(attack.tire_usage, conserve.tire_usage)

    def test_battle_delta_rewards_attack_mode(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        attacker = engine.driver_states[2]
        car_ahead.position = 1
        attacker.position = 2
        _place_close_pair(engine, car_ahead, attacker, 1.020)
        attacker.pace_mode = PaceMode.CONSERVE
        conserve_delta = engine._battle_lap_time_delta(attacker, car_ahead)

        attacker.pace_mode = PaceMode.ATTACK
        attack_delta = engine._battle_lap_time_delta(attacker, car_ahead)

        self.assertTrue(attacker.drs_active)
        self.assertLess(attack_delta, conserve_delta)

    def test_battle_event_probability_only_applies_to_overtaking_segments(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]

        attacker.drs_active = True
        attacker.progress = 0.05
        straight_probability = engine._battle_event_probability(attacker, car_ahead, 0.4)

        attacker.progress = 0.45
        technical_probability = engine._battle_event_probability(attacker, car_ahead, 0.4)

        self.assertGreater(straight_probability, 0.0)
        self.assertEqual(technical_probability, 0.0)

    def test_spa_explicitly_blocks_dangerous_straight_attack_zones(self) -> None:
        engine = _make_engine_for_circuit(6)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        attacker.drs_active = True

        attacker.progress = 0.08
        eau_rouge_approach = engine._battle_event_probability(
            attacker,
            car_ahead,
            0.5,
        )
        attacker.progress = 0.20
        kemmel_straight = engine._battle_event_probability(
            attacker,
            car_ahead,
            0.5,
        )

        self.assertEqual(eau_rouge_approach, 0.0)
        self.assertGreater(kemmel_straight, 0.0)

    def test_spa_disallowed_side_by_side_zone_aborts_pull_out(self) -> None:
        engine = _make_engine_for_circuit(6)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        defender_total = 1.08
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 6.0 / engine.track_length_m,
        )
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            phase="pull_out",
        )

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        self.assertIsNotNone(battle)
        self.assertEqual(battle.phase, "abort")
        self.assertEqual([event.type for event in events], ["overtake_abort"])

    def test_pull_out_aborts_before_corner_when_overlap_is_not_established(self) -> None:
        engine = _make_engine_for_circuit(3)
        defender = engine.driver_states[4]
        attacker = engine.driver_states[3]
        defender.position = 1
        attacker.position = 2
        engine._set_state_total_progress(defender, 1.248)
        engine._set_state_total_progress(
            attacker,
            defender.total_progress - 8.0 / engine.track_length_m,
        )
        attacker.speed_kph = 300.0
        defender.speed_kph = 295.0
        attacker.lateral_offset_m = defender.lateral_offset_m
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_OUTSIDE,
            DEFENDER_LINE_RACING,
            phase="pull_out",
        )

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)

        self.assertIsNotNone(battle)
        self.assertEqual(battle.phase, "abort")
        self.assertEqual([event.type for event in events], ["overtake_abort"])

    def test_configured_straight_ignores_small_curvature_spikes(self) -> None:
        engine = _make_engine_for_circuit(4)
        attacker = engine.driver_states[1]
        engine._set_state_total_progress(attacker, 1.155)

        self.assertIsNone(
            engine._distance_to_next_corner_entry_m(attacker, 160.0)
        )

        engine._set_state_total_progress(attacker, 1.270)
        self.assertIsNotNone(
            engine._distance_to_next_corner_entry_m(attacker, 160.0)
        )

    def test_maybe_battle_event_creates_attack_when_forced(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.020)
        attacker.pace_mode = PaceMode.ATTACK
        engine.rng.random = lambda: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        _prepare_local_pull_out_plan(engine, attacker)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "attack")
        self.assertEqual(event.driver, "VER")
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        self.assertIsNotNone(battle)
        assert battle is not None
        self.assertEqual(battle.phase, "approach")
        self.assertTrue(battle.trajectory_authorized)
        self.assertNotEqual(battle.trajectory_candidate_id, "")
        self.assertGreaterEqual(
            battle.trajectory_minimum_clearance_m,
            0.35,
        )
        candidate = next(
            item
            for item in engine._local_trajectory_plans[
                attacker.driver_id
            ].candidates
            if item.candidate_id == battle.trajectory_candidate_id
        )
        self.assertTrue(candidate.viable)
        target = engine._physics_v2_target_lateral_offset(
            attacker,
            engine._track_physics.at_progress(attacker.progress),
            DRIVING_LINE_RACING,
            car_ahead,
            None,
        )
        racing_target = (
            engine._track_physics_for_driver(attacker).line_offset_at_progress(
                DRIVING_LINE_RACING,
                attacker.progress,
            )
        )
        self.assertGreater(
            (target - racing_target) * battle.trajectory_lateral_bias_m,
            0.0,
        )
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_straight_attack_is_rejected_without_a_safe_local_plan(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.020)
        attacker.pace_mode = PaceMode.ATTACK
        engine.rng.random = lambda: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNone(event)
        self.assertFalse(engine._is_side_by_side_active(attacker.driver_id))

    def test_trajectory_authorized_attack_uses_the_planned_pull_out(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        for state in engine.driver_states.values():
            if state.driver_id not in {attacker.driver_id, car_ahead.driver_id}:
                state.retired = True
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.020)
        attacker.pace_mode = PaceMode.ATTACK
        engine.rng.random = lambda: 0.0
        engine._battle_lap_time_delta(attacker, car_ahead)
        _prepare_local_pull_out_plan(engine, attacker)
        event = engine._maybe_battle_event(attacker, car_ahead)
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert event is not None and battle is not None
        start_lateral_offset_m = attacker.lateral_offset_m
        planned_direction = battle.trajectory_lateral_bias_m
        engine.rng.random = lambda: 0.99

        for _ in range(3):
            engine.tick(GAME_TICK_SECONDS)

        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        self.assertIsNotNone(battle)
        assert battle is not None
        self.assertTrue(battle.trajectory_authorized)
        self.assertIn(battle.phase, {"pull_out", "overlap"})
        self.assertGreater(
            (attacker.lateral_offset_m - start_lateral_offset_m)
            * planned_direction,
            0.0,
        )
        self.assertIsNone(
            oriented_body_overlap(
                engine._body_pose(attacker),
                engine._body_pose(car_ahead),
            )
        )

    def test_defend_event_adds_time_cost_to_attacker(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[1]
        attacker = engine.driver_states[14]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.136)
        attacker.lateral_offset_m = (
            car_ahead.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )
        attacker.pace_mode = PaceMode.CONSERVE
        rolls = iter([0.0, 1.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "defend")
        self.assertTrue(engine._is_side_by_side_active(attacker.driver_id))
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_close_heavy_braking_scores_create_side_by_side_event(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.136)
        attacker.lateral_offset_m = (
            car_ahead.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )
        attacker.pace_mode = PaceMode.STANDARD
        engine._driver_meta[attacker.driver_id]["overtaking"] = 0.90
        engine._driver_meta[car_ahead.driver_id]["defending"] = 0.75
        rolls = iter([0.0, 1.0, 0.0, 0.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "attack")
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)
        self.assertTrue(engine._is_side_by_side_active(attacker.driver_id))
        self.assertTrue(engine._is_side_by_side_active(car_ahead.driver_id))

        tick = engine.build_tick_state()
        positions = {position.driver_id: position for position in tick.positions}
        self.assertTrue(positions[attacker.driver_id].maneuver_active)
        self.assertTrue(positions[car_ahead.driver_id].maneuver_active)
        self.assertEqual(positions[attacker.driver_id].maneuver_phase, "approach")
        self.assertEqual(positions[attacker.driver_id].maneuver_role, "attacker")
        self.assertEqual(positions[car_ahead.driver_id].maneuver_role, "defender")
        self.assertFalse(positions[attacker.driver_id].side_by_side_active)

    def test_side_by_side_state_blocks_repeat_event_and_expires(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.136)
        attacker.lateral_offset_m = (
            car_ahead.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )
        attacker.pace_mode = PaceMode.STANDARD
        engine._driver_meta[attacker.driver_id]["overtaking"] = 0.90
        engine._driver_meta[car_ahead.driver_id]["defending"] = 0.75
        rolls = iter([0.0, 1.0, 0.0, 0.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "attack")
        engine._battle_event_cooldown.clear()
        engine.rng.random = lambda: 0.0
        self.assertIsNone(engine._maybe_battle_event(attacker, car_ahead))

        engine._tick_side_by_side_battles(2.0)
        self.assertTrue(engine._is_side_by_side_active(attacker.driver_id))
        events = engine._tick_side_by_side_battles(16.1)
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        self.assertIsNotNone(battle)
        self.assertEqual(battle.phase, "abort")
        self.assertEqual([item.type for item in events], ["overtake_abort"])

    def test_overtake_maneuver_advances_from_body_positions_and_rejoins(self) -> None:
        engine = _make_engine_for_circuit(3)
        defender = engine.driver_states[4]
        attacker = engine.driver_states[3]
        defender.position = 1
        attacker.position = 2
        engine._set_state_total_progress(defender, 1.200)
        engine._set_state_total_progress(
            attacker,
            1.200 - (PHYSICAL_CAR_LENGTH_M + 1.5) / engine.track_length_m,
        )
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            "T1 Braking",
        )
        attacker.lateral_offset_m = (
            defender.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )

        self.assertEqual(engine._physics_v2_virtual_line(attacker), DEFENDER_LINE_RACING)
        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        self.assertEqual(battle.phase, "pull_out")
        self.assertEqual(engine._physics_v2_virtual_line(attacker), ATTACK_LINE_INSIDE)

        attacker.lateral_offset_m = defender.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        engine._set_state_total_progress(
            attacker,
            defender.total_progress - 4.0 / engine.track_length_m,
        )
        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        self.assertEqual([event.type for event in events], ["side_by_side"])
        self.assertEqual(battle.phase, "overlap")

        attacker.position = 1
        defender.position = 2
        engine._set_state_total_progress(
            attacker,
            defender.total_progress + 5.3 / engine.track_length_m,
        )
        clear_events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        self.assertEqual(battle.phase, "clear")
        self.assertEqual([event.type for event in clear_events], ["pass"])
        engine._tick_side_by_side_battles(0.4)
        self.assertEqual(battle.phase, "merge")
        self.assertEqual(engine._physics_v2_virtual_line(attacker), DEFENDER_LINE_RACING)

        attacker.lateral_offset_m = engine._track_physics.line_offset_at_progress(
            DEFENDER_LINE_RACING,
            attacker.progress,
        )
        defender.lateral_offset_m = engine._track_physics.line_offset_at_progress(
            DEFENDER_LINE_RACING,
            defender.progress,
        )
        engine._tick_side_by_side_battles(0.4)
        self.assertFalse(engine._is_side_by_side_active(attacker.driver_id))

    def test_identical_attack_car_passes_or_aborts_without_contact(self) -> None:
        circuit = next(c for c in load_circuits() if c.id == 3)
        base = load_drivers()[0]
        defender = base.model_copy(
            update={
                "id": 101,
                "name": "Control Defender",
                "abbreviation": "DEF",
                "number": 91,
            }
        )
        attacker = base.model_copy(
            update={
                "id": 102,
                "name": "Control Attacker",
                "abbreviation": "ATK",
                "number": 92,
            }
        )
        engine = RaceEngine(
            circuit=circuit,
            drivers=[defender, attacker],
            teams={team.id: team for team in load_teams()},
            player_team_id=base.team_id,
            player_driver_ids=[defender.id, attacker.id],
            grid_order=[defender.id, attacker.id],
            seed=1,
            start_sequence_enabled=False,
        )
        engine.total_laps = 99
        engine.set_pace_mode(defender.id, PaceMode.STANDARD)
        engine.set_pace_mode(attacker.id, PaceMode.ATTACK)
        straight = max(
            (
                segment
                for segment in circuit.segments
                if segment.type == TrackSegmentType.STRAIGHT
                and segment.side_by_side_allowed
                and segment.overtake_start_allowed is not False
            ),
            key=lambda segment: (
                segment.end - segment.start
            ) * circuit.track_length_m,
        )
        leader_progress = 1.0 + straight.start + 20.0 / engine.track_length_m
        engine._set_state_total_progress(
            engine.driver_states[defender.id],
            leader_progress,
        )
        engine._set_state_total_progress(
            engine.driver_states[attacker.id],
            leader_progress - 15.0 / engine.track_length_m,
        )
        for state in engine.driver_states.values():
            state.lateral_offset_m = engine._track_physics_for_driver(
                state
            ).line_offset_at_progress(DRIVING_LINE_RACING, state.progress)
            state.speed_kph = 250.0
        engine._update_positions()

        event_types: list[str] = []
        pass_events = []
        contact_seen = False
        track_limit_seen = False
        for _ in range(300):
            tick_events = engine.tick(GAME_TICK_SECONDS)
            event_types.extend(event.type for event in tick_events)
            pass_events.extend(event for event in tick_events if event.type == "pass")
            contact_seen = contact_seen or any(
                state.contact_active for state in engine.driver_states.values()
            )
            track_limit_seen = track_limit_seen or any(
                state.track_limits_active for state in engine.driver_states.values()
            )
            if "pass" in event_types:
                break

        # The predictive following controller may correctly make an identical
        # car abandon the attempt before its front axle reaches overlap.  The
        # maneuver must still produce a tactical intent and a resolved outcome
        # without inventing a side-by-side state.
        self.assertTrue({"attack", "defend"} & set(event_types))
        self.assertTrue({"pass", "overtake_abort"} & set(event_types))
        if pass_events:
            self.assertEqual(len(pass_events), 1)
            self.assertGreaterEqual(
                pass_events[0].payload["clearance_m"],
                pass_events[0].payload["required_clearance_m"],
            )
        self.assertFalse(contact_seen)
        self.assertFalse(track_limit_seen)
        if pass_events:
            self.assertLess(
                engine.driver_states[attacker.id].position,
                engine.driver_states[defender.id].position,
            )

    def test_ranking_and_pass_feed_change_on_the_same_physical_clear_tick(self) -> None:
        engine = _make_engine_for_circuit(3)
        defender = engine.driver_states[4]
        attacker = engine.driver_states[3]
        defender.position = 1
        attacker.position = 2
        engine._set_state_total_progress(defender, 1.2)
        engine._set_state_total_progress(
            attacker,
            defender.total_progress + 2.0 / engine.track_length_m,
        )
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None

        engine._update_positions()

        self.assertEqual(defender.position, 1)
        self.assertEqual(attacker.position, 2)
        self.assertFalse(battle.pass_confirmed)
        self.assertEqual(
            [event.type for event in engine._tick_side_by_side_battles(0.1)],
            [],
        )

        engine._set_state_total_progress(
            attacker,
            defender.total_progress + 5.3 / engine.track_length_m,
        )
        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertTrue(battle.pass_confirmed)
        self.assertTrue(battle.pass_event_emitted)
        self.assertGreaterEqual(battle.pass_clearance_m, 5.10)
        self.assertEqual(attacker.position, 1)
        self.assertEqual(defender.position, 2)
        self.assertEqual([event.type for event in events], ["pass"])
        self.assertIn("P1", events[0].message)
        rendered_attacker = next(
            position
            for position in engine.build_tick_state().positions
            if position.driver_id == attacker.driver_id
        )
        self.assertEqual(rendered_attacker.position, 1)
        self.assertEqual(rendered_attacker.maneuver_phase, "clear")
        self.assertEqual(
            [
                event.type
                for event in engine._tick_side_by_side_battles(
                    GAME_TICK_SECONDS,
                )
            ],
            [],
        )

    def test_local_yellow_blocks_candidates_and_aborts_unconfirmed_battle(self) -> None:
        engine = _make_engine_for_circuit(3)
        defender, attacker, hazard = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:3]
        defender.position = 1
        attacker.position = 2
        hazard.position = 3
        engine._set_state_total_progress(attacker, 1.2)
        engine._set_state_total_progress(
            defender,
            attacker.total_progress + 4.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            hazard,
            attacker.total_progress + 100.0 / engine.track_length_m,
        )
        hazard.retired = True
        hazard.vehicle_status = "stopped_on_track"
        hazard.hazard_active = True
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None

        self.assertTrue(engine._local_yellow_active_for(attacker))
        self.assertTrue(engine._local_yellow_active_for(defender))
        self.assertFalse(
            engine._overtaking_candidate_allowed(attacker, defender)
        )
        attacker.lateral_offset_m = (
            defender.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )
        self.assertFalse(
            engine._physics_v2_passing_authorized(attacker, defender)
        )
        self.assertEqual(
            engine._battle_event_probability(attacker, defender, 0.4),
            0.0,
        )

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertEqual(battle.phase, "abort")
        self.assertFalse(battle.pass_confirmed)
        self.assertEqual([event.type for event in events], ["overtake_abort"])
        rendered = {
            position.driver_id: position
            for position in engine.build_tick_state().positions
        }
        self.assertTrue(rendered[attacker.driver_id].local_yellow_active)
        self.assertTrue(rendered[defender.driver_id].local_yellow_active)

    def test_inside_line_uses_physical_profile_without_lap_time_delta(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = car_ahead.total_progress - 10.0 / engine.track_length_m

        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="pull_out",
        )

        self.assertEqual(engine._physics_v2_virtual_line(attacker), ATTACK_LINE_INSIDE)
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_outside_line_uses_physical_profile_without_lap_time_delta(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = car_ahead.total_progress - 10.0 / engine.track_length_m

        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_OUTSIDE,
            DEFENDER_LINE_RACING,
            phase="pull_out",
        )

        self.assertEqual(engine._physics_v2_virtual_line(attacker), ATTACK_LINE_OUTSIDE)
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_defensive_line_uses_physical_profile_without_lap_time_delta(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = car_ahead.total_progress - 10.0 / engine.track_length_m

        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_DEFENSIVE,
            phase="pull_out",
        )

        self.assertEqual(
            engine._physics_v2_virtual_line(car_ahead),
            DEFENDER_LINE_DEFENSIVE,
        )
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_timed_out_maneuver_aborts_without_random_run_wide(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = car_ahead.total_progress - 10.0 / engine.track_length_m
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            "T1 Braking",
            phase="pull_out",
        )
        events = engine._tick_side_by_side_battles(17.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "overtake_abort")
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_approach_time_does_not_consume_committed_maneuver_timer(self) -> None:
        engine = _make_engine_for_circuit(3)
        defender = engine.driver_states[4]
        attacker = engine.driver_states[3]
        defender.position = 1
        attacker.position = 2
        engine._set_state_total_progress(defender, 1.2)
        engine._set_state_total_progress(
            attacker,
            1.2 - 20.0 / engine.track_length_m,
        )
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
        )

        engine._tick_side_by_side_battles(4.0)
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        self.assertEqual(battle.phase, "approach")
        self.assertEqual(battle.committed_seconds, 0.0)

        _place_close_pair(engine, defender, attacker, 1.2, bumper_gap_m=1.5)
        attacker.lateral_offset_m = (
            defender.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        )
        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        self.assertEqual(battle.phase, "pull_out")
        self.assertEqual(battle.committed_seconds, 0.0)

        self.assertEqual(engine._tick_side_by_side_battles(15.9), [])
        self.assertEqual(battle.phase, "pull_out")
        events = engine._tick_side_by_side_battles(0.2)
        self.assertEqual([event.type for event in events], ["overtake_abort"])
        self.assertEqual(battle.phase, "abort")

    def test_lateral_clearance_creates_side_by_side_event(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = car_ahead.total_progress - 10.0 / engine.track_length_m
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_OUTSIDE,
            DEFENDER_LINE_RACING,
            "T1 Braking",
            phase="pull_out",
        )
        attacker.lateral_offset_m = car_ahead.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3
        attacker.total_progress = car_ahead.total_progress - 4.0 / engine.track_length_m

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "side_by_side")
        self.assertTrue(engine.build_tick_state().positions[1].side_by_side_active)

    def test_maneuver_does_not_generate_contact_without_body_overlap(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = 1.130
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_DEFENSIVE,
            "T1 Braking",
            phase="overlap",
        )
        attacker.lateral_offset_m = car_ahead.lateral_offset_m + PHYSICAL_CAR_WIDTH_M + 0.3

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertEqual(events, [])
        self.assertNotIn(attacker.driver_id, engine._battle_effects)
        self.assertNotIn(car_ahead.driver_id, engine._battle_effects)

    def test_swept_body_contact_stops_cars_at_impact_and_emits_event(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        first, second = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {first.driver_id, second.driver_id}:
                state.retired = True
        first.position = 1
        second.position = 2
        engine._set_state_total_progress(first, 1.2)
        engine._set_state_total_progress(
            second,
            1.2 - 5.5 / engine.track_length_m,
        )
        first.lateral_offset_m = second.lateral_offset_m = 0.0
        first.slip_angle_rad = second.slip_angle_rad = 0.0
        first.speed_kph = 144.0
        second.speed_kph = 172.8
        start = {
            first.driver_id: engine._body_pose(first),
            second.driver_id: engine._body_pose(second),
        }
        engine._set_state_total_progress(
            first,
            first.total_progress + 4.0 / engine.track_length_m,
        )
        intended_second_end = second.total_progress + 4.8 / engine.track_length_m
        engine._set_state_total_progress(second, intended_second_end)
        engine.rng.random = lambda: 0.99

        events = engine._resolve_vehicle_collisions(start, 0.1)

        self.assertEqual([event.type for event in events], ["minor_contact"])
        self.assertTrue(first.contact_active)
        self.assertTrue(second.contact_active)
        self.assertEqual(first.contact_opponent_id, second.driver_id)
        self.assertEqual(first.contact_type, "front_rear")
        self.assertGreater(first.contact_impact_speed_mps, 0.0)
        self.assertGreater(first.collision_damage, 0.0)
        self.assertGreater(second.collision_damage, 0.0)
        self.assertGreater(first.speed_kph, 130.0)
        self.assertGreater(second.speed_kph, 130.0)
        self.assertLess(second.total_progress, intended_second_end)
        overlap = oriented_body_overlap(
            engine._body_pose(first),
            engine._body_pose(second),
        )
        self.assertTrue(overlap is None or overlap[0] <= 0.003)

        tick = engine.build_tick_state()
        first_tick = next(
            position
            for position in tick.positions
            if position.driver_id == first.driver_id
        )
        self.assertTrue(first_tick.contact_active)
        self.assertGreater(first_tick.contact_impact_speed_mps, 0.0)
        self.assertEqual(first_tick.collision_damage, first.collision_damage)

    def test_high_impact_body_contact_can_escalate_to_crash(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        first, second = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {first.driver_id, second.driver_id}:
                state.retired = True
        engine._set_state_total_progress(first, 1.2)
        engine._set_state_total_progress(
            second,
            1.2 - 10.0 / engine.track_length_m,
        )
        first.lateral_offset_m = second.lateral_offset_m = 0.0
        first.speed_kph = 144.0
        second.speed_kph = 360.0
        start = {
            first.driver_id: engine._body_pose(first),
            second.driver_id: engine._body_pose(second),
        }
        engine._set_state_total_progress(
            first,
            first.total_progress + 4.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            second,
            second.total_progress + 10.0 / engine.track_length_m,
        )
        engine.rng.random = lambda: 0.0

        events = engine._resolve_vehicle_collisions(start, 0.1)

        self.assertIn("collision", [event.type for event in events])
        self.assertTrue(first.retired)
        self.assertTrue(second.retired)
        self.assertEqual(engine.race_phase, "sc")

    def test_lateral_near_miss_does_not_emit_random_contact(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        first, second = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {first.driver_id, second.driver_id}:
                state.retired = True
        engine._set_state_total_progress(first, 1.2)
        engine._set_state_total_progress(
            second,
            1.2 - 4.0 / engine.track_length_m,
        )
        first.lateral_offset_m = 0.0
        second.lateral_offset_m = 2.2
        first.speed_kph = second.speed_kph = 220.0
        start = {
            first.driver_id: engine._body_pose(first),
            second.driver_id: engine._body_pose(second),
        }
        engine._set_state_total_progress(
            first,
            first.total_progress + 6.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            second,
            second.total_progress + 6.0 / engine.track_length_m,
        )

        events = engine._resolve_vehicle_collisions(start, 0.1)

        self.assertEqual(events, [])
        self.assertFalse(first.contact_active)
        self.assertFalse(second.contact_active)

    def test_static_side_overlap_separates_without_artificial_lateral_kick(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        first, second = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {first.driver_id, second.driver_id}:
                state.retired = True
        for state in (first, second):
            engine._set_state_total_progress(state, 1.2)
            state.speed_kph = 200.0
            state.lateral_speed_mps = 0.0
            state.slip_angle_rad = 0.0
        first.lateral_offset_m = 0.0
        second.lateral_offset_m = first.car_width_m - 0.08
        start = {
            first.driver_id: engine._body_pose(first),
            second.driver_id: engine._body_pose(second),
        }

        events = engine._resolve_vehicle_collisions(start, 0.1)

        self.assertEqual([event.type for event in events], ["minor_contact"])
        self.assertAlmostEqual(first.lateral_speed_mps, 0.0, places=6)
        self.assertAlmostEqual(second.lateral_speed_mps, 0.0, places=6)
        self.assertAlmostEqual(first.slip_angle_rad, 0.0, places=6)
        self.assertAlmostEqual(second.slip_angle_rad, 0.0, places=6)
        overlap = oriented_body_overlap(
            engine._body_pose(first),
            engine._body_pose(second),
        )
        self.assertTrue(overlap is None or overlap[0] <= 0.003)

    def test_contact_pair_cooldown_prevents_duplicate_feed_event(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        first, second = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        for state in engine.driver_states.values():
            if state.driver_id not in {first.driver_id, second.driver_id}:
                state.retired = True
        engine._set_state_total_progress(first, 1.2)
        engine._set_state_total_progress(
            second,
            1.2 - 4.5 / engine.track_length_m,
        )
        first.lateral_offset_m = second.lateral_offset_m = 0.0
        first.speed_kph = second.speed_kph = 200.0
        start = {
            first.driver_id: engine._body_pose(first),
            second.driver_id: engine._body_pose(second),
        }

        first_events = engine._resolve_vehicle_collisions(start, 0.1)
        second_events = engine._resolve_vehicle_collisions(start, 0.1)

        self.assertEqual([event.type for event in first_events], ["minor_contact"])
        self.assertEqual(second_events, [])

    def test_maneuver_group_keeps_four_car_chain_without_pair_eviction(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        cars = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:5]
        active_ids = {state.driver_id for state in cars}
        for state in engine.driver_states.values():
            if state.driver_id not in active_ids:
                state.retired = True

        straight = max(
            (
                segment
                for segment in engine.circuit.segments
                if segment.type == TrackSegmentType.STRAIGHT
                and segment.side_by_side_allowed
            ),
            key=lambda segment: (
                (segment.end - segment.start) % 1.0
            ),
        )
        straight_progress = (
            straight.start + 0.5 * ((straight.end - straight.start) % 1.0)
        ) % 1.0

        lateral_offsets = (-3.3, -1.1, 1.1, 3.3, 0.0)
        for index, state in enumerate(cars):
            engine._set_state_total_progress(
                state,
                1.0 + straight_progress - index * 1.0 / engine.track_length_m,
            )
            state.lateral_offset_m = lateral_offsets[index]
            state.target_lateral_offset_m = lateral_offsets[index]
            state.speed_kph = 250.0

        engine._start_side_by_side_battle(
            cars[1].driver_id,
            cars[0].driver_id,
            phase="overlap",
        )
        engine._start_side_by_side_battle(
            cars[2].driver_id,
            cars[1].driver_id,
            phase="overlap",
        )
        engine._start_side_by_side_battle(
            cars[3].driver_id,
            cars[2].driver_id,
            phase="overlap",
        )
        # A fifth connected car exceeds the explicit 4-wide contract.
        engine._start_side_by_side_battle(
            cars[4].driver_id,
            cars[3].driver_id,
            phase="overlap",
        )

        events = engine._rebuild_maneuver_groups()

        self.assertEqual(len(engine._side_by_side_battles), 3)
        middle_edges = [
            battle
            for battle in engine._side_by_side_battles.values()
            if cars[1].driver_id in (battle.attacker_id, battle.defender_id)
        ]
        self.assertEqual(len(middle_edges), 2)
        group = engine._maneuver_group_for_driver(cars[1].driver_id)
        assert group is not None
        self.assertEqual(group.size, 4)
        self.assertEqual(group.phase, "four_wide")
        self.assertEqual(set(group.member_ids), {state.driver_id for state in cars[:4]})
        self.assertEqual([event.type for event in events], ["maneuver_group_formed"])
        middle = cars[1]
        self.assertEqual(
            engine._physics_v2_virtual_line(middle),
            DRIVING_LINE_RACING,
        )
        self.assertTrue(
            engine._local_trajectory_planning_allowed(
                middle,
                DRIVING_LINE_RACING,
            )
        )

        tick = engine.build_tick_state(events)
        telemetry = {
            position.driver_id: position
            for position in tick.positions
            if position.driver_id in group.member_ids
        }
        self.assertTrue(all(item.maneuver_group_size == 4 for item in telemetry.values()))
        self.assertEqual(
            {item.maneuver_group_corridor_index for item in telemetry.values()},
            {0, 1, 2, 3},
        )

        corner = next(
            segment
            for segment in engine.circuit.segments
            if segment.type != TrackSegmentType.STRAIGHT
            and segment.side_by_side_allowed
        )
        corner_progress = (
            corner.start + 0.5 * ((corner.end - corner.start) % 1.0)
        ) % 1.0
        for index, state in enumerate(cars[:4]):
            engine._set_state_total_progress(
                state,
                1.0 + corner_progress - index * 1.0 / engine.track_length_m,
            )
        engine._rebuild_maneuver_groups()
        corner_group = engine._maneuver_group_for_driver(middle.driver_id)
        assert corner_group is not None
        self.assertNotEqual(corner_group.corner_turn_direction, 0)
        self.assertEqual(corner_group.corner_priority_ids[0], cars[0].driver_id)
        self.assertTrue(
            engine._local_trajectory_planning_allowed(
                middle,
                DRIVING_LINE_RACING,
            )
        )

    def test_bahrain_two_three_four_wide_fixed_step_golden(self) -> None:
        """Keep wide occupancy deterministic on the main straight and in T1."""

        def prepare(count: int, progress: float):
            engine = _make_engine_for_circuit(3, seed=99)
            cars = sorted(
                engine.driver_states.values(),
                key=lambda state: state.position,
            )[:count]
            active_ids = {state.driver_id for state in cars}
            for state in engine.driver_states.values():
                if state.driver_id not in active_ids:
                    state.retired = True
            offsets_by_count = {
                2: (-1.1, 1.1),
                3: (-2.2, 0.0, 2.2),
                4: (-3.3, -1.1, 1.1, 3.3),
            }
            for index, state in enumerate(cars):
                engine._set_state_total_progress(
                    state,
                    1.0 + progress - index / engine.track_length_m,
                )
                state.lateral_offset_m = offsets_by_count[count][index]
                state.target_lateral_offset_m = offsets_by_count[count][index]
                state.speed_kph = 100.0 if progress == 0.14 else 250.0
            for index in range(1, count):
                engine._start_side_by_side_battle(
                    cars[index].driver_id,
                    cars[index - 1].driver_id,
                    phase="overlap",
                )
            formation_events = engine._rebuild_maneuver_groups()
            return engine, cars, formation_events

        def snapshot(engine: RaceEngine, cars) -> tuple:
            group = engine._maneuver_group_for_driver(cars[0].driver_id)
            assert group is not None
            return (
                group.member_ids,
                group.phase,
                group.corner_turn_direction,
                group.corner_priority_ids,
                tuple(
                    (
                        state.driver_id,
                        round(state.total_progress, 9),
                        round(state.lateral_offset_m, 6),
                        round(state.speed_kph, 6),
                        state.contact_active,
                        state.off_track,
                    )
                    for state in cars
                ),
            )

        expected_phases = {2: "overlap", 3: "three_wide", 4: "four_wide"}
        for progress, expected_turn_direction in ((0.03, 0), (0.14, 1)):
            for count in (2, 3, 4):
                with self.subTest(
                    section="T1" if progress == 0.14 else "main_straight",
                    count=count,
                ):
                    batch, batch_cars, batch_formation = prepare(count, progress)
                    stepped, stepped_cars, stepped_formation = prepare(count, progress)

                    batch_events = batch.tick(0.1)
                    stepped_events = []
                    for _ in range(5):
                        stepped_events.extend(stepped.tick(0.02))

                    batch_snapshot = snapshot(batch, batch_cars)
                    stepped_snapshot = snapshot(stepped, stepped_cars)
                    expected_member_ids = tuple(
                        state.driver_id for state in batch_cars
                    )
                    self.assertEqual(batch_snapshot, stepped_snapshot)
                    self.assertEqual(batch_snapshot[0], expected_member_ids)
                    self.assertEqual(batch_snapshot[1], expected_phases[count])
                    self.assertEqual(
                        batch_snapshot[2],
                        expected_turn_direction,
                    )
                    self.assertEqual(
                        batch_snapshot[3],
                        expected_member_ids if expected_turn_direction else (),
                    )
                    self.assertEqual(
                        [event.type for event in batch_formation],
                        (
                            []
                            if count == 2
                            else ["maneuver_group_formed"]
                        ),
                    )
                    self.assertEqual(
                        [event.type for event in stepped_formation],
                        [event.type for event in batch_formation],
                    )
                    self.assertEqual(
                        [event.type for event in batch_events],
                        [],
                    )
                    self.assertEqual(
                        [event.type for event in stepped_events],
                        [event.type for event in batch_events],
                    )
                    lateral_offsets = sorted(
                        state.lateral_offset_m for state in batch_cars
                    )
                    self.assertTrue(
                        all(
                            right - left >= PHYSICAL_CAR_WIDTH_M + 0.1
                            for left, right in zip(
                                lateral_offsets,
                                lateral_offsets[1:],
                            )
                        )
                    )
                    self.assertTrue(
                        all(
                            not state.contact_active and not state.off_track
                            for state in batch_cars
                        )
                    )

    def test_group_forced_wide_records_the_actual_inside_neighbor(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        outside, middle, inside = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:3]
        positive_corner = max(
            engine._track_physics.samples,
            key=lambda sample: sample.turn_signal,
        )
        for index, state in enumerate((outside, middle, inside)):
            engine._set_state_total_progress(
                state,
                1.0 + positive_corner.progress - index / engine.track_length_m,
            )
        track = engine._track_physics.at_progress(outside.progress)
        outside.lateral_offset_m = -track.right_width_m - 1.0
        middle.lateral_offset_m = outside.lateral_offset_m + 2.0
        inside.lateral_offset_m = middle.lateral_offset_m + 2.0
        engine._start_side_by_side_battle(
            middle.driver_id,
            outside.driver_id,
            phase="overlap",
        )
        engine._start_side_by_side_battle(
            inside.driver_id,
            middle.driver_id,
            phase="overlap",
        )
        engine._rebuild_maneuver_groups()
        group = engine._maneuver_group_for_driver(outside.driver_id)
        assert group is not None
        self.assertEqual(group.size, 3)
        self.assertEqual(group.corner_turn_direction, 1)
        surface = engine._track_surface.vehicle_state(
            progress=outside.progress,
            lateral_offset_m=outside.lateral_offset_m,
            track_length_m=engine.track_length_m,
        )

        engine._apply_surface_state(outside, surface)

        self.assertTrue(outside.off_track)
        self.assertEqual(outside.off_track_cause, "forced_wide")
        self.assertEqual(
            engine._forced_wide_by_driver[outside.driver_id],
            middle.driver_id,
        )
        self.assertIn(
            engine._battle_pair_key(outside.driver_id, middle.driver_id),
            engine._forced_wide_aftermaths,
        )
        self.assertEqual(
            [event.type for event in engine._pending_forced_wide_events],
            ["forced_wide"],
        )
        telemetry = next(
            item
            for item in engine.build_tick_state().positions
            if item.driver_id == outside.driver_id
        )
        self.assertEqual(telemetry.forced_wide_by_driver_id, middle.driver_id)

    def test_group_corner_capacity_makes_lowest_priority_car_yield(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        cars = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:3]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        for index, state in enumerate(cars):
            engine._set_state_total_progress(
                state,
                1.0 + corner.progress - index / engine.track_length_m,
            )
            state.lateral_offset_m = -2.2 + 2.2 * index
        engine._start_side_by_side_battle(
            cars[1].driver_id,
            cars[0].driver_id,
            phase="overlap",
        )
        engine._start_side_by_side_battle(
            cars[2].driver_id,
            cars[1].driver_id,
            phase="overlap",
        )
        engine._rebuild_maneuver_groups()
        group = engine._maneuver_group_for_driver(cars[1].driver_id)
        assert group is not None
        yielding_driver_id = group.corner_priority_ids[-1]
        original_capacity = engine._maneuver_group_capacity_for
        engine._maneuver_group_capacity_for = lambda _member_ids: 2  # type: ignore[method-assign]
        try:
            events = engine._enforce_maneuver_group_corner_capacity()
        finally:
            engine._maneuver_group_capacity_for = original_capacity  # type: ignore[method-assign]

        self.assertEqual(
            [event.type for event in events],
            ["maneuver_group_corner_yield"],
        )
        self.assertEqual(events[0].payload["driver_id"], yielding_driver_id)
        yielding_edges = [
            battle
            for battle in engine._side_by_side_battles.values()
            if yielding_driver_id in (battle.attacker_id, battle.defender_id)
        ]
        self.assertTrue(yielding_edges)
        self.assertTrue(all(battle.phase == "abort" for battle in yielding_edges))

    def test_sc_settles_existing_body_overlap_without_contact_event(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        first, second, third = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:3]
        active_ids = {first.driver_id, second.driver_id, third.driver_id}
        for state in engine.driver_states.values():
            if state.driver_id not in active_ids:
                state.retired = True
        for index, state in enumerate((first, second, third)):
            engine._set_state_total_progress(
                state,
                1.2 - index * 4.5 / engine.track_length_m,
            )
            state.lateral_offset_m = 0.0
            state.slip_angle_rad = 0.0
        engine.race_phase = "sc"
        start = {state.driver_id: engine._body_pose(state) for state in (first, second, third)}

        events = engine._resolve_vehicle_collisions(start, 0.1)

        self.assertEqual(events, [])
        self.assertFalse(first.contact_active)
        for ahead, behind in ((first, second), (second, third)):
            overlap = oriented_body_overlap(
                engine._body_pose(ahead),
                engine._body_pose(behind),
            )
            self.assertTrue(overlap is None or overlap[0] <= 0.003)

    def test_overlap_earns_corner_corridors_when_front_axle_is_alongside(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        defender_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 4.0 / engine.track_length_m,
        )
        attacker.brake = 0.6
        defender.brake = 0.5
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True
        corridor_targets = engine._corner_corridor_targets(
            battle,
            attacker.progress,
        )
        assert corridor_targets is not None
        attacker.lateral_offset_m = corridor_targets[attacker.driver_id]
        defender.lateral_offset_m = corridor_targets[defender.driver_id]
        attacker.speed_kph = 80.0
        defender.speed_kph = 80.0

        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertTrue(battle.corner_authorized)
        self.assertTrue(battle.corner_active)
        self.assertEqual(battle.corner_prediction_horizon_seconds, 2.5)
        self.assertGreater(battle.corner_prediction_minimum_clearance_m, 0.0)
        self.assertGreaterEqual(battle.corner_prediction_braking_margin_m, 0.0)
        self.assertEqual(battle.corner_prediction_rejection_reason, "")
        # Corner allocation is a corridor, not a second defensive move.  The
        # defender retains the line selected before braking while the exposed
        # corridor reports which car owns the outside.
        self.assertEqual(battle.defender_line, DEFENDER_LINE_RACING)
        self.assertTrue(battle.line_committed)
        targets = engine._corner_corridor_targets(battle, attacker.progress)
        assert targets is not None
        self.assertGreaterEqual(
            abs(targets[attacker.driver_id] - targets[defender.driver_id]),
            PHYSICAL_CAR_WIDTH_M + 0.35 - 1e-6,
        )

        tick = engine.build_tick_state()
        attacker_tick = next(
            position
            for position in tick.positions
            if position.driver_id == attacker.driver_id
        )
        defender_tick = next(
            position
            for position in tick.positions
            if position.driver_id == defender.driver_id
        )
        self.assertTrue(attacker_tick.maneuver_corner_active)
        self.assertEqual(attacker_tick.maneuver_corridor, "inside")
        self.assertEqual(defender_tick.maneuver_corridor, "outside")

    def test_bahrain_t1_can_choose_outside_when_racing_line_is_occupied(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._set_state_total_progress(attacker, 1.10)
        engine._set_state_total_progress(
            defender,
            attacker.total_progress + 8.0 / engine.track_length_m,
        )
        defender.lateral_offset_m = engine._track_physics_for_driver(
            defender
        ).line_offset_at_progress(DRIVING_LINE_RACING, defender.progress)
        defender.lateral_speed_mps = 0.0

        self.assertEqual(
            engine._choose_attack_line(attacker, defender),
            ATTACK_LINE_OUTSIDE,
        )

    def test_corner_line_commitment_preserves_one_move_before_braking(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._set_state_total_progress(attacker, 1.09)
        engine._set_state_total_progress(
            defender,
            attacker.total_progress + 4.0 / engine.track_length_m,
        )
        attacker.speed_kph = defender.speed_kph = 270.0
        defender.lateral_offset_m = -1.2
        attacker.lateral_offset_m = 1.2
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None

        engine._commit_maneuver_lines_before_braking(battle)

        self.assertTrue(battle.line_committed)
        committed_bias_m = battle.attacker_committed_lateral_bias_m
        engine._set_state_total_progress(
            attacker,
            attacker.total_progress + 10.0 / engine.track_length_m,
        )
        defender.lateral_offset_m = 3.0
        expected_target_m = (
            engine._track_physics_for_driver(attacker).line_offset_at_progress(
                DRIVING_LINE_RACING,
                attacker.progress,
            )
            + committed_bias_m
        )
        target_m = engine._physics_v2_target_lateral_offset(
            attacker,
            engine._track_physics.at_progress(attacker.progress),
            DRIVING_LINE_RACING,
            defender,
            None,
        )
        self.assertAlmostEqual(target_m, expected_target_m, places=6)

    def test_corner_rejects_attack_without_required_longitudinal_overlap(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        defender_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 5.9 / engine.track_length_m,
        )
        attacker.brake = 0.5
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertFalse(battle.corner_authorized)
        self.assertEqual(battle.phase, "yield")
        self.assertEqual([event.type for event in events], ["overtake_abort"])

    def test_rejected_corner_holds_corridor_then_smoothly_rejoins(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        defender_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 5.9 / engine.track_length_m,
        )
        attacker.lateral_offset_m = 1.15
        defender.lateral_offset_m = -1.15
        attacker.brake = 0.5
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True

        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertEqual(battle.phase, "yield")
        track_sample = engine._track_physics.at_progress(attacker.progress)
        held_target_m = engine._physics_v2_target_lateral_offset(
            attacker,
            track_sample,
            DRIVING_LINE_RACING,
            defender,
            None,
        )
        self.assertAlmostEqual(held_target_m, 1.15, places=6)

        full_rear_gap_m = PHYSICAL_CAR_LENGTH_M + 1.35
        engine._set_state_total_progress(
            attacker,
            defender.total_progress - full_rear_gap_m / engine.track_length_m,
        )
        engine._tick_side_by_side_battles(0.2)
        self.assertEqual(battle.phase, "abort")

        rejoin_start_m = engine._physics_v2_target_lateral_offset(
            attacker,
            engine._track_physics.at_progress(attacker.progress),
            DRIVING_LINE_RACING,
            defender,
            None,
        )
        self.assertAlmostEqual(rejoin_start_m, 1.15, places=6)
        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        self.assertTrue(battle.abort_rejoin_started)
        targets_m: list[float] = []
        for step in range(64):
            battle.abort_rejoin_elapsed_seconds = step * GAME_TICK_SECONDS
            targets_m.append(
                engine._physics_v2_target_lateral_offset(
                    attacker,
                    engine._track_physics.at_progress(attacker.progress),
                    DRIVING_LINE_RACING,
                    defender,
                    None,
                )
            )

        largest_target_step_m = max(
            abs(current - previous)
            for previous, current in zip(targets_m, targets_m[1:])
        )
        # The old direct overlap -> abort transition jumped by roughly 3.4 m
        # in one tick at Bahrain T1.  Rejoin now moves the target progressively.
        self.assertLess(largest_target_step_m, 0.25)
        battle.abort_rejoin_elapsed_seconds = 2.0
        settled_target_m = engine._physics_v2_target_lateral_offset(
            attacker,
            engine._track_physics.at_progress(attacker.progress),
            DRIVING_LINE_RACING,
            defender,
            None,
        )
        self.assertAlmostEqual(targets_m[-1], settled_target_m, places=6)

    def test_corner_prediction_rejects_insufficient_braking_distance(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        defender_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 4.0 / engine.track_length_m,
        )
        attacker.brake = defender.brake = 0.6
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True
        targets = engine._corner_corridor_targets(battle, attacker.progress)
        assert targets is not None
        attacker.lateral_offset_m = targets[attacker.driver_id]
        defender.lateral_offset_m = targets[defender.driver_id]
        attacker.speed_kph = defender.speed_kph = 320.0

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertFalse(battle.corner_authorized)
        self.assertEqual(
            battle.corner_prediction_rejection_reason,
            "insufficient_braking",
        )
        self.assertLess(battle.corner_prediction_braking_margin_m, 0.0)
        self.assertEqual(battle.phase, "yield")
        self.assertEqual([event.type for event in events], ["overtake_abort"])

    def test_corner_prediction_rejects_overlapping_body_occupancy(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        defender_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 4.0 / engine.track_length_m,
        )
        attacker.speed_kph = defender.speed_kph = 80.0
        attacker.brake = defender.brake = 0.5
        attacker.lateral_offset_m = defender.lateral_offset_m = 0.0
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True

        events = engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertFalse(battle.corner_authorized)
        self.assertEqual(
            battle.corner_prediction_rejection_reason,
            "predicted_collision",
        )
        self.assertEqual(battle.phase, "yield")
        self.assertEqual([event.type for event in events], ["overtake_abort"])

    def test_corner_battle_replans_local_paths_without_legacy_lines(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        defender_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, defender_total)
        engine._set_state_total_progress(
            attacker,
            defender_total - 4.0 / engine.track_length_m,
        )
        attacker.speed_kph = defender.speed_kph = 80.0
        attacker.brake = defender.brake = 0.5
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True
        bootstrap_targets = engine._corner_corridor_targets(
            battle,
            attacker.progress,
        )
        assert bootstrap_targets is not None
        attacker.lateral_offset_m = bootstrap_targets[attacker.driver_id]
        defender.lateral_offset_m = bootstrap_targets[defender.driver_id]
        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        self.assertTrue(battle.corner_active)
        self.assertEqual(
            engine._physics_v2_virtual_line(attacker),
            DRIVING_LINE_RACING,
        )
        self.assertEqual(
            engine._physics_v2_virtual_line(defender),
            DRIVING_LINE_RACING,
        )
        for state in (attacker, defender):
            engine._update_local_trajectory_plan(
                state,
                DRIVING_LINE_RACING,
                engine._physics_v2_modifiers(state),
                LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
            )

        attacker_plan = engine._local_trajectory_plans[attacker.driver_id]
        defender_plan = engine._local_trajectory_plans[defender.driver_id]
        self.assertTrue(
            attacker_plan.selected.viable,
            [
                (
                    candidate.candidate_id,
                    candidate.body_boundary_violations,
                    candidate.predicted_collision_count,
                    candidate.minimum_opponent_clearance_m,
                    candidate.first_collision_time_seconds,
                )
                for candidate in attacker_plan.candidates
            ],
        )
        self.assertTrue(
            defender_plan.selected.viable,
            [
                (
                    candidate.candidate_id,
                    candidate.body_boundary_violations,
                    candidate.predicted_collision_count,
                    candidate.minimum_opponent_clearance_m,
                )
                for candidate in defender_plan.candidates
            ],
        )
        self.assertTrue(
            attacker_plan.selected_candidate_id.startswith("inside_")
        )
        self.assertTrue(
            defender_plan.selected_candidate_id.startswith("outside_")
        )
        self.assertEqual(
            battle.corner_attacker_trajectory_candidate_id,
            attacker_plan.selected_candidate_id,
        )
        self.assertEqual(
            battle.corner_defender_trajectory_candidate_id,
            defender_plan.selected_candidate_id,
        )
        self.assertEqual(battle.corner_trajectory_replan_count, 2)
        attacker_target = engine._physics_v2_target_lateral_offset(
            attacker,
            engine._track_physics.at_progress(attacker.progress),
            DRIVING_LINE_RACING,
            defender,
            None,
        )
        self.assertAlmostEqual(
            attacker_target,
            attacker_plan.selected.control_target_lateral_offset_m,
        )

    def test_active_corner_path_is_recomputed_after_vehicle_state_changes(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        engine._set_state_total_progress(defender, 1.0 + corner.progress)
        engine._set_state_total_progress(
            attacker,
            defender.total_progress - 4.0 / engine.track_length_m,
        )
        attacker.speed_kph = defender.speed_kph = 80.0
        attacker.brake = defender.brake = 0.5
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_entry_candidate = True
        targets = engine._corner_corridor_targets(battle, attacker.progress)
        assert targets is not None
        attacker.lateral_offset_m = targets[attacker.driver_id]
        defender.lateral_offset_m = targets[defender.driver_id]
        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)
        engine._update_local_trajectory_plan(
            attacker,
            DRIVING_LINE_RACING,
            engine._physics_v2_modifiers(attacker),
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
        )
        first_plan = engine._local_trajectory_plans[attacker.driver_id]

        attacker.tire_wear = 0.65
        attacker.tire_lateral_grip = 0.82
        engine._set_state_total_progress(
            attacker,
            attacker.total_progress + 8.0 / engine.track_length_m,
        )
        engine._update_local_trajectory_plan(
            attacker,
            DRIVING_LINE_RACING,
            engine._physics_v2_modifiers(attacker),
            LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS,
        )
        second_plan = engine._local_trajectory_plans[attacker.driver_id]

        self.assertIsNot(second_plan, first_plan)
        self.assertNotEqual(
            second_plan.selected.longitudinal_samples[0].total_progress,
            first_plan.selected.longitudinal_samples[0].total_progress,
        )
        self.assertEqual(battle.corner_trajectory_replan_count, 2)

    def test_full_car_advantage_is_held_until_corner_exit(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: abs(sample.turn_signal),
        )
        corner_total = 1.0 + corner.progress
        engine._set_state_total_progress(defender, corner_total)
        engine._set_state_total_progress(
            attacker,
            corner_total + 5.1 / engine.track_length_m,
        )
        attacker.position = 1
        defender.position = 2
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_active = True
        battle.corner_authorized = True
        battle.corner_turn_direction = 1 if corner.turn_signal > 0 else -1

        self.assertEqual(engine._tick_side_by_side_battles(GAME_TICK_SECONDS), [])
        self.assertEqual(battle.phase, "overlap")

        straight = next(
            sample
            for sample in engine._track_physics.samples
            if abs(sample.turn_signal) < 0.01
            and all(
                abs(
                    engine._track_physics.at_progress(
                        sample.progress + distance_m / engine.track_length_m
                    ).turn_signal
                )
                < 0.08
                for distance_m in (20.0, 40.0, 70.0)
            )
        )
        straight_total = 1.0 + straight.progress
        engine._set_state_total_progress(defender, straight_total)
        engine._set_state_total_progress(
            attacker,
            straight_total + 5.3 / engine.track_length_m,
        )
        events = engine._tick_side_by_side_battles(0.31)

        self.assertEqual(battle.phase, "clear")
        self.assertEqual([event.type for event in events], ["pass"])

    def test_inside_corridor_has_tighter_exit_traction_limit(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_active = True
        battle.corner_authorized = True

        inside = engine._corner_maneuver_factors(attacker)
        outside = engine._corner_maneuver_factors(defender)

        self.assertLess(inside[0], outside[0])
        self.assertLess(inside[1], outside[1])

    def test_inside_role_is_preserved_when_corner_direction_changes(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_active = True
        battle.corner_authorized = True
        positive = max(
            engine._track_physics.samples,
            key=lambda sample: sample.turn_signal,
        )
        negative = min(
            engine._track_physics.samples,
            key=lambda sample: sample.turn_signal,
        )

        positive_targets = engine._corner_corridor_targets(battle, positive.progress)
        negative_targets = engine._corner_corridor_targets(battle, negative.progress)
        assert positive_targets is not None and negative_targets is not None

        self.assertGreater(
            positive_targets[attacker.driver_id],
            positive_targets[defender.driver_id],
        )
        self.assertLess(
            negative_targets[attacker.driver_id],
            negative_targets[defender.driver_id],
        )
        self.assertEqual(
            engine._corner_corridor_for_driver(battle, attacker.driver_id),
            "inside",
        )

    def test_new_corner_direction_keeps_each_car_on_its_current_side(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_active = True
        battle.corner_inside_driver_id = attacker.driver_id
        battle.corner_turn_direction = 1
        attacker.lateral_offset_m = 2.0
        defender.lateral_offset_m = -2.0
        engine._local_trajectory_plans[attacker.driver_id] = SimpleNamespace()
        engine._local_trajectory_plans[defender.driver_id] = SimpleNamespace()

        engine._set_corner_turn_direction(battle, -1)

        self.assertEqual(battle.corner_inside_driver_id, defender.driver_id)
        self.assertEqual(
            engine._corner_corridor_for_driver(battle, defender.driver_id),
            "inside",
        )
        self.assertEqual(
            engine._corner_corridor_for_driver(battle, attacker.driver_id),
            "outside",
        )
        self.assertNotIn(attacker.driver_id, engine._local_trajectory_plans)
        self.assertNotIn(defender.driver_id, engine._local_trajectory_plans)

    def test_outside_car_records_forced_wide_when_opponent_takes_its_space(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        defender, attacker = sorted(
            engine.driver_states.values(),
            key=lambda state: state.position,
        )[:2]
        corner = max(
            engine._track_physics.samples,
            key=lambda sample: sample.turn_signal,
        )
        engine._set_state_total_progress(defender, 1.0 + corner.progress)
        engine._set_state_total_progress(
            attacker,
            defender.total_progress - 2.0 / engine.track_length_m,
        )
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            ATTACK_LINE_INSIDE,
            ATTACK_LINE_OUTSIDE,
            phase="overlap",
        )
        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.corner_active = True
        battle.corner_authorized = True
        track = engine._track_physics.at_progress(defender.progress)
        defender.lateral_offset_m = -track.right_width_m - 1.0
        attacker.lateral_offset_m = defender.lateral_offset_m + 2.0
        surface = engine._track_surface.vehicle_state(
            progress=defender.progress,
            lateral_offset_m=defender.lateral_offset_m,
            track_length_m=engine.track_length_m,
        )

        engine._apply_surface_state(defender, surface)

        self.assertTrue(defender.off_track)
        self.assertEqual(defender.off_track_cause, "forced_wide")
        self.assertEqual(battle.forced_wide_driver_id, defender.driver_id)

    def test_side_by_side_state_clears_after_attacker_gets_ahead(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.136)
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            phase="overlap",
        )

        attacker.position = 1
        car_ahead.position = 2
        attacker.total_progress = car_ahead.total_progress + 5.3 / engine.track_length_m
        car_ahead.total_progress = 1.136
        engine._tick_side_by_side_battles(GAME_TICK_SECONDS)

        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        self.assertIsNotNone(battle)
        self.assertEqual(battle.phase, "clear")

    def test_strong_heavy_braking_attack_can_force_defender_wide(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.136)
        attacker.pace_mode = PaceMode.ATTACK
        rolls = iter([0.0, 1.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.04

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "attack")
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)
        self.assertTrue(engine._is_side_by_side_active(attacker.driver_id))
        self.assertEqual(len(engine._forced_wide_aftermaths), 0)

    def test_forced_wide_can_make_defender_run_wide_on_exit(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = 1.130
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.0])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "run_wide")
        self.assertEqual(events[0].driver, engine._driver_meta[car_ahead.driver_id]["abbreviation"])
        self.assertIn("forced out", events[0].message)
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_forced_wide_can_create_defender_throttle_input_error_on_exit(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = 1.130
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.5])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(events, [])
        input_error = engine._driver_input_errors[car_ahead.driver_id]
        self.assertEqual(input_error.kind, "throttle")
        self.assertEqual(input_error.trigger, "forced_wide_exit")
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_forced_wide_can_cost_attacker_on_tight_exit(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = 1.130
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.8])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(events, [])
        input_error = engine._driver_input_errors[attacker.driver_id]
        self.assertEqual(input_error.kind, "throttle")
        self.assertEqual(input_error.trigger, "tight_exit_after_forcing_wide")
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_forced_wide_does_not_generate_contact_without_body_overlap(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.136
        attacker.total_progress = 1.130
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.95])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(events, [])
        self.assertIn(attacker.driver_id, engine._driver_input_errors)
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_heavy_braking_attack_creates_brake_input_error_before_lockup(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        _place_close_pair(engine, car_ahead, attacker, 1.136)
        attacker.pace_mode = PaceMode.ATTACK
        attacker.tire_usage = 24.0
        rolls = iter([0.0, 0.0])
        engine.rng.random = lambda: next(rolls)

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNone(event)
        input_error = engine._driver_input_errors[attacker.driver_id]
        self.assertEqual(input_error.kind, "brake")
        self.assertEqual(input_error.trigger, "battle_braking_error")
        self.assertEqual(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertEqual(engine._battle_effect_tire_usage_multiplier(attacker), 1.0)

    def test_pass_event_reports_position_gain(self) -> None:
        engine = _make_engine_for_circuit(3)
        previous_positions = {1: 2, 2: 1}
        engine.driver_states[1].position = 1
        engine.driver_states[2].position = 2
        engine._set_state_total_progress(engine.driver_states[2], 1.2)
        engine._set_state_total_progress(
            engine.driver_states[1],
            1.2 + 6.0 / engine.track_length_m,
        )

        events = engine._build_pass_events(previous_positions)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "pass")
        self.assertIn("passes", events[0].message)

    def test_position_swap_does_not_duplicate_maneuver_pass_feed(self) -> None:
        engine = _make_engine_for_circuit(3)
        defender = engine.driver_states[4]
        attacker = engine.driver_states[3]
        previous_positions = {
            attacker.driver_id: 2,
            defender.driver_id: 1,
        }
        attacker.position = 1
        defender.position = 2
        engine._set_state_total_progress(defender, 1.2)
        engine._set_state_total_progress(
            attacker,
            1.2 + 6.0 / engine.track_length_m,
        )
        engine._start_side_by_side_battle(
            attacker.driver_id,
            defender.driver_id,
            phase="overlap",
        )

        self.assertEqual(engine._build_pass_events(previous_positions), [])

        battle = engine._side_by_side_battle_for_driver(attacker.driver_id)
        assert battle is not None
        battle.phase = "clear"
        self.assertEqual(engine._build_pass_events(previous_positions), [])

    def test_tick_activates_drs_when_within_one_second_of_car_ahead(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        chaser = engine.driver_states[2]
        car_ahead.position = 1
        chaser.position = 2
        car_ahead.current_lap = 1
        chaser.current_lap = 1
        car_ahead.progress = 0.058
        chaser.progress = 0.050
        car_ahead.total_progress = 1.058
        chaser.total_progress = 1.050

        engine.tick(GAME_TICK_SECONDS)

        self.assertFalse(chaser.dirty_air_active)
        self.assertTrue(chaser.drs_active)

    def test_dirty_air_can_be_active_without_drs_outside_drs_zone(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        chaser = engine.driver_states[2]
        car_ahead.position = 1
        chaser.position = 2
        car_ahead.current_lap = 1
        chaser.current_lap = 1
        chaser.progress = 0.300
        chaser.total_progress = 1.300
        car_ahead.progress = (
            chaser.progress + 30.0 / engine.track_length_m
        )
        car_ahead.total_progress = (
            chaser.total_progress + 30.0 / engine.track_length_m
        )
        chaser.speed_kph = 280.0
        car_ahead.speed_kph = 280.0
        car_ahead.lateral_offset_m = chaser.lateral_offset_m

        engine._update_wake_state(chaser, car_ahead)

        self.assertTrue(chaser.dirty_air_active)
        self.assertFalse(chaser.drs_active)

    def test_wake_is_tow_dominant_on_straight_and_dirty_in_corner(self) -> None:
        engine = _make_engine()
        leader = engine.driver_states[1]
        chaser = engine.driver_states[2]
        chaser.speed_kph = 280.0
        leader.speed_kph = 280.0
        leader.lateral_offset_m = 0.0
        chaser.lateral_offset_m = 0.0
        gap_progress = 30.0 / engine.track_length_m

        chaser.progress = 0.05
        chaser.total_progress = 1.05
        leader.total_progress = chaser.total_progress + gap_progress
        straight = engine._update_wake_state(chaser, leader)
        straight_modifiers = engine._physics_v2_modifiers(chaser)

        chaser.progress = 0.30
        chaser.total_progress = 1.30
        leader.total_progress = chaser.total_progress + gap_progress
        corner = engine._update_wake_state(chaser, leader)
        corner_modifiers = engine._physics_v2_modifiers(chaser)

        self.assertGreater(straight.tow_strength, straight.dirty_air_strength)
        self.assertGreater(corner.dirty_air_strength, corner.tow_strength)
        self.assertLess(straight_modifiers.drag_multiplier, 1.0)
        self.assertLess(
            corner_modifiers.downforce_multiplier,
            straight_modifiers.downforce_multiplier,
        )
        self.assertLess(corner_modifiers.braking, straight_modifiers.braking)

    def test_bahrain_main_straight_and_t1_wake_golden(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        leader = engine.driver_states[1]
        chaser = engine.driver_states[5]
        leader.speed_kph = chaser.speed_kph = 270.0
        leader.lateral_offset_m = chaser.lateral_offset_m = 0.0
        expected = {
            0.05: (
                0.875293,
                0.875293,
                0.0,
                0.912471,
                1.0,
                1.0,
                1.0,
            ),
            0.14: (
                0.875293,
                0.202491,
                0.841002,
                0.979751,
                0.869645,
                0.93272,
                0.98318,
            ),
        }

        for progress, golden in expected.items():
            with self.subTest(progress=progress):
                engine._set_state_total_progress(chaser, 1.0 + progress)
                engine._set_state_total_progress(
                    leader,
                    chaser.total_progress + 30.0 / engine.track_length_m,
                )
                effects = engine._update_wake_state(chaser, leader)
                actual = tuple(
                    round(value, 6)
                    for value in (
                        effects.wake_strength,
                        effects.tow_strength,
                        effects.dirty_air_strength,
                        effects.drag_multiplier,
                        effects.downforce_multiplier,
                        effects.braking_grip_multiplier,
                        effects.lateral_grip_multiplier,
                    )
                )
                self.assertEqual(actual, golden)
                self.assertEqual(
                    chaser.dirty_air_active,
                    progress == 0.14,
                )

    def test_wake_selects_aligned_second_car_when_immediate_car_pulls_out(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        aligned_source = engine.driver_states[1]
        pulled_out_source = engine.driver_states[5]
        chaser = engine.driver_states[3]
        engine._set_state_total_progress(chaser, 1.05)
        engine._set_state_total_progress(
            pulled_out_source,
            chaser.total_progress + 20.0 / engine.track_length_m,
        )
        engine._set_state_total_progress(
            aligned_source,
            chaser.total_progress + 40.0 / engine.track_length_m,
        )
        chaser.speed_kph = 270.0
        chaser.lateral_offset_m = 0.0
        pulled_out_source.lateral_offset_m = 5.0
        aligned_source.lateral_offset_m = 0.0

        effects = engine._update_wake_state(
            chaser,
            pulled_out_source,
            [pulled_out_source, aligned_source],
        )

        self.assertGreater(effects.wake_strength, 0.0)
        self.assertEqual(chaser.wake_source_driver_id, aligned_source.driver_id)
        self.assertAlmostEqual(chaser.wake_longitudinal_gap_m, 40.0)
        self.assertAlmostEqual(chaser.wake_lateral_separation_m, 0.0)
        telemetry = next(
            item
            for item in engine.build_tick_state().positions
            if item.driver_id == chaser.driver_id
        )
        self.assertEqual(
            telemetry.wake_source_driver_id,
            aligned_source.driver_id,
        )

    def test_pulling_out_laterally_removes_tow_and_dirty_air(self) -> None:
        engine = _make_engine()
        leader = engine.driver_states[1]
        chaser = engine.driver_states[2]
        chaser.speed_kph = 280.0
        chaser.progress = 0.05
        chaser.total_progress = 1.05
        leader.total_progress = chaser.total_progress + 30.0 / engine.track_length_m
        leader.lateral_offset_m = 0.0
        chaser.lateral_offset_m = 5.0

        effects = engine._update_wake_state(chaser, leader)

        self.assertAlmostEqual(effects.wake_strength, 0.0)
        self.assertAlmostEqual(chaser.wake_drag_multiplier, 1.0)
        self.assertAlmostEqual(chaser.wake_downforce_multiplier, 1.0)
        self.assertFalse(chaser.dirty_air_active)

    def test_wake_telemetry_exposes_continuous_strengths(self) -> None:
        engine = _make_engine()
        leader = engine.driver_states[1]
        chaser = engine.driver_states[2]
        chaser.speed_kph = 280.0
        chaser.progress = 0.05
        chaser.total_progress = 1.05
        leader.total_progress = chaser.total_progress + 30.0 / engine.track_length_m
        leader.lateral_offset_m = chaser.lateral_offset_m
        engine._update_wake_state(chaser, leader)

        position = next(
            item
            for item in engine.build_tick_state().positions
            if item.driver_id == chaser.driver_id
        )

        self.assertGreater(position.wake_strength, 0.0)
        self.assertGreater(position.tow_strength, 0.0)
        self.assertEqual(position.dirty_air_strength, 0.0)
        self.assertLess(position.wake_drag_multiplier, 1.0)

    def test_tow_reduces_drag_and_increases_real_straight_acceleration(self) -> None:
        engine = _make_engine()
        leader = engine.driver_states[1]
        chaser = engine.driver_states[2]
        chaser.speed_kph = 288.0
        chaser.progress = 0.05
        chaser.total_progress = 1.05
        leader.total_progress = chaser.total_progress + 30.0 / engine.track_length_m
        leader.lateral_offset_m = chaser.lateral_offset_m
        distance_m = engine._track_physics.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            chaser.total_progress,
        )

        engine._reset_wake_state(chaser)
        clean_modifiers = engine._physics_v2_modifiers(chaser)
        clean = engine._vehicle_physics.advance(
            distance_m=distance_m,
            speed_mps=80.0,
            delta_seconds=0.5,
            modifiers=clean_modifiers,
        )
        engine._update_wake_state(chaser, leader)
        chaser.drs_active = False
        tow_modifiers = engine._physics_v2_modifiers(chaser)
        tow = engine._vehicle_physics.advance(
            distance_m=distance_m,
            speed_mps=80.0,
            delta_seconds=0.5,
            modifiers=tow_modifiers,
        )

        self.assertGreater(tow.speed_mps, clean.speed_mps)

    def test_corner_dirty_air_lowers_real_physical_target_speed(self) -> None:
        engine = _make_engine()
        leader = engine.driver_states[1]
        chaser = engine.driver_states[2]
        chaser.speed_kph = 250.0
        chaser.progress = 0.08
        chaser.total_progress = 1.08
        leader.total_progress = chaser.total_progress + 30.0 / engine.track_length_m
        leader.lateral_offset_m = chaser.lateral_offset_m
        distance_m = engine._track_physics.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            chaser.total_progress,
        )

        engine._reset_wake_state(chaser)
        clean_target = engine._vehicle_physics.target_speed_mps(
            distance_m,
            engine._physics_v2_modifiers(chaser),
        )
        engine._update_wake_state(chaser, leader)
        dirty_target = engine._vehicle_physics.target_speed_mps(
            distance_m,
            engine._physics_v2_modifiers(chaser),
        )

        self.assertGreater(clean_target, dirty_target)

    def test_consistency_controls_lap_random_spread(self) -> None:
        engine = _make_engine()

        self.assertLess(
            engine._consistency_variation_spread(0.96),
            engine._consistency_variation_spread(0.74),
        )

    def test_dead_tire_penalty_only_adds_lap_time_loss(self) -> None:
        engine = _make_engine()

        self.assertEqual(engine._dead_tire_lap_penalty(0.5), 0.0)
        self.assertGreater(engine._dead_tire_lap_penalty(1.0), 0.0)
        self.assertGreater(
            engine._dead_tire_lap_penalty(1.0),
            engine._dead_tire_lap_penalty(0.85),
        )

    def test_tire_wear_updates_during_lap_progress(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        state.tire_usage = 5.0
        start_wear = engine._current_tire_wear(state)

        state.tire_usage = 5.5
        mid_lap_wear = engine._current_tire_wear(state)

        self.assertGreater(mid_lap_wear, start_wear)

    def test_tire_management_reduces_live_wear_for_same_tire_age(self) -> None:
        engine = _make_engine()
        high_management = engine.driver_states[7]
        low_management = engine.driver_states[14]
        _set_test_tire(engine, high_management, TireCompound.HARD)
        _set_test_tire(engine, low_management, TireCompound.HARD)
        high_management.tire_usage = 20.5
        low_management.tire_usage = 20.5

        self.assertLess(
            engine._effective_tire_age(high_management),
            engine._effective_tire_age(low_management),
        )
        self.assertLess(
            engine._current_tire_wear(high_management),
            engine._current_tire_wear(low_management),
        )

    def test_physics_modifiers_use_compound_and_wear_specific_tire_forces(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        engine._tire_random[state.driver_id] = 0.0

        _set_test_tire(engine, state, TireCompound.SOFT)
        state.tire_usage = 0.0
        fresh_soft = engine._physics_v2_modifiers(state)
        state.tire_usage = 20.0
        worn_soft = engine._physics_v2_modifiers(state)
        _set_test_tire(engine, state, TireCompound.HARD)
        state.tire_usage = 0.0
        fresh_hard = engine._physics_v2_modifiers(state)

        self.assertGreater(fresh_soft.grip, fresh_hard.grip)
        self.assertGreater(fresh_soft.traction, fresh_hard.traction)
        self.assertGreater(fresh_soft.braking, fresh_hard.braking)
        self.assertGreater(fresh_soft.grip, worn_soft.grip)
        self.assertGreater(fresh_soft.traction, worn_soft.traction)
        self.assertGreater(fresh_soft.braking, worn_soft.braking)

    def test_tick_telemetry_exposes_separate_tire_grip_channels(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        _set_test_tire(engine, state, TireCompound.SOFT)
        state.tire_usage = 20.0
        engine._tire_random[state.driver_id] = 0.0

        position = next(
            item
            for item in engine.build_tick_state().positions
            if item.driver_id == state.driver_id
        )

        self.assertLess(position.tire_lateral_grip, 1.0)
        self.assertLess(position.tire_traction_grip, position.tire_lateral_grip)
        self.assertGreater(position.tire_braking_grip, position.tire_traction_grip)

    def test_tick_telemetry_normalizes_grip_index_to_fresh_c3(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]
        _set_test_tire(engine, state, TireCompound.MEDIUM)
        state.tire_usage = 0.0
        state.tire_surface_temperature_c = 95.0
        engine._tire_random[state.driver_id] = 0.0

        position = next(
            item
            for item in engine.build_tick_state().positions
            if item.driver_id == state.driver_id
        )

        self.assertEqual(position.physical_tire_compound, "C3")
        self.assertAlmostEqual(position.tire_lateral_grip_index, 1.0)
        self.assertAlmostEqual(position.tire_traction_grip_index, 1.0)
        self.assertAlmostEqual(position.tire_braking_grip_index, 1.0)

    def test_compound_and_wear_change_live_physical_corner_target(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]
        engine._tire_random[state.driver_id] = 0.0

        _set_test_tire(engine, state, TireCompound.SOFT)
        state.tire_usage = 0.0
        fresh_soft = _physics_target_speed_at(engine, state, 0.87)
        _set_test_tire(engine, state, TireCompound.HARD)
        state.tire_usage = 0.0
        fresh_hard = _physics_target_speed_at(engine, state, 0.87)
        _set_test_tire(engine, state, TireCompound.SOFT)
        state.tire_usage = 20.0
        worn_soft = _physics_target_speed_at(engine, state, 0.87)

        self.assertGreater(fresh_soft, fresh_hard)
        self.assertGreater(fresh_soft, worn_soft)

    def test_segment_lap_time_delta_changes_by_track_section(self) -> None:
        engine = _make_engine_for_circuit(3)
        state = engine.driver_states[1]
        _set_test_tire(engine, state, TireCompound.MEDIUM)
        state.tire_usage = 2.0

        state.progress = 0.05
        straight_delta = engine._segment_lap_time_delta(state)
        state.progress = 0.45
        technical_delta = engine._segment_lap_time_delta(state)

        self.assertNotEqual(straight_delta, technical_delta)

    def test_worn_tires_hurt_traction_segment_more_than_straight(self) -> None:
        engine = _make_engine_for_circuit(3)
        state = engine.driver_states[1]
        _set_test_tire(engine, state, TireCompound.SOFT)
        state.tire_usage = 22.0

        state.progress = 0.05
        straight_delta = engine._segment_lap_time_delta(state)
        state.progress = 0.70
        traction_delta = engine._segment_lap_time_delta(state)

        self.assertGreater(traction_delta, straight_delta)

    def test_base_lap_time_uses_segment_modifier(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        state = engine.driver_states[1]
        _set_test_tire(engine, state, TireCompound.SOFT)
        state.tire_usage = 22.0

        state.progress = 0.05
        straight_lap_time = engine._base_lap_time_for_state(state)
        state.progress = 0.70
        traction_lap_time = engine._base_lap_time_for_state(state)

        self.assertGreater(traction_lap_time, straight_lap_time)

    def test_speed_model_converts_car_speed_to_progress_rate(self) -> None:
        large_tick = _make_engine_for_circuit(4, seed=99)
        split_ticks = _make_engine_for_circuit(4, seed=99)
        large_state = large_tick.driver_states[1]
        distance_before = large_state.total_distance_m

        large_tick.tick(GAME_TICK_SECONDS)
        for _ in range(5):
            split_ticks.tick(PHYSICS_STEP_SECONDS)

        self.assertGreater(large_state.speed_kph, 0.0)
        self.assertGreater(large_state.total_distance_m, distance_before)
        self.assertAlmostEqual(
            large_tick._progress_rate[1],
            split_ticks._progress_rate[1],
            delta=0.000001,
        )

    def test_speed_profile_is_built_from_track_curvature(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)

        self.assertIsNotNone(engine._speed_profile)
        self.assertGreater(len(engine._speed_profile.progress), 10)
        self.assertGreater(
            max(engine._speed_profile.raw_speeds_mps),
            min(engine._speed_profile.raw_speeds_mps),
        )

    def test_speed_model_is_faster_on_straights_than_heavy_braking(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]

        straight_speed = _physics_target_speed_at(engine, state, 0.05)
        braking_speed = _physics_target_speed_at(engine, state, 0.08)

        self.assertGreater(straight_speed, braking_speed)

    def test_speed_model_uses_curvature_specific_corner_speeds(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]

        uphill_straight_speed = _physics_target_speed_at(engine, state, 0.27)
        t3_hairpin_speed = _physics_target_speed_at(engine, state, 0.30)
        top_straight_speed = _physics_target_speed_at(engine, state, 0.80)
        rindt_speed = _physics_target_speed_at(engine, state, 0.87)

        self.assertGreater(uphill_straight_speed, t3_hairpin_speed)
        self.assertGreater(top_straight_speed, rindt_speed)

    def test_speed_model_starts_braking_before_tight_corner(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]

        late_straight_speed = _physics_target_speed_at(engine, state, 0.27)
        braking_zone_speed = _physics_target_speed_at(engine, state, 0.29)
        corner_apex_speed = _physics_target_speed_at(engine, state, 0.30)

        self.assertGreater(late_straight_speed, braking_zone_speed)
        self.assertGreater(braking_zone_speed, corner_apex_speed)

    def test_drs_directly_increases_straight_line_acceleration(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]
        distance_m = 0.05 * engine.track_length_m

        state.drs_active = False
        no_drs = engine._vehicle_physics.advance(
            distance_m=distance_m,
            speed_mps=80.0,
            delta_seconds=1.0,
            modifiers=engine._physics_v2_modifiers(state),
        )
        state.drs_active = True
        with_drs = engine._vehicle_physics.advance(
            distance_m=distance_m,
            speed_mps=80.0,
            delta_seconds=1.0,
            modifiers=engine._physics_v2_modifiers(state),
        )

        self.assertGreater(with_drs.speed_mps, no_drs.speed_mps)

    def test_drs_activation_uses_configured_zone(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)

        self.assertTrue(engine._is_drs_zone(0.05))
        self.assertFalse(engine._is_drs_zone(0.30))

    def test_race_waits_for_all_running_drivers_after_leader_finishes(self) -> None:
        engine = _make_engine()
        leader = min(engine.driver_states.values(), key=lambda s: s.position)
        leader.finished = True
        leader.current_lap = engine.total_laps
        leader.total_progress = engine.total_laps
        engine._finish_order.append(leader.driver_id)

        engine._update_positions()
        engine._update_gaps()
        tick = engine.build_tick_state()
        p1 = tick.positions[0]

        self.assertFalse(engine._check_race_finished())
        self.assertEqual(p1.driver_id, leader.driver_id)
        self.assertEqual(p1.gap, "FIN")
        self.assertTrue(p1.finished)

    def test_race_finishes_only_when_every_running_driver_is_done(self) -> None:
        engine = _make_engine()
        for state in engine.driver_states.values():
            state.finished = True
            state.current_lap = engine.total_laps
            state.total_progress = engine.total_laps
            engine._finish_order.append(state.driver_id)

        self.assertTrue(engine._check_race_finished())

    def test_driver_finishing_final_lap_does_not_loop_forever(self) -> None:
        engine = _make_engine()
        leader = min(engine.driver_states.values(), key=lambda s: s.position)
        leader.current_lap = engine.total_laps - 1
        leader.progress = 0.999
        leader.total_progress = leader.current_lap + leader.progress
        setattr(leader, "_lap_start_time", leader.total_time - engine.circuit.base_lap_time)

        engine.tick(1.0)

        self.assertTrue(leader.finished)
        self.assertEqual(leader.current_lap, engine.total_laps)
        self.assertEqual(leader.total_progress, engine.total_laps)
        self.assertEqual(engine._finish_order.count(leader.driver_id), 1)
        self.assertFalse(engine.finished)


if __name__ == "__main__":
    unittest.main()
