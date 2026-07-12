"""Unit tests for simulation engine logic."""

from __future__ import annotations

import asyncio
import unittest
from math import hypot

from data_loader import load_circuits, load_drivers, load_teams
from pydantic import ValidationError

from models.schemas import (
    Circuit,
    CircuitEditorState,
    EditorPoint,
    PaceMode,
    PitLaneConfig,
    RaceSetupRequest,
    TireCompound,
    TrackLayoutSegment,
    TrackLayoutSegmentType,
    TrackSegmentType,
)
from session import SessionManager
from simulation.ai_strategy import choose_tire_for_remaining_laps, should_pit
from simulation.incidents import Incident, IncidentCause, IncidentSeverity
from simulation.physics import GAME_TICK_SECONDS, car_performance_multiplier, driver_pace_multiplier
from simulation.qualifying import run_qualifying
from simulation.race_engine import (
    ATTACK_LINE_INSIDE,
    ATTACK_LINE_OUTSIDE,
    BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA,
    DEFENDER_LINE_DEFENSIVE,
    DEFENDER_LINE_RACING,
    PIT_LANE_SPEED_LIMIT_KPH,
    SC_CAR_LENGTH_M,
    SC_CATCH_UP_FAST_LAP_TIME_FACTOR,
    SC_CATCH_UP_NEAR_LAP_TIME_FACTOR,
    SC_CLEANUP_SECONDS,
    SC_MAX_GAP_CAR_LENGTHS,
    SC_QUEUE_TARGET_CAR_LENGTHS,
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
from simulation.track_physics import PHYSICAL_CAR_WIDTH_M, TRACK_EDGE_MARGIN_M
from simulation.track_compiler import _point_and_tangent_at_progress, compile_circuit_layout, compile_layout_segments
from simulation.tire_model import (
    compute_managed_tire_age,
    compute_tire_performance,
    compute_wear,
    tire_management_age_multiplier,
)


def _make_engine(seed: int = 42) -> RaceEngine:
    drivers = load_drivers()
    teams = {t.id: t for t in load_teams()}
    circuit = load_circuits()[0]
    return RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=1,
        player_driver_ids=[1, 2],
        seed=seed,
    )


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
    )


def _physics_target_speed_at(
    engine: RaceEngine,
    state,
    progress: float,
) -> float:
    state.progress = progress
    return engine._vehicle_physics.target_speed_mps(
        progress * engine.track_length_m,
        engine._physics_v2_modifiers(state),
    )


class RaceSetupTests(unittest.TestCase):
    def test_setup_request_accepts_lap_count_between_5_and_100(self) -> None:
        request = RaceSetupRequest(circuit_id=1, player_team_id=1, total_laps=5)
        self.assertEqual(request.total_laps, 5)

        request = RaceSetupRequest(circuit_id=1, player_team_id=1, total_laps=100)
        self.assertEqual(request.total_laps, 100)

    def test_setup_request_rejects_lap_count_outside_range(self) -> None:
        with self.assertRaises(ValidationError):
            RaceSetupRequest(circuit_id=1, player_team_id=1, total_laps=4)

        with self.assertRaises(ValidationError):
            RaceSetupRequest(circuit_id=1, player_team_id=1, total_laps=101)

    def test_session_uses_requested_lap_count_without_mutating_circuit_data(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()

        session = manager.create_session(
            RaceSetupRequest(circuit_id=1, player_team_id=1, total_laps=12),
            drivers,
            teams,
            circuits,
        )

        self.assertEqual(session.circuit.total_laps, 12)
        self.assertEqual(session.engine.total_laps, 12)
        self.assertEqual(circuits[0].total_laps, 30)

    def test_session_uses_supplied_qualifying_grid_order(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()
        grid_order = [drivers[-1].id, drivers[0].id, drivers[1].id]

        session = manager.create_session(
            RaceSetupRequest(
                circuit_id=1,
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

    def test_can_create_session_with_second_circuit(self) -> None:
        drivers = load_drivers()
        teams = load_teams()
        circuits = load_circuits()
        manager = SessionManager()

        session = manager.create_session(
            RaceSetupRequest(circuit_id=2, player_team_id=1, total_laps=42),
            drivers,
            teams,
            circuits,
        )

        self.assertEqual(session.circuit.id, 2)
        self.assertEqual(session.circuit.name, "Technical Park Circuit")
        self.assertEqual(session.engine.total_laps, 42)
        self.assertGreater(len(session.circuit.track_coords), 20)
        self.assertEqual(len(session.circuit.drs_zones), 2)
        self.assertGreaterEqual(len(session.circuit.segments), 5)

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
        self.assertEqual(circuits[3].total_laps, 71)
        self.assertEqual(session.engine.total_laps, 71)
        self.assertGreater(len(session.circuit.track_coords), 40)
        self.assertEqual(len(session.circuit.drs_zones), 3)
        self.assertGreaterEqual(len(session.circuit.segments), 10)

    def test_session_pause_and_resume_commands_keep_connection_state_valid(self) -> None:
        manager = SessionManager()
        session = manager.create_session(
            RaceSetupRequest(circuit_id=1, player_team_id=1, total_laps=12),
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
        self.assertGreater(len(session.race_info.racing_line_profile), 10)

        resume_response = asyncio.run(session.handle_command({"type": "resume"}))
        self.assertEqual(resume_response["type"], "command_ack")
        self.assertFalse(session.engine.paused)

    def test_circuit_track_coords_do_not_self_intersect(self) -> None:
        for circuit in load_circuits():
            self.assertEqual(self_intersections(circuit.track_coords), [], circuit.name)

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

        for circuit_id in (3, 4, 5, 6, 7):
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

    def test_car_performance_mapping_is_compressed_for_closer_racing(self) -> None:
        self.assertAlmostEqual(car_performance_multiplier(1.030), 1.015)
        self.assertAlmostEqual(car_performance_multiplier(0.980), 0.990)

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
                    car_performance_multiplier(team.car_performance)
                    * driver_pace_multiplier(driver.stats.pace)
                    * tire_performance
                )
            )

        self.assertLess(max(lap_times) - min(lap_times), 5.5)


class QualifyingTests(unittest.TestCase):
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
    def test_engine_uses_fixed_step_physics(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]
        progress_before = state.total_progress

        engine.tick(GAME_TICK_SECONDS)
        tick = engine.build_tick_state()
        position = next(item for item in tick.positions if item.driver_id == state.driver_id)
        self.assertEqual(tick.physics_hz, 50)
        self.assertGreater(position.target_speed_kph, 0.0)
        self.assertGreater(state.total_progress, progress_before)

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
        )

        self.assertEqual(engine._physics_v2_virtual_line(attacker), ATTACK_LINE_INSIDE)
        self.assertEqual(engine._physics_v2_virtual_line(defender), DEFENDER_LINE_RACING)
        self.assertFalse(engine._physics_v2_passing_authorized(attacker, defender))
        attacker.lateral_offset_m = 2.2
        self.assertTrue(engine._physics_v2_passing_authorized(attacker, defender))

        engine._trigger_vsc([])
        self.assertFalse(engine._physics_v2_passing_authorized(attacker, defender))

        engine.race_phase = "sc"
        engine._sc_unlap_driver_ids.add(attacker.driver_id)
        self.assertTrue(engine._physics_v2_passing_authorized(attacker, defender))

    def test_physics_v2_green_battles_never_overlap_physical_car_bodies(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        pass_events = 0

        for _ in range(800):
            events = engine.tick(GAME_TICK_SECONDS)
            pass_events += sum(event.type == "pass" for event in events)
            ordered = sorted(
                (
                    state
                    for state in engine.driver_states.values()
                    if not state.retired and not state.finished and not state.in_pit
                ),
                key=lambda state: state.position,
            )
            for state in ordered:
                track_sample = engine._track_physics.at_progress(state.progress)
                positive_limit_m = (
                    track_sample.left_width_m
                    - PHYSICAL_CAR_WIDTH_M / 2.0
                    - TRACK_EDGE_MARGIN_M
                )
                negative_limit_m = (
                    track_sample.right_width_m
                    - PHYSICAL_CAR_WIDTH_M / 2.0
                    - TRACK_EDGE_MARGIN_M
                )
                self.assertLessEqual(state.lateral_offset_m, positive_limit_m + 1e-6)
                self.assertGreaterEqual(state.lateral_offset_m, -negative_limit_m - 1e-6)
            for ahead, behind in zip(ordered, ordered[1:]):
                lateral_separation_m = abs(
                    ahead.lateral_offset_m - behind.lateral_offset_m
                )
                if lateral_separation_m >= PHYSICAL_CAR_WIDTH_M + 0.15:
                    continue
                gap_m = (
                    ahead.total_progress - behind.total_progress
                ) * engine.track_length_m
                self.assertGreaterEqual(gap_m, SC_CAR_LENGTH_M - 1e-6)

        self.assertGreater(pass_events, 0)

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

    def test_car_stopped_incident_triggers_vsc(self) -> None:
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
        self.assertEqual(engine.race_phase, "vsc")
        self.assertTrue(any(e.type == "retirement" for e in events))

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

        self.assertEqual(
            progress_before,
            {state.driver_id: state.total_progress for state in engine.driver_states.values()},
        )

        engine._sc_cleanup_until = float("inf")
        for _ in range(3000):
            engine.tick(GAME_TICK_SECONDS)
            if engine._safety_car_queue_formed:
                break
        self.assertTrue(engine._safety_car_queue_formed)
        self.assertLessEqual(
            engine.race_elapsed - deployed_at,
            engine.circuit.base_lap_time * 2.0,
        )

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
            self.assertLessEqual(gap_m, target_gap_m + SC_CAR_LENGTH_M)

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

    def test_safety_car_pit_lane_distance_updates_order_without_track_overtakes(self) -> None:
        engine = _make_engine_for_circuit(3)
        engine._trigger_safety_car([])

        ordered = sorted(engine.driver_states.values(), key=lambda state: state.position)
        pitting_car = ordered[0]
        next_on_track = ordered[1]
        frozen_on_track_order = [state.driver_id for state in ordered[1:]]

        pitting_car.in_pit = True
        pitting_car.total_progress = next_on_track.total_progress - 0.001
        engine._update_positions()

        self.assertLess(next_on_track.position, pitting_car.position)
        self.assertEqual(
            frozen_on_track_order,
            [
                state.driver_id
                for state in sorted(engine.driver_states.values(), key=lambda state: state.position)
                if state is not pitting_car
            ],
        )

        position_at_pit_exit = pitting_car.position
        pitting_car.in_pit = False
        engine._update_positions()
        self.assertEqual(pitting_car.position, position_at_pit_exit)

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
        for circuit_id in (3, 4, 5, 6, 7):
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
        # 모든 AI 타이어를 충분히 닳게 만들어 SC 피트 자격을 부여한다.
        for driver_id, state in engine.driver_states.items():
            if driver_id in engine.player_driver_ids:
                continue
            state.tire_usage = 30.0
        # 확정적으로 피트를 굴리도록 확률을 1.0으로.
        import simulation.race_engine as re_mod

        original = re_mod.SC_PIT_PROBABILITY
        re_mod.SC_PIT_PROBABILITY = 1.0
        try:
            engine._trigger_safety_car([])
        finally:
            re_mod.SC_PIT_PROBABILITY = original

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
        leader = engine._on_track_leader()
        backmarker = max(engine.driver_states.values(), key=lambda state: state.position)
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
        for _ in range(1000):
            engine.tick(GAME_TICK_SECONDS)
            if state.last_lap_time > 0:
                break

        self.assertGreater(state.last_lap_time, 0)
        self.assertGreater(
            abs((state.last_lap_time * 10) - round(state.last_lap_time * 10)),
            0.001,
        )

    def test_tick_state_exposes_driver_lap_history(self) -> None:
        engine = _make_engine()
        state = engine.driver_states[1]
        for _ in range(1000):
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
        self.assertEqual(state.tire_usage, 0.0)
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
        for _ in range(1000):
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

        # 세 단계를 모두 거친다.
        self.assertEqual({"in", "stop", "out"}, set(phases_seen))
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
        engine._update_positions()
        self.assertLess(state.position, rival.position)

        rival.total_progress = expected_total_progress + 0.001
        engine._update_positions()
        self.assertLess(rival.position, state.position)

    def test_real_circuit_pit_order_progress_is_monotonic(self) -> None:
        for circuit_id in (3, 4, 5, 6, 7):
            engine = _make_engine_for_circuit(circuit_id)
            driver_id = 1
            state = engine.driver_states[driver_id]
            entry = engine.circuit.pit_lane.entry_progress
            exit_ = engine.circuit.pit_lane.exit_progress
            self.assertIsNotNone(entry)
            self.assertIsNotNone(exit_)

            state.pit_request = TireCompound.HARD
            state.progress = entry - 0.001
            state.total_progress = state.current_lap + state.progress
            engine.tick(GAME_TICK_SECONDS)
            self.assertTrue(state.in_pit, engine.circuit.name)

            observed = [state.total_progress]
            while state.in_pit:
                engine.tick(GAME_TICK_SECONDS)
                observed.append(state.total_progress)

            self.assertEqual(observed, sorted(observed), engine.circuit.name)
            self.assertGreater(observed[-1] - observed[0], 0.02, engine.circuit.name)
            self.assertAlmostEqual(
                observed[-1],
                state.current_lap + exit_,
                places=6,
                msg=engine.circuit.name,
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

    def test_pit_exits_at_configured_exit_progress(self) -> None:
        engine = _make_engine_for_circuit(3)
        driver_id = 1
        state = engine.driver_states[driver_id]
        entry = engine.circuit.pit_lane.entry_progress
        exit_ = engine.circuit.pit_lane.exit_progress
        self.assertIsNotNone(entry)
        self.assertIsNotNone(exit_)

        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.001
        state.total_progress = state.current_lap + state.progress
        engine.tick(GAME_TICK_SECONDS)
        self.assertTrue(state.in_pit)

        while state.in_pit:
            engine.tick(GAME_TICK_SECONDS)

        self.assertEqual(state.pit_count, 1)
        self.assertAlmostEqual(state.progress, exit_, places=6)
        self.assertEqual(state.current_lap, 1)
        self.assertAlmostEqual(state.total_progress, state.current_lap + exit_, places=6)

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

    def test_pit_lane_speed_is_limited_to_sixty_kph(self) -> None:
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
        self.assertEqual(driver.pit_phase, "in")
        self.assertEqual(driver.speed_kph, PIT_LANE_SPEED_LIMIT_KPH)

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
        self.assertEqual(driver.speed_kph, PIT_LANE_SPEED_LIMIT_KPH)

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
        engine_fast.set_speed(5)
        engine_slow.tick(GAME_TICK_SECONDS)
        engine_fast.tick(GAME_TICK_SECONDS * 5)
        slow_progress = engine_slow.driver_states[1].total_progress
        fast_progress = engine_fast.driver_states[1].total_progress
        self.assertGreater(fast_progress, slow_progress)

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

    def test_ai_pace_conserves_when_tire_life_is_critical(self) -> None:
        engine = _make_engine()
        ai_driver = engine.driver_states[3]
        ai_driver.tire_compound = TireCompound.SOFT
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
        car_ahead.progress = 0.500
        attacker.progress = 0.492
        car_ahead.total_progress = 1.500
        attacker.total_progress = 1.492
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

    def test_maybe_battle_event_creates_attack_when_forced(self) -> None:
        engine = _make_engine_for_circuit(3)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.058
        attacker.progress = 0.050
        car_ahead.total_progress = 1.058
        attacker.total_progress = 1.050
        attacker.pace_mode = PaceMode.ATTACK
        engine.rng.random = lambda: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "attack")
        self.assertEqual(event.driver, "VER")
        self.assertLess(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_defend_event_adds_time_cost_to_attacker(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[1]
        attacker = engine.driver_states[14]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.356
        attacker.progress = 0.350
        car_ahead.total_progress = 1.356
        attacker.total_progress = 1.350
        attacker.pace_mode = PaceMode.CONSERVE
        engine.rng.random = lambda: 0.0
        engine.rng.uniform = lambda _low, _high: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "defend")
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertEqual(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_close_heavy_braking_scores_create_side_by_side_event(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.186
        attacker.progress = 0.180
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        attacker.pace_mode = PaceMode.STANDARD
        engine._driver_meta[attacker.driver_id]["overtaking"] = 0.90
        engine._driver_meta[car_ahead.driver_id]["defending"] = 0.75
        rolls = iter([0.0, 1.0, 0.0, 0.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "side_by_side")
        self.assertIn("inside", event.message)
        self.assertIn("라인", event.message_ko)
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)
        self.assertTrue(engine._is_side_by_side_active(attacker.driver_id))
        self.assertTrue(engine._is_side_by_side_active(car_ahead.driver_id))

        tick = engine.build_tick_state()
        positions = {position.driver_id: position for position in tick.positions}
        self.assertTrue(positions[attacker.driver_id].side_by_side_active)
        self.assertTrue(positions[car_ahead.driver_id].side_by_side_active)

    def test_side_by_side_state_blocks_repeat_event_and_expires(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.186
        attacker.progress = 0.180
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        attacker.pace_mode = PaceMode.STANDARD
        engine._driver_meta[attacker.driver_id]["overtaking"] = 0.90
        engine._driver_meta[car_ahead.driver_id]["defending"] = 0.75
        rolls = iter([0.0, 1.0, 0.0, 0.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.0

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "side_by_side")
        engine._battle_event_cooldown.clear()
        engine.rng.random = lambda: 0.0
        self.assertIsNone(engine._maybe_battle_event(attacker, car_ahead))

        engine._tick_side_by_side_battles(2.0)
        self.assertTrue(engine._is_side_by_side_active(attacker.driver_id))
        engine._tick_side_by_side_battles(10.0)
        self.assertFalse(engine._is_side_by_side_active(attacker.driver_id))

    def test_inside_line_gains_entry_but_loses_corner_exit(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180

        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
        )

        self.assertLess(
            engine._battle_effect_lap_time_delta(attacker),
            BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA,
        )
        engine.rng.random = lambda: 1.0
        engine._tick_side_by_side_battles(10.0)

        self.assertFalse(engine._is_side_by_side_active(attacker.driver_id))
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_outside_line_costs_entry_but_gains_corner_exit(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180

        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_OUTSIDE,
            DEFENDER_LINE_RACING,
        )

        self.assertGreater(
            engine._battle_effect_lap_time_delta(attacker),
            BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA,
        )
        engine.rng.random = lambda: 1.0
        engine._tick_side_by_side_battles(10.0)

        self.assertFalse(engine._is_side_by_side_active(attacker.driver_id))
        self.assertLess(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_defensive_line_slows_defender_and_corner_exit(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180

        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_DEFENSIVE,
        )

        self.assertGreater(
            engine._battle_effect_lap_time_delta(car_ahead),
            BATTLE_SIDE_BY_SIDE_LAP_TIME_DELTA,
        )
        engine.rng.random = lambda: 1.0
        engine._tick_side_by_side_battles(10.0)

        self.assertFalse(engine._is_side_by_side_active(car_ahead.driver_id))
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_inside_line_can_end_with_run_wide_result(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_RACING,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.0])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_side_by_side_battles(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "run_wide")
        self.assertIn("T1 Braking", events[0].message)
        self.assertIn("바깥", events[0].message_ko)
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_outside_line_can_end_with_traction_loss_result(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_OUTSIDE,
            DEFENDER_LINE_RACING,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.0])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_side_by_side_battles(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "traction_loss")
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_defensive_line_can_end_with_minor_contact_result(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_side_by_side_battle(
            attacker.driver_id,
            car_ahead.driver_id,
            ATTACK_LINE_INSIDE,
            DEFENDER_LINE_DEFENSIVE,
            "T1 Braking",
        )
        # 세 번째 roll(0.5)은 contact escalation 판정: MINOR 유지(>=0.10).
        rolls = iter([0.0, 0.0, 0.5])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_side_by_side_battles(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "minor_contact")
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_side_by_side_state_clears_after_attacker_gets_ahead(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[4]
        attacker = engine.driver_states[3]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.186
        attacker.progress = 0.180
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_side_by_side_battle(attacker.driver_id, car_ahead.driver_id)

        attacker.position = 1
        car_ahead.position = 2
        attacker.total_progress = 1.190
        car_ahead.total_progress = 1.186
        engine._prune_side_by_side_battles()

        self.assertFalse(engine._is_side_by_side_active(attacker.driver_id))
        self.assertFalse(engine._is_side_by_side_active(car_ahead.driver_id))

    def test_strong_heavy_braking_attack_can_force_defender_wide(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.186
        attacker.progress = 0.180
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        attacker.pace_mode = PaceMode.ATTACK
        rolls = iter([0.0, 1.0])
        engine.rng.random = lambda: next(rolls)
        engine.rng.uniform = lambda _low, _high: 0.04

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "forced_wide")
        self.assertLess(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)
        self.assertEqual(len(engine._forced_wide_aftermaths), 1)

    def test_forced_wide_can_make_defender_run_wide_on_exit(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
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

    def test_forced_wide_can_make_defender_lose_traction_on_exit(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.5])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "traction_loss")
        self.assertEqual(events[0].driver, engine._driver_meta[car_ahead.driver_id]["abbreviation"])
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_forced_wide_can_cost_attacker_on_tight_exit(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        rolls = iter([0.0, 0.8])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "traction_loss")
        self.assertEqual(events[0].driver, "VER")
        self.assertIn("tight exit", events[0].message)
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)

    def test_forced_wide_can_end_with_minor_contact(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        engine._start_forced_wide_aftermath(
            attacker.driver_id,
            car_ahead.driver_id,
            "T1 Braking",
        )
        # 세 번째 roll(0.5)은 contact escalation 판정: MINOR 유지(>=0.10).
        rolls = iter([0.0, 0.95, 0.5])
        engine.rng.random = lambda: next(rolls)

        events = engine._tick_forced_wide_aftermaths(10.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "minor_contact")
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertGreater(engine._battle_effect_lap_time_delta(car_ahead), 0.0)

    def test_heavy_braking_attack_can_create_lockup_penalty(self) -> None:
        engine = _make_engine_for_circuit(2)
        car_ahead = engine.driver_states[14]
        attacker = engine.driver_states[1]
        car_ahead.position = 1
        attacker.position = 2
        car_ahead.current_lap = 1
        attacker.current_lap = 1
        car_ahead.progress = 0.186
        attacker.progress = 0.180
        car_ahead.total_progress = 1.186
        attacker.total_progress = 1.180
        attacker.pace_mode = PaceMode.ATTACK
        attacker.tire_usage = 24.0
        rolls = iter([0.0, 0.0])
        engine.rng.random = lambda: next(rolls)

        engine._battle_lap_time_delta(attacker, car_ahead)
        event = engine._maybe_battle_event(attacker, car_ahead)

        self.assertIsNotNone(event)
        self.assertEqual(event.type, "lockup")
        self.assertGreater(engine._battle_effect_lap_time_delta(attacker), 0.0)
        self.assertGreater(engine._battle_effect_tire_usage_multiplier(attacker), 1.0)

    def test_pass_event_reports_position_gain(self) -> None:
        engine = _make_engine_for_circuit(3)
        previous_positions = {1: 2, 2: 1}
        engine.driver_states[1].position = 1
        engine.driver_states[2].position = 2

        events = engine._build_pass_events(previous_positions)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].type, "pass")
        self.assertIn("passes", events[0].message)

    def test_tick_activates_drs_when_within_one_second_of_car_ahead(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        chaser = engine.driver_states[2]
        car_ahead.position = 1
        chaser.position = 2
        car_ahead.current_lap = 1
        chaser.current_lap = 1
        car_ahead.progress = 0.500
        chaser.progress = 0.492
        car_ahead.total_progress = 1.500
        chaser.total_progress = 1.492

        engine.tick(GAME_TICK_SECONDS)

        self.assertTrue(chaser.dirty_air_active)
        self.assertTrue(chaser.drs_active)

    def test_dirty_air_can_be_active_without_drs_outside_drs_zone(self) -> None:
        engine = _make_engine()
        car_ahead = engine.driver_states[1]
        chaser = engine.driver_states[2]
        car_ahead.position = 1
        chaser.position = 2
        car_ahead.current_lap = 1
        chaser.current_lap = 1
        car_ahead.progress = 0.300
        chaser.progress = 0.292
        car_ahead.total_progress = 1.300
        chaser.total_progress = 1.292

        engine.tick(GAME_TICK_SECONDS)

        self.assertTrue(chaser.dirty_air_active)
        self.assertFalse(chaser.drs_active)

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
        high_management.tire_compound = TireCompound.MEDIUM
        low_management.tire_compound = TireCompound.MEDIUM
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

    def test_segment_lap_time_delta_changes_by_track_section(self) -> None:
        engine = _make_engine_for_circuit(3)
        state = engine.driver_states[1]
        state.tire_compound = TireCompound.MEDIUM
        state.tire_usage = 2.0

        state.progress = 0.05
        straight_delta = engine._segment_lap_time_delta(state)
        state.progress = 0.45
        technical_delta = engine._segment_lap_time_delta(state)

        self.assertNotEqual(straight_delta, technical_delta)

    def test_worn_tires_hurt_traction_segment_more_than_straight(self) -> None:
        engine = _make_engine_for_circuit(3)
        state = engine.driver_states[1]
        state.tire_compound = TireCompound.SOFT
        state.tire_usage = 22.0

        state.progress = 0.05
        straight_delta = engine._segment_lap_time_delta(state)
        state.progress = 0.70
        traction_delta = engine._segment_lap_time_delta(state)

        self.assertGreater(traction_delta, straight_delta)

    def test_base_lap_time_uses_segment_modifier(self) -> None:
        engine = _make_engine_for_circuit(3, seed=99)
        state = engine.driver_states[1]
        state.tire_compound = TireCompound.SOFT
        state.tire_usage = 22.0

        state.progress = 0.05
        straight_lap_time = engine._base_lap_time_for_state(state)
        state.progress = 0.70
        traction_lap_time = engine._base_lap_time_for_state(state)

        self.assertGreater(traction_lap_time, straight_lap_time)

    def test_speed_model_converts_car_speed_to_progress_rate(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]
        distance_before = state.total_distance_m

        engine.tick(GAME_TICK_SECONDS)

        self.assertGreater(state.speed_kph, 0.0)
        self.assertAlmostEqual(
            engine._progress_rate[state.driver_id],
            (state.total_distance_m - distance_before)
            / engine.track_length_m
            / GAME_TICK_SECONDS,
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

    def test_drs_directly_increases_straight_line_target_speed(self) -> None:
        engine = _make_engine_for_circuit(4, seed=99)
        state = engine.driver_states[1]

        state.drs_active = False
        no_drs_speed = _physics_target_speed_at(engine, state, 0.05)
        state.drs_active = True
        drs_speed = _physics_target_speed_at(engine, state, 0.05)

        self.assertGreater(drs_speed, no_drs_speed)

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
