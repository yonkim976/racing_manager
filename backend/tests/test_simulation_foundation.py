"""Regression tests for the simulation foundation baseline contract."""

from __future__ import annotations

import unittest
from math import cos, hypot, pi, sin
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import (
    Circuit,
    CircuitThermalProfile,
    RaceInfoMessage,
    ThermalPresetName,
    TireCompound,
    TrackConditionSample,
    TrackConditions,
    TrackWidthSample,
)
from simulation.incidents import Incident, IncidentCause, IncidentSeverity
from simulation.race_engine import (
    MAX_PHYSICS_DIAGNOSTIC_SAMPLES,
    PIT_LANE_SPEED_LIMIT_KPH,
    RaceEngine,
)
from simulation.state_contract import PhysicsStepResult, TickPhase
from simulation.track_data_validation import (
    track_data_warnings,
    validate_track_data_v2,
)
from simulation.track_geometry import (
    FLAT_TRACK_BANK_ANGLE_DEG,
    FLAT_TRACK_ELEVATION_M,
    FLAT_TRACK_GRADE,
    TRACK_GEOMETRY_MODE,
)
from simulation.track_physics import (
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
    build_track_physics_profile,
    clear_track_physics_caches,
)
from simulation.track_surface import CAR_WHEELBASE_M
from simulation.vehicle_dimensions import PHYSICAL_CAR_WHEELBASE_M
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS


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


class SimulationFoundationBaselineTests(unittest.TestCase):
    def _make_engine(self) -> RaceEngine:
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        return RaceEngine(
            circuit=circuit,
            drivers=load_drivers(),
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=123,
            start_sequence_enabled=False,
        )

    def test_all_seed_teams_use_the_single_baseline_engine_power(self) -> None:
        self.assertEqual(
            {team.engine_power_kw for team in load_teams()},
            {750.0},
        )

    def test_seed_tracks_have_required_source_lineage(self) -> None:
        for circuit in load_circuits():
            self.assertEqual(validate_track_data_v2(circuit), [], circuit.name)

    def test_v2_tracks_require_complete_source_lineage(self) -> None:
        legacy = load_circuits()[0]
        assert legacy.geo is not None
        incomplete_source = legacy.geo.model_copy(
            update={
                "source_url": None,
                "acquired_at": None,
                "transform_version": None,
            }
        )
        v2 = legacy.model_copy(
            update={"data_version": "v2", "geo": incomplete_source}
        )

        errors = validate_track_data_v2(v2)

        self.assertIn("track source metadata field is missing: source_url", errors)
        self.assertIn("track source metadata field is missing: acquired_at", errors)
        self.assertIn(
            "track source metadata field is missing: transform_version",
            errors,
        )

    def _make_v2_fixture(self, **updates) -> Circuit:
        circuit = load_circuits()[0]
        assert circuit.geo is not None
        source = circuit.geo.model_copy(
            update={
                "source_url": "https://example.test/track",
                "acquired_at": "2026-07-21",
                "transform_version": "fixture-v2",
            }
        )
        return circuit.model_copy(
            update={
                "data_version": "v2",
                "geo": source,
                "track_width_profile": [
                    TrackWidthSample(progress=0.0, leftWidthM=6.0, rightWidthM=6.0),
                    TrackWidthSample(progress=0.5, leftWidthM=6.0, rightWidthM=6.0),
                ],
                "track_conditions": [
                    TrackConditionSample(progress=0.0, grade=0.0, bankAngleDeg=0.0),
                    TrackConditionSample(progress=0.5, grade=0.0, bankAngleDeg=0.0),
                ],
                **updates,
            }
        )

    def test_v2_track_fixtures_report_duplicate_and_narrow_profiles(self) -> None:
        duplicate = self._make_v2_fixture(
            track_width_profile=[
                TrackWidthSample(progress=0.0, leftWidthM=6.0, rightWidthM=6.0),
                TrackWidthSample(progress=0.0, leftWidthM=6.0, rightWidthM=6.0),
            ]
        )
        narrow = self._make_v2_fixture(
            track_width_profile=[
                TrackWidthSample(progress=0.0, leftWidthM=1.2, rightWidthM=6.0),
                TrackWidthSample(progress=0.5, leftWidthM=6.0, rightWidthM=6.0),
            ]
        )

        self.assertIn(
            "track width profile progress must be strictly increasing",
            validate_track_data_v2(duplicate),
        )
        self.assertIn(
            "track width profile must clear the car half-width and safety margin",
            validate_track_data_v2(narrow),
        )

    def test_v2_track_warnings_use_closed_loop_physical_distance(self) -> None:
        width_rapid = self._make_v2_fixture(
            track_width_profile=[
                TrackWidthSample(progress=0.0, leftWidthM=9.0, rightWidthM=6.0),
                TrackWidthSample(progress=0.999, leftWidthM=6.0, rightWidthM=6.0),
            ]
        )
        condition_rapid = self._make_v2_fixture(
            track_conditions=[
                TrackConditionSample(progress=0.0, grade=0.5, bankAngleDeg=0.0),
                TrackConditionSample(progress=0.999, grade=0.0, bankAngleDeg=0.0),
            ]
        )

        self.assertIn(
            "track width profile contains a rapid change",
            track_data_warnings(width_rapid),
        )
        self.assertIn(
            "track condition profile contains a rapid grade/bank change",
            track_data_warnings(condition_rapid),
        )

    def test_track_validator_supports_direct_file_execution(self) -> None:
        backend_dir = Path(__file__).resolve().parents[1]
        tool = backend_dir / "tools" / "validate_track_data.py"

        completed = subprocess.run(
            [sys.executable, str(tool)],
            cwd=backend_dir,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn('"Bahrain International Circuit": []', completed.stdout)

    def test_track_v2_preserves_width_and_25d_channels(self) -> None:
        width = TrackWidthSample(
            progress=0.25,
            leftWidthM=7.2,
            rightWidthM=6.8,
        )
        condition = TrackConditionSample(
            progress=0.25,
            elevationM=18.0,
            grade=0.04,
            bankAngleDeg=-2.5,
            surface="asphalt",
        )
        self.assertEqual(width.left_width_m, 7.2)
        self.assertEqual(width.right_width_m, 6.8)
        self.assertEqual(condition.elevation_m, 18.0)
        self.assertEqual(condition.bank_angle_deg, -2.5)

    def test_bahrain_v2_profile_uses_official_width_and_dem_elevation_bounds(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        clear_track_physics_caches()
        profile = build_track_physics_profile(circuit)

        self.assertEqual(circuit.data_version, "v2")
        self.assertEqual(validate_track_data_v2(circuit), [])
        self.assertEqual(track_data_warnings(circuit), [])
        self.assertAlmostEqual(
            profile.at_progress(0.5).left_width_m
            + profile.at_progress(0.5).right_width_m,
            14.0,
            delta=0.05,
        )
        self.assertGreater(
            profile.at_progress(0.13).left_width_m
            + profile.at_progress(0.13).right_width_m,
            21.5,
        )
        self.assertEqual(
            (
                min(sample.elevation_m for sample in circuit.track_conditions),
                max(sample.elevation_m for sample in circuit.track_conditions),
            ),
            (0.0, 18.0),
        )
        self.assertGreaterEqual(
            min(sample.grade for sample in circuit.track_conditions),
            -0.056,
        )
        self.assertLessEqual(
            max(sample.grade for sample in circuit.track_conditions),
            0.036,
        )

    def test_all_circuits_use_the_planar_runtime_contract(self) -> None:
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        self.assertEqual(TRACK_GEOMETRY_MODE, "planar_2d")
        self.assertEqual(FLAT_TRACK_BANK_ANGLE_DEG, 0.0)

        for circuit in load_circuits():
            with self.subTest(circuit=circuit.name):
                self.assertNotIn(
                    "track_conditions is empty; using legacy asphalt/flat fallback",
                    track_data_warnings(circuit),
                )
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
                state.progress = 0.69
                state.total_progress = 0.69
                state.speed_kph = 220.0

                engine.tick(PHYSICS_STEP_SECONDS)
                telemetry = engine.build_tick_state().positions[0]

                self.assertEqual(state.track_elevation_m, FLAT_TRACK_ELEVATION_M)
                self.assertEqual(state.track_grade, FLAT_TRACK_GRADE)
                self.assertEqual(
                    telemetry.track_elevation_m,
                    FLAT_TRACK_ELEVATION_M,
                )
                self.assertEqual(telemetry.track_grade, FLAT_TRACK_GRADE)

    def test_vehicle_dimensions_are_shared_across_physics_and_surface_models(self) -> None:
        self.assertEqual(PHYSICAL_CAR_WIDTH_M, 1.9)
        self.assertEqual(PHYSICAL_CAR_LENGTH_M, 5.0)
        self.assertEqual(PHYSICAL_CAR_WHEELBASE_M, 3.4)
        self.assertEqual(CAR_WHEELBASE_M, PHYSICAL_CAR_WHEELBASE_M)

    def test_race_info_exposes_the_baseline_wheelbase(self) -> None:
        race_info = RaceInfoMessage(
            circuit_name="Test Circuit",
            total_laps=1,
            player_team="Test Team",
            player_team_color="#000000",
            player_drivers=[],
            track_length_m=1000.0,
            track_coords=[],
        )
        self.assertEqual(race_info.car_width_m, 1.9)
        self.assertEqual(race_info.car_length_m, 5.0)
        self.assertEqual(race_info.wheelbase_m, 3.4)

    def test_tick_exposes_ordered_physics_frame_and_derived_telemetry(self) -> None:
        engine = self._make_engine()

        events = engine.tick(0.1)
        tick = engine.build_tick_state(events)
        state = engine.driver_states[1]
        position = next(item for item in tick.positions if item.driver_id == 1)

        self.assertEqual(
            engine._tick_phase_history,
            [
                TickPhase.COMMAND,
                TickPhase.PHYSICS,
                TickPhase.RULES,
                TickPhase.TELEMETRY,
            ],
        )
        self.assertEqual(tick.physics_frame, 5)
        self.assertEqual(state.physics_frame, tick.physics_frame)
        self.assertEqual(position.physics_frame, tick.physics_frame)
        self.assertEqual(position.telemetry_source, "physics_derived")
        self.assertGreaterEqual(position.engine_rpm, 0.0)

    def test_solo_incident_is_applied_after_physical_movement(self) -> None:
        engine = self._make_engine()
        initial_progress = engine.driver_states[1].total_progress
        observed_progress: list[float] = []
        original_apply = engine._apply_incident

        def observe_apply(incident, events) -> None:
            observed_progress.append(engine.driver_states[1].total_progress)
            original_apply(incident, events)

        engine._apply_incident = observe_apply
        incident = Incident(
            cause=IncidentCause.DRIVER_ERROR,
            severity=IncidentSeverity.MINOR,
            primary_driver_id=1,
        )
        with patch("simulation.race_engine.roll_solo_incident", return_value=incident):
            engine.tick(0.1)

        self.assertTrue(observed_progress)
        self.assertGreater(observed_progress[0], initial_progress)

    def test_world_coordinates_use_local_metres_not_render_units(self) -> None:
        def circuit_with_transform(scale: float, offset_x: float, offset_y: float) -> Circuit:
            render_radius = 100.0
            points = [
                [
                    offset_x + scale * render_radius * cos(2.0 * pi * index / 32.0),
                    offset_y + scale * render_radius * sin(2.0 * pi * index / 32.0),
                ]
                for index in range(32)
            ]
            return Circuit(
                id="metric-test",
                name="Metric Test",
                country="Test",
                base_lap_time=60.0,
                track_length_m=2.0 * pi * render_radius,
                thermal_profile=_test_thermal_profile(),
                track_coords=points,
            )

        clear_track_physics_caches()
        first = build_track_physics_profile(circuit_with_transform(1.0, 0.0, 0.0))
        clear_track_physics_caches()
        second = build_track_physics_profile(circuit_with_transform(3.0, 700.0, -250.0))
        first_pose = first.line_pose_at_progress_m("racing_line", 0.37)
        second_pose = second.line_pose_at_progress_m("racing_line", 0.37)

        self.assertAlmostEqual(first_pose[0], second_pose[0], places=5)
        self.assertAlmostEqual(first_pose[1], second_pose[1], places=5)
        self.assertAlmostEqual(first_pose[2], second_pose[2], places=5)
        self.assertAlmostEqual(
            hypot(
                (first_pose[0] - sin(first_pose[2])) - first_pose[0],
                (first_pose[1] + cos(first_pose[2])) - first_pose[1],
            ),
            1.0,
            places=6,
        )

        closed_start = first.line_pose_at_progress_m("racing_line", 0.0)
        closed_end = first.line_pose_at_progress_m("racing_line", 1.0)
        self.assertAlmostEqual(closed_start[0], closed_end[0], places=6)
        self.assertAlmostEqual(closed_start[1], closed_end[1], places=6)
        self.assertAlmostEqual(first.coordinate_frame.track_length_m, 2.0 * pi * 100.0)

        local_centerline = [
            first.coordinate_frame.to_local_m(x, y)
            for x, y in circuit_with_transform(1.0, 0.0, 0.0).track_coords
        ]
        local_length = sum(
            hypot(
                local_centerline[(index + 1) % len(local_centerline)][0] - point[0],
                local_centerline[(index + 1) % len(local_centerline)][1] - point[1],
            )
            for index, point in enumerate(local_centerline)
        )
        self.assertAlmostEqual(
            local_length,
            first.coordinate_frame.track_length_m,
            places=9,
        )

    def test_fixed_step_accumulator_preserves_residual_time(self) -> None:
        engine = self._make_engine()
        engine.tick(0.01)
        self.assertEqual(engine._physics_frame, 0)
        self.assertAlmostEqual(engine._physics_accumulator.remainder_s, 0.01)
        engine.tick(0.01)
        self.assertEqual(engine._physics_frame, 1)
        self.assertEqual(engine._physics_step_deltas[-1], 0.02)
        self.assertAlmostEqual(engine.driver_states[1].simulation_time_s, 0.02)

    def test_chunking_is_independent_and_physics_frame_counts_real_steps(self) -> None:
        first = self._make_engine()
        second = self._make_engine()
        for _ in range(4):
            first.tick(0.05)
        for _ in range(2):
            second.tick(0.1)

        self.assertEqual(first._physics_frame, 10)
        self.assertEqual(second._physics_frame, 10)
        self.assertTrue(all(step == 0.02 for step in first._physics_step_deltas))
        self.assertTrue(all(step == 0.02 for step in second._physics_step_deltas))
        for driver_id in (1, 2):
            first_state = first.driver_states[driver_id]
            second_state = second.driver_states[driver_id]
            self.assertAlmostEqual(first_state.total_progress, second_state.total_progress, places=8)
            self.assertAlmostEqual(first_state.speed_kph, second_state.speed_kph, places=6)

    def test_high_frequency_diagnostics_are_bounded(self) -> None:
        engine = self._make_engine()

        for _ in range(MAX_PHYSICS_DIAGNOSTIC_SAMPLES + 64):
            engine.tick(PHYSICS_STEP_SECONDS)

        self.assertEqual(
            len(engine._physics_step_deltas),
            MAX_PHYSICS_DIAGNOSTIC_SAMPLES,
        )
        self.assertEqual(
            len(engine._consumed_physics_frame_ids),
            MAX_PHYSICS_DIAGNOSTIC_SAMPLES,
        )
        self.assertEqual(
            engine._consumed_physics_frame_ids[-1],
            engine._physics_frame,
        )

    def test_total_time_chunking_preserves_full_deterministic_state(self) -> None:
        def snapshot(engine: RaceEngine) -> tuple:
            return (
                tuple(
                    (
                        driver_id,
                        state.model_dump_json(
                            exclude={"planner_generation_ms"},
                        ),
                    )
                    for driver_id, state in sorted(engine.driver_states.items())
                ),
                tuple(sorted(engine._progress_rate.items())),
                engine._physics_frame,
                round(engine.race_elapsed, 12),
                round(engine._physics_accumulator.remainder_s, 12),
                engine.rng.getstate(),
                tuple(sorted((key, repr(value)) for key, value in engine._side_by_side_battles.items())),
                tuple(sorted((key, repr(value)) for key, value in engine._pit_phase.items())),
                tuple(sorted((key, repr(value)) for key, value in engine._battle_effects.items())),
                tuple(sorted(repr(value) for value in engine._pending_incidents)),
            )

        fixed_small = self._make_engine()
        fixed_large = self._make_engine()
        irregular = self._make_engine()
        for _ in range(20):
            fixed_small.tick(0.05)
        for _ in range(10):
            fixed_large.tick(0.1)
        for elapsed in (0.01, 0.04, 0.07, 0.03, 0.11, 0.09, 0.02, 0.16, 0.08, 0.14, 0.05, 0.10, 0.10):
            irregular.tick(elapsed)

        self.assertEqual(fixed_small._physics_frame, 50)
        self.assertEqual(fixed_large._physics_frame, 50)
        self.assertEqual(irregular._physics_frame, 50)
        self.assertEqual(snapshot(fixed_small), snapshot(fixed_large))
        self.assertEqual(snapshot(fixed_small), snapshot(irregular))

    def test_paused_engine_does_not_consume_fixed_steps(self) -> None:
        engine = self._make_engine()
        engine.pause_race()
        engine.tick(0.5)
        self.assertEqual(engine._physics_frame, 0)
        self.assertEqual(engine.race_elapsed, 0.0)

    def test_tick_snapshot_build_is_pure(self) -> None:
        engine = self._make_engine()
        engine.tick(0.1)
        before = engine.model_dump_json() if hasattr(engine, "model_dump_json") else {
            driver_id: state.model_dump_json()
            for driver_id, state in engine.driver_states.items()
        }
        first = engine.build_tick_state()
        middle = {
            driver_id: state.model_dump_json()
            for driver_id, state in engine.driver_states.items()
        }
        second = engine.build_tick_state()
        after = {
            driver_id: state.model_dump_json()
            for driver_id, state in engine.driver_states.items()
        }
        self.assertEqual(middle, after)
        self.assertEqual(before if isinstance(before, dict) else middle, middle)
        self.assertEqual(first.model_dump(), second.model_dump())

    def test_rules_consumers_reject_wrong_phase(self) -> None:
        engine = self._make_engine()
        engine.complete_setup()
        engine._tick_phase_history = [TickPhase.COMMAND]
        engine._tick_phase = TickPhase.COMMAND

        with self.assertRaises(RuntimeError):
            engine._update_positions()
        with self.assertRaises(RuntimeError):
            engine._update_gaps()
        with self.assertRaises(RuntimeError):
            engine._build_pass_events({})
        with self.assertRaises(RuntimeError):
            engine._apply_collision_facts([], [])

        engine._tick_phase = TickPhase.PHYSICS
        with self.assertRaises(RuntimeError):
            engine._update_positions()

        engine._tick_phase = TickPhase.TELEMETRY
        with self.assertRaises(RuntimeError):
            engine._update_positions()

        engine._tick_phase = TickPhase.RULES
        engine._update_positions()

    def test_physics_step_result_is_consumed_once(self) -> None:
        engine = self._make_engine()
        engine.complete_setup()
        engine._tick_phase_history = [TickPhase.PHYSICS, TickPhase.RULES]
        engine._tick_phase = TickPhase.RULES

        first = PhysicsStepResult(physics_frame_id=1, delta_seconds=0.02)
        second = PhysicsStepResult(physics_frame_id=2, delta_seconds=0.02)
        engine._consume_physics_step_result(first)
        engine._consume_physics_step_result(second)
        with self.assertRaises(RuntimeError):
            engine._consume_physics_step_result(first)
        with self.assertRaises(RuntimeError):
            engine._consume_physics_step_result(
                PhysicsStepResult(physics_frame_id=1, delta_seconds=0.02),
            )
        with self.assertRaises(RuntimeError):
            engine._consume_physics_step_result(
                PhysicsStepResult(physics_frame_id=0, delta_seconds=0.02),
            )

    def test_pit_exit_is_independent_of_external_tick_chunking(self) -> None:
        def make_ready_engine() -> RaceEngine:
            circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
            engine = RaceEngine(
                circuit=circuit,
                drivers=load_drivers(),
                teams={team.id: team for team in load_teams()},
                player_team_id=1,
                player_driver_ids=[1, 2],
                seed=123,
                start_sequence_enabled=False,
            )
            state = engine.driver_states[1]
            for other in engine.driver_states.values():
                if other.driver_id != 1:
                    other.retired = True
            entry = circuit.pit_lane.entry_progress
            assert entry is not None
            state.pit_request = TireCompound.HARD
            state.progress = entry - 0.00001
            state.total_progress = state.current_lap + state.progress
            for _ in range(20):
                engine.tick(0.02)
                if state.in_pit:
                    break
            self.assertTrue(state.in_pit)
            engine._pit_phase[1] = "out"
            rejoin = circuit.pit_lane.side_rejoin_progress
            route_length_m = engine._pit_route_length_m()
            engine._pit_route_progress[1] = rejoin - (30.0 * 0.06 / route_length_m)
            engine._pit_route_speed_mps[1] = 30.0
            state.speed_kph = 108.0
            engine._pit_tire[1] = TireCompound.HARD
            return engine

        def snapshot(engine: RaceEngine) -> tuple[object, ...]:
            state = engine.driver_states[1]
            return (
                engine._physics_frame,
                state.in_pit,
                state.progress,
                state.total_progress,
                state.speed_kph,
                state.world_x_m,
                state.world_y_m,
                state.heading_rad,
                state.position,
            )

        large_tick = make_ready_engine()
        split_ticks = make_ready_engine()
        large_events = large_tick.tick(0.1)
        split_events = []
        for _ in range(5):
            split_events.extend(split_ticks.tick(0.02))

        self.assertEqual(snapshot(large_tick), snapshot(split_ticks))
        self.assertEqual(
            [(event.type, event.payload) for event in large_events],
            [(event.type, event.payload) for event in split_events],
        )

    def test_pit_route_pose_is_authoritative_until_track_rejoin(self) -> None:
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        engine = RaceEngine(
            circuit=circuit,
            drivers=load_drivers(),
            teams={team.id: team for team in load_teams()},
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=123,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[1]
        for other in engine.driver_states.values():
            if other.driver_id != 1:
                other.retired = True
        entry = circuit.pit_lane.entry_progress
        exit_ = circuit.pit_lane.exit_progress
        assert entry is not None and exit_ is not None
        state.pit_request = TireCompound.HARD
        state.progress = entry - 0.00001
        state.total_progress = state.current_lap + state.progress
        pre_entry_pose = engine._track_physics.line_pose_at_progress_m(
            "racing_line",
            state.progress,
        )

        engine.tick(0.02)
        self.assertTrue(state.in_pit)
        self.assertLess(
            hypot(
                state.world_x_m - pre_entry_pose[0],
                state.world_y_m - pre_entry_pose[1],
            ),
            2.0,
        )
        self.assertAlmostEqual(
            cos(state.heading_rad - pre_entry_pose[2]),
            1.0,
            places=6,
        )
        entry_pose = engine._track_physics.line_pose_at_progress_m(
            "racing_line",
            entry,
        )
        entry_route_pose = engine._pit_lane_pose_at_progress_m(0.0)
        self.assertAlmostEqual(entry_route_pose[0], entry_pose[0], places=6)
        self.assertAlmostEqual(entry_route_pose[1], entry_pose[1], places=6)
        self.assertAlmostEqual(entry_route_pose[2], entry_pose[2], places=6)

        previous_lane_progress = engine._pit_lane_progress(1)
        moved_pose = (state.world_x_m, state.world_y_m)
        for _ in range(100):
            engine.tick(0.02)
            if not state.in_pit:
                break
            lane_progress = engine._pit_lane_progress(1)
            self.assertGreaterEqual(lane_progress, previous_lane_progress)
            previous_lane_progress = lane_progress
            moved_pose = (state.world_x_m, state.world_y_m)

        self.assertGreater(previous_lane_progress, 0.0)
        main_pose_at_entry = engine._track_physics.line_pose_at_progress_m(
            "racing_line",
            state.progress,
        )
        self.assertGreater(
            hypot(moved_pose[0] - main_pose_at_entry[0], moved_pose[1] - main_pose_at_entry[1]),
            1.0,
        )

        events = []
        last_pit_pose = (state.world_x_m, state.world_y_m)
        while state.in_pit:
            last_pit_pose = (state.world_x_m, state.world_y_m)
            events.extend(engine.tick(0.02))
        pit_exit = next(event for event in events if event.type == "pit_exit")
        self.assertLess(
            hypot(
                state.world_x_m - last_pit_pose[0],
                state.world_y_m - last_pit_pose[1],
            ),
            2.0,
        )
        rejoin = circuit.pit_lane.exit_lane_rejoin_progress
        assert rejoin is not None
        self.assertAlmostEqual(state.progress, rejoin, places=6)
        self.assertGreater(state.progress, exit_)
        self.assertAlmostEqual(pit_exit.payload["world_x_m"], state.world_x_m)
        self.assertAlmostEqual(pit_exit.payload["world_y_m"], state.world_y_m)

    def test_bahrain_complete_pit_cycle_is_one_continuous_physical_route(self) -> None:
        """Golden: entry braking, limiter, box, release and side rejoin agree."""
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        engine = RaceEngine(
            circuit=circuit,
            drivers=load_drivers(),
            teams={team.id: team for team in load_teams()},
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=123,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[1]
        for other in engine.driver_states.values():
            if other.driver_id != state.driver_id:
                other.retired = True

        pit_lane = circuit.pit_lane
        assert pit_lane is not None
        assert pit_lane.entry_progress is not None
        assert pit_lane.exit_progress is not None
        state.pit_request = TireCompound.HARD
        state.speed_kph = 250.0
        state.progress = pit_lane.entry_progress - 2.0 / engine.track_length_m
        state.total_progress = state.current_lap + state.progress
        start_pose = engine._track_physics_for_driver(state).line_pose_at_progress_m(
            "racing_line",
            state.progress,
        )

        samples: list[dict[str, object]] = []
        observed_events = []
        previous_point = (start_pose[0], start_pose[1])
        maximum_step_m = 0.0
        exit_event = None
        for _ in range(6000):
            frame_events = engine.tick(PHYSICS_STEP_SECONDS)
            observed_events.extend(frame_events)
            point = (state.world_x_m, state.world_y_m)
            maximum_step_m = max(
                maximum_step_m,
                hypot(
                    point[0] - previous_point[0],
                    point[1] - previous_point[1],
                ),
            )
            previous_point = point
            if state.in_pit:
                samples.append(
                    {
                        "frame": engine._physics_frame,
                        "route_progress": engine._pit_lane_progress(state.driver_id),
                        "main_route_progress": engine._pit_main_route_progress(
                            state.driver_id
                        ),
                        "exit_lane_progress": engine._pit_exit_lane_progress_value(
                            state.driver_id
                        ),
                        "phase": engine._pit_phase[state.driver_id],
                        "merge_state": engine._pit_merge_state[state.driver_id],
                        "speed_kph": state.speed_kph,
                        "point": point,
                        "track_progress": state.progress,
                    }
                )
            exit_event = next(
                (event for event in frame_events if event.type == "pit_exit"),
                None,
            )
            if exit_event is not None:
                break

        self.assertIsNotNone(exit_event)
        assert exit_event is not None
        self.assertTrue(samples)
        self.assertLess(maximum_step_m, 3.0)
        self.assertEqual(
            [event.type for event in observed_events if event.type.startswith("pit_")][0],
            "pit_entry",
        )
        self.assertEqual(
            [event.type for event in observed_events if event.type in {"pit_entry", "pit_exit"}],
            ["pit_entry", "pit_exit"],
        )

        main_samples = [sample for sample in samples if sample["phase"] != "exit_lane"]
        exit_lane_samples = [sample for sample in samples if sample["phase"] == "exit_lane"]
        main_route_progress = [
            float(sample["main_route_progress"]) for sample in main_samples
        ]
        exit_lane_progress = [
            float(sample["exit_lane_progress"]) for sample in exit_lane_samples
        ]
        self.assertEqual(main_route_progress, sorted(main_route_progress))
        self.assertEqual(exit_lane_progress, sorted(exit_lane_progress))
        self.assertLess(main_route_progress[0], pit_lane.speed_limit_start)
        self.assertGreaterEqual(main_route_progress[-1], pit_lane.speed_limit_end)
        self.assertLessEqual(exit_lane_progress[0], 0.01)
        self.assertGreater(exit_lane_progress[-1], 0.9)

        limit_entry = next(
            sample
            for sample in main_samples
            if abs(
                float(sample["route_progress"]) - pit_lane.speed_limit_start
            ) <= 1e-12
        )
        self.assertAlmostEqual(
            float(limit_entry["speed_kph"]),
            PIT_LANE_SPEED_LIMIT_KPH,
            places=6,
        )
        pre_limit_speeds = [
            float(sample["speed_kph"])
            for sample in main_samples
            if float(sample["route_progress"]) < pit_lane.speed_limit_start
        ]
        self.assertGreater(pre_limit_speeds[0], PIT_LANE_SPEED_LIMIT_KPH)
        self.assertTrue(
            all(
                following <= previous + 1e-6
                for previous, following in zip(
                    pre_limit_speeds,
                    pre_limit_speeds[1:],
                )
            ),
            msg=str(pre_limit_speeds),
        )

        limited_samples = [
            sample
            for sample in main_samples
            if pit_lane.speed_limit_start
            <= float(sample["route_progress"])
            <= pit_lane.speed_limit_end
        ]
        self.assertTrue(limited_samples)
        self.assertLessEqual(
            max(float(sample["speed_kph"]) for sample in limited_samples),
            PIT_LANE_SPEED_LIMIT_KPH + 1e-6,
        )
        stop_samples = [sample for sample in samples if sample["phase"] == "stop"]
        self.assertTrue(stop_samples)
        self.assertTrue(
            all(abs(float(sample["speed_kph"])) <= 1e-9 for sample in stop_samples)
        )
        self.assertEqual(state.tire_compound, TireCompound.HARD)

        limit_exit = next(
            sample
            for sample in main_samples
            if sample["phase"] == "out"
            and abs(float(sample["route_progress"]) - pit_lane.speed_limit_end)
            <= 1e-12
        )
        self.assertAlmostEqual(
            float(limit_exit["speed_kph"]),
            PIT_LANE_SPEED_LIMIT_KPH,
            places=6,
        )
        released_samples = [
            sample
            for sample in samples
            if float(sample["route_progress"]) > pit_lane.speed_limit_end
        ]
        self.assertTrue(released_samples)
        self.assertGreater(
            max(float(sample["speed_kph"]) for sample in released_samples),
            PIT_LANE_SPEED_LIMIT_KPH,
        )
        self.assertIn("merge", {sample["merge_state"] for sample in released_samples})

        middle = min(
            main_samples,
            key=lambda sample: abs(float(sample["main_route_progress"]) - 0.5),
        )
        middle_track_pose = engine._track_physics_for_driver(
            state
        ).line_pose_at_progress_m(
            "racing_line",
            float(middle["track_progress"]),
        )
        middle_point = middle["point"]
        assert isinstance(middle_point, tuple)
        self.assertGreater(
            hypot(
                middle_point[0] - middle_track_pose[0],
                middle_point[1] - middle_track_pose[1],
            ),
            2.0,
        )

        self.assertFalse(state.in_pit)
        self.assertEqual(state.pit_count, 1)
        self.assertGreater(state.speed_kph, PIT_LANE_SPEED_LIMIT_KPH)
        assert pit_lane.exit_lane_rejoin_progress is not None
        self.assertAlmostEqual(
            state.progress,
            pit_lane.exit_lane_rejoin_progress,
            places=6,
        )
        self.assertAlmostEqual(exit_event.payload["world_x_m"], state.world_x_m)
        self.assertAlmostEqual(exit_event.payload["world_y_m"], state.world_y_m)

    def test_pit_route_entry_and_exit_tangents_match_track_heading(self) -> None:
        drivers = load_drivers()[:2]
        teams = {team.id: team for team in load_teams()}
        for circuit in load_circuits():
            if circuit.pit_lane is None or circuit.pit_lane.entry_progress is None:
                continue
            engine = RaceEngine(
                circuit=circuit,
                drivers=drivers,
                teams=teams,
                player_team_id=drivers[0].team_id,
                player_driver_ids=[drivers[0].id],
                seed=123,
                start_sequence_enabled=False,
            )
            entry_heading = engine._track_physics.line_pose_at_progress_m(
                "racing_line",
                circuit.pit_lane.entry_progress,
            )[2]
            exit_heading = engine._track_physics.line_pose_at_progress_m(
                "racing_line",
                circuit.pit_lane.exit_progress,
            )[2]
            pit_entry_heading = engine._pit_lane_pose_at_progress_m(0.0001)[2]
            pit_exit_heading = engine._pit_lane_pose_at_progress_m(0.9999)[2]

            self.assertAlmostEqual(
                cos(pit_entry_heading - entry_heading),
                1.0,
                places=6,
                msg=f"{circuit.name} pit entry",
            )
            self.assertAlmostEqual(
                cos(pit_exit_heading - exit_heading),
                1.0,
                places=6,
                msg=f"{circuit.name} pit exit",
            )


if __name__ == "__main__":
    unittest.main()
