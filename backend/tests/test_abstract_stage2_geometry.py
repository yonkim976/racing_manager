"""Stage 2 FULL/ABSTRACT geometry parity and metric-frame tests."""

from __future__ import annotations

import json
import unittest
from math import hypot, pi

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import TrackConditions
from engines.abstract.runtime import AbstractRaceEngine, AbstractSessionSnapshot, SingleVehiclePoseSynthesizer
from engines.abstract.runtime.broadcast import AbstractBroadcastSession
from simulation.race_engine import RaceEngine
from session import RaceSession
from simulation.track_physics import DRIVING_LINE_RACING, build_track_physics_profile


class AbstractStage2GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = {circuit.id: circuit for circuit in load_circuits()}
        cls.drivers = load_drivers()
        cls.teams = load_teams()

    def _broadcast_session(self, circuit_id: int) -> AbstractBroadcastSession:
        circuit = self.circuits[circuit_id]
        snapshot = AbstractSessionSnapshot.from_content(
            session_id=f"stage2-baseline-{circuit_id}",
            session_seed=42,
            circuit=circuit,
            drivers=self.drivers,
            teams=self.teams,
        )
        result = AbstractRaceEngine(snapshot).run_race(total_laps=1, tick_seconds=0.1)
        player_team = self.teams[0]
        return AbstractBroadcastSession(
            snapshot=snapshot,
            result=result,
            circuit=circuit,
            player_team=player_team,
            player_drivers=[driver for driver in self.drivers if driver.team_id == player_team.id],
            drivers=self.drivers,
            teams=self.teams,
            track_conditions=TrackConditions(),
            thermal_preset=None,
            conditions_source="stage2-baseline",
        )

    def _full_race_info(self, circuit_id: int) -> dict:
        circuit = self.circuits[circuit_id]
        player_team = self.teams[0]
        engine = RaceEngine(
            circuit=circuit,
            drivers=self.drivers,
            teams={team.id: team for team in self.teams},
            player_team_id=player_team.id,
            player_driver_ids=[driver.id for driver in self.drivers if driver.team_id == player_team.id],
        )
        session = RaceSession(
            session_id=f"stage2-full-{circuit_id}",
            engine=engine,
            circuit=circuit,
            player_team=player_team,
            player_drivers=[driver for driver in self.drivers if driver.team_id == player_team.id],
        )
        return session.race_info.model_dump(mode="json")

    def test_bahrain_and_rbr_abstract_race_info_is_complete(self) -> None:
        required_fields = (
            "grid_slots",
            "racing_line_profile",
            "track_width_profile",
            "racing_line_coords",
            "driving_line_coords",
            "driving_line_lengths_m",
            "pit_lane_coords",
            "pit_exit_lane_coords",
        )
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                info = self._broadcast_session(circuit_id).race_info
                for field in required_fields:
                    self.assertTrue(info[field], f"{circuit_id} race_info.{field} is empty")

    def test_bahrain_and_rbr_abstract_pose_matches_full_compiled_sampler(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                circuit = self.circuits[circuit_id]
                profile = build_track_physics_profile(circuit)
                snapshot = AbstractSessionSnapshot.from_content(
                    session_id=f"stage2-pose-{circuit_id}",
                    session_seed=42,
                    circuit=circuit,
                    drivers=self.drivers,
                    teams=self.teams,
                )
                synthesizer = SingleVehiclePoseSynthesizer(snapshot)
                position_errors = []
                heading_errors = []
                for sample_index in range(1000):
                    progress = sample_index / 1000.0
                    abstract_pose = synthesizer.pose_at("probe", progress)
                    full_x, full_y, full_heading = profile.line_pose_at_progress_m(
                        DRIVING_LINE_RACING,
                        progress,
                    )
                    position_errors.append(
                        hypot(abstract_pose.world_x_m - full_x, abstract_pose.world_y_m - full_y)
                    )
                    heading_delta = (abstract_pose.heading_rad - full_heading + pi) % (2.0 * pi) - pi
                    heading_errors.append(abs(heading_delta) * 180.0 / pi)
                position_errors.sort()
                heading_errors.sort()
                self.assertLessEqual(position_errors[949], 0.10)
                self.assertLessEqual(position_errors[-1], 0.30)
                self.assertLessEqual(heading_errors[949], 0.5)

    def test_abstract_race_info_geometry_is_not_render_units_in_metric_pose(self) -> None:
        circuit = self.circuits[3]
        profile = build_track_physics_profile(circuit)
        snapshot = AbstractSessionSnapshot.from_content(
            session_id="stage2-coordinate-space",
            session_seed=42,
            circuit=circuit,
            drivers=self.drivers,
            teams=self.teams,
        )
        pose = SingleVehiclePoseSynthesizer(snapshot).pose_at(1, 0.0)
        expected_x, expected_y, _ = profile.line_pose_at_progress_m(DRIVING_LINE_RACING, 0.0)
        self.assertAlmostEqual(pose.world_x_m, expected_x, delta=0.30)
        self.assertAlmostEqual(pose.world_y_m, expected_y, delta=0.30)

    def test_full_and_abstract_race_info_share_geometry_and_metric_ratios(self) -> None:
        geometry_fields = (
            "track_length_m",
            "world_origin_x_render",
            "world_origin_y_render",
            "world_meters_per_render_unit",
            "track_width_m",
            "car_width_m",
            "car_length_m",
            "wheelbase_m",
            "grid_slots",
            "racing_line_profile",
            "track_width_profile",
            "racing_line_coords",
            "racing_line_length_m",
            "predicted_racing_lap_time",
            "driving_line_coords",
            "driving_line_lengths_m",
            "predicted_line_lap_times",
            "track_coords",
            "start_finish_index",
            "pit_lane_coords",
            "pit_exit_lane_coords",
            "pit_wall_coords",
            "pit_box_offset",
            "pit_lane_width_m",
            "pit_speed_limit_kph",
            "pit_side_entry_progress",
            "pit_speed_limit_start",
            "pit_box_progress",
            "pit_speed_limit_end",
            "pit_side_rejoin_progress",
        )
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                abstract_info = self._broadcast_session(circuit_id).race_info
                full_info = self._full_race_info(circuit_id)
                for field in geometry_fields:
                    self.assertEqual(abstract_info[field], full_info[field], field)
                slots = abstract_info["grid_slots"]
                self.assertEqual(len(slots), 20)
                self.assertEqual(
                    [slot["position"] for slot in slots],
                    list(range(1, 21)),
                )
                average_width_m = sum(
                    sample[1] + sample[2]
                    for sample in abstract_info["track_width_profile"]
                ) / len(abstract_info["track_width_profile"])
                self.assertAlmostEqual(
                    abstract_info["car_width_m"] / average_width_m,
                    full_info["car_width_m"] / average_width_m,
                    places=12,
                )
                round_trip = json.loads(json.dumps(abstract_info, ensure_ascii=False))
                self.assertEqual(
                    round_trip["world_meters_per_render_unit"],
                    abstract_info["world_meters_per_render_unit"],
                )

    def test_compiled_metric_length_and_frame_round_trip_meet_stage2_limit(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                circuit = self.circuits[circuit_id]
                profile = build_track_physics_profile(circuit)
                frame = profile.coordinate_frame
                self.assertIsNotNone(frame)
                self.assertLessEqual(
                    abs(frame.track_length_m - circuit.track_length_m) / circuit.track_length_m,
                    0.001,
                )
                local_point = profile.line_pose_at_progress_m(DRIVING_LINE_RACING, 0.37)[:2]
                render_point = frame.from_local_m(*local_point)
                self.assertAlmostEqual(
                    hypot(*(
                        frame.to_local_m(*render_point)[index] - local_point[index]
                        for index in (0, 1)
                    )),
                    0.0,
                    places=9,
                )


if __name__ == "__main__":
    unittest.main()
