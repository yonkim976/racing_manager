"""Stage 4R correction contracts.

These tests are intentionally written against the final Stage 4R contract.
They capture the known conditional-approval failures before implementation:
there is no immutable reservation value object, no independent reservation
intersection audit, and a maneuver starts at pull_out without approach.
"""

from __future__ import annotations

import unittest
from dataclasses import FrozenInstanceError, replace

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import TrackConditions
from simulation.abstract import AbstractRaceEngine, AbstractSessionSnapshot
from simulation.abstract.racecraft import (
    TrafficCorridorReservation,
    accepted_swept_distance_m,
    audit_accepted_frame_reservations,
    audit_reservation_intersections,
    local_corridor_assessment,
    swept_vehicle_intersects_reservation,
)


class AbstractStage4CorrectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = {item.id: item for item in load_circuits()}
        cls.drivers = load_drivers()
        cls.teams = load_teams()

    def snapshot(self, circuit_id: int = 4, seed: int = 0) -> AbstractSessionSnapshot:
        return AbstractSessionSnapshot.from_content(
            session_id=f"stage4r-correction-{circuit_id}-{seed}",
            session_seed=seed,
            circuit=self.circuits[circuit_id],
            drivers=self.drivers,
            teams=self.teams,
        )

    def reservation(self, corridor_id: str, *, lateral_min: float = -1.2) -> TrafficCorridorReservation:
        return TrafficCorridorReservation(
            corridor_id=corridor_id,
            attacker_id=1,
            defender_id=2,
            start_arc_distance_m=100.0,
            end_arc_distance_m=150.0,
            start_time_s=10.0,
            expiry_time_s=12.0,
            lateral_min_m=lateral_min,
            lateral_max_m=lateral_min + 4.6,
            side_assignment=((1, "outside"), (2, "inside")),
            required_width_m=4.55,
            available_width_m=5.2,
            segment_type="straight",
            corner_phase="entry",
            third_vehicle_ids=(3,),
            admission_reason="deterministic-test-fixture",
        )

    def test_reservation_is_immutable_and_serializable(self) -> None:
        reservation = self.reservation("corridor:test")
        self.assertEqual(reservation.to_dict()["corridor_id"], "corridor:test")
        with self.assertRaises(FrozenInstanceError):
            reservation.expiry_time_s = 99.0  # type: ignore[misc]

    def test_reservation_intersection_audit_detects_time_arc_lateral_overlap(self) -> None:
        first = self.reservation("corridor:first")
        second = self.reservation("corridor:second", lateral_min=0.0)
        audit = audit_reservation_intersections(
            (first, second),
            line_length_m=1_000.0,
        )
        self.assertEqual(audit["admitted_reservation_conflict_count"], 1)
        self.assertEqual(audit["reservation_conflict_pairs"], (("corridor:first", "corridor:second"),))

    def test_accepted_frame_audit_checks_unlisted_third_vehicle(self) -> None:
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        cursor = AbstractTrafficSimulationCursor(self.snapshot(), total_laps=1, tick_seconds=0.10)
        try:
            frame = cursor.current_frame
            vehicles = tuple(
                replace(
                    vehicle,
                    line_distance_m=125.0,
                    speed_mps=0.0,
                    lateral_offset_m=0.0,
                )
                if vehicle.driver_id == 11
                else vehicle
                for vehicle in frame.vehicles
            )
            frame = replace(frame, vehicles=vehicles)
            reservation = replace(
                self.reservation("corridor:unlisted-third"),
                start_time_s=0.0,
                expiry_time_s=4.0,
                third_vehicle_ids=(),
            )
            audit = audit_accepted_frame_reservations(
                frame,
                (reservation,),
                line_length_m=cursor.distance_contract.line_length_m,
                vehicle_length_m=cursor.geometry.car_length_m,
                vehicle_width_m=cursor.geometry.car_width_m,
            )
            self.assertEqual(audit["third_vehicle_occupancy_conflict_count"], 1)
            self.assertEqual(
                audit["third_vehicle_conflict_pairs"],
                (("corridor:unlisted-third", 11),),
            )
        finally:
            cursor.dispose()

    def test_reservation_covers_full_deadline_and_rejoin_horizon(self) -> None:
        cursor, _ = self._fixture_cursor(
            circuit_id=4,
            progress=0.55,
            outcome="abort",
            gap_m=40.0,
        )
        try:
            reservation = cursor.active_reservations[0]
            self.assertGreaterEqual(
                reservation.expiry_time_s - reservation.start_time_s,
                4.0 + 1.5,
            )
            attacker = cursor.cars[reservation.attacker_id]
            defender = cursor.cars[reservation.defender_id]
            expected_end = max(
                attacker.line_distance_m
                + accepted_swept_distance_m(
                    attacker.speed_mps,
                    4.0 + 1.5 + 2.5,
                    target_speed_mps=98.0,
                ),
                defender.line_distance_m
                + accepted_swept_distance_m(
                    defender.speed_mps,
                    4.0 + 1.5 + 2.5,
                    target_speed_mps=98.0,
                ),
            ) + cursor.geometry.car_length_m * 0.5
            self.assertGreaterEqual(reservation.end_arc_distance_m, expected_end - 1e-6)
        finally:
            cursor.dispose()

    def test_time_aligned_occupancy_does_not_reserve_the_whole_future_arc_at_once(self) -> None:
        common = {
            "reservation_start_arc_m": -2.7,
            "reservation_end_arc_m": 410.0,
            "reservation_horizon_s": 8.0,
            "vehicle_speed_mps": 50.0,
            "vehicle_target_speed_mps": 50.0,
            "vehicle_lateral_offset_m": 0.0,
            "line_length_m": 1_000.0,
            "vehicle_length_m": 5.4,
            "vehicle_width_m": 1.9,
            "lateral_min_m": -2.3,
            "lateral_max_m": 2.3,
            "reservation_attacker_start_arc_m": 0.0,
            "reservation_attacker_speed_mps": 50.0,
            "reservation_attacker_target_speed_mps": 50.0,
            "reservation_defender_start_arc_m": 10.0,
            "reservation_defender_speed_mps": 50.0,
            "reservation_defender_target_speed_mps": 50.0,
        }
        self.assertFalse(
            swept_vehicle_intersects_reservation(
                **common,
                vehicle_start_arc_m=200.0,
            )
        )
        self.assertTrue(
            swept_vehicle_intersects_reservation(
                **common,
                vehicle_start_arc_m=15.0,
            )
        )

    def test_maneuver_cannot_start_at_pull_out_without_approach(self) -> None:
        cursor, corridor_id = self._fixture_cursor(
            circuit_id=4,
            progress=0.55,
            outcome="success",
            gap_m=40.0,
        )
        try:
            starts = [
                event
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
                and event.event_type == "attack_started"
            ]
            self.assertEqual(len(starts), 1)
            self.assertEqual(dict(starts[0].payload).get("maneuver_phase"), "approach")
        finally:
            cursor.dispose()

    def _fixture_cursor(
        self,
        *,
        circuit_id: int,
        progress: float,
        outcome: str,
        gap_m: float = 40.0,
        finish_ready: bool = False,
    ):
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        cursor = AbstractTrafficSimulationCursor(
            self.snapshot(circuit_id=circuit_id),
            total_laps=2 if not finish_ready else 1,
            tick_seconds=0.10,
        )
        corridor_id = cursor.inject_maneuver_fixture(
            attacker_id=2,
            defender_id=1,
            progress=progress,
            gap_m=gap_m,
            attacker_speed_mps=60.0,
            defender_speed_mps=100.0 if finish_ready else 0.0,
            outcome=outcome,
            finish_ready=finish_ready,
        )
        return cursor, corridor_id

    def _advance_until_event(self, cursor, corridor_id: str, event_type: str, limit: int = 300):
        frames = [cursor.current_frame]
        for _ in range(limit):
            frames.append(cursor.advance_one_tick().frame)
            if any(
                event.event_id.startswith(corridor_id)
                and event.event_type == event_type
                for event in cursor.events
            ):
                break
        return frames

    def test_scenario_01_fast_car_follows_when_corridor_is_absent(self) -> None:
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        cursor = AbstractTrafficSimulationCursor(
            self.snapshot(circuit_id=3), total_laps=1, tick_seconds=0.10
        )
        try:
            previous = cursor.current_frame
            for _ in range(8):
                tick = cursor.advance_one_tick()
                self.assertFalse(cursor.maneuvers)
                for old, new in zip(previous.vehicles, tick.frame.vehicles):
                    self.assertLessEqual(
                        abs(new.line_distance_m - old.line_distance_m),
                        max(old.speed_mps, new.speed_mps) * 0.10 + 0.5,
                    )
                previous = tick.frame
        finally:
            cursor.dispose()

    def test_scenario_02_wide_straight_success_has_full_phase_chain(self) -> None:
        cursor, corridor_id = self._fixture_cursor(
            circuit_id=4, progress=0.36, outcome="success"
        )
        try:
            frames = self._advance_until_event(cursor, corridor_id, "rejoin_complete")
            event_types = [
                event.event_type
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
            ]
            self.assertEqual(
                event_types,
                [
                    "attack_approach_started",
                    "attack_started",
                    "attack_pull_out_started",
                    "attack_overlap_started",
                    "overtake_crossing_confirmed",
                    "overtake_clearance_confirmed",
                    "overtake_completed",
                    "rejoin_complete",
                ],
            )
            self.assertEqual(cursor.metrics["attack_terminal_event_count"], 1)
            self.assertEqual(cursor.metrics["attack_terminal_event_duplicate_count"], 0)
            self.assertEqual(cursor.active_reservation_count, 0)
            self.assertEqual(dict(next(event for event in cursor.events if event.event_type == "overtake_completed").payload)["rank_swap"], "atomic")
            self.assertGreater(len(frames), 1)
        finally:
            cursor.dispose()

    def test_scenario_03_heavy_braking_crossing_clearance_and_swap(self) -> None:
        cursor, corridor_id = self._fixture_cursor(
            circuit_id=4, progress=0.30, outcome="success"
        )
        try:
            self._advance_until_event(cursor, corridor_id, "rejoin_complete")
            attack = next(
                event
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
                and event.event_type == "attack_started"
            )
            reservation = dict(attack.payload)["reservation"]
            self.assertEqual(reservation["segment_type"], "heavy_braking")
            event_types = [
                event.event_type
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
            ]
            self.assertLess(
                event_types.index("overtake_crossing_confirmed"),
                event_types.index("overtake_clearance_confirmed"),
            )
            self.assertEqual(cursor.metrics["crossing_before_rank_swap_count"], 0)
        finally:
            cursor.dispose()

    def test_scenario_04_local_width_rejects_corridor(self) -> None:
        assessment = local_corridor_assessment(
            local_left_width_m=2.2,
            local_right_width_m=2.2,
            racing_line_offset_m=0.0,
            segment_type="straight",
            corner_phase="entry",
            remaining_distance_m=30.0,
            gap_m=10.0,
            closing_speed_mps=4.0,
            third_vehicle_conflict=False,
            reservation_conflict=False,
        )
        self.assertFalse(assessment.allowed)
        self.assertEqual(assessment.reason_code, "corridor_too_narrow")

    def test_scenario_05_corner_phase_rejects_attack(self) -> None:
        assessment = local_corridor_assessment(
            local_left_width_m=5.0,
            local_right_width_m=5.0,
            racing_line_offset_m=0.0,
            segment_type="traction",
            corner_phase="apex",
            remaining_distance_m=5.0,
            gap_m=10.0,
            closing_speed_mps=4.0,
            third_vehicle_conflict=False,
            reservation_conflict=False,
        )
        self.assertFalse(assessment.allowed)
        self.assertEqual(assessment.reason_code, "corner_phase_unsafe")

    def test_scenario_06_third_vehicle_anticipated_occupancy_rejects(self) -> None:
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        cursor = AbstractTrafficSimulationCursor(
            self.snapshot(circuit_id=4), total_laps=1, tick_seconds=0.10
        )
        try:
            third = next(vehicle for vehicle in cursor.current_frame.vehicles if vehicle.driver_id == 11)
            reservation = TrafficCorridorReservation(
                corridor_id="third:occupancy",
                attacker_id=1,
                defender_id=2,
                start_arc_distance_m=third.line_distance_m - 10.0,
                end_arc_distance_m=third.line_distance_m + 10.0,
                start_time_s=0.0,
                expiry_time_s=1.0,
                lateral_min_m=third.lateral_offset_m - 3.0,
                lateral_max_m=third.lateral_offset_m + 3.0,
                side_assignment=((1, "outside"), (2, "inside")),
                required_width_m=4.55,
                available_width_m=8.0,
                segment_type="straight",
                corner_phase="entry",
                third_vehicle_ids=(11,),
                admission_reason="deterministic-test-fixture",
            )
            audit = audit_accepted_frame_reservations(
                cursor.current_frame,
                (reservation,),
                line_length_m=cursor.distance_contract.line_length_m,
            )
            self.assertEqual(audit["third_vehicle_occupancy_conflict_count"], 1)
            self.assertEqual(audit["third_vehicle_conflict_pairs"], (("third:occupancy", 11),))
        finally:
            cursor.dispose()

    def test_scenario_07_active_reservation_intersection_rejects_second(self) -> None:
        first = self.reservation("active:first")
        second = self.reservation("active:second", lateral_min=-0.5)
        audit = audit_reservation_intersections((first, second), line_length_m=1_000.0)
        self.assertEqual(audit["admitted_reservation_conflict_count"], 1)
        assessment = local_corridor_assessment(
            local_left_width_m=5.0,
            local_right_width_m=5.0,
            racing_line_offset_m=0.0,
            segment_type="straight",
            corner_phase="entry",
            remaining_distance_m=30.0,
            gap_m=10.0,
            closing_speed_mps=4.0,
            third_vehicle_conflict=False,
            reservation_conflict=True,
        )
        self.assertFalse(assessment.allowed)
        self.assertEqual(assessment.reason_code, "corridor_conflict")

    def test_scenario_08_defense_fallback_rejoins_with_time_loss(self) -> None:
        cursor, corridor_id = self._fixture_cursor(
            circuit_id=4, progress=0.36, outcome="defense"
        )
        try:
            starting_line = cursor.cars[2].line_distance_m
            self._advance_until_event(cursor, corridor_id, "rejoin_complete")
            event_types = [
                event.event_type
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
            ]
            self.assertEqual(
                event_types,
                [
                    "attack_approach_started",
                    "attack_started",
                    "attack_pull_out_started",
                    "attack_overlap_started",
                    "defense_hold",
                    "attack_fallback_started",
                    "rejoin_complete",
                ],
            )
            self.assertLess(
                cursor.cars[2].line_distance_m,
                starting_line + 60.0 * cursor.current_time_s,
            )
        finally:
            cursor.dispose()

    def test_scenario_09_lap_wrap_side_by_side_is_monotonic_and_clear(self) -> None:
        cursor, corridor_id = self._fixture_cursor(
            circuit_id=4, progress=0.995, outcome="success"
        )
        try:
            previous = {driver_id: car.line_distance_m for driver_id, car in cursor.cars.items()}
            frames = self._advance_until_event(cursor, corridor_id, "rejoin_complete")
            for frame in frames[1:]:
                for vehicle in frame.vehicles:
                    self.assertGreaterEqual(vehicle.line_distance_m, previous[vehicle.driver_id] - 1e-9)
                    previous[vehicle.driver_id] = vehicle.line_distance_m
            self.assertEqual(cursor.metrics["body_overlap_count"], 0)
            self.assertEqual(cursor.metrics["crossing_before_rank_swap_count"], 0)
        finally:
            cursor.dispose()

    def test_scenario_10_finish_closes_incomplete_attack_once(self) -> None:
        cursor, corridor_id = self._fixture_cursor(
            circuit_id=4,
            progress=0.999,
            outcome="defense",
            finish_ready=True,
        )
        try:
            while not cursor.finished:
                cursor.advance_one_tick()
            self.assertEqual(len(cursor.finish_order), 20)
            self.assertEqual(len(set(cursor.finish_order)), 20)
            event_types = [
                event.event_type
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
            ]
            self.assertIn("attack_aborted", event_types)
            self.assertIn("rejoin_complete", event_types)
            self.assertEqual(cursor.metrics["attack_terminal_event_count"], 1)
            self.assertEqual(cursor.metrics["attack_terminal_event_duplicate_count"], 0)
        finally:
            cursor.dispose()


if __name__ == "__main__":
    unittest.main()
