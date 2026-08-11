"""Contracts for the opt-in progress-authoritative race kernel."""

from __future__ import annotations

import unittest
from collections import Counter
from dataclasses import replace
from math import hypot
from types import SimpleNamespace

from data_loader import load_circuits, load_drivers, load_teams
from simulation.abstract import (
    AbstractRaceEngine,
    AbstractSessionSnapshot,
    PROGRESS_RACE_ENGINE_VERSION,
    ProgressBroadcastCursor,
    ProgressRaceCursor,
)
from simulation.track_display import build_track_display_geometry


class AbstractProgressRaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = {circuit.id: circuit for circuit in load_circuits()}
        cls.drivers = load_drivers()
        cls.teams = load_teams()

    def snapshot(self, *, circuit_id: int = 3, seed: int = 42):
        return AbstractSessionSnapshot.from_content(
            session_id=f"progress-race-{circuit_id}-{seed}",
            session_seed=seed,
            circuit=self.circuits[circuit_id],
            drivers=self.drivers,
            teams=self.teams,
        )

    def test_standing_start_moves_all_twenty_after_shared_hold(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(),
            total_laps=1,
            tick_seconds=0.10,
        )
        initial_distances = {
            vehicle.driver_id: vehicle.race_distance_m
            for vehicle in cursor.current_tick.vehicles
        }
        for _ in range(10):
            hold = cursor.advance_one_tick()
        self.assertEqual(sum(vehicle.logical_speed_mps == 0.0 for vehicle in hold.vehicles), 20)
        self.assertEqual(
            {vehicle.driver_id: vehicle.race_distance_m for vehicle in hold.vehicles},
            initial_distances,
        )

        launch = cursor.advance_one_tick()
        self.assertEqual(sum(vehicle.logical_speed_mps > 0.0 for vehicle in launch.vehicles), 20)
        self.assertTrue(
            all(
                vehicle.race_distance_m > initial_distances[vehicle.driver_id]
                for vehicle in launch.vehicles
            )
        )

    def test_result_is_exactly_tick_size_invariant(self) -> None:
        snapshot = self.snapshot()
        results = {
            tick_seconds: ProgressRaceCursor(
                snapshot,
                total_laps=3,
                tick_seconds=tick_seconds,
            ).run_to_finish()
            for tick_seconds in (0.05, 0.10, 0.50)
        }
        baseline = results[0.10]
        for tick_seconds, result in results.items():
            with self.subTest(tick_seconds=tick_seconds):
                self.assertEqual(result.finish_order, baseline.finish_order)
                self.assertEqual(result.classification, baseline.classification)
                self.assertEqual(result.logical_duration_s, baseline.logical_duration_s)
                self.assertEqual(
                    result.canonical_result_hash,
                    baseline.canonical_result_hash,
                )
        self.assertEqual(len({result.integration_tick_count for result in results.values()}), 3)

    def test_distance_is_monotonic_and_ordering_is_contiguous(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(circuit_id=4),
            total_laps=1,
            tick_seconds=0.50,
        )
        previous = {
            vehicle.driver_id: vehicle.race_distance_m
            for vehicle in cursor.current_tick.vehicles
        }
        for _ in range(80):
            tick = cursor.advance_one_tick()
            self.assertEqual(
                [vehicle.position for vehicle in tick.vehicles],
                list(range(1, 21)),
            )
            self.assertEqual(len({vehicle.driver_id for vehicle in tick.vehicles}), 20)
            for vehicle in tick.vehicles:
                self.assertGreaterEqual(
                    vehicle.race_distance_m,
                    previous[vehicle.driver_id] - 1e-9,
                )
                self.assertGreaterEqual(vehicle.gap_to_ahead_m, 0.0)
                previous[vehicle.driver_id] = vehicle.race_distance_m

    def test_gap_and_interval_are_time_authority_with_common_loop_anchors(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=43),
            total_laps=2,
            tick_seconds=0.50,
        )
        for _ in range(60):
            tick = cursor.advance_one_tick()

        leader = tick.vehicles[0]
        self.assertEqual(leader.gap_to_leader_s, 0.0)
        self.assertTrue(leader.timing_gap_valid)
        self.assertTrue(leader.interval_timing_gap_valid)
        self.assertTrue(
            all(vehicle.gap_to_leader_s >= 0.0 for vehicle in tick.vehicles)
        )
        self.assertTrue(
            all(vehicle.interval_to_ahead_s >= 0.0 for vehicle in tick.vehicles)
        )
        self.assertTrue(
            all(vehicle.timing_gap_valid for vehicle in tick.vehicles[1:])
        )
        self.assertTrue(
            all(vehicle.interval_timing_gap_valid for vehicle in tick.vehicles[1:])
        )

    def test_time_authority_is_external_tick_size_invariant(self) -> None:
        snapshot = self.snapshot(seed=44)
        ticks = {}
        for tick_seconds in (0.05, 0.10, 0.50):
            cursor = ProgressRaceCursor(
                snapshot,
                total_laps=2,
                tick_seconds=tick_seconds,
            )
            while cursor.logical_time_s < 30.0 - 1e-9:
                tick = cursor.advance_one_tick()
            ticks[tick_seconds] = tuple(
                (
                    vehicle.driver_id,
                    round(vehicle.gap_to_leader_s, 9),
                    round(vehicle.interval_to_ahead_s, 9),
                    vehicle.timing_gap_valid,
                    vehicle.interval_timing_gap_valid,
                )
                for vehicle in tick.vehicles
            )
        self.assertEqual(ticks[0.05], ticks[0.10])
        self.assertEqual(ticks[0.10], ticks[0.50])

    def test_drs_and_dirty_air_use_snapshot_zones_and_logical_gap(self) -> None:
        snapshot = self.snapshot(seed=45)
        self.assertGreater(len(snapshot.track.drs_zones), 0)
        cursor = ProgressRaceCursor(
            snapshot,
            total_laps=3,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        observed_drs = []
        observed_dirty_air = False
        while not cursor.finished:
            tick = cursor.advance_one_tick()
            leader_progress = tick.vehicles[0].total_progress
            if leader_progress < 1.0 - 1e-9:
                self.assertFalse(any(vehicle.drs_active for vehicle in tick.vehicles))
            observed_drs.extend(
                vehicle for vehicle in tick.vehicles if vehicle.drs_active
            )
            observed_dirty_air = observed_dirty_air or any(
                vehicle.dirty_air_active for vehicle in tick.vehicles
            )

        self.assertTrue(observed_drs)
        self.assertTrue(observed_dirty_air)
        for vehicle in observed_drs:
            progress = vehicle.total_progress % 1.0
            self.assertTrue(
                any(
                    (
                        zone.start_progress <= progress <= zone.end_progress
                        if zone.start_progress <= zone.end_progress
                        else progress >= zone.start_progress
                        or progress <= zone.end_progress
                    )
                    for zone in snapshot.track.drs_zones
                )
            )
        metrics = cursor.metrics
        self.assertGreater(metrics["drs_activation_count"], 0)
        self.assertGreater(metrics["dirty_air_active_decision_count"], 0)

    def test_sc_clears_drs_and_applies_one_lap_restart_lockout(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=46),
            total_laps=5,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        while not any(car.drs_active for car in cursor._cars.values()):
            cursor.advance_one_tick()
        cursor._request_race_control(
            "sc",
            now=cursor.logical_time_s,
            cause="drs_lockout_fixture",
            severity=1.0,
        )
        self.assertFalse(any(car.drs_active for car in cursor._cars.values()))
        self.assertTrue(
            all(not car.drs_eligibility for car in cursor._cars.values())
        )
        while cursor.race_control_state != "green":
            cursor.advance_one_tick()
        leader = cursor._cars[cursor._race_order[0]]
        self.assertAlmostEqual(
            cursor._drs_enable_after_leader_distance_m,
            leader.race_distance_m + cursor.lap_length_m,
            places=6,
        )
        while (
            cursor._cars[cursor._race_order[0]].race_distance_m
            < cursor._drs_enable_after_leader_distance_m - 1e-9
        ):
            tick = cursor.advance_one_tick()
            self.assertFalse(any(vehicle.drs_active for vehicle in tick.vehicles))

    def test_drs_train_and_three_car_group_are_bounded_logical_context(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=0),
            total_laps=10,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        observed_train = False
        observed_group = False
        while not cursor.finished:
            tick = cursor.advance_one_tick()
            leader_progress = tick.vehicles[0].total_progress
            train_vehicles = [
                vehicle for vehicle in tick.vehicles if vehicle.drs_train_size >= 3
            ]
            if leader_progress < 1.0 - 1e-9:
                self.assertFalse(train_vehicles)
            observed_train = observed_train or bool(train_vehicles)
            group_vehicles = [
                vehicle
                for vehicle in tick.vehicles
                if vehicle.maneuver_group_size >= 3
            ]
            if not group_vehicles:
                continue
            observed_group = True
            grouped = {}
            for vehicle in group_vehicles:
                grouped.setdefault(vehicle.maneuver_group_id, []).append(vehicle)
            for members in grouped.values():
                self.assertEqual(len(members), 3)
                self.assertEqual(
                    {vehicle.maneuver_group_corridor_index for vehicle in members},
                    {0, 1, 2},
                )
            self.assertTrue(
                all(vehicle.maneuver_group_phase in {"three_wide", "merge"}
                    for vehicle in group_vehicles)
            )

        self.assertTrue(observed_train)
        self.assertTrue(observed_group)
        self.assertEqual(cursor.metrics["max_maneuver_group_size"], 3)
        self.assertEqual(cursor.metrics["implicit_rank_swap_count"], 0)
        self.assertEqual(cursor.metrics["distance_reversal_count"], 0)

    def test_racecraft_v2_group_events_are_tick_and_checkpoint_invariant(self) -> None:
        snapshot = self.snapshot(seed=1)
        results = tuple(
            ProgressRaceCursor(
                snapshot,
                total_laps=3,
                tick_seconds=tick_seconds,
                enable_incidents=False,
                enable_pit=False,
            ).run_to_finish()
            for tick_seconds in (0.10, 0.50)
        )
        self.assertEqual(results[0].logical_events, results[1].logical_events)
        self.assertEqual(results[0].metrics, results[1].metrics)
        self.assertEqual(
            results[0].canonical_result_hash,
            results[1].canonical_result_hash,
        )

        cursor = ProgressRaceCursor(
            snapshot,
            total_laps=3,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        while cursor.logical_time_s < 40.0 - 1e-9:
            cursor.advance_one_tick()
        checkpoint = cursor.capture_runtime_checkpoint()
        first = tuple(cursor.advance_one_tick().to_dict() for _ in range(20))
        cursor.restore_runtime_checkpoint(checkpoint)
        second = tuple(cursor.advance_one_tick().to_dict() for _ in range(20))
        self.assertEqual(first, second)

    def test_overtake_animation_has_distinct_eased_phases(self) -> None:
        cursor = ProgressBroadcastCursor(
            self.snapshot(seed=47),
            total_laps=2,
            tick_seconds=0.10,
            display_geometry=build_track_display_geometry(
                self.circuits[3],
                grid_driver_ids=self.snapshot(seed=47).initial_grid_order,
            ),
        )

        def target(state: str, progress: float, side: int = 1) -> float:
            return cursor._target_lateral_offset(
                SimpleNamespace(
                    pit_state="none",
                    race_status="running",
                    driver_id=1,
                    target_driver_id=2,
                    maneuver_side=side,
                    maneuver_progress=progress,
                    traffic_state=state,
                )
            )

        self.assertLess(target("attack", 0.0), target("attack", 0.5))
        self.assertLess(target("attack", 0.5), target("attack", 1.0))
        self.assertAlmostEqual(target("side_by_side", 0.5), 2.10)
        self.assertGreater(target("clearance", 0.0), target("clearance", 0.5))
        self.assertAlmostEqual(target("clearance", 1.0), 0.0)
        self.assertLess(target("attack", 1.0, -1), 0.0)

        grouped = SimpleNamespace(
            pit_state="none",
            race_status="running",
            driver_id=3,
            target_driver_id=2,
            maneuver_side=0,
            maneuver_progress=0.5,
            traffic_state="follow",
            maneuver_group_size=3,
            maneuver_group_phase="three_wide",
            maneuver_group_corridor_index=0,
        )
        self.assertAlmostEqual(cursor._target_lateral_offset(grouped), -2.10)
        grouped.maneuver_group_corridor_index = 1
        self.assertAlmostEqual(cursor._target_lateral_offset(grouped), 0.0)
        grouped.maneuver_group_corridor_index = 2
        self.assertAlmostEqual(cursor._target_lateral_offset(grouped), 2.10)

    def test_authoritative_state_contains_no_pose_or_force_fields(self) -> None:
        state = ProgressRaceCursor(
            self.snapshot(),
            total_laps=1,
        ).current_tick.vehicles[0].to_dict()
        self.assertFalse(
            {
                "world_x_m",
                "world_y_m",
                "heading_rad",
                "lateral_offset_m",
                "acceleration_mps2",
                "slip_ratio",
                "tire_force_n",
            }
            & set(state)
        )
        self.assertIn("race_distance_m", state)
        self.assertIn("logical_speed_mps", state)

    def test_engine_exposes_progress_path_without_replacing_v4_default(self) -> None:
        engine = AbstractRaceEngine(self.snapshot())
        cursor = engine.create_progress_cursor(total_laps=1, tick_seconds=0.50)
        self.assertIsInstance(cursor, ProgressRaceCursor)
        progress = engine.run_progress_race(total_laps=1, tick_seconds=0.50)
        self.assertEqual(len(progress.finish_order), 20)
        self.assertEqual(
            progress.to_dict()["engine_version"],
            PROGRESS_RACE_ENGINE_VERSION,
        )
        self.assertEqual(engine.snapshot.abstract_engine_version, "abstract-stage4-traffic-v4")

    def test_progress_broadcast_is_a_bounded_presentation_derivative(self) -> None:
        snapshot = self.snapshot(seed=142)
        cursor = ProgressBroadcastCursor(
            snapshot,
            total_laps=2,
            tick_seconds=0.10,
            display_geometry=build_track_display_geometry(
                self.circuits[3],
                grid_driver_ids=snapshot.initial_grid_order,
            ),
        )
        previous = cursor.current_frame
        maximum_jump_m = 0.0
        while not cursor.finished:
            frame = cursor.advance_one_tick().frame
            old_by_driver = {
                vehicle.driver_id: vehicle for vehicle in previous.vehicles
            }
            maximum_jump_m = max(
                maximum_jump_m,
                max(
                    hypot(
                        vehicle.world_x_m - old_by_driver[vehicle.driver_id].world_x_m,
                        vehicle.world_y_m - old_by_driver[vehicle.driver_id].world_y_m,
                    )
                    for vehicle in frame.vehicles
                ),
            )
            self.assertEqual(len(frame.vehicles), 20)
            previous = frame
        result = cursor.finalize()
        instant = ProgressRaceCursor(
            snapshot,
            total_laps=2,
            tick_seconds=0.10,
        ).run_to_finish()
        self.assertEqual(len(result.classification), 20)
        self.assertEqual(result.canonical_result_hash, instant.canonical_result_hash)
        self.assertLessEqual(maximum_jump_m, (370.0 / 3.6) * 0.10 + 1e-9)
        self.assertEqual(cursor.timing_checkpoints[0].checkpoint_kind, "race_start")
        self.assertEqual(cursor.timing_checkpoints[-1].checkpoint_kind, "race_finish")

    def test_progress_broadcast_checkpoint_replays_same_command_future(self) -> None:
        snapshot = self.snapshot(seed=143)
        cursor = ProgressBroadcastCursor(
            snapshot,
            total_laps=2,
            tick_seconds=0.10,
            display_geometry=build_track_display_geometry(
                self.circuits[3],
                grid_driver_ids=snapshot.initial_grid_order,
            ),
        )
        for _ in range(100):
            cursor.advance_one_tick()
        checkpoint = cursor.export_runtime_checkpoint()
        driver_id = snapshot.initial_grid_order[0]

        first_event = cursor.set_pace_mode(driver_id, "ATTACK")
        first_frames = tuple(
            cursor.advance_one_tick().frame.to_dict() for _ in range(30)
        )
        cursor.restore_runtime_checkpoint(checkpoint)
        second_event = cursor.set_pace_mode(driver_id, "ATTACK")
        second_frames = tuple(
            cursor.advance_one_tick().frame.to_dict() for _ in range(30)
        )

        self.assertEqual(first_event, second_event)
        self.assertEqual(first_frames, second_frames)

    def test_vehicle_performance_remains_a_pace_input_not_a_pose_model(self) -> None:
        driver = self.drivers[0]
        baseline = AbstractSessionSnapshot.from_content(
            session_id="progress-performance-baseline",
            session_seed=42,
            circuit=self.circuits[3],
            drivers=[driver],
            teams=self.teams,
        )
        entry = baseline.entries[0]
        upgraded_entry = replace(
            entry,
            vehicle=replace(
                entry.vehicle,
                performance=entry.vehicle.performance.with_upgrades(
                    {
                        "power": 0.08,
                        "braking": 0.08,
                        "traction": 0.08,
                    }
                ),
            ),
        )
        upgraded = replace(baseline, entries=(upgraded_entry,))
        baseline_result = ProgressRaceCursor(
            baseline,
            total_laps=3,
            tick_seconds=0.50,
        ).run_to_finish()
        upgraded_result = ProgressRaceCursor(
            upgraded,
            total_laps=3,
            tick_seconds=0.50,
        ).run_to_finish()
        self.assertLess(
            upgraded_result.logical_duration_s,
            baseline_result.logical_duration_s,
        )

    def test_fifty_seven_laps_finishes_with_bounded_current_state(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(),
            total_laps=57,
            tick_seconds=1.0,
        )
        result = cursor.run_to_finish()
        self.assertEqual(len(result.finish_order), 20)
        self.assertLess(result.integration_tick_count, 6_000)
        self.assertEqual(len(cursor.current_tick.vehicles), 20)
        self.assertEqual(cursor.current_tick.tick_index, result.integration_tick_count)

    def test_traffic_uses_explicit_maneuver_chain_without_hidden_rank_swaps(self) -> None:
        result = ProgressRaceCursor(
            self.snapshot(),
            total_laps=10,
            tick_seconds=0.50,
        ).run_to_finish()
        metrics = dict(result.metrics)
        event_counts = {
            event_type: sum(
                event.event_type == event_type for event in result.logical_events
            )
            for event_type in {
                "attack_started",
                "attack_side_by_side",
                "overtake_completed",
                "attack_failed",
            }
        }
        self.assertGreater(metrics["attack_started_count"], 0)
        self.assertGreater(metrics["overtake_completed_count"], 0)
        self.assertEqual(metrics["implicit_rank_swap_count"], 0)
        self.assertEqual(metrics["distance_reversal_count"], 0)
        early_aborts = sum(
            event.event_type == "attack_failed"
            and dict(event.payload).get("phase_at_abort") == "attack"
            for event in result.logical_events
        )
        self.assertLessEqual(
            event_counts["attack_side_by_side"],
            event_counts["attack_started"],
        )
        self.assertEqual(
            event_counts["attack_started"] - event_counts["attack_side_by_side"],
            early_aborts,
        )
        self.assertEqual(
            event_counts["attack_started"],
            event_counts["overtake_completed"] + event_counts["attack_failed"],
        )

    def test_incident_hazard_is_contextual_and_tick_invariant(self) -> None:
        snapshot = self.snapshot(seed=42)
        results = tuple(
            ProgressRaceCursor(
                snapshot,
                total_laps=10,
                tick_seconds=tick_seconds,
            ).run_to_finish()
            for tick_seconds in (0.10, 0.50)
        )
        first, second = results
        self.assertEqual(first.logical_events, second.logical_events)
        self.assertEqual(first.metrics, second.metrics)
        self.assertEqual(first.canonical_result_hash, second.canonical_result_hash)
        self.assertGreater(dict(first.metrics)["incident_count"], 0)

        cursor = ProgressRaceCursor(snapshot, total_laps=1)
        car = next(iter(cursor._cars.values()))
        clear_rate = cursor._incident_rate_per_s(car)
        target = next(
            candidate
            for candidate in cursor._cars.values()
            if candidate.entry.driver_id != car.entry.driver_id
        )
        car.traffic_state = "side_by_side"
        car.target_driver_id = target.entry.driver_id
        target.race_distance_m = car.race_distance_m + 1.0
        self.assertGreater(cursor._incident_rate_per_s(car), clear_rate)

    def test_traffic_and_incidents_can_be_disabled_for_v1_pace_baseline(self) -> None:
        result = ProgressRaceCursor(
            self.snapshot(),
            total_laps=3,
            tick_seconds=0.50,
            enable_traffic=False,
            enable_incidents=False,
            enable_pit=False,
        ).run_to_finish()
        event_types = [event.event_type for event in result.logical_events]
        self.assertEqual(event_types.count("race_started"), 1)
        self.assertEqual(event_types.count("driver_finished"), 20)
        self.assertEqual(set(event_types), {"race_started", "driver_finished"})
        self.assertTrue(all(value == 0 for _, value in result.metrics))

    def test_disabling_incidents_blocks_pair_contact_hazard(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=0),
            total_laps=10,
            tick_seconds=0.50,
            enable_incidents=False,
        )
        result = cursor.run_to_finish()
        metrics = dict(result.metrics)
        self.assertEqual(metrics["incident_count"], 0)
        self.assertEqual(metrics["pair_contact_count"], 0)
        self.assertEqual(metrics["retirement_count"], 0)
        self.assertFalse(
            {
                "contact_started",
                "driver_retired",
                "lockup_started",
                "run_wide_started",
                "spin_started",
            }
            & {event.event_type for event in result.logical_events}
        )

    def test_public_distance_never_reverses_at_fine_cadence(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=42),
            total_laps=3,
            tick_seconds=0.05,
        )
        previous = {
            vehicle.driver_id: vehicle.race_distance_m
            for vehicle in cursor.current_tick.vehicles
        }
        while not cursor.finished:
            for vehicle in cursor.advance_one_tick().vehicles:
                self.assertGreaterEqual(
                    vehicle.race_distance_m,
                    previous[vehicle.driver_id] - 1e-9,
                )
                previous[vehicle.driver_id] = vehicle.race_distance_m

    def test_progress_pit_has_complete_timeline_and_replaces_tires(self) -> None:
        snapshot = self.snapshot(seed=42)
        strategy = {entry.driver_id: (1,) for entry in snapshot.entries}
        cursor = ProgressRaceCursor(
            snapshot,
            total_laps=2,
            tick_seconds=0.50,
            pit_strategy=strategy,
        )
        previous = {
            vehicle.driver_id: vehicle.race_distance_m
            for vehicle in cursor.current_tick.vehicles
        }
        while not cursor.finished:
            for vehicle in cursor.advance_one_tick().vehicles:
                self.assertGreaterEqual(
                    vehicle.race_distance_m,
                    previous[vehicle.driver_id] - 1e-9,
                )
                previous[vehicle.driver_id] = vehicle.race_distance_m
        result = cursor.run_to_finish()
        event_counts = Counter(event.event_type for event in result.logical_events)
        for event_type in (
            "pit_requested",
            "pit_lane_entry",
            "pit_stop_started",
            "pit_stop_completed",
            "pit_exit",
        ):
            self.assertEqual(event_counts[event_type], 20)
        self.assertEqual(dict(result.metrics)["pit_stop_completed_count"], 20)
        self.assertEqual(dict(result.metrics)["implicit_rank_swap_count"], 0)
        self.assertEqual(dict(result.metrics)["distance_reversal_count"], 0)
        self.assertTrue(
            all(vehicle.pit_state == "none" for vehicle in cursor.current_tick.vehicles)
        )
        self.assertTrue(
            all(vehicle.pit_stop_count == 1 for vehicle in cursor.current_tick.vehicles)
        )
        self.assertTrue(
            all(
                vehicle.physical_compound == "C2"
                and vehicle.tire_role == "MEDIUM"
                and 0.0 < vehicle.wear_laps < 1.0
                and vehicle.stint_lap == 1
                for vehicle in cursor.current_tick.vehicles
            )
        )
        for entry in snapshot.entries:
            timeline = [
                event.logical_time_s
                for event in result.logical_events
                if entry.driver_id in event.driver_ids
                and event.event_type
                in {
                    "pit_requested",
                    "pit_lane_entry",
                    "pit_stop_started",
                    "pit_stop_completed",
                    "pit_exit",
                }
            ]
            self.assertEqual(timeline, sorted(timeline))
            self.assertEqual(len(timeline), 5)

    def test_progress_pit_result_is_tick_invariant(self) -> None:
        snapshot = self.snapshot(seed=42)
        strategy = {entry.driver_id: (2,) for entry in snapshot.entries}
        results = tuple(
            ProgressRaceCursor(
                snapshot,
                total_laps=5,
                tick_seconds=tick_seconds,
                pit_strategy=strategy,
            ).run_to_finish()
            for tick_seconds in (0.05, 0.10, 0.50)
        )
        for result in results[1:]:
            self.assertEqual(results[0].classification, result.classification)
            self.assertEqual(results[0].logical_events, result.logical_events)
            self.assertEqual(results[0].metrics, result.metrics)
            self.assertEqual(
                results[0].canonical_result_hash,
                result.canonical_result_hash,
            )

    def test_progress_pit_strategy_validation_and_multiple_stints(self) -> None:
        snapshot = self.snapshot(seed=42)
        driver_id = snapshot.entries[0].driver_id
        with self.assertRaises(ValueError):
            ProgressRaceCursor(
                snapshot,
                total_laps=3,
                pit_strategy={driver_id: (3,)},
            )
        with self.assertRaises(ValueError):
            ProgressRaceCursor(
                snapshot,
                total_laps=3,
                pit_strategy={"missing-driver": (1,)},
            )

        cursor = ProgressRaceCursor(
            snapshot,
            total_laps=4,
            tick_seconds=0.50,
            pit_strategy={driver_id: (1, 2)},
        )
        cursor.run_to_finish()
        vehicle = next(
            item for item in cursor.current_tick.vehicles if item.driver_id == driver_id
        )
        self.assertEqual(vehicle.pit_stop_count, 2)
        self.assertEqual(vehicle.physical_compound, "C1")
        self.assertEqual(vehicle.tire_role, "HARD")

    def test_pair_contact_damages_both_cars_and_classifies_retirement(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=42),
            total_laps=2,
            tick_seconds=0.50,
            enable_pit=False,
        )
        first, second = tuple(cursor._cars.values())[:2]
        baseline_second_pace = cursor._pace_multiplier(second)
        first.damage_level = 0.74
        cursor._apply_pair_contact(first, second, severity=1.0, now=0.0)
        retired_distance_m = first.race_distance_m

        self.assertGreater(first.damage_level, 0.74)
        self.assertGreater(second.damage_level, 0.0)
        self.assertLess(cursor._pace_multiplier(second), baseline_second_pace)
        self.assertIsNotNone(first.retirement_time_s)
        self.assertEqual(first.retirement_reason, "contact_damage")
        self.assertEqual(dict(cursor.metrics)["pair_contact_count"], 1)
        self.assertGreaterEqual(dict(cursor.metrics)["retirement_count"], 1)

        result = cursor.run_to_finish()
        self.assertEqual(len(result.classification), 20)
        self.assertEqual(len({item.driver_id for item in result.classification}), 20)
        self.assertIn(first.entry.driver_id, result.retired_driver_ids)
        retired = next(
            item
            for item in result.classification
            if item.driver_id == first.entry.driver_id
        )
        self.assertEqual(retired.status, "retired")
        self.assertIsNone(retired.finish_time_s)
        self.assertEqual(retired.retirement_reason, "contact_damage")
        self.assertEqual(retired.race_distance_m, retired_distance_m)
        statuses = [item.status for item in result.classification]
        self.assertEqual(statuses, sorted(statuses, key=lambda value: value == "retired"))
        event_types = [event.event_type for event in result.logical_events]
        self.assertIn("contact_started", event_types)
        self.assertIn("driver_retired", event_types)

        repeat_cursor = ProgressRaceCursor(
            self.snapshot(seed=42),
            total_laps=2,
            tick_seconds=0.10,
            enable_pit=False,
        )
        repeat_first, repeat_second = tuple(repeat_cursor._cars.values())[:2]
        repeat_first.damage_level = 0.74
        repeat_cursor._apply_pair_contact(
            repeat_first,
            repeat_second,
            severity=1.0,
            now=0.0,
        )
        repeat = repeat_cursor.run_to_finish()
        self.assertEqual(result.classification, repeat.classification)
        self.assertEqual(result.logical_events, repeat.logical_events)
        self.assertEqual(result.metrics, repeat.metrics)
        self.assertEqual(result.canonical_result_hash, repeat.canonical_result_hash)

    def test_natural_pair_contact_is_explicit_and_order_safe(self) -> None:
        result = ProgressRaceCursor(
            self.snapshot(seed=0),
            total_laps=10,
            tick_seconds=0.50,
        ).run_to_finish()
        metrics = dict(result.metrics)
        contact_events = [
            event
            for event in result.logical_events
            if event.event_type == "contact_started"
        ]
        self.assertGreater(metrics["pair_contact_count"], 0)
        self.assertEqual(len(contact_events), metrics["pair_contact_count"])
        self.assertTrue(all(len(event.driver_ids) == 2 for event in contact_events))
        self.assertEqual(metrics["implicit_rank_swap_count"], 0)
        self.assertEqual(metrics["distance_reversal_count"], 0)

    def test_vsc_freezes_order_and_returns_to_green(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(circuit_id=4, seed=7),
            total_laps=2,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        while cursor.logical_time_s < 20.0 - 1e-9:
            cursor.advance_one_tick()
        frozen_order = tuple(
            vehicle.driver_id for vehicle in cursor.current_tick.vehicles
        )
        cursor._request_race_control(
            "vsc",
            now=20.0,
            cause="test_fixture",
            severity=0.60,
        )
        attacks_at_start = cursor.metrics["attack_started_count"]
        while cursor.race_control_state != "green":
            tick = cursor.advance_one_tick()
            self.assertEqual(
                tuple(vehicle.driver_id for vehicle in tick.vehicles),
                frozen_order,
            )
            self.assertIn(tick.race_control_state, {"vsc", "green"})
            self.assertTrue(
                all(
                    vehicle.logical_speed_mps > 0.1
                    for vehicle in tick.vehicles
                    if vehicle.race_status == "running"
                )
            )
        self.assertEqual(cursor.logical_time_s, 35.0)
        self.assertEqual(cursor.metrics["attack_started_count"], attacks_at_start)
        self.assertEqual(cursor.current_tick.race_control_phase, "green")
        event_types = [event.event_type for event in cursor.logical_events]
        self.assertLess(event_types.index("vsc_started"), event_types.index("vsc_ended"))

    def test_local_yellow_and_vsc_to_sc_upgrade_are_explicit(self) -> None:
        local = ProgressRaceCursor(
            self.snapshot(seed=42),
            total_laps=2,
            enable_incidents=False,
            enable_pit=False,
        )
        first, second = tuple(local._cars.values())[:2]
        local._apply_pair_contact(first, second, severity=0.10, now=0.0)
        self.assertEqual(local.race_control_state, "green")
        self.assertEqual(local.metrics["local_yellow_count"], 1)
        self.assertIn(
            "local_yellow_started",
            [event.event_type for event in local.logical_events],
        )

        upgraded = ProgressRaceCursor(
            self.snapshot(seed=42),
            total_laps=2,
            enable_incidents=False,
            enable_pit=False,
        )
        upgraded._request_race_control(
            "vsc", now=0.0, cause="initial_fixture", severity=0.70
        )
        upgraded._request_race_control(
            "sc", now=0.0, cause="upgrade_fixture", severity=1.0
        )
        self.assertEqual(upgraded.race_control_state, "sc")
        self.assertEqual(upgraded.race_control_phase, "deploying")
        self.assertEqual(upgraded.metrics["vsc_period_count"], 1)
        self.assertEqual(upgraded.metrics["safety_car_period_count"], 1)
        deployed = next(
            event
            for event in upgraded.logical_events
            if event.event_type == "safety_car_deployed"
        )
        self.assertTrue(dict(deployed.payload)["upgraded_from_vsc"])

    def test_sc_lifecycle_is_tick_invariant_and_never_stops_queue(self) -> None:
        snapshot = self.snapshot(seed=42)
        results = []
        for tick_seconds in (0.10, 0.50):
            cursor = ProgressRaceCursor(
                snapshot,
                total_laps=3,
                tick_seconds=tick_seconds,
                enable_incidents=False,
                enable_pit=False,
            )
            while cursor.logical_time_s < 20.0 - 1e-9:
                cursor.advance_one_tick()
            order_at_deployment = tuple(
                vehicle.driver_id for vehicle in cursor.current_tick.vehicles
            )
            cursor._request_race_control(
                "sc",
                now=20.0,
                cause="test_fixture",
                driver_ids=(order_at_deployment[-1],),
                severity=1.0,
            )
            observed_phases = []
            while cursor.race_control_state != "green":
                tick = cursor.advance_one_tick()
                observed_phases.append(tick.race_control_phase)
                self.assertEqual(
                    tuple(vehicle.driver_id for vehicle in tick.vehicles),
                    order_at_deployment,
                )
                self.assertTrue(
                    all(
                        vehicle.logical_speed_mps > 0.1
                        for vehicle in tick.vehicles
                        if vehicle.race_status == "running"
                        and vehicle.pit_state != "stop"
                    )
                )
            self.assertEqual(
                list(dict.fromkeys(observed_phases)),
                [
                    "deploying",
                    "catch_up",
                    "queue_formed",
                    "restart_ready",
                    "in_this_lap",
                    "green",
                ],
            )
            results.append(cursor.run_to_finish())

        first, second = results
        self.assertEqual(first.classification, second.classification)
        self.assertEqual(first.logical_events, second.logical_events)
        self.assertEqual(first.metrics, second.metrics)
        self.assertEqual(first.canonical_result_hash, second.canonical_result_hash)
        event_types = [event.event_type for event in first.logical_events]
        lifecycle = [
            "safety_car_deployed",
            "safety_car_catch_up_started",
            "safety_car_queue_formed",
            "safety_car_restart_ready",
            "safety_car_in_this_lap",
            "race_restarted",
        ]
        self.assertEqual(
            [event_type for event_type in event_types if event_type in lifecycle],
            lifecycle,
        )

    def test_sc_checkpoint_replays_unpublished_future(self) -> None:
        cursor = ProgressRaceCursor(
            self.snapshot(seed=42),
            total_laps=3,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        while cursor.logical_time_s < 20.0 - 1e-9:
            cursor.advance_one_tick()
        cursor._request_race_control(
            "sc", now=20.0, cause="test_fixture", severity=1.0
        )
        while cursor.logical_time_s < 30.0 - 1e-9:
            cursor.advance_one_tick()
        checkpoint = cursor.capture_runtime_checkpoint()
        first = cursor.run_to_finish()
        cursor.restore_runtime_checkpoint(checkpoint)
        second = cursor.run_to_finish()
        self.assertEqual(first.classification, second.classification)
        self.assertEqual(first.logical_events, second.logical_events)
        self.assertEqual(first.metrics, second.metrics)
        self.assertEqual(first.integration_tick_count, second.integration_tick_count)
        self.assertEqual(first.canonical_result_hash, second.canonical_result_hash)

        incompatible = ProgressRaceCursor(
            self.snapshot(seed=43),
            total_laps=3,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        with self.assertRaises(ValueError):
            incompatible.restore_runtime_checkpoint(checkpoint)

    def test_sc_preserves_lap_deficit_and_allows_explicit_pit_merge(self) -> None:
        snapshot = self.snapshot(seed=42)
        strategy = {entry.driver_id: (1,) for entry in snapshot.entries}
        cursor = ProgressRaceCursor(
            snapshot,
            total_laps=2,
            tick_seconds=0.50,
            enable_incidents=False,
            pit_strategy=strategy,
        )
        while not any(car.pit_state != "none" for car in cursor._cars.values()):
            cursor.advance_one_tick()
        now = cursor.logical_time_s
        cursor._request_race_control(
            "sc", now=now, cause="pit_test_fixture", severity=1.0
        )
        result = cursor.run_to_finish()
        metrics = dict(result.metrics)
        self.assertEqual(metrics["pit_stop_completed_count"], 20)
        self.assertEqual(metrics["safety_car_period_count"], 1)
        self.assertEqual(metrics["restart_count"], 1)
        self.assertGreater(metrics["pit_order_change_count"], 0)
        self.assertEqual(metrics["implicit_rank_swap_count"], 0)
        self.assertEqual(metrics["distance_reversal_count"], 0)

        lapped = ProgressRaceCursor(
            snapshot,
            total_laps=4,
            tick_seconds=0.50,
            enable_incidents=False,
            enable_pit=False,
        )
        while lapped.logical_time_s < 30.0 - 1e-9:
            lapped.advance_one_tick()
        leader = lapped._cars[lapped._race_order[0]]
        tail = lapped._cars[lapped._race_order[-1]]
        distance_m = leader.race_distance_m - lapped.lap_length_m - 100.0
        tail.race_distance_m = distance_m
        segment_index, fraction = lapped._segment_location(distance_m)
        tail.segment_index = segment_index
        tail.segment_elapsed_s = tail.segment_durations_s[segment_index] * fraction
        tail.decision_distance_m = distance_m
        lapped._request_race_control(
            "sc",
            now=30.0,
            cause="lapped_test_fixture",
            driver_ids=(tail.entry.driver_id,),
            severity=1.0,
        )
        self.assertEqual(lapped._sc_lap_deficit[tail.entry.driver_id], 1)
        while lapped.race_control_state != "green":
            lapped.advance_one_tick()
        self.assertGreaterEqual(
            (leader.race_distance_m - tail.race_distance_m) / lapped.lap_length_m,
            1.0,
        )
        self.assertEqual(lapped.metrics["implicit_rank_swap_count"], 0)


if __name__ == "__main__":
    unittest.main()
