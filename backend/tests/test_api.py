"""API-level tests for lightweight validation endpoints."""

from __future__ import annotations

import unittest
import os
import asyncio
import threading
import struct
import time
from typing import Any
from unittest.mock import AsyncMock, patch

from starlette.requests import Request

from data_loader import load_circuits
from engines import simulation_engine_factory
from main import desktop_diagnostics, run_qualifying_session, setup_race, validate_circuit
from models.schemas import (
    AbstractRaceAuthority,
    QualifyingRequest,
    RaceSetupRequest,
    SimulationMode,
)
from engines.abstract.runtime.broadcast import (
    abstract_broadcast_session_manager,
)


class _FakeWebSocket:
    def __init__(self) -> None:
        self.messages = []
        self.closed = False

    async def send_json(self, payload) -> None:
        self.messages.append(payload)

    async def close(self, **_kwargs) -> None:
        self.closed = True


class _FailingWebSocket(_FakeWebSocket):
    async def send_json(self, payload) -> None:
        raise RuntimeError("client disconnected during initial snapshot")


class _BinaryFakeWebSocket(_FakeWebSocket):
    def __init__(self) -> None:
        super().__init__()
        self.binary_messages = []

    async def send_bytes(self, payload) -> None:
        self.binary_messages.append(payload)


class _CountingWebSocket:
    """Consume a long controlled replay without retaining every pose payload."""

    def __init__(self) -> None:
        self.message_types = []
        self.pose_payload_count = 0
        self.event_ids = []
        self.race_end_count = 0
        self.closed = False

    async def send_json(self, payload) -> None:
        message_type = payload["type"]
        self.message_types.append(message_type)
        if message_type == "pose_tick":
            self.pose_payload_count += len(payload["poses"])
        elif message_type == "race_events":
            self.event_ids.extend(event["event_id"] for event in payload["events"])
        elif message_type == "race_end":
            self.race_end_count += 1

    async def close(self, **_kwargs) -> None:
        self.closed = True


class CircuitValidationApiTests(unittest.TestCase):
    def test_validate_circuit_accepts_compilable_seed_circuit(self) -> None:
        circuit = load_circuits()[0]

        response = validate_circuit(circuit)

        self.assertTrue(response["ok"])
        self.assertEqual(response["errors"], [])
        self.assertGreater(len(response["circuit"].track_coords), 20)


class DesktopDiagnosticsApiTests(unittest.TestCase):
    @staticmethod
    def _request(token: str = "") -> Request:
        return Request({
            "type": "http",
            "method": "GET",
            "path": "/api/desktop/diagnostics",
            "headers": [(b"x-f1-desktop-token", token.encode())],
            "query_string": b"",
            "server": ("127.0.0.1", 1),
            "client": ("127.0.0.1", 2),
            "scheme": "http",
        })

    def test_diagnostics_requires_desktop_mode_and_token(self) -> None:
        with patch.dict(os.environ, {"F1_DESKTOP_MODE": "1", "F1_DESKTOP_TOKEN": "secret"}):
            with self.assertRaisesRegex(Exception, "Desktop diagnostics token required"):
                desktop_diagnostics(self._request("wrong"))
            payload = desktop_diagnostics(self._request("secret"))

        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["tire_temperature"]["schema_version"], 2)
        self.assertFalse(payload["active_session"])
        self.assertIn("rss_bytes", payload["process"])
        self.assertIn("cpu_percent", payload["process"])
        self.assertEqual(payload["safety_car"]["drivers"], [])
        self.assertEqual(
            payload["tire_temperature"]["thresholds"]["rear_surface_overheat_c"],
            130.0,
        )
        self.assertIn("peak", payload["tire_temperature"])
        self.assertNotIn("positions", payload)

    def test_diagnostics_is_disabled_in_normal_web_mode(self) -> None:
        with patch.dict(os.environ, {"F1_DESKTOP_MODE": "0", "F1_DESKTOP_TOKEN": "secret"}):
            with self.assertRaisesRegex(Exception, "Desktop diagnostics are disabled"):
                desktop_diagnostics(self._request("secret"))

    def test_diagnostics_reports_active_abstract_buffered_session(self) -> None:
        async def run() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=91,
                )
            )
            with patch.dict(
                os.environ,
                {"F1_DESKTOP_MODE": "1", "F1_DESKTOP_TOKEN": "secret"},
            ):
                payload = desktop_diagnostics(self._request("secret"))
            self.assertTrue(payload["active_session"])
            self.assertEqual(payload["source_mode"], "abstract")
            self.assertTrue(payload["producer_task_exists"])
            self.assertGreaterEqual(
                payload["producer_buffer_count"],
                payload["producer_start_buffer_ticks"],
            )
            await abstract_broadcast_session_manager.clear_async()

        asyncio.run(run())


class SimulationModeApiTests(unittest.TestCase):
    def test_progress_v5_instant_is_selected_explicitly(self) -> None:
        response = asyncio.run(
            setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_INSTANT,
                    abstract_engine=AbstractRaceAuthority.PROGRESS_V5,
                    session_seed=141,
                )
            )
        )

        self.assertEqual(response.abstract_engine, AbstractRaceAuthority.PROGRESS_V5)
        self.assertEqual(response.abstract_result_summary["abstract_engine"], "PROGRESS_V5")
        self.assertEqual(len(response.abstract_result_summary["finish_order"]), 20)
        self.assertIsNotNone(response.abstract_result_summary["canonical_result_hash"])

    def test_progress_v5_broadcast_uses_progress_authority_and_live_commands(self) -> None:
        async def run() -> None:
            response = await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    abstract_engine=AbstractRaceAuthority.PROGRESS_V5,
                    session_seed=142,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            self.assertEqual(response.abstract_engine, AbstractRaceAuthority.PROGRESS_V5)
            self.assertEqual(active.race_info["authority_mode"], "progress_v5")
            self.assertEqual(
                active.race_info["presentation_contract"],
                "progress-v5-derived-pose-v2",
            )
            self.assertIsNone(active.result)
            self.assertEqual(len(active._current_frame().vehicles), 20)
            timing_payload = active._position_payload(active._current_frame())
            self.assertEqual(timing_payload[0]["gap"], "LEADER")
            self.assertTrue(
                all("m" not in item["gap"] for item in timing_payload[1:])
            )
            self.assertTrue(
                all(item["gap_seconds"] is not None for item in timing_payload[1:])
            )
            self.assertTrue(
                all(item["source_mode"] == "abstract" for item in timing_payload)
            )

            driver_id = active.player_drivers[0].id
            command = await active.handle_command(
                {
                    "type": "set_pace_mode",
                    "driver_id": driver_id,
                    "pace_mode": "ATTACK",
                }
            )
            self.assertEqual(command["type"], "command_ack")
            vehicle = next(
                item
                for item in active._current_frame().vehicles
                if item.driver_id == driver_id
            )
            self.assertEqual(vehicle.pace_mode, "ATTACK")
            self.assertGreater(active.diagnostic_counts()["producer_generation"], 0)
            websocket = _CountingWebSocket()
            await active.add_client(websocket)
            with patch.object(
                active.replay_clock,
                "wait",
                new=AsyncMock(return_value=True),
            ):
                await active.start_loop()
                await asyncio.wait_for(active._loop_task, timeout=30.0)
            diagnostics = active.diagnostic_counts()
            self.assertIsNotNone(active.result)
            self.assertEqual(len(active.result.classification), 20)
            self.assertEqual(websocket.race_end_count, 1)
            self.assertEqual(len(websocket.event_ids), len(set(websocket.event_ids)))
            self.assertEqual(diagnostics["missing_pose_count"], 0)
            self.assertEqual(diagnostics["duplicate_pose_count"], 0)
            self.assertEqual(diagnostics["position_speed_violation_count"], 0)
            await abstract_broadcast_session_manager.clear_async()

        asyncio.run(run())

    def test_abstract_qualifying_uses_result_engine_and_returns_hash(self) -> None:
        response = run_qualifying_session(
            QualifyingRequest(
                circuit_id=3,
                player_team_id=1,
                simulation_mode=SimulationMode.ABSTRACT,
                session_seed=42,
            )
        )

        self.assertEqual(response.simulation_mode, SimulationMode.ABSTRACT)
        self.assertEqual(len(response.results), 20)
        self.assertEqual(len(response.grid_order), 20)
        self.assertEqual(len(set(response.grid_order)), 20)
        self.assertIsNotNone(response.canonical_result_hash)

    def test_abstract_race_setup_returns_summary_without_creating_full_session(self) -> None:
        worker_threads = []
        adapter = simulation_engine_factory.for_mode(SimulationMode.ABSTRACT)
        original_compute = adapter.compute_race

        def wrapped_compute(**kwargs):
            worker_threads.append(threading.get_ident())
            return original_compute(**kwargs)

        with patch.object(adapter, "compute_race", side_effect=wrapped_compute):
            response = asyncio.run(
                setup_race(
                    RaceSetupRequest(
                        circuit_id=3,
                        player_team_id=1,
                        total_laps=5,
                        simulation_mode=SimulationMode.ABSTRACT,
                        session_seed=42,
                    )
                )
            )

        self.assertEqual(response.simulation_mode, SimulationMode.ABSTRACT)
        self.assertIsNotNone(response.abstract_result_summary)
        self.assertEqual(response.abstract_result_summary["total_laps"], 5)
        self.assertEqual(len(response.abstract_result_summary["finish_order"]), 20)
        self.assertIsNone(__import__("session").session_manager.session)
        self.assertEqual(len(worker_threads), 1)
        self.assertNotEqual(worker_threads[0], threading.get_ident())

    def test_abstract_broadcast_setup_creates_separate_replay_session(self) -> None:
        async def run_setup_and_cleanup():
            started = time.perf_counter()
            response = await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=42,
                )
            )
            setup_elapsed = time.perf_counter() - started
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            self.assertLess(setup_elapsed, 3.0)
            self.assertEqual(response.simulation_mode, SimulationMode.ABSTRACT_BROADCAST)
            self.assertEqual(active.race_info["simulation_mode"], "ABSTRACT_BROADCAST")
            self.assertIsNone(active.result)
            self.assertIsNone(response.abstract_result_summary["canonical_result_hash"])
            self.assertEqual(response.abstract_result_summary["finish_order"], [])
            self.assertEqual(response.abstract_result_summary["status"], "buffered_broadcast_running")
            self.assertEqual(active.diagnostic_counts()["frame_count"], 0)
            self.assertEqual(active._current_checkpoint().checkpoint_kind, "race_start")
            self.assertGreaterEqual(
                active.diagnostic_counts()["producer_buffer_count"],
                active.diagnostic_counts()["producer_start_buffer_ticks"],
            )
            self.assertEqual(
                active.race_info["presentation_contract"],
                "stage4-full-field-traffic",
            )
            self.assertEqual(
                active.race_info["transport_contract"],
                "abstract-interactive-buffered-binary-v2",
            )
            self.assertEqual(active.race_info["kinematic_probe"]["pose_count"], 20)
            websocket = _BinaryFakeWebSocket()
            await active.add_client(websocket)
            self.assertEqual(
                [message["type"] for message in websocket.messages[:2]],
                ["race_info", "race_state"],
            )
            self.assertEqual(len(websocket.binary_messages), 1)
            self.assertEqual(len(websocket.binary_messages[0]), 491)
            magic, physics_frame, speed, paused, driver_count = struct.unpack_from(
                "<4sIBBB", websocket.binary_messages[0], 0
            )
            self.assertEqual((magic, physics_frame, speed, paused, driver_count), (b"F1P1", 0, 1, 0, 20))
            initial_states = [
                message
                for message in websocket.messages
                if message["type"] == "race_state"
            ]
            self.assertTrue(initial_states)
            self.assertLess(initial_states[0]["lap"], response.circuit.total_laps)
            self.assertNotIn("race_end", [message["type"] for message in websocket.messages])

            command = await active.handle_command({"type": "set_speed", "multiplier": 5})
            self.assertEqual(command["type"], "command_ack")
            self.assertEqual(active.speed_multiplier, 5)
            self.assertLessEqual(
                active.diagnostic_counts()["probe_frame_count"],
                active.diagnostic_counts()["probe_frame_capacity"],
            )
            failing_websocket = _FailingWebSocket()
            with self.assertRaisesRegex(RuntimeError, "client disconnected"):
                await active.add_client(failing_websocket)
            self.assertNotIn(failing_websocket, active.clients)
            await abstract_broadcast_session_manager.clear_async()
            self.assertIsNone(active._loop_task)
            self.assertIsNone(active._producer_task)
            self.assertFalse(active.replay_clock.waiting)
            self.assertIsNone(active.replay_clock._wake_event)
            return response

        response = asyncio.run(run_setup_and_cleanup())
        self.assertEqual(len(response.abstract_result_summary["grid"]), 20)
        self.assertIsNone(__import__("session").session_manager.session)

    def test_abstract_broadcast_57_lap_setup_does_not_precompute_result(self) -> None:
        async def run() -> None:
            started = time.perf_counter()
            adapter = simulation_engine_factory.for_mode(
                SimulationMode.ABSTRACT_BROADCAST
            )
            with patch.object(
                adapter,
                "compute_race",
                side_effect=AssertionError("broadcast must not precompute the race"),
            ):
                response = await setup_race(
                    RaceSetupRequest(
                        circuit_id=3,
                        player_team_id=1,
                        total_laps=57,
                        simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                        session_seed=42,
                    )
                )
            elapsed = time.perf_counter() - started
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            self.assertLess(elapsed, 3.0)
            self.assertIsNone(active.result)
            self.assertEqual(response.abstract_result_summary["finish_order"], [])
            self.assertGreaterEqual(
                active.diagnostic_counts()["producer_buffer_count"],
                active.diagnostic_counts()["producer_start_buffer_ticks"],
            )
            await abstract_broadcast_session_manager.clear_async()

        asyncio.run(run())

    def test_abstract_broadcast_player_strategy_rewinds_unpublished_buffer(self) -> None:
        async def run() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=142,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            player_driver_id = active.player_drivers[0].id
            other_driver_id = next(
                driver.id
                for driver in active.drivers
                if driver.id not in {item.id for item in active.player_drivers}
            )
            before = active.diagnostic_counts()
            self.assertGreaterEqual(before["producer_buffer_count"], 20)

            pace = await active.handle_command({
                "type": "set_pace_mode",
                "driver_id": player_driver_id,
                "pace_mode": "ATTACK",
            })
            self.assertEqual(pace["type"], "command_ack")
            displayed = next(
                vehicle
                for vehicle in active._current_frame().vehicles
                if vehicle.driver_id == player_driver_id
            )
            self.assertEqual(displayed.pace_mode, "ATTACK")
            self.assertGreater(active.diagnostic_counts()["producer_generation"], 0)
            self.assertGreater(active.diagnostic_counts()["invalidated_tick_count"], 0)

            pit = await active.handle_command({
                "type": "pit_call",
                "driver_id": player_driver_id,
                "tire_choice": "HARD",
            })
            self.assertEqual(pit["type"], "command_ack")
            displayed = next(
                vehicle
                for vehicle in active._current_frame().vehicles
                if vehicle.driver_id == player_driver_id
            )
            self.assertTrue(displayed.pit_request_pending)
            self.assertEqual(displayed.pit_request_role, "HARD")

            cancel = await active.handle_command({
                "type": "pit_cancel",
                "driver_id": player_driver_id,
            })
            self.assertEqual(cancel["type"], "command_ack")
            displayed = next(
                vehicle
                for vehicle in active._current_frame().vehicles
                if vehicle.driver_id == player_driver_id
            )
            self.assertFalse(displayed.pit_request_pending)
            self.assertEqual(
                [record.command for record in active.traffic_cursor.command_log],
                ["set_pace_mode", "pit_call", "pit_cancel"],
            )

            rejected = await active.handle_command({
                "type": "set_pace_mode",
                "driver_id": other_driver_id,
                "pace_mode": "ATTACK",
            })
            self.assertEqual(rejected["type"], "command_error")
            await abstract_broadcast_session_manager.clear_async()

        asyncio.run(run())

    def test_abstract_strategy_command_preserves_live_pose_sequence(self) -> None:
        async def run() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=143,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            websocket = _FakeWebSocket()
            await active.add_client(websocket)
            await active.handle_command({"type": "set_speed", "multiplier": 5})
            await asyncio.sleep(0.25)
            await active.handle_command({"type": "pause"})
            command_tick = active.diagnostic_counts()["probe_cursor_tick"]
            self.assertGreater(command_tick, 0)

            response = await active.handle_command({
                "type": "set_pace_mode",
                "driver_id": active.player_drivers[0].id,
                "pace_mode": "CONSERVE",
            })
            self.assertEqual(response["effective_tick"], command_tick + 1)
            self.assertEqual(active.diagnostic_counts()["probe_cursor_tick"], command_tick)
            await active.handle_command({"type": "resume"})
            await asyncio.sleep(0.20)
            await active.handle_command({"type": "pause"})

            diagnostics = active.diagnostic_counts()
            self.assertEqual(diagnostics["missing_pose_count"], 0)
            self.assertEqual(diagnostics["duplicate_pose_count"], 0)
            self.assertLessEqual(diagnostics["max_frame_gap"], 1)
            event_ids = [
                event["event_id"]
                for message in websocket.messages
                if message["type"] == "race_events"
                for event in message["events"]
            ]
            self.assertEqual(
                len([event_id for event_id in event_ids if ":set_pace_mode:" in event_id]),
                1,
            )
            await abstract_broadcast_session_manager.clear_async()

        asyncio.run(run())

    def test_abstract_broadcast_single_probe_pause_speed_and_close(self) -> None:
        async def run_probe() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=43,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            websocket = _FakeWebSocket()
            await active.add_client(websocket)
            await active.start_loop()
            await asyncio.sleep(0.02)

            await active.handle_command({"type": "pause"})
            paused_checkpoint = active.diagnostic_counts()["checkpoint_index"]
            paused_probe_count = active.diagnostic_counts()["probe_frame_count"]
            paused_probe_tick = active.diagnostic_counts()["probe_cursor_tick"]
            await asyncio.sleep(0.20)
            self.assertEqual(active.diagnostic_counts()["checkpoint_index"], paused_checkpoint)
            self.assertEqual(active.diagnostic_counts()["probe_frame_count"], paused_probe_count)
            self.assertEqual(active.diagnostic_counts()["probe_cursor_tick"], paused_probe_tick)

            reconnect = _FakeWebSocket()
            before_reconnect = active.diagnostic_counts()
            await active.add_client(reconnect)
            after_reconnect = active.diagnostic_counts()
            self.assertEqual(after_reconnect["probe_cursor_tick"], before_reconnect["probe_cursor_tick"])
            self.assertEqual(after_reconnect["probe_frame_count"], before_reconnect["probe_frame_count"])
            active.remove_client(reconnect)

            await active.handle_command({"type": "resume"})
            await active.handle_command({"type": "set_speed", "multiplier": 5})
            await asyncio.sleep(0.02)
            self.assertLessEqual(
                active.diagnostic_counts()["probe_frame_count"],
                active.diagnostic_counts()["probe_frame_capacity"],
            )
            pose_messages = [message for message in websocket.messages if message["type"] == "pose_tick"]
            self.assertTrue(pose_messages)
            self.assertTrue(all(message["pose_count"] == 20 for message in pose_messages))
            self.assertTrue(all(len(message["poses"]) == 20 for message in pose_messages))
            await active.close()
            self.assertIsNone(active._loop_task)
            self.assertFalse(active.replay_clock.waiting)
            self.assertIsNone(active.replay_clock._wake_event)
            self.assertEqual(len(active.probe_buffer), 0)
            self.assertIsNone(active.probe_cursor)
            self.assertFalse(active.diagnostic_counts()["activity_event_exists"])
            self.assertEqual(active.diagnostic_counts()["retained_pose_count_after_close"], 0)
            self.assertTrue(websocket.closed)

        asyncio.run(run_probe())

    def test_abstract_broadcast_controlled_clock_hash_is_speed_invariant(self) -> None:
        async def run_speeds() -> list[dict[str, Any]]:
            outputs = []
            for multiplier in (1, 2, 5):
                await setup_race(
                    RaceSetupRequest(
                        circuit_id=3,
                        player_team_id=1,
                        total_laps=5,
                        simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                        session_seed=45,
                    )
                )
                active = abstract_broadcast_session_manager.session
                self.assertIsNotNone(active)
                websocket = _FakeWebSocket()
                await active.add_client(websocket)
                before_command = active.diagnostic_counts()
                command = await active.handle_command(
                    {"type": "set_speed", "multiplier": multiplier}
                )
                self.assertEqual(command["type"], "command_ack")
                self.assertEqual(active.diagnostic_counts()["probe_cursor_tick"], before_command["probe_cursor_tick"])
                self.assertEqual(active.diagnostic_counts()["probe_frame_count"], before_command["probe_frame_count"])
                websocket.messages.clear()
                with patch.object(active.replay_clock, "wait", new=AsyncMock(return_value=True)):
                    await active.start_loop()
                    await active._loop_task
                counts = active.diagnostic_counts()
                pose_messages = [message for message in websocket.messages if message["type"] == "pose_tick"]
                self.assertEqual(counts["missing_pose_count"], 0)
                self.assertEqual(counts["duplicate_pose_count"], 0)
                self.assertEqual(counts["max_frame_gap"], 1)
                self.assertAlmostEqual(counts["max_logical_time_gap"], 0.10, delta=1e-9)
                self.assertEqual(counts["position_speed_violation_count"], 0)
                self.assertTrue(pose_messages)
                outputs.append(
                    {
                        "result_hash": active.result.canonical_result_hash,
                        "pose_hash": counts["broadcast_pose_hash"],
                        "pose_count": counts["emitted_pose_count"],
                        "payload_count": len(pose_messages),
                        "race_end_count": sum(
                            message["type"] == "race_end" for message in websocket.messages
                        ),
                    }
                )
                await abstract_broadcast_session_manager.clear_async()
            return outputs

        outputs = asyncio.run(run_speeds())
        self.assertEqual(len({item["result_hash"] for item in outputs}), 1)
        self.assertEqual(len({item["pose_hash"] for item in outputs}), 1)
        self.assertEqual(len({item["pose_count"] for item in outputs}), 1)
        self.assertEqual(len({item["payload_count"] for item in outputs}), 1)
        self.assertTrue(all(item["race_end_count"] == 1 for item in outputs))

    def test_abstract_broadcast_57_lap_controlled_clock_keeps_single_probe_bounded(self) -> None:
        async def run_replay() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=57,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=44,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            websocket = _CountingWebSocket()
            await active.add_client(websocket)
            with patch.object(active.replay_clock, "wait", new=AsyncMock(return_value=True)):
                await active.start_loop()
                await active._loop_task

            self.assertIsNotNone(active.result)
            final_tick = active.result.timing_checkpoints[-1].tick_index
            expected_event_ids = [event.event_id for event in active.result.logical_events]
            counts = active.diagnostic_counts()
            self.assertEqual(websocket.pose_payload_count, (final_tick + 1) * 20)
            self.assertEqual(counts["emitted_pose_count"], (final_tick + 1) * 20)
            self.assertEqual(counts["missing_pose_count"], 0)
            self.assertEqual(counts["duplicate_pose_count"], 0)
            self.assertEqual(counts["max_frame_gap"], 1)
            self.assertAlmostEqual(counts["max_logical_time_gap"], 0.10, delta=1e-9)
            self.assertEqual(counts["position_speed_violation_count"], 0)
            self.assertLessEqual(counts["max_position_derived_speed_mps"], 370.0 / 3.6 + 1e-9)
            self.assertEqual(websocket.event_ids, expected_event_ids)
            self.assertEqual(len(websocket.event_ids), len(set(websocket.event_ids)))
            self.assertEqual(websocket.race_end_count, 1)
            self.assertEqual(websocket.message_types[-1], "race_end")
            self.assertLessEqual(counts["probe_frame_count"], 128)
            self.assertEqual(counts["probe_buffer_peak"], 128)

            await abstract_broadcast_session_manager.clear_async()
            self.assertTrue(websocket.closed)
            self.assertIsNone(active.probe_cursor)
            self.assertIsNone(active._activity_event)
            self.assertEqual(len(active.probe_buffer), 0)
            self.assertIsNone(active._loop_task)

        asyncio.run(run_replay())

    def test_abstract_broadcast_mid_replay_speed_changes_keep_pose_sequence_contiguous(self) -> None:
        async def run_replay() -> dict[str, Any]:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=46,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            websocket = _FakeWebSocket()
            await active.add_client(websocket)
            wait_calls = 0

            async def controlled_wait(_logical_duration_s: float) -> bool:
                nonlocal wait_calls
                wait_calls += 1
                if wait_calls == 2:
                    await active.handle_command({"type": "set_speed", "multiplier": 5})
                elif wait_calls == 5:
                    await active.handle_command({"type": "set_speed", "multiplier": 2})
                elif wait_calls == 8:
                    await active.handle_command({"type": "set_speed", "multiplier": 1})
                return True

            websocket.messages.clear()
            with patch.object(active.replay_clock, "wait", new=controlled_wait):
                await active.start_loop()
                await active._loop_task
            counts = active.diagnostic_counts()
            await abstract_broadcast_session_manager.clear_async()
            return counts

        counts = asyncio.run(run_replay())
        self.assertEqual(counts["missing_pose_count"], 0)
        self.assertEqual(counts["duplicate_pose_count"], 0)
        self.assertEqual(counts["max_frame_gap"], 1)
        self.assertAlmostEqual(counts["max_logical_time_gap"], 0.10, delta=1e-9)

    def test_abstract_broadcast_replays_probe_checkpoints_events_then_race_end(self) -> None:
        async def run_replay() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=44,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            websocket = _FakeWebSocket()
            await active.add_client(websocket)
            websocket.messages.clear()
            with patch.object(active.replay_clock, "wait", new=AsyncMock(return_value=True)):
                await active.start_loop()
                await active._loop_task

            message_types = [message["type"] for message in websocket.messages]
            self.assertEqual(message_types[-1], "race_end")
            self.assertNotIn("race_end", message_types[:-1])
            pose_messages = [message for message in websocket.messages if message["type"] == "pose_tick"]
            self.assertTrue(pose_messages)
            self.assertTrue(all(message["pose_count"] == 20 for message in pose_messages))
            self.assertTrue(all(len(message["poses"]) == 20 for message in pose_messages))
            physics_frames = [message["physics_frame"] for message in pose_messages]
            expected_frame_count = active.result.timing_checkpoints[-1].tick_index + 1
            self.assertEqual(physics_frames, list(range(1, expected_frame_count)))
            logical_times = [message["simulation_time_s"] for message in pose_messages]
            self.assertTrue(
                all(
                    abs(current - previous - 0.10) <= 1e-9
                    for previous, current in zip(logical_times, logical_times[1:])
                )
            )
            event_ids = [
                event["event_id"]
                for message in websocket.messages
                if message["type"] == "race_events"
                for event in message["events"]
            ]
            self.assertEqual(event_ids, [event.event_id for event in active.result.logical_events])
            self.assertEqual(len(event_ids), len(set(event_ids)))
            self.assertEqual(
                active.diagnostic_counts()["event_index"],
                len(active.result.logical_events),
            )
            self.assertEqual(active._current_checkpoint().checkpoint_kind, "race_finish")
            self.assertGreater(active.diagnostic_counts()["probe_frame_count"], 0)
            self.assertLessEqual(
                active.diagnostic_counts()["probe_frame_count"],
                active.diagnostic_counts()["probe_frame_capacity"],
            )
            await active.close()

        asyncio.run(run_replay())

    def test_abstract_broadcast_race_started_is_sent_at_initial_pose_timeline(self) -> None:
        async def run_replay() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=44,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            websocket = _FakeWebSocket()
            await active.add_client(websocket)
            with patch.object(active.replay_clock, "wait", new=AsyncMock(return_value=True)):
                await active.start_loop()
                await active._loop_task

            race_started = [
                (index, event)
                for index, message in enumerate(websocket.messages)
                if message["type"] == "race_events"
                for event in message["events"]
                if event["type"] == "race_started"
            ]
            self.assertEqual(len(race_started), 1)
            race_started_index, race_started_event = race_started[0]
            self.assertEqual(race_started_event["payload"]["logical_time_s"], 0.0)
            poses_before_event = [
                message
                for message in websocket.messages[:race_started_index]
                if message["type"] == "pose_tick"
            ]
            self.assertTrue(poses_before_event)
            self.assertEqual(poses_before_event[-1]["physics_frame"], 0)
            self.assertEqual(poses_before_event[-1]["simulation_time_s"], 0.0)
            first_frame_one_index = next(
                index
                for index, message in enumerate(websocket.messages)
                if message["type"] == "pose_tick" and message["physics_frame"] == 1
            )
            self.assertLess(race_started_index, first_frame_one_index)

            race_finished = [
                (index, event)
                for index, message in enumerate(websocket.messages)
                if message["type"] == "race_events"
                for event in message["events"]
                if event["type"] == "race_finished"
            ]
            self.assertEqual(len(race_finished), 1)
            race_finished_index, race_finished_event = race_finished[0]
            final_checkpoint = active.result.timing_checkpoints[-1]
            self.assertEqual(
                race_finished_event["payload"]["logical_time_s"],
                final_checkpoint.logical_time_s,
            )
            poses_before_finish = [
                message
                for message in websocket.messages[:race_finished_index]
                if message["type"] == "pose_tick"
            ]
            self.assertEqual(poses_before_finish[-1]["physics_frame"], final_checkpoint.tick_index)
            self.assertEqual(
                poses_before_finish[-1]["simulation_time_s"],
                final_checkpoint.logical_time_s,
            )
            self.assertEqual(websocket.messages[-1]["type"], "race_end")

            before_reconnect = active.diagnostic_counts()
            reconnect = _FakeWebSocket()
            await active.add_client(reconnect)
            after_reconnect = active.diagnostic_counts()
            self.assertEqual(after_reconnect["probe_cursor_tick"], before_reconnect["probe_cursor_tick"])
            self.assertEqual(after_reconnect["probe_frame_count"], before_reconnect["probe_frame_count"])
            active.remove_client(reconnect)
            all_race_started = [
                event
                for client_messages in (websocket.messages, reconnect.messages)
                for message in client_messages
                if message["type"] == "race_events"
                for event in message["events"]
                if event["type"] == "race_started"
            ]
            self.assertEqual(len(all_race_started), 1)
            await active.close()

        asyncio.run(run_replay())

    def test_abstract_broadcast_failed_initial_snapshot_preserves_start_event(self) -> None:
        async def run_replay() -> None:
            await setup_race(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    simulation_mode=SimulationMode.ABSTRACT_BROADCAST,
                    session_seed=47,
                )
            )
            active = abstract_broadcast_session_manager.session
            self.assertIsNotNone(active)
            failing_websocket = _FailingWebSocket()
            with self.assertRaisesRegex(RuntimeError, "client disconnected"):
                await active.add_client(failing_websocket)
            self.assertEqual(active.diagnostic_counts()["event_index"], 0)

            websocket = _FakeWebSocket()
            await active.add_client(websocket)
            with patch.object(active.replay_clock, "wait", new=AsyncMock(return_value=True)):
                await active.start_loop()
                await active._loop_task
            race_started = [
                event
                for message in websocket.messages
                if message["type"] == "race_events"
                for event in message["events"]
                if event["type"] == "race_started"
            ]
            self.assertEqual(len(race_started), 1)
            self.assertEqual(race_started[0]["payload"]["logical_time_s"], 0.0)
            await active.close()

        asyncio.run(run_replay())

if __name__ == "__main__":
    unittest.main()
