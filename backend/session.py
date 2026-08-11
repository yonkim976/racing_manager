"""In-memory race session management."""

from __future__ import annotations

import asyncio
import gc
import os
import struct
import uuid
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from fastapi import WebSocket
from engines.full import FullEngineAdapter, empty_tire_temperature_diagnostic_snapshot

from models.schemas import (
    Circuit,
    Driver,
    PaceMode,
    RaceEvent,
    RaceEventsMessage,
    RaceInfoMessage,
    RaceSetupRequest,
    Team,
)
from engines.full.runtime.pit_stop import parse_tire_choice
from simulation.track_physics import clear_vehicle_track_physics_cache, track_physics_cache_counts
from simulation.track_display import build_track_display_geometry
from engines.full.runtime.vehicle_physics import PHYSICS_STEP_SECONDS

if TYPE_CHECKING:
    from engines.full.runtime.race_engine import RaceEngine

POSE_BROADCAST_HZ = 30
POSE_BROADCAST_INTERVAL = 1.0 / POSE_BROADCAST_HZ
DASHBOARD_BROADCAST_HZ = 4
DASHBOARD_BROADCAST_INTERVAL = 1.0 / DASHBOARD_BROADCAST_HZ
TIMING_BROADCAST_HZ = 1
TIMING_BROADCAST_INTERVAL = 1.0 / TIMING_BROADCAST_HZ
PAUSED_BROADCAST_HZ = 1
PAUSED_BROADCAST_INTERVAL = 1.0 / PAUSED_BROADCAST_HZ
# Compatibility names used by cadence metrics and existing integrations.
BROADCAST_HZ = POSE_BROADCAST_HZ
BROADCAST_INTERVAL = POSE_BROADCAST_INTERVAL
MAX_PHYSICS_STEPS_PER_SLICE = 5
MAX_SIMULATION_BACKLOG_SECONDS = 0.5
MAX_TRAJECTORY_SAMPLES_PER_DRIVER = 64
MAX_PUBLIC_EVENTS_PER_PACKET = 8
INTERNAL_EVENT_TYPES = {
    "maneuver_group_corner_yield",
    "maneuver_group_dissolved",
    "maneuver_group_formed",
}
EVENT_FEED_COOLDOWN_SECONDS = {
    "attack": 10.0,
    "defend": 12.0,
    "side_by_side": 6.0,
    "overtake_abort": 12.0,
    "minor_contact": 5.0,
    "lockup": 8.0,
    "traction_loss": 8.0,
    "wheelspin": 8.0,
    "run_wide": 8.0,
    "forced_wide": 8.0,
}
ROUTINE_BATTLE_EVENT_TYPES = {"attack", "defend", "overtake_abort"}
ROUTINE_BATTLE_GLOBAL_COOLDOWN_SECONDS = 1.5
POSE_PACKET_MAGIC = b"F1P1"
POSE_PACKET_HEADER = struct.Struct("<4sIBBB")
POSE_DRIVER_HEADER = struct.Struct("<HBB")
POSE_SAMPLE = struct.Struct("<fIfff")
DashboardInclude = dict[str, Any]
DASHBOARD_STATE_FIELDS = {
    "lap",
    "total_laps",
    "race_phase",
    "race_phase_remaining_seconds",
    "race_phase_remaining_laps",
    "safety_car_stage",
    "safety_car_visible",
    "safety_car_route",
    "safety_car_progress",
    "safety_car_progress_rate",
    "safety_car_pit_lane_progress",
    "pit_window_open",
    "start_sequence_phase",
    "start_light_count",
    "speed_multiplier",
    "paused",
    "physics_hz",
    "broadcast_hz",
    "effective_speed_multiplier",
    "simulation_backlog_seconds",
    "broadcast_jitter_ms",
    "track_conditions",
    "thermal_preset",
    "track_conditions_source",
}
DASHBOARD_POSITION_FIELDS = {
    "driver_id",
    "name",
    "full_name",
    "team",
    "team_color",
    "position",
    "speed_kph",
    "gap",
    "interval",
    "timing_gap_valid",
    "interval_timing_gap_valid",
    "timing_gap_source",
    "interval_timing_gap_source",
    "current_sector",
    "current_mini_sector",
    "current_timing_loop",
    "tire_compound",
    "tire_role",
    "physical_tire_compound",
    "tire_age",
    "tire_wear",
    "tire_lateral_grip",
    "tire_traction_grip",
    "tire_braking_grip",
    "tire_surface_temperature_c",
    "tire_core_temperature_c",
    "tire_thermal_grip",
    "front_tire_surface_temperature_c",
    "front_tire_core_temperature_c",
    "front_tire_thermal_grip",
    "rear_tire_surface_temperature_c",
    "rear_tire_core_temperature_c",
    "rear_tire_thermal_grip",
    "fuel_mass_kg",
    "fuel_burned_kg",
    "fuel_laps_remaining",
    "vehicle_mass_kg",
    "front_normal_load_n",
    "rear_normal_load_n",
    "longitudinal_load_transfer_n",
    "front_wheel_speed_rad_s",
    "rear_wheel_speed_rad_s",
    "front_axle_slip_ratio",
    "rear_axle_slip_ratio",
    "applied_brake_force_n",
    "front_brake_temperature_c",
    "rear_brake_temperature_c",
    "brake_fade_factor",
    "grip_utilization",
    "handling_state",
    "wheel_lock_ratio",
    "traction_slip_ratio",
    "front_tire_slide_energy_j",
    "rear_tire_slide_energy_j",
    "tire_slide_energy_j",
    "rear_applied_drive_energy_j",
    "front_applied_brake_energy_j",
    "rear_applied_brake_energy_j",
    "pace_mode",
    "pace_mode_from",
    "pace_mode_transition_progress",
    "last_lap_time",
    "best_lap_time",
    "in_pit",
    "pit_count",
    "pit_phase",
    "pit_merge_state",
    "pit_box_progress",
    "pit_elapsed",
    "pit_stop_elapsed",
    "retired",
    "finished",
    "drs_active",
    "dirty_air_active",
    "side_by_side_active",
    "maneuver_group_size",
    "maneuver_group_id",
    "maneuver_group_member_ids",
    "maneuver_group_phase",
    "maneuver_group_corridor_index",
    "drs_train_id",
    "drs_train_size",
    "drs_train_position",
    "drs_train_member_ids",
    "hazard_active",
    "local_yellow_active",
}
TIMING_POSITION_FIELDS = {
    "last_mini_sector_time",
    "last_mini_sector_delta_to_best",
    "mini_sector_splits",
    "mini_sector_statuses",
}
DASHBOARD_PAYLOAD_INCLUDE: DashboardInclude = {
    **{field: True for field in DASHBOARD_STATE_FIELDS},
    "positions": {"__all__": DASHBOARD_POSITION_FIELDS},
}
DEV_RACE_CONTROLS_ENABLED = os.getenv("F1_ENABLE_DEV_CONTROLS", "1").lower() in {
    "1",
    "true",
    "yes",
}


@dataclass
class SessionCadenceMetrics:
    """Measure delivered simulation speed without changing authoritative time."""

    requested_speed_multiplier: int = 1
    window_started_at: float | None = None
    simulated_seconds: float = 0.0
    physics_steps_since_broadcast: int = 0

    def reset(self, now: float, requested_speed_multiplier: int) -> None:
        self.requested_speed_multiplier = requested_speed_multiplier
        self.window_started_at = now
        self.simulated_seconds = 0.0
        self.physics_steps_since_broadcast = 0

    def record_physics_step(self, delta_seconds: float) -> None:
        self.simulated_seconds += delta_seconds
        self.physics_steps_since_broadcast += 1

    def snapshot(
        self,
        *,
        now: float,
        scheduled_broadcast_at: float,
        simulation_backlog_seconds: float,
        requested_speed_multiplier: int,
        paused: bool,
    ) -> dict[str, float | int]:
        if (
            self.window_started_at is None
            or requested_speed_multiplier != self.requested_speed_multiplier
            or paused
        ):
            self.reset(now, requested_speed_multiplier)

        window_started_at = (
            self.window_started_at if self.window_started_at is not None else now
        )
        wall_seconds = max(0.0, now - window_started_at)
        effective_speed = (
            self.simulated_seconds / wall_seconds
            if wall_seconds > 1e-9 and not paused
            else 0.0
        )
        snapshot = {
            "broadcast_hz": BROADCAST_HZ,
            "effective_speed_multiplier": effective_speed,
            "simulation_backlog_seconds": max(0.0, simulation_backlog_seconds),
            "broadcast_jitter_ms": max(
                0.0,
                (now - scheduled_broadcast_at) * 1000.0,
            ),
            "physics_steps_last_broadcast": self.physics_steps_since_broadcast,
        }
        self.physics_steps_since_broadcast = 0
        return snapshot


class RaceSession:
    """Single active race session with WebSocket subscribers."""

    def __init__(
        self,
        session_id: str,
        engine: RaceEngine,
        circuit: Circuit,
        player_team: Team,
        player_drivers: list[Driver],
    ):
        self.session_id = session_id
        self.engine = engine
        self.circuit = circuit
        self.player_team = player_team
        self.player_drivers = player_drivers
        self.resolved_track_conditions = getattr(
            engine,
            "track_conditions",
            None,
        )
        self.resolved_thermal_preset = getattr(engine, "thermal_preset", None)
        self.track_conditions_source = getattr(
            engine,
            "track_conditions_source",
            "explicit_override",
        )
        self.display_geometry = None
        if self.circuit is not None:
            track_profile = getattr(engine, "track_physics_profile", None)
            grid_order = (
                engine.get_grid_order()
                if hasattr(engine, "get_grid_order")
                else ()
            )
            self.display_geometry = build_track_display_geometry(
                self.circuit,
                track_profile=track_profile,
                grid_driver_ids=(item["driver_id"] for item in grid_order),
                start_sequence_enabled=getattr(
                    engine,
                    "start_sequence_enabled",
                    True,
                ),
            )
        self.clients: set[WebSocket] = set()
        self._loop_task: asyncio.Task | None = None
        self._trajectory_samples: dict[int, list[bytes]] = {}
        self._cadence_metrics = SessionCadenceMetrics()
        self._latest_runtime_metrics: dict[str, float | int] = {}
        self._event_sequence = 0
        self._event_feed_last_sent_at: dict[tuple[str, str], float] = {}
        self._routine_battle_event_last_sent_at = float("-inf")
        self._last_history_lengths: dict[int, int] = {}

    def diagnostic_counts(self) -> dict[str, Any]:
        """Return bounded session/engine ownership counts for desktop logs."""
        task = self._loop_task
        return {
            "session_id": self.session_id,
            "engine_id": f"engine-{id(self.engine):x}",
            "loop_task_exists": task is not None,
            "loop_task_done": bool(task and task.done()),
            "loop_task_cancelled": bool(task and task.cancelled()),
            "client_count": len(self.clients),
            "trajectory_driver_count": len(self._trajectory_samples),
            "trajectory_sample_count": sum(
                len(samples) for samples in self._trajectory_samples.values()
            ),
            "event_feed_auxiliary_entries": len(self._event_feed_last_sent_at),
            "last_history_length_entries": len(self._last_history_lengths),
            "safety_car": self.engine.safety_car_diagnostic_snapshot(),
            **self.engine.diagnostic_counts(),
        }

    @property
    def race_info(self) -> RaceInfoMessage:
        geometry = self.display_geometry.to_race_info_geometry()
        return RaceInfoMessage(
            circuit_name=self.circuit.name,
            total_laps=self.circuit.total_laps,
            player_team=self.player_team.name,
            player_team_color=self.player_team.color,
            player_drivers=[d.id for d in self.player_drivers],
            **geometry,
            surface_zones=self.engine._track_surface.zones,
            track_conditions=self.circuit.track_conditions,
            environment_conditions=self.engine.track_conditions,
            thermal_preset=self.resolved_thermal_preset,
            track_conditions_source=self.track_conditions_source,
            tire_compound_nomination=self.circuit.tire_compound_nomination,
            drs_zones=self.circuit.drs_zones,
            sectors=self.circuit.sectors,
            landmarks=self.circuit.landmarks,
            segments=self.circuit.segments,
        )

    async def start_loop(self) -> None:
        if self._loop_task is None or self._loop_task.done():
            self._loop_task = asyncio.create_task(self._game_loop())

    def stop_loop(self) -> None:
        self.engine.finished = True
        if self._loop_task is not None and not self._loop_task.done():
            self._loop_task.cancel()
        self._loop_task = None
        clients = tuple(self.clients)
        self.clients.clear()
        self._trajectory_samples.clear()
        self._event_feed_last_sent_at.clear()
        self._routine_battle_event_last_sent_at = float("-inf")
        self._last_history_lengths.clear()
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None
        if loop is not None:
            for ws in clients:
                loop.create_task(ws.close(code=1001, reason="Race session closed"))

    async def close(self) -> None:
        """Stop the session and release tasks, sockets and per-race buffers."""
        self.engine.finished = True
        task = self._loop_task
        self._loop_task = None
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        clients = tuple(self.clients)
        self.clients.clear()
        if clients:
            await asyncio.gather(
                *(
                    ws.close(code=1001, reason="Race session closed")
                    for ws in clients
                ),
                return_exceptions=True,
            )
        self._trajectory_samples.clear()
        self._event_feed_last_sent_at.clear()
        self._routine_battle_event_last_sent_at = float("-inf")
        self._last_history_lengths.clear()

    async def _game_loop(self) -> None:
        """Advance authoritative physics independently from screen broadcasts.

        Physics always consumes exact 20 ms simulation steps.  Wall-clock time is
        accumulated as simulation credit, while snapshots are emitted on their own
        30 Hz deadline.  This avoids adding physics compute time to every sleep and
        preserves all intermediate poses for the renderer.
        """
        loop = asyncio.get_running_loop()
        last_wall_time = loop.time()
        next_pose_broadcast_time = last_wall_time
        next_dashboard_broadcast_time = last_wall_time
        next_timing_broadcast_time = last_wall_time
        simulation_credit = 0.0
        pending_events: list[RaceEvent] = []
        self._cadence_metrics.reset(last_wall_time, 1)

        while not self.engine.finished:
            now = loop.time()
            wall_delta = max(0.0, min(now - last_wall_time, 0.25))
            last_wall_time = now

            if not self.clients:
                simulation_credit = 0.0
                pending_events.clear()
                self._trajectory_samples.clear()
                self._cadence_metrics.reset(now, 1)
                next_pose_broadcast_time = now + POSE_BROADCAST_INTERVAL
                next_dashboard_broadcast_time = (
                    now + DASHBOARD_BROADCAST_INTERVAL
                )
                next_timing_broadcast_time = now + TIMING_BROADCAST_INTERVAL
                await asyncio.sleep(POSE_BROADCAST_INTERVAL)
                continue

            if self.engine.paused:
                simulation_credit = 0.0
            else:
                simulation_credit = min(
                    MAX_SIMULATION_BACKLOG_SECONDS,
                    simulation_credit
                    + wall_delta
                    * (self.engine.speed_multiplier if self.engine.race_started else 1),
                )

            steps_run = 0
            while (
                simulation_credit + 1e-12 >= PHYSICS_STEP_SECONDS
                and steps_run < MAX_PHYSICS_STEPS_PER_SLICE
                and not self.engine.finished
                and not self.engine.paused
            ):
                pending_events.extend(self.engine.tick(PHYSICS_STEP_SECONDS))
                simulation_credit -= PHYSICS_STEP_SECONDS
                steps_run += 1
                self._cadence_metrics.record_physics_step(PHYSICS_STEP_SECONDS)
                self._record_trajectory_samples()

            now = loop.time()
            if now >= next_pose_broadcast_time or self.engine.finished:
                requested_speed = (
                    self.engine.speed_multiplier if self.engine.race_started else 1
                )
                runtime_metrics = self._cadence_metrics.snapshot(
                    now=now,
                    scheduled_broadcast_at=next_pose_broadcast_time,
                    simulation_backlog_seconds=simulation_credit,
                    requested_speed_multiplier=requested_speed,
                    paused=self.engine.paused,
                )
                self._latest_runtime_metrics = runtime_metrics
                await self._broadcast_bytes(self._pose_payload())
                if pending_events:
                    await self._broadcast_public_events(
                        pending_events,
                        now=now,
                    )
                pending_events = []
                self._trajectory_samples = {}
                pose_interval = (
                    PAUSED_BROADCAST_INTERVAL
                    if self.engine.paused
                    else POSE_BROADCAST_INTERVAL
                )
                next_pose_broadcast_time = now + pose_interval

            if now >= next_dashboard_broadcast_time or self.engine.finished:
                dashboard_payload = self.engine.build_dashboard_payload(
                    runtime_metrics=self._latest_runtime_metrics,
                )
                await self._broadcast(dashboard_payload)
                await self._broadcast_history_if_changed()
                if now >= next_timing_broadcast_time or self.engine.finished:
                    await self._broadcast(self.engine.build_timing_payload())
                    next_timing_broadcast_time = (
                        now + TIMING_BROADCAST_INTERVAL
                    )
                dashboard_interval = (
                    PAUSED_BROADCAST_INTERVAL
                    if self.engine.paused
                    else DASHBOARD_BROADCAST_INTERVAL
                )
                next_dashboard_broadcast_time = now + dashboard_interval

            if steps_run >= MAX_PHYSICS_STEPS_PER_SLICE:
                await asyncio.sleep(0)
                continue

            time_until_broadcast = max(
                0.0,
                min(
                    next_pose_broadcast_time,
                    next_dashboard_broadcast_time,
                )
                - loop.time(),
            )
            if self.engine.paused:
                sleep_seconds = min(
                    PAUSED_BROADCAST_INTERVAL,
                    time_until_broadcast,
                )
            else:
                speed = self.engine.speed_multiplier if self.engine.race_started else 1
                wall_until_step = max(
                    0.0,
                    (PHYSICS_STEP_SECONDS - simulation_credit) / max(1, speed),
                )
                sleep_seconds = min(time_until_broadcast, wall_until_step, 0.01)
            await asyncio.sleep(max(0.0, sleep_seconds))

        results = self.engine.build_results()
        await self._broadcast({"type": "race_end", "results": results})

    def _advance_engine(self, game_seconds: float) -> list:
        """Advance game time in exact authoritative physics steps (test helper)."""
        events: list = []
        steps = int((game_seconds + 1e-12) / PHYSICS_STEP_SECONDS)
        for _ in range(steps):
            if self.engine.finished:
                break
            events.extend(self.engine.tick(PHYSICS_STEP_SECONDS))
            self._record_trajectory_samples()
        return events

    def _record_trajectory_samples(self) -> None:
        """Retain authoritative poses that would otherwise fall between packets."""
        for state in self.engine.driver_states.values():
            samples = self._trajectory_samples.setdefault(state.driver_id, [])
            samples.append(
                POSE_SAMPLE.pack(
                    state.simulation_time_s,
                    state.physics_frame,
                    state.world_x_m,
                    state.world_y_m,
                    state.heading_rad,
                )
            )
            if len(samples) > MAX_TRAJECTORY_SAMPLES_PER_DRIVER:
                del samples[:-MAX_TRAJECTORY_SAMPLES_PER_DRIVER]

    async def _send_to_clients(
        self,
        sender: Any,
    ) -> None:
        dead: set[WebSocket] = set()
        for ws in list(self.clients):
            try:
                await asyncio.wait_for(sender(ws), timeout=1.0)
            except Exception:
                dead.add(ws)
        self.clients -= dead

    async def _broadcast(self, message: dict[str, Any]) -> None:
        await self._send_to_clients(lambda ws: ws.send_json(message))

    async def _broadcast_bytes(self, payload: bytes) -> None:
        await self._send_to_clients(lambda ws: ws.send_bytes(payload))

    def _pose_payload(self) -> bytes:
        """Encode authoritative poses directly, without transient Pydantic models."""
        states = tuple(self.engine.driver_states.values())
        payload = bytearray(
            POSE_PACKET_HEADER.pack(
                POSE_PACKET_MAGIC,
                self.engine._physics_frame,
                self.engine.speed_multiplier,
                1 if self.engine.paused else 0,
                len(states),
            )
        )
        for state in states:
            samples = self._trajectory_samples.get(state.driver_id, ())
            if not samples:
                samples = (
                    POSE_SAMPLE.pack(
                        state.simulation_time_s,
                        state.physics_frame,
                        state.world_x_m,
                        state.world_y_m,
                        state.heading_rad,
                    ),
                )
            flags = (1 if state.retired else 0) | (
                2 if state.hazard_active else 0
            )
            payload.extend(
                POSE_DRIVER_HEADER.pack(
                    state.driver_id,
                    flags,
                    len(samples),
                )
            )
            for sample in samples:
                payload.extend(sample)
        return bytes(payload)

    @staticmethod
    def _dashboard_payload(dashboard: Any) -> dict[str, Any]:
        """Serialize a complete logical snapshot in compact JSON form."""
        payload = dashboard.model_dump(
            exclude_defaults=True,
            include=DASHBOARD_PAYLOAD_INCLUDE,
        )
        payload["type"] = "race_state"
        return payload

    @staticmethod
    def _timing_payload(dashboard: Any) -> dict[str, Any]:
        return {
            "type": "race_timing",
            "positions": [
                {
                    "driver_id": driver.driver_id,
                    **{
                        field: getattr(driver, field)
                        for field in TIMING_POSITION_FIELDS
                    },
                }
                for driver in dashboard.positions
            ],
        }

    def _history_lengths(self) -> dict[int, int]:
        return {
            state.driver_id: len(
                self.engine._lap_history.get(state.driver_id, [])
            )
            for state in self.engine.driver_states.values()
        }

    async def _broadcast_history_if_changed(self) -> None:
        lengths = self._history_lengths()
        if lengths == self._last_history_lengths:
            return
        history_update = self.engine.build_history_state(
            since_by_driver=self._last_history_lengths,
        )
        self._last_history_lengths = lengths
        if history_update.histories:
            await self._broadcast(history_update.model_dump())

    def _prepare_public_events(
        self,
        events: list[RaceEvent],
        *,
        now: float,
    ) -> list[RaceEvent]:
        """Return a compact, stable-ID feed without altering physics events."""
        prepared: list[RaceEvent] = []
        seen_signatures: set[tuple[str, str, str]] = set()
        for event in events:
            if event.type in INTERNAL_EVENT_TYPES:
                continue
            signature = (event.type, event.driver, event.message)
            if signature in seen_signatures:
                continue
            seen_signatures.add(signature)
            cooldown = EVENT_FEED_COOLDOWN_SECONDS.get(event.type, 0.0)
            cooldown_key = (event.type, event.driver)
            last_sent_at = self._event_feed_last_sent_at.get(
                cooldown_key,
                float("-inf"),
            )
            if cooldown > 0 and now - last_sent_at < cooldown:
                continue
            if (
                event.type in ROUTINE_BATTLE_EVENT_TYPES
                and now - self._routine_battle_event_last_sent_at
                < ROUTINE_BATTLE_GLOBAL_COOLDOWN_SECONDS
            ):
                continue
            self._event_feed_last_sent_at[cooldown_key] = now
            if event.type in ROUTINE_BATTLE_EVENT_TYPES:
                self._routine_battle_event_last_sent_at = now
            self._event_sequence += 1
            prepared.append(
                event.model_copy(update={"event_id": self._event_sequence})
            )
            if len(prepared) >= MAX_PUBLIC_EVENTS_PER_PACKET:
                break
        return prepared

    async def _broadcast_public_events(
        self,
        events: list[RaceEvent],
        *,
        now: float | None = None,
    ) -> None:
        if not events:
            return
        current_time = (
            now
            if now is not None
            else asyncio.get_running_loop().time()
        )
        prepared = self._prepare_public_events(events, now=current_time)
        if prepared:
            await self._broadcast(
                RaceEventsMessage(events=prepared).model_dump()
            )

    async def _broadcast_immediate_snapshot(
        self,
        events: list[RaceEvent] | None = None,
    ) -> None:
        """Publish command-visible state without restoring the 30 Hz full tick."""
        dashboard_payload = self.engine.build_dashboard_payload(
            runtime_metrics=self._latest_runtime_metrics,
        )
        await self._broadcast_bytes(self._pose_payload())
        await self._broadcast(dashboard_payload)
        if events:
            await self._broadcast_public_events(events)

    async def add_client(self, ws: WebSocket) -> None:
        self.clients.add(ws)
        await ws.send_json(self.race_info.model_dump())
        dashboard_payload = self.engine.build_dashboard_payload()
        await ws.send_bytes(self._pose_payload())
        await ws.send_json(dashboard_payload)
        await ws.send_json(self.engine.build_history_state().model_dump())
        await ws.send_json(self.engine.build_timing_payload())
        self._last_history_lengths = self._history_lengths()

    def remove_client(self, ws: WebSocket) -> None:
        self.clients.discard(ws)

    async def handle_command(self, data: dict[str, Any]) -> dict[str, Any]:
        cmd_type = data.get("type", "")

        if cmd_type == "pit_call":
            driver_id = data.get("driver_id")
            tire_choice = data.get("tire_choice", "MEDIUM")
            if driver_id is None:
                return {
                    "type": "command_error",
                    "message": "driver_id required",
                    "message_ko": "드라이버 ID가 필요합니다",
                }
            try:
                compound = parse_tire_choice(tire_choice)
            except ValueError:
                return {
                    "type": "command_error",
                    "message": f"Invalid tire: {tire_choice}",
                    "message_ko": f"잘못된 타이어 선택입니다: {tire_choice}",
                }
            error = self.engine.request_pit(int(driver_id), compound)
            if error:
                return {
                    "type": "command_error",
                    "message": error,
                    "message_ko": f"피트콜을 처리할 수 없습니다: {error}",
                }
            return {
                "type": "command_ack",
                "command": "pit_call",
                "driver_id": driver_id,
                "message": f"Pit call registered for driver {driver_id} ({tire_choice})",
                "message_ko": f"드라이버 {driver_id}에게 피트콜을 전달했습니다 ({tire_choice})",
            }

        if cmd_type == "set_pace_mode":
            driver_id = data.get("driver_id")
            pace_mode_value = data.get("pace_mode", PaceMode.STANDARD.value)
            if driver_id is None:
                return {
                    "type": "command_error",
                    "message": "driver_id required",
                    "message_ko": "드라이버 ID가 필요합니다",
                }
            try:
                pace_mode = PaceMode(str(pace_mode_value).upper())
            except ValueError:
                return {
                    "type": "command_error",
                    "message": f"Invalid pace mode: {pace_mode_value}",
                    "message_ko": f"잘못된 페이스 모드입니다: {pace_mode_value}",
                }
            error = self.engine.set_pace_mode(int(driver_id), pace_mode)
            if error:
                return {
                    "type": "command_error",
                    "message": error,
                    "message_ko": f"페이스 모드를 변경할 수 없습니다: {error}",
                }
            return {
                "type": "command_ack",
                "command": "set_pace_mode",
                "driver_id": driver_id,
                "pace_mode": pace_mode.value,
                "message": f"Pace mode set to {pace_mode.value} for driver {driver_id}",
                "message_ko": f"드라이버 {driver_id}의 페이스 모드를 {pace_mode.value}로 설정했습니다",
            }

        if cmd_type == "set_speed":
            multiplier = data.get("multiplier", 1)
            try:
                accepted = self.engine.set_speed(int(multiplier))
            except (TypeError, ValueError):
                accepted = False
            if not accepted:
                return {
                    "type": "command_error",
                    "command": "set_speed",
                    "message": "Speed multiplier must be 1 or 2",
                    "message_ko": "배속은 1 또는 2여야 합니다",
                }
            return {
                "type": "command_ack",
                "command": "set_speed",
                "multiplier": multiplier,
                "message": f"Speed set to {multiplier}x",
                "message_ko": f"레이스 속도를 {multiplier}배로 설정했습니다",
            }

        if cmd_type == "pause":
            self.engine.pause_race()
            await self._broadcast_immediate_snapshot()
            return {
                "type": "command_ack",
                "command": "pause",
                "message": "Race paused",
                "message_ko": "레이스를 일시정지했습니다",
            }

        if cmd_type == "resume":
            self.engine.resume_race()
            await self._broadcast_immediate_snapshot()
            return {
                "type": "command_ack",
                "command": "resume",
                "message": "Race resumed",
                "message_ko": "레이스를 재개했습니다",
            }

        # DEV RACE CONTROL: remove this block with frontend/components/DevRaceControl.
        if cmd_type == "dev_set_race_phase":
            if not DEV_RACE_CONTROLS_ENABLED:
                return {
                    "type": "command_error",
                    "message": "Development race controls are disabled",
                    "message_ko": "개발용 레이스 컨트롤이 비활성화되어 있습니다",
                }

            requested_phase = str(data.get("phase", "")).lower()
            try:
                events = self.engine.set_race_control_phase_for_testing(requested_phase)
            except ValueError as exc:
                return {
                    "type": "command_error",
                    "message": str(exc),
                    "message_ko": f"지원하지 않는 레이스 상태입니다: {requested_phase}",
                }

            await self._broadcast_immediate_snapshot(events)
            active_phase = self.engine.race_phase.upper()
            if requested_phase == "green" and self.engine.race_phase == "sc":
                return {
                    "type": "command_ack",
                    "command": "dev_set_race_phase",
                    "race_phase": self.engine.race_phase,
                    "message": "Safety Car withdrawal requested",
                    "message_ko": "세이프티카 철수를 요청했습니다",
                }
            return {
                "type": "command_ack",
                "command": "dev_set_race_phase",
                "race_phase": self.engine.race_phase,
                "message": f"Development race control set to {active_phase}",
                "message_ko": f"개발용 레이스 컨트롤을 {active_phase}(으)로 변경했습니다",
            }

        if cmd_type == "dev_retire_driver":
            if not DEV_RACE_CONTROLS_ENABLED:
                return {
                    "type": "command_error",
                    "message": "Development race controls are disabled",
                    "message_ko": "개발용 레이스 컨트롤이 비활성화되어 있습니다",
                }
            driver_id = data.get("driver_id")
            if driver_id is None:
                return {
                    "type": "command_error",
                    "message": "driver_id required",
                    "message_ko": "드라이버 ID가 필요합니다",
                }
            try:
                events = self.engine.retire_driver_for_testing(int(driver_id))
            except (TypeError, ValueError) as exc:
                return {
                    "type": "command_error",
                    "message": str(exc),
                    "message_ko": f"개발용 리타이어를 적용할 수 없습니다: {exc}",
                }
            await self._broadcast_immediate_snapshot(events)
            return {
                "type": "command_ack",
                "command": "dev_retire_driver",
                "driver_id": int(driver_id),
                "message": f"Development retirement applied to driver {driver_id}",
                "message_ko": f"드라이버 {driver_id}에게 개발용 리타이어를 적용했습니다",
            }

        return {
            "type": "command_error",
            "message": f"Unknown command: {cmd_type}",
            "message_ko": f"알 수 없는 명령입니다: {cmd_type}",
        }


class SessionManager:
    """Manages the single active race session (prototype)."""

    def __init__(self) -> None:
        self._session: RaceSession | None = None

    @property
    def session(self) -> RaceSession | None:
        return self._session

    def diagnostic_counts(self) -> dict[str, Any]:
        session = self._session
        if session is None:
            return {
                "active_session": False,
                "session_id": None,
                "client_count": 0,
                "loop_task_exists": False,
                "trajectory_sample_count": 0,
                "event_feed_auxiliary_entries": 0,
                "last_history_length_entries": 0,
                "safety_car": {
                    "race_phase": "green",
                    "safety_car_stage": "inactive",
                    "queue_formed": False,
                    "running_order": [],
                    "physical_order": [],
                    "caught_driver_ids": [],
                    "unlap_driver_ids": [],
                    "pit_exit_order_target_ids": [],
                    "order_yield_targets": {},
                    "active_correction": None,
                    "drivers": [],
                },
                "tire_temperature": empty_tire_temperature_diagnostic_snapshot(),
                **track_physics_cache_counts(),
            }
        return {"active_session": True, **session.diagnostic_counts()}

    def create_session(
        self,
        request: RaceSetupRequest,
        drivers: list[Driver],
        teams: list[Team],
        circuits: list[Circuit],
    ) -> RaceSession:
        self.clear()
        build = FullEngineAdapter().build_race(
            request=request,
            drivers=drivers,
            teams=teams,
            circuits=circuits,
        )
        session_id = str(uuid.uuid4())
        self._session = RaceSession(
            session_id=session_id,
            engine=build.engine,
            circuit=build.circuit,
            player_team=build.player_team,
            player_drivers=list(build.player_drivers),
        )
        return self._session

    def clear(self) -> None:
        session = self._session
        self._session = None
        if session is not None:
            session.stop_loop()
            del session
            clear_vehicle_track_physics_cache()
            gc.collect()

    async def clear_async(self) -> None:
        """Fully close the active session before allowing the next race."""
        session = self._session
        self._session = None
        if session is None:
            return
        await session.close()
        del session
        clear_vehicle_track_physics_cache()
        gc.collect()


session_manager = SessionManager()
