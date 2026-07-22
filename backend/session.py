"""In-memory race session management."""

from __future__ import annotations

import asyncio
import os
import uuid
from dataclasses import dataclass
from typing import Any

from fastapi import WebSocket

from models.schemas import (
    Circuit,
    Driver,
    PaceMode,
    RaceInfoMessage,
    RaceSetupRequest,
    Team,
    VehicleTrajectorySample,
)
from simulation.pit_stop import parse_tire_choice
from simulation.race_engine import RaceEngine
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS
from simulation.vehicle_dimensions import (
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
    PHYSICAL_CAR_WHEELBASE_M,
)

BROADCAST_HZ = 30
BROADCAST_INTERVAL = 1.0 / BROADCAST_HZ
MAX_PHYSICS_STEPS_PER_SLICE = 5
MAX_SIMULATION_BACKLOG_SECONDS = 0.5
MAX_TRAJECTORY_SAMPLES_PER_DRIVER = 64
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
        self.clients: set[WebSocket] = set()
        self._loop_task: asyncio.Task | None = None
        self._trajectory_samples: dict[int, list[VehicleTrajectorySample]] = {}
        self._cadence_metrics = SessionCadenceMetrics()

    @property
    def race_info(self) -> RaceInfoMessage:
        return RaceInfoMessage(
            circuit_name=self.circuit.name,
            total_laps=self.circuit.total_laps,
            player_team=self.player_team.name,
            player_team_color=self.player_team.color,
            player_drivers=[d.id for d in self.player_drivers],
            track_length_m=self.circuit.track_length_m,
            world_origin_x_render=(
                self.engine._track_physics.coordinate_frame.origin_x_render
                if self.engine._track_physics.coordinate_frame
                else 0.0
            ),
            world_origin_y_render=(
                self.engine._track_physics.coordinate_frame.origin_y_render
                if self.engine._track_physics.coordinate_frame
                else 0.0
            ),
            world_meters_per_render_unit=(
                self.engine._track_physics.coordinate_frame.meters_per_render_unit
                if self.engine._track_physics.coordinate_frame
                else 1.0
            ),
            track_width_m=self.circuit.track_width_m,
            car_width_m=PHYSICAL_CAR_WIDTH_M,
            car_length_m=PHYSICAL_CAR_LENGTH_M,
            wheelbase_m=PHYSICAL_CAR_WHEELBASE_M,
            grid_slots=self.engine.get_grid_slots(),
            racing_line_profile=[
                [sample.progress, sample.racing_line_offset_m]
                for sample in self.engine._track_physics.samples
            ],
            track_width_profile=[
                [sample.progress, sample.left_width_m, sample.right_width_m]
                for sample in self.engine._track_physics.samples
            ],
            surface_zones=self.engine._track_surface.zones,
            track_conditions=self.circuit.track_conditions,
            racing_line_coords=self.engine._track_physics.racing_line_coords,
            racing_line_length_m=self.engine._track_physics.racing_line_length_m,
            predicted_racing_lap_time=self.engine._track_physics.predicted_racing_lap_time,
            driving_line_coords=self.engine._track_physics.driving_line_coords,
            driving_line_lengths_m=self.engine._track_physics.driving_line_lengths_m,
            predicted_line_lap_times=self.engine._track_physics.predicted_line_lap_times,
            track_coords=self.circuit.track_coords,
            start_finish_index=self.circuit.start_finish_index,
            # Graphics and marker interpolation must use the same entry/exit
            # anchors and arc-length frame as authoritative pit physics.
            pit_lane_coords=self.engine.get_pit_route_coords(),
            pit_wall_coords=self.circuit.pit_wall_coords,
            pit_box_offset=self.circuit.pit_lane.box_offset if self.circuit.pit_lane else 11.0,
            pit_lane_width_m=(
                self.circuit.pit_lane.lane_width_m if self.circuit.pit_lane else 4.0
            ),
            pit_side_entry_progress=(
                self.circuit.pit_lane.side_entry_progress
                if self.circuit.pit_lane
                else 0.02
            ),
            pit_speed_limit_start=(
                self.circuit.pit_lane.speed_limit_start if self.circuit.pit_lane else 0.12
            ),
            pit_box_progress=(
                self.circuit.pit_lane.box_progress if self.circuit.pit_lane else 0.50
            ),
            pit_speed_limit_end=(
                self.circuit.pit_lane.speed_limit_end if self.circuit.pit_lane else 0.88
            ),
            pit_side_rejoin_progress=(
                self.circuit.pit_lane.side_rejoin_progress
                if self.circuit.pit_lane
                else 0.94
            ),
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
        self.clients.clear()

    async def _game_loop(self) -> None:
        """Advance authoritative physics independently from screen broadcasts.

        Physics always consumes exact 20 ms simulation steps.  Wall-clock time is
        accumulated as simulation credit, while snapshots are emitted on their own
        30 Hz deadline.  This avoids adding physics compute time to every sleep and
        preserves all intermediate poses for the renderer.
        """
        loop = asyncio.get_running_loop()
        last_wall_time = loop.time()
        next_broadcast_time = last_wall_time
        simulation_credit = 0.0
        pending_events: list = []
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
                next_broadcast_time = now + BROADCAST_INTERVAL
                await asyncio.sleep(BROADCAST_INTERVAL)
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
            if now >= next_broadcast_time or self.engine.finished:
                requested_speed = (
                    self.engine.speed_multiplier if self.engine.race_started else 1
                )
                runtime_metrics = self._cadence_metrics.snapshot(
                    now=now,
                    scheduled_broadcast_at=next_broadcast_time,
                    simulation_backlog_seconds=simulation_credit,
                    requested_speed_multiplier=requested_speed,
                    paused=self.engine.paused,
                )
                tick = self.engine.build_tick_state(
                    pending_events,
                    self._trajectory_samples,
                    runtime_metrics,
                )
                await self._broadcast(tick.model_dump())
                pending_events = []
                self._trajectory_samples = {}
                next_broadcast_time = now + BROADCAST_INTERVAL

            if steps_run >= MAX_PHYSICS_STEPS_PER_SLICE:
                await asyncio.sleep(0)
                continue

            time_until_broadcast = max(0.0, next_broadcast_time - loop.time())
            if self.engine.paused:
                sleep_seconds = min(BROADCAST_INTERVAL, time_until_broadcast)
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
                VehicleTrajectorySample(
                    simulation_time_s=state.simulation_time_s,
                    physics_frame=state.physics_frame,
                    world_x_m=state.world_x_m,
                    world_y_m=state.world_y_m,
                    heading_rad=state.heading_rad,
                    progress=state.progress,
                    lateral_offset_m=state.lateral_offset_m,
                    in_pit=state.in_pit,
                    pit_lane_progress=self.engine._pit_lane_progress(state.driver_id),
                )
            )
            if len(samples) > MAX_TRAJECTORY_SAMPLES_PER_DRIVER:
                del samples[:-MAX_TRAJECTORY_SAMPLES_PER_DRIVER]

    async def _broadcast(self, message: dict[str, Any]) -> None:
        dead: set[WebSocket] = set()
        for ws in list(self.clients):
            try:
                await asyncio.wait_for(ws.send_json(message), timeout=1.0)
            except Exception:
                dead.add(ws)
        self.clients -= dead

    async def add_client(self, ws: WebSocket) -> None:
        self.clients.add(ws)
        await ws.send_json(self.race_info.model_dump())
        tick = self.engine.build_tick_state()
        await ws.send_json(tick.model_dump())

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
                    "message": "Speed multiplier must be 1, 2, or 3",
                    "message_ko": "배속은 1, 2, 3 중 하나여야 합니다",
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
            await self._broadcast(self.engine.build_tick_state().model_dump())
            return {
                "type": "command_ack",
                "command": "pause",
                "message": "Race paused",
                "message_ko": "레이스를 일시정지했습니다",
            }

        if cmd_type == "resume":
            self.engine.resume_race()
            await self._broadcast(self.engine.build_tick_state().model_dump())
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

            await self._broadcast(self.engine.build_tick_state(events).model_dump())
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

    def create_session(
        self,
        request: RaceSetupRequest,
        drivers: list[Driver],
        teams: list[Team],
        circuits: list[Circuit],
    ) -> RaceSession:
        self.clear()

        circuit = next((c for c in circuits if c.id == request.circuit_id), None)
        if circuit is None:
            raise ValueError(f"Circuit {request.circuit_id} not found")
        circuit = circuit.model_copy(update={"total_laps": request.total_laps})

        player_team = next((t for t in teams if t.id == request.player_team_id), None)
        if player_team is None:
            raise ValueError(f"Team {request.player_team_id} not found")

        team_map = {t.id: t for t in teams}
        player_drivers = [d for d in drivers if d.team_id == player_team.id]
        if len(player_drivers) < 1:
            raise ValueError("Player team has no drivers")
        player_driver_ids = {d.id for d in player_drivers}

        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=team_map,
            player_team_id=player_team.id,
            player_driver_ids=list(player_driver_ids),
            starting_tires={
                driver_id: compound
                for driver_id, compound in request.starting_tires.items()
                if driver_id in player_driver_ids
            },
            grid_order=request.grid_order,
        )

        session_id = str(uuid.uuid4())
        self._session = RaceSession(
            session_id=session_id,
            engine=engine,
            circuit=circuit,
            player_team=player_team,
            player_drivers=player_drivers,
        )
        return self._session

    def clear(self) -> None:
        if self._session is not None:
            self._session.stop_loop()
        self._session = None


session_manager = SessionManager()
