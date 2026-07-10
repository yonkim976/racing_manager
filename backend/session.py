"""In-memory race session management."""

from __future__ import annotations

import asyncio
import uuid
from typing import Any

from fastapi import WebSocket

from models.schemas import Circuit, Driver, PaceMode, RaceInfoMessage, RaceSetupRequest, Team
from simulation.physics import GAME_TICK_SECONDS
from simulation.pit_stop import parse_tire_choice
from simulation.race_engine import RaceEngine

BROADCAST_INTERVAL = 0.2  # seconds (real time)


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

    @property
    def race_info(self) -> RaceInfoMessage:
        return RaceInfoMessage(
            circuit_name=self.circuit.name,
            total_laps=self.circuit.total_laps,
            player_team=self.player_team.name,
            player_team_color=self.player_team.color,
            player_drivers=[d.id for d in self.player_drivers],
            track_length_m=self.circuit.track_length_m,
            track_coords=self.circuit.track_coords,
            start_finish_index=self.circuit.start_finish_index,
            pit_lane_coords=self.circuit.pit_lane_coords,
            pit_wall_coords=self.circuit.pit_wall_coords,
            pit_box_offset=self.circuit.pit_lane.box_offset if self.circuit.pit_lane else 11.0,
            drs_zones=self.circuit.drs_zones,
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
        while not self.engine.finished:
            await asyncio.sleep(BROADCAST_INTERVAL)
            if not self.clients:
                continue

            events: list = []
            if not self.engine.paused:
                delta = BROADCAST_INTERVAL * self.engine.speed_multiplier
                events = self._advance_engine(delta)

            tick = self.engine.build_tick_state(events)
            await self._broadcast(tick.model_dump())

        results = self.engine.build_results()
        await self._broadcast({"type": "race_end", "results": results})

    def _advance_engine(self, game_seconds: float) -> list:
        """Advance game time using small fixed simulation steps."""
        events: list = []
        remaining = game_seconds
        while remaining > 1e-9 and not self.engine.finished:
            step = min(GAME_TICK_SECONDS, remaining)
            events.extend(self.engine.tick(step))
            remaining -= step
        return events

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
            self.engine.set_speed(int(multiplier))
            return {
                "type": "command_ack",
                "command": "set_speed",
                "multiplier": multiplier,
                "message": f"Speed set to {multiplier}x",
                "message_ko": f"레이스 속도를 {multiplier}배로 설정했습니다",
            }

        if cmd_type == "pause":
            self.engine.pause()
            return {
                "type": "command_ack",
                "command": "pause",
                "message": "Race paused",
                "message_ko": "레이스를 일시정지했습니다",
            }

        if cmd_type == "resume":
            self.engine.resume()
            return {
                "type": "command_ack",
                "command": "resume",
                "message": "Race resumed",
                "message_ko": "레이스를 재개했습니다",
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
