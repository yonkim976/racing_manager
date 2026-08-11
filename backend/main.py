"""FastAPI application entry point."""

from __future__ import annotations

from collections import Counter
from contextlib import asynccontextmanager
from dataclasses import replace
import asyncio
import hmac
import logging
import os
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

from data_loader import (
    enrich_drivers,
    load_circuits,
    load_drivers,
    load_teams,
    resolve_circuit_thermal_conditions,
    resolve_tire_compound,
)
from desktop_diagnostics import desktop_mode_enabled, desktop_token, process_metrics
from engines import simulation_engine_factory
from models.schemas import (
    AbstractRaceAuthority,
    Circuit,
    DryTireRole,
    QualifyingRequest,
    QualifyingResponse,
    RaceSetupRequest,
    RaceSetupResponse,
    SimulationMode,
    TireCompound,
)
from session import session_manager
from simulation.abstract import (
    AbstractRaceEngine,
    AbstractSessionSnapshot,
    TireConditionSnapshot,
)
from simulation.abstract.broadcast import abstract_broadcast_session_manager
from simulation.qualifying import run_qualifying
from simulation.track_compiler import compile_circuit_layout
from simulation.track_geometry import validate_circuit_geometry_detailed

FRONTEND_DIST = Path(
    os.getenv(
        "F1_FRONTEND_DIST",
        str(Path(__file__).resolve().parent.parent / "frontend" / "dist"),
    )
).resolve()
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    logger.info("FastAPI sidecar starting pid=%s desktop=%s", os.getpid(), desktop_mode_enabled())
    try:
        yield
    finally:
        await session_manager.clear_async()
        await abstract_broadcast_session_manager.clear_async()
        logger.info("FastAPI sidecar shutdown complete pid=%s", os.getpid())


app = FastAPI(title="F1 Race Manager", version="0.1.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

_drivers = load_drivers()
_teams = load_teams()
_circuits = load_circuits()


def _abstract_snapshot(
    *,
    session_id: str,
    session_seed: int | str,
    circuit: Circuit,
    drivers: list,
    teams: list,
    track_conditions,
    starting_tires: dict[int, TireCompound] | None = None,
) -> AbstractSessionSnapshot:
    """Build the abstract immutable snapshot at the API boundary."""

    snapshot = AbstractSessionSnapshot.from_content(
        session_id=session_id,
        session_seed=session_seed,
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        content_version=circuit.data_version or f"circuit-{circuit.id}",
        ruleset_version=(
            circuit.tire_compound_nomination.ruleset
            if circuit.tire_compound_nomination is not None
            else "2026_C1_C5"
        ),
        track_conditions=track_conditions,
        tire_role=DryTireRole.MEDIUM,
    )
    requested_tires = starting_tires or {}
    entries = []
    for entry in snapshot.entries:
        requested_role = requested_tires.get(entry.driver_id, TireCompound.MEDIUM)
        physical_compound, tire_role = resolve_tire_compound(circuit, requested_role)
        entries.append(
            replace(
                entry,
                tire=TireConditionSnapshot(
                    physical_compound=physical_compound.value,
                    tire_role=tire_role.value,
                ),
            )
        )
    return replace(snapshot, entries=tuple(entries))


def _abstract_qualifying_response(
    *,
    snapshot: AbstractSessionSnapshot,
    result,
    circuit: Circuit,
    player_team,
    drivers: list,
    teams: list,
    track_conditions,
    thermal_preset,
    conditions_source: str,
    simulation_mode: SimulationMode = SimulationMode.ABSTRACT,
) -> QualifyingResponse:
    driver_map = {driver.id: driver for driver in drivers}
    team_map = {team.id: team for team in teams}
    result_map = {item.driver_id: item for item in result.driver_results}
    qualifying_results = []
    for grid_entry in result.grid:
        driver = driver_map[grid_entry.driver_id]
        team = team_map[driver.team_id]
        driver_result = result_map[driver.id]
        session_best = {}
        for run in driver_result.runs:
            current = session_best.get(run.session_name)
            if current is None or run.flying_lap_time_s < current:
                session_best[run.session_name] = run.flying_lap_time_s
        qualifying_results.append(
            {
                "position": grid_entry.position,
                "driver_id": driver.id,
                "name": driver.name,
                "full_name": driver.name,
                "team": team.name,
                "team_color": team.color,
                "tire_role": snapshot.entry_by_driver_id()[driver.id].tire.tire_role,
                "physical_tire_compound": snapshot.entry_by_driver_id()[driver.id].tire.physical_compound,
                "best_lap_time": grid_entry.best_lap_time_s,
                "gap": "—" if grid_entry.position == 1 else f"+{grid_entry.gap_s:.3f}",
                "laps": [run.flying_lap_time_s for run in driver_result.runs],
                "knockout": driver_result.eliminated_in or driver_result.advanced_to,
                "q1_time": session_best.get("Q1"),
                "q2_time": session_best.get("Q2"),
                "q3_time": session_best.get("Q3"),
            }
        )
    return QualifyingResponse(
        simulation_mode=simulation_mode,
        session_seed=snapshot.session_seed,
        circuit=circuit,
        player_team=player_team,
        results=qualifying_results,
        grid_order=[entry.driver_id for entry in result.grid],
        track_conditions=track_conditions,
        thermal_preset=thermal_preset,
        track_conditions_source=conditions_source,
        tire_compound_nomination=circuit.tire_compound_nomination,
        canonical_result_hash=result.canonical_result_hash,
    )


def _abstract_race_summary(result, snapshot: AbstractSessionSnapshot, drivers: list, teams: list) -> dict:
    driver_map = {driver.id: driver for driver in drivers}
    team_map = {team.id: team for team in teams}
    grid = []
    for position, driver_id in enumerate(result.grid_order, start=1):
        driver = driver_map[driver_id]
        team = team_map[driver.team_id]
        grid.append(
            {
                "position": position,
                "driver_id": driver_id,
                "name": driver.name,
                "abbreviation": driver.abbreviation,
                "team": team.name,
                "team_color": team.color,
            }
        )
    authoritative_classification = getattr(result, "classification", None)
    if authoritative_classification is None:
        classification = [
            {
                "position": position,
                "driver_id": driver_id,
                "status": "finished",
                "finish_time_s": None,
                "retirement_time_s": None,
                "retirement_reason": None,
            }
            for position, driver_id in enumerate(result.finish_order, start=1)
        ]
    else:
        classification = [item.to_dict() for item in authoritative_classification]
    return {
        "total_laps": result.total_laps,
        "grid": grid,
        "finish_order": list(result.finish_order),
        "classification": classification,
        "event_count": len(result.logical_events),
        "event_types": dict(sorted(Counter(event.event_type for event in result.logical_events).items())),
        "canonical_result_hash": result.canonical_result_hash,
        "source_mode": "abstract",
        "snapshot_hash": snapshot.snapshot_hash,
    }


def _abstract_broadcast_setup(
    *,
    snapshot: AbstractSessionSnapshot,
    total_laps: int,
    requested_grid: tuple[int | str, ...] | None,
    abstract_engine: AbstractRaceAuthority = AbstractRaceAuthority.STAGE4,
) -> tuple[AbstractSessionSnapshot, tuple[int | str, ...], dict]:
    """Resolve only the grid needed to start a buffered live broadcast."""

    qualifying = AbstractRaceEngine(snapshot).run_qualifying()
    grid_order = requested_grid or tuple(entry.driver_id for entry in qualifying.grid)
    resolved_snapshot = replace(snapshot, initial_grid_order=grid_order)
    driver_map = {driver.id: driver for driver in _drivers}
    team_map = {team.id: team for team in _teams}
    grid = []
    for position, driver_id in enumerate(grid_order, start=1):
        driver = driver_map[driver_id]
        team = team_map[driver.team_id]
        grid.append(
            {
                "position": position,
                "driver_id": driver_id,
                "name": driver.name,
                "abbreviation": driver.abbreviation,
                "team": team.name,
                "team_color": team.color,
            }
        )
    return resolved_snapshot, grid_order, {
        "total_laps": total_laps,
        "grid": grid,
        "finish_order": [],
        "event_count": 0,
        "event_types": {},
        "canonical_result_hash": None,
        "source_mode": "abstract",
        "abstract_engine": abstract_engine.value,
        "status": "buffered_broadcast_running",
        "snapshot_hash": resolved_snapshot.snapshot_hash,
    }


def _compute_abstract_race(
    *,
    snapshot: AbstractSessionSnapshot,
    total_laps: int,
    requested_grid: tuple[int | str, ...] | None,
    abstract_engine: AbstractRaceAuthority = AbstractRaceAuthority.STAGE4,
) -> tuple[AbstractSessionSnapshot, object, dict]:
    """CPU-bound ABSTRACT setup work, isolated from the FastAPI event loop."""

    qualifying = AbstractRaceEngine(snapshot).run_qualifying()
    grid_order = requested_grid or tuple(entry.driver_id for entry in qualifying.grid)
    resolved_snapshot = replace(snapshot, initial_grid_order=grid_order)
    engine = AbstractRaceEngine(resolved_snapshot)
    if abstract_engine is AbstractRaceAuthority.PROGRESS_V5:
        result = engine.run_progress_race(
            total_laps=total_laps,
            tick_seconds=0.10,
        )
    else:
        result = engine.run_race(
            total_laps=total_laps,
            tick_seconds=0.10,
        )
    summary = _abstract_race_summary(result, resolved_snapshot, _drivers, _teams)
    summary["abstract_engine"] = abstract_engine.value
    return resolved_snapshot, result, summary


@app.get("/api/drivers")
def get_drivers():
    return enrich_drivers(_drivers, _teams)


@app.get("/api/teams")
def get_teams():
    return _teams


@app.get("/api/circuits")
def get_circuits():
    return _circuits


@app.post("/api/circuits/validate")
def validate_circuit(circuit: Circuit):
    try:
        compiled = compile_circuit_layout(circuit)
    except ValueError as exc:
        return {"ok": False, "errors": [str(exc)], "warnings": [], "circuit": circuit}

    result = validate_circuit_geometry_detailed(compiled)
    errors = result["errors"]
    warnings = result["warnings"]
    return {"ok": len(errors) == 0, "errors": errors, "warnings": warnings, "circuit": compiled}


@app.post("/api/qualifying/run", response_model=QualifyingResponse)
def run_qualifying_session(request: QualifyingRequest):
    circuit = next((c for c in _circuits if c.id == request.circuit_id), None)
    if circuit is None:
        raise HTTPException(status_code=400, detail=f"Circuit {request.circuit_id} not found")

    player_team = next((t for t in _teams if t.id == request.player_team_id), None)
    if player_team is None:
        raise HTTPException(status_code=400, detail=f"Team {request.player_team_id} not found")

    try:
        track_conditions, thermal_preset, conditions_source = (
            resolve_circuit_thermal_conditions(
                circuit,
                thermal_preset=request.thermal_preset,
            )
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    adapter = simulation_engine_factory.for_mode(request.simulation_mode)
    return adapter.run_qualifying(
        request=request,
        circuit=circuit,
        player_team=player_team,
        drivers=_drivers,
        teams=_teams,
        track_conditions=track_conditions,
        thermal_preset=thermal_preset,
        conditions_source=conditions_source,
    )


@app.post("/api/race/setup", response_model=RaceSetupResponse)
async def setup_race(request: RaceSetupRequest):
    try:
        await session_manager.clear_async()
        await abstract_broadcast_session_manager.clear_async()

        circuit = next((c for c in _circuits if c.id == request.circuit_id), None)
        player_team = next((t for t in _teams if t.id == request.player_team_id), None)
        if circuit is None:
            raise ValueError(f"Circuit {request.circuit_id} not found")
        if player_team is None:
            raise ValueError(f"Team {request.player_team_id} not found")
        resolved_conditions, resolved_preset, conditions_source = (
            resolve_circuit_thermal_conditions(
                circuit,
                thermal_preset=request.thermal_preset,
                track_conditions=request.track_conditions,
            )
        )

        adapter = simulation_engine_factory.for_mode(request.simulation_mode)
        return await adapter.setup_race(
            request=request,
            circuit=circuit,
            player_team=player_team,
            drivers=_drivers,
            teams=_teams,
            circuits=_circuits,
            track_conditions=resolved_conditions,
            thermal_preset=resolved_preset,
            conditions_source=conditions_source,
            session_manager=session_manager,
            broadcast_manager=abstract_broadcast_session_manager,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.delete("/api/race/session")
async def clear_race_session():
    await session_manager.clear_async()
    await abstract_broadcast_session_manager.clear_async()
    return {"status": "cleared"}


@app.websocket("/ws/race")
async def race_websocket(ws: WebSocket):
    await ws.accept()

    session = session_manager.session
    broadcast_session = abstract_broadcast_session_manager.session
    active_session = broadcast_session or session
    if active_session is None:
        await ws.send_json({"type": "error", "message": "No active race. POST /api/race/setup first."})
        await ws.close()
        return

    try:
        await active_session.add_client(ws)
    except WebSocketDisconnect:
        # A browser reconnect can close the socket during the initial snapshot.
        # Treat that as a normal client lifecycle event instead of leaking a
        # dead client into the abstract broadcast loop.
        active_session.remove_client(ws)
        return
    except Exception:
        active_session.remove_client(ws)
        logger.exception("Race WebSocket initial snapshot failed")
        return

    try:
        while True:
            data = await ws.receive_json()
            try:
                response = await active_session.handle_command(data)
            except Exception:
                logger.exception("Race WebSocket command failed")
                response = {
                    "type": "command_error",
                    "message": "Command failed; the race connection remains active",
                    "message_ko": "명령 처리에 실패했지만 레이스 연결은 유지됩니다",
                }
            await ws.send_json(response)
    except WebSocketDisconnect:
        active_session.remove_client(ws)
    except Exception:
        active_session.remove_client(ws)
        raise


@app.get("/api/health")
def health():
    return {
        "status": "ok",
        "active_session": (
            session_manager.session is not None
            or abstract_broadcast_session_manager.session is not None
        ),
        "desktop_mode": desktop_mode_enabled(),
        "pid": os.getpid(),
    }


def _require_desktop_token(request: Request) -> None:
    if not desktop_mode_enabled():
        raise HTTPException(status_code=404, detail="Desktop diagnostics are disabled")
    expected = desktop_token()
    received = request.headers.get("x-f1-desktop-token", "")
    if not expected or not hmac.compare_digest(received, expected):
        raise HTTPException(status_code=403, detail="Desktop diagnostics token required")


@app.get("/api/desktop/diagnostics")
def desktop_diagnostics(request: Request):
    """Return bounded ownership and SC control facts, never raw pose/dashboard state."""
    _require_desktop_token(request)
    counts = session_manager.diagnostic_counts()
    abstract_session = abstract_broadcast_session_manager.session
    if abstract_session is not None:
        counts = {
            **counts,
            **abstract_session.diagnostic_counts(),
            "active_session": True,
        }
    return {
        "schema_version": 1,
        "pid": os.getpid(),
        "process": process_metrics(),
        **counts,
    }


if FRONTEND_DIST.is_dir() and (FRONTEND_DIST / "index.html").is_file():
    app.mount("/assets", StaticFiles(directory=FRONTEND_DIST / "assets"), name="assets")

    @app.get("/")
    def serve_frontend():
        return FileResponse(FRONTEND_DIST / "index.html")

    @app.get("/vite.svg")
    def serve_vite_icon():
        icon = FRONTEND_DIST / "vite.svg"
        if icon.is_file():
            return FileResponse(icon)
        raise HTTPException(status_code=404)
else:
    @app.get("/")
    def root():
        return HTMLResponse(
            """
            <!DOCTYPE html>
            <html lang="ko">
            <head><meta charset="UTF-8"><title>F1 Race Manager</title></head>
            <body style="font-family:sans-serif;background:#0a0a0f;color:#f0f0f5;padding:40px;">
              <h1>F1 Race Manager — Backend Running</h1>
              <p>API server is up, but the UI has not been built yet.</p>
              <pre style="background:#111;padding:16px;border-radius:8px;">
cd frontend
npm install
npm run build
              </pre>
              <p>Then refresh this page, or run <code>npm run dev</code> and open
                 <a href="http://localhost:5173" style="color:#ff1801;">http://localhost:5173</a>.</p>
            </body>
            </html>
            """
        )
