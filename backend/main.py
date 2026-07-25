"""FastAPI application entry point."""

from __future__ import annotations

import logging
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

from data_loader import enrich_drivers, load_circuits, load_drivers, load_teams
from models.schemas import Circuit, QualifyingRequest, QualifyingResponse, RaceSetupRequest, RaceSetupResponse
from session import session_manager
from simulation.qualifying import run_qualifying
from simulation.track_compiler import compile_circuit_layout
from simulation.track_geometry import validate_circuit_geometry_detailed

FRONTEND_DIST = Path(__file__).resolve().parent.parent / "frontend" / "dist"
logger = logging.getLogger(__name__)

app = FastAPI(title="F1 Race Manager", version="0.1.0")

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

    team_map = {team.id: team for team in _teams}
    return run_qualifying(
        circuit=circuit,
        drivers=_drivers,
        teams=team_map,
        player_team=player_team,
        attempt_laps=request.attempt_laps,
    )


@app.post("/api/race/setup", response_model=RaceSetupResponse)
async def setup_race(request: RaceSetupRequest):
    try:
        await session_manager.clear_async()
        session = session_manager.create_session(request, _drivers, _teams, _circuits)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    player_drivers = session.player_drivers
    await session.start_loop()

    return RaceSetupResponse(
        session_id=session.session_id,
        circuit=session.circuit,
        player_team=session.player_team,
        player_drivers=player_drivers,
        grid_order=session.engine.get_grid_order(),
    )


@app.delete("/api/race/session")
async def clear_race_session():
    await session_manager.clear_async()
    return {"status": "cleared"}


@app.websocket("/ws/race")
async def race_websocket(ws: WebSocket):
    await ws.accept()

    session = session_manager.session
    if session is None:
        await ws.send_json({"type": "error", "message": "No active race. POST /api/race/setup first."})
        await ws.close()
        return

    await session.add_client(ws)

    try:
        while True:
            data = await ws.receive_json()
            try:
                response = await session.handle_command(data)
            except Exception:
                logger.exception("Race WebSocket command failed")
                response = {
                    "type": "command_error",
                    "message": "Command failed; the race connection remains active",
                    "message_ko": "명령 처리에 실패했지만 레이스 연결은 유지됩니다",
                }
            await ws.send_json(response)
    except WebSocketDisconnect:
        session.remove_client(ws)
    except Exception:
        session.remove_client(ws)
        raise


@app.get("/api/health")
def health():
    return {"status": "ok", "active_session": session_manager.session is not None}


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
