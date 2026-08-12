"""Product adapter for the progress/event-authoritative management engine."""

from __future__ import annotations

import asyncio
from collections import Counter
from dataclasses import replace
from typing import Any

from data_loader import resolve_tire_compound
from models.schemas import (
    AbstractRaceAuthority,
    Circuit,
    Driver,
    DryTireRole,
    QualifyingRequest,
    QualifyingResponse,
    RaceSetupRequest,
    RaceSetupResponse,
    SimulationMode,
    Team,
    TireCompound,
)
from .runtime import (
    AbstractRaceEngine,
    AbstractSessionSnapshot,
    TireConditionSnapshot,
)

from ..contracts import EngineCapabilities, EngineFamily


class AbstractEngineAdapter:
    """Own every ABSTRACT construction path without importing FULL internals."""

    capabilities = EngineCapabilities(
        family=EngineFamily.ABSTRACT,
        supported_modes=frozenset(
            {
                SimulationMode.ABSTRACT,
                SimulationMode.ABSTRACT_INSTANT,
                SimulationMode.ABSTRACT_BROADCAST,
            }
        ),
        result_authority="logical-progress-and-events",
        presentation_authority="derived-strategic-pose",
        physics_telemetry=False,
        interactive_strategy=True,
    )

    def supports(self, mode: SimulationMode) -> bool:
        return mode in self.capabilities.supported_modes

    @staticmethod
    def build_snapshot(
        *,
        session_id: str,
        session_seed: int | str,
        circuit: Circuit,
        drivers: list[Driver],
        teams: list[Team],
        track_conditions: Any,
        starting_tires: dict[int, TireCompound] | None = None,
    ) -> AbstractSessionSnapshot:
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
            requested_role = requested_tires.get(
                entry.driver_id, TireCompound.MEDIUM
            )
            physical_compound, tire_role = resolve_tire_compound(
                circuit, requested_role
            )
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

    @staticmethod
    def qualifying_response(
        *,
        snapshot: AbstractSessionSnapshot,
        result: Any,
        circuit: Circuit,
        player_team: Team,
        drivers: list[Driver],
        teams: list[Team],
        track_conditions: Any,
        thermal_preset: Any,
        conditions_source: str,
        simulation_mode: SimulationMode,
    ) -> QualifyingResponse:
        driver_map = {driver.id: driver for driver in drivers}
        team_map = {team.id: team for team in teams}
        result_map = {item.driver_id: item for item in result.driver_results}
        entries_by_driver = snapshot.entry_by_driver_id()
        qualifying_results = []
        for grid_entry in result.grid:
            driver = driver_map[grid_entry.driver_id]
            team = team_map[driver.team_id]
            driver_result = result_map[driver.id]
            session_best: dict[str, float] = {}
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
                    "tire_role": entries_by_driver[driver.id].tire.tire_role,
                    "physical_tire_compound": (
                        entries_by_driver[driver.id].tire.physical_compound
                    ),
                    "best_lap_time": grid_entry.best_lap_time_s,
                    "gap": (
                        "—"
                        if grid_entry.position == 1
                        else f"+{grid_entry.gap_s:.3f}"
                    ),
                    "laps": [run.flying_lap_time_s for run in driver_result.runs],
                    "knockout": driver_result.eliminated_in
                    or driver_result.advanced_to,
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

    def run_qualifying(
        self,
        *,
        request: QualifyingRequest,
        circuit: Circuit,
        player_team: Team,
        drivers: list[Driver],
        teams: list[Team],
        track_conditions: Any,
        thermal_preset: Any,
        conditions_source: str,
    ) -> QualifyingResponse:
        snapshot = self.build_snapshot(
            session_id=(
                f"abstract-qualifying:{request.circuit_id}:"
                f"{request.player_team_id}:{request.session_seed}"
            ),
            session_seed=request.session_seed,
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            track_conditions=track_conditions,
        )
        result = AbstractRaceEngine(snapshot).run_qualifying(
            attempts_per_session=min(4, max(1, request.attempt_laps))
        )
        return self.qualifying_response(
            snapshot=snapshot,
            result=result,
            circuit=circuit,
            player_team=player_team,
            drivers=drivers,
            teams=teams,
            track_conditions=track_conditions,
            thermal_preset=thermal_preset,
            conditions_source=conditions_source,
            simulation_mode=request.simulation_mode,
        )

    @staticmethod
    def race_summary(
        result: Any,
        snapshot: AbstractSessionSnapshot,
        drivers: list[Driver],
        teams: list[Team],
    ) -> dict[str, Any]:
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
        authoritative = getattr(result, "classification", None)
        classification = (
            [item.to_dict() for item in authoritative]
            if authoritative is not None
            else [
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
        )
        return {
            "total_laps": result.total_laps,
            "grid": grid,
            "finish_order": list(result.finish_order),
            "classification": classification,
            "event_count": len(result.logical_events),
            "event_types": dict(
                sorted(
                    Counter(
                        event.event_type for event in result.logical_events
                    ).items()
                )
            ),
            "canonical_result_hash": result.canonical_result_hash,
            "source_mode": "abstract",
            "snapshot_hash": snapshot.snapshot_hash,
        }

    def broadcast_setup(
        self,
        *,
        snapshot: AbstractSessionSnapshot,
        total_laps: int,
        requested_grid: tuple[int | str, ...] | None,
        abstract_engine: AbstractRaceAuthority,
        drivers: list[Driver],
        teams: list[Team],
    ) -> tuple[AbstractSessionSnapshot, tuple[int | str, ...], dict[str, Any]]:
        qualifying = AbstractRaceEngine(snapshot).run_qualifying()
        grid_order = requested_grid or tuple(
            entry.driver_id for entry in qualifying.grid
        )
        resolved_snapshot = replace(snapshot, initial_grid_order=grid_order)
        driver_map = {driver.id: driver for driver in drivers}
        team_map = {team.id: team for team in teams}
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

    def compute_race(
        self,
        *,
        snapshot: AbstractSessionSnapshot,
        total_laps: int,
        requested_grid: tuple[int | str, ...] | None,
        abstract_engine: AbstractRaceAuthority,
        drivers: list[Driver],
        teams: list[Team],
    ) -> tuple[AbstractSessionSnapshot, Any, dict[str, Any]]:
        qualifying = AbstractRaceEngine(snapshot).run_qualifying()
        grid_order = requested_grid or tuple(
            entry.driver_id for entry in qualifying.grid
        )
        resolved_snapshot = replace(snapshot, initial_grid_order=grid_order)
        engine = AbstractRaceEngine(resolved_snapshot)
        if abstract_engine is AbstractRaceAuthority.PROGRESS_V5:
            result = engine.run_progress_race(total_laps=total_laps, tick_seconds=0.10)
        else:
            result = engine.run_race(total_laps=total_laps, tick_seconds=0.10)
        summary = self.race_summary(result, resolved_snapshot, drivers, teams)
        summary["abstract_engine"] = abstract_engine.value
        return resolved_snapshot, result, summary

    async def setup_race(
        self,
        *,
        request: RaceSetupRequest,
        circuit: Circuit,
        player_team: Team,
        drivers: list[Driver],
        teams: list[Team],
        circuits: list[Circuit],
        track_conditions: Any,
        thermal_preset: Any,
        conditions_source: str,
        session_manager: Any,
        broadcast_manager: Any,
    ) -> RaceSetupResponse:
        del circuits, session_manager
        session_id = (
            f"abstract-race:{request.circuit_id}:{request.player_team_id}:"
            f"{request.total_laps}:{request.session_seed}"
        )
        snapshot = self.build_snapshot(
            session_id=session_id,
            session_seed=request.session_seed,
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            track_conditions=track_conditions,
            starting_tires=request.starting_tires,
        )
        requested_grid = tuple(request.grid_order) if request.grid_order else None
        player_drivers = [
            driver for driver in drivers if driver.team_id == player_team.id
        ]
        if request.simulation_mode is SimulationMode.ABSTRACT_BROADCAST:
            snapshot, grid_order, summary = await asyncio.to_thread(
                self.broadcast_setup,
                snapshot=snapshot,
                total_laps=request.total_laps,
                requested_grid=requested_grid,
                abstract_engine=request.abstract_engine,
                drivers=drivers,
                teams=teams,
            )
            session = broadcast_manager.create_session(
                snapshot=snapshot,
                grid_order=grid_order,
                total_laps=request.total_laps,
                circuit=circuit.model_copy(update={"total_laps": request.total_laps}),
                player_team=player_team,
                player_drivers=player_drivers,
                drivers=drivers,
                teams=teams,
                track_conditions=track_conditions,
                thermal_preset=thermal_preset,
                conditions_source=conditions_source,
                authority_mode=(
                    "progress_v5"
                    if request.abstract_engine is AbstractRaceAuthority.PROGRESS_V5
                    else "stage4"
                ),
            )
            try:
                await session.start_loop()
                await session.wait_until_buffered()
            except Exception:
                await broadcast_manager.clear_async()
                raise
        else:
            snapshot, _result, summary = await asyncio.to_thread(
                self.compute_race,
                snapshot=snapshot,
                total_laps=request.total_laps,
                requested_grid=requested_grid,
                abstract_engine=request.abstract_engine,
                drivers=drivers,
                teams=teams,
            )
        return RaceSetupResponse(
            session_id=session_id,
            simulation_mode=request.simulation_mode,
            abstract_engine=request.abstract_engine,
            session_seed=request.session_seed,
            circuit=circuit.model_copy(update={"total_laps": request.total_laps}),
            player_team=player_team,
            player_drivers=player_drivers,
            grid_order=summary["grid"],
            track_conditions=track_conditions,
            thermal_preset=thermal_preset,
            track_conditions_source=conditions_source,
            tire_compound_nomination=circuit.tire_compound_nomination,
            abstract_result_summary=summary,
        )
