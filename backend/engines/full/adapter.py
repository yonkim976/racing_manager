"""API-neutral boundary around the legacy FULL physics runtime."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from data_loader import resolve_circuit_thermal_conditions, resolve_tire_compound
from models.schemas import (
    AbstractRaceAuthority,
    Circuit,
    Driver,
    QualifyingRequest,
    RaceSetupResponse,
    SimulationMode,
    Team,
    TireCompound,
)
from simulation.race_engine import RaceEngine, empty_tire_temperature_diagnostic_snapshot

from ..contracts import EngineCapabilities, EngineFamily
from .runtime.qualifying import run_qualifying


@dataclass(frozen=True, slots=True)
class FullRaceBuild:
    engine: RaceEngine
    circuit: Circuit
    player_team: Team
    player_drivers: tuple[Driver, ...]


class FullEngineAdapter:
    """Own construction of FULL qualifying and race implementations."""

    capabilities = EngineCapabilities(
        family=EngineFamily.FULL,
        supported_modes=frozenset({SimulationMode.FULL}),
        result_authority="50hz-vehicle-physics",
        presentation_authority="physics-pose",
        physics_telemetry=True,
        interactive_strategy=True,
    )

    def supports(self, mode: SimulationMode) -> bool:
        return mode in self.capabilities.supported_modes

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
    ):
        team_map = {team.id: team for team in teams}
        response = run_qualifying(
            circuit=circuit,
            drivers=drivers,
            teams=team_map,
            player_team=player_team,
            attempt_laps=request.attempt_laps,
            track_conditions=track_conditions,
            thermal_preset=thermal_preset,
            track_conditions_source=conditions_source,
            tire_compound_nomination=circuit.tire_compound_nomination,
        )
        return response.model_copy(
            update={
                "simulation_mode": SimulationMode.FULL,
                "session_seed": request.session_seed,
            }
        )

    def build_race(
        self,
        *,
        request,
        drivers: list[Driver],
        teams: list[Team],
        circuits: list[Circuit],
    ) -> FullRaceBuild:
        circuit = next((item for item in circuits if item.id == request.circuit_id), None)
        if circuit is None:
            raise ValueError(f"Circuit {request.circuit_id} not found")
        circuit = circuit.model_copy(update={"total_laps": request.total_laps})
        resolved_conditions, resolved_preset, conditions_source = (
            resolve_circuit_thermal_conditions(
                circuit,
                thermal_preset=request.thermal_preset,
                track_conditions=request.track_conditions,
            )
        )
        player_team = next(
            (team for team in teams if team.id == request.player_team_id), None
        )
        if player_team is None:
            raise ValueError(f"Team {request.player_team_id} not found")
        player_drivers = tuple(
            driver for driver in drivers if driver.team_id == player_team.id
        )
        if not player_drivers:
            raise ValueError("Player team has no drivers")
        player_driver_ids = {driver.id for driver in player_drivers}
        starting_tires = {
            driver_id: role
            for driver_id, role in request.starting_tires.items()
            if driver_id in player_driver_ids
        }
        starting_physical_tires = {
            driver.id: resolve_tire_compound(
                circuit,
                starting_tires.get(driver.id, TireCompound.MEDIUM),
            )[0]
            for driver in drivers
        }
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams={team.id: team for team in teams},
            player_team_id=player_team.id,
            player_driver_ids=list(player_driver_ids),
            starting_tires=starting_tires,
            starting_physical_tires=starting_physical_tires,
            grid_order=request.grid_order,
            track_conditions=resolved_conditions,
            thermal_preset=resolved_preset,
            track_conditions_source=conditions_source,
        )
        return FullRaceBuild(
            engine=engine,
            circuit=circuit,
            player_team=player_team,
            player_drivers=player_drivers,
        )

    async def setup_race(
        self,
        *,
        request,
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
        del circuit, player_team, track_conditions, thermal_preset, conditions_source
        del broadcast_manager
        session = session_manager.create_session(
            request, drivers, teams, circuits
        )
        await session.start_loop()
        return RaceSetupResponse(
            session_id=session.session_id,
            simulation_mode=SimulationMode.FULL,
            abstract_engine=AbstractRaceAuthority.STAGE4,
            session_seed=request.session_seed,
            circuit=session.circuit,
            player_team=session.player_team,
            player_drivers=session.player_drivers,
            grid_order=session.engine.get_grid_order(),
            track_conditions=session.engine.track_conditions,
            thermal_preset=session.resolved_thermal_preset,
            track_conditions_source=session.track_conditions_source,
            tire_compound_nomination=session.circuit.tire_compound_nomination,
        )
