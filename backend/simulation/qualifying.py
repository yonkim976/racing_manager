"""Qualifying session simulation (Q1/Q2/Q3 knockout format)."""

from __future__ import annotations

import random

from models.schemas import (
    Circuit,
    Driver,
    QualifyingResponse,
    QualifyingResult,
    Team,
    TireCompound,
)
from simulation.car_performance import car_performance_factors
from simulation.physics import compute_effective_lap_time, driver_pace_multiplier
from simulation.tire_model import compute_tire_performance

# Each entry: (session name, number of drivers advancing to the next session).
# ``None`` marks the final session, where every remaining driver fights for pole.
SESSION_PLAN: list[tuple[str, int | None]] = [
    ("Q1", 15),
    ("Q2", 10),
    ("Q3", None),
]

# Track rubbers in across the hour, so each session is marginally faster.
TRACK_EVOLUTION: dict[str, float] = {
    "Q1": 1.0,
    "Q2": 0.997,
    "Q3": 0.994,
}


def _format_gap(lap_time: float, pole_time: float) -> str:
    gap = lap_time - pole_time
    if gap <= 0:
        return "POLE"
    return f"+{gap:.3f}"


def _normalize_to_reference_pole(
    circuit: Circuit,
    session_best: dict[int, dict[str, float]],
    session_laps: dict[int, dict[str, list[float]]],
    pole_driver_id: int,
) -> None:
    """Anchor the simulated dry Q3 field to an empirical pole reference.

    The performance model still determines every relative gap and knockout result.
    The circuit reference only corrects the absolute clock, preventing the legacy
    ``base_lap_time`` value from producing implausible qualifying records.
    """
    calibration = circuit.physics_calibration
    reference = (
        calibration.reference_lap_time_seconds
        if calibration is not None
        else None
    )
    raw_pole = session_best[pole_driver_id].get("Q3")
    if reference is None or raw_pole is None or raw_pole <= 0.0:
        return

    scale = reference / raw_pole
    for driver_id, sessions in session_best.items():
        for session_name, lap_time in sessions.items():
            session_best[driver_id][session_name] = round(lap_time * scale, 3)
        for session_name, laps in session_laps[driver_id].items():
            session_laps[driver_id][session_name] = [
                round(lap_time * scale, 3) for lap_time in laps
            ]


def _qualifying_lap_time(
    circuit: Circuit,
    driver: Driver,
    team: Team,
    rng: random.Random,
    evolution: float = 1.0,
) -> float:
    car_performance = car_performance_factors(team).qualifying
    driver_pace = driver_pace_multiplier(driver.stats.pace)
    tire_performance = compute_tire_performance(
        TireCompound.SOFT,
        0,
        rng.uniform(-0.0015, 0.0015),
    )

    consistency_loss = 1.0 - driver.stats.consistency
    lap_spread = 0.14 + consistency_loss * 0.58
    lap_random = rng.uniform(-lap_spread, lap_spread)

    return compute_effective_lap_time(
        circuit.base_lap_time * evolution,
        car_performance,
        driver_pace,
        tire_performance,
        rng,
        lap_random,
    )


def run_qualifying(
    circuit: Circuit,
    drivers: list[Driver],
    teams: dict[int, Team],
    player_team: Team,
    attempt_laps: int = 3,
    seed: int | None = None,
) -> QualifyingResponse:
    """Run a Q1/Q2/Q3 knockout qualifying simulation and return the starting grid."""
    rng = random.Random(seed)
    driver_by_id = {driver.id: driver for driver in drivers}

    session_best: dict[int, dict[str, float]] = {driver.id: {} for driver in drivers}
    session_laps: dict[int, dict[str, list[float]]] = {driver.id: {} for driver in drivers}
    knockout: dict[int, str] = {}

    pool: list[Driver] = list(drivers)
    final_block: list[int] = []
    eliminated_blocks: list[list[int]] = []  # in session order: [Q1 out, Q2 out, ...]

    for session_name, advance in SESSION_PLAN:
        evolution = TRACK_EVOLUTION.get(session_name, 1.0)
        ranked: list[tuple[float, Driver]] = []
        for driver in pool:
            team = teams[driver.team_id]
            laps = [
                round(_qualifying_lap_time(circuit, driver, team, rng, evolution), 3)
                for _ in range(attempt_laps)
            ]
            best = min(laps)
            session_best[driver.id][session_name] = best
            session_laps[driver.id][session_name] = laps
            ranked.append((best, driver))

        ranked.sort(key=lambda item: item[0])

        is_final = advance is None or advance >= len(pool)
        if is_final:
            for _, driver in ranked:
                knockout[driver.id] = session_name
            final_block = [driver.id for _, driver in ranked]
            break

        advancing = ranked[:advance]
        eliminated = ranked[advance:]
        for _, driver in eliminated:
            knockout[driver.id] = session_name
        eliminated_blocks.append([driver.id for _, driver in eliminated])
        pool = [driver for _, driver in advancing]

    # Grid: final session order on top, then later eliminations above earlier ones.
    grid_order: list[int] = list(final_block)
    for block in reversed(eliminated_blocks):
        grid_order.extend(block)

    if final_block:
        _normalize_to_reference_pole(
            circuit,
            session_best,
            session_laps,
            final_block[0],
        )
    pole_time = session_best[final_block[0]]["Q3"] if final_block else 0.0
    results: list[QualifyingResult] = []

    for position, driver_id in enumerate(grid_order, start=1):
        driver = driver_by_id[driver_id]
        team = teams[driver.team_id]
        reached = knockout[driver_id]
        best_lap_time = session_best[driver_id][reached]
        results.append(
            QualifyingResult(
                position=position,
                driver_id=driver_id,
                name=driver.abbreviation,
                full_name=driver.name,
                team=team.name,
                team_color=team.color,
                best_lap_time=best_lap_time,
                gap=_format_gap(best_lap_time, pole_time),
                laps=session_laps[driver_id][reached],
                knockout=reached,
                q1_time=session_best[driver_id].get("Q1"),
                q2_time=session_best[driver_id].get("Q2"),
                q3_time=session_best[driver_id].get("Q3"),
            )
        )

    return QualifyingResponse(
        circuit=circuit,
        player_team=player_team,
        results=results,
        grid_order=grid_order,
    )
