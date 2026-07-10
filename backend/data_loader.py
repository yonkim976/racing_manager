"""Load seed JSON data from backend/data/."""

from __future__ import annotations

import json
from pathlib import Path

from models.schemas import Circuit, Driver, DriverResponse, Team
from simulation.track_compiler import compile_circuit_layout

DATA_DIR = (Path(__file__).resolve().parent / "data").resolve()


def _load_json(filename: str) -> list | dict:
    with open(DATA_DIR / filename, encoding="utf-8") as f:
        return json.load(f)


def _load_source_fragment(filename: str) -> dict:
    path = (DATA_DIR / filename).resolve()
    if not path.is_relative_to(DATA_DIR):
        raise ValueError(f"Source data path must stay inside {DATA_DIR}: {filename}")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _pop_first(data: dict, *keys: str):
    for key in keys:
        if key in data:
            return data.pop(key)
    return None


def _prepare_circuit_seed(data: dict) -> dict:
    prepared = dict(data)
    geo_file = _pop_first(prepared, "geo_file", "geoFile")
    metric_file = _pop_first(prepared, "metric_file", "metricFile")
    if geo_file:
        prepared["geo"] = _load_source_fragment(geo_file)
    if metric_file:
        prepared["metric"] = _load_source_fragment(metric_file)
    return prepared


def load_drivers() -> list[Driver]:
    return [Driver.model_validate(d) for d in _load_json("drivers.json")]


def load_teams() -> list[Team]:
    return [Team.model_validate(t) for t in _load_json("teams.json")]


def load_circuits() -> list[Circuit]:
    return [
        compile_circuit_layout(Circuit.model_validate(_prepare_circuit_seed(c)))
        for c in _load_json("circuits.json")
    ]


def enrich_drivers(drivers: list[Driver], teams: list[Team]) -> list[DriverResponse]:
    """Attach team name and color to drivers."""
    team_map = {t.id: t for t in teams}
    result = []
    for driver in drivers:
        team = team_map.get(driver.team_id)
        result.append(
            DriverResponse(
                **driver.model_dump(),
                team_name=team.name if team else "",
                team_color=team.color if team else "#666",
            )
        )
    return result
