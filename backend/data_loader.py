"""Load seed JSON data from backend/data/."""

from __future__ import annotations

import json
from pathlib import Path

from models.schemas import (
    Circuit,
    CircuitTireWearProfile,
    DryTireRole,
    Driver,
    DriverResponse,
    PhysicalTireCompound,
    Team,
    TireCompound,
    TireCompoundNomination,
    ThermalPresetName,
    TrackConditions,
)
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


def _calibrated_track_width_profile(profile: dict) -> list[dict]:
    samples = profile.get("track_width_profile", [])
    calibration = profile.get("track_width_calibration")
    if not calibration:
        return samples
    if calibration.get("method") != "normalize_total_to_range_preserve_side_ratio":
        raise ValueError("unsupported track width calibration method")
    minimum = float(calibration["minimum_total_width_m"])
    maximum = float(calibration["maximum_total_width_m"])
    if minimum <= 0.0 or maximum < minimum:
        raise ValueError("invalid calibrated track width range")
    raw_totals = [
        float(sample["left_width_m"]) + float(sample["right_width_m"])
        for sample in samples
    ]
    if not raw_totals or min(raw_totals) <= 0.0:
        raise ValueError("track width samples must have positive total width")
    raw_minimum = min(raw_totals)
    raw_span = max(raw_totals) - raw_minimum
    calibrated = []
    for sample in samples:
        left = float(sample["left_width_m"])
        right = float(sample["right_width_m"])
        total = left + right
        source_ratio = (
            (total - raw_minimum) / raw_span
            if raw_span > 1e-9
            else 0.5
        )
        target_total = minimum + source_ratio * (maximum - minimum)
        scale = target_total / total
        calibrated.append(
            {
                **sample,
                "left_width_m": round(left * scale, 3),
                "right_width_m": round(right * scale, 3),
            }
        )
    return calibrated


def _prepare_circuit_seed(data: dict) -> dict:
    prepared = dict(data)
    geo_file = _pop_first(prepared, "geo_file", "geoFile")
    metric_file = _pop_first(prepared, "metric_file", "metricFile")
    profile_file = _pop_first(prepared, "profile_file", "profileFile")
    if geo_file:
        prepared["geo"] = _load_source_fragment(geo_file)
    if metric_file:
        prepared["metric"] = _load_source_fragment(metric_file)
    if profile_file:
        profile = _load_source_fragment(profile_file)
        for field in ("track_conditions", "surface_zones"):
            if field in profile:
                prepared[field] = profile[field]
        if "track_width_profile" in profile:
            prepared["track_width_profile"] = _calibrated_track_width_profile(profile)
        if "physics_calibration" in profile:
            prepared["physics_calibration"] = {
                **prepared.get("physics_calibration", {}),
                **profile["physics_calibration"],
            }
    return prepared


def load_drivers() -> list[Driver]:
    return [Driver.model_validate(d) for d in _load_json("drivers.json")]


def load_teams() -> list[Team]:
    return [Team.model_validate(t) for t in _load_json("teams.json")]


def load_circuits() -> list[Circuit]:
    circuit_seeds = _load_json("circuits.json")
    thermal_records = _load_json("circuit_thermal_profiles.json")
    circuit_ids = [seed.get("id") for seed in circuit_seeds]
    if len(circuit_ids) != len(set(circuit_ids)):
        raise ValueError("circuits.json contains duplicate circuit IDs")

    profiles_by_circuit_id: dict[int | str, dict] = {}
    for record in thermal_records:
        if not isinstance(record, dict) or "circuit_id" not in record:
            raise ValueError("thermal profile records must contain circuit_id")
        circuit_id = record["circuit_id"]
        if circuit_id in profiles_by_circuit_id:
            raise ValueError(f"duplicate thermal profile for circuit {circuit_id}")
        profile_data = dict(record)
        profile_data.pop("circuit_id")
        profiles_by_circuit_id[circuit_id] = profile_data

    circuit_id_set = set(circuit_ids)
    unknown_profile_ids = set(profiles_by_circuit_id) - circuit_id_set
    if unknown_profile_ids:
        raise ValueError(
            "thermal profiles reference unregistered circuit IDs: "
            + ", ".join(str(item) for item in sorted(unknown_profile_ids, key=str))
        )
    missing_profile_ids = circuit_id_set - set(profiles_by_circuit_id)
    if missing_profile_ids:
        raise ValueError(
            "missing thermal profiles for circuit IDs: "
            + ", ".join(str(item) for item in sorted(missing_profile_ids, key=str))
        )

    wear_profile_file = _load_json("circuit_tire_wear_profiles.json")
    if not isinstance(wear_profile_file, dict):
        raise ValueError("circuit tire wear profiles must be an object")
    if wear_profile_file.get("schema_version") != 1:
        raise ValueError("unsupported circuit tire wear profile schema version")
    reference_lap_distance_m = wear_profile_file.get("reference_lap_distance_m")
    if not isinstance(reference_lap_distance_m, (int, float)) or isinstance(
        reference_lap_distance_m,
        bool,
    ) or reference_lap_distance_m <= 0:
        raise ValueError("circuit tire wear profiles require a positive reference lap distance")
    wear_profile_records = wear_profile_file.get("circuits")
    if not isinstance(wear_profile_records, list):
        raise ValueError("circuit tire wear profiles require a circuits array")

    wear_profiles_by_circuit_id: dict[int | str, CircuitTireWearProfile] = {}
    for record in wear_profile_records:
        if not isinstance(record, dict) or "circuit_id" not in record:
            raise ValueError("circuit tire wear profile records must contain circuit_id")
        profile = CircuitTireWearProfile.model_validate(record)
        if profile.circuit_id in wear_profiles_by_circuit_id:
            raise ValueError(f"duplicate tire wear profile for circuit {profile.circuit_id}")
        wear_profiles_by_circuit_id[profile.circuit_id] = profile

    unknown_wear_profile_ids = set(wear_profiles_by_circuit_id) - circuit_id_set
    if unknown_wear_profile_ids:
        raise ValueError(
            "tire wear profiles reference unregistered circuit IDs: "
            + ", ".join(
                str(item)
                for item in sorted(unknown_wear_profile_ids, key=str)
            )
        )
    missing_wear_profile_ids = circuit_id_set - set(wear_profiles_by_circuit_id)
    if missing_wear_profile_ids:
        raise ValueError(
            "missing tire wear profiles for circuit IDs: "
            + ", ".join(
                str(item)
                for item in sorted(missing_wear_profile_ids, key=str)
            )
        )

    nomination_file = _load_json("tire_compound_nominations.json")
    if not isinstance(nomination_file, dict):
        raise ValueError("tire compound nominations must be an object")
    if nomination_file.get("schema_version") != 1:
        raise ValueError("unsupported tire compound nomination schema version")
    ruleset = nomination_file.get("ruleset")
    nomination_records = nomination_file.get("circuits")
    if not isinstance(ruleset, str) or not ruleset:
        raise ValueError("tire compound nominations require a non-empty ruleset")
    if not isinstance(nomination_records, list):
        raise ValueError("tire compound nominations require a circuits array")

    nominations_by_circuit_id: dict[int | str, TireCompoundNomination] = {}
    for record in nomination_records:
        if not isinstance(record, dict) or "circuit_id" not in record:
            raise ValueError("tire compound nomination records must contain circuit_id")
        circuit_id = record["circuit_id"]
        if circuit_id in nominations_by_circuit_id:
            raise ValueError(f"duplicate tire compound nomination for circuit {circuit_id}")
        nomination_data = dict(record)
        nomination_data["ruleset"] = ruleset
        nomination = TireCompoundNomination.model_validate(nomination_data)
        if nomination.circuit_id != circuit_id:
            raise ValueError(f"tire compound nomination circuit mismatch for {circuit_id}")
        nominations_by_circuit_id[circuit_id] = nomination

    unknown_nomination_ids = set(nominations_by_circuit_id) - set(circuit_ids)
    if unknown_nomination_ids:
        raise ValueError(
            "tire compound nominations reference unregistered circuit IDs: "
            + ", ".join(str(item) for item in sorted(unknown_nomination_ids, key=str))
        )
    missing_nomination_ids = set(circuit_ids) - set(nominations_by_circuit_id)
    if missing_nomination_ids:
        raise ValueError(
            "missing tire compound nominations for circuit IDs: "
            + ", ".join(str(item) for item in sorted(missing_nomination_ids, key=str))
        )

    circuits: list[Circuit] = []
    for seed in circuit_seeds:
        prepared = _prepare_circuit_seed(seed)
        prepared["thermal_profile"] = profiles_by_circuit_id[seed["id"]]
        prepared["tire_wear_profile"] = wear_profiles_by_circuit_id[seed["id"]]
        prepared["tire_compound_nomination"] = nominations_by_circuit_id[seed["id"]]
        circuits.append(
            compile_circuit_layout(Circuit.model_validate(prepared))
        )
    return circuits


def resolve_tire_role(value: DryTireRole | TireCompound | str) -> DryTireRole:
    """Normalize a weekend role at the API/session compatibility boundary."""

    if isinstance(value, DryTireRole):
        return value
    raw = value.value if isinstance(value, TireCompound) else str(value)
    try:
        return DryTireRole(raw.upper())
    except ValueError as exc:
        raise ValueError(f"{raw} is not a dry weekend tire role") from exc


def resolve_tire_compound(
    circuit: Circuit,
    role: DryTireRole | TireCompound | str,
) -> tuple[PhysicalTireCompound, DryTireRole]:
    """Resolve a weekend role to the circuit's immutable physical nomination."""

    nomination = circuit.tire_compound_nomination
    if nomination is None:
        raise ValueError(f"circuit {circuit.id} has no tire compound nomination")
    resolved_role = resolve_tire_role(role)
    return nomination.physical_for_role(resolved_role), resolved_role


def resolve_circuit_thermal_conditions(
    circuit: Circuit,
    thermal_preset: ThermalPresetName | None = None,
    track_conditions: TrackConditions | None = None,
) -> tuple[TrackConditions, ThermalPresetName | None, str]:
    """Resolve a setup choice once at the session/API boundary."""

    if thermal_preset is not None and track_conditions is not None:
        raise ValueError(
            "thermal_preset and track_conditions are mutually exclusive"
        )
    if track_conditions is not None:
        return track_conditions, None, "explicit_override"

    resolved_preset = thermal_preset or circuit.thermal_profile.default_preset
    try:
        conditions = circuit.thermal_profile.presets[resolved_preset]
    except KeyError as exc:
        raise ValueError(
            f"thermal preset {resolved_preset.value} is not configured for circuit {circuit.id}"
        ) from exc
    return conditions, resolved_preset, "circuit_preset"


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
