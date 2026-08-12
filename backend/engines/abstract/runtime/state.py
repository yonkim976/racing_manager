"""Immutable snapshots and serializable Stage A result structures."""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

import hashlib
import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable, Mapping

from models.schemas import (
    Circuit,
    Driver,
    DryTireRole,
    PhysicalTireCompound,
    Team,
    ThermalPresetName,
    TrackConditions,
)

from .performance import (
    CALIBRATION_STATUS,
    SegmentRequirement,
    VehiclePerformance,
    build_segment_requirements,
)
from simulation.track_display import TrackDisplayGeometry, build_track_display_geometry


ABSTRACT_ENGINE_VERSION = "abstract-stage4-traffic-v4"
ABSTRACT_RESULT_CONTRACT_VERSION = "abstract-result-v2"


def _primitive(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if hasattr(value, "to_dict") and callable(value.to_dict):
        return value.to_dict()
    if isinstance(value, Mapping):
        return {
            str(key): _primitive(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (tuple, list)):
        return [_primitive(item) for item in value]
    if hasattr(value, "__dataclass_fields__"):
        return {
            field_name: _primitive(getattr(value, field_name))
            for field_name in value.__dataclass_fields__
        }
    return value


def canonical_json(value: Any) -> str:
    """Serialize JSON-compatible output with stable key and float formatting."""

    return json.dumps(
        _primitive(value),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def canonical_hash_sections(sections: tuple[tuple[str, Any], ...]) -> str:
    """Hash canonical result sections incrementally and in explicit order.

    The length prefix prevents adjacent sections from becoming ambiguous while
    allowing the result hash to be assembled without first joining one large
    canonical payload string.  Section names are part of the contract and are
    never derived from mapping/set iteration order.
    """

    digest = hashlib.sha256(b"abstract-result-hash-v2\0")
    for name, value in sections:
        name_bytes = name.encode("utf-8")
        value_bytes = canonical_json(value).encode("utf-8")
        digest.update(len(name_bytes).to_bytes(4, "big"))
        digest.update(name_bytes)
        digest.update(len(value_bytes).to_bytes(8, "big"))
        digest.update(value_bytes)
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class SectorRequirement:
    sector_id: str
    name: str
    start_progress: float
    end_progress: float
    mini_sector_count: int = 1

    def to_dict(self) -> dict[str, Any]:
        return {
            "sector_id": self.sector_id,
            "name": self.name,
            "start_progress": self.start_progress,
            "end_progress": self.end_progress,
            "mini_sector_count": self.mini_sector_count,
        }


@dataclass(frozen=True, slots=True)
class DRSZoneRequirement:
    """Immutable DRS detection and activation range for ABSTRACT authority."""

    name: str
    detection_progress: float
    start_progress: float
    end_progress: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "detection_progress": self.detection_progress,
            "start_progress": self.start_progress,
            "end_progress": self.end_progress,
        }


@dataclass(frozen=True, slots=True)
class AbstractTrackSnapshot:
    circuit_id: int | str
    name: str
    track_length_m: float
    base_lap_time_s: float
    sectors: tuple[SectorRequirement, ...]
    segments: tuple[SegmentRequirement, ...]
    content_version: str
    drs_zones: tuple[DRSZoneRequirement, ...] = ()
    calibration_status: str = CALIBRATION_STATUS
    centerline_points: tuple[tuple[float, float], ...] = ()
    track_width_m: float = 12.0
    overtaking_difficulty: float = 0.5
    display_geometry: TrackDisplayGeometry | None = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.track_length_m <= 0.0 or self.base_lap_time_s <= 0.0:
            raise ValueError("track length and base lap time must be positive")
        if self.track_width_m <= 0.0 or not 0.0 <= self.overtaking_difficulty <= 1.0:
            raise ValueError("track width and overtaking difficulty are invalid")
        if not self.sectors or not self.segments:
            raise ValueError("track snapshot requires sectors and segments")
        normalized_points = tuple(
            (round(float(point[0]), 9), round(float(point[1]), 9))
            for point in self.centerline_points
        )
        if normalized_points and len(normalized_points) < 3:
            raise ValueError("centerline requires at least three points")
        object.__setattr__(self, "centerline_points", normalized_points)
        object.__setattr__(
            self,
            "sectors",
            tuple(sorted(self.sectors, key=lambda item: (item.start_progress, item.sector_id))),
        )
        object.__setattr__(
            self,
            "segments",
            tuple(sorted(self.segments, key=lambda item: (item.start_progress, item.segment_id))),
        )
        object.__setattr__(
            self,
            "drs_zones",
            tuple(
                sorted(
                    self.drs_zones,
                    key=lambda item: (item.start_progress, item.end_progress, item.name),
                )
            ),
        )

    @classmethod
    def from_circuit(
        cls,
        circuit: Circuit,
        content_version: str | None = None,
        grid_driver_ids: Iterable[int | str] = (),
    ) -> "AbstractTrackSnapshot":
        sector_models = sorted(
            circuit.sectors,
            key=lambda sector: (sector.start if sector.start is not None else 0.0, sector.name),
        )
        if sector_models:
            sectors = tuple(
                SectorRequirement(
                    sector_id=f"S{index}",
                    name=sector.name,
                    start_progress=sector.start if sector.start is not None else (index - 1) / len(sector_models),
                    end_progress=sector.end if sector.end is not None else index / len(sector_models),
                    mini_sector_count=sector.mini_sector_count,
                )
                for index, sector in enumerate(sector_models, start=1)
            )
        else:
            sectors = tuple(
                SectorRequirement(
                    sector_id=f"S{index}",
                    name=f"Sector {index}",
                    start_progress=(index - 1) / 3.0,
                    end_progress=index / 3.0,
                    mini_sector_count=1,
                )
                for index in range(1, 4)
            )
        centerline_points = []
        for point in circuit.track_coords:
            if hasattr(point, "x") and hasattr(point, "y"):
                centerline_points.append((float(point.x), float(point.y)))
            else:
                centerline_points.append((float(point[0]), float(point[1])))
        return cls(
            circuit_id=circuit.id,
            name=circuit.name,
            track_length_m=circuit.track_length_m,
            base_lap_time_s=circuit.base_lap_time,
            sectors=sectors,
            segments=build_segment_requirements(circuit),
            content_version=content_version or circuit.data_version or "content-unknown",
            drs_zones=tuple(
                DRSZoneRequirement(
                    name=zone.name,
                    detection_progress=(
                        float(zone.detection) % 1.0
                        if zone.detection is not None
                        else (
                            float(zone.start)
                            - 200.0 / max(circuit.track_length_m, 1.0)
                        ) % 1.0
                    ),
                    start_progress=float(zone.start) % 1.0,
                    end_progress=float(zone.end) % 1.0,
                )
                for zone in circuit.drs_zones
            ),
            centerline_points=tuple(centerline_points),
            track_width_m=circuit.track_width_m,
            overtaking_difficulty=circuit.overtaking_difficulty,
            display_geometry=build_track_display_geometry(
                circuit,
                grid_driver_ids=grid_driver_ids,
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "circuit_id": self.circuit_id,
            "name": self.name,
            "track_length_m": self.track_length_m,
            "base_lap_time_s": self.base_lap_time_s,
            "sectors": [sector.to_dict() for sector in self.sectors],
            "segments": [segment.to_dict() for segment in self.segments],
            "drs_zones": [zone.to_dict() for zone in self.drs_zones],
            "content_version": self.content_version,
            "calibration_status": self.calibration_status,
            "centerline_points": [list(point) for point in self.centerline_points],
            "track_width_m": self.track_width_m,
            "overtaking_difficulty": self.overtaking_difficulty,
        }


@dataclass(frozen=True, slots=True)
class AbstractDriverProfile:
    driver_id: int | str
    name: str
    abbreviation: str
    team_id: int | str
    pace: float
    consistency: float
    tire_management: float
    overtaking: float
    defending: float

    @classmethod
    def from_driver(cls, driver: Driver) -> "AbstractDriverProfile":
        return cls(
            driver_id=driver.id,
            name=driver.name,
            abbreviation=driver.abbreviation,
            team_id=driver.team_id,
            pace=driver.stats.pace,
            consistency=driver.stats.consistency,
            tire_management=driver.stats.tire_management,
            overtaking=driver.stats.overtaking,
            defending=driver.stats.defending,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver_id": self.driver_id,
            "name": self.name,
            "abbreviation": self.abbreviation,
            "team_id": self.team_id,
            "pace": self.pace,
            "consistency": self.consistency,
            "tire_management": self.tire_management,
            "overtaking": self.overtaking,
            "defending": self.defending,
        }


@dataclass(frozen=True, slots=True)
class AbstractVehicleSnapshot:
    vehicle_id: str
    team_id: int | str
    team_name: str
    performance: VehiclePerformance

    @classmethod
    def from_team(cls, team: Team, driver_id: int | str) -> "AbstractVehicleSnapshot":
        vehicle_id = f"vehicle:{team.id}:{driver_id}"
        return cls(
            vehicle_id=vehicle_id,
            team_id=team.id,
            team_name=team.name,
            performance=VehiclePerformance.from_team(team, vehicle_id),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "team_id": self.team_id,
            "team_name": self.team_name,
            "performance": self.performance.to_dict(),
        }


@dataclass(frozen=True, slots=True)
class TireConditionSnapshot:
    physical_compound: str = PhysicalTireCompound.C3.value
    tire_role: str = DryTireRole.SOFT.value
    wear_laps: float = 0.0
    temperature_band: str = "optimal"

    def __post_init__(self) -> None:
        if self.wear_laps < 0.0:
            raise ValueError("wear_laps must not be negative")
        if self.temperature_band not in {"cold", "optimal", "hot"}:
            raise ValueError("temperature_band must be cold, optimal or hot")

    def to_dict(self) -> dict[str, Any]:
        return {
            "physical_compound": self.physical_compound,
            "tire_role": self.tire_role,
            "wear_laps": self.wear_laps,
            "temperature_band": self.temperature_band,
        }


@dataclass(frozen=True, slots=True)
class AbstractEnvironmentSnapshot:
    weather: str = "dry"
    ambient_temperature_c: float = 30.0
    track_temperature_c: float = 40.0
    initial_track_evolution: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "weather": self.weather,
            "ambient_temperature_c": self.ambient_temperature_c,
            "track_temperature_c": self.track_temperature_c,
            "initial_track_evolution": self.initial_track_evolution,
        }


@dataclass(frozen=True, slots=True)
class AbstractEntrySnapshot:
    driver: AbstractDriverProfile
    vehicle: AbstractVehicleSnapshot
    tire: TireConditionSnapshot

    @property
    def driver_id(self) -> int | str:
        return self.driver.driver_id

    @property
    def vehicle_id(self) -> str:
        return self.vehicle.vehicle_id

    @property
    def team_id(self) -> int | str:
        return self.vehicle.team_id

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver": self.driver.to_dict(),
            "vehicle": self.vehicle.to_dict(),
            "tire": self.tire.to_dict(),
        }


@dataclass(frozen=True, slots=True)
class AbstractSessionSnapshot:
    """Immutable input contract frozen before a Stage A run starts."""

    session_id: str
    session_seed: int | str
    content_version: str
    ruleset_version: str
    abstract_engine_version: str
    track: AbstractTrackSnapshot
    entries: tuple[AbstractEntrySnapshot, ...]
    environment: AbstractEnvironmentSnapshot
    initial_grid_order: tuple[int | str, ...] = ()
    qualifying_tire_role: str = DryTireRole.SOFT.value

    def __post_init__(self) -> None:
        if not self.session_id or not self.content_version or not self.ruleset_version:
            raise ValueError("session and content versions must be non-empty")
        if not self.entries:
            raise ValueError("session snapshot requires at least one entry")
        entries = tuple(sorted(self.entries, key=lambda entry: (str(entry.driver_id), entry.vehicle_id)))
        driver_ids = [entry.driver_id for entry in entries]
        vehicle_ids = [entry.vehicle_id for entry in entries]
        if len(driver_ids) != len(set(driver_ids)):
            raise ValueError("session snapshot contains duplicate driver IDs")
        if len(vehicle_ids) != len(set(vehicle_ids)):
            raise ValueError("session snapshot contains duplicate vehicle IDs")
        object.__setattr__(self, "entries", entries)
        grid = self.initial_grid_order or tuple(entry.driver_id for entry in entries)
        if set(grid) != set(driver_ids) or len(grid) != len(driver_ids):
            raise ValueError("initial_grid_order must contain every driver exactly once")
        object.__setattr__(self, "initial_grid_order", tuple(grid))

    @classmethod
    def from_content(
        cls,
        *,
        session_id: str,
        session_seed: int | str,
        circuit: Circuit,
        drivers: list[Driver] | tuple[Driver, ...],
        teams: list[Team] | tuple[Team, ...] | Mapping[int | str, Team],
        content_version: str | None = None,
        ruleset_version: str = "2026_C1_C5",
        abstract_engine_version: str = ABSTRACT_ENGINE_VERSION,
        track_conditions: TrackConditions | None = None,
        weather: str = "dry",
        tire_role: DryTireRole = DryTireRole.SOFT,
        tire_compound: PhysicalTireCompound | str | None = None,
    ) -> "AbstractSessionSnapshot":
        team_map = teams if isinstance(teams, Mapping) else {team.id: team for team in teams}
        resolved_compound = tire_compound
        if resolved_compound is None and circuit.tire_compound_nomination is not None:
            resolved_compound = circuit.tire_compound_nomination.physical_for_role(tire_role)
        if resolved_compound is None:
            resolved_compound = PhysicalTireCompound.C3
        resolved_compound_value = getattr(resolved_compound, "value", str(resolved_compound))
        conditions = track_conditions or TrackConditions()
        entries = []
        for driver in sorted(drivers, key=lambda item: str(item.id)):
            team = team_map.get(driver.team_id)
            if team is None:
                raise ValueError(f"driver {driver.id} references missing team {driver.team_id}")
            entries.append(
                AbstractEntrySnapshot(
                    driver=AbstractDriverProfile.from_driver(driver),
                    vehicle=AbstractVehicleSnapshot.from_team(team, driver.id),
                    tire=TireConditionSnapshot(
                        physical_compound=resolved_compound_value,
                        tire_role=tire_role.value,
                    ),
                )
            )
        return cls(
            session_id=session_id,
            session_seed=session_seed,
            content_version=content_version or circuit.data_version or "content-unknown",
            ruleset_version=ruleset_version,
            abstract_engine_version=abstract_engine_version,
            track=AbstractTrackSnapshot.from_circuit(
                circuit,
                content_version,
                grid_driver_ids=(entry.driver_id for entry in entries),
            ),
            entries=tuple(entries),
            environment=AbstractEnvironmentSnapshot(
                weather=weather,
                ambient_temperature_c=conditions.ambient_temperature_c,
                track_temperature_c=conditions.track_temperature_c,
            ),
            qualifying_tire_role=tire_role.value,
        )

    def entry_by_driver_id(self) -> dict[int | str, AbstractEntrySnapshot]:
        return {entry.driver_id: entry for entry in self.entries}

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "session_seed": self.session_seed,
            "content_version": self.content_version,
            "ruleset_version": self.ruleset_version,
            "abstract_engine_version": self.abstract_engine_version,
            "track": self.track.to_dict(),
            "entries": [entry.to_dict() for entry in self.entries],
            "environment": self.environment.to_dict(),
            "initial_grid_order": list(self.initial_grid_order),
            "qualifying_tire_role": self.qualifying_tire_role,
        }

    @property
    def snapshot_hash(self) -> str:
        return canonical_hash(self.to_dict())

    def canonical_json(self) -> str:
        return canonical_json(self.to_dict())


@dataclass(frozen=True, slots=True)
class LogicalEvent:
    event_id: str
    event_type: str
    logical_time_s: float
    driver_ids: tuple[int | str, ...] = ()
    payload: tuple[tuple[str, Any], ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "driver_ids", tuple(sorted(self.driver_ids, key=str)))
        object.__setattr__(self, "payload", tuple(sorted(self.payload, key=lambda item: item[0])))

    def to_dict(self) -> dict[str, Any]:
        return {
            "event_id": self.event_id,
            "event_type": self.event_type,
            "logical_time_s": round(self.logical_time_s, 9),
            "driver_ids": list(self.driver_ids),
            "payload": {key: _primitive(value) for key, value in self.payload},
        }


@dataclass(frozen=True, slots=True)
class AbstractCommandRecord:
    """Stable ordered command-log record included in the result contract."""

    sequence: int
    command: str
    logical_time_s: float
    payload: Any = None

    def __post_init__(self) -> None:
        if self.sequence < 0:
            raise ValueError("command sequence must not be negative")
        if not self.command:
            raise ValueError("command name must not be empty")

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "command": self.command,
            "logical_time_s": round(self.logical_time_s, 9),
            "payload": _primitive(self.payload),
        }


@dataclass(frozen=True, slots=True)
class QualifyingRunResult:
    run_id: str
    session_name: str
    driver_id: int | str
    vehicle_id: str
    run_number: int
    run_departure_s: float
    out_lap_time_s: float
    flying_lap_start_s: float
    sector_times_s: tuple[float, ...]
    flying_lap_time_s: float
    flying_lap_end_s: float
    traffic_penalty_s: float
    track_evolution: float
    mistake_loss_s: float
    mistakes: tuple[str, ...] = ()
    valid: bool = True
    tire: TireConditionSnapshot = TireConditionSnapshot()

    def to_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "session_name": self.session_name,
            "driver_id": self.driver_id,
            "vehicle_id": self.vehicle_id,
            "run_number": self.run_number,
            "run_departure_s": round(self.run_departure_s, 9),
            "out_lap_time_s": round(self.out_lap_time_s, 9),
            "flying_lap_start_s": round(self.flying_lap_start_s, 9),
            "sector_times_s": [round(value, 9) for value in self.sector_times_s],
            "flying_lap_time_s": round(self.flying_lap_time_s, 9),
            "flying_lap_end_s": round(self.flying_lap_end_s, 9),
            "traffic_penalty_s": round(self.traffic_penalty_s, 9),
            "track_evolution": round(self.track_evolution, 9),
            "mistake_loss_s": round(self.mistake_loss_s, 9),
            "mistakes": list(self.mistakes),
            "valid": self.valid,
            "tire": self.tire.to_dict(),
        }


@dataclass(frozen=True, slots=True)
class DriverQualifyingResult:
    driver_id: int | str
    vehicle_id: str
    best_lap_time_s: float
    best_sector_times_s: tuple[float, ...]
    best_run_id: str
    runs: tuple[QualifyingRunResult, ...]
    sessions_entered: tuple[str, ...]
    advanced_to: str
    eliminated_in: str | None
    grid_position: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver_id": self.driver_id,
            "vehicle_id": self.vehicle_id,
            "best_lap_time_s": round(self.best_lap_time_s, 9),
            "best_sector_times_s": [round(value, 9) for value in self.best_sector_times_s],
            "best_run_id": self.best_run_id,
            "runs": [run.to_dict() for run in self.runs],
            "sessions_entered": list(self.sessions_entered),
            "advanced_to": self.advanced_to,
            "eliminated_in": self.eliminated_in,
            "grid_position": self.grid_position,
        }


@dataclass(frozen=True, slots=True)
class QualifyingSessionSummary:
    session_name: str
    eligible_driver_ids: tuple[int | str, ...]
    advanced_driver_ids: tuple[int | str, ...]
    eliminated_driver_ids: tuple[int | str, ...]
    track_evolution: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "eligible_driver_ids", tuple(sorted(self.eligible_driver_ids, key=str)))
        object.__setattr__(self, "advanced_driver_ids", tuple(sorted(self.advanced_driver_ids, key=str)))
        object.__setattr__(self, "eliminated_driver_ids", tuple(sorted(self.eliminated_driver_ids, key=str)))

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_name": self.session_name,
            "eligible_driver_ids": list(self.eligible_driver_ids),
            "advanced_driver_ids": list(self.advanced_driver_ids),
            "eliminated_driver_ids": list(self.eliminated_driver_ids),
            "track_evolution": round(self.track_evolution, 9),
        }


@dataclass(frozen=True, slots=True)
class GridEntry:
    position: int
    driver_id: int | str
    vehicle_id: str
    best_lap_time_s: float
    gap_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position,
            "driver_id": self.driver_id,
            "vehicle_id": self.vehicle_id,
            "best_lap_time_s": round(self.best_lap_time_s, 9),
            "gap_s": round(self.gap_s, 9),
        }


@dataclass(frozen=True, slots=True)
class AbstractQualifyingResult:
    session_id: str
    session_seed: int | str
    content_version: str
    ruleset_version: str
    abstract_engine_version: str
    driver_results: tuple[DriverQualifyingResult, ...]
    sessions: tuple[QualifyingSessionSummary, ...]
    grid: tuple[GridEntry, ...]
    logical_events: tuple[LogicalEvent, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "driver_results", tuple(sorted(self.driver_results, key=lambda item: str(item.driver_id))))
        object.__setattr__(self, "sessions", tuple(sorted(self.sessions, key=lambda item: item.session_name)))
        object.__setattr__(self, "grid", tuple(sorted(self.grid, key=lambda item: item.position)))
        object.__setattr__(
            self,
            "logical_events",
            tuple(
                sorted(
                    self.logical_events,
                    key=lambda item: (round(item.logical_time_s, 9), item.event_id),
                )
            ),
        )

    def canonical_payload(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "session_seed": self.session_seed,
            "content_version": self.content_version,
            "ruleset_version": self.ruleset_version,
            "abstract_engine_version": self.abstract_engine_version,
            "driver_results": [item.to_dict() for item in self.driver_results],
            "sessions": [item.to_dict() for item in self.sessions],
            "grid": [item.to_dict() for item in self.grid],
            "logical_events": [item.to_dict() for item in self.logical_events],
        }

    @property
    def canonical_result_hash(self) -> str:
        return canonical_hash(self.canonical_payload())

    @property
    def result_hash(self) -> str:
        return self.canonical_result_hash

    def to_dict(self) -> dict[str, Any]:
        return {
            **self.canonical_payload(),
            "canonical_result_hash": self.canonical_result_hash,
        }

    def canonical_json(self) -> str:
        """Canonical output JSON; the hash excludes its own field."""

        return canonical_json(self.to_dict())

    def to_json(self) -> str:
        return self.canonical_json()

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AbstractQualifyingResult":
        def tire(raw: Mapping[str, Any]) -> TireConditionSnapshot:
            return TireConditionSnapshot(**raw)

        def run(raw: Mapping[str, Any]) -> QualifyingRunResult:
            return QualifyingRunResult(
                run_id=raw["run_id"],
                session_name=raw["session_name"],
                driver_id=raw["driver_id"],
                vehicle_id=raw["vehicle_id"],
                run_number=raw["run_number"],
                run_departure_s=raw["run_departure_s"],
                out_lap_time_s=raw["out_lap_time_s"],
                flying_lap_start_s=raw["flying_lap_start_s"],
                sector_times_s=tuple(raw["sector_times_s"]),
                flying_lap_time_s=raw["flying_lap_time_s"],
                flying_lap_end_s=raw["flying_lap_end_s"],
                traffic_penalty_s=raw["traffic_penalty_s"],
                track_evolution=raw["track_evolution"],
                mistake_loss_s=raw["mistake_loss_s"],
                mistakes=tuple(raw.get("mistakes", ())),
                valid=raw.get("valid", True),
                tire=tire(raw.get("tire", {})),
            )

        drivers = tuple(
            DriverQualifyingResult(
                driver_id=raw["driver_id"],
                vehicle_id=raw["vehicle_id"],
                best_lap_time_s=raw["best_lap_time_s"],
                best_sector_times_s=tuple(raw["best_sector_times_s"]),
                best_run_id=raw["best_run_id"],
                runs=tuple(run(item) for item in raw["runs"]),
                sessions_entered=tuple(raw["sessions_entered"]),
                advanced_to=raw["advanced_to"],
                eliminated_in=raw.get("eliminated_in"),
                grid_position=raw["grid_position"],
            )
            for raw in data["driver_results"]
        )
        sessions = tuple(
            QualifyingSessionSummary(
                session_name=raw["session_name"],
                eligible_driver_ids=tuple(raw["eligible_driver_ids"]),
                advanced_driver_ids=tuple(raw["advanced_driver_ids"]),
                eliminated_driver_ids=tuple(raw["eliminated_driver_ids"]),
                track_evolution=raw["track_evolution"],
            )
            for raw in data["sessions"]
        )
        grid = tuple(GridEntry(**raw) for raw in data["grid"])
        events = tuple(
            LogicalEvent(
                event_id=raw["event_id"],
                event_type=raw["event_type"],
                logical_time_s=raw["logical_time_s"],
                driver_ids=tuple(raw.get("driver_ids", ())),
                payload=tuple(sorted(raw.get("payload", {}).items())),
            )
            for raw in data["logical_events"]
        )
        return cls(
            session_id=data["session_id"],
            session_seed=data["session_seed"],
            content_version=data["content_version"],
            ruleset_version=data["ruleset_version"],
            abstract_engine_version=data["abstract_engine_version"],
            driver_results=drivers,
            sessions=sessions,
            grid=grid,
            logical_events=events,
        )


@dataclass(frozen=True, slots=True)
class RaceVehicleState:
    """One vehicle's logical and minimal display state at a race tick."""

    driver_id: int | str
    vehicle_id: str
    position: int
    total_progress: float
    progress: float
    lap_number: int
    sector_id: str
    progress_rate_per_s: float
    gap_to_ahead_m: float
    interval_to_ahead_s: float
    ahead_driver_id: int | str | None
    lateral_offset_m: float
    visual_state: str
    maneuver: str
    corridor_id: str | None = None
    corridor_side: str | None = None
    world_x_m: float = 0.0
    world_y_m: float = 0.0
    heading_rad: float = 0.0
    source_mode: str = "abstract"
    gap_to_leader_s: float = 0.0
    timing_gap_valid: bool = False
    interval_timing_gap_valid: bool = False
    drs_active: bool = False
    dirty_air_active: bool = False
    dirty_air_strength: float = 0.0
    tow_strength: float = 0.0
    attack_mode: str = "none"
    maneuver_side: int = 0
    maneuver_progress: float = 0.0
    drs_train_id: str | None = None
    drs_train_size: int = 0
    drs_train_position: int | None = None
    drs_train_member_ids: tuple[int | str, ...] = ()
    maneuver_group_id: str | None = None
    maneuver_group_size: int = 0
    maneuver_group_member_ids: tuple[int | str, ...] = ()
    maneuver_group_phase: str | None = None
    maneuver_group_corridor_index: int | None = None
    physical_compound: str = PhysicalTireCompound.C3.value
    tire_role: str = DryTireRole.SOFT.value
    stint_lap: int = 0
    wear_laps: float = 0.0
    temperature_band: str = "optimal"
    pit_state: str = "none"
    pit_lane_progress: float = 0.0
    pit_stop_count: int = 0
    pit_request_pending: bool = False
    pit_request_role: str | None = None
    pit_request_compound: str | None = None
    pace_mode: str = "STANDARD"
    damage_level: float = 0.0
    incident_state: str | None = None
    reliability_state: str = "nominal"
    # Stage 4 accepted presentation state.  These are kinematic display
    # values only; no force, tire-temperature or slip fields are implied.
    line_distance_m: float = 0.0
    speed_mps: float = 0.0
    longitudinal_acceleration_mps2: float = 0.0
    lateral_velocity_mps: float = 0.0
    lateral_acceleration_mps2: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver_id": self.driver_id,
            "vehicle_id": self.vehicle_id,
            "position": self.position,
            "total_progress": round(self.total_progress, 9),
            "progress": round(self.progress, 9),
            "lap_number": self.lap_number,
            "sector_id": self.sector_id,
            "progress_rate_per_s": round(self.progress_rate_per_s, 9),
            "gap_to_ahead_m": round(self.gap_to_ahead_m, 9),
            "interval_to_ahead_s": round(self.interval_to_ahead_s, 9),
            "ahead_driver_id": self.ahead_driver_id,
            "lateral_offset_m": round(self.lateral_offset_m, 9),
            "visual_state": self.visual_state,
            "maneuver": self.maneuver,
            "corridor_id": self.corridor_id,
            "corridor_side": self.corridor_side,
            "world_x_m": round(self.world_x_m, 9),
            "world_y_m": round(self.world_y_m, 9),
            "heading_rad": round(self.heading_rad, 9),
            "source_mode": self.source_mode,
            "gap_to_leader_s": round(self.gap_to_leader_s, 9),
            "timing_gap_valid": self.timing_gap_valid,
            "interval_timing_gap_valid": self.interval_timing_gap_valid,
            "drs_active": self.drs_active,
            "dirty_air_active": self.dirty_air_active,
            "dirty_air_strength": round(self.dirty_air_strength, 9),
            "tow_strength": round(self.tow_strength, 9),
            "attack_mode": self.attack_mode,
            "maneuver_side": self.maneuver_side,
            "maneuver_progress": round(self.maneuver_progress, 9),
            "drs_train_id": self.drs_train_id,
            "drs_train_size": self.drs_train_size,
            "drs_train_position": self.drs_train_position,
            "drs_train_member_ids": list(self.drs_train_member_ids),
            "maneuver_group_id": self.maneuver_group_id,
            "maneuver_group_size": self.maneuver_group_size,
            "maneuver_group_member_ids": list(self.maneuver_group_member_ids),
            "maneuver_group_phase": self.maneuver_group_phase,
            "maneuver_group_corridor_index": self.maneuver_group_corridor_index,
            "physical_compound": self.physical_compound,
            "tire_role": self.tire_role,
            "stint_lap": self.stint_lap,
            "wear_laps": round(self.wear_laps, 9),
            "temperature_band": self.temperature_band,
            "pit_state": self.pit_state,
            "pit_lane_progress": round(self.pit_lane_progress, 9),
            "pit_stop_count": self.pit_stop_count,
            "pit_request_pending": self.pit_request_pending,
            "pit_request_role": self.pit_request_role,
            "pit_request_compound": self.pit_request_compound,
            "pace_mode": self.pace_mode,
            "damage_level": round(self.damage_level, 9),
            "incident_state": self.incident_state,
            "reliability_state": self.reliability_state,
            "line_distance_m": round(self.line_distance_m, 9),
            "speed_mps": round(self.speed_mps, 9),
            "longitudinal_acceleration_mps2": round(self.longitudinal_acceleration_mps2, 9),
            "lateral_velocity_mps": round(self.lateral_velocity_mps, 9),
            "lateral_acceleration_mps2": round(self.lateral_acceleration_mps2, 9),
        }


@dataclass(frozen=True, slots=True)
class AbstractRaceFrame:
    tick_index: int
    logical_time_s: float
    vehicles: tuple[RaceVehicleState, ...]

    def __post_init__(self) -> None:
        vehicles = tuple(sorted(self.vehicles, key=lambda item: item.position))
        positions = tuple(vehicle.position for vehicle in vehicles)
        driver_ids = tuple(vehicle.driver_id for vehicle in vehicles)
        if positions != tuple(range(1, len(vehicles) + 1)):
            raise ValueError("race frame positions must be unique and contiguous")
        if len(driver_ids) != len(set(driver_ids)):
            raise ValueError("race frame contains duplicate drivers")
        object.__setattr__(
            self,
            "vehicles",
            vehicles,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "tick_index": self.tick_index,
            "logical_time_s": round(self.logical_time_s, 9),
            "vehicles": [vehicle.to_dict() for vehicle in self.vehicles],
        }


@dataclass(frozen=True, slots=True)
class AbstractTimingCheckpoint:
    """Compact logical checkpoint retained by the authoritative result.

    This is timing/ranking data only.  It intentionally contains no world
    coordinates, heading, force, slip or per-tick vehicle pose.
    """

    checkpoint_id: str
    checkpoint_kind: str
    tick_index: int
    logical_time_s: float
    lap_number: int
    order: tuple[int | str, ...]
    progress_by_driver: tuple[tuple[int | str, float], ...]

    def __post_init__(self) -> None:
        order = tuple(self.order)
        if len(order) != len(set(order)):
            raise ValueError("timing checkpoint contains duplicate drivers")
        progress = tuple(
            sorted(
                ((driver_id, float(value)) for driver_id, value in self.progress_by_driver),
                key=lambda item: str(item[0]),
            )
        )
        if tuple(driver_id for driver_id, _ in progress) != tuple(sorted(order, key=str)):
            raise ValueError("timing checkpoint progress must contain every ordered driver")
        object.__setattr__(self, "order", order)
        object.__setattr__(self, "progress_by_driver", progress)

    def to_dict(self) -> dict[str, Any]:
        return {
            "checkpoint_id": self.checkpoint_id,
            "checkpoint_kind": self.checkpoint_kind,
            "tick_index": self.tick_index,
            "logical_time_s": round(self.logical_time_s, 9),
            "lap_number": self.lap_number,
            "order": list(self.order),
            "progress_by_driver": [
                {"driver_id": driver_id, "progress": round(progress, 9)}
                for driver_id, progress in self.progress_by_driver
            ],
        }


@dataclass(frozen=True, slots=True)
class AbstractRaceResult:
    """Immutable authoritative result, without an unbounded pose replay."""

    session_id: str
    session_seed: int | str
    content_version: str
    ruleset_version: str
    abstract_engine_version: str
    snapshot_hash: str
    total_laps: int
    grid_order: tuple[int | str, ...]
    finish_order: tuple[int | str, ...]
    logical_events: tuple[LogicalEvent, ...]
    timing_checkpoints: tuple[AbstractTimingCheckpoint, ...] = ()
    command_log: tuple[AbstractCommandRecord, ...] = ()
    logical_tick_count: int = 0
    _canonical_hash: str = field(init=False, repr=False, compare=False)
    _canonical_json: str = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        grid_order = tuple(self.grid_order)
        finish_order = tuple(self.finish_order)
        if len(grid_order) != len(set(grid_order)) or len(finish_order) != len(set(finish_order)):
            raise ValueError("race result contains duplicate grid or finish drivers")
        if set(grid_order) != set(finish_order):
            raise ValueError("race result grid and finish drivers do not match")
        object.__setattr__(self, "grid_order", grid_order)
        object.__setattr__(self, "finish_order", finish_order)
        object.__setattr__(
            self,
            "logical_events",
            tuple(
                sorted(
                    self.logical_events,
                    key=lambda item: (round(item.logical_time_s, 9), item.event_id),
                )
            ),
        )
        object.__setattr__(
            self,
            "timing_checkpoints",
            tuple(
                sorted(
                    self.timing_checkpoints,
                    key=lambda item: (item.tick_index, item.checkpoint_id),
                )
            ),
        )
        normalized_commands = []
        for index, item in enumerate(self.command_log):
            if isinstance(item, AbstractCommandRecord):
                normalized = item
            elif isinstance(item, Mapping):
                normalized = AbstractCommandRecord(
                    sequence=int(item.get("sequence", index)),
                    command=str(item["command"]),
                    logical_time_s=float(item.get("logical_time_s", 0.0)),
                    payload=item.get("payload"),
                )
            elif isinstance(item, tuple) and len(item) == 2:
                normalized = AbstractCommandRecord(
                    sequence=index,
                    command=str(item[0]),
                    logical_time_s=0.0,
                    payload=item[1],
                )
            else:
                raise TypeError("command_log entries must be AbstractCommandRecord values")
            normalized_commands.append(normalized)
        sequences = [item.sequence for item in normalized_commands]
        if len(sequences) != len(set(sequences)):
            raise ValueError("command log sequences must be unique")
        # Preserve the supplied tuple order. Sequence is explicit authority;
        # sorting here would make a reordered command log hash-identical.
        object.__setattr__(self, "command_log", tuple(normalized_commands))
        sections = self._canonical_sections()
        canonical_hash_value = canonical_hash_sections(sections)
        object.__setattr__(self, "_canonical_hash", canonical_hash_value)
        payload = self._canonical_payload()
        object.__setattr__(
            self,
            "_canonical_json",
            canonical_json({**payload, "canonical_result_hash": canonical_hash_value}),
        )

    def _canonical_payload(self) -> dict[str, Any]:
        return {
            "result_contract_version": ABSTRACT_RESULT_CONTRACT_VERSION,
            "session_id": self.session_id,
            "session_seed": self.session_seed,
            "content_version": self.content_version,
            "ruleset_version": self.ruleset_version,
            "abstract_engine_version": self.abstract_engine_version,
            "snapshot_hash": self.snapshot_hash,
            "total_laps": self.total_laps,
            "grid_order": list(self.grid_order),
            "finish_order": list(self.finish_order),
            "logical_tick_count": self.logical_tick_count,
            "command_log": [item.to_dict() for item in self.command_log],
            "logical_events": [event.to_dict() for event in self.logical_events],
            "timing_checkpoints": [item.to_dict() for item in self.timing_checkpoints],
        }

    def _canonical_sections(self) -> tuple[tuple[str, Any], ...]:
        return (
            (
                "identity",
                {
                    "result_contract_version": ABSTRACT_RESULT_CONTRACT_VERSION,
                    "session_id": self.session_id,
                    "session_seed": self.session_seed,
                    "content_version": self.content_version,
                    "ruleset_version": self.ruleset_version,
                    "abstract_engine_version": self.abstract_engine_version,
                    "snapshot_hash": self.snapshot_hash,
                    "total_laps": self.total_laps,
                    "logical_tick_count": self.logical_tick_count,
                },
            ),
            ("grid", list(self.grid_order)),
            (
                "command_log",
                [item.to_dict() for item in self.command_log],
            ),
            ("logical_events", [event.to_dict() for event in self.logical_events]),
            ("timing_checkpoints", [item.to_dict() for item in self.timing_checkpoints]),
            ("finish_classification", list(self.finish_order)),
        )

    def canonical_payload(self) -> dict[str, Any]:
        """Return the small serializable authority payload, excluding its hash."""

        return self._canonical_payload()

    @property
    def result_contract_version(self) -> str:
        return ABSTRACT_RESULT_CONTRACT_VERSION

    @property
    def canonical_result_hash(self) -> str:
        return self._canonical_hash

    @property
    def result_hash(self) -> str:
        return self.canonical_result_hash

    def to_dict(self) -> dict[str, Any]:
        return {
            **self.canonical_payload(),
            "canonical_result_hash": self.canonical_result_hash,
        }

    def canonical_json(self) -> str:
        return self._canonical_json

    def to_json(self) -> str:
        return self.canonical_json()

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AbstractRaceResult":
        events = tuple(
            LogicalEvent(
                event_id=raw["event_id"],
                event_type=raw["event_type"],
                logical_time_s=raw["logical_time_s"],
                driver_ids=tuple(raw.get("driver_ids", ())),
                payload=tuple(sorted(raw.get("payload", {}).items())),
            )
            for raw in data["logical_events"]
        )
        checkpoints = []
        for raw in data.get("timing_checkpoints", ()):
            progress_raw = raw.get("progress_by_driver", ())
            if isinstance(progress_raw, Mapping):
                # Compatibility with an early draft that used JSON object
                # keys and therefore lost integer ID types.
                progress = tuple(
                    (next((item for item in raw["order"] if str(item) == key), key), value)
                    for key, value in progress_raw.items()
                )
            else:
                progress = tuple(
                    (item["driver_id"], item["progress"])
                    for item in progress_raw
                )
            checkpoints.append(
                AbstractTimingCheckpoint(
                    checkpoint_id=raw["checkpoint_id"],
                    checkpoint_kind=raw["checkpoint_kind"],
                    tick_index=raw["tick_index"],
                    logical_time_s=raw["logical_time_s"],
                    lap_number=raw["lap_number"],
                    order=tuple(raw["order"]),
                    progress_by_driver=progress,
                )
            )
        raw_command_log = data.get("command_log", ())
        if isinstance(raw_command_log, Mapping):
            command_log = tuple(
                AbstractCommandRecord(
                    sequence=index,
                    command=str(key),
                    logical_time_s=0.0,
                    payload=value,
                )
                for index, (key, value) in enumerate(raw_command_log.items())
            )
        else:
            command_log = tuple(
                AbstractCommandRecord(
                    sequence=int(item.get("sequence", index)),
                    command=str(item["command"]),
                    logical_time_s=float(item.get("logical_time_s", 0.0)),
                    payload=item.get("payload"),
                )
                if isinstance(item, Mapping)
                else AbstractCommandRecord(
                    sequence=index,
                    command=str(item[0]),
                    logical_time_s=0.0,
                    payload=item[1],
                )
                for index, item in enumerate(raw_command_log)
            )
        return cls(
            snapshot_hash=data.get("snapshot_hash", "legacy-unavailable"),
            session_id=data["session_id"],
            session_seed=data["session_seed"],
            content_version=data["content_version"],
            ruleset_version=data["ruleset_version"],
            abstract_engine_version=data["abstract_engine_version"],
            total_laps=data["total_laps"],
            grid_order=tuple(data["grid_order"]),
            finish_order=tuple(data["finish_order"]),
            logical_events=events,
            timing_checkpoints=tuple(checkpoints),
            command_log=command_log,
            logical_tick_count=data.get("logical_tick_count", data.get("frame_count", 0)),
        )
