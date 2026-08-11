"""Provisional, axis-separated performance model for Stage A."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, Mapping

from models.schemas import Circuit, Team, TrackSegmentType


# These are game-balance axes, not official F1 physical measurements.
PERFORMANCE_AXES = (
    "power",
    "drag_efficiency",
    "high_speed_aero",
    "medium_speed_aero",
    "low_speed_grip",
    "braking",
    "traction",
    "tire_management",
    "cooling",
    "reliability",
)
CALIBRATION_STATUS = "game_calibration_provisional"


@dataclass(frozen=True, slots=True)
class VehiclePerformance:
    """Independent vehicle axes used by segment calculations.

    Values are normalized around 1.0.  ``reliability`` is intentionally not
    consumed by normal segment pace; it is reserved for a later incident
    layer.  All values in this first engine are provisional game corrections.
    """

    vehicle_id: str
    team_id: int | str
    power: float = 1.0
    drag_efficiency: float = 1.0
    high_speed_aero: float = 1.0
    medium_speed_aero: float = 1.0
    low_speed_grip: float = 1.0
    braking: float = 1.0
    traction: float = 1.0
    tire_management: float = 1.0
    cooling: float = 1.0
    reliability: float = 1.0
    calibration_status: str = CALIBRATION_STATUS

    def __post_init__(self) -> None:
        if not self.vehicle_id:
            raise ValueError("vehicle_id must be non-empty")
        for axis in PERFORMANCE_AXES:
            value = getattr(self, axis)
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{axis} must be finite and positive")

    @classmethod
    def from_team(cls, team: Team, vehicle_id: str | None = None) -> "VehiclePerformance":
        """Map validated team content into separate normalized game axes."""

        vehicle_id = vehicle_id or f"vehicle:{team.id}"
        mass_factor = 768.0 / team.mass_kg
        return cls(
            vehicle_id=vehicle_id,
            team_id=team.id,
            power=(team.engine_power_kw / 750.0)
            * (team.drivetrain_efficiency / 0.965)
            * mass_factor,
            drag_efficiency=0.85 / team.straight_drag_area_m2,
            high_speed_aero=team.corner_downforce_area_m2 / 5.20,
            medium_speed_aero=(
                (team.corner_downforce_area_m2 / 5.20)
                + (team.straight_downforce_area_m2 / 3.30)
            )
            / 2.0,
            low_speed_grip=team.mechanical_grip,
            braking=(team.brake_force_n / 44000.0) * mass_factor,
            traction=team.traction_factor * team.mechanical_grip,
            tire_management=1.0,
            cooling=1.0,
            reliability=team.reliability,
        )

    def with_upgrades(self, upgrades: Mapping[str, float]) -> "VehiclePerformance":
        """Return a new vehicle with additive axis upgrades.

        Upgrade values are local game-balance deltas (for example,
        ``{"power": 0.05}``), not official units.
        """

        values = {axis: getattr(self, axis) for axis in PERFORMANCE_AXES}
        for axis, delta in upgrades.items():
            if axis not in PERFORMANCE_AXES:
                raise ValueError(f"unknown performance axis: {axis}")
            if not isfinite(delta):
                raise ValueError(f"upgrade for {axis} must be finite")
            values[axis] += delta
            if values[axis] <= 0.0:
                raise ValueError(f"upgrade would make {axis} non-positive")
        return VehiclePerformance(
            vehicle_id=self.vehicle_id,
            team_id=self.team_id,
            calibration_status=self.calibration_status,
            **values,
        )

    upgrade = with_upgrades

    def to_dict(self) -> dict[str, Any]:
        return {
            "vehicle_id": self.vehicle_id,
            "team_id": self.team_id,
            **{axis: round(getattr(self, axis), 9) for axis in PERFORMANCE_AXES},
            "calibration_status": self.calibration_status,
        }


@dataclass(frozen=True, slots=True)
class SegmentRequirement:
    """A timing segment with explicit performance requirements."""

    segment_id: str
    name: str
    sector_id: str
    segment_type: str
    start_progress: float
    end_progress: float
    base_time_s: float
    weights: tuple[tuple[str, float], ...]
    calibration_status: str = CALIBRATION_STATUS

    def __post_init__(self) -> None:
        if not self.segment_id or not self.name or not self.sector_id:
            raise ValueError("segment identifiers must be non-empty")
        if not 0.0 <= self.start_progress < self.end_progress <= 1.0:
            raise ValueError("segment progress must be increasing within [0, 1]")
        if self.base_time_s <= 0.0:
            raise ValueError("segment base_time_s must be positive")
        if not self.weights or abs(sum(weight for _, weight in self.weights) - 1.0) > 1e-9:
            raise ValueError("segment weights must be non-empty and sum to one")
        if tuple(sorted(axis for axis, _ in self.weights)) != tuple(axis for axis, _ in self.weights):
            raise ValueError("segment weights must be stored in stable axis order")
        if any(axis not in PERFORMANCE_AXES or weight < 0.0 for axis, weight in self.weights):
            raise ValueError("segment weights contain an invalid axis or weight")

    def to_dict(self) -> dict[str, Any]:
        return {
            "segment_id": self.segment_id,
            "name": self.name,
            "sector_id": self.sector_id,
            "segment_type": self.segment_type,
            "start_progress": self.start_progress,
            "end_progress": self.end_progress,
            "base_time_s": round(self.base_time_s, 9),
            "weights": {axis: weight for axis, weight in self.weights},
            "calibration_status": self.calibration_status,
        }


DEFAULT_SEGMENT_WEIGHTS: dict[str, tuple[tuple[str, float], ...]] = {
    TrackSegmentType.STRAIGHT.value: (
        ("drag_efficiency", 0.40),
        ("power", 0.55),
        ("traction", 0.05),
    ),
    TrackSegmentType.HEAVY_BRAKING.value: (
        ("braking", 0.72),
        ("high_speed_aero", 0.14),
        ("tire_management", 0.14),
    ),
    TrackSegmentType.TRACTION.value: (
        ("low_speed_grip", 0.35),
        ("traction", 0.55),
        ("tire_management", 0.10),
    ),
    TrackSegmentType.SWEEPING.value: (
        ("high_speed_aero", 0.72),
        ("medium_speed_aero", 0.18),
        ("tire_management", 0.10),
    ),
    TrackSegmentType.TECHNICAL.value: (
        ("low_speed_grip", 0.28),
        ("medium_speed_aero", 0.52),
        ("braking", 0.10),
        ("tire_management", 0.10),
    ),
}


def segment_weights(segment_type: str) -> tuple[tuple[str, float], ...]:
    raw = DEFAULT_SEGMENT_WEIGHTS.get(segment_type, DEFAULT_SEGMENT_WEIGHTS[TrackSegmentType.TECHNICAL.value])
    return tuple(sorted(raw))


def _sector_boundaries(circuit: Circuit) -> list[tuple[str, float, float, int]]:
    sectors = sorted(
        circuit.sectors,
        key=lambda sector: (sector.start if sector.start is not None else 0.0, sector.name),
    )
    if not sectors:
        return [(f"S{index}", (index - 1) / 3.0, index / 3.0, 1) for index in range(1, 4)]
    boundaries: list[tuple[str, float, float, int]] = []
    for index, sector in enumerate(sectors, start=1):
        start = sector.start if sector.start is not None else (index - 1) / len(sectors)
        end = sector.end if sector.end is not None else index / len(sectors)
        boundaries.append((f"S{index}", start, end, sector.mini_sector_count))
    return boundaries


def build_segment_requirements(circuit: Circuit) -> tuple[SegmentRequirement, ...]:
    """Convert validated circuit segments into normalized provisional timings."""

    sectors = _sector_boundaries(circuit)
    raw_segments = sorted(
        circuit.segments,
        key=lambda segment: (segment.start, segment.end, segment.name),
    )
    if not raw_segments:
        raw_segments = [
            type("FallbackSegment", (), {
                "name": f"Sector {index}",
                "start": start,
                "end": end,
                "type": TrackSegmentType.TECHNICAL,
                "speed_factor": 1.0,
            })()
            for index, (_, start, end, _) in enumerate(sectors, start=1)
        ]

    pieces: list[tuple[str, str, str, float, float, float]] = []
    for raw_index, raw in enumerate(raw_segments, start=1):
        raw_type = raw.type.value if hasattr(raw.type, "value") else str(raw.type)
        cuts = [raw.start]
        cuts.extend(
            start for _, start, _, _ in sectors if raw.start < start < raw.end
        )
        cuts.append(raw.end)
        cuts = sorted(set(cuts))
        for piece_index, (start, end) in enumerate(zip(cuts, cuts[1:]), start=1):
            midpoint = (start + end) / 2.0
            sector_id = next(
                sector_id
                for sector_id, sector_start, sector_end, _ in sectors
                if sector_start <= midpoint < sector_end or (midpoint == 1.0 and sector_end == 1.0)
            )
            pieces.append(
                (
                    f"segment:{raw_index}:{piece_index}",
                    raw.name if piece_index == 1 else f"{raw.name} ({piece_index})",
                    sector_id,
                    start,
                    end,
                    max(float(raw.speed_factor or 1.0), 0.01),
                )
            )

    unscaled = [
        (end - start) * circuit.track_length_m / speed_factor
        for _, _, _, start, end, speed_factor in pieces
    ]
    total_unscaled = sum(unscaled)
    scale = circuit.base_lap_time / total_unscaled if total_unscaled > 0.0 else 1.0
    requirements = []
    for piece, unscaled_time in zip(pieces, unscaled):
        segment_id, name, sector_id, start, end, _ = piece
        raw_type = next(
            (
                segment.type.value if hasattr(segment.type, "value") else str(segment.type)
                for segment in raw_segments
                if segment.start <= start + 1e-12 and end <= segment.end + 1e-12
            ),
            TrackSegmentType.TECHNICAL.value,
        )
        requirements.append(
            SegmentRequirement(
                segment_id=segment_id,
                name=name,
                sector_id=sector_id,
                segment_type=raw_type,
                start_progress=start,
                end_progress=end,
                base_time_s=unscaled_time * scale,
                weights=segment_weights(raw_type),
            )
        )
    return tuple(requirements)


TIRE_GRIP_PROVISIONAL = {
    "C1": 0.970,
    "C2": 0.990,
    "C3": 1.005,
    "C4": 1.012,
    "C5": 1.020,
    "INTER": 0.950,
    "WET": 0.900,
}


@dataclass(frozen=True, slots=True)
class SegmentTimeBreakdown:
    segment_id: str
    base_time_s: float
    weighted_vehicle_score: float
    vehicle_factor: float
    driver_factor: float
    tire_factor: float
    track_factor: float
    traffic_penalty_s: float
    variance_s: float
    mistake_loss_s: float
    time_s: float

    def to_dict(self) -> dict[str, float | str]:
        return {
            "segment_id": self.segment_id,
            "base_time_s": round(self.base_time_s, 9),
            "weighted_vehicle_score": round(self.weighted_vehicle_score, 9),
            "vehicle_factor": round(self.vehicle_factor, 9),
            "driver_factor": round(self.driver_factor, 9),
            "tire_factor": round(self.tire_factor, 9),
            "track_factor": round(self.track_factor, 9),
            "traffic_penalty_s": round(self.traffic_penalty_s, 9),
            "variance_s": round(self.variance_s, 9),
            "mistake_loss_s": round(self.mistake_loss_s, 9),
            "time_s": round(self.time_s, 9),
        }


def _compound_value(tire: Any) -> str:
    value = getattr(tire, "physical_compound", tire)
    return getattr(value, "value", str(value))


def _tire_factor(tire: Any, vehicle: VehiclePerformance) -> float:
    compound = _compound_value(tire)
    grip = TIRE_GRIP_PROVISIONAL.get(compound, 1.0)
    temperature_band = getattr(tire, "temperature_band", "optimal")
    temperature_factor = {"optimal": 1.0, "cold": 1.006, "hot": 1.004}.get(
        temperature_band,
        1.0,
    )
    wear_laps = max(float(getattr(tire, "wear_laps", 0.0)), 0.0)
    wear_penalty = max(wear_laps - 1.0, 0.0) * 0.002 / max(vehicle.tire_management, 0.1)
    return temperature_factor / max(grip * (1.0 - wear_penalty), 0.5)


def calculate_segment_time(
    segment: SegmentRequirement,
    vehicle: VehiclePerformance,
    driver: Any,
    tire: Any,
    *,
    track_evolution: float = 0.0,
    traffic_penalty_s: float = 0.0,
    variance_s: float = 0.0,
    mistake_loss_s: float = 0.0,
) -> SegmentTimeBreakdown:
    """Calculate one sector segment without using reliability as pace."""

    weighted_score = sum(getattr(vehicle, axis) * weight for axis, weight in segment.weights)
    vehicle_factor = 1.0 - 0.060 * (weighted_score - 1.0)
    driver_factor = 1.0 - 0.035 * (float(getattr(driver, "pace", 0.85)) - 0.85)
    tire_factor = _tire_factor(tire, vehicle)
    track_factor = max(0.97, 1.0 - 0.008 * max(track_evolution, 0.0))
    base = segment.base_time_s * vehicle_factor * driver_factor * tire_factor * track_factor
    time_s = max(
        0.001,
        base + max(0.0, traffic_penalty_s) + variance_s + max(0.0, mistake_loss_s),
    )
    return SegmentTimeBreakdown(
        segment_id=segment.segment_id,
        base_time_s=segment.base_time_s,
        weighted_vehicle_score=weighted_score,
        vehicle_factor=vehicle_factor,
        driver_factor=driver_factor,
        tire_factor=tire_factor,
        track_factor=track_factor,
        traffic_penalty_s=max(0.0, traffic_penalty_s),
        variance_s=variance_s,
        mistake_loss_s=max(0.0, mistake_loss_s),
        time_s=time_s,
    )


def segment_time(*args: Any, **kwargs: Any) -> float:
    """Convenience numeric interface for A/B tests."""

    return calculate_segment_time(*args, **kwargs).time_s


compute_segment_time = segment_time
