"""Validation rules for imported track-data lineage and physics channels."""

from __future__ import annotations

from math import isfinite

from models.schemas import Circuit
from simulation.track_geometry import TRACK_GEOMETRY_MODE
from simulation.track_physics import PHYSICAL_CAR_WIDTH_M, TRACK_EDGE_MARGIN_M


V2_LINEAGE_FIELDS = (
    "source",
    "source_id",
    "license",
    "attribution",
    "source_url",
    "acquired_at",
    "transform_version",
    "coordinate_system",
)

RAPID_WIDTH_CHANGE_M_PER_M = 0.25
RAPID_GRADE_CHANGE_PER_M = 0.05
RAPID_BANK_CHANGE_DEG_PER_M = 5.0


def _is_v2(circuit: Circuit) -> bool:
    return bool(
        circuit.data_version == "v2"
        or circuit.metric is not None
        or circuit.track_width_profile
        or circuit.track_conditions
    )


def _closed_profile_segment_length_m(first, second, track_length_m: float) -> float:
    """Return the physical distance between adjacent closed-loop samples."""
    return (
        (second.progress - first.progress) % 1.0
    ) * max(1.0, float(track_length_m))


def validate_track_data_v2(circuit: Circuit) -> list[str]:
    """Return actionable errors for the track-data v2 import contract."""
    errors: list[str] = []
    source = circuit.metric or circuit.geo
    if source is None:
        errors.append("track source metadata is missing")
    else:
        fields = V2_LINEAGE_FIELDS if _is_v2(circuit) else V2_LINEAGE_FIELDS[:4]
        for field in fields:
            if not getattr(source, field, None):
                errors.append(f"track source metadata field is missing: {field}")
        if _is_v2(circuit) and getattr(source, "coordinate_system", "") not in {
            "EPSG:4326",
            "local_meters",
        }:
            errors.append("v2 coordinate_system must be EPSG:4326 or local_meters")

    if not circuit.track_coords:
        errors.append("compiled centerline is empty")
    if circuit.track_width_profile:
        previous = -1.0
        for sample in circuit.track_width_profile:
            if sample.progress <= previous:
                errors.append(
                    "track width profile progress must be strictly increasing"
                )
                break
            if not all(
                isfinite(value)
                for value in (sample.progress, sample.left_width_m, sample.right_width_m)
            ):
                errors.append("track width profile values must be finite")
                break
            minimum_width = PHYSICAL_CAR_WIDTH_M / 2.0 + TRACK_EDGE_MARGIN_M
            if sample.left_width_m <= minimum_width or sample.right_width_m <= minimum_width:
                errors.append(
                    "track width profile must clear the car half-width and safety margin"
                )
                break
            previous = sample.progress
    elif circuit.data_version == "v2":
        errors.append("v2 track_width_profile is missing")

    if circuit.track_conditions:
        previous = -1.0
        for sample in circuit.track_conditions:
            if sample.progress <= previous:
                errors.append(
                    "track condition progress must be strictly increasing"
                )
                break
            if any(
                value is not None and not isfinite(value)
                for value in (
                    sample.elevation_m,
                    sample.grade,
                    sample.bank_angle_deg,
                )
            ):
                errors.append("track condition values must be finite")
                break
            previous = sample.progress
    elif circuit.data_version == "v2" and TRACK_GEOMETRY_MODE != "planar_2d":
        errors.append("v2 track_conditions is missing")
    return errors


def track_data_warnings(circuit: Circuit) -> list[str]:
    """Return explicit non-blocking warnings for legacy/fallback channels."""
    warnings: list[str] = []
    if not circuit.track_width_profile:
        warnings.append("track_width_profile is empty; using legacy constant-width fallback")
    if not circuit.track_conditions and TRACK_GEOMETRY_MODE != "planar_2d":
        warnings.append("track_conditions is empty; using legacy asphalt/flat fallback")

    if len(circuit.track_width_profile) >= 2:
        samples = circuit.track_width_profile
        for index, first in enumerate(samples):
            second = samples[(index + 1) % len(samples)]
            segment_length_m = _closed_profile_segment_length_m(
                first,
                second,
                circuit.track_length_m,
            )
            if segment_length_m <= 1e-9:
                continue
            if max(
                abs(second.left_width_m - first.left_width_m),
                abs(second.right_width_m - first.right_width_m),
            ) / segment_length_m > RAPID_WIDTH_CHANGE_M_PER_M:
                warnings.append("track width profile contains a rapid change")
                break
    if len(circuit.track_conditions) >= 2:
        samples = circuit.track_conditions
        for index, first in enumerate(samples):
            second = samples[(index + 1) % len(samples)]
            segment_length_m = _closed_profile_segment_length_m(
                first,
                second,
                circuit.track_length_m,
            )
            if segment_length_m <= 1e-9:
                continue
            grade_delta = abs((second.grade or 0.0) - (first.grade or 0.0))
            bank_delta = abs(
                (second.bank_angle_deg or 0.0) - (first.bank_angle_deg or 0.0)
            )
            if (
                grade_delta / segment_length_m > RAPID_GRADE_CHANGE_PER_M
                or bank_delta / segment_length_m > RAPID_BANK_CHANGE_DEG_PER_M
            ):
                warnings.append("track condition profile contains a rapid grade/bank change")
                break
    return warnings
