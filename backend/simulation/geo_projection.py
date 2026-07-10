"""Project source circuit geometry into the game's 2D track coordinate space."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isclose, radians

from models.schemas import (
    Circuit,
    CircuitGeoFitBox,
    CircuitGeoState,
    CircuitMetricState,
    GeoPoint,
    MetricTrackPoint,
    TrackBoundaries,
)
from simulation.track_geometry import calculate_normal, normalize_points, resample_polyline_by_distance

Point = tuple[float, float]
WidthSample = tuple[float | None, float | None]

MIN_GEO_POINTS = 4
DEDUP_EPSILON_M = 0.05


@dataclass(frozen=True)
class GeoCompileResult:
    track_coords: list[list[float]]
    pit_lane_coords: list[list[float]]
    source_length_m: float
    computed_length_m: float
    scale_factor: float
    track_boundaries: TrackBoundaries | None = None


@dataclass(frozen=True)
class _ProjectionContext:
    origin_lat: float
    origin_lon: float
    meters_per_lat: float
    meters_per_lon: float


@dataclass(frozen=True)
class _FitTransform:
    scale: float
    offset_x: float
    offset_y: float


def compile_geo_circuit(circuit: Circuit) -> GeoCompileResult | None:
    """Compile optional ``Circuit.geo`` WGS84 data into canvas coordinates."""
    geo = circuit.geo
    if not geo or len(geo.centerline) < MIN_GEO_POINTS:
        return None

    projection = _projection_context(geo.centerline)
    source_points = _prepare_closed_meter_path(geo, projection)
    if len(source_points) < MIN_GEO_POINTS:
        return None

    source_length_m = _polyline_length(source_points)
    if source_length_m <= 0:
        return None

    scale_factor = 1.0
    if geo.scale_to_official_length and circuit.track_length_m > 0:
        scale_factor = circuit.track_length_m / source_length_m

    scaled_track = _scale_points(source_points, scale_factor)
    computed_length_m = _polyline_length(scaled_track)
    fit = _fit_points(scaled_track, geo.fit)
    fitted_track = _apply_fit(scaled_track, fit)
    track_coords = resample_polyline_by_distance(
        fitted_track,
        spacing=geo.sample_spacing,
        closed=True,
    )

    pit_lane_coords: list[list[float]] = []
    if len(geo.pit_lane) >= 2:
        pit_meter = [_project_point(point, projection) for point in geo.pit_lane]
        pit_meter = _dedupe_points(pit_meter)
        pit_scaled = _scale_points(pit_meter, scale_factor)
        pit_fitted = _apply_fit(pit_scaled, fit)
        pit_lane_coords = resample_polyline_by_distance(
            pit_fitted,
            spacing=geo.pit_sample_spacing,
            closed=False,
        )

    geo.source_length_m = round(source_length_m, 3)
    geo.computed_length_m = round(computed_length_m, 3)
    geo.scale_factor = round(scale_factor, 9)

    return GeoCompileResult(
        track_coords=track_coords,
        pit_lane_coords=pit_lane_coords,
        source_length_m=source_length_m,
        computed_length_m=computed_length_m,
        scale_factor=scale_factor,
    )


def compile_metric_circuit(circuit: Circuit) -> GeoCompileResult | None:
    """Compile optional ``Circuit.metric`` local-meter data into canvas coordinates."""
    metric = circuit.metric
    if not metric or len(metric.centerline) < MIN_GEO_POINTS:
        return None

    source_points, width_samples = _prepare_closed_metric_path(metric)
    if len(source_points) < MIN_GEO_POINTS:
        return None

    source_length_m = _polyline_length(source_points)
    if source_length_m <= 0:
        return None

    scale_factor = 1.0
    if metric.scale_to_official_length and circuit.track_length_m > 0:
        scale_factor = circuit.track_length_m / source_length_m

    scaled_track = _scale_points(source_points, scale_factor)
    computed_length_m = _polyline_length(scaled_track)
    fit = _fit_points(scaled_track, metric.fit)
    fitted_track = _apply_fit(scaled_track, fit)
    track_coords = resample_polyline_by_distance(
        fitted_track,
        spacing=metric.sample_spacing,
        closed=True,
    )

    pit_lane_coords: list[list[float]] = []
    if len(metric.pit_lane) >= 2:
        pit_meter = _dedupe_points([_metric_point(point) for point in metric.pit_lane])
        if metric.reverse:
            pit_meter = list(reversed(pit_meter))
        pit_scaled = _scale_points(pit_meter, scale_factor)
        pit_fitted = _apply_fit(pit_scaled, fit)
        pit_lane_coords = resample_polyline_by_distance(
            pit_fitted,
            spacing=metric.pit_sample_spacing,
            closed=False,
        )

    track_boundaries = _build_variable_width_boundaries(
        track_coords,
        fitted_track,
        width_samples,
        width_scale=scale_factor * fit.scale,
    )

    metric.source_length_m = round(source_length_m, 3)
    metric.computed_length_m = round(computed_length_m, 3)
    metric.scale_factor = round(scale_factor, 9)

    return GeoCompileResult(
        track_coords=track_coords,
        pit_lane_coords=pit_lane_coords,
        source_length_m=source_length_m,
        computed_length_m=computed_length_m,
        scale_factor=scale_factor,
        track_boundaries=track_boundaries,
    )


def _prepare_closed_meter_path(geo: CircuitGeoState, projection: _ProjectionContext) -> list[Point]:
    points = [_project_point(point, projection) for point in geo.centerline]
    points = _dedupe_points(points)
    if len(points) < MIN_GEO_POINTS:
        return points

    if _same_point(points[0], points[-1]):
        points = points[:-1]

    if geo.reverse:
        points = [points[0], *reversed(points[1:])]

    start_index = _start_finish_index(points, geo, projection)
    if start_index:
        points = [*points[start_index:], *points[:start_index]]

    points.append(points[0])
    return points


def _prepare_closed_metric_path(metric: CircuitMetricState) -> tuple[list[Point], list[WidthSample]]:
    pairs = [(_metric_point(point), _metric_width(point)) for point in metric.centerline]
    pairs = _dedupe_point_width_pairs(pairs)
    if len(pairs) < MIN_GEO_POINTS:
        return [point for point, _ in pairs], [width for _, width in pairs]

    if _same_point(pairs[0][0], pairs[-1][0]):
        pairs = pairs[:-1]

    if metric.reverse:
        pairs = [(point, _swap_widths(width)) for point, width in [pairs[0], *reversed(pairs[1:])]]

    start_index = _metric_start_finish_index(pairs, metric)
    if start_index:
        pairs = [*pairs[start_index:], *pairs[:start_index]]

    pairs.append(pairs[0])
    return [point for point, _ in pairs], [width for _, width in pairs]


def _projection_context(points: list[GeoPoint]) -> _ProjectionContext:
    origin_lat = sum(point.lat for point in points) / len(points)
    origin_lon = sum(point.lon for point in points) / len(points)
    lat_rad = radians(origin_lat)
    meters_per_lat = (
        111132.92
        - 559.82 * cos(2 * lat_rad)
        + 1.175 * cos(4 * lat_rad)
        - 0.0023 * cos(6 * lat_rad)
    )
    meters_per_lon = (
        111412.84 * cos(lat_rad)
        - 93.5 * cos(3 * lat_rad)
        + 0.118 * cos(5 * lat_rad)
    )
    return _ProjectionContext(
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        meters_per_lat=meters_per_lat,
        meters_per_lon=meters_per_lon,
    )


def _project_point(point: GeoPoint, projection: _ProjectionContext) -> Point:
    x = (point.lon - projection.origin_lon) * projection.meters_per_lon
    y = (projection.origin_lat - point.lat) * projection.meters_per_lat
    return x, y


def _metric_point(point: MetricTrackPoint) -> Point:
    return point.x_m, point.y_m


def _metric_width(point: MetricTrackPoint) -> WidthSample:
    return point.w_tr_right_m, point.w_tr_left_m


def _swap_widths(width: WidthSample) -> WidthSample:
    return width[1], width[0]


def _start_finish_index(
    points: list[Point],
    geo: CircuitGeoState,
    projection: _ProjectionContext,
) -> int:
    if geo.start_finish_index is not None:
        return min(len(points) - 1, max(0, geo.start_finish_index))
    if geo.start_finish is None:
        return 0

    target = _project_point(geo.start_finish, projection)
    return min(range(len(points)), key=lambda index: _distance(points[index], target))


def _metric_start_finish_index(
    pairs: list[tuple[Point, WidthSample]],
    metric: CircuitMetricState,
) -> int:
    if metric.start_finish_index is None:
        return 0
    return min(len(pairs) - 1, max(0, metric.start_finish_index))


def _fit_points(points: list[Point], fit_box: CircuitGeoFitBox) -> _FitTransform:
    min_x = min(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_x = max(point[0] for point in points)
    max_y = max(point[1] for point in points)
    source_width = max(1.0, max_x - min_x)
    source_height = max(1.0, max_y - min_y)
    target_width = max(1.0, fit_box.width - fit_box.padding * 2)
    target_height = max(1.0, fit_box.height - fit_box.padding * 2)
    scale = min(target_width / source_width, target_height / source_height)
    offset_x = fit_box.x + (fit_box.width - source_width * scale) / 2 - min_x * scale
    offset_y = fit_box.y + (fit_box.height - source_height * scale) / 2 - min_y * scale
    return _FitTransform(scale=scale, offset_x=offset_x, offset_y=offset_y)


def _apply_fit(points: list[Point], fit: _FitTransform) -> list[Point]:
    return [
        (
            point[0] * fit.scale + fit.offset_x,
            point[1] * fit.scale + fit.offset_y,
        )
        for point in points
    ]


def _scale_points(points: list[Point], scale: float) -> list[Point]:
    return [(point[0] * scale, point[1] * scale) for point in points]


def _dedupe_points(points: list[Point]) -> list[Point]:
    deduped: list[Point] = []
    for point in points:
        if not deduped or _distance(deduped[-1], point) > DEDUP_EPSILON_M:
            deduped.append(point)
    return deduped


def _dedupe_point_width_pairs(pairs: list[tuple[Point, WidthSample]]) -> list[tuple[Point, WidthSample]]:
    deduped: list[tuple[Point, WidthSample]] = []
    for point, width in pairs:
        if not deduped or _distance(deduped[-1][0], point) > DEDUP_EPSILON_M:
            deduped.append((point, width))
    return deduped


def _build_variable_width_boundaries(
    track_coords: list[list[float]],
    source_points: list[Point],
    width_samples: list[WidthSample],
    *,
    width_scale: float,
) -> TrackBoundaries | None:
    if not track_coords or not source_points or len(source_points) != len(width_samples):
        return None

    fallback = _fallback_half_width(width_samples)
    if fallback is None:
        return None

    points = normalize_points(track_coords)
    source_points_without_duplicate = source_points[:-1] if _same_point(source_points[0], source_points[-1]) else source_points
    width_samples_without_duplicate = width_samples[:-1] if len(source_points_without_duplicate) != len(source_points) else width_samples
    left: list[list[float]] = []
    right: list[list[float]] = []
    for index, point in enumerate(points):
        source_index = _nearest_point_index(source_points_without_duplicate, point)
        right_width_m, left_width_m = width_samples_without_duplicate[source_index]
        right_width = (right_width_m if right_width_m is not None else fallback) * width_scale
        left_width = (left_width_m if left_width_m is not None else fallback) * width_scale
        normal = calculate_normal(points, index)
        left.append([
            round(point[0] + normal[0] * left_width, 3),
            round(point[1] + normal[1] * left_width, 3),
        ])
        right.append([
            round(point[0] - normal[0] * right_width, 3),
            round(point[1] - normal[1] * right_width, 3),
        ])
    return TrackBoundaries(left=left, right=right)


def _fallback_half_width(width_samples: list[WidthSample]) -> float | None:
    widths = [
        width
        for sample in width_samples
        for width in sample
        if width is not None and width > 0
    ]
    if not widths:
        return None
    widths.sort()
    return widths[len(widths) // 2]


def _nearest_point_index(points: list[Point], target: Point) -> int:
    return min(range(len(points)), key=lambda index: _distance(points[index], target))


def _polyline_length(points: list[Point]) -> float:
    return sum(_distance(points[index], points[index + 1]) for index in range(len(points) - 1))


def _distance(a: Point, b: Point) -> float:
    return hypot(b[0] - a[0], b[1] - a[1])


def _same_point(a: Point, b: Point) -> bool:
    return isclose(a[0], b[0], abs_tol=DEDUP_EPSILON_M) and isclose(a[1], b[1], abs_tol=DEDUP_EPSILON_M)
