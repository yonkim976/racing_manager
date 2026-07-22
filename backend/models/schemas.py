"""Pydantic schemas for API request/response validation and data transfer."""

from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import AliasChoices, BaseModel, ConfigDict, Field, model_validator


# ── Enums ────────────────────────────────────────────────────────────────────


class TireCompound(str, Enum):
    """Available tire compounds."""
    SOFT = "SOFT"
    MEDIUM = "MEDIUM"
    HARD = "HARD"
    INTER = "INTER"
    WET = "WET"


class StrategyStyle(str, Enum):
    """AI strategy style for a team."""
    AGGRESSIVE = "aggressive"
    CONSERVATIVE = "conservative"
    BALANCED = "balanced"


class Weather(str, Enum):
    """Weather conditions during a race."""
    DRY = "dry"
    LIGHT_RAIN = "light_rain"
    HEAVY_RAIN = "heavy_rain"


class PaceMode(str, Enum):
    """Driver pace command from the pit wall."""
    CONSERVE = "CONSERVE"
    STANDARD = "STANDARD"
    ATTACK = "ATTACK"


class TrackSegmentType(str, Enum):
    """Driving-relevant section type for a circuit."""
    STRAIGHT = "straight"
    HEAVY_BRAKING = "heavy_braking"
    TECHNICAL = "technical"
    TRACTION = "traction"
    SWEEPING = "sweeping"


class TrackLayoutSegmentType(str, Enum):
    """Source geometry segment type for circuit centerlines."""
    STRAIGHT = "straight"
    BEZIER = "bezier"


class TrackSide(str, Enum):
    """Side of the circuit relative to increasing centerline progress."""

    LEFT = "left"
    RIGHT = "right"


class RunoffSurface(str, Enum):
    """Surface immediately outside a white line or kerb."""

    ASPHALT = "asphalt_runoff"
    GRASS = "grass"
    GRAVEL = "gravel"


# ── Driver ───────────────────────────────────────────────────────────────────


class DriverStats(BaseModel):
    """Statistical ratings for a driver, all on a 0.0–1.0 scale."""
    pace: float = Field(ge=0.0, le=1.0)
    consistency: float = Field(ge=0.0, le=1.0)
    tire_management: float = Field(ge=0.0, le=1.0)
    wet_skill: float = Field(ge=0.0, le=1.0)
    overtaking: float = Field(ge=0.0, le=1.0)
    defending: float = Field(ge=0.0, le=1.0)


class Driver(BaseModel):
    """A driver in the F1 game."""
    id: int
    name: str
    abbreviation: str = Field(min_length=3, max_length=3)
    number: int
    team_id: int
    stats: DriverStats


class DriverResponse(Driver):
    """Driver response including resolved team name."""
    team_name: str = ""
    team_color: str = ""


# ── Team ─────────────────────────────────────────────────────────────────────


class Team(BaseModel):
    """An F1 team / constructor."""
    id: int
    name: str
    mass_kg: float = Field(ge=760.0, le=850.0)
    engine_power_kw: float = Field(ge=680.0, le=800.0)
    drivetrain_efficiency: float = Field(ge=0.90, le=1.0)
    corner_drag_area_m2: float = Field(ge=0.8, le=1.6)
    straight_drag_area_m2: float = Field(ge=0.6, le=1.3)
    corner_downforce_area_m2: float = Field(ge=3.0, le=6.5)
    straight_downforce_area_m2: float = Field(ge=2.0, le=5.0)
    brake_force_n: float = Field(ge=30000.0, le=55000.0)
    mechanical_grip: float = Field(ge=0.90, le=1.10)
    front_aero_share: float = Field(ge=0.40, le=0.50)
    traction_factor: float = Field(ge=0.90, le=1.10)
    pit_crew_skill: float = Field(ge=0.0, le=1.0)
    reliability: float = Field(ge=0.0, le=1.0)
    strategy_style: StrategyStyle
    color: str


# ── Circuit ──────────────────────────────────────────────────────────────────


class Sector(BaseModel):
    """A sector of a circuit."""
    name: str
    type: str
    speed_factor: float
    start: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    end: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    mini_sector_count: int = Field(default=6, ge=1, le=20)

    @model_validator(mode="after")
    def validate_timing_range(self) -> "Sector":
        if (self.start is None) != (self.end is None):
            raise ValueError("sector start and end must be provided together")
        if self.start is not None and self.end is not None and self.start >= self.end:
            raise ValueError("sector start must be smaller than sector end")
        return self


class DRSZone(BaseModel):
    """A DRS zone as a normalized progress range."""
    name: str
    detection: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    start: float = Field(ge=0.0, le=1.0)
    end: float = Field(ge=0.0, le=1.0)


class Landmark(BaseModel):
    """A named marker attached to a track coordinate index."""
    type: str
    label: str = ""
    name: Optional[str] = None
    track_index: int = Field(default=0, ge=0)
    progress: Optional[float] = Field(default=None, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def fill_label_from_name(self) -> "Landmark":
        if not self.label and self.name:
            self.label = self.name
        if not self.label:
            self.label = self.type
        return self


class TrackSegment(BaseModel):
    """A driving-relevant circuit segment."""
    name: str
    start: float = Field(ge=0.0, le=1.0)
    end: float = Field(ge=0.0, le=1.0)
    type: TrackSegmentType
    speed_factor: Optional[float] = Field(default=None, gt=0.0, le=1.5)
    overtake_start_allowed: Optional[bool] = None
    side_by_side_allowed: bool = True
    overtake_risk: float = Field(default=0.5, ge=0.0, le=1.0)


class TrackSurfaceZone(BaseModel):
    """Per-side kerb and runoff override for a normalized track range."""

    start: float = Field(ge=0.0, le=1.0)
    end: float = Field(ge=0.0, le=1.0)
    side: TrackSide
    kerb_width_m: float = Field(default=1.2, ge=0.0, le=3.0)
    kerb_height: str = Field(default="low", pattern="^(low|high)$")
    runoff_surface: RunoffSurface = RunoffSurface.ASPHALT
    runoff_width_m: float = Field(default=3.0, ge=0.0, le=20.0)
    generated: bool = False


class TrackLayoutSegment(BaseModel):
    """An editable source segment that can be compiled into sampled coordinates."""
    type: TrackLayoutSegmentType
    start: list[float] = Field(min_length=2, max_length=2)
    end: list[float] = Field(min_length=2, max_length=2)
    cp1: Optional[list[float]] = Field(default=None, min_length=2, max_length=2)
    cp2: Optional[list[float]] = Field(default=None, min_length=2, max_length=2)
    samples: Optional[int] = Field(default=None, ge=4, le=160)


class PitLaneConfig(BaseModel):
    """Pit route anchors and operational landmarks.

    ``entry_progress`` and ``exit_progress`` locate the branch/rejoin points on
    the racing circuit.  The remaining progress values are measured along the
    open pit route itself: cars may decelerate before the speed-limit line and
    accelerate again after the speed-limit release line.
    """
    entry_progress: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    exit_progress: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    lane_offset: float = 0.0
    wall_offset: Optional[float] = None
    box_offset: float = 11.0
    lane_width_m: float = Field(default=4.0, ge=3.0, le=8.0)
    side_entry_progress: float = Field(default=0.02, ge=0.0, le=1.0)
    speed_limit_start: float = Field(default=0.12, ge=0.0, le=1.0)
    box_progress: float = Field(default=0.50, ge=0.0, le=1.0)
    speed_limit_end: float = Field(default=0.88, ge=0.0, le=1.0)
    side_rejoin_progress: float = Field(default=0.94, ge=0.0, le=1.0)
    entry_blend: float = Field(default=0.015, ge=0.0, le=0.25)
    exit_blend: float = Field(default=0.015, ge=0.0, le=0.25)
    samples: int = Field(default=44, ge=8, le=180)

    @model_validator(mode="after")
    def validate_operational_landmarks(self) -> "PitLaneConfig":
        if not (
            self.side_entry_progress
            < self.speed_limit_start
            < self.box_progress
            < self.speed_limit_end
            < self.side_rejoin_progress
        ):
            raise ValueError(
                "pit route landmarks must satisfy side_entry_progress "
                "< speed_limit_start "
                "< box_progress < speed_limit_end < side_rejoin_progress"
            )
        return self


class TrackPoint(BaseModel):
    """Detailed point metadata derived from a compiled circuit centerline."""
    x: float
    y: float
    s: float = 0.0
    progress: float = 0.0
    tangent_x: float = 1.0
    tangent_y: float = 0.0
    normal_x: float = 0.0
    normal_y: float = 1.0
    curvature: float = 0.0


class TrackBoundaries(BaseModel):
    """Left/right preview boundaries derived from centerline and track width."""
    left: list[list[float]] = Field(default_factory=list)
    right: list[list[float]] = Field(default_factory=list)


TrackCoord = list[float] | TrackPoint


class TrackWidthSample(BaseModel):
    """Per-progress distance from the centerline to each legal edge."""

    model_config = ConfigDict(populate_by_name=True)

    progress: float = Field(ge=0.0, lt=1.0)
    left_width_m: float = Field(
        ge=0.0,
        validation_alias=AliasChoices(
            "left_width_m",
            "leftWidthM",
            "w_tr_left_m",
            "wTrLeftM",
        ),
    )
    right_width_m: float = Field(
        ge=0.0,
        validation_alias=AliasChoices(
            "right_width_m",
            "rightWidthM",
            "w_tr_right_m",
            "wTrRightM",
        ),
    )


class TrackConditionSample(BaseModel):
    """Scalar 2.5D and surface channels sampled along track progress."""

    model_config = ConfigDict(populate_by_name=True)

    progress: float = Field(ge=0.0, lt=1.0)
    elevation_m: Optional[float] = Field(
        default=None,
        validation_alias=AliasChoices("elevation_m", "elevationM"),
    )
    grade: Optional[float] = Field(default=None, ge=-1.0, le=1.0)
    bank_angle_deg: Optional[float] = Field(
        default=None,
        ge=-45.0,
        le=45.0,
        validation_alias=AliasChoices("bank_angle_deg", "bankAngleDeg"),
    )
    surface: str = Field(default="asphalt", min_length=1)


class GeoPoint(BaseModel):
    """A WGS84 coordinate used as source geometry for real-world circuits."""
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)
    elevation_m: Optional[float] = None


class MetricTrackPoint(BaseModel):
    """A local metric circuit point, optionally carrying track-width samples."""
    model_config = ConfigDict(populate_by_name=True)

    x_m: float = Field(validation_alias=AliasChoices("x_m", "x", "xM"))
    y_m: float = Field(validation_alias=AliasChoices("y_m", "y", "yM"))
    w_tr_right_m: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("w_tr_right_m", "wTrRightM", "width_right_m", "widthRightM", "rightWidthM"),
    )
    w_tr_left_m: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("w_tr_left_m", "wTrLeftM", "width_left_m", "widthLeftM", "leftWidthM"),
    )
    elevation_m: Optional[float] = Field(
        default=None,
        validation_alias=AliasChoices("elevation_m", "elevationM"),
    )
    grade: Optional[float] = Field(default=None, ge=-1.0, le=1.0)
    bank_angle_deg: Optional[float] = Field(
        default=None,
        ge=-45.0,
        le=45.0,
        validation_alias=AliasChoices("bank_angle_deg", "bankAngleDeg"),
    )
    surface: str = Field(default="asphalt", min_length=1)


class CircuitGeoFitBox(BaseModel):
    """Canvas area used when projecting source geo data into game coordinates."""
    x: float = 0.0
    y: float = 0.0
    width: float = 820.0
    height: float = 620.0
    padding: float = Field(default=48.0, ge=0.0)


class CircuitGeoState(BaseModel):
    """Source geospatial circuit data and projection metadata."""
    model_config = ConfigDict(populate_by_name=True)

    source: str = "manual"
    source_id: Optional[str] = Field(default=None, validation_alias=AliasChoices("source_id", "sourceId"))
    license: Optional[str] = None
    attribution: Optional[str] = None
    source_url: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("source_url", "sourceUrl"),
    )
    acquired_at: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("acquired_at", "acquiredAt"),
    )
    transform_version: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("transform_version", "transformVersion"),
    )
    coordinate_system: str = Field(default="EPSG:4326", validation_alias=AliasChoices("coordinate_system", "coordinateSystem"))
    centerline: list[GeoPoint] = Field(
        default_factory=list,
        validation_alias=AliasChoices("centerline", "centerline_lonlat", "centerlineLonLat"),
    )
    pit_lane: list[GeoPoint] = Field(
        default_factory=list,
        validation_alias=AliasChoices("pit_lane", "pit_lane_lonlat", "pitLaneLonLat", "pitLane"),
    )
    start_finish: Optional[GeoPoint] = Field(
        default=None,
        validation_alias=AliasChoices("start_finish", "start_finish_lonlat", "startFinishLonLat", "startFinish"),
    )
    start_finish_index: Optional[int] = Field(
        default=None,
        ge=0,
        validation_alias=AliasChoices("start_finish_index", "startFinishIndex"),
    )
    reverse: bool = False
    scale_to_official_length: bool = Field(
        default=True,
        validation_alias=AliasChoices("scale_to_official_length", "scaleToOfficialLength"),
    )
    sample_spacing: float = Field(
        default=16.0,
        gt=2.0,
        le=80.0,
        validation_alias=AliasChoices("sample_spacing", "sampleSpacing"),
    )
    pit_sample_spacing: float = Field(
        default=14.0,
        gt=2.0,
        le=80.0,
        validation_alias=AliasChoices("pit_sample_spacing", "pitSampleSpacing"),
    )
    fit: CircuitGeoFitBox = Field(default_factory=CircuitGeoFitBox)
    source_length_m: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("source_length_m", "sourceLengthM"),
    )
    computed_length_m: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("computed_length_m", "computedLengthM"),
    )
    scale_factor: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("scale_factor", "scaleFactor"),
    )


class CircuitMetricState(BaseModel):
    """Source circuit data already expressed in local meter coordinates."""
    model_config = ConfigDict(populate_by_name=True)

    source: str = "manual"
    source_id: Optional[str] = Field(default=None, validation_alias=AliasChoices("source_id", "sourceId"))
    license: Optional[str] = None
    attribution: Optional[str] = None
    source_url: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("source_url", "sourceUrl"),
    )
    acquired_at: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("acquired_at", "acquiredAt"),
    )
    transform_version: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("transform_version", "transformVersion"),
    )
    coordinate_system: str = Field(
        default="local_meters",
        validation_alias=AliasChoices("coordinate_system", "coordinateSystem"),
    )
    centerline: list[MetricTrackPoint] = Field(
        default_factory=list,
        validation_alias=AliasChoices("centerline", "centerline_m", "centerlineM", "centerlineMeters", "track"),
    )
    pit_lane: list[MetricTrackPoint] = Field(
        default_factory=list,
        validation_alias=AliasChoices("pit_lane", "pit_lane_m", "pitLaneM", "pitLaneMeters", "pitLane"),
    )
    racing_line: list[MetricTrackPoint] = Field(
        default_factory=list,
        validation_alias=AliasChoices("racing_line", "racingLine", "raceline"),
    )
    start_finish_index: Optional[int] = Field(
        default=None,
        ge=0,
        validation_alias=AliasChoices("start_finish_index", "startFinishIndex"),
    )
    reverse: bool = False
    scale_to_official_length: bool = Field(
        default=True,
        validation_alias=AliasChoices("scale_to_official_length", "scaleToOfficialLength"),
    )
    sample_spacing: float = Field(
        default=16.0,
        gt=2.0,
        le=80.0,
        validation_alias=AliasChoices("sample_spacing", "sampleSpacing"),
    )
    pit_sample_spacing: float = Field(
        default=14.0,
        gt=2.0,
        le=80.0,
        validation_alias=AliasChoices("pit_sample_spacing", "pitSampleSpacing"),
    )
    fit: CircuitGeoFitBox = Field(default_factory=CircuitGeoFitBox)
    source_length_m: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("source_length_m", "sourceLengthM"),
    )
    computed_length_m: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("computed_length_m", "computedLengthM"),
    )
    scale_factor: Optional[float] = Field(
        default=None,
        ge=0.0,
        validation_alias=AliasChoices("scale_factor", "scaleFactor"),
    )


class EditorPoint(BaseModel):
    """Editable control point used by the circuit maker."""
    x: float
    y: float


class CircuitEditorView(BaseModel):
    """Circuit maker view transform."""
    offsetX: float = 0.0
    offsetY: float = 0.0
    scale: float = 1.0


class CircuitEditorState(BaseModel):
    """Non-race editor state persisted with draft circuits."""
    backgroundImage: Optional[str] = None
    backgroundOpacity: float = Field(default=0.5, ge=0.0, le=1.0)
    view: CircuitEditorView = Field(default_factory=CircuitEditorView)
    centerlineControlPoints: list[EditorPoint] = Field(default_factory=list)
    pitLaneControlPoints: list[EditorPoint] = Field(default_factory=list)
    trackWidth: float = Field(default=18.0, gt=0.0, le=80.0)
    sampleSpacing: float = Field(default=16.0, gt=2.0, le=80.0)


class RacingLineReferenceSample(BaseModel):
    """Measured lateral racing-line offset at normalized track progress."""

    model_config = ConfigDict(populate_by_name=True)

    progress: float = Field(ge=0.0, lt=1.0)
    lateral_offset_m: float = Field(
        ge=-12.0,
        le=12.0,
        validation_alias=AliasChoices("lateral_offset_m", "lateralOffsetM"),
    )


class TelemetryReferenceSample(BaseModel):
    """Distance-normalized speed and control reference from measured laps."""

    model_config = ConfigDict(populate_by_name=True)

    progress: float = Field(ge=0.0, lt=1.0)
    speed_kph: float = Field(
        gt=0.0,
        le=400.0,
        validation_alias=AliasChoices("speed_kph", "speedKph"),
    )
    throttle_percent: float = Field(
        default=0.0,
        ge=0.0,
        le=100.0,
        validation_alias=AliasChoices("throttle_percent", "throttlePercent"),
    )
    braking_fraction: float = Field(
        default=0.0,
        ge=0.0,
        le=1.0,
        validation_alias=AliasChoices("braking_fraction", "brakingFraction"),
    )


class CircuitPhysicsCalibration(BaseModel):
    """Measured-track calibration used by the predictive driving controller."""
    model_config = ConfigDict(populate_by_name=True)

    source: str
    source_url: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("source_url", "sourceUrl"),
    )
    reference_season: Optional[int] = Field(
        default=None,
        validation_alias=AliasChoices("reference_season", "referenceSeason"),
    )
    reference_session: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("reference_session", "referenceSession"),
    )
    reference_laps: list[str] = Field(
        default_factory=list,
        validation_alias=AliasChoices("reference_laps", "referenceLaps"),
    )
    reference_lap_time_seconds: Optional[float] = Field(
        default=None,
        gt=0.0,
        validation_alias=AliasChoices(
            "reference_lap_time_seconds",
            "referenceLapTimeSeconds",
        ),
    )
    racing_line_reference: list[RacingLineReferenceSample] = Field(
        default_factory=list,
        validation_alias=AliasChoices(
            "racing_line_reference",
            "racingLineReference",
        ),
    )
    telemetry_reference: list[TelemetryReferenceSample] = Field(
        default_factory=list,
        validation_alias=AliasChoices(
            "telemetry_reference",
            "telemetryReference",
        ),
    )
    telemetry_speed_reference_weight: float = Field(
        default=0.0,
        ge=0.0,
        le=1.0,
        validation_alias=AliasChoices(
            "telemetry_speed_reference_weight",
            "telemetrySpeedReferenceWeight",
        ),
    )
    telemetry_braking_curvature_threshold: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        validation_alias=AliasChoices(
            "telemetry_braking_curvature_threshold",
            "telemetryBrakingCurvatureThreshold",
        ),
    )
    planner_braking_utilization: float = Field(
        default=1.0,
        ge=0.3,
        le=1.0,
        validation_alias=AliasChoices(
            "planner_braking_utilization",
            "plannerBrakingUtilization",
        ),
    )
    telemetry_max_braking_utilization: float = Field(
        default=1.0,
        ge=0.3,
        le=1.0,
        validation_alias=AliasChoices(
            "telemetry_max_braking_utilization",
            "telemetryMaxBrakingUtilization",
        ),
    )
    telemetry_braking_speed_reserve: float = Field(
        default=1.0,
        ge=0.8,
        le=1.0,
        validation_alias=AliasChoices(
            "telemetry_braking_speed_reserve",
            "telemetryBrakingSpeedReserve",
        ),
    )
    braking_longitudinal_grip_factor: float = Field(
        default=1.0,
        ge=1.0,
        le=1.5,
        validation_alias=AliasChoices(
            "braking_longitudinal_grip_factor",
            "brakingLongitudinalGripFactor",
        ),
    )
    brake_control_error_fraction: float = Field(
        default=0.1,
        ge=0.03,
        le=0.2,
        validation_alias=AliasChoices(
            "brake_control_error_fraction",
            "brakeControlErrorFraction",
        ),
    )
    controller_sample_distance_m: float = Field(
        default=50.0,
        ge=5.0,
        le=50.0,
        validation_alias=AliasChoices(
            "controller_sample_distance_m",
            "controllerSampleDistanceM",
        ),
    )
    controller_speed_scale_floor: float = Field(
        default=0.6,
        ge=0.6,
        le=1.1,
        validation_alias=AliasChoices(
            "controller_speed_scale_floor",
            "controllerSpeedScaleFloor",
        ),
    )
    racing_line_reference_weight: float = Field(
        default=0.0,
        ge=0.0,
        le=2.0,
        validation_alias=AliasChoices(
            "racing_line_reference_weight",
            "racingLineReferenceWeight",
        ),
    )
    racing_line_initial_smoothing_passes: int = Field(
        default=3,
        ge=0,
        le=6,
        validation_alias=AliasChoices(
            "racing_line_initial_smoothing_passes",
            "racingLineInitialSmoothingPasses",
        ),
    )


class Circuit(BaseModel):
    """An F1 circuit."""
    model_config = ConfigDict(populate_by_name=True)

    id: int | str
    name: str
    country: str
    base_lap_time: float
    total_laps: int = Field(default=30, validation_alias=AliasChoices("total_laps", "laps"))
    pit_loss_time: float = 20.0
    track_length_m: float = Field(
        default=5000.0,
        gt=0.0,
        validation_alias=AliasChoices("track_length_m", "length_m"),
    )
    data_version: Optional[str] = Field(
        default=None,
        validation_alias=AliasChoices("data_version", "dataVersion"),
    )
    track_width_m: float = Field(default=12.0, ge=8.0, le=24.0)
    track_width_profile: list[TrackWidthSample] = Field(default_factory=list)
    track_conditions: list[TrackConditionSample] = Field(default_factory=list)
    overtaking_difficulty: float = Field(default=0.5, ge=0.0, le=1.0)
    sectors: list[Sector] = Field(default_factory=list)
    layout_segments: list[TrackLayoutSegment] = Field(default_factory=list)
    track_coords: list[TrackCoord] = Field(default_factory=list)
    track_points: list[TrackPoint] = Field(default_factory=list)
    track_boundaries: Optional[TrackBoundaries] = None
    start_finish_index: int = 0
    pit_lane: Optional[PitLaneConfig] = None
    pit_lane_segments: list[TrackLayoutSegment] = Field(default_factory=list)
    pit_lane_coords: list[TrackCoord] = Field(default_factory=list)
    pit_wall_coords: list[list[float]] = Field(default_factory=list)
    drs_zones: list[DRSZone] = Field(default_factory=list)
    landmarks: list[Landmark] = Field(default_factory=list)
    segments: list[TrackSegment] = Field(default_factory=list)
    surface_zones: list[TrackSurfaceZone] = Field(default_factory=list)
    editor: Optional[CircuitEditorState] = None
    geo: Optional[CircuitGeoState] = None
    metric: Optional[CircuitMetricState] = None
    physics_calibration: Optional[CircuitPhysicsCalibration] = Field(
        default=None,
        validation_alias=AliasChoices("physics_calibration", "physicsCalibration"),
    )


# ── Race State ───────────────────────────────────────────────────────────────


class DriverRaceState(BaseModel):
    """Per-driver state during a race."""
    driver_id: int
    position: int = 0
    progress: float = 0.0  # 0.0–1.0 track position within current lap
    current_lap: int = 0
    total_progress: float = 0.0  # total laps completed + fractional progress
    tire_compound: TireCompound = TireCompound.MEDIUM
    tire_age: int = 0  # laps since last tire change
    tire_usage: float = 0.0  # tire load accumulated since last tire change
    tire_wear: float = 0.0  # 0.0 = fresh, higher = more worn
    tire_lateral_grip: float = 1.0
    tire_traction_grip: float = 1.0
    tire_braking_grip: float = 1.0
    tire_surface_temperature_c: float = 90.0
    tire_core_temperature_c: float = 90.0
    tire_thermal_grip: float = 1.0
    fuel_mass_kg: float = 0.0
    fuel_burned_kg: float = 0.0
    fuel_laps_remaining: float = 0.0
    planner_tier_hz: int = 5
    planner_mode: str = "cruise"
    planner_generation_ms: float = 0.0
    planner_fallback_active: bool = False
    planner_nearby_vehicle_count: int = 0
    planner_replan_count: int = 0
    gap_to_leader: float = 0.0  # seconds behind leader
    last_lap_time: float = 0.0
    best_lap_time: float = 0.0
    in_pit: bool = False
    pit_count: int = 0
    retired: bool = False
    pit_request: Optional[TireCompound] = None  # pending pit request
    finished: bool = False
    total_time: float = 0.0  # cumulative race time in seconds
    simulation_time_s: float = 0.0
    physics_frame: int = 0
    world_x_m: float = 0.0
    world_y_m: float = 0.0
    heading_rad: float = 0.0
    yaw_rate_rad_s: float = 0.0
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    acceleration_x_mps2: float = 0.0
    acceleration_y_mps2: float = 0.0
    lateral_acceleration_mps2: float = 0.0
    steering_angle_rad: float = 0.0
    gear: int = 0
    engine_rpm: float = 0.0
    drive_force_n: float = 0.0
    track_elevation_m: float = 0.0
    track_grade: float = 0.0
    telemetry_source: str = "physics"
    speed_kph: float = 0.0  # current smoothed car speed
    total_distance_m: float = 0.0  # physical race distance, including completed laps
    acceleration_mps2: float = 0.0
    target_speed_kph: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    racing_line: str = "racing_line"
    lateral_offset_m: float = 0.0
    lateral_speed_mps: float = 0.0
    target_lateral_offset_m: float = 0.0
    grip_utilization: float = 0.0
    handling_state: str = "stable"
    slip_angle_rad: float = 0.0
    wheel_lock_ratio: float = 0.0
    traction_slip_ratio: float = 0.0
    tire_slide_energy_j: float = 0.0
    car_width_m: float = 1.9
    car_length_m: float = 5.0
    wheelbase_m: float = 3.4
    drs_active: bool = False
    dirty_air_active: bool = False
    wake_strength: float = 0.0
    tow_strength: float = 0.0
    dirty_air_strength: float = 0.0
    wake_source_driver_id: Optional[int] = None
    wake_longitudinal_gap_m: float = 0.0
    wake_lateral_separation_m: float = 0.0
    wake_drag_multiplier: float = 1.0
    wake_downforce_multiplier: float = 1.0
    wake_braking_grip_multiplier: float = 1.0
    wake_lateral_grip_multiplier: float = 1.0
    side_by_side_active: bool = False
    surface_state: str = "track"
    wheel_surfaces: list[str] = Field(default_factory=lambda: ["track"] * 4)
    kerb_contact: bool = False
    off_track: bool = False
    off_track_cause: str = ""
    track_limits_active: bool = False
    surface_grip_multiplier: float = 1.0
    surface_drag_deceleration_mps2: float = 0.0
    contact_active: bool = False
    contact_opponent_id: Optional[int] = None
    contact_impact_speed_mps: float = 0.0
    contact_type: str = ""
    contact_severity: str = ""
    contact_progress: float = 0.0
    contact_lateral_offset_m: float = 0.0
    contact_normal_longitudinal: float = 0.0
    contact_normal_lateral: float = 0.0
    collision_damage: float = 0.0
    vehicle_status: str = "moving"
    hazard_active: bool = False
    hazard_cause: str = ""
    local_yellow_active: bool = False
    avoidance_active: bool = False
    avoidance_hazard_driver_id: Optional[int] = None
    avoidance_side: str = ""
    avoidance_ttc_seconds: float = 0.0
    avoidance_target_lateral_offset_m: float = 0.0
    emergency_braking: bool = False
    pace_mode: PaceMode = PaceMode.STANDARD


class RaceEvent(BaseModel):
    """An event that occurred during the race."""
    type: str
    driver: str = ""
    message: str = ""
    message_ko: str = ""
    payload: dict[str, float | int | str] = Field(default_factory=dict)


class LapTimeInfo(BaseModel):
    """Completed lap timing entry for driver statistics."""
    lap: int
    lap_time: float
    tire_compound: str
    stint: int
    pit_stop: bool = False
    sector_times: list[float] = Field(default_factory=list)
    mini_sector_times: list[float] = Field(default_factory=list)


class VehicleTrajectorySample(BaseModel):
    """Authoritative 50 Hz pose retained between screen-state broadcasts."""

    simulation_time_s: float
    physics_frame: int
    world_x_m: float
    world_y_m: float
    heading_rad: float
    progress: float
    lateral_offset_m: float
    in_pit: bool = False
    pit_lane_progress: float = 0.0


class RaceTickState(BaseModel):
    """Complete state snapshot for a single tick broadcast."""
    type: str = "tick"
    lap: int
    total_laps: int
    weather: str = "dry"
    safety_car: bool = False
    race_phase: str = "green"  # green | vsc | sc
    race_phase_remaining_seconds: float = 0.0
    race_phase_remaining_laps: int = 0
    safety_car_stage: str = "inactive"
    safety_car_visible: bool = False
    safety_car_route: str = "track"  # track | pit
    safety_car_progress: float = 0.0
    safety_car_progress_rate: float = 0.0
    safety_car_pit_lane_progress: float = 0.0
    safety_car_queue_formed: bool = False
    overtaking_allowed: bool = True
    race_started: bool = True
    start_sequence_phase: str = "racing"  # grid | lights | lights_out | racing
    start_light_count: int = 0
    restart_line_progress: float = 0.0
    pit_window_open: bool = False
    race_elapsed: float = 0.0
    physics_frame: int = 0
    speed_multiplier: int = 1
    paused: bool = False
    physics_hz: int = 50
    broadcast_hz: int = 30
    effective_speed_multiplier: float = 0.0
    simulation_backlog_seconds: float = 0.0
    broadcast_jitter_ms: float = 0.0
    physics_steps_last_broadcast: int = 0
    positions: list[DriverPositionInfo] = []
    events: list[RaceEvent] = []


class DriverPositionInfo(BaseModel):
    """Driver info for the tick broadcast."""
    driver_id: int
    name: str  # abbreviation
    full_name: str
    team: str
    team_color: str
    position: int
    progress: float
    progress_rate: float = 0.0  # track progress per game second
    speed_kph: float = 0.0
    acceleration_mps2: float = 0.0
    target_speed_kph: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    racing_line: str = "racing_line"
    lateral_offset_m: float = 0.0
    lateral_speed_mps: float = 0.0
    target_lateral_offset_m: float = 0.0
    grip_utilization: float = 0.0
    handling_state: str = "stable"
    slip_angle_rad: float = 0.0
    wheel_lock_ratio: float = 0.0
    traction_slip_ratio: float = 0.0
    tire_slide_energy_j: float = 0.0
    car_width_m: float = 1.9
    car_length_m: float = 5.0
    wheelbase_m: float = 3.4
    gap: str  # formatted gap string
    interval: str  # gap to car ahead
    gap_seconds: Optional[float] = None
    interval_seconds: Optional[float] = None
    timing_gap_valid: bool = False
    current_sector: int = 1
    current_mini_sector: int = 1
    current_timing_loop: int = 1
    last_sector_time: float = 0.0
    last_mini_sector_time: float = 0.0
    tire_compound: str
    tire_age: int
    tire_wear: float
    tire_lateral_grip: float = 1.0
    tire_traction_grip: float = 1.0
    tire_braking_grip: float = 1.0
    tire_surface_temperature_c: float = 90.0
    tire_core_temperature_c: float = 90.0
    tire_thermal_grip: float = 1.0
    fuel_mass_kg: float = 0.0
    fuel_burned_kg: float = 0.0
    fuel_laps_remaining: float = 0.0
    planner_tier_hz: int = 5
    planner_mode: str = "cruise"
    planner_generation_ms: float = 0.0
    planner_fallback_active: bool = False
    planner_nearby_vehicle_count: int = 0
    planner_replan_count: int = 0
    pace_mode: str = PaceMode.STANDARD.value
    pace_mode_from: str = PaceMode.STANDARD.value
    pace_mode_transition_progress: float = 1.0
    pace_mode_effective_intensity: float = 0.0
    last_lap_time: float
    best_lap_time: float
    in_pit: bool
    pit_count: int
    pit_phase: Optional[str] = None  # in | stop | out
    pit_merge_state: Optional[str] = None  # approach | limited | stop | yield | hold | merge
    pit_merge_conflict_driver_id: Optional[int] = None
    pit_merge_conflict_group_id: Optional[str] = None
    pit_merge_conflict_group_member_ids: list[int] = Field(default_factory=list)
    pit_lane_progress: float = 0.0  # 0=pit entry, 1=pit exit
    pit_lane_progress_rate: float = 0.0  # pit-lane progress per game second
    pit_elapsed: float = 0.0  # total time spent in this pit stop (grows)
    pit_stop_elapsed: float = 0.0  # stationary tire-change time so far (grows)
    lap_history: list[LapTimeInfo] = Field(default_factory=list)
    retired: bool
    finished: bool = False
    simulation_time_s: float = 0.0
    physics_frame: int = 0
    world_x_m: float = 0.0
    world_y_m: float = 0.0
    heading_rad: float = 0.0
    yaw_rate_rad_s: float = 0.0
    velocity_x_mps: float = 0.0
    velocity_y_mps: float = 0.0
    acceleration_x_mps2: float = 0.0
    acceleration_y_mps2: float = 0.0
    lateral_acceleration_mps2: float = 0.0
    steering_angle_rad: float = 0.0
    gear: int = 0
    engine_rpm: float = 0.0
    drive_force_n: float = 0.0
    track_elevation_m: float = 0.0
    track_grade: float = 0.0
    telemetry_source: str = "physics"
    drs_active: bool = False
    dirty_air_active: bool = False
    wake_strength: float = 0.0
    tow_strength: float = 0.0
    dirty_air_strength: float = 0.0
    wake_source_driver_id: Optional[int] = None
    wake_longitudinal_gap_m: float = 0.0
    wake_lateral_separation_m: float = 0.0
    wake_drag_multiplier: float = 1.0
    wake_downforce_multiplier: float = 1.0
    maneuver_active: bool = False
    maneuver_phase: Optional[str] = None
    maneuver_role: Optional[str] = None
    maneuver_opponent_id: Optional[int] = None
    maneuver_line: Optional[str] = None
    maneuver_elapsed_seconds: float = 0.0
    side_by_side_active: bool = False
    maneuver_corner_active: bool = False
    maneuver_corner_authorized: bool = False
    maneuver_corridor: Optional[str] = None
    maneuver_corner_turn_direction: int = 0
    maneuver_corner_entry_advantage_m: float = 0.0
    maneuver_group_id: Optional[str] = None
    maneuver_group_size: int = 0
    maneuver_group_member_ids: list[int] = Field(default_factory=list)
    maneuver_group_phase: Optional[str] = None
    maneuver_group_corridor_index: Optional[int] = None
    maneuver_group_corner_priority: Optional[int] = None
    maneuver_group_corner_turn_direction: int = 0
    forced_wide_by_driver_id: Optional[int] = None
    surface_state: str = "track"
    wheel_surfaces: list[str] = Field(default_factory=lambda: ["track"] * 4)
    kerb_contact: bool = False
    off_track: bool = False
    off_track_cause: str = ""
    track_limits_active: bool = False
    surface_grip_multiplier: float = 1.0
    surface_drag_deceleration_mps2: float = 0.0
    contact_active: bool = False
    contact_opponent_id: Optional[int] = None
    contact_impact_speed_mps: float = 0.0
    contact_type: str = ""
    contact_severity: str = ""
    contact_progress: float = 0.0
    contact_lateral_offset_m: float = 0.0
    contact_normal_longitudinal: float = 0.0
    contact_normal_lateral: float = 0.0
    collision_damage: float = 0.0
    vehicle_status: str = "moving"
    hazard_active: bool = False
    hazard_cause: str = ""
    local_yellow_active: bool = False
    avoidance_active: bool = False
    avoidance_hazard_driver_id: Optional[int] = None
    avoidance_side: str = ""
    avoidance_ttc_seconds: float = 0.0
    avoidance_target_lateral_offset_m: float = 0.0
    emergency_braking: bool = False
    trajectory_samples: list[VehicleTrajectorySample] = Field(default_factory=list)


# Rebuild models that reference forward-declared types
RaceTickState.model_rebuild()


# ── API Request/Response Models ──────────────────────────────────────────────


class RaceSetupRequest(BaseModel):
    """Request body for setting up a new race."""
    circuit_id: int
    player_team_id: int
    total_laps: int = Field(default=30, ge=5, le=100)
    starting_tires: dict[int, TireCompound] = Field(default_factory=dict)
    grid_order: list[int] = Field(default_factory=list)


class RaceSetupResponse(BaseModel):
    """Response after setting up a race."""
    session_id: str
    circuit: Circuit
    player_team: Team
    player_drivers: list[Driver]
    grid_order: list[dict]  # list of {driver_id, name, team, position}


class StartingGridSlot(BaseModel):
    """Physical start box shared by the simulation and track renderer."""

    position: int
    progress: float
    lateral_offset_m: float


class QualifyingRequest(BaseModel):
    """Request body for running a qualifying simulation."""
    circuit_id: int
    player_team_id: int
    attempt_laps: int = Field(default=3, ge=1, le=6)


class QualifyingResult(BaseModel):
    """One driver's qualifying result."""
    position: int
    driver_id: int
    name: str
    full_name: str
    team: str
    team_color: str
    best_lap_time: float
    gap: str
    laps: list[float]
    knockout: str = "Q3"  # last session the driver participated in: Q1/Q2/Q3
    q1_time: Optional[float] = None
    q2_time: Optional[float] = None
    q3_time: Optional[float] = None


class QualifyingResponse(BaseModel):
    """Response after running qualifying."""
    circuit: Circuit
    player_team: Team
    results: list[QualifyingResult]
    grid_order: list[int]


class RaceInfoMessage(BaseModel):
    """Initial race info sent via WebSocket on connection."""
    type: str = "race_info"
    circuit_name: str
    total_laps: int
    player_team: str
    player_team_color: str
    player_drivers: list[int]
    track_length_m: float
    world_origin_x_render: float = 0.0
    world_origin_y_render: float = 0.0
    world_meters_per_render_unit: float = 1.0
    track_width_m: float = 12.0
    car_width_m: float = 1.9
    car_length_m: float = 5.0
    wheelbase_m: float = 3.4
    grid_slots: list[StartingGridSlot] = Field(default_factory=list)
    racing_line_profile: list[list[float]] = Field(default_factory=list)
    track_width_profile: list[list[float]] = Field(default_factory=list)
    surface_zones: list[TrackSurfaceZone] = Field(default_factory=list)
    track_conditions: list[TrackConditionSample] = Field(default_factory=list)
    racing_line_coords: list[list[float]] = Field(default_factory=list)
    racing_line_length_m: float = 0.0
    predicted_racing_lap_time: float = 0.0
    driving_line_coords: dict[str, list[list[float]]] = Field(default_factory=dict)
    driving_line_lengths_m: dict[str, float] = Field(default_factory=dict)
    predicted_line_lap_times: dict[str, float] = Field(default_factory=dict)
    track_coords: list[list[float]]
    start_finish_index: int = 0
    pit_lane_coords: list[list[float]] = []
    pit_wall_coords: list[list[float]] = []
    pit_box_offset: float = 11.0
    pit_lane_width_m: float = 4.0
    pit_side_entry_progress: float = 0.02
    pit_speed_limit_start: float = 0.12
    pit_box_progress: float = 0.50
    pit_speed_limit_end: float = 0.88
    pit_side_rejoin_progress: float = 0.94
    drs_zones: list[DRSZone] = Field(default_factory=list)
    sectors: list[Sector] = Field(default_factory=list)
    landmarks: list[Landmark] = Field(default_factory=list)
    segments: list[TrackSegment] = Field(default_factory=list)


class RaceEndMessage(BaseModel):
    """Final race results sent via WebSocket."""
    type: str = "race_end"
    results: list[dict]


class PlayerCommand(BaseModel):
    """A command received from the player via WebSocket."""
    type: str
    driver_id: Optional[int] = None
    tire_choice: Optional[str] = None
    multiplier: Optional[int] = None
    pace_mode: Optional[str] = None
