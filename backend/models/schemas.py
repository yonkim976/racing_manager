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
    car_performance: float = Field(ge=0.90, le=1.10)
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


class TrackLayoutSegment(BaseModel):
    """An editable source segment that can be compiled into sampled coordinates."""
    type: TrackLayoutSegmentType
    start: list[float] = Field(min_length=2, max_length=2)
    end: list[float] = Field(min_length=2, max_length=2)
    cp1: Optional[list[float]] = Field(default=None, min_length=2, max_length=2)
    cp2: Optional[list[float]] = Field(default=None, min_length=2, max_length=2)
    samples: Optional[int] = Field(default=None, ge=4, le=160)


class PitLaneConfig(BaseModel):
    """Pit lane generated from track progress anchors and lateral offsets."""
    entry_progress: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    exit_progress: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    lane_offset: float = 0.0
    wall_offset: Optional[float] = None
    box_offset: float = 11.0
    entry_blend: float = Field(default=0.015, ge=0.0, le=0.25)
    exit_blend: float = Field(default=0.015, ge=0.0, le=0.25)
    samples: int = Field(default=44, ge=8, le=180)


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
    editor: Optional[CircuitEditorState] = None
    geo: Optional[CircuitGeoState] = None
    metric: Optional[CircuitMetricState] = None


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
    gap_to_leader: float = 0.0  # seconds behind leader
    last_lap_time: float = 0.0
    best_lap_time: float = 0.0
    in_pit: bool = False
    pit_count: int = 0
    retired: bool = False
    pit_request: Optional[TireCompound] = None  # pending pit request
    finished: bool = False
    total_time: float = 0.0  # cumulative race time in seconds
    speed_kph: float = 0.0  # current smoothed car speed
    drs_active: bool = False
    dirty_air_active: bool = False
    side_by_side_active: bool = False
    pace_mode: PaceMode = PaceMode.STANDARD


class RaceEvent(BaseModel):
    """An event that occurred during the race."""
    type: str
    driver: str = ""
    message: str = ""
    message_ko: str = ""


class LapTimeInfo(BaseModel):
    """Completed lap timing entry for driver statistics."""
    lap: int
    lap_time: float
    tire_compound: str
    stint: int
    pit_stop: bool = False


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
    restart_line_progress: float = 0.0
    pit_window_open: bool = False
    race_elapsed: float = 0.0
    speed_multiplier: int = 1
    paused: bool = False
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
    gap: str  # formatted gap string
    interval: str  # gap to car ahead
    tire_compound: str
    tire_age: int
    tire_wear: float
    pace_mode: str = PaceMode.STANDARD.value
    last_lap_time: float
    best_lap_time: float
    in_pit: bool
    pit_count: int
    pit_phase: Optional[str] = None  # in | stop | out
    pit_lane_progress: float = 0.0  # 0=pit entry, 1=pit exit
    pit_lane_progress_rate: float = 0.0  # pit-lane progress per game second
    pit_elapsed: float = 0.0  # total time spent in this pit stop (grows)
    pit_stop_elapsed: float = 0.0  # stationary tire-change time so far (grows)
    lap_history: list[LapTimeInfo] = Field(default_factory=list)
    retired: bool
    finished: bool = False
    drs_active: bool = False
    dirty_air_active: bool = False
    side_by_side_active: bool = False


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
    track_coords: list[list[float]]
    start_finish_index: int = 0
    pit_lane_coords: list[list[float]] = []
    pit_wall_coords: list[list[float]] = []
    pit_box_offset: float = 11.0
    drs_zones: list[DRSZone] = Field(default_factory=list)
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
