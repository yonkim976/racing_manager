"""Deterministic ABSTRACT broadcast replay over the existing low-frequency contract."""

from __future__ import annotations

import asyncio
import hashlib
import json
import struct
import threading
from collections.abc import Iterable
from math import hypot
from typing import Any

from models.schemas import Circuit, Driver, DryTireRole, PaceMode, Team, TrackConditions

from .replay import InterruptibleReplayClock, RollingFrameBuffer
from .progress_broadcast import (
    PROGRESS_BROADCAST_PRESENTATION_CONTRACT,
    ProgressBroadcastCursor,
)
from .progress_race import (
    PROGRESS_RACECRAFT_AUTHORITY_VERSION,
    PROGRESS_RACE_ENGINE_VERSION,
    PROGRESS_TIMING_AUTHORITY_VERSION,
)
from .race import (
    AbstractCursorRuntimeCheckpoint,
    AbstractTrafficSimulationCursor,
    AbstractTrafficTick,
)
from .state import (
    AbstractRaceFrame,
    AbstractRaceResult,
    AbstractSessionSnapshot,
    LogicalEvent,
    canonical_json,
)
from simulation.track_display import TrackDisplayGeometry, build_track_display_geometry


BROADCAST_LOGICAL_TICK_SECONDS = 0.10
BROADCAST_SPEEDS = (1, 2, 5)
BROADCAST_MAX_EVENTS_PER_MESSAGE = 32
BROADCAST_MAX_EVENT_PAYLOAD_BYTES = 16_384
BROADCAST_PROBE_BUFFER_CAPACITY = 128
BROADCAST_MAX_POSITION_SPEED_MPS = 370.0 / 3.6
STAGE4_FULL_FIELD_CONTRACT = "stage4-full-field-traffic"
BROADCAST_TRANSPORT_CONTRACT = "abstract-interactive-buffered-binary-v2"
BROADCAST_START_BUFFER_TICKS = 20
BROADCAST_AHEAD_BUFFER_TICKS = 300
BROADCAST_PRODUCER_BATCH_TICKS = 10
BROADCAST_DASHBOARD_EVERY_TICKS = 2
BROADCAST_CHECKPOINT_EVERY_TICKS = 100
POSE_PACKET_MAGIC = b"F1P1"
POSE_PACKET_HEADER = struct.Struct("<4sIBBB")
POSE_DRIVER_HEADER = struct.Struct("<HBB")
POSE_SAMPLE = struct.Struct("<fIfff")


def _dump(value: Any) -> Any:
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if isinstance(value, (list, tuple)):
        return [_dump(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _dump(item) for key, item in value.items()}
    return value


def _event_message(event: LogicalEvent, driver_names: dict[int | str, str]) -> tuple[str, str]:
    names = ", ".join(driver_names.get(driver_id, str(driver_id)) for driver_id in event.driver_ids)
    subject = f" ({names})" if names else ""
    labels = {
        "race_started": ("Abstract broadcast started", "추상 방송을 시작했습니다"),
        "race_finished": ("Abstract broadcast finished", "추상 방송을 종료했습니다"),
        "driver_finished": ("Finished", "완주"),
        "overtake_completed": ("Overtake completed", "추월 완료"),
        "attack_started": ("Attack started", "공격 시작"),
        "maneuver_group_formed": ("Three-car battle", "3대 전투 형성"),
        "maneuver_group_dissolved": ("Three-car battle cleared", "3대 전투 해산"),
        "defense_hold": ("Defense held", "방어 성공"),
        "attack_aborted": ("Attack aborted", "공격 철회"),
        "pit_requested": ("Pit stop requested", "피트스톱 요청"),
        "pit_lane_entry": ("Entered pit lane", "피트 레인 진입"),
        "pit_stop_started": ("Pit stop started", "피트 작업 시작"),
        "pit_stop_completed": ("Pit stop completed", "피트 작업 완료"),
        "pit_exit": ("Exited pit lane", "피트 레인 탈출"),
        "pit_call_registered": ("Pit call registered", "피트콜 접수"),
        "pit_call_cancelled": ("Pit call cancelled", "피트콜 취소"),
        "pace_mode_changed": ("Pace mode changed", "페이스 모드 변경"),
        "lockup": ("Logical lockup", "논리 락업"),
        "minor_contact": ("Minor contact", "경미한 접촉"),
        "contact_started": ("Contact", "접촉 발생"),
        "driver_retired": ("Retired", "리타이어"),
        "local_yellow_started": ("Local yellow", "로컬 옐로"),
        "local_yellow_ended": ("Track clear", "로컬 옐로 종료"),
        "vsc_started": ("Virtual Safety Car", "버추얼 세이프티카 발동"),
        "vsc_ended": ("VSC ending", "버추얼 세이프티카 종료"),
        "safety_car_deployed": ("Safety Car deployed", "세이프티카 발동"),
        "safety_car_catch_up_started": ("Safety Car catch-up", "세이프티카 대열 합류"),
        "safety_car_queue_formed": ("Safety Car queue formed", "세이프티카 대열 형성"),
        "safety_car_restart_ready": ("Restart ready", "재시작 준비"),
        "safety_car_in_this_lap": ("Safety Car in this lap", "이번 랩 세이프티카 복귀"),
        "race_restarted": ("Race restarted", "레이스 재시작"),
    }
    english, korean = labels.get(
        event.event_type,
        (event.event_type.replace("_", " ").title(), event.event_type),
    )
    return f"{english}{subject}", f"{korean}{subject}"


class AbstractBroadcastSession:
    """Single-pass buffered presenter for one authoritative traffic cursor.

    The producer advances the cursor exactly once and keeps only a bounded
    future window.  The consumer releases those accepted ticks according to
    the interruptible replay clock.  A precomputed result may be supplied by
    diagnostics as a parity oracle, but product broadcast sessions do not need
    to compute the race before the first client can connect.
    """

    def __init__(
        self,
        *,
        snapshot: AbstractSessionSnapshot,
        result: AbstractRaceResult | None = None,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int | None = None,
        circuit: Circuit,
        player_team: Team,
        player_drivers: list[Driver],
        drivers: list[Driver],
        teams: list[Team],
        track_conditions: TrackConditions,
        thermal_preset: str | None,
        conditions_source: str,
        frame_buffer: RollingFrameBuffer | None = None,
        display_geometry: TrackDisplayGeometry | None = None,
        probe_driver_id: int | str | None = None,
        authority_mode: str = "stage4",
    ) -> None:
        self.snapshot = snapshot
        self._expected_result = result
        self.result = result
        resolved_grid_order = tuple(
            result.grid_order
            if result is not None
            else (grid_order or snapshot.initial_grid_order)
        )
        resolved_total_laps = result.total_laps if result is not None else total_laps
        if resolved_total_laps is None:
            raise ValueError("abstract broadcast total_laps is required")
        self.grid_order = resolved_grid_order
        self.total_laps = int(resolved_total_laps)
        self.frame_buffer = frame_buffer or RollingFrameBuffer()
        self.circuit = circuit
        self.player_team = player_team
        self.player_drivers = list(player_drivers)
        self.drivers = tuple(sorted(drivers, key=lambda driver: str(driver.id)))
        self.teams = tuple(sorted(teams, key=lambda team: str(team.id)))
        self.track_conditions = track_conditions
        self.thermal_preset = thermal_preset
        self.conditions_source = conditions_source
        self.display_geometry = (
            display_geometry
            or snapshot.track.display_geometry
            or build_track_display_geometry(circuit, grid_driver_ids=resolved_grid_order)
        )
        normalized_authority = str(authority_mode).lower()
        if normalized_authority not in {"stage4", "progress_v5"}:
            raise ValueError("abstract broadcast authority_mode must be stage4 or progress_v5")
        self.authority_mode = normalized_authority
        self.presentation_contract = (
            PROGRESS_BROADCAST_PRESENTATION_CONTRACT
            if self.authority_mode == "progress_v5"
            else STAGE4_FULL_FIELD_CONTRACT
        )
        self.session_id = snapshot.session_id
        self.clients: set[Any] = set()
        self._loop_task: asyncio.Task | None = None
        self._producer_task: asyncio.Task | None = None
        self._frame_index = 0
        self.replay_cursor = None
        self.replay_clock = InterruptibleReplayClock(speed_multiplier=1)
        self._paused = False
        self._speed_multiplier = 1
        self._finished = False
        self._race_end_sent = False
        self._initial_checkpoint_events_drained = False
        self._checkpoint_index = 0
        self._event_index = 0
        self._producer_error: BaseException | None = None
        self._pending_events: list[LogicalEvent] = []
        self._final_frame_consumed = False
        self._producer_stop = threading.Event()
        self._producer_pause = threading.Event()
        self._cursor_lock = threading.RLock()
        self._command_lock = asyncio.Lock()
        self._command_in_progress = False
        self._producer_generation = 0
        self._invalidated_tick_count = 0
        self._tick_buffer: asyncio.Queue[AbstractTrafficTick] = asyncio.Queue(
            maxsize=BROADCAST_AHEAD_BUFFER_TICKS,
        )
        self._buffer_ready_event: asyncio.Event | None = asyncio.Event()
        self._buffer_drained_event: asyncio.Event | None = asyncio.Event()
        self._producer_finished_event: asyncio.Event | None = asyncio.Event()
        self._driver_map = {driver.id: driver for driver in self.drivers}
        self._team_map = {team.id: team for team in self.teams}
        self._entry_map = snapshot.entry_by_driver_id()
        self._driver_names = {
            driver.id: driver.abbreviation for driver in self.drivers
        }
        self._finish_times = {
            event.driver_ids[0]: event.logical_time_s
            for event in (result.logical_events if result is not None else ())
            if event.event_type == "driver_finished" and event.driver_ids
        }
        player_driver_ids = tuple(sorted((driver.id for driver in self.player_drivers), key=str))
        if not player_driver_ids:
            player_driver_ids = tuple(sorted(self._entry_map, key=str))[:1]
        if not player_driver_ids:
            raise ValueError("abstract broadcast requires a player driver for the Stage 3 probe")
        if probe_driver_id is not None and probe_driver_id not in player_driver_ids:
            raise ValueError("probe_driver_id must be one of the player drivers")
        self.probe_driver_id = probe_driver_id if probe_driver_id is not None else player_driver_ids[0]
        self.traffic_cursor: AbstractTrafficSimulationCursor | ProgressBroadcastCursor | None
        if self.authority_mode == "progress_v5":
            self.traffic_cursor = ProgressBroadcastCursor(
                snapshot,
                grid_order=resolved_grid_order,
                total_laps=self.total_laps,
                tick_seconds=BROADCAST_LOGICAL_TICK_SECONDS,
                display_geometry=self.display_geometry,
            )
        else:
            self.traffic_cursor = AbstractTrafficSimulationCursor(
                snapshot,
                grid_order=resolved_grid_order,
                total_laps=self.total_laps,
                tick_seconds=BROADCAST_LOGICAL_TICK_SECONDS,
            )
        self._cursor_checkpoints: dict[int, AbstractCursorRuntimeCheckpoint] = {
            0: self.traffic_cursor.export_runtime_checkpoint(),
        }
        self._displayed_frame = self.traffic_cursor.current_frame
        self._last_checkpoint = self.traffic_cursor.timing_checkpoints[0]
        self._initial_events = tuple(self.traffic_cursor.events)
        self._pose_digest = hashlib.sha256(b"abstract-broadcast-full-field-pose-v1\0")
        self._emitted_pose_count = 0
        self._missing_pose_count = 0
        self._duplicate_pose_count = 0
        self._pose_interval_violation_count = 0
        self._position_speed_violation_count = 0
        self._last_emitted_frame: AbstractRaceFrame | None = None
        self._max_frame_gap = 0
        self._max_logical_time_gap = 0.0
        self._max_world_jump_m = 0.0
        self._max_position_derived_speed_mps = 0.0
        self._frame_buffer_peak = 0
        self._activity_event: asyncio.Event | None = asyncio.Event()
        self._no_clients_paused = False

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def speed_multiplier(self) -> int:
        return self._speed_multiplier

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def probe_cursor(self):
        """Compatibility marker; Stage 4 has no independent single probe."""

        return None

    @property
    def probe_buffer(self) -> RollingFrameBuffer:
        """Compatibility view over the bounded full-field frame window."""

        return self.frame_buffer

    def diagnostic_counts(self) -> dict[str, Any]:
        traffic_cursor = self.traffic_cursor
        traffic_metrics = traffic_cursor.metrics if traffic_cursor else {}
        producer = self._producer_task
        return {
            "active_session": not self._finished,
            "session_id": self.session_id,
            "source_mode": "abstract",
            "abstract_engine_version": (
                PROGRESS_RACE_ENGINE_VERSION
                if self.authority_mode == "progress_v5"
                else self.snapshot.abstract_engine_version
            ),
            "timing_authority_version": (
                PROGRESS_TIMING_AUTHORITY_VERSION
                if self.authority_mode == "progress_v5"
                else None
            ),
            "racecraft_authority_version": (
                PROGRESS_RACECRAFT_AUTHORITY_VERSION
                if self.authority_mode == "progress_v5"
                else None
            ),
            "authority_mode": self.authority_mode,
            "client_count": len(self.clients),
            "loop_task_exists": self._loop_task is not None,
            "loop_task_done": bool(self._loop_task and self._loop_task.done()),
            "producer_task_exists": producer is not None,
            "producer_task_done": bool(producer and producer.done()),
            "producer_error": str(self._producer_error) if self._producer_error else None,
            "producer_generation": self._producer_generation,
            "producer_paused_for_command": self._producer_pause.is_set(),
            "interactive_command_in_progress": self._command_in_progress,
            "interactive_checkpoint_count": len(self._cursor_checkpoints),
            "invalidated_tick_count": self._invalidated_tick_count,
            "producer_buffer_count": self._tick_buffer.qsize(),
            "producer_buffer_capacity": self._tick_buffer.maxsize,
            "producer_start_buffer_ticks": BROADCAST_START_BUFFER_TICKS,
            "frame_index": self._frame_index,
            "checkpoint_index": self._checkpoint_index,
            "checkpoint_count": (
                len(self.result.timing_checkpoints) if self.result is not None else self._checkpoint_index + 1
            ),
            "event_index": self._event_index,
            "event_count": len(self.result.logical_events) if self.result is not None else self._event_index,
            "remaining_logical_s": self.replay_clock.remaining_logical_s,
            "frame_count": len(self.frame_buffer),
            "frame_capacity": self.frame_buffer.max_frames,
            "probe_driver_id": self.probe_driver_id,
            "probe_frame_count": len(self.frame_buffer),
            "probe_frame_capacity": self.frame_buffer.max_frames,
            "probe_buffer_peak": self._frame_buffer_peak,
            "probe_cursor_tick": self._frame_index if self._displayed_frame is not None else None,
            "probe_cursor_exists": False,
            "traffic_cursor_exists": traffic_cursor is not None,
            "traffic_cursor_tick": traffic_cursor.current_tick if traffic_cursor else None,
            "traffic_active_reservations": traffic_cursor.active_reservation_count if traffic_cursor else 0,
            "traffic_buffer_count": len(self.frame_buffer),
            "traffic_buffer_capacity": self.frame_buffer.max_frames,
            "traffic_buffer_peak": self._frame_buffer_peak,
            "traffic_corridor_conflict_count": traffic_metrics.get("corridor_conflict_count", 0),
            "traffic_admitted_reservation_conflict_count": traffic_metrics.get("admitted_reservation_conflict_count", 0),
            "traffic_third_vehicle_occupancy_conflict_count": traffic_metrics.get("third_vehicle_occupancy_conflict_count", 0),
            "traffic_reservation_body_clearance_violation_count": traffic_metrics.get("reservation_body_clearance_violation_count", 0),
            "traffic_same_lane_longitudinal_overlap_count": traffic_metrics.get("same_lane_longitudinal_overlap_count", 0),
            "traffic_attack_terminal_event_count": traffic_metrics.get("attack_terminal_event_count", 0),
            "traffic_attack_terminal_event_duplicate_count": traffic_metrics.get("attack_terminal_event_duplicate_count", 0),
            "pose_count": 20,
            "emitted_pose_count": self._emitted_pose_count,
            "missing_pose_count": self._missing_pose_count,
            "duplicate_pose_count": self._duplicate_pose_count,
            "pose_interval_violation_count": self._pose_interval_violation_count,
            "position_speed_violation_count": self._position_speed_violation_count,
            "max_frame_gap": self._max_frame_gap,
            "max_logical_time_gap": self._max_logical_time_gap,
            "max_world_jump_m": self._max_world_jump_m,
            "max_position_derived_speed_mps": self._max_position_derived_speed_mps,
            "broadcast_pose_hash": self._pose_digest.hexdigest(),
            "retained_pose_count_after_close": len(self.frame_buffer) * 20,
            "activity_event_exists": self._activity_event is not None,
            "race_end_sent": self._race_end_sent,
            "initial_checkpoint_events_drained": self._initial_checkpoint_events_drained,
            "presentation_contract": self.presentation_contract,
            "transport_contract": BROADCAST_TRANSPORT_CONTRACT,
            "paused": self._paused,
            "speed_multiplier": self._speed_multiplier,
        }

    def _sendable_circuit_list(self, values: Iterable[Any]) -> list[Any]:
        return [_dump(value) for value in values]

    @property
    def race_info(self) -> dict[str, Any]:
        geometry = self.display_geometry.to_race_info_geometry()
        return {
            "type": "race_info",
            "session_id": self.session_id,
            "simulation_mode": "ABSTRACT_BROADCAST",
            "source_mode": "abstract",
            "presentation_contract": self.presentation_contract,
            "authority_mode": self.authority_mode,
            "transport_contract": BROADCAST_TRANSPORT_CONTRACT,
            "buffering": {
                "mode": "single-pass-producer-consumer",
                "start_buffer_ticks": BROADCAST_START_BUFFER_TICKS,
                "ahead_buffer_ticks": self._tick_buffer.maxsize,
                "logical_tick_seconds": BROADCAST_LOGICAL_TICK_SECONDS,
            },
            "interactive_strategy": {
                "commands": ["set_pace_mode", "pit_call", "pit_cancel"],
                "player_driver_ids": list(
                    sorted((driver.id for driver in self.player_drivers), key=str)
                ),
                "effective_from": "next-logical-tick",
                "unpublished_buffer_policy": "rewind-discard-regenerate",
                "checkpoint_interval_ticks": BROADCAST_CHECKPOINT_EVERY_TICKS,
            },
            "pose_count": 20,
            "frame_buffer_capacity": self.frame_buffer.max_frames,
            "kinematic_probe": {
                "driver_id": self.probe_driver_id,
                "pose_count": 20,
                "buffer_capacity": self.frame_buffer.max_frames,
                "preview_scope": (
                    "progress-v5-derived-pose; poses are presentation only"
                    if self.authority_mode == "progress_v5"
                    else "bounded-full-field-stage4-preview; poses are presentation only"
                ),
            },
            "traffic_preview": {
                "pose_count": 20,
                "buffer_capacity": self.frame_buffer.max_frames,
                "scope": "single authoritative cursor with bounded future buffer",
                "rank_authority": (
                    "progress-v5-rank"
                    if self.authority_mode == "progress_v5"
                    else "accepted-traffic-frame"
                ),
            },
            "abstract_engine_version": (
                PROGRESS_RACE_ENGINE_VERSION
                if self.authority_mode == "progress_v5"
                else self.snapshot.abstract_engine_version
            ),
            "canonical_result_hash": (
                self.result.canonical_result_hash if self.result is not None else None
            ),
            "circuit_name": self.circuit.name,
            "total_laps": self.total_laps,
            "player_team": self.player_team.name,
            "player_team_color": self.player_team.color,
            "player_drivers": [driver.id for driver in self.player_drivers],
            **geometry,
            "surface_zones": self._sendable_circuit_list(self.circuit.surface_zones),
            "track_conditions": self._sendable_circuit_list(self.circuit.track_conditions),
            "environment_conditions": self.snapshot.environment.to_dict(),
            "thermal_preset": self.thermal_preset,
            "track_conditions_source": self.conditions_source,
            "tire_compound_nomination": _dump(self.circuit.tire_compound_nomination),
            "drs_zones": self._sendable_circuit_list(self.circuit.drs_zones),
            "sectors": self._sendable_circuit_list(self.circuit.sectors),
            "landmarks": self._sendable_circuit_list(self.circuit.landmarks),
            "segments": self._sendable_circuit_list(self.circuit.segments),
        }

    @property
    def _frames(self) -> tuple[AbstractRaceFrame, ...]:
        return self.frame_buffer.snapshot()

    def _frame_at_cursor(self) -> AbstractRaceFrame | None:
        frames = self._frames
        if not frames:
            return None
        return frames[min(self._frame_index, len(frames) - 1)]

    def _current_checkpoint(self):
        return self._last_checkpoint

    def _logical_positions(self, checkpoint=None) -> list[dict[str, Any]]:
        checkpoint = checkpoint or self._current_checkpoint()
        progress = dict(checkpoint.progress_by_driver) if checkpoint else {}
        order = checkpoint.order if checkpoint else self.grid_order
        return [
            {
                "driver_id": driver_id,
                "position": position,
                "total_progress": progress.get(driver_id, 0.0),
                "finished": (
                    driver_id in self.result.finish_order if self.result is not None else False
                ),
                "source_mode": "abstract",
                "telemetry_source": "logical-checkpoint",
            }
            for position, driver_id in enumerate(order, start=1)
        ]

    def _position_payload(self, frame: AbstractRaceFrame) -> list[dict[str, Any]]:
        positions = []
        for vehicle in frame.vehicles:
            driver = self._driver_map[vehicle.driver_id]
            team = self._team_map[driver.team_id]
            entry = self._entry_map[vehicle.driver_id]
            is_retired = (
                vehicle.reliability_state == "failed"
                or vehicle.incident_state == "terminal_damage"
            )
            is_finished = vehicle.total_progress >= self.total_laps and not is_retired
            current_sector = int(str(vehicle.sector_id).removeprefix("S") or 1)
            gap = f"+{vehicle.gap_to_leader_s:.3f}"
            interval = f"+{vehicle.interval_to_ahead_s:.3f}"
            positions.append(
                {
                    "driver_id": vehicle.driver_id,
                    "name": driver.abbreviation,
                    "full_name": driver.name,
                    "team": team.name,
                    "team_color": team.color,
                    "position": vehicle.position,
                    "progress": vehicle.progress,
                    "total_progress": vehicle.total_progress,
                    "progress_rate": vehicle.progress_rate_per_s,
                    "speed_kph": max(0.0, vehicle.speed_mps * 3.6),
                    "gap": "LEADER" if vehicle.position == 1 else gap,
                    "interval": "—" if vehicle.position == 1 else interval,
                    "gap_seconds": None if vehicle.position == 1 else vehicle.gap_to_leader_s,
                    "interval_seconds": None if vehicle.position == 1 else vehicle.interval_to_ahead_s,
                    "timing_gap_valid": vehicle.timing_gap_valid,
                    "interval_timing_gap_valid": vehicle.interval_timing_gap_valid,
                    "timing_gap_source": "live" if vehicle.timing_gap_valid else "estimated",
                    "interval_timing_gap_source": "live" if vehicle.interval_timing_gap_valid else "estimated",
                    "current_sector": current_sector,
                    "current_mini_sector": 1,
                    "current_timing_loop": 1,
                    "last_sector_time": 0.0,
                    "last_mini_sector_time": 0.0,
                    "mini_sector_splits": [],
                    "mini_sector_statuses": [],
                    "tire_compound": vehicle.tire_role,
                    "tire_role": vehicle.tire_role,
                    "physical_tire_compound": vehicle.physical_compound,
                    "tire_age": vehicle.stint_lap,
                    "tire_wear": min(1.0, vehicle.wear_laps / max(1, self.total_laps)),
                    "last_lap_time": 0.0,
                    "best_lap_time": 0.0,
                    "in_pit": vehicle.pit_state != "none",
                    "pit_count": vehicle.pit_stop_count,
                    "pit_phase": vehicle.pit_state if vehicle.pit_state != "none" else None,
                    "pit_elapsed": 0.0,
                    "pit_lane_progress": vehicle.pit_lane_progress,
                    "pit_request_pending": vehicle.pit_request_pending,
                    "pit_request_role": vehicle.pit_request_role,
                    "pit_request_compound": vehicle.pit_request_compound,
                    "pace_mode": vehicle.pace_mode,
                    "pace_mode_from": vehicle.pace_mode,
                    "pace_mode_transition_progress": 1.0,
                    "damage_level": vehicle.damage_level,
                    "retired": is_retired,
                    "finished": is_finished,
                    "simulation_time_s": frame.logical_time_s,
                    "physics_frame": frame.tick_index,
                    "world_x_m": vehicle.world_x_m,
                    "world_y_m": vehicle.world_y_m,
                    "heading_rad": vehicle.heading_rad,
                    "line_distance_m": vehicle.line_distance_m,
                    "speed_mps": vehicle.speed_mps,
                    "longitudinal_acceleration_mps2": vehicle.longitudinal_acceleration_mps2,
                    "lateral_velocity_mps": vehicle.lateral_velocity_mps,
                    "lateral_acceleration_mps2": vehicle.lateral_acceleration_mps2,
                    "lateral_offset_m": vehicle.lateral_offset_m,
                    "handling_state": vehicle.visual_state,
                    "maneuver": vehicle.maneuver,
                    "maneuver_active": (
                        vehicle.maneuver_group_size >= 3
                        or vehicle.maneuver in {
                            "attack", "side_by_side", "defend", "clearance"
                        }
                    ),
                    "maneuver_phase": (
                        vehicle.maneuver_group_phase or vehicle.maneuver
                    ),
                    "maneuver_role": (
                        "opportunist"
                        if vehicle.maneuver_group_size >= 3
                        and vehicle.maneuver not in {
                            "attack", "side_by_side", "defend", "clearance"
                        }
                        else "attacker"
                        if vehicle.maneuver in {"attack", "side_by_side", "clearance"}
                        else "defender"
                        if vehicle.maneuver == "defend"
                        else None
                    ),
                    "maneuver_opponent_id": vehicle.ahead_driver_id,
                    "side_by_side_active": vehicle.maneuver in {
                        "attack",
                        "side_by_side",
                        "defend",
                        "clearance",
                    },
                    "attack_mode": vehicle.attack_mode,
                    "maneuver_side": vehicle.maneuver_side,
                    "maneuver_progress": vehicle.maneuver_progress,
                    "maneuver_group_id": vehicle.maneuver_group_id,
                    "maneuver_group_size": vehicle.maneuver_group_size,
                    "maneuver_group_member_ids": list(
                        vehicle.maneuver_group_member_ids
                    ),
                    "maneuver_group_phase": vehicle.maneuver_group_phase,
                    "maneuver_group_corridor_index": (
                        vehicle.maneuver_group_corridor_index
                    ),
                    "drs_train_id": vehicle.drs_train_id,
                    "drs_train_size": vehicle.drs_train_size,
                    "drs_train_position": vehicle.drs_train_position,
                    "drs_train_member_ids": list(vehicle.drs_train_member_ids),
                    "local_yellow_active": (
                        vehicle.incident_state is not None and not is_retired
                    ),
                    "hazard_active": vehicle.incident_state is not None,
                    "drs_active": vehicle.drs_active,
                    "dirty_air_active": vehicle.dirty_air_active,
                    "dirty_air_strength": vehicle.dirty_air_strength,
                    "tow_strength": vehicle.tow_strength,
                    "source_mode": "abstract",
                    "telemetry_source": "abstract",
                }
            )
        return positions

    def _current_frame(self) -> AbstractRaceFrame:
        if self._displayed_frame is None:
            raise RuntimeError("abstract broadcast displayed frame is closed")
        return self._displayed_frame

    def _record_emitted_frame(self, frame: AbstractRaceFrame) -> None:
        """Record metrics and a digest without retaining an unbounded replay."""

        previous = self._last_emitted_frame
        if previous is not None:
            frame_gap = frame.tick_index - previous.tick_index
            logical_gap = frame.logical_time_s - previous.logical_time_s
            self._max_frame_gap = max(self._max_frame_gap, frame_gap)
            self._max_logical_time_gap = max(self._max_logical_time_gap, logical_gap)
            if frame_gap > 1:
                self._missing_pose_count += frame_gap - 1
            elif frame_gap <= 0:
                self._duplicate_pose_count += 1
                return
            if abs(logical_gap - BROADCAST_LOGICAL_TICK_SECONDS) > 1e-9:
                self._pose_interval_violation_count += 1
            previous_by_driver = {item.driver_id: item for item in previous.vehicles}
            for vehicle in frame.vehicles:
                old = previous_by_driver.get(vehicle.driver_id)
                if old is None:
                    continue
                world_jump_m = hypot(
                    vehicle.world_x_m - old.world_x_m,
                    vehicle.world_y_m - old.world_y_m,
                )
                position_derived_speed_mps = world_jump_m / BROADCAST_LOGICAL_TICK_SECONDS
                self._max_world_jump_m = max(self._max_world_jump_m, world_jump_m)
                self._max_position_derived_speed_mps = max(
                    self._max_position_derived_speed_mps,
                    position_derived_speed_mps,
                )
                if position_derived_speed_mps > BROADCAST_MAX_POSITION_SPEED_MPS + 1e-9:
                    self._position_speed_violation_count += 1
        payload = canonical_json(frame.to_dict()).encode("utf-8")
        self._pose_digest.update(len(payload).to_bytes(8, "big"))
        self._pose_digest.update(payload)
        self._last_emitted_frame = frame
        self._emitted_pose_count += len(frame.vehicles)

    def _append_frame(self, frame: AbstractRaceFrame) -> None:
        self.frame_buffer.append(frame)
        self._frame_buffer_peak = max(self._frame_buffer_peak, len(self.frame_buffer))
        self._record_emitted_frame(frame)

    def _pose_payload(self, frame: AbstractRaceFrame) -> dict[str, Any]:
        return {
            "type": "pose_tick",
            "physics_frame": frame.tick_index,
            "simulation_time_s": frame.logical_time_s,
            "speed_multiplier": self._speed_multiplier,
            "paused": self._paused,
            "source_mode": "abstract",
            "presentation_contract": self.presentation_contract,
            "probe_driver_id": self.probe_driver_id,
            "pose_count": len(frame.vehicles),
            "poses": [
                {
                    **vehicle.to_dict(),
                    "retired": (
                        vehicle.reliability_state == "failed"
                        or vehicle.incident_state == "terminal_damage"
                    ),
                    "hazard_active": vehicle.incident_state is not None,
                    "probe_only": False,
                    "rank_authority": (
                        "progress-v5-rank"
                        if self.authority_mode == "progress_v5"
                        else "accepted-traffic-frame"
                    ),
                    "preview_scope": (
                        "progress-v5-derived-pose"
                        if self.authority_mode == "progress_v5"
                        else "bounded-full-field-stage4"
                    ),
                }
                for vehicle in frame.vehicles
            ],
        }

    def _binary_pose_payload(self, frame: AbstractRaceFrame) -> bytes:
        """Encode the accepted abstract frame using the FULL pose wire format."""

        payload = bytearray(
            POSE_PACKET_HEADER.pack(
                POSE_PACKET_MAGIC,
                frame.tick_index,
                self._speed_multiplier,
                1 if self._paused else 0,
                len(frame.vehicles),
            )
        )
        for vehicle in frame.vehicles:
            flags = 2 if vehicle.incident_state is not None else 0
            payload.extend(POSE_DRIVER_HEADER.pack(int(vehicle.driver_id), flags, 1))
            payload.extend(
                POSE_SAMPLE.pack(
                    frame.logical_time_s,
                    frame.tick_index,
                    vehicle.world_x_m,
                    vehicle.world_y_m,
                    vehicle.heading_rad,
                )
            )
        return bytes(payload)

    def _state_payload(
        self,
        frame: AbstractRaceFrame | None,
        checkpoint=None,
    ) -> dict[str, Any]:
        checkpoint = checkpoint or self._current_checkpoint()
        logical_lap = checkpoint.lap_number if checkpoint else 0
        frame_lap = (
            max((vehicle.lap_number for vehicle in frame.vehicles), default=logical_lap)
            if frame is not None
            else logical_lap
        )
        race_control_state = getattr(self.traffic_cursor, "race_control_state", "green")
        race_control_phase = getattr(self.traffic_cursor, "race_control_phase", "green")
        race_control_remaining_s = getattr(
            self.traffic_cursor, "race_control_remaining_s", 0.0
        )
        return {
            "type": "race_state",
            "source_mode": "abstract",
            "simulation_mode": "ABSTRACT_BROADCAST",
            "session_id": self.session_id,
            "lap": min(self.total_laps, max(1, frame_lap)),
            "total_laps": self.total_laps,
            "weather": self.snapshot.environment.weather,
            "track_conditions": self.track_conditions.model_dump(mode="json"),
            "thermal_preset": self.thermal_preset,
            "track_conditions_source": self.conditions_source,
            "race_phase": race_control_state,
            "safety_car_stage": race_control_phase if race_control_state == "sc" else "none",
            "race_phase_remaining_seconds": race_control_remaining_s,
            "race_phase_remaining_laps": 0,
            "overtaking_allowed": race_control_state == "green",
            "pit_window_open": race_control_state == "sc",
            "race_started": True,
            "start_sequence_phase": "racing",
            "start_light_count": 0,
            "race_elapsed": frame.logical_time_s if frame else (checkpoint.logical_time_s if checkpoint else 0.0),
            "physics_frame": frame.tick_index if frame else (checkpoint.tick_index if checkpoint else None),
            "speed_multiplier": self._speed_multiplier,
            "paused": self._paused,
            "physics_hz": round(1.0 / BROADCAST_LOGICAL_TICK_SECONDS),
            "broadcast_hz": round(1.0 / BROADCAST_LOGICAL_TICK_SECONDS),
            "effective_speed_multiplier": 0.0 if self._paused else float(self._speed_multiplier),
            "simulation_backlog_seconds": (
                self._tick_buffer.qsize() * BROADCAST_LOGICAL_TICK_SECONDS
            ),
            "broadcast_jitter_ms": 0.0,
            "positions": self._position_payload(frame) if frame else self._logical_positions(checkpoint),
        }

    def _event_payload(self, event: LogicalEvent) -> dict[str, Any]:
        message, message_ko = _event_message(event, self._driver_names)
        return {
            "event_id": event.event_id,
            "type": event.event_type,
            "driver": self._driver_names.get(event.driver_ids[0], "") if event.driver_ids else "",
            "message": message,
            "message_ko": message_ko,
            "payload": {
                **dict(event.payload),
                "logical_time_s": event.logical_time_s,
                "source_mode": "abstract",
            },
        }

    def _event_batch_size(self, events: tuple[LogicalEvent, ...]) -> int:
        return len(
            json.dumps(
                {"type": "race_events", "events": [self._event_payload(event) for event in events]},
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")
        )

    async def _send_to_clients(self, message: dict[str, Any]) -> bool:
        dead = set()
        for ws in tuple(self.clients):
            try:
                await asyncio.wait_for(ws.send_json(message), timeout=1.0)
            except Exception:
                dead.add(ws)
        self.clients.difference_update(dead)
        return bool(self.clients)

    async def _send_pose(self, frame: AbstractRaceFrame, ws: Any | None = None) -> None:
        binary_payload = self._binary_pose_payload(frame)
        if ws is not None:
            sender = getattr(ws, "send_bytes", None)
            if sender is not None:
                await sender(binary_payload)
            else:
                await ws.send_json(self._pose_payload(frame))
        else:
            dead = set()
            json_payload = None
            for client in tuple(self.clients):
                try:
                    sender = getattr(client, "send_bytes", None)
                    if sender is not None:
                        await asyncio.wait_for(sender(binary_payload), timeout=1.0)
                    else:
                        if json_payload is None:
                            json_payload = self._pose_payload(frame)
                        await asyncio.wait_for(client.send_json(json_payload), timeout=1.0)
                except Exception:
                    dead.add(client)
            self.clients.difference_update(dead)

    async def _send_events(self, events: tuple[LogicalEvent, ...]) -> int:
        if not events:
            return 0
        index = 0
        while index < len(events):
            end = min(len(events), index + BROADCAST_MAX_EVENTS_PER_MESSAGE)
            while end > index + 1 and self._event_batch_size(events[index:end]) > BROADCAST_MAX_EVENT_PAYLOAD_BYTES:
                end -= 1
            batch = events[index:end]
            delivered = await self._send_to_clients(
                {
                    "type": "race_events",
                    "events": [self._event_payload(event) for event in batch],
                }
            )
            if not delivered:
                return index
            self._event_index += len(batch)
            index = end
        return index

    def _result_payload(self) -> dict[str, Any]:
        result = self.result
        if result is None:
            raise RuntimeError("abstract broadcast result is not finalized")
        classification = getattr(result, "classification", None)
        if classification is None:
            result_rows = [
                {
                    "position": position,
                    "driver_id": driver_id,
                    "name": self._driver_map[driver_id].abbreviation,
                    "full_name": self._driver_map[driver_id].name,
                    "team": self._team_map[self._driver_map[driver_id].team_id].name,
                    "total_time": self._finish_times.get(driver_id, 0.0),
                    "retired": False,
                }
                for position, driver_id in enumerate(result.finish_order, start=1)
            ]
        else:
            result_rows = [
                {
                    "position": item.position,
                    "driver_id": item.driver_id,
                    "name": self._driver_map[item.driver_id].abbreviation,
                    "full_name": self._driver_map[item.driver_id].name,
                    "team": self._team_map[
                        self._driver_map[item.driver_id].team_id
                    ].name,
                    "total_time": (
                        item.finish_time_s
                        if item.finish_time_s is not None
                        else item.retirement_time_s or 0.0
                    ),
                    "retired": item.status == "retired",
                    "status": item.status,
                    "retirement_reason": item.retirement_reason,
                }
                for item in classification
            ]
        return {
            "type": "race_end",
            "source_mode": "abstract",
            "simulation_mode": "ABSTRACT_BROADCAST",
            "canonical_result_hash": result.canonical_result_hash,
            "results": result_rows,
        }

    async def start_loop(self) -> None:
        if self._producer_task is None or self._producer_task.done():
            self._producer_task = asyncio.create_task(self._producer_loop())
        if self._loop_task is None or self._loop_task.done():
            self._loop_task = asyncio.create_task(self._game_loop())

    def _produce_batch_sync(
        self,
        maximum_ticks: int,
        generation: int,
    ) -> tuple[int, tuple[AbstractTrafficTick, ...], AbstractRaceResult | None]:
        with self._cursor_lock:
            cursor = self.traffic_cursor
            if cursor is None or generation != self._producer_generation:
                return generation, (), None
            ticks: list[AbstractTrafficTick] = []
            while (
                len(ticks) < maximum_ticks
                and not cursor.finished
                and not self._producer_stop.is_set()
                and not self._producer_pause.is_set()
                and generation == self._producer_generation
            ):
                ticks.append(cursor.advance_one_tick())
                if cursor.current_tick % BROADCAST_CHECKPOINT_EVERY_TICKS == 0:
                    self._cursor_checkpoints[cursor.current_tick] = (
                        cursor.export_runtime_checkpoint()
                    )
                    self._prune_runtime_checkpoints_unlocked(self._frame_index)
            result = cursor.finalize() if cursor.finished else None
            return generation, tuple(ticks), result

    async def _producer_loop(self) -> None:
        try:
            while not self._producer_stop.is_set():
                if self._producer_pause.is_set():
                    await asyncio.sleep(0.005)
                    continue
                available = self._tick_buffer.maxsize - self._tick_buffer.qsize()
                if available <= 0:
                    event = self._buffer_drained_event
                    if event is None:
                        return
                    event.clear()
                    if self._tick_buffer.full() and not self._producer_stop.is_set():
                        await event.wait()
                    continue
                batch_size = min(BROADCAST_PRODUCER_BATCH_TICKS, available)
                generation = self._producer_generation
                produced_generation, ticks, result = await asyncio.to_thread(
                    self._produce_batch_sync,
                    batch_size,
                    generation,
                )
                if self._producer_stop.is_set():
                    return
                if produced_generation != self._producer_generation:
                    continue
                for tick in ticks:
                    self._tick_buffer.put_nowait(tick)
                if (
                    self._tick_buffer.qsize() >= BROADCAST_START_BUFFER_TICKS
                    or result is not None
                ):
                    if self._buffer_ready_event is not None:
                        self._buffer_ready_event.set()
                if result is not None:
                    if (
                        self._expected_result is not None
                        and result.canonical_result_hash
                        != self._expected_result.canonical_result_hash
                    ):
                        raise RuntimeError(
                            "abstract buffered producer diverged from expected result hash"
                        )
                    self.result = result
                    self._finish_times = {
                        event.driver_ids[0]: event.logical_time_s
                        for event in result.logical_events
                        if event.event_type == "driver_finished" and event.driver_ids
                    }
                    return
                await asyncio.sleep(0)
        except BaseException as exc:
            if not isinstance(exc, asyncio.CancelledError):
                self._producer_error = exc
            if self._buffer_ready_event is not None:
                self._buffer_ready_event.set()
            raise
        finally:
            if self._producer_finished_event is not None:
                self._producer_finished_event.set()

    async def wait_until_buffered(self, timeout_seconds: float = 3.0) -> None:
        """Wait for the small startup window, never for the complete race."""

        event = self._buffer_ready_event
        if event is None:
            raise RuntimeError("abstract broadcast buffer is closed")
        await asyncio.wait_for(event.wait(), timeout=timeout_seconds)
        if self._producer_error is not None:
            raise RuntimeError("abstract broadcast producer failed") from self._producer_error

    async def _game_loop(self) -> None:
        ready_event = self._buffer_ready_event
        if ready_event is None:
            return
        await ready_event.wait()
        while not self._finished:
            if self._producer_error is not None:
                await self._send_to_clients(
                    {
                        "type": "race_error",
                        "message": "Abstract race buffer production failed",
                        "message_ko": "추상 레이스 버퍼 생성에 실패했습니다",
                    }
                )
                self._finished = True
                break
            if not self.clients or self._paused or self._command_in_progress:
                activity_event = self._activity_event
                if activity_event is None:
                    break
                activity_event.clear()
                if not self.clients or self._paused or self._command_in_progress:
                    await activity_event.wait()
                continue
            if not self._initial_checkpoint_events_drained:
                delivered_count = await self._send_events(self._initial_events)
                if delivered_count == len(self._initial_events):
                    self._initial_checkpoint_events_drained = True
                continue
            if self._pending_events:
                delivered_count = await self._send_events(tuple(self._pending_events))
                if delivered_count:
                    del self._pending_events[:delivered_count]
                if self._pending_events:
                    continue
                if self._final_frame_consumed:
                    self._race_end_sent = True
                    self._finished = True
                    await self._send_to_clients(self._result_payload())
                    break
            completed = await self.replay_clock.wait(BROADCAST_LOGICAL_TICK_SECONDS)
            if not completed:
                continue
            if not self.clients or self._paused or self._command_in_progress:
                continue
            tick = await self._tick_buffer.get()
            if self._buffer_drained_event is not None:
                self._buffer_drained_event.set()
            self._displayed_frame = tick.frame
            self._frame_index = tick.frame.tick_index
            self._append_frame(tick.frame)
            await self._send_pose(tick.frame)
            for checkpoint in tick.checkpoints:
                self._last_checkpoint = checkpoint
                self._checkpoint_index += 1
            if tick.checkpoints or tick.frame.tick_index % BROADCAST_DASHBOARD_EVERY_TICKS == 0:
                await self._send_to_clients(
                    self._state_payload(tick.frame, self._last_checkpoint)
                )
            ordered_events = tuple(
                sorted(
                    tick.logical_events,
                    key=lambda event: (round(event.logical_time_s, 9), event.event_id),
                )
            )
            self._pending_events.extend(ordered_events)
            delivered_count = await self._send_events(tuple(self._pending_events))
            if delivered_count:
                del self._pending_events[:delivered_count]
            self._final_frame_consumed = (
                self.result is not None
                and tick.frame.tick_index >= self.result.logical_tick_count
            )
            if self._pending_events:
                continue
            if self._final_frame_consumed:
                self._race_end_sent = True
                self._finished = True
                await self._send_to_clients(self._result_payload())
                break

    async def add_client(self, ws: Any) -> None:
        # Do not publish a socket until the initial snapshot has been sent.
        # React/Vite development reconnects can close a previous socket while
        # the initial race_info is still in flight; retaining that socket would
        # make every later broadcast pay the dead-client timeout.
        try:
            await ws.send_json(self.race_info)
            frame = self._current_frame()
            await self._send_pose(frame, ws=ws)
            if self._last_emitted_frame is None:
                self._append_frame(frame)
            await ws.send_json(self._state_payload(frame, self._current_checkpoint()))
            if self._finished:
                await ws.send_json(self._result_payload())
        except Exception:
            self.clients.discard(ws)
            raise
        self.clients.add(ws)
        if self._no_clients_paused:
            self._no_clients_paused = False
            if not self._paused:
                self.replay_clock.resume()
        if self._activity_event is not None:
            self._activity_event.set()

    def remove_client(self, ws: Any) -> None:
        self.clients.discard(ws)
        if not self.clients and not self._finished and not self._paused:
            self._no_clients_paused = True
            self.replay_clock.pause()
        if self._activity_event is not None:
            self._activity_event.set()

    def _prune_runtime_checkpoints_unlocked(self, displayed_tick: int) -> None:
        past = [tick for tick in self._cursor_checkpoints if tick <= displayed_tick]
        if not past:
            return
        keep = max(past)
        for tick in past:
            if tick != keep:
                del self._cursor_checkpoints[tick]

    def _resolve_player_driver_id(self, value: Any) -> int | str:
        candidates = {driver.id for driver in self.player_drivers}
        if value in candidates:
            return value
        for candidate in candidates:
            if str(candidate) == str(value):
                return candidate
        raise ValueError("not a player driver")

    def _rewind_and_apply_command_sync(
        self,
        command: str,
        payload: dict[str, Any],
    ) -> LogicalEvent:
        with self._cursor_lock:
            cursor = self.traffic_cursor
            if cursor is None:
                raise RuntimeError("abstract traffic cursor is closed")
            displayed_tick = self._frame_index
            checkpoint_ticks = [
                tick for tick in self._cursor_checkpoints if tick <= displayed_tick
            ]
            if not checkpoint_ticks:
                raise RuntimeError("no interactive checkpoint covers the displayed tick")
            checkpoint_tick = max(checkpoint_ticks)
            checkpoint = self._cursor_checkpoints[checkpoint_tick]
            previous_producer_tick = cursor.current_tick
            self._producer_generation += 1
            cursor.restore_runtime_checkpoint(checkpoint)
            while cursor.current_tick < displayed_tick:
                cursor.advance_one_tick()
            if cursor.current_tick != displayed_tick:
                raise RuntimeError("interactive rewind did not reach the displayed tick")

            driver_id = payload["driver_id"]
            if command == "set_pace_mode":
                event = cursor.set_pace_mode(driver_id, payload["pace_mode"])
            elif command == "pit_call":
                event = cursor.request_pit(
                    driver_id,
                    tire_role=payload["tire_role"],
                    physical_compound=payload["physical_compound"],
                )
            elif command == "pit_cancel":
                event = cursor.cancel_pit(driver_id)
            else:
                raise ValueError(f"unsupported interactive command: {command}")

            self._invalidated_tick_count += max(0, previous_producer_tick - displayed_tick)
            self._cursor_checkpoints = {
                displayed_tick: cursor.export_runtime_checkpoint(),
            }
            self._displayed_frame = cursor.current_frame
            self.result = None
            self._expected_result = None
            self._finish_times.clear()
            self._producer_error = None
            self._final_frame_consumed = False
            self._race_end_sent = False
            return event

    def _drain_speculative_ticks(self) -> int:
        drained = 0
        while not self._tick_buffer.empty():
            try:
                self._tick_buffer.get_nowait()
                drained += 1
            except asyncio.QueueEmpty:
                break
        if self._buffer_drained_event is not None:
            self._buffer_drained_event.set()
        return drained

    async def _apply_interactive_command(
        self,
        command: str,
        payload: dict[str, Any],
    ) -> LogicalEvent:
        async with self._command_lock:
            if self._finished:
                raise ValueError("race has already finished")
            self._command_in_progress = True
            self._producer_pause.set()
            if self._activity_event is not None:
                self._activity_event.set()
            try:
                event = await asyncio.to_thread(
                    self._rewind_and_apply_command_sync,
                    command,
                    payload,
                )
            finally:
                self._drain_speculative_ticks()
                self._producer_pause.clear()
                self._command_in_progress = False
                if self._activity_event is not None:
                    self._activity_event.set()
                producer = self._producer_task
                if producer is None or producer.done():
                    self._producer_finished_event = asyncio.Event()
                    self._producer_task = asyncio.create_task(self._producer_loop())
            await self._send_current_state()
            if self.clients and not self._initial_checkpoint_events_drained:
                delivered = await self._send_events(self._initial_events)
                self._initial_checkpoint_events_drained = delivered == len(self._initial_events)
            self._pending_events.append(event)
            self._pending_events.sort(
                key=lambda item: (round(item.logical_time_s, 9), item.event_id)
            )
            if self.clients and self._initial_checkpoint_events_drained:
                delivered = await self._send_events(tuple(self._pending_events))
                if delivered:
                    del self._pending_events[:delivered]
            return event

    async def handle_command(self, data: dict[str, Any]) -> dict[str, Any]:
        command = data.get("type", "")
        if command in {"set_pace_mode", "pit_call", "pit_cancel"}:
            try:
                driver_id = self._resolve_player_driver_id(data.get("driver_id"))
                if command == "set_pace_mode":
                    pace_mode = PaceMode(
                        str(data.get("pace_mode", PaceMode.STANDARD.value)).upper()
                    )
                    await self._apply_interactive_command(
                        command,
                        {
                            "driver_id": driver_id,
                            "pace_mode": pace_mode.value,
                        },
                    )
                    return {
                        "type": "command_ack",
                        "command": command,
                        "driver_id": driver_id,
                        "pace_mode": pace_mode.value,
                        "effective_tick": self._frame_index + 1,
                        "message": f"Pace mode set to {pace_mode.value} for driver {driver_id}",
                        "message_ko": f"드라이버 {driver_id}의 페이스 모드를 {pace_mode.value}로 설정했습니다",
                    }
                if command == "pit_call":
                    role = DryTireRole(str(data.get("tire_choice", "MEDIUM")).upper())
                    nomination = self.circuit.tire_compound_nomination
                    if nomination is None:
                        raise ValueError("circuit has no dry tire nomination")
                    physical = nomination.physical_for_role(role).value
                    await self._apply_interactive_command(
                        command,
                        {
                            "driver_id": driver_id,
                            "tire_role": role.value,
                            "physical_compound": physical,
                        },
                    )
                    return {
                        "type": "command_ack",
                        "command": command,
                        "driver_id": driver_id,
                        "tire_choice": role.value,
                        "physical_compound": physical,
                        "effective_tick": self._frame_index + 1,
                        "message": f"Pit call registered for driver {driver_id} ({role.value} · {physical})",
                        "message_ko": f"드라이버 {driver_id}의 피트콜을 접수했습니다 ({role.value} · {physical})",
                    }
                await self._apply_interactive_command(
                    command,
                    {"driver_id": driver_id},
                )
                return {
                    "type": "command_ack",
                    "command": command,
                    "driver_id": driver_id,
                    "effective_tick": self._frame_index + 1,
                    "message": f"Pit call cancelled for driver {driver_id}",
                    "message_ko": f"드라이버 {driver_id}의 피트콜을 취소했습니다",
                }
            except (TypeError, ValueError, RuntimeError) as exc:
                return {
                    "type": "command_error",
                    "command": command,
                    "message": str(exc),
                    "message_ko": f"추상 전략 명령을 처리할 수 없습니다: {exc}",
                }
        if command == "set_speed":
            try:
                multiplier = int(data.get("multiplier", 1))
            except (TypeError, ValueError):
                multiplier = 0
            if multiplier not in BROADCAST_SPEEDS:
                return {
                    "type": "command_error",
                    "command": "set_speed",
                    "message": "Abstract broadcast speed must be 1, 2 or 5",
                    "message_ko": "추상 방송 배속은 1, 2 또는 5여야 합니다",
                }
            self._speed_multiplier = multiplier
            self.replay_clock.set_speed(multiplier)
            await self._send_current_state()
            return {
                "type": "command_ack",
                "command": "set_speed",
                "multiplier": multiplier,
                "message": f"Abstract broadcast speed set to {multiplier}x",
                "message_ko": f"추상 방송 배속을 {multiplier}배로 설정했습니다",
            }
        if command == "pause":
            self._paused = True
            self.replay_clock.pause()
            await self._send_current_state()
            return {"type": "command_ack", "command": "pause", "message": "Broadcast paused", "message_ko": "방송을 일시정지했습니다"}
        if command == "resume":
            self._paused = False
            self.replay_clock.resume()
            await self._send_current_state()
            return {"type": "command_ack", "command": "resume", "message": "Broadcast resumed", "message_ko": "방송을 재개했습니다"}
        return {
            "type": "command_error",
            "message": "Unsupported abstract broadcast command",
            "message_ko": "지원하지 않는 추상 방송 명령입니다",
        }

    async def _send_current_state(self) -> None:
        if not self.clients:
            return
        await self._send_to_clients(
            self._state_payload(self._current_frame(), self._current_checkpoint())
        )

    async def close(self) -> None:
        self._finished = True
        self._producer_stop.set()
        self._producer_pause.clear()
        if self._activity_event is not None:
            self._activity_event.set()
        if self._buffer_ready_event is not None:
            self._buffer_ready_event.set()
        if self._buffer_drained_event is not None:
            self._buffer_drained_event.set()
        self.replay_clock.close()
        task = self._loop_task
        self._loop_task = None
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        producer_task = self._producer_task
        self._producer_task = None
        if producer_task is not None:
            try:
                await producer_task
            except asyncio.CancelledError:
                pass
            except Exception:
                pass
        self.replay_clock.dispose()
        self.frame_buffer.clear()
        self._pending_events.clear()
        self._cursor_checkpoints.clear()
        while not self._tick_buffer.empty():
            try:
                self._tick_buffer.get_nowait()
            except asyncio.QueueEmpty:
                break
        self._last_emitted_frame = None
        if self.traffic_cursor is not None:
            self.traffic_cursor.dispose()
        self.traffic_cursor = None
        self.replay_cursor = None
        self._displayed_frame = None
        clients = tuple(self.clients)
        self.clients.clear()
        if clients:
            await asyncio.gather(
                *(ws.close(code=1001, reason="Abstract broadcast closed") for ws in clients),
                return_exceptions=True,
            )
        self._activity_event = None
        self._buffer_ready_event = None
        self._buffer_drained_event = None
        self._producer_finished_event = None


class AbstractBroadcastSessionManager:
    def __init__(self) -> None:
        self._session: AbstractBroadcastSession | None = None

    @property
    def session(self) -> AbstractBroadcastSession | None:
        return self._session

    def create_session(self, **kwargs: Any) -> AbstractBroadcastSession:
        self.clear()
        self._session = AbstractBroadcastSession(**kwargs)
        return self._session

    def clear(self) -> None:
        session = self._session
        self._session = None
        if session is not None:
            session._finished = True

    async def clear_async(self) -> None:
        session = self._session
        self._session = None
        if session is not None:
            await session.close()


abstract_broadcast_session_manager = AbstractBroadcastSessionManager()
