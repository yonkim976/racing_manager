#!/usr/bin/env python3
"""Measure the real RaceSession cadence at Bahrain for 1x and 2x.

The default is a short developer check.  ``--full`` runs a ten-minute total
soak (200 wall-clock seconds per speed) using the same authoritative session
loop and WebSocket serialization path as the application.
"""

from __future__ import annotations

import argparse
import asyncio
from collections import Counter
import json
import resource
import statistics
import struct
import sys
from contextlib import suppress
from dataclasses import dataclass, field
from math import hypot
from pathlib import Path
from typing import Any

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits, load_drivers, load_teams
from session import BROADCAST_INTERVAL, RaceSession
from simulation.race_engine import RaceEngine


@dataclass(eq=False)
class TickCollector:
    ticks: list[dict[str, float]] = field(default_factory=list)
    trajectory_samples: int = 0
    trajectory_frames_monotonic: bool = True
    maximum_trajectory_frame_gap: int = 0
    maximum_trajectory_step_m: float = 0.0
    maximum_trajectory_step_driver_id: int = 0
    maximum_trajectory_step_frame: int = 0
    maximum_trajectory_step_in_pit: bool = False
    maximum_trajectory_step_contact: bool = False
    maximum_trajectory_step_speed_kph: float = 0.0
    maximum_trajectory_step_previous_progress: float = 0.0
    maximum_trajectory_step_progress: float = 0.0
    maximum_trajectory_step_previous_lateral_m: float = 0.0
    maximum_trajectory_step_lateral_m: float = 0.0
    planner_generation_ms: list[float] = field(default_factory=list)
    planner_tier_samples: Counter[int] = field(default_factory=Counter)
    planner_fallback_samples: int = 0
    planner_fallback_activations: int = 0
    contact_samples: int = 0
    track_limit_samples: int = 0
    _last_replan_count_by_driver: dict[int, int] = field(default_factory=dict)
    _fallback_active_by_driver: dict[int, bool] = field(default_factory=dict)
    _last_frame_by_driver: dict[int, int] = field(default_factory=dict)
    _last_point_by_driver: dict[int, tuple[float, float]] = field(default_factory=dict)
    _last_progress_by_driver: dict[int, float] = field(default_factory=dict)
    _last_lateral_by_driver: dict[int, float] = field(default_factory=dict)
    _speed_by_driver: dict[int, float] = field(default_factory=dict)
    _contact_by_driver: dict[int, bool] = field(default_factory=dict)

    async def send_json(self, message: dict[str, Any]) -> None:
        if message.get("type") in {"tick", "race_state"}:
            self.ticks.append(
                {
                    # The current compact dashboard intentionally omits the
                    # full race clock. Dashboard cadence is sufficient for
                    # this bounded runtime check after the first second.
                    "race_elapsed": len(self.ticks) / 4.0,
                    "effective_speed_multiplier": float(
                        message.get("effective_speed_multiplier", 0.0)
                    ),
                    "simulation_backlog_seconds": float(
                        message.get("simulation_backlog_seconds", 0.0)
                    ),
                    "broadcast_jitter_ms": float(
                        message.get("broadcast_jitter_ms", 0.0)
                    ),
                }
            )
            for position in message.get("positions", []):
                driver_id = int(position["driver_id"])
                self._speed_by_driver[driver_id] = float(
                    position.get("speed_kph", 0.0)
                )
                self._contact_by_driver[driver_id] = bool(
                    position.get("contact_active", False)
                )
            for position in message.get("positions", []):
                driver_id = int(position["driver_id"])
                planner_tier = int(position.get("planner_tier_hz", 0))
                self.planner_tier_samples[planner_tier] += 1
                fallback_active = bool(
                    position.get("planner_fallback_active", False)
                )
                self.planner_fallback_samples += int(fallback_active)
                if fallback_active and not self._fallback_active_by_driver.get(
                    driver_id,
                    False,
                ):
                    self.planner_fallback_activations += 1
                self._fallback_active_by_driver[driver_id] = fallback_active
                self.contact_samples += int(
                    bool(position.get("contact_active", False))
                )
                self.track_limit_samples += int(
                    bool(position.get("track_limits_active", False))
                )
                replan_count = int(position.get("planner_replan_count", 0))
                if replan_count > self._last_replan_count_by_driver.get(
                    driver_id,
                    0,
                ):
                    self.planner_generation_ms.append(
                        float(position.get("planner_generation_ms", 0.0))
                    )
                    self._last_replan_count_by_driver[driver_id] = replan_count
                for sample in position.get("trajectory_samples", []):
                    frame = int(sample["physics_frame"])
                    point = (
                        float(sample["world_x_m"]),
                        float(sample["world_y_m"]),
                    )
                    previous_frame = self._last_frame_by_driver.get(driver_id)
                    previous_point = self._last_point_by_driver.get(driver_id)
                    previous_progress = self._last_progress_by_driver.get(driver_id)
                    previous_lateral = self._last_lateral_by_driver.get(driver_id)
                    if previous_frame is not None:
                        frame_gap = frame - previous_frame
                        self.trajectory_frames_monotonic = (
                            self.trajectory_frames_monotonic and frame_gap > 0
                        )
                        self.maximum_trajectory_frame_gap = max(
                            self.maximum_trajectory_frame_gap,
                            frame_gap,
                        )
                    if previous_point is not None:
                        step_m = hypot(
                            point[0] - previous_point[0],
                            point[1] - previous_point[1],
                        )
                        if step_m > self.maximum_trajectory_step_m:
                            self.maximum_trajectory_step_m = step_m
                            self.maximum_trajectory_step_driver_id = driver_id
                            self.maximum_trajectory_step_frame = frame
                            self.maximum_trajectory_step_in_pit = bool(
                                sample.get("in_pit", False)
                            )
                            self.maximum_trajectory_step_contact = bool(
                                position.get("contact_active", False)
                            )
                            self.maximum_trajectory_step_speed_kph = float(
                                position.get("speed_kph", 0.0)
                            )
                            self.maximum_trajectory_step_previous_progress = float(
                                previous_progress or 0.0
                            )
                            self.maximum_trajectory_step_progress = float(
                                sample.get("progress", 0.0)
                            )
                            self.maximum_trajectory_step_previous_lateral_m = float(
                                previous_lateral or 0.0
                            )
                            self.maximum_trajectory_step_lateral_m = float(
                                sample.get("lateral_offset_m", 0.0)
                            )
                    self._last_frame_by_driver[driver_id] = frame
                    self._last_point_by_driver[driver_id] = point
                    self._last_progress_by_driver[driver_id] = float(
                        sample.get("progress", 0.0)
                    )
                    self._last_lateral_by_driver[driver_id] = float(
                        sample.get("lateral_offset_m", 0.0)
                    )
                    self.trajectory_samples += 1

    async def send_bytes(self, payload: bytes) -> None:
        """Consume the current compact F1P1 pose stream for cadence checks."""
        if len(payload) < 11 or payload[:4] != b"F1P1":
            return
        _, _, _, _, driver_count = struct.unpack_from("<4sIBBB", payload, 0)
        offset = 11
        for _ in range(driver_count):
            if offset + 4 > len(payload):
                break
            driver_id, flags, sample_count = struct.unpack_from(
                "<HBB", payload, offset
            )
            offset += 4
            retired = bool(flags & 1)
            hazard_active = bool(flags & 2)
            if retired and not hazard_active:
                offset += min(sample_count * 20, max(0, len(payload) - offset))
                continue
            for _ in range(sample_count):
                if offset + 20 > len(payload):
                    break
                _, frame, x_m, y_m, _ = struct.unpack_from(
                    "<fIfff", payload, offset
                )
                offset += 20
                previous_frame = self._last_frame_by_driver.get(driver_id)
                previous_point = self._last_point_by_driver.get(driver_id)
                if previous_frame is not None:
                    frame_gap = frame - previous_frame
                    self.trajectory_frames_monotonic = (
                        self.trajectory_frames_monotonic and frame_gap > 0
                    )
                    self.maximum_trajectory_frame_gap = max(
                        self.maximum_trajectory_frame_gap,
                        frame_gap,
                    )
                if previous_point is not None:
                    step_m = hypot(
                        x_m - previous_point[0],
                        y_m - previous_point[1],
                    )
                    if step_m > self.maximum_trajectory_step_m:
                        self.maximum_trajectory_step_m = step_m
                        self.maximum_trajectory_step_driver_id = driver_id
                        self.maximum_trajectory_step_frame = frame
                        self.maximum_trajectory_step_speed_kph = self._speed_by_driver.get(
                            driver_id,
                            0.0,
                        )
                        self.maximum_trajectory_step_contact = self._contact_by_driver.get(
                            driver_id,
                            False,
                        )
                self._last_frame_by_driver[driver_id] = frame
                self._last_point_by_driver[driver_id] = (x_m, y_m)
                self.trajectory_samples += 1


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
    return ordered[index]


async def measure_speed(
    speed: int,
    wall_seconds: float,
    circuit_id: int = 3,
    race_phase: str = "green",
) -> dict[str, float | int | bool | str]:
    drivers = load_drivers()
    teams = load_teams()
    team_map = {team.id: team for team in teams}
    circuit = next(item for item in load_circuits() if item.id == circuit_id)
    player_team = team_map[1]
    player_drivers = [driver for driver in drivers if driver.team_id == player_team.id]
    engine = RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=team_map,
        player_team_id=player_team.id,
        player_driver_ids=[driver.id for driver in player_drivers],
        seed=42,
        start_sequence_enabled=False,
    )
    if not engine.set_speed(speed):
        raise RuntimeError(f"unsupported speed: {speed}")
    if race_phase != "green":
        engine.set_race_control_phase_for_testing(race_phase)

    session = RaceSession(
        session_id=f"circuit-{circuit_id}-soak-{speed}x",
        engine=engine,
        circuit=circuit,
        player_team=player_team,
        player_drivers=player_drivers,
    )
    collector = TickCollector()
    session.clients.add(collector)  # type: ignore[arg-type]
    await session.start_loop()
    try:
        await asyncio.sleep(wall_seconds)
    finally:
        if session._loop_task is not None:
            session._loop_task.cancel()
            with suppress(asyncio.CancelledError):
                await session._loop_task

    # Ignore the first second while the cumulative cadence window stabilizes.
    stable_ticks = [
        tick
        for tick in collector.ticks
        if float(tick.get("effective_speed_multiplier", 0.0)) > 0.0
    ]
    rates = [float(tick["effective_speed_multiplier"]) for tick in stable_ticks]
    backlogs = [float(tick["simulation_backlog_seconds"]) for tick in stable_ticks]
    jitters = [float(tick["broadcast_jitter_ms"]) for tick in stable_ticks]
    median_rate = statistics.median(rates) if rates else 0.0
    p95_backlog = percentile(backlogs, 0.95)
    p95_jitter = percentile(jitters, 0.95)
    planner_p50_ms = percentile(collector.planner_generation_ms, 0.50)
    planner_p95_ms = percentile(collector.planner_generation_ms, 0.95)
    planner_p99_ms = percentile(collector.planner_generation_ms, 0.99)
    maximum_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    maximum_rss_mb = (
        maximum_rss / (1024.0 * 1024.0)
        if sys.platform == "darwin"
        else maximum_rss / 1024.0
    )
    passed = (
        len(stable_ticks) >= max(1, int((wall_seconds - 1.0) * 3))
        and median_rate >= speed * 0.90
        and median_rate <= speed * 1.10
        and p95_backlog <= 0.15
        and p95_jitter <= BROADCAST_INTERVAL * 1000.0
        and collector.trajectory_frames_monotonic
        and collector.maximum_trajectory_frame_gap <= 1
        and collector.maximum_trajectory_step_m <= 5.0
        and planner_p95_ms <= 20.0
    )
    return {
        "speed": speed,
        "race_phase": race_phase,
        "wall_seconds": wall_seconds,
        "ticks": len(stable_ticks),
        "median_effective_speed": median_rate,
        "p95_backlog_ms": p95_backlog * 1000.0,
        "p95_jitter_ms": p95_jitter,
        "final_physics_frame": engine._physics_frame,
        "trajectory_samples": collector.trajectory_samples,
        "trajectory_frames_monotonic": collector.trajectory_frames_monotonic,
        "maximum_trajectory_frame_gap": collector.maximum_trajectory_frame_gap,
        "maximum_trajectory_step_m": collector.maximum_trajectory_step_m,
        "maximum_trajectory_step_driver_id": (
            collector.maximum_trajectory_step_driver_id
        ),
        "maximum_trajectory_step_frame": collector.maximum_trajectory_step_frame,
        "maximum_trajectory_step_in_pit": collector.maximum_trajectory_step_in_pit,
        "maximum_trajectory_step_contact": collector.maximum_trajectory_step_contact,
        "maximum_trajectory_step_speed_kph": (
            collector.maximum_trajectory_step_speed_kph
        ),
        "maximum_trajectory_step_previous_progress": (
            collector.maximum_trajectory_step_previous_progress
        ),
        "maximum_trajectory_step_progress": collector.maximum_trajectory_step_progress,
        "maximum_trajectory_step_previous_lateral_m": (
            collector.maximum_trajectory_step_previous_lateral_m
        ),
        "maximum_trajectory_step_lateral_m": (
            collector.maximum_trajectory_step_lateral_m
        ),
        "planner_replans_observed": len(collector.planner_generation_ms),
        "planner_generation_p50_ms": planner_p50_ms,
        "planner_generation_p95_ms": planner_p95_ms,
        "planner_generation_p99_ms": planner_p99_ms,
        "planner_generation_max_ms": max(
            collector.planner_generation_ms,
            default=0.0,
        ),
        "planner_tier_samples": dict(
            sorted(collector.planner_tier_samples.items())
        ),
        "planner_fallback_samples": collector.planner_fallback_samples,
        "planner_fallback_activations": (
            collector.planner_fallback_activations
        ),
        "contact_samples": collector.contact_samples,
        "track_limit_samples": collector.track_limit_samples,
        "maximum_rss_mb": maximum_rss_mb,
        "passed": passed,
    }


async def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds-per-speed", type=float, default=10.0)
    parser.add_argument("--circuit-id", type=int, default=3)
    parser.add_argument(
        "--speeds",
        type=int,
        nargs="+",
        default=(1, 2),
        choices=(1, 2),
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="run a ten-minute total soak (200 seconds per speed)",
    )
    parser.add_argument(
        "--simulation-minutes",
        type=float,
        default=None,
        help="run this many simulated minutes at each selected speed",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="write the full JSON result to this path",
    )
    parser.add_argument(
        "--race-phase",
        choices=("green", "sc", "vsc"),
        default="green",
        help="start the measured session in this race-control phase",
    )
    args = parser.parse_args()
    seconds_per_speed = 200.0 if args.full else max(2.0, args.seconds_per_speed)

    results = []
    for speed in args.speeds:
        wall_seconds = (
            max(2.0, args.simulation_minutes * 60.0 / speed)
            if args.simulation_minutes is not None
            else seconds_per_speed
        )
        result = await measure_speed(
            speed,
            wall_seconds,
            circuit_id=args.circuit_id,
            race_phase=args.race_phase,
        )
        results.append(result)
        status = "PASS" if result["passed"] else "FAIL"
        print(
            f"{speed}x {args.race_phase.upper()} {status}: "
            f"effective={result['median_effective_speed']:.3f}x "
            f"backlog_p95={result['p95_backlog_ms']:.1f}ms "
            f"jitter_p95={result['p95_jitter_ms']:.1f}ms "
            f"trajectory_max_step={result['maximum_trajectory_step_m']:.2f}m"
            f"(D{result['maximum_trajectory_step_driver_id']}@"
            f"F{result['maximum_trajectory_step_frame']},"
            f"contact={result['maximum_trajectory_step_contact']},"
            f"speed={result['maximum_trajectory_step_speed_kph']:.1f}kph,"
            f"progress={result['maximum_trajectory_step_previous_progress']:.6f}->"
            f"{result['maximum_trajectory_step_progress']:.6f},"
            f"lateral={result['maximum_trajectory_step_previous_lateral_m']:.2f}->"
            f"{result['maximum_trajectory_step_lateral_m']:.2f}m) "
            f"ticks={result['ticks']} frame={result['final_physics_frame']}"
        )
        print(
            "  planner "
            f"p50={result['planner_generation_p50_ms']:.2f}ms "
            f"p95={result['planner_generation_p95_ms']:.2f}ms "
            f"p99={result['planner_generation_p99_ms']:.2f}ms "
            f"fallback_activations={result['planner_fallback_activations']} "
            f"fallback_samples={result['planner_fallback_samples']} "
            f"rss={result['maximum_rss_mb']:.1f}MB"
        )

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(
                {
                    "circuit_id": args.circuit_id,
                    "circuit": next(
                        item.name for item in load_circuits() if item.id == args.circuit_id
                    ),
                    "car_count": len(load_drivers()),
                    "race_phase": args.race_phase,
                    "simulation_minutes_per_speed": args.simulation_minutes,
                    "results": results,
                    "passed": all(bool(result["passed"]) for result in results),
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

    return 0 if all(bool(result["passed"]) for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
