"""Measure Stage 1 ABSTRACT result retention and canonical hash costs.

It runs one race per process so ``ru_maxrss`` is scoped to the requested lap
count.  The retained collection byte count is a shallow accounting of the
immutable result/event/checkpoint containers; the record counts are the
authoritative storage metric.
"""

from __future__ import annotations

import argparse
import json
import resource
import sys
import time
from pathlib import Path
from sys import getsizeof

if __package__ in {None, ""}:
    backend_root = str(Path(__file__).resolve().parents[1])
    if backend_root not in sys.path:
        sys.path.insert(0, backend_root)

from data_loader import load_circuits, load_drivers, load_teams
from simulation.abstract import AbstractRaceEngine, AbstractSessionSnapshot


def _peak_rss_mb() -> float:
    value = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    if sys.platform == "darwin":
        return value / (1024.0 * 1024.0)
    return value / 1024.0


def _shallow_retained_bytes(result) -> int:
    """Count retained result containers without materializing canonical JSON."""

    total = (
        getsizeof(result)
        + getsizeof(result.logical_events)
        + getsizeof(result.timing_checkpoints)
        + getsizeof(result.command_log)
    )
    for event in result.logical_events:
        total += getsizeof(event) + getsizeof(event.driver_ids) + getsizeof(event.payload)
    for checkpoint in result.timing_checkpoints:
        total += (
            getsizeof(checkpoint)
            + getsizeof(checkpoint.order)
            + getsizeof(checkpoint.progress_by_driver)
        )
    return total


def run(laps: int) -> dict[str, object]:
    circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
    snapshot = AbstractSessionSnapshot.from_content(
        session_id=f"stage1-benchmark:{laps}",
        session_seed=42,
        circuit=circuit,
        drivers=load_drivers(),
        teams=load_teams(),
        content_version="benchmark-content-v1",
        ruleset_version="benchmark-rules-v1",
    )

    peak_before = _peak_rss_mb()
    started = time.perf_counter()
    result = AbstractRaceEngine(snapshot).run_race(total_laps=laps, tick_seconds=0.10)
    elapsed = time.perf_counter() - started
    peak_after = _peak_rss_mb()

    first_started = time.perf_counter()
    first_hash = result.canonical_result_hash
    first_hash_elapsed = time.perf_counter() - first_started
    second_started = time.perf_counter()
    second_hash = result.canonical_result_hash
    second_hash_elapsed = time.perf_counter() - second_started

    return {
        "lap_count": laps,
        "race_elapsed_s": round(elapsed, 6),
        "peak_python_rss_before_mb": round(peak_before, 3),
        "peak_python_rss_mb": round(peak_after, 3),
        "peak_python_rss_increase_mb": round(max(0.0, peak_after - peak_before), 3),
        "frame_count": 0,
        "vehicle_state_record_count": 0,
        "logical_tick_count": result.logical_tick_count,
        "timing_checkpoint_count": len(result.timing_checkpoints),
        "logical_event_count": len(result.logical_events),
        "shallow_retained_collection_bytes": _shallow_retained_bytes(result),
        "hash_first_elapsed_s": round(first_hash_elapsed, 6),
        "hash_second_elapsed_s": round(second_hash_elapsed, 6),
        "hash_equal": first_hash == second_hash,
        "canonical_result_hash": first_hash,
        "grid_order": list(result.grid_order),
        "finish_order": list(result.finish_order),
        "logical_event_types": sorted({event.event_type for event in result.logical_events}),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--laps", type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.laps), ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
