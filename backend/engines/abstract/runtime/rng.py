"""Stable, independent random streams for the abstract result engine."""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

import hashlib
import json
import random
from collections.abc import Sequence
from typing import TypeVar


T = TypeVar("T")


def _master_seed_token(master_seed: int | str) -> str:
    """Encode a seed without relying on Python's process-randomized hash()."""

    if isinstance(master_seed, bool):
        raise TypeError("master_seed must be an int or str, not bool")
    if not isinstance(master_seed, (int, str)):
        raise TypeError("master_seed must be an int or str")
    return json.dumps(
        {"type": type(master_seed).__name__, "value": str(master_seed)},
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def stream_name(kind: str, entity_id: int | str | None = None) -> str:
    """Build a documented stream name from a stable kind/entity pair."""

    if not kind or not isinstance(kind, str):
        raise ValueError("stream kind must be a non-empty string")
    if entity_id is None:
        return kind
    return f"{kind}:{entity_id}"


def derive_stream_seed(master_seed: int | str, name: str) -> int:
    """Derive a stream seed with a versioned SHA-256 domain separator."""

    if not name or not isinstance(name, str):
        raise ValueError("stream name must be a non-empty string")
    material = (
        b"abstract-race-rng-v1\0"
        + _master_seed_token(master_seed).encode("utf-8")
        + b"\0"
        + name.encode("utf-8")
    )
    return int.from_bytes(hashlib.sha256(material).digest()[:16], "big")


class RNGStream:
    """A local RNG whose state is isolated from every other named stream."""

    __slots__ = ("name", "seed", "_random")

    def __init__(self, seed: int, name: str):
        self.name = name
        self.seed = seed
        self._random = random.Random(seed)

    def random(self) -> float:
        return self._random.random()

    def uniform(self, a: float, b: float) -> float:
        return self._random.uniform(a, b)

    def randint(self, a: int, b: int) -> int:
        return self._random.randint(a, b)

    def randrange(self, stop: int) -> int:
        return self._random.randrange(stop)

    def choice(self, values: Sequence[T]) -> T:
        if not values:
            raise IndexError("cannot choose from an empty sequence")
        return values[self._random.randrange(len(values))]

    def getstate(self) -> object:
        """Return the deterministic stream state for an interactive checkpoint."""

        return self._random.getstate()

    def setstate(self, state: object) -> None:
        """Restore a state previously returned by :meth:`getstate`."""

        self._random.setstate(state)


class IndependentRNG:
    """Registry of named streams derived from one master session seed.

    The registry caches each stream so a caller can advance one logical stream
    without changing any other stream.  The public stream names intentionally
    include stable entity identifiers, e.g. ``driver:7:pace``.
    """

    REQUIRED_STREAMS = (
        "session:weather",
        "session:track_evolution",
        "session:race_control",
        "driver:{driver_id}:pace",
        "driver:{driver_id}:mistake",
        "driver:{driver_id}:incident",
        "driver:{driver_id}:contact",
        "team:{team_id}:pit",
        "vehicle:{vehicle_id}:reliability",
        "presentation:{event_id}",
    )

    def __init__(self, master_seed: int | str):
        self.master_seed = master_seed
        self._streams: dict[str, RNGStream] = {}

    def stream(self, name: str) -> RNGStream:
        if name not in self._streams:
            self._streams[name] = RNGStream(derive_stream_seed(self.master_seed, name), name)
        return self._streams[name]

    get_stream = stream

    def driver_pace(self, driver_id: int | str) -> RNGStream:
        return self.stream(f"driver:{driver_id}:pace")

    def driver_mistake(self, driver_id: int | str) -> RNGStream:
        return self.stream(f"driver:{driver_id}:mistake")

    def driver_incident(self, driver_id: int | str) -> RNGStream:
        return self.stream(f"driver:{driver_id}:incident")

    def driver_contact(self, driver_id: int | str) -> RNGStream:
        return self.stream(f"driver:{driver_id}:contact")

    def race_control(self) -> RNGStream:
        return self.stream("session:race_control")

    def team_pit(self, team_id: int | str) -> RNGStream:
        return self.stream(f"team:{team_id}:pit")

    def vehicle_reliability(self, vehicle_id: int | str) -> RNGStream:
        return self.stream(f"vehicle:{vehicle_id}:reliability")

    def presentation(self, event_id: int | str) -> RNGStream:
        return self.stream(f"presentation:{event_id}")

    def derived_seeds(self, names: Sequence[str]) -> dict[str, int]:
        """Return stable metadata without advancing any stream."""

        return {
            name: derive_stream_seed(self.master_seed, name)
            for name in sorted(set(names))
        }

    def export_states(self) -> dict[str, object]:
        """Capture every materialized stream without creating new streams."""

        return {
            name: self._streams[name].getstate()
            for name in sorted(self._streams)
        }

    def restore_states(self, states: dict[str, object]) -> None:
        """Restore the exact materialized stream registry and its states."""

        for name in tuple(self._streams):
            if name not in states:
                del self._streams[name]
        for name in sorted(states):
            stream = self.stream(name)
            stream.setstate(states[name])
