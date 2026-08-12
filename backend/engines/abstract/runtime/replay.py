"""Bounded presentation replay storage for ABSTRACT broadcast sessions."""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

import asyncio
import time
from collections import deque
from collections.abc import Callable, Iterable

from .state import AbstractRaceFrame, AbstractTimingCheckpoint, LogicalEvent


class RollingFrameBuffer:
    """A bounded, non-authoritative frame window.

    The result engine may send presentation frames here when a live presenter
    needs them.  The buffer is deliberately separate from
    :class:`AbstractRaceResult`: dropping old frames never changes the logical
    result or its hash.
    """

    def __init__(self, max_frames: int = 128) -> None:
        if max_frames < 1:
            raise ValueError("max_frames must be positive")
        self.max_frames = max_frames
        self._frames: deque[AbstractRaceFrame] = deque(maxlen=max_frames)

    def append(self, frame: AbstractRaceFrame) -> None:
        self._frames.append(frame)

    def extend(self, frames: Iterable[AbstractRaceFrame]) -> None:
        for frame in frames:
            self.append(frame)

    def snapshot(self) -> tuple[AbstractRaceFrame, ...]:
        return tuple(self._frames)

    @property
    def latest(self) -> AbstractRaceFrame | None:
        return self._frames[-1] if self._frames else None

    def __len__(self) -> int:
        return len(self._frames)

    def clear(self) -> None:
        self._frames.clear()


class LogicalReplayCursor:
    """Ordered cursor over authoritative checkpoints and logical events."""

    def __init__(
        self,
        checkpoints: Iterable[AbstractTimingCheckpoint],
        events: Iterable[LogicalEvent],
    ) -> None:
        self.checkpoints = tuple(checkpoints)
        self.events = tuple(events)
        if not self.checkpoints:
            raise ValueError("logical replay requires at least one checkpoint")
        try:
            self._checkpoint_index = next(
                index
                for index, checkpoint in enumerate(self.checkpoints)
                if checkpoint.checkpoint_kind == "race_start"
            )
        except StopIteration as exc:
            raise ValueError("logical replay requires a race_start checkpoint") from exc
        if self.checkpoints[-1].checkpoint_kind != "race_finish":
            raise ValueError("logical replay requires a final race_finish checkpoint")
        self._event_index = 0

    @property
    def checkpoint_index(self) -> int:
        return self._checkpoint_index

    @property
    def event_index(self) -> int:
        return self._event_index

    @property
    def current_checkpoint(self) -> AbstractTimingCheckpoint:
        return self.checkpoints[self._checkpoint_index]

    @property
    def is_final_checkpoint(self) -> bool:
        return self._checkpoint_index == len(self.checkpoints) - 1

    @property
    def events_complete(self) -> bool:
        return self._event_index == len(self.events)

    def restore_event_index(self, event_index: int) -> None:
        """Rollback an undelivered batch when every client disconnected."""

        if not 0 <= event_index <= len(self.events):
            raise ValueError("event_index is outside the replay event range")
        self._event_index = event_index

    @property
    def complete(self) -> bool:
        return self.is_final_checkpoint and self.events_complete

    def advance_checkpoint(self) -> AbstractTimingCheckpoint:
        if not self.is_final_checkpoint:
            self._checkpoint_index += 1
        return self.current_checkpoint

    def next_event_batch_until(
        self,
        logical_time_s: float,
        *,
        max_events: int,
        max_payload_bytes: int,
        encoded_size: Callable[[tuple[LogicalEvent, ...]], int],
    ) -> tuple[LogicalEvent, ...]:
        """Consume one bounded, ordered event batch up to a checkpoint time."""

        if max_events < 1 or max_payload_bytes < 1:
            raise ValueError("event batch limits must be positive")
        batch: list[LogicalEvent] = []
        while self._event_index < len(self.events):
            event = self.events[self._event_index]
            if event.logical_time_s > logical_time_s + 1e-9:
                break
            candidate = tuple((*batch, event))
            if batch and (
                len(candidate) > max_events
                or encoded_size(candidate) > max_payload_bytes
            ):
                break
            batch.append(event)
            self._event_index += 1
            if len(batch) >= max_events:
                break
            if encoded_size(tuple(batch)) > max_payload_bytes:
                # A single oversized event is sent alone rather than dropped.
                break
        return tuple(batch)


class InterruptibleReplayClock:
    """Wall-clock wait that can be interrupted by speed/pause/close commands."""

    def __init__(self, speed_multiplier: int = 1) -> None:
        if speed_multiplier < 1:
            raise ValueError("speed_multiplier must be positive")
        self.speed_multiplier = speed_multiplier
        self.paused = False
        self.closed = False
        self.remaining_logical_s = 0.0
        self._wake_event: asyncio.Event | None = asyncio.Event()
        self._last_wall_time = 0.0
        self._waiting = False

    @property
    def waiting(self) -> bool:
        return self._waiting

    def _consume_elapsed(self, now: float | None = None) -> None:
        if not self._waiting:
            return
        current = time.monotonic() if now is None else now
        elapsed_wall_s = max(0.0, current - self._last_wall_time)
        self._last_wall_time = current
        if not self.paused:
            self.remaining_logical_s = max(
                0.0,
                self.remaining_logical_s - elapsed_wall_s * self.speed_multiplier,
            )

    def _wake(self) -> None:
        if self._wake_event is not None and self._waiting:
            self._wake_event.set()

    def set_speed(self, speed_multiplier: int) -> None:
        if speed_multiplier < 1:
            raise ValueError("speed_multiplier must be positive")
        if self.closed:
            return
        self._consume_elapsed()
        self.speed_multiplier = speed_multiplier
        self._wake()

    def pause(self) -> None:
        if self.closed:
            return
        self._consume_elapsed()
        self.paused = True
        self._wake()

    def resume(self) -> None:
        if self.closed:
            return
        self._consume_elapsed()
        self.paused = False
        self._last_wall_time = time.monotonic()
        self._wake()

    def close(self) -> None:
        self.closed = True
        if self._wake_event is not None:
            self._wake_event.set()

    def dispose(self) -> None:
        """Release the Event reference after the owning loop has stopped."""

        self.closed = True
        self._waiting = False
        self.remaining_logical_s = 0.0
        self._wake_event = None

    async def wait(self, logical_duration_s: float) -> bool:
        """Consume a logical duration; return False when closed/cancelled."""

        if logical_duration_s < 0.0:
            raise ValueError("logical_duration_s must not be negative")
        if self.closed:
            return False
        self.remaining_logical_s = logical_duration_s
        self._last_wall_time = time.monotonic()
        self._waiting = True
        event = self._wake_event
        if event is None:
            self._waiting = False
            return False
        event.clear()
        try:
            while self.remaining_logical_s > 1e-9 and not self.closed:
                if self.paused:
                    await event.wait()
                    event.clear()
                    self._last_wall_time = time.monotonic()
                    continue
                timeout = self.remaining_logical_s / self.speed_multiplier
                try:
                    await asyncio.wait_for(event.wait(), timeout=timeout)
                except asyncio.TimeoutError:
                    self._consume_elapsed()
                    break
                event.clear()
                self._consume_elapsed()
            return not self.closed and self.remaining_logical_s <= 1e-9
        finally:
            self._waiting = False
            self.remaining_logical_s = max(0.0, self.remaining_logical_s)
