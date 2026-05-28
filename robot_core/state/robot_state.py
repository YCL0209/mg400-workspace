"""RobotState — edge-triggered, subscribable snapshot of the arm.

Phase 1 replacement for the reference fork's module-level globals + single lock
+ busy-wait polling (CLAUDE.md anti-patterns 4 and 5). Instead:

* The latest status is an immutable :class:`RobotStateSnapshot` — readers get a
  consistent value, never a half-updated struct.
* Notification is **edge-triggered**: subscribers are called only when a watched
  field (mode / enable / error / pose) actually changes, not on every frame.
  The 30004 stream is high-frequency; notifying per frame would drown consumers.
* The subscription API (``subscribe`` -> unsubscribe callback) is the interface
  prototype meant to be exposed upward to future layers (AI agent / Web).

Concurrency model: single asyncio event loop. :meth:`update` is called from the
feedback consumer task; subscribers run inline in that same loop (keep them fast
and non-blocking). No threads, so no locks. Don't call ``update`` from another
thread without a bridge.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional

from robot_core.transport.feedback import FeedbackFrame

logger = logging.getLogger(__name__)

#: Subscriber callback: ``(snapshot, changed_field_names)``.
ChangeCallback = Callable[["RobotStateSnapshot", "frozenset[str]"], None]

#: Fields whose change triggers a notification. Integer fields use exact
#: comparison; ``tool_vector_actual`` (the float pose) uses a deadband so sensor
#: jitter on an idle arm doesn't spam notifications (see DEFAULT_POSE_DEADBAND).
WATCHED_FIELDS = ("robot_mode", "enable_status", "error_status", "tool_vector_actual")

#: A pose component must move by more than this (per component, absolute) to
#: count as a change. Units are mixed (x/y/z in mm, rx/ry/rz in deg); using one
#: band is a deliberate simplification — split per-axis later if needed.
DEFAULT_POSE_DEADBAND = 0.1


@dataclass(frozen=True)
class RobotStateSnapshot:
    """Immutable point-in-time view of the arm's status.

    ``seq`` increments once per processed frame, so callers can tell whether the
    state advanced; ``monotonic_ts`` is from :func:`time.monotonic` and is for
    staleness checks (not wall-clock time).
    """

    robot_mode: int
    enable_status: int
    error_status: int
    tool_vector_actual: tuple[float, float, float, float, float, float]
    q_actual: tuple[float, float, float, float, float, float]
    seq: int
    monotonic_ts: float

    @property
    def joints(self) -> tuple[float, float, float, float]:
        """The four MG400 joint angles (J1..J4), in degrees."""
        return self.q_actual[0], self.q_actual[1], self.q_actual[2], self.q_actual[3]

    @property
    def is_enabled(self) -> bool:
        """True when the controller reports the arm as enabled (powered)."""
        return self.enable_status == 1

    @property
    def has_error(self) -> bool:
        """True when the controller reports an active error/alarm."""
        return self.error_status == 1

    @classmethod
    def from_frame(
        cls, frame: FeedbackFrame, *, seq: int, monotonic_ts: float
    ) -> "RobotStateSnapshot":
        return cls(
            robot_mode=frame.robot_mode,
            enable_status=frame.enable_status,
            error_status=frame.error_status,
            tool_vector_actual=frame.tool_vector_actual,
            q_actual=frame.q_actual,
            seq=seq,
            monotonic_ts=monotonic_ts,
        )


class RobotState:
    """Latest snapshot + edge-triggered callback subscriptions."""

    def __init__(
        self,
        *,
        pose_deadband: float = DEFAULT_POSE_DEADBAND,
        time_source: Callable[[], float] = time.monotonic,
    ) -> None:
        self._snapshot: Optional[RobotStateSnapshot] = None
        self._seq = 0
        self._pose_deadband = pose_deadband
        self._time_source = time_source
        self._subscribers: list[ChangeCallback] = []

    @property
    def snapshot(self) -> Optional[RobotStateSnapshot]:
        """The most recent snapshot, or ``None`` before the first frame arrives."""
        return self._snapshot

    def subscribe(self, callback: ChangeCallback) -> Callable[[], None]:
        """Register ``callback`` for edge changes; returns an unsubscribe callable.

        ``callback(snapshot, changed)`` is invoked synchronously from
        :meth:`update` whenever a watched field changes (and once for the first
        frame, with all watched fields reported as changed). Keep it fast and
        non-blocking — it runs on the event loop.
        """
        self._subscribers.append(callback)

        def unsubscribe() -> None:
            try:
                self._subscribers.remove(callback)
            except ValueError:
                pass  # already removed; idempotent.

        return unsubscribe

    def update(self, frame: FeedbackFrame) -> "frozenset[str]":
        """Record a frame as the current snapshot; notify subscribers on change.

        Returns the set of changed watched-field names (empty when nothing
        watched changed, so the caller can tell an edge from a no-op frame).
        """
        previous = self._snapshot
        self._seq += 1
        snapshot = RobotStateSnapshot.from_frame(
            frame, seq=self._seq, monotonic_ts=self._time_source()
        )
        changed = self._changed_fields(previous, snapshot)
        self._snapshot = snapshot

        if changed:
            self._notify(snapshot, changed)
        return changed

    def _changed_fields(
        self, previous: Optional[RobotStateSnapshot], current: RobotStateSnapshot
    ) -> "frozenset[str]":
        if previous is None:
            return frozenset(WATCHED_FIELDS)  # first frame: everything is "new".
        return frozenset(
            name
            for name in WATCHED_FIELDS
            if self._field_changed(name, getattr(previous, name), getattr(current, name))
        )

    def _field_changed(self, name: str, previous_value, current_value) -> bool:
        """Per-field change test: deadband for the float pose, exact for ints."""
        if name == "tool_vector_actual":
            # Changed only if some component moved beyond the deadband, so sensor
            # jitter on an otherwise-idle arm does not register as a change.
            return any(
                abs(p - c) > self._pose_deadband
                for p, c in zip(previous_value, current_value)
            )
        return previous_value != current_value

    def _notify(self, snapshot: RobotStateSnapshot, changed: "frozenset[str]") -> None:
        # Iterate a copy so a subscriber may unsubscribe from within its callback.
        for callback in list(self._subscribers):
            try:
                callback(snapshot, changed)
            except Exception:  # noqa: BLE001 — one bad subscriber must not break others.
                logger.exception("RobotState subscriber raised; continuing")

    async def wait_for_change(
        self, *, timeout: Optional[float] = None
    ) -> tuple[RobotStateSnapshot, "frozenset[str]"]:
        """Await the next edge change. Convenience built on :meth:`subscribe`.

        Raises :class:`asyncio.TimeoutError` if no watched field changes within
        ``timeout`` seconds.
        """
        loop = asyncio.get_running_loop()
        future: asyncio.Future = loop.create_future()

        def _once(snapshot: RobotStateSnapshot, changed: "frozenset[str]") -> None:
            if not future.done():
                future.set_result((snapshot, changed))

        unsubscribe = self.subscribe(_once)
        try:
            return await asyncio.wait_for(future, timeout)
        finally:
            unsubscribe()
