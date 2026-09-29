"""Lifecycle state for the policy runner.

    IDLE ──load──> IDLE ──arm──> ARMING ──(ramp done)──> ARMED ──start──> RUNNING
      ^                             │                      │                │
      └──────── disarm ─────────────┴──────────────────────┴────────────────┘
      ^                                                                     │
      └──────────────── FAULT <──── (fall / estop / error) ─────────────────┘

FAULT is latched and only the operator clears it. Nothing here ever transitions
back into motion on its own -- a robot that starts moving unprompted while someone
has their hands near it is the failure mode worth designing hardest against.

ARMING exists so torque comes on and the robot reaches standing under a slow ramp,
rather than snapping there the instant the policy starts.
"""

from __future__ import annotations

import threading
import time
from enum import Enum


class PolicyState(str, Enum):
    IDLE = "idle"        # no model loaded, or loaded and torque off
    ARMING = "arming"    # torque on, ramping to standing
    ARMED = "armed"      # holding standing, policy NOT stepping
    RUNNING = "running"  # policy driving the robot
    FAULT = "fault"      # latched; requires explicit operator clear


_ALLOWED = {
    PolicyState.IDLE:    {PolicyState.ARMING, PolicyState.FAULT},
    PolicyState.ARMING:  {PolicyState.ARMED, PolicyState.IDLE, PolicyState.FAULT},
    PolicyState.ARMED:   {PolicyState.RUNNING, PolicyState.IDLE, PolicyState.FAULT},
    PolicyState.RUNNING: {PolicyState.ARMED, PolicyState.IDLE, PolicyState.FAULT},
    PolicyState.FAULT:   {PolicyState.IDLE},
}


class StateMachine:
    """Thread-safe. The bus thread faults; the web thread commands."""

    def __init__(self):
        self._lock = threading.RLock()
        self._state = PolicyState.IDLE
        self._since = time.monotonic()
        self._fault_reason: str | None = None
        self._fault_detail: str = ""
        self._log: list[dict] = []

    @property
    def state(self) -> PolicyState:
        with self._lock:
            return self._state

    @property
    def seconds_in_state(self) -> float:
        with self._lock:
            return time.monotonic() - self._since

    @property
    def fault(self) -> tuple[str | None, str]:
        with self._lock:
            return self._fault_reason, self._fault_detail

    def is_moving(self) -> bool:
        """True when the robot may be commanded, i.e. torque is expected on."""
        with self._lock:
            return self._state in (PolicyState.ARMING, PolicyState.ARMED, PolicyState.RUNNING)

    def can(self, target: PolicyState) -> bool:
        with self._lock:
            return target in _ALLOWED[self._state]

    def to(self, target: PolicyState, reason: str = "") -> bool:
        """Attempt a transition. Returns False if not allowed from the current state."""
        with self._lock:
            if target not in _ALLOWED[self._state]:
                return False
            self._record(self._state, target, reason)
            self._state = target
            self._since = time.monotonic()
            if target != PolicyState.FAULT:
                self._fault_reason, self._fault_detail = None, ""
            return True

    def fault_now(self, reason: str, detail: str = "") -> None:
        """Force FAULT from any state. Always succeeds -- safety must not be refusable."""
        with self._lock:
            if self._state == PolicyState.FAULT:
                return  # keep the first reason; it explains the rest
            self._record(self._state, PolicyState.FAULT, f"{reason}: {detail}")
            self._state = PolicyState.FAULT
            self._since = time.monotonic()
            self._fault_reason = reason
            self._fault_detail = detail

    def clear_fault(self) -> bool:
        with self._lock:
            if self._state != PolicyState.FAULT:
                return False
            self._record(self._state, PolicyState.IDLE, "operator cleared")
            self._state = PolicyState.IDLE
            self._since = time.monotonic()
            self._fault_reason, self._fault_detail = None, ""
            return True

    def recent(self, n: int = 20) -> list[dict]:
        with self._lock:
            return self._log[-n:]

    def _record(self, frm, to, reason) -> None:
        self._log.append({
            "t": time.time(),
            "from": frm.value,
            "to": to.value if hasattr(to, "value") else str(to),
            "reason": reason,
        })
        if len(self._log) > 200:
            del self._log[:-200]
