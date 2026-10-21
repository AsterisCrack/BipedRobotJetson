"""Auto-recover after a fall: re-arm and re-start once the robot is stood upright again.

Opt-in (off at boot), and deliberately narrow:

    FAULT(tilt_exceeded) ──upright + still for settle_s──> clear, arm (2 s ramp)
        ──ARMED, countdown_s──> start                         (back to RUNNING)

* Only a tilt fault is recovered. Any other fault (estop, stale sensor, inference error, bad
  observation, overrun) disables auto and stays latched for the operator.
* PolicyRunner.disarm() and estop() disable it, so the operator's stop always wins.
* An ARMED the operator reached by hand is never auto-started; neither is the ARMED an operator
  Stop leaves behind. Only an arm this class made itself counts down.
* Tipped during the countdown (tilt over the runner's fault limit) -> tilt fault again, torque off,
  back to waiting for upright.

Runs on its own thread, never on the bus thread: arm() does blocking register audits and goal
preloads over the bus. The loop body is ``tick(now)`` so tests can drive it with fake time.

Locking: the runner takes ``runner._lock`` then this class's lock (telemetry), so this class never
calls into the runner while holding its own lock. ``tick`` decides under the lock and acts after
releasing it. The fault clear re-checks under ``runner._lock`` that auto is still on and the fault
is still a tilt, so an e-stop that lands between the decision and the action wins: the e-stop
fault is never cleared, and arm()/start() refuse from FAULT on their own.
"""

from __future__ import annotations

import logging
import math
import threading
import time

import numpy as np

from .safety import FaultReason, sensor_age, tilt_from_projected_gravity
from .state_machine import PolicyState

logger = logging.getLogger(__name__)

TICK_S = 0.05
# Gyro norm (rad/s) below which the robot counts as held still. Hand-holding wobbles a little;
# swinging it up through vertical does not pass.
STILL_GYRO_RAD_S = 0.6
# After a failed arm, wait this long before the next attempt; give up after MAX_ARM_FAILURES.
ARM_RETRY_S = 3.0
MAX_ARM_FAILURES = 3

# Operator-settable, range-checked (web/routers/policy.py AutoBody mirrors these).
SETTING_RANGES = {
    "upright_deg": (3.0, 20.0),
    "settle_s": (0.3, 5.0),
    "countdown_s": (1.0, 15.0),
}
DEFAULT_SETTINGS = {"upright_deg": 8.0, "settle_s": 1.0, "countdown_s": 5.0}


class AutoPilot:
    def __init__(self, runner):
        self._runner = runner
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None

        self.enabled = False
        self.settings = dict(DEFAULT_SETTINGS)
        self.phase = "off"
        self.recoveries = 0
        self.last_error = ""

        self._auto_armed = False
        self._upright_since: float | None = None
        self._upright_s = 0.0
        self._countdown_start: float | None = None
        self._countdown_left: float | None = None
        self._retry_at = 0.0
        self._arm_failures = 0

    # ------------------------------------------------------------------
    # Operator API
    # ------------------------------------------------------------------

    def configure(self, enabled: bool | None = None, **settings) -> dict:
        """Update settings and/or enable. Raises ValueError (nothing applied) if out of range."""
        clean = {}
        for k, v in settings.items():
            if v is None:
                continue
            if k not in SETTING_RANGES:
                raise ValueError(f"unknown auto setting {k!r}")
            v = float(v)
            lo, hi = SETTING_RANGES[k]
            if not (lo - 1e-9 <= v <= hi + 1e-9):   # also rejects NaN
                raise ValueError(f"{k}={v:g} outside the allowed range [{lo:g}, {hi:g}]")
            clean[k] = v
        with self._lock:
            self.settings.update(clean)
        if enabled is True:
            self.enable()
        elif enabled is False:
            self.disable()
        return self.status()

    def enable(self, start_thread: bool = True) -> None:
        with self._lock:
            if self.enabled:
                return
            self.enabled = True
            self.last_error = ""
            self.phase = "watching"
            self._reset_progress()
            if start_thread and (self._thread is None or not self._thread.is_alive()):
                self._thread = threading.Thread(target=self._loop, name="policy-autopilot",
                                                daemon=True)
                self._thread.start()
        logger.info("auto-recover enabled")

    def disable(self, reason: str = "") -> None:
        """Turn auto off. ``reason`` (if any) is shown to the operator. Takes only this class's
        lock, briefly, so it is safe to call first thing in an e-stop."""
        with self._lock:
            if self.enabled:
                logger.info("auto-recover disabled%s", f" ({reason})" if reason else "")
            self.enabled = False
            self.phase = "off"
            if reason:
                self.last_error = reason
            self._reset_progress()

    def status(self) -> dict:
        with self._lock:
            return {
                "enabled": self.enabled,
                "phase": self.phase,
                "upright_s": round(self._upright_s, 2),
                "countdown_left": (round(self._countdown_left, 1)
                                   if self._countdown_left is not None else None),
                "recoveries": self.recoveries,
                "last_error": self.last_error,
                "settings": dict(self.settings),
            }

    # ------------------------------------------------------------------
    # Loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        while True:
            with self._lock:
                if not self.enabled:
                    self._thread = None
                    return
            try:
                self.tick(time.monotonic())
            except Exception as exc:     # never let the thread die silently mid-recovery
                logger.exception("autopilot tick failed")
                self.disable(f"autopilot error: {exc}")
            time.sleep(TICK_S)

    def tick(self, now: float) -> None:
        action = self._decide(now)
        if action == "rearm":
            self._rearm(now)
        elif action == "start":
            self._start()
        elif action is not None and action[0] == "tip":
            self._runner.fault(FaultReason.TILT, action[1])

    def _decide(self, now: float):
        """Pick the next action from the runner state. Under this class's lock; never calls the runner
        except for read-only state (state machine, cached bus state, limits)."""
        runner = self._runner
        with self._lock:
            if not self.enabled:
                self.phase = "off"
                return None
            sm = runner.sm
            state = sm.state

            if state == PolicyState.FAULT:
                self._auto_armed = False
                self._countdown_start = self._countdown_left = None
                reason, _ = sm.fault
                if reason != FaultReason.TILT:
                    self.enabled = False      # disable() without re-taking anything
                    self.phase = "off"
                    self.last_error = f"{reason} fault needs the operator"
                    self._reset_progress()
                    logger.info("auto-recover disabled (%s)", self.last_error)
                    return None
                self.phase = "wait_upright"
                return self._wait_upright(now)

            if state == PolicyState.ARMING:
                self.phase = "arming"
                return None

            if state == PolicyState.ARMED and self._auto_armed:
                self.phase = "countdown"
                rl = runner._robot._bus_manager.get_rl_state()
                tilt = tilt_from_projected_gravity(rl["projected_gravity"])
                if tilt > runner.limits.tilt_fault_rad:
                    self._auto_armed = False
                    self._countdown_start = self._countdown_left = None
                    return ("tip", f"tipped during auto countdown ({math.degrees(tilt):.1f} deg)")
                if self._countdown_start is None:
                    self._countdown_start = now
                self._countdown_left = max(
                    0.0, self.settings["countdown_s"] - (now - self._countdown_start))
                if self._countdown_left <= 0.0:
                    self._auto_armed = False
                    self._countdown_start = self._countdown_left = None
                    return "start"
                return None

            # RUNNING, IDLE, or an ARMED the operator reached by hand.
            self._auto_armed = False
            self._countdown_start = self._countdown_left = None
            self._upright_since, self._upright_s = None, 0.0
            self.phase = "watching"
            return None

    def _wait_upright(self, now: float):
        if now < self._retry_at:
            self._upright_since, self._upright_s = None, 0.0
            return None
        rl = self._runner._robot._bus_manager.get_rl_state()
        if not self._is_upright(rl):
            self._upright_since, self._upright_s = None, 0.0
            return None
        if self._upright_since is None:
            self._upright_since = now
        self._upright_s = now - self._upright_since
        if self._upright_s < self.settings["settle_s"]:
            return None
        self._upright_since, self._upright_s = None, 0.0
        return "rearm"

    def _is_upright(self, rl: dict) -> bool:
        if rl.get("servo_missing"):
            return False
        # Stamps are time.monotonic(); compare against the real clock, not the tick's ``now``.
        age = sensor_age(time.monotonic(), rl.get("servo_stamp"), rl.get("imu_stamp"))
        if age > self._runner.limits.max_sensor_age_s:
            return False
        tilt = tilt_from_projected_gravity(rl["projected_gravity"])
        if not tilt < math.radians(self.settings["upright_deg"]):
            return False
        gyro = np.asarray(rl.get("angular_vel", (0.0, 0.0, 0.0)), dtype=np.float64)
        return bool(np.linalg.norm(gyro) < STILL_GYRO_RAD_S)

    # ------------------------------------------------------------------
    # Actions (no autopilot lock held)
    # ------------------------------------------------------------------

    def _rearm(self, now: float) -> None:
        runner = self._runner
        # Atomic with respect to estop()/disarm(): both disable auto before taking runner._lock.
        with runner._lock:
            if not self.enabled or runner.sm.state != PolicyState.FAULT \
                    or runner.sm.fault[0] != FaultReason.TILT:
                return
            runner.clear_fault()
        logger.info("auto-recover: robot upright, re-arming")
        try:
            runner._robot.attach_policy_hook()
            runner.arm()
        except Exception as exc:
            with self._lock:
                if not self.enabled:
                    return
                self._arm_failures += 1
                self.last_error = f"arm failed: {exc}"
                failures = self._arm_failures
            logger.warning("auto-recover arm failed (%d/%d): %s", failures, MAX_ARM_FAILURES, exc)
            if failures >= MAX_ARM_FAILURES:
                self.disable(f"gave up after {failures} failed arms: {exc}")
                return
            # clear_fault() left us in IDLE; back to FAULT(tilt) so the next ticks wait for
            # upright again (after the back-off) instead of treating it as an operator IDLE.
            runner.fault(FaultReason.TILT, "auto re-arm failed, waiting to retry")
            with self._lock:
                self._retry_at = now + ARM_RETRY_S
            return
        with self._lock:
            if not self.enabled:
                return
            self._arm_failures = 0
            self.last_error = ""
            self._auto_armed = True
            self._countdown_start = None
            self.phase = "arming"

    def _start(self) -> None:
        if not self.enabled:
            return
        try:
            self._runner.start()     # refuses on its own if an e-stop / disarm got there first
        except Exception as exc:
            self.disable(f"auto start failed: {exc}")
            return
        with self._lock:
            self.recoveries += 1
            self.phase = "watching"
        logger.info("auto-recover: policy restarted (#%d)", self.recoveries)

    def _reset_progress(self) -> None:
        self._auto_armed = False
        self._upright_since, self._upright_s = None, 0.0
        self._countdown_start = self._countdown_left = None
        self._retry_at = 0.0
        self._arm_failures = 0
