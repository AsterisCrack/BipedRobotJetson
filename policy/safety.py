"""Safety checks that sit between the policy and the servos.

Every check here assumes the policy may output anything at all -- a NaN, a
full-scale step, a command that would fold the robot into itself. None of these
depend on the policy behaving.

Ordering matters: fault detection runs on the OBSERVATION (before inference) so a
fall is caught even if inference then throws; action limiting runs on the OUTPUT.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


class FaultReason:
    NONE = None
    TILT = "tilt_exceeded"
    ESTOP = "estop"
    STALE_SENSOR = "stale_sensor"
    BAD_OBSERVATION = "bad_observation"
    INFERENCE_ERROR = "inference_error"
    LOOP_OVERRUN = "loop_overrun"
    OPERATOR = "operator_disarm"


# Runtime safety defaults. A bundle may tighten (never disable or loosen) them on load.
DEFAULT_MAX_ACTION_RATE = 0.5
DEFAULT_TILT_FAULT_RAD = 0.784   # ~45 deg, the training termination angle

# Operator-settable bounds (web/routers/policy.py LimitsBody mirrors these).
MAX_ACTION_RATE_RANGE = (0.05, 2.0)
VELOCITY_SCALE_RANGE = (0.0, 1.0)
TILT_FAULT_RAD_RANGE = (math.radians(10.0), math.radians(45.0))


@dataclass
class SafetyLimits:
    """Runtime caps applied regardless of what the policy asks for.

    These are intentionally separate from the policy's own joint limits: the
    affine action map already clamps into the joint box, but that box is the full
    mechanical range and says nothing about how FAST you may traverse it.
    """

    # Tilt past which we declare a fall. Default matches biped_env._tilt_exceeded
    # (0.784 rad = 45 deg) -- the policy was terminated at this angle during
    # training, so it has never learned to recover beyond it.
    tilt_fault_rad: float = DEFAULT_TILT_FAULT_RAD

    # Max change in normalized action per control step. The policy trained with an
    # action-rate penalty, so large steps are already unlikely; this catches the
    # pathological case. 2.0 would be a full -1 -> +1 swing in one tick.
    max_action_rate: float = DEFAULT_MAX_ACTION_RATE

    # Scales the velocity command before it reaches the observation. Lets an
    # operator dial the whole gait down without retraining.
    velocity_scale: float = 1.0

    # Consecutive control cycles allowed to exceed budget before faulting.
    max_consecutive_overruns: int = 10

    # Sensor data older than this is treated as stale.
    max_sensor_age_s: float = 0.15


def sensor_age(now: float, servo_stamp: float | None, imu_stamp: float | None) -> float:
    """Age in seconds of the OLDEST sensor feeding the observation.

    A missing stamp (a sensor that has never produced data) is infinitely old.
    """
    if servo_stamp is None or imu_stamp is None:
        return math.inf
    return now - min(servo_stamp, imu_stamp)


@dataclass
class SafetyState:
    fault: str | None = None
    fault_detail: str = ""
    consecutive_overruns: int = 0
    clamp_events: int = 0
    last_tilt_rad: float = 0.0
    rate_limited_steps: int = 0
    history: list = field(default_factory=list)


def select_action_rate(bundle_rate, default: float) -> float:
    """Runtime action-rate clip: the bundle value if > 0 (clamped to MAX_ACTION_RATE_RANGE), else ``default``.

    None (old bundle) and 0 (trained without an in-env limiter) both keep the safety
    default; a huge bundle value is clamped to the operator maximum (2.0 = a full swing per
    step), so a bundle can tighten or loosen the clip within that range but never disable it.
    """
    if bundle_rate is not None and float(bundle_rate) > 0.0:
        lo, hi = MAX_ACTION_RATE_RANGE
        return min(max(float(bundle_rate), lo), hi)
    return float(default)


def select_tilt_fault(bundle_tilt, default: float = DEFAULT_TILT_FAULT_RAD) -> float:
    """Runtime tilt fault: the bundle's value if it is tighter than ``default``, else ``default``.

    Never loosened: the policy was terminated at the trained angle, so a bundle asking for a
    larger one gets the default instead. None / non-finite / non-positive keep the default.
    """
    if bundle_tilt is None:
        return float(default)
    t = float(bundle_tilt)
    if not math.isfinite(t) or t <= 0.0:
        return float(default)
    return min(t, float(default))


def tilt_from_projected_gravity(projected_gravity_b) -> float:
    """Angle between the robot's up-axis and world up, in radians.

    Uses projected gravity rather than Euler angles because it is already in the
    observation, is singularity-free, and is exactly what Isaac's termination used:
    ``acos(clamp(-pg_z, -1, 1))``. The clamp matters -- normalisation round-off can
    push the component a hair outside [-1,1], and acos would return NaN, which
    compares False against any threshold and would silently suppress the fault.
    """
    pg = np.asarray(projected_gravity_b, dtype=np.float64)
    return float(math.acos(float(np.clip(-pg[2], -1.0, 1.0))))


class SafetyMonitor:
    """Stateful safety checks for one policy run."""

    def __init__(self, limits: SafetyLimits):
        self.limits = limits
        self.state = SafetyState()

    def reset(self) -> None:
        self.state = SafetyState()

    # -- pre-inference -------------------------------------------------------

    def check_observation(self, frame: np.ndarray, projected_gravity_b,
                          sensor_age_s: float | None = None) -> str | None:
        """Validate sensors BEFORE running the policy. Returns a fault or None."""
        if not np.all(np.isfinite(frame)):
            bad = int(np.count_nonzero(~np.isfinite(frame)))
            self._fault(FaultReason.BAD_OBSERVATION, f"{bad} non-finite values")
            return self.state.fault

        # A sane proprio frame is bounded: accel ~10, gyro a few rad/s, joints a few
        # rad. Anything past 100 means a unit error or a corrupt read, not a pose.
        worst = float(np.abs(frame).max())
        if worst > 100.0:
            self._fault(FaultReason.BAD_OBSERVATION, f"magnitude {worst:.1f} exceeds 100")
            return self.state.fault

        tilt = tilt_from_projected_gravity(projected_gravity_b)
        self.state.last_tilt_rad = tilt
        if tilt > self.limits.tilt_fault_rad:
            self._fault(FaultReason.TILT,
                        f"{math.degrees(tilt):.1f} deg > "
                        f"{math.degrees(self.limits.tilt_fault_rad):.1f} deg")
            return self.state.fault

        if sensor_age_s is not None and sensor_age_s > self.limits.max_sensor_age_s:
            self._fault(FaultReason.STALE_SENSOR, f"{sensor_age_s * 1000:.0f} ms old"
                            if math.isfinite(sensor_age_s) else "no data yet")
            return self.state.fault

        return None

    # -- post-inference ------------------------------------------------------

    def limit_action(self, action: np.ndarray, previous_action: np.ndarray) -> np.ndarray:
        """Clip to [-1,1] and bound the per-step change."""
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        rate = self.limits.max_action_rate
        if rate > 0:
            delta = np.clip(a - previous_action, -rate, rate)
            limited = previous_action + delta
            if not np.allclose(limited, a):
                self.state.rate_limited_steps += 1
            a = limited
        return a

    # -- loop health ---------------------------------------------------------

    def note_cycle_time(self, elapsed_s: float, budget_s: float) -> str | None:
        if elapsed_s > budget_s:
            self.state.consecutive_overruns += 1
            if self.state.consecutive_overruns > self.limits.max_consecutive_overruns:
                self._fault(
                    FaultReason.LOOP_OVERRUN,
                    f"{self.state.consecutive_overruns} consecutive cycles over "
                    f"{budget_s * 1000:.1f} ms")
                return self.state.fault
        else:
            self.state.consecutive_overruns = 0
        return None

    def trigger(self, reason: str, detail: str = "") -> None:
        self._fault(reason, detail)

    def _fault(self, reason: str, detail: str) -> None:
        # Latch the FIRST fault: it is the one that explains the others.
        if self.state.fault is None:
            self.state.fault = reason
            self.state.fault_detail = detail
