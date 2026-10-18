"""Motion guard: refuse servo motion / reconfiguration while the policy owns the servos.

While the policy is ARMING, ARMED or RUNNING its 50 Hz writes own the goal registers, so a web
endpoint that moves a servo races them, and one that changes zero offsets, default positions,
PID or direction shifts the calibration under a running policy. ``Robot.policy_motion_locked()``
is the single predicate (IDLE and FAULT are not locked). Torque DISABLE and the policy's own
endpoints (arm/start/stop/disarm/estop/load/limits/safety) are never gated: those are the way out.
"""

from __future__ import annotations

from fastapi import HTTPException

MOTION_LOCK_MESSAGE = (
    "Policy is armed or running: servo motion and configuration are locked. "
    "Disarm (or e-stop) the policy first."
)

# WebSocket commands that move or reconfigure servos. set_torque is gated only when enabling.
_WS_MOTION = {"set_position", "set_pose", "set_foot_ik", "set_joints_fk", "set_pid"}


def require_motion_unlocked(robot) -> None:
    """HTTP 409 while the policy is ARMING / ARMED / RUNNING."""
    if robot.policy_motion_locked():
        raise HTTPException(status_code=409, detail=MOTION_LOCK_MESSAGE)


def ws_motion_blocked(robot, msg: dict) -> bool:
    """True if this WebSocket message must be refused because the policy owns the servos."""
    t = msg.get("type")
    gated = t in _WS_MOTION or (t == "set_torque" and bool(msg.get("enable")))
    return gated and robot.policy_motion_locked()
