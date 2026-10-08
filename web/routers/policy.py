"""RL policy control endpoints.

Follows the existing router conventions: a ``_robot(request)`` accessor, Pydantic
bodies, and plain dict responses. Goes through ``robot/`` only -- never touches
``hardware/`` directly (CLAUDE.md layering rule).

Route ordering note: this router has no path parameters, so it does not have the
``/scan`` vs ``/{servo_id}`` shadowing hazard the servos router does.
"""

from __future__ import annotations

import logging
import os

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

logger = logging.getLogger(__name__)
router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


def _runner(request: Request):
    runner = _robot(request).policy
    if runner is None:
        raise HTTPException(
            status_code=503,
            detail="policy support unavailable -- is onnxruntime installed?")
    return runner


class LoadBody(BaseModel):
    name: str


class CommandBody(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class LimitsBody(BaseModel):
    tilt_fault_deg: float | None = None
    max_action_rate: float | None = None
    velocity_scale: float | None = None
    max_consecutive_overruns: int | None = None


@router.get("/models")
def list_models(request: Request):
    """Discover bundles under models/. Invalid ones are listed with their error
    rather than hidden, so a bad export is visible in the UI."""
    from policy import discover_bundles
    runner = _robot(request).policy
    root = runner.models_root if runner else os.path.join(os.getcwd(), "models")
    return {"models_root": root, "models": discover_bundles(root)}


@router.get("/status")
def status(request: Request):
    runner = _robot(request).policy
    if runner is None:
        return {"available": False, "state": "unavailable"}
    return {"available": True, **runner.status()}


@router.post("/load")
def load(body: LoadBody, request: Request):
    runner = _runner(request)
    path = os.path.join(runner.models_root, body.name)
    if not os.path.isdir(path):
        raise HTTPException(status_code=404, detail=f"no model named {body.name!r}")
    try:
        return runner.load(path)
    except Exception as exc:
        logger.exception("model load failed")
        raise HTTPException(status_code=400, detail=str(exc))


@router.post("/arm")
def arm(request: Request):
    """Torque on and ramp to standing. Does NOT start the policy."""
    robot = _robot(request)
    runner = _runner(request)
    try:
        robot.attach_policy_hook()
        return runner.arm()
    except Exception as exc:
        raise HTTPException(status_code=409, detail=str(exc))


@router.post("/start")
def start(request: Request):
    """Hand control to the policy. Only valid from ARMED."""
    try:
        return _runner(request).start()
    except Exception as exc:
        raise HTTPException(status_code=409, detail=str(exc))


@router.post("/stop")
def stop(request: Request):
    """Stop stepping the policy but keep torque and hold pose."""
    return _runner(request).stop()


@router.post("/disarm")
def disarm(request: Request):
    robot = _robot(request)
    runner = _runner(request)
    result = runner.disarm()
    robot.detach_policy_hook()
    return result


@router.post("/estop")
def estop(request: Request):
    """Immediate torque cut. Deliberately never raises -- an e-stop that can fail
    with a 409 is not an e-stop."""
    robot = _robot(request)
    robot.abort_sysid()        # a running recording would keep commanding its servo
    runner = robot.policy
    if runner is None:
        try:
            robot.disable_all_torques()
        except Exception:
            logger.exception("bare torque cut failed during e-stop")
        return {"state": "unavailable", "torque": "cut"}
    result = runner.estop()
    robot.detach_policy_hook()
    return result


@router.post("/clear_fault")
def clear_fault(request: Request):
    return _runner(request).clear_fault()


@router.post("/command")
def command(body: CommandBody, request: Request):
    return _runner(request).set_command(body.vx, body.vy, body.wz)


@router.get("/safety")
def get_safety(request: Request):
    return _runner(request).safety_status()


@router.post("/safety")
def set_safety(body: LimitsBody, request: Request):
    import math
    runner = _runner(request)
    kw = {}
    if body.tilt_fault_deg is not None:
        kw["tilt_fault_rad"] = math.radians(body.tilt_fault_deg)
    if body.max_action_rate is not None:
        kw["max_action_rate"] = body.max_action_rate
    if body.velocity_scale is not None:
        kw["velocity_scale"] = body.velocity_scale
    if body.max_consecutive_overruns is not None:
        kw["max_consecutive_overruns"] = body.max_consecutive_overruns
    return runner.set_limits(**kw)


@router.get("/transitions")
def transitions(request: Request):
    """Recent state transitions -- the first thing to look at after a fall."""
    return {"transitions": _runner(request).sm.recent(30)}
