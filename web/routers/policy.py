"""RL policy control endpoints.

Follows the existing router conventions: a ``_robot(request)`` accessor, Pydantic
bodies, and plain dict responses. Goes through ``robot/`` only -- never touches
``hardware/`` directly (CLAUDE.md layering rule).

Route ordering note: the only path parameter is ``/traces/{name}``, which cannot shadow
``/traces`` (different segment count), so there is no ``/scan`` vs ``/{servo_id}``-style hazard.
"""

from __future__ import annotations

import logging
import os

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

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
    # Bounds mirror policy/safety.py. 45 deg is the angle the policy trained with (episodes ended
    # there), so the tilt fault can only be tightened from it, never loosened.
    tilt_fault_deg: float | None = Field(None, ge=10.0, le=45.0)
    max_action_rate: float | None = Field(None, ge=0.05, le=2.0)
    velocity_scale: float | None = Field(None, ge=0.0, le=1.0)
    max_consecutive_overruns: int | None = None


class AutoBody(BaseModel):
    # Bounds mirror policy/autopilot.py SETTING_RANGES.
    enabled: bool | None = None
    upright_deg: float | None = Field(None, ge=3.0, le=20.0)
    settle_s: float | None = Field(None, ge=0.3, le=5.0)
    countdown_s: float | None = Field(None, ge=1.0, le=15.0)


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
    try:
        return runner.set_limits(**kw)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))


@router.get("/auto")
def get_auto(request: Request):
    return _runner(request).auto.status()


@router.post("/auto")
def set_auto(body: AutoBody, request: Request):
    """Auto-recover after falls: settings and/or enable. Enabling attaches the policy hook so
    a later re-arm can run even if this process never armed by hand."""
    robot = _robot(request)
    runner = _runner(request)
    if body.enabled:
        robot.attach_policy_hook()
    try:
        return runner.auto.configure(enabled=body.enabled, upright_deg=body.upright_deg,
                                     settle_s=body.settle_s, countdown_s=body.countdown_s)
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))


@router.get("/transitions")
def transitions(request: Request):
    """Recent state transitions -- the first thing to look at after a fall."""
    return {"transitions": _runner(request).sm.recent(30)}


@router.get("/traces")
def list_traces(request: Request):
    """Recorded policy runs (data/traces/*.npz), newest first. See policy/trace.py for the format."""
    runner = _runner(request)
    return {"root": runner.traces.root, "recording": runner.traces.active,
            "traces": runner.traces.list()}


@router.get("/traces/{name}")
def download_trace(name: str, request: Request):
    path = _runner(request).traces.path(name)
    if path is None:
        raise HTTPException(status_code=404, detail=f"no trace named {name!r}")
    return FileResponse(path, media_type="application/octet-stream", filename=name)
