"""System-identification endpoints for the SysID tab.

Thin wrapper over ``robot.sysid`` (robot/sysid.py), following the router conventions:
a ``_robot(request)`` accessor, Pydantic bodies, plain dict responses, and nothing from
``hardware/`` (CLAUDE.md layering rule).

Long operations (a 6 s recording, the 12-joint screen) start in a background thread and
return immediately. The UI polls ``/live`` for samples and ``/status`` for the result.
RuntimeError (wrong state: policy armed, job running, no session) -> 409.
ValueError (bad argument) -> 400.
"""

from __future__ import annotations

import logging

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response
from pydantic import BaseModel

logger = logging.getLogger(__name__)
router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


def _call(fn, *args, **kwargs):
    try:
        return fn(*args, **kwargs)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


class SessionBody(BaseModel):
    name: str
    servo_id: int


class RunBody(BaseModel):
    trajectory: str
    kp: int
    hole: int
    rate_hz: float = 250.0
    force_robot_servo: bool = False


class ScreenBody(BaseModel):
    amplitude_deg: float = 10.0
    period_s: float = 6.0
    cycles: int = 2


@router.get("/status")
def status(request: Request):
    return _robot(request).sysid.status()


@router.post("/session")
def start_session(body: SessionBody, request: Request):
    return _call(_robot(request).sysid.start_session, body.name, body.servo_id)


@router.post("/zero")
def capture_zero(request: Request):
    return _call(_robot(request).sysid.capture_zero)


@router.post("/run")
def start_run(body: RunBody, request: Request):
    return _call(_robot(request).sysid.start_run, body.trajectory, body.kp, body.hole,
                 rate_hz=body.rate_hz, force_robot_servo=body.force_robot_servo)


@router.post("/screen")
def start_screen(body: ScreenBody, request: Request):
    return _call(_robot(request).sysid.start_screen, body.amplitude_deg, body.period_s, body.cycles)


@router.post("/abort")
def abort(request: Request):
    """Never raises: like the e-stop, an abort that can fail is not an abort."""
    try:
        return _robot(request).sysid.abort()
    except Exception:
        logger.exception("sysid abort failed")
        return {"state": "unknown"}


@router.get("/live")
def live(request: Request, since: int = 0):
    return _robot(request).sysid.live(since)


@router.get("/download")
def download(request: Request):
    mgr = _robot(request).sysid
    data = _call(mgr.session_zip)
    return Response(content=data, media_type="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="{mgr.session}.zip"'})
