from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


class LoadCmd(BaseModel):
    weights_path: str
    submodule_path: str
    config_path: str | None = None
    action_scale_deg: float = 40.0


@router.get("/status")
def get_status(request: Request) -> dict:
    robot = _robot(request)
    if robot.policy is None:
        return {"loaded": False, "state": "idle", "vx": 0.0, "vy": 0.0, "wz": 0.0}
    vx, vy, wz = robot.policy.get_command()
    return {
        "loaded": True,
        "state": robot.policy.state,
        "vx": vx,
        "vy": vy,
        "wz": wz,
    }


@router.post("/load")
def load_policy(cmd: LoadCmd, request: Request) -> dict:
    try:
        _robot(request).load_policy(
            weights_path=cmd.weights_path,
            submodule_path=cmd.submodule_path,
            config_path=cmd.config_path,
            action_scale_deg=cmd.action_scale_deg,
        )
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True}


@router.post("/enable")
def enable_policy(request: Request) -> dict:
    robot = _robot(request)
    if robot.policy is None:
        raise HTTPException(status_code=400, detail="No policy loaded")
    robot.policy.enable()
    return {"ok": True, "state": robot.policy.state}


@router.post("/disable")
def disable_policy(request: Request) -> dict:
    robot = _robot(request)
    if robot.policy is None:
        raise HTTPException(status_code=400, detail="No policy loaded")
    robot.policy.disable()
    return {"ok": True, "state": robot.policy.state}
