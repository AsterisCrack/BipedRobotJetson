from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


# ------------------------------------------------------------------
# Request models
# ------------------------------------------------------------------

class IKCmd(BaseModel):
    x: float
    y: float
    z: float
    execute: bool = True   # if True, send motion command on success

class FKCmd(BaseModel):
    angles_deg: list[float]


# ------------------------------------------------------------------
# Endpoints
# ------------------------------------------------------------------

@router.get("/fk/{leg}")
def forward_kinematics(leg: str, request: Request) -> dict:
    _validate_leg(leg)
    return _robot(request).get_foot_position(leg)


@router.post("/fk/{leg}")
def forward_kinematics_from_angles(leg: str, cmd: FKCmd, request: Request) -> dict:
    _validate_leg(leg)
    if len(cmd.angles_deg) != 6:
        raise HTTPException(400, "Exactly 6 joint angles required")
    return _robot(request).compute_fk(leg, cmd.angles_deg)


@router.post("/ik/{leg}")
def inverse_kinematics(leg: str, cmd: IKCmd, request: Request) -> dict:
    _validate_leg(leg)
    robot = _robot(request)
    result = (
        robot.set_foot_position(leg, cmd.x, cmd.y, cmd.z)
        if cmd.execute
        else robot.compute_ik(leg, cmd.x, cmd.y, cmd.z)
    )
    return {
        "success": result.success,
        "angles_deg": result.angles_deg,
        "position_error_m": result.position_error_m,
        "message": result.message,
    }


@router.get("/poses")
def list_poses(request: Request) -> list[str]:
    return _robot(request).list_pose_names()


@router.post("/poses/{name}")
def execute_pose(name: str, request: Request):
    robot = _robot(request)
    try:
        robot.go_to_pose(name)
        return {"ok": True, "pose": name}
    except KeyError:
        raise HTTPException(404, f"Pose {name!r} not found")


@router.post("/home")
def go_home(request: Request):
    robot = _robot(request)
    robot.go_to_pose(robot.home_pose_name())
    return {"ok": True, "pose": robot.home_pose_name()}


# ------------------------------------------------------------------

def _validate_leg(leg: str) -> None:
    if leg not in ("left", "right"):
        raise HTTPException(400, "leg must be 'left' or 'right'")
