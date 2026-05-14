from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from robot import SerialBusError, ServoStatus

router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


# ------------------------------------------------------------------
# Request / response models
# ------------------------------------------------------------------

class PositionCmd(BaseModel):
    deg: float
    speed: int | None = None

class TorqueCmd(BaseModel):
    enable: bool

class SpeedCmd(BaseModel):
    speed: int

class AccelCmd(BaseModel):
    accel: int

class TorqueLimitCmd(BaseModel):
    limit: int

class RawPositionCmd(BaseModel):
    steps: int
    speed: int | None = None

class PIDCmd(BaseModel):
    p: int
    d: int
    i: int

class IDCmd(BaseModel):
    new_id: int

class SyncPositionsCmd(BaseModel):
    joints: dict[str, float]   # {joint_name: degrees}
    speed: int = 300


# ------------------------------------------------------------------
# Endpoints
# ------------------------------------------------------------------

@router.get("/")
def list_servos(request: Request) -> list[dict]:
    robot = _robot(request)
    statuses = robot.get_all_statuses()
    return [_status_dict(s) for s in statuses]


@router.get("/scan")
def scan_servos(request: Request, start: int = 0, end: int = 253) -> list[dict]:
    robot = _robot(request)
    try:
        ids = robot.scan_servo_ids(start=start, end=end)
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))

    result = []
    for servo_id in ids:
        info = robot.get_servo_id_info(servo_id)
        result.append({"id": servo_id, "joint": info["joint"] if info else None})
    return result


@router.get("/{servo_id}")
def get_servo(servo_id: int, request: Request) -> dict:
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        return _status_dict(servo.get_status())
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.get("/{servo_id}/raw")
def get_servo_raw(servo_id: int, request: Request) -> dict:
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        steps = servo.get_position_steps()
        return {
            "id": servo_id,
            "raw_steps": steps,
            "raw_deg": round(servo.steps_to_deg_raw(steps), 2),
            "zero_offset_steps": servo.zero_offset_steps,
            "direction_sign": servo.direction_sign,
        }
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/position")
def set_position(servo_id: int, cmd: PositionCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        speed = cmd.speed if cmd.speed is not None else 0
        servo.set_position(cmd.deg, speed=speed)
        return {"ok": True, "servo_id": servo_id, "deg": cmd.deg, "speed": speed}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/raw_position")
def set_raw_position(servo_id: int, cmd: RawPositionCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        speed = cmd.speed if cmd.speed is not None else 0
        servo.set_position_steps(cmd.steps, speed=speed)
        return {"ok": True, "servo_id": servo_id, "steps": cmd.steps, "speed": speed}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/torque")
def set_torque(servo_id: int, cmd: TorqueCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.enable_torque() if cmd.enable else servo.disable_torque()
        return {"ok": True, "servo_id": servo_id, "enabled": cmd.enable}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/pid")
def set_pid(servo_id: int, cmd: PIDCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.set_pid(cmd.p, cmd.d, cmd.i)
        return {"ok": True, "servo_id": servo_id, "p": cmd.p, "d": cmd.d, "i": cmd.i}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/zero")
def set_zero(servo_id: int, request: Request):
    """Record current encoder position as the joint zero point."""
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.set_zero_here()
        return {"ok": True, "servo_id": servo_id, "zero_steps": servo.zero_offset_steps}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/id")
def change_id(servo_id: int, cmd: IDCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.set_id(cmd.new_id)
        robot.update_servo_id(servo_id, cmd.new_id)
        return {"ok": True, "old_id": servo_id, "new_id": cmd.new_id}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except (SerialBusError, ValueError) as exc:
        raise HTTPException(400, str(exc))


@router.post("/{servo_id}/speed")
def set_speed(servo_id: int, cmd: SpeedCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.set_speed(cmd.speed)
        return {"ok": True, "servo_id": servo_id, "speed": cmd.speed}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/accel")
def set_accel(servo_id: int, cmd: AccelCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.set_acceleration(cmd.accel)
        return {"ok": True, "servo_id": servo_id, "accel": cmd.accel}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/torque_limit")
def set_torque_limit(servo_id: int, cmd: TorqueLimitCmd, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        servo.set_torque_limit(cmd.limit)
        return {"ok": True, "servo_id": servo_id, "limit": cmd.limit}
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))


@router.post("/{servo_id}/zero_offset")
def set_zero_offset(servo_id: int, request: Request):
    robot = _robot(request)
    try:
        servo = robot.get_servo_any(servo_id)
        steps = servo.get_position_steps()
    except KeyError:
        raise HTTPException(404, f"Servo {servo_id} not found")
    except SerialBusError as exc:
        raise HTTPException(503, str(exc))

    if not _update_robot_yaml(servo_id, steps):
        raise HTTPException(404, f"Servo {servo_id} not found in config")

    servo.zero_offset_steps = steps
    servo.default_position_deg = 0.0
    return {"ok": True, "servo_id": servo_id, "zero_offset_steps": steps}


@router.post("/sync_positions")
def sync_positions(cmd: SyncPositionsCmd, request: Request):
    robot = _robot(request)
    robot.sync_write_positions(cmd.joints, speed=cmd.speed)
    return {"ok": True, "joints": cmd.joints}


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _status_dict(s: ServoStatus) -> dict:
    return {
        "id": s.servo_id,
        "joint": s.joint_name,
        "position_deg": s.position_deg,
        "raw_steps": s.raw_steps,
        "raw_deg": s.raw_deg,
        "speed": s.speed,
        "load": s.load,
        "voltage_v": s.voltage_v,
        "temperature_c": s.temperature_c,
        "torque_enabled": s.torque_enabled,
    }


def _update_robot_yaml(servo_id: int, zero_offset_steps: int) -> bool:
    path = Path(__file__).parents[2] / "config" / "robot.yaml"
    if not path.exists():
        return False
    data = yaml.safe_load(path.read_text()) or {}
    servos = data.get("servos") or []
    updated = False
    for s in servos:
        if s.get("servo_id") == servo_id:
            s["zero_offset_steps"] = int(zero_offset_steps)
            s["default_position_deg"] = 0.0
            updated = True
            break
    if not updated:
        return False

    # Atomic write: write to a temp file then rename so a crash cannot
    # leave a partially-written config.
    tmp_fd, tmp_path = tempfile.mkstemp(dir=path.parent, suffix=".tmp")
    try:
        with os.fdopen(tmp_fd, "w") as f:
            f.write(yaml.safe_dump(data, sort_keys=False))
        os.replace(tmp_path, str(path))
    except Exception:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise
    return True
