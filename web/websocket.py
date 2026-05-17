from __future__ import annotations

import asyncio
import json
import logging
import math

import numpy as np

from fastapi import WebSocket, WebSocketDisconnect

logger = logging.getLogger(__name__)


class TelemetryBroadcaster:
    """
    Manages active WebSocket connections and the telemetry broadcast loop.

    The Robot telemetry thread puts frames onto `queue`.
    broadcast_loop() drains the queue and sends to all connected clients.
    Incoming WS messages are dispatched to the robot as commands.
    """

    def __init__(self, robot) -> None:
        self._robot = robot
        self._connections: set[WebSocket] = set()
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=5)

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.add(ws)
        logger.info("WS client connected (%d total)", len(self._connections))

    async def disconnect(self, ws: WebSocket) -> None:
        self._connections.discard(ws)
        logger.info("WS client disconnected (%d total)", len(self._connections))

    async def broadcast_loop(self) -> None:
        while True:
            frame = await self.queue.get()
            dead = set()
            for ws in list(self._connections):
                try:
                    await ws.send_text(json.dumps(frame))
                except Exception as exc:
                    logger.warning("Broadcast error, dropping client: %s", exc)
                    dead.add(ws)
            self._connections -= dead

    async def handle(self, ws: WebSocket) -> None:
        """Main handler for one WebSocket connection."""
        await self.connect(ws)
        try:
            while True:
                raw = await ws.receive_text()
                await self._dispatch(ws, raw)
        except WebSocketDisconnect:
            pass
        finally:
            await self.disconnect(ws)

    async def _dispatch(self, ws: WebSocket, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            await ws.send_text(json.dumps({"type": "error", "message": "Invalid JSON"}))
            return

        msg_type = msg.get("type")
        robot = self._robot

        try:
            if msg_type == "set_position":
                # raw=True → Debug tab (URDF space); raw=False → Servos tab (logical space)
                servo = robot.get_servo_by_id(msg["servo_id"])
                is_raw = bool(msg.get("raw", False))
                robot.sync_write_positions(
                    {servo.joint_name: float(msg["position_deg"])},
                    raw=is_raw,
                )
                await _ack(ws, msg_type, servo_id=msg["servo_id"])

            elif msg_type == "set_torque":
                servo = robot.get_servo_by_id(msg["servo_id"])
                servo.enable_torque() if msg["enable"] else servo.disable_torque()
                await _ack(ws, msg_type, servo_id=msg["servo_id"])

            elif msg_type == "set_pid":
                servo = robot.get_servo_by_id(msg["servo_id"])
                servo.set_pid(int(msg["p"]), int(msg["d"]), int(msg["i"]))
                await _ack(ws, msg_type, servo_id=msg["servo_id"])

            elif msg_type == "set_pose":
                robot.go_to_pose(msg["pose"])
                await _ack(ws, msg_type)

            elif msg_type == "set_foot_ik":
                leg = msg["leg"]
                x, y, z = float(msg["x"]), float(msg["y"]), float(msg["z"])
                target_rot = None
                if "roll" in msg:
                    target_rot = _rpy_to_matrix(
                        math.radians(float(msg["roll"])),
                        math.radians(float(msg["pitch"])),
                        math.radians(float(msg["yaw"])),
                    )
                loop = asyncio.get_event_loop()
                result = await loop.run_in_executor(
                    None, robot.set_foot_position, leg, x, y, z, target_rot
                )
                # result.angles_deg is URDF space; convert to logical for UI.
                logical_angles = robot.urdf_to_logical(leg, result.angles_deg)
                await ws.send_text(json.dumps({
                    "type": "ik_result",
                    "leg": leg,
                    "success": result.success,
                    "angles_deg": logical_angles,
                    "position_error_m": result.position_error_m,
                    "message": result.message,
                }))

            elif msg_type == "set_joints_fk":
                # Incoming angles are logical space (0 = default standing).
                # Convert to URDF before writing and before FK computation.
                leg = msg["leg"]
                logical_angles = msg["angles_deg"]
                names = robot.leg_joint_names(leg)
                urdf_angles = robot.logical_to_urdf(leg, logical_angles)
                robot.sync_write_positions(dict(zip(names, urdf_angles)), raw=True)
                fk = robot.compute_fk(leg, urdf_angles)
                await ws.send_text(json.dumps({
                    "type": "fk_result",
                    "leg": leg,
                    "position": fk["position"],
                    "rotation_matrix": fk["rotation_matrix"],
                }))

            else:
                await ws.send_text(json.dumps({"type": "error", "message": f"Unknown command: {msg_type}"}))

        except KeyError as exc:
            await ws.send_text(json.dumps({"type": "error", "cmd": msg_type, "message": str(exc)}))
        except Exception as exc:
            logger.exception("WS command error")
            await ws.send_text(json.dumps({"type": "error", "cmd": msg_type, "message": str(exc)}))


async def _ack(ws: WebSocket, cmd: str, **extra) -> None:
    await ws.send_text(json.dumps({"type": "ack", "cmd": cmd, "success": True, **extra}))


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Extrinsic XYZ Euler angles (radians) → 3×3 rotation matrix (Rz @ Ry @ Rx)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1, 0, 0],  [0, cr, -sr], [0, sr,  cr]])
    Ry = np.array([[cp, 0, sp], [0,  1,   0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0],  [0,  0,  1]])
    return Rz @ Ry @ Rx
