from __future__ import annotations

import asyncio
import json
import logging

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
                except Exception:
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
                servo = robot.get_servo_by_id(msg["servo_id"])
                servo.set_position(float(msg["position_deg"]))
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
                result = robot.set_foot_position(leg, float(msg["x"]), float(msg["y"]), float(msg["z"]))
                await ws.send_text(json.dumps({
                    "type": "ik_result",
                    "leg": leg,
                    "success": result.success,
                    "angles_deg": result.angles_deg,
                    "position_error_m": result.position_error_m,
                    "message": result.message,
                }))

            elif msg_type == "set_joints_fk":
                leg = msg["leg"]
                angles = msg["angles_deg"]
                from robot.kinematics.chain import KinematicChain
                names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
                joint_angles = dict(zip(names, angles))
                robot.sync_write_positions(joint_angles)
                fk = robot.compute_fk(leg, angles)
                await ws.send_text(json.dumps({
                    "type": "fk_result",
                    "leg": leg,
                    "position": fk["position"],
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
