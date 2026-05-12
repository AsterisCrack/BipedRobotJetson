from __future__ import annotations

import asyncio
import logging
import threading
import time

import numpy as np

from robot.config import ServoConfig, Settings
from robot.hardware.imu.bno085 import BNO085, IMUReading
from robot.hardware.serial_bus import SerialBus, SerialBusError
from robot.hardware.st3215.protocol import encode_ping, encode_sync_write, steps_to_bytes, pack_u16
from robot.hardware.st3215.registers import Reg
from robot.hardware.st3215.servo import ST3215, ServoStatus
from robot.kinematics.chain import KinematicChain
from robot.kinematics.solver import IKResult, KinematicSolver

logger = logging.getLogger(__name__)


class Robot:
    """
    Central orchestrator.  Owns all hardware objects and the telemetry loop.

    Servo objects are always built from config regardless of hardware availability,
    so the frontend always receives valid telemetry frames (with simulated/cached
    positions when the bus is offline).

    Typical lifecycle:
        robot = Robot(settings)
        robot.initialize()           # opens hardware — non-fatal if unavailable
        robot.start_telemetry(q)     # background thread → asyncio queue at 20 Hz
        ...
        robot.shutdown()
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._bus = SerialBus(
            port=settings.hardware.uart_port,
            baud_rate=settings.hardware.baud_rate,
            timeout=settings.hardware.uart_timeout_s,
        )
        self._imu = BNO085(
            i2c_bus=settings.hardware.i2c_bus,
            address=settings.hardware.i2c_address,
        )
        self._chain = KinematicChain(settings.robot.urdf_path)
        self._solver = KinematicSolver(self._chain)

        # Build servo instances from config immediately — no hardware needed.
        self._servos: dict[str, ST3215] = {}
        self._servos_by_id: dict[int, ST3215] = {}
        for cfg in settings.robot.servos:
            servo = ST3215(cfg, self._bus)
            self._servos[cfg.joint_name] = servo
            self._servos_by_id[cfg.servo_id] = servo

        # Cached positions used when hardware reads fail (sim / offline mode).
        self._cached_positions: dict[str, float] = {
            cfg.joint_name: cfg.default_position_deg for cfg in settings.robot.servos
        }

        self._telemetry_thread: threading.Thread | None = None
        self._stop_telemetry = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        try:
            self._bus.open()
        except Exception as exc:
            logger.warning("Serial bus unavailable (%s) — running without servos", exc)
            return

        # Ping all servos (non-fatal)
        for servo in self._servos.values():
            if not servo.ping():
                logger.warning("Servo %d (%s) did not respond", servo.servo_id, servo.joint_name)

        # Apply PID from config
        for servo in self._servos.values():
            try:
                servo.apply_config_pid()
            except SerialBusError as exc:
                logger.warning("PID apply failed for %s: %s", servo.joint_name, exc)

        # Safety: ensure torques are disabled on startup
        self.disable_all_torques()

        # Init IMU (non-fatal)
        try:
            self._imu.initialize()
        except Exception as exc:
            logger.warning("IMU unavailable (%s) — running without IMU", exc)

        logger.info("Robot initialised — %d servos, bus=%s", len(self._servos), self._bus.is_open)

    def shutdown(self) -> None:
        self._stop_telemetry.set()
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=2.0)
        try:
            self.disable_all_torques()
        except Exception:
            pass
        self._bus.close()
        self._imu.close()
        logger.info("Robot shut down")

    # ------------------------------------------------------------------
    # Servo access
    # ------------------------------------------------------------------

    @property
    def servos(self) -> dict[str, ST3215]:
        return self._servos

    def get_servo_by_id(self, servo_id: int) -> ST3215:
        if servo_id not in self._servos_by_id:
            raise KeyError(f"No servo with id {servo_id}")
        return self._servos_by_id[servo_id]

    def get_servo_any(self, servo_id: int) -> ST3215:
        if servo_id in self._servos_by_id:
            return self._servos_by_id[servo_id]
        cfg = ServoConfig(
            servo_id=servo_id,
            joint_name=f"unknown_{servo_id}",
            direction_sign=1,
            zero_offset_steps=2048,
            default_position_deg=0.0,
        )
        servo = ST3215(cfg, self._bus)
        self._servos_by_id[servo_id] = servo
        return servo

    def update_servo_id(self, old_id: int, new_id: int) -> None:
        if old_id in self._servos_by_id:
            servo = self._servos_by_id.pop(old_id)
            self._servos_by_id[new_id] = servo

    def scan_servo_ids(self, start: int = 0, end: int = 253) -> list[int]:
        if not self._bus.is_open:
            raise SerialBusError("Serial bus is not open")
        start = max(0, min(253, start))
        end = max(0, min(253, end))
        if end < start:
            start, end = end, start

        found: list[int] = []
        for servo_id in range(start, end + 1):
            try:
                self._bus.transfer(encode_ping(servo_id), response_data_len=0)
                found.append(servo_id)
            except SerialBusError:
                pass
        return found

    def enable_all_torques(self) -> None:
        for servo in self._servos.values():
            try:
                servo.enable_torque()
            except SerialBusError as exc:
                logger.warning("Torque enable failed for %s: %s", servo.joint_name, exc)

    def disable_all_torques(self) -> None:
        for servo in self._servos.values():
            try:
                servo.disable_torque()
            except SerialBusError as exc:
                logger.warning("Torque disable failed for %s: %s", servo.joint_name, exc)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def sync_write_positions(self, joint_angles: dict[str, float], speed: int = 300) -> None:
        """Send a SYNC_WRITE broadcast to move all listed joints simultaneously."""
        # Update cache regardless of bus state so FK/IK stay consistent
        self._cached_positions.update(joint_angles)

        if not self._bus.is_open:
            return

        servo_data: list[tuple[int, bytes]] = []
        for joint_name, deg in joint_angles.items():
            servo = self._servos.get(joint_name)
            if servo is None:
                continue
            steps = servo.deg_to_steps(deg)
            data = steps_to_bytes(steps) + pack_u16(speed)
            servo_data.append((servo.servo_id, data))

        if servo_data:
            packet = encode_sync_write(Reg.TARGET_POS_L, 4, servo_data)
            self._bus.send_no_reply(packet)

    def go_to_pose(self, pose_name: str, speed: int = 300) -> None:
        if pose_name not in self._settings.robot.poses:
            raise KeyError(f"Unknown pose: {pose_name!r}")
        self.sync_write_positions(self._settings.robot.poses[pose_name], speed=speed)

    # ------------------------------------------------------------------
    # IK / FK
    # ------------------------------------------------------------------

    def set_foot_position(self, leg: str, x: float, y: float, z: float) -> IKResult:
        current = self.get_current_joint_angles_deg(leg)
        result = self._solver.ik(leg, target_pos=np.array([x, y, z]), initial_angles_deg=current)
        if result.success:
            names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
            self.sync_write_positions(dict(zip(names, result.angles_deg)))
        return result

    def get_foot_position(self, leg: str) -> dict:
        return self._solver.fk(leg, self.get_current_joint_angles_deg(leg))

    def get_current_joint_angles_deg(self, leg: str) -> list[float]:
        names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
        angles = []
        for name in names:
            servo = self._servos.get(name)
            if servo is None or not self._bus.is_open:
                angles.append(self._cached_positions.get(name, 0.0))
                continue
            try:
                pos = servo.get_position()
                self._cached_positions[name] = pos
                angles.append(pos)
            except SerialBusError:
                angles.append(self._cached_positions.get(name, 0.0))
        return angles

    def compute_fk(self, leg: str, angles_deg: list[float]) -> dict:
        return self._solver.fk(leg, angles_deg)

    # ------------------------------------------------------------------
    # Status reads
    # ------------------------------------------------------------------

    def get_all_statuses(self) -> list[ServoStatus]:
        statuses = []
        for servo in self._servos.values():
            if not self._bus.is_open:
                # Return a mock status from cache so the frontend always gets data
                statuses.append(ServoStatus(
                    servo_id=servo.servo_id,
                    joint_name=servo.joint_name,
                    position_deg=round(self._cached_positions.get(servo.joint_name, 0.0), 2),
                        raw_steps=0,
                        raw_deg=0.0,
                    speed=0,
                    load=0,
                    voltage_v=0.0,
                    temperature_c=0,
                    torque_enabled=servo.torque_enabled,
                ))
                continue
            try:
                status = servo.get_status()
                self._cached_positions[servo.joint_name] = status.position_deg
                statuses.append(status)
            except SerialBusError as exc:
                logger.debug("Status read failed for %s: %s", servo.joint_name, exc)
                statuses.append(ServoStatus(
                    servo_id=servo.servo_id,
                    joint_name=servo.joint_name,
                    position_deg=round(self._cached_positions.get(servo.joint_name, 0.0), 2),
                    raw_steps=0,
                    raw_deg=0.0,
                    speed=0, load=0, voltage_v=0.0, temperature_c=0,
                    torque_enabled=servo.torque_enabled,
                ))
        return statuses

    def get_imu(self) -> IMUReading:
        return self._imu.read()

    # ------------------------------------------------------------------
    # Telemetry loop
    # ------------------------------------------------------------------

    def start_telemetry(
        self, queue: asyncio.Queue, loop: asyncio.AbstractEventLoop, rate_hz: float = 20.0
    ) -> None:
        self._stop_telemetry.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop,
            args=(queue, loop, 1.0 / rate_hz),
            daemon=True,
            name="telemetry",
        )
        self._telemetry_thread.start()
        logger.info("Telemetry loop started at %.0f Hz", rate_hz)

    def _telemetry_loop(
        self, queue: asyncio.Queue, loop: asyncio.AbstractEventLoop, interval: float
    ) -> None:
        while not self._stop_telemetry.is_set():
            t_start = time.monotonic()

            statuses = self.get_all_statuses()
            imu = self.get_imu()
            left_foot  = self.get_foot_position("left")
            right_foot = self.get_foot_position("right")

            frame = {
                "type": "telemetry",
                "timestamp": time.time(),
                "servos": [
                    {
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
                    for s in statuses
                ],
                "imu": {
                    "quaternion": {"w": imu.quaternion[0], "x": imu.quaternion[1],
                                   "y": imu.quaternion[2], "z": imu.quaternion[3]},
                    "euler_deg":  {"roll": imu.euler_deg[0], "pitch": imu.euler_deg[1],
                                   "yaw": imu.euler_deg[2]},
                    "accel": {"x": imu.accel[0], "y": imu.accel[1], "z": imu.accel[2]},
                    "gyro":  {"x": imu.gyro[0],  "y": imu.gyro[1],  "z": imu.gyro[2]},
                    "calibration": imu.calibration_status,
                },
                "kinematics": {
                    "left_foot":  {"x": left_foot["position"][0],  "y": left_foot["position"][1],  "z": left_foot["position"][2]},
                    "right_foot": {"x": right_foot["position"][0], "y": right_foot["position"][1], "z": right_foot["position"][2]},
                },
            }

            asyncio.run_coroutine_threadsafe(queue.put(frame), loop)

            elapsed = time.monotonic() - t_start
            time.sleep(max(0.0, interval - elapsed))
