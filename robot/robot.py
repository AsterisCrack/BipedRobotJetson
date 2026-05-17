from __future__ import annotations

import asyncio
import logging
import math
import threading
import time

import numpy as np

from hardware.config import ServoConfig
from hardware.imu.bno055 import BNO055, IMUReading
from hardware.serial_bus import SerialBus, SerialBusError
from hardware.servo_bus_manager import ServoBusManager
from hardware.st3215.protocol import encode_ping
from hardware.st3215.servo import ST3215, ServoStatus
from kinematics.chain import KinematicChain
from kinematics.solver import IKResult, KinematicSolver
from robot.config import Settings

logger = logging.getLogger(__name__)


class Robot:
    """
    Central orchestrator.  Owns all hardware objects and the telemetry loop.

    Two coordinate spaces:
      - URDF space:    0° = physical servo center after zero_offset_steps /
                       direction_sign correction.  Used by FK/IK solver and
                       servo driver internals.
      - Logical space: 0° = calibrated standing position (default_position_deg).
                       Used in poses, sliders, and telemetry display.
                       logical = urdf − default_position_deg

    All public motion commands (sync_write_positions, go_to_pose, …) accept
    logical angles by default.  Pass raw=True to bypass the offset and send
    URDF angles directly (Debug tab only).

    Typical lifecycle:
        robot = Robot(settings)
        robot.initialize()           # opens hardware — non-fatal if unavailable
        robot.start_telemetry(q)     # background thread → asyncio queue
        ...
        robot.shutdown()
    """

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._bus = SerialBus(
            port=settings.hardware.uart_port,
            baud_rate=settings.hardware.baud_rate,
            timeout=settings.hardware.uart_timeout_s,
            expect_echo=settings.hardware.expect_echo,
        )
        self._imu = BNO055(
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

        # default_position_deg offsets: logical zero → URDF angle.
        self._default_offsets: dict[str, float] = {
            cfg.joint_name: cfg.default_position_deg
            for cfg in settings.robot.servos
        }

        # Joint limits from URDF (degrees, logical space):
        #   limit_lower ≤ logical_angle ≤ limit_upper
        # "Logical" means relative to the calibrated standing position, so
        # a limit of [-120°, +120°] means ±120° from default standing.
        # In URDF space these become [default+lower, default+upper].
        self._joint_limits_deg: dict[str, tuple[float, float]] = {}
        for leg in ("left", "right"):
            names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
            for name, (lo_rad, hi_rad) in zip(names, self._chain.joint_limits(leg)):
                self._joint_limits_deg[name] = (math.degrees(lo_rad), math.degrees(hi_rad))

        # Cached URDF positions used when BusManager has no data yet.
        self._cached_positions: dict[str, float] = {
            cfg.joint_name: cfg.default_position_deg for cfg in settings.robot.servos
        }

        # When True, sync_write_positions() skips the bus write so the 3D
        # model and IK/FK still work but no commands reach the physical servos.
        self._sim_mode: bool = False

        # Bus manager: owns the serial bus in a dedicated 50 Hz thread.
        self._bus_manager = ServoBusManager(
            list(self._servos.values()), self._bus, self._imu,
            rt_scheduling=self._settings.biped_rt_scheduling,
            fast_mode=self._settings.biped_fast_mode,
        )

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
            self._bus_manager.start()
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

        # Start bus manager loop AFTER hardware is ready
        self._bus_manager.start()

        logger.info("Robot initialised — %d servos, bus=%s", len(self._servos), self._bus.is_open)

    def shutdown(self) -> None:
        self._stop_telemetry.set()
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            self._telemetry_thread.join(timeout=2.0)
        self._bus_manager.stop()
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

    def get_servo_id_info(self, servo_id: int) -> dict | None:
        """Return {'id': ..., 'joint': ...} for a known servo ID, or None."""
        servo = self._servos_by_id.get(servo_id)
        return {"id": servo_id, "joint": servo.joint_name} if servo else None

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

    def sync_write_positions(
        self, joint_angles: dict[str, float], speed: int = 300, raw: bool = False
    ) -> None:
        """
        Send target positions to the bus manager command buffer (non-blocking).

        Args:
            joint_angles: joint_name → angle in logical space (0 = default
                          standing) unless raw=True.
            speed:        servo movement speed (steps/s).
            raw:          if True, treat angles as URDF space (no offset added).
                          Use only for Debug-tab direct servo control.
        """
        # Enforce URDF joint limits before converting / forwarding.
        # Limits are stored in logical space (relative to default standing).
        # For raw=True (URDF input), shift limits by default_position_deg.
        safe: dict[str, float] = {}
        for name, angle in joint_angles.items():
            if name in self._joint_limits_deg:
                lo, hi = self._joint_limits_deg[name]
                if raw:
                    default = self._default_offsets.get(name, 0.0)
                    lo, hi = lo + default, hi + default
                clamped = max(lo, min(hi, angle))
                if abs(clamped - angle) > 0.1:
                    logger.warning(
                        "Joint %s: command %.2f° clamped to %.2f° (%s space)",
                        name, angle, clamped, "URDF" if raw else "logical",
                    )
                safe[name] = clamped
            else:
                safe[name] = angle

        if raw:
            urdf_angles = safe
        else:
            urdf_angles = {
                name: deg + self._default_offsets.get(name, 0.0)
                for name, deg in safe.items()
            }

        # Keep cache in URDF space for FK/IK warm-start (always, even in sim mode).
        self._cached_positions.update(urdf_angles)
        if not self._sim_mode:
            self._bus_manager.set_target_positions(urdf_angles, speed)

    def go_to_pose(self, pose_name: str, speed: int = 300) -> None:
        """Send robot to a named pose. Pose angles are in logical space."""
        if pose_name not in self._settings.robot.poses:
            raise KeyError(f"Unknown pose: {pose_name!r}")
        self.sync_write_positions(self._settings.robot.poses[pose_name], speed=speed)

    def go_to_defaults(self, speed: int = 300) -> None:
        """Send all joints to logical zero (= calibrated standing position)."""
        self.sync_write_positions({name: 0.0 for name in self._servos}, speed=speed)

    # ------------------------------------------------------------------
    # IK / FK
    # ------------------------------------------------------------------

    def set_foot_position(
        self, leg: str, x: float, y: float, z: float, target_rot=None
    ) -> IKResult:
        """IK solver — runs in caller's thread; use run_in_executor from async code."""
        result = self._run_ik(leg, x, y, z, target_rot)
        if result.success:
            names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
            self.sync_write_positions(dict(zip(names, result.angles_deg)), raw=True)
        else:
            logger.warning(
                "IK failed for leg=%s target=[%.4f, %.4f, %.4f] error=%.6fm msg=%r",
                leg, x, y, z, result.position_error_m, result.message,
            )
        return result

    def compute_ik(
        self, leg: str, x: float, y: float, z: float, target_rot=None
    ) -> IKResult:
        """Run IK without executing motion. Returns IKResult in URDF space."""
        return self._run_ik(leg, x, y, z, target_rot)

    def _run_ik(
        self, leg: str, x: float, y: float, z: float, target_rot=None
    ) -> IKResult:
        """Run IK with multi-start: first from current angles, then from standing."""
        target = np.array([x, y, z])
        current = self._get_current_urdf_angles(leg)
        result = self._solver.ik(leg, target, target_rot, initial_angles_deg=current)
        if not result.success:
            # Retry from calibrated standing angles — avoids local minima when the
            # robot is in an unusual posture that misguides the warm-start.
            standing = [self._default_offsets.get(n, 0.0) for n in self.leg_joint_names(leg)]
            r2 = self._solver.ik(leg, target, target_rot, initial_angles_deg=standing)
            if r2.position_error_m < result.position_error_m:
                logger.debug(
                    "IK leg=%s: retry from standing improved error %.4f→%.4f m",
                    leg, result.position_error_m, r2.position_error_m,
                )
                result = r2
        return result

    def get_foot_position(self, leg: str) -> dict:
        return self._solver.fk(leg, self._get_current_urdf_angles(leg))

    def compute_fk(self, leg: str, angles_deg: list[float]) -> dict:
        return self._solver.fk(leg, angles_deg)

    def _get_current_urdf_angles(self, leg: str) -> list[float]:
        """Return current joint angles in URDF space from bus manager cache."""
        names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
        cached = self._bus_manager.get_cached_positions()
        return [
            cached.get(name, self._cached_positions.get(name, self._default_offsets.get(name, 0.0)))
            for name in names
        ]

    def get_current_joint_angles_deg(self, leg: str) -> list[float]:
        """Kept for backwards compatibility — returns URDF angles."""
        return self._get_current_urdf_angles(leg)

    # ------------------------------------------------------------------
    # Coordinate space helpers (public — for web layer)
    # ------------------------------------------------------------------

    def urdf_to_logical(self, leg: str, urdf_angles: list[float]) -> list[float]:
        """Convert URDF-space angles to logical space for the given leg."""
        names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
        return [
            a - self._default_offsets.get(n, 0.0)
            for a, n in zip(urdf_angles, names)
        ]

    def logical_to_urdf(self, leg: str, logical_angles: list[float]) -> list[float]:
        """Convert logical-space angles to URDF space for the given leg."""
        names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
        return [
            a + self._default_offsets.get(n, 0.0)
            for a, n in zip(logical_angles, names)
        ]

    def leg_joint_names(self, leg: str) -> list[str]:
        """Return the ordered list of joint names for the given leg."""
        return KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS

    def set_servo_default_position(self, servo_id: int, deg: float) -> None:
        servo = self._servos_by_id[servo_id]
        servo.default_position_deg = deg
        self._default_offsets[servo.joint_name] = deg

    def get_servo_configs(self) -> list[dict]:
        return [
            {
                "servo_id": servo.servo_id,
                "joint_name": servo.joint_name,
                "direction_sign": servo.direction_sign,
                "zero_offset_steps": servo.zero_offset_steps,
                "default_position_deg": round(servo.default_position_deg, 4),
                "pid": {
                    "p": servo._cfg.pid.p,
                    "d": servo._cfg.pid.d,
                    "i": servo._cfg.pid.i,
                },
            }
            for servo in self._servos.values()
        ]

    def export_config(self) -> dict:
        return {
            "urdf_path": self._settings.robot.urdf_path,
            "home_pose": self._settings.robot.home_pose,
            "servos": self.get_servo_configs(),
            "poses": self._settings.robot.poses,
        }

    # ------------------------------------------------------------------
    # Config helpers (public — for web layer)
    # ------------------------------------------------------------------

    def set_simulation_mode(self, enabled: bool) -> None:
        self._sim_mode = enabled
        logger.info("Simulation mode %s", "ON — servo writes suppressed" if enabled else "OFF")

    @property
    def simulation_mode(self) -> bool:
        return self._sim_mode

    def list_pose_names(self) -> list[str]:
        """Return names of all configured poses."""
        return list(self._settings.robot.poses.keys())

    def get_pose(self, name: str) -> dict[str, float]:
        """Return pose joint angles in logical space (0 = default standing)."""
        if name not in self._settings.robot.poses:
            raise KeyError(f"Unknown pose: {name!r}")
        all_joints = KinematicChain.LEFT_JOINTS + KinematicChain.RIGHT_JOINTS
        pose = self._settings.robot.poses[name]
        return {j: float(pose.get(j, 0.0)) for j in all_joints}

    def home_pose_name(self) -> str:
        """Return the name of the home pose."""
        return self._settings.robot.home_pose

    def calibrate_imu(self) -> None:
        """Start dynamic IMU calibration."""
        self._imu.calibrate()

    # ------------------------------------------------------------------
    # Status reads (non-blocking — reads from bus manager cache)
    # ------------------------------------------------------------------

    def get_all_statuses(self) -> list[ServoStatus]:
        states = self._bus_manager.get_servo_states()
        if states:
            return states

        # Bus manager has no data yet (startup) — return cached mock statuses.
        return [
            ServoStatus(
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
            )
            for servo in self._servos.values()
        ]

    def get_imu(self) -> IMUReading:
        return self._bus_manager.get_imu_state()

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

            # All reads are non-blocking — bus manager owns the hardware.
            statuses = self.get_all_statuses()
            imu = self.get_imu()
            left_foot  = self.get_foot_position("left")
            right_foot = self.get_foot_position("right")

            frame = {
                "type": "telemetry",
                "timestamp": time.time(),
                "bus_hz": round(self._bus_manager.cycle_hz, 1),
                "servos": [
                    {
                        "id": s.servo_id,
                        "joint": s.joint_name,
                        "position_deg": s.position_deg,
                        "logical_deg": round(
                            s.position_deg - self._default_offsets.get(s.joint_name, 0.0), 2
                        ),
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
                    "left_foot": {
                        "x": round(left_foot["position"][0], 4),
                        "y": round(left_foot["position"][1], 4),
                        "z": round(left_foot["position"][2], 4),
                    },
                    "right_foot": {
                        "x": round(right_foot["position"][0], 4),
                        "y": round(right_foot["position"][1], 4),
                        "z": round(right_foot["position"][2], 4),
                    },
                },
            }

            asyncio.run_coroutine_threadsafe(queue.put(frame), loop)

            elapsed = time.monotonic() - t_start
            time.sleep(max(0.0, interval - elapsed))
