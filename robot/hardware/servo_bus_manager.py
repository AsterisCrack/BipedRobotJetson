"""
ServoBusManager — dedicated thread that owns the serial bus at 50 Hz.

All servo reads (SYNC_READ) and writes (SYNC_WRITE) happen inside this
thread.  External code interacts only through thread-safe, non-blocking
methods:

    manager.set_target_positions({"l_hip_yaw": 47.0, ...}, speed=300)
    states  = manager.get_servo_states()   # list[ServoStatus]
    imu     = manager.get_imu_state()      # IMUReading
    cache   = manager.get_cached_positions()  # dict[joint_name, urdf_deg]

Commands use "latest-wins" semantics: if the control loop posts a new
target before the bus thread has consumed the previous one, the older
command is silently replaced.
"""
from __future__ import annotations

import logging
import threading
import time

from robot.hardware.imu.bno085 import BNO085, IMUReading
from robot.hardware.serial_bus import SerialBus, SerialBusError
from robot.hardware.st3215.protocol import (
    bytes_to_steps,
    encode_sync_read,
    encode_sync_write,
    pack_u16,
    steps_to_bytes,
    unpack_u16,
)
from robot.hardware.st3215.registers import Reg
from robot.hardware.st3215.servo import ST3215, ServoStatus

logger = logging.getLogger(__name__)

_TARGET_HZ = 50
_PERIOD = 1.0 / _TARGET_HZ  # 20 ms


class ServoBusManager:
    """Owns the servo bus in a dedicated 50 Hz thread."""

    def __init__(
        self,
        servos: list[ST3215],
        bus: SerialBus,
        imu: BNO085,
    ) -> None:
        self._servos = servos
        self._servo_ids = [s.servo_id for s in servos]
        self._servo_by_id: dict[int, ST3215] = {s.servo_id: s for s in servos}
        self._servo_by_name: dict[str, ST3215] = {s.joint_name: s for s in servos}
        self._bus = bus
        self._imu = imu

        # Precompute the SYNC_READ packet (constant for this robot).
        self._sync_read_packet = encode_sync_read(
            Reg.STATUS_START, Reg.STATUS_LEN, self._servo_ids
        )

        # --- shared sensor state (updated by bus thread, read by anyone) ---
        self._state_lock = threading.Lock()
        self._servo_states: list[ServoStatus] = []
        self._imu_state: IMUReading = IMUReading()
        self._cached_positions: dict[str, float] = {}

        # --- command buffer (written by anyone, consumed by bus thread) ---
        self._cmd_lock = threading.Lock()
        self._pending_positions: dict[str, float] | None = None
        self._pending_speed: int = 300

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._cycle_hz: float = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="bus_manager"
        )
        self._thread.start()
        logger.info("ServoBusManager started at %d Hz target", _TARGET_HZ)

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("ServoBusManager stopped")

    # ------------------------------------------------------------------
    # Public API (thread-safe, non-blocking)
    # ------------------------------------------------------------------

    def set_target_positions(
        self, joint_angles: dict[str, float], speed: int = 300
    ) -> None:
        """Queue new target positions. Latest-wins per joint."""
        with self._cmd_lock:
            if self._pending_positions is None:
                self._pending_positions = {}
            self._pending_positions.update(joint_angles)
            self._pending_speed = speed

    def get_servo_states(self) -> list[ServoStatus]:
        with self._state_lock:
            return list(self._servo_states)

    def get_imu_state(self) -> IMUReading:
        with self._state_lock:
            return self._imu_state

    def get_cached_positions(self) -> dict[str, float]:
        """Returns joint_name → urdf_deg for the last successful read."""
        with self._state_lock:
            return dict(self._cached_positions)

    @property
    def cycle_hz(self) -> float:
        return self._cycle_hz

    # ------------------------------------------------------------------
    # Bus thread
    # ------------------------------------------------------------------

    def _run(self) -> None:
        last_hz_time = time.monotonic()
        cycles = 0

        while not self._stop.is_set():
            t0 = time.monotonic()

            if self._bus.is_open:
                servo_states = self._sync_read_all()
                cmd, speed = self._consume_pending()
                if cmd:
                    self._sync_write(cmd, speed)
            else:
                servo_states = {}

            imu = self._imu.read()
            self._update_state(servo_states, imu)

            # Maintain target rate
            elapsed = time.monotonic() - t0
            sleep_for = _PERIOD - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

            # Track actual Hz (updated every second)
            cycles += 1
            now = time.monotonic()
            if now - last_hz_time >= 1.0:
                self._cycle_hz = cycles / (now - last_hz_time)
                cycles = 0
                last_hz_time = now

    def _sync_read_all(self) -> dict[int, ServoStatus]:
        """Issue one SYNC_READ and parse responses for all servos."""
        results: dict[int, ServoStatus] = {}
        try:
            raw = self._bus.sync_read(
                self._sync_read_packet, self._servo_ids, Reg.STATUS_LEN
            )
        except SerialBusError as exc:
            logger.debug("SYNC_READ failed: %s", exc)
            return results

        for sid, data in raw.items():
            servo = self._servo_by_id.get(sid)
            if servo is None or len(data) < Reg.STATUS_LEN:
                continue
            pos_steps = bytes_to_steps(data, 0)
            speed_val = unpack_u16(data, 2)
            load      = unpack_u16(data, 4)
            voltage   = data[6]
            temp      = data[7]
            results[sid] = ServoStatus(
                servo_id=sid,
                joint_name=servo.joint_name,
                position_deg=round(servo.steps_to_deg(pos_steps), 2),
                raw_steps=pos_steps,
                raw_deg=round(pos_steps / (4096.0 / 360.0), 2),
                speed=speed_val,
                load=load,
                voltage_v=round(voltage * 0.1, 2),
                temperature_c=temp,
                torque_enabled=servo.torque_enabled,
            )
        return results

    def _sync_write(self, positions: dict[str, float], speed: int) -> None:
        """Send SYNC_WRITE with target positions (already in URDF space)."""
        servo_data: list[tuple[int, bytes]] = []
        for joint_name, deg in positions.items():
            servo = self._servo_by_name.get(joint_name)
            if servo is None:
                continue
            steps = servo.deg_to_steps(deg)
            servo_data.append((servo.servo_id, steps_to_bytes(steps) + pack_u16(speed)))

        if not servo_data:
            return
        packet = encode_sync_write(Reg.TARGET_POS_L, 4, servo_data)
        try:
            self._bus.send_no_reply(packet)
        except SerialBusError as exc:
            logger.debug("SYNC_WRITE failed: %s", exc)

    def _consume_pending(self) -> tuple[dict[str, float] | None, int]:
        with self._cmd_lock:
            cmd = self._pending_positions
            speed = self._pending_speed
            self._pending_positions = None
        return cmd, speed

    def _update_state(
        self, servo_states: dict[int, ServoStatus], imu: IMUReading
    ) -> None:
        with self._state_lock:
            if servo_states:
                self._servo_states = list(servo_states.values())
                for status in servo_states.values():
                    self._cached_positions[status.joint_name] = status.position_deg
            self._imu_state = imu
