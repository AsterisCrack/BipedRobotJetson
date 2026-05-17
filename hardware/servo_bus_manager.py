"""
ServoBusManager — dedicated thread that owns the serial bus at 50 Hz.

All servo reads (SYNC_READ) and writes (SYNC_WRITE) happen inside this
thread.  External code interacts only through thread-safe, non-blocking
methods:

    manager.set_target_positions({"l_hip_yaw": 47.0, ...}, speed=300)
    states  = manager.get_servo_states()   # list[ServoStatus]
    imu     = manager.get_imu_state()      # IMUReading
    cache   = manager.get_cached_positions()  # dict[joint_name, urdf_deg]
    rl      = manager.get_rl_state()       # dict with positions/velocities/IMU/gravity

Commands use "latest-wins" semantics: if the control loop posts a new
target before the bus thread has consumed the previous one, the older
command is silently replaced.
"""
from __future__ import annotations

import collections
import logging
import threading
import time

from hardware.imu.bno055 import BNO055, IMUReading
from hardware.serial_bus import SerialBus, SerialBusError
from hardware.st3215.protocol import (
    bytes_to_steps,
    encode_sync_read,
    encode_sync_write,
    pack_u16,
    steps_to_bytes,
    unpack_u16,
)
from hardware.st3215.registers import Reg
from hardware.st3215.servo import ST3215, ServoStatus

logger = logging.getLogger(__name__)

_TARGET_HZ = 50
_PERIOD = 1.0 / _TARGET_HZ   # 20 ms
_PERIOD_NS = 20_000_000       # 20 ms in nanoseconds (integer for fast arithmetic)


class _IMUReaderThread:
    """Reads BNO055 continuously on a daemon thread so the servo cycle can call
    get() for a ~2 µs cache read instead of blocking ~4 ms on I2C."""

    def __init__(self, imu: BNO055, buffer_size: int = 200) -> None:
        self._imu = imu
        self._latest: IMUReading = IMUReading()  # default until first read completes
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._durations: collections.deque = collections.deque(maxlen=buffer_size)
        self._read_count = 0
        self._hz_t0 = 0.0
        self._read_hz = 0.0

    def start(self) -> None:
        self._stop.clear()
        self._hz_t0 = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True, name="imu_reader")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def get(self) -> IMUReading:
        return self._latest

    def get_read_durations_us(self) -> list[float]:
        """Snapshot copy of recent per-read I2C durations in µs."""
        return list(self._durations)

    @property
    def read_hz(self) -> float:
        return self._read_hz

    @property
    def is_ready(self) -> bool:
        """True after at least one real I2C read has completed."""
        return len(self._durations) > 0

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic_ns()
            reading = self._imu.read()                      # blocks ~4 ms on I2C
            dt_us = (time.monotonic_ns() - t0) / 1000.0
            self._latest = reading                           # GIL-atomic reference write
            self._durations.append(dt_us)
            self._read_count += 1
            now = time.monotonic()
            if now - self._hz_t0 >= 1.0:
                self._read_hz = self._read_count / (now - self._hz_t0)
                self._read_count = 0
                self._hz_t0 = now


class ServoBusManager:
    """Owns the servo bus in a dedicated 50 Hz thread."""

    def __init__(
        self,
        servos: list[ST3215],
        bus: SerialBus,
        imu: BNO055,
        rt_scheduling: bool = False,
        fast_mode: bool = False,
        profiling: bool = False,
    ) -> None:
        self._servos = servos
        self._servo_ids = [s.servo_id for s in servos]
        self._servo_by_id: dict[int, ST3215] = {s.servo_id: s for s in servos}
        self._servo_by_name: dict[str, ST3215] = {s.joint_name: s for s in servos}
        self._bus = bus
        self._imu = imu
        self._rt_scheduling = rt_scheduling
        self._fast_mode = fast_mode
        self._profiling = profiling

        # fast_mode reads only position (2 bytes); normal reads full 8-byte status.
        self._sync_read_data_len: int = 2 if fast_mode else Reg.STATUS_LEN

        # Precompute the SYNC_READ packet (constant for this robot).
        self._sync_read_packet = encode_sync_read(
            Reg.STATUS_START, self._sync_read_data_len, self._servo_ids
        )

        # --- shared state (bus thread writes, any thread reads) ---
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
        self._imu_reader: _IMUReaderThread | None = None
        self._imu_stats_snapshot: dict | None = None
        self._cycle_hz: float = 0.0
        self._rt_scheduling_active: bool = False

        # Profiling buffers — only appended to when profiling=True
        self._prof_t_uart_us:    list[float] = []
        self._prof_t_write_w_us: list[float] = []
        self._prof_t_state_us:   list[float] = []
        self._prof_t_spare_us:   list[float] = []
        self._prof_t_cycle_us:   list[float] = []
        self._prof_miss_count:   int = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._stop.clear()
        if self._imu._smbus is not None:
            self._imu_reader = _IMUReaderThread(self._imu)
            self._imu_reader.start()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="bus_manager"
        )
        self._thread.start()
        logger.info("ServoBusManager started at %d Hz target", _TARGET_HZ)

    def stop(self) -> None:
        self._stop.set()
        # Stop IMU reader first so it stops writing _latest while bus thread winds down.
        # Save stats snapshot before clearing so get_profile_stats() still works after stop().
        if self._imu_reader is not None:
            self._imu_reader.stop()
            self._imu_stats_snapshot = {
                "durations": self._imu_reader.get_read_durations_us(),
                "hz": self._imu_reader.read_hz,
            }
            self._imu_reader = None
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

    def get_rl_state(self) -> dict:
        """RL observation vector — all data needed for one policy step.

        Acquires state lock once; projected_gravity math runs outside lock.

        Keys:
            positions         list[float]              servo deg in config order
            velocities        list[int]                servo speed counts (0 in fast_mode)
            linear_accel      tuple[float, float, float]  body-frame m/s² (x,y,z)
            angular_vel       tuple[float, float, float]  body-frame rad/s (x,y,z)
            projected_gravity tuple[float, float, float]  world [0,0,-1] in body frame
        """
        with self._state_lock:
            states_by_name = {s.joint_name: s for s in self._servo_states}
            imu = self._imu_state

        positions:  list[float] = []
        velocities: list[int]   = []
        for servo in self._servos:          # preserves config ordering
            status = states_by_name.get(servo.joint_name)
            positions.append(status.position_deg if status else 0.0)
            velocities.append(status.speed       if status else 0)

        # Rotate world gravity [0,0,-1] into body frame: g_body = R_wb^T @ [0,0,-1]
        # Identity quaternion (w=1,x=y=z=0) → (0, 0, -1): upright robot, gravity straight down ✓
        w, x, y, z = imu.quaternion
        gx =  2.0 * (w * y - x * z)
        gy = -2.0 * (y * z + w * x)
        gz =  2.0 * (x * x + y * y) - 1.0

        return {
            "positions":         positions,
            "velocities":        velocities,
            "linear_accel":      imu.accel,
            "angular_vel":       imu.gyro,
            "projected_gravity": (gx, gy, gz),
        }

    @property
    def cycle_hz(self) -> float:
        return self._cycle_hz

    @property
    def rt_scheduling_active(self) -> bool:
        """True if SCHED_FIFO was successfully applied to the bus thread.
        Valid after the first servo cycle completes (i.e. after warmup)."""
        return self._rt_scheduling_active

    @property
    def profile_cycle_count(self) -> int:
        """Number of profiling samples collected (0 when profiling=False)."""
        return len(self._prof_t_cycle_us) if self._profiling else 0

    def reset_profile_stats(self) -> None:
        """Clear profiling buffers. Call after warmup to start a clean collection."""
        if not self._profiling:
            return
        self._prof_t_uart_us.clear()
        self._prof_t_write_w_us.clear()
        self._prof_t_state_us.clear()
        self._prof_t_spare_us.clear()
        self._prof_t_cycle_us.clear()
        self._prof_miss_count = 0

    def get_profile_stats(self) -> dict:
        """Return collected timing data. Only populated when profiling=True.

        Keys:
            t_uart_us      list[float]  SYNC_READ full transaction (TX+RX), µs
            t_write_w_us   list[float]  SYNC_WRITE TX (write cycles only), µs
            t_state_us     list[float]  imu_reader.get() + _update_state(), µs
            t_spare_us     list[float]  spare budget remaining before sleep, µs
            t_cycle_us     list[float]  full cycle wall time, µs
            miss_count     int          total missing servo responses
            imu_read_us    list[float]  IMU thread actual I2C read durations, µs
            imu_read_hz    float        measured IMU read rate
        """
        imu_dur: list[float] = []
        imu_hz: float = 0.0
        if self._imu_reader is not None:
            imu_dur = self._imu_reader.get_read_durations_us()
            imu_hz  = self._imu_reader.read_hz
        elif self._imu_stats_snapshot is not None:
            imu_dur = self._imu_stats_snapshot["durations"]
            imu_hz  = self._imu_stats_snapshot["hz"]
        return {
            "t_uart_us":    list(self._prof_t_uart_us),
            "t_write_w_us": list(self._prof_t_write_w_us),
            "t_state_us":   list(self._prof_t_state_us),
            "t_spare_us":   list(self._prof_t_spare_us),
            "t_cycle_us":   list(self._prof_t_cycle_us),
            "miss_count":   self._prof_miss_count,
            "imu_read_us":  imu_dur,
            "imu_read_hz":  imu_hz,
        }

    # ------------------------------------------------------------------
    # Bus thread
    # ------------------------------------------------------------------

    def _thread_init(self) -> None:
        """Apply SCHED_FIFO to this thread. Requires root or CAP_SYS_NICE."""
        if not self._rt_scheduling:
            return
        try:
            import os
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(10))
            logger.info("Bus thread: SCHED_FIFO priority 10 applied")
            self._rt_scheduling_active = True
        except PermissionError:
            try:
                with open("/proc/self/status") as _f:
                    cap_lines = [l.rstrip() for l in _f if l.startswith("Cap")]
            except OSError:
                cap_lines = []
            logger.debug(
                "sched_setscheduler EPERM — CAP_SYS_NICE not effective. "
                "Check nosuid: findmnt -T $(readlink -f venv/bin/python3). "
                "Or run: sudo python main.py. "
                "Process caps: %s",
                ", ".join(cap_lines) or "unknown",
            )
        except Exception as exc:
            logger.debug("RT scheduling unavailable: %s", exc)

    def _run(self) -> None:
        self._thread_init()
        last_hz_time = time.monotonic()
        cycles = 0
        p = self._profiling

        while not self._stop.is_set():
            t_cycle_start = time.monotonic_ns()

            if self._bus.is_open:
                # --- SYNC_READ (write packet + bulk read, timed together) ---
                t0 = time.monotonic_ns()
                servo_states = self._sync_read_all()
                if p:
                    self._prof_t_uart_us.append((time.monotonic_ns() - t0) / 1000.0)

                # --- optional SYNC_WRITE ---
                cmd, speed = self._consume_pending()
                if cmd:
                    t0 = time.monotonic_ns()
                    self._sync_write(cmd, speed)
                    if p:
                        self._prof_t_write_w_us.append((time.monotonic_ns() - t0) / 1000.0)
            else:
                servo_states = {}

            # --- IMU get (cache read, ~2 µs) + state update ---
            t0 = time.monotonic_ns()
            if self._imu_reader is not None:
                imu = self._imu_reader.get()
            else:
                imu = self._imu.read()   # fallback when IMU not initialized
            self._update_state(servo_states, imu)
            if p:
                self._prof_t_state_us.append((time.monotonic_ns() - t0) / 1000.0)

            # --- Spare time (available for NN inference between cycles) ---
            t_work_done = time.monotonic_ns()
            spare_us = max(0.0, (_PERIOD_NS - (t_work_done - t_cycle_start)) / 1000.0)
            if p:
                self._prof_t_spare_us.append(spare_us)

            # --- Pace to target Hz ---
            elapsed = (t_work_done - t_cycle_start) / 1e9
            sleep_for = _PERIOD - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

            if p:
                self._prof_t_cycle_us.append((time.monotonic_ns() - t_cycle_start) / 1000.0)

            # --- Track actual Hz (updated every second) ---
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
                self._sync_read_packet, self._servo_ids, self._sync_read_data_len
            )
        except SerialBusError as exc:
            logger.debug("SYNC_READ failed: %s", exc)
            if self._profiling:
                self._prof_miss_count += len(self._servo_ids)
            return results

        for sid, data in raw.items():
            servo = self._servo_by_id.get(sid)
            if servo is None or len(data) < self._sync_read_data_len:
                if servo is not None:
                    logger.debug(
                        "SYNC_READ: short response from servo %d (%d bytes)", sid, len(data)
                    )
                continue
            pos_steps = bytes_to_steps(data, 0)
            if self._fast_mode:
                results[sid] = ServoStatus(
                    servo_id=sid,
                    joint_name=servo.joint_name,
                    position_deg=round(servo.steps_to_deg(pos_steps), 2),
                    raw_steps=pos_steps,
                    raw_deg=round(pos_steps / (4096.0 / 360.0), 2),
                    speed=0,
                    load=0,
                    voltage_v=0.0,
                    temperature_c=0,
                    torque_enabled=servo.torque_enabled,
                )
            else:
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

        if self._profiling:
            missed = len(self._servo_ids) - len(results)
            if missed > 0:
                self._prof_miss_count += missed

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
