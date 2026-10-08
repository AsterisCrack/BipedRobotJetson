"""
hardware/sysid.py -- high-rate single-servo recording for system identification.

Runs ONE servo through a fixed trajectory at ~250 Hz and logs exactly what Rhoban BAM's
identification pipeline consumes (bam.process -> bam.fit -> bam.mae). Standalone, like the
rest of hardware/: no project imports beyond hardware/ itself.

HOW IT KEEPS UP
---------------
Per-servo register calls (ST3215.read_register / write_register) go through
SerialBus.transfer(), which calls flush(), and on the Jetson L4T kernel flush() costs a
whole ~10 ms kernel tick. The hot loop therefore never calls them. It only uses:
  * a one-id SYNC_READ of the status block (bus.sync_read, which skips the flush);
  * SYNC_WRITE for the goal AND for the torque toggle. SYNC_WRITE is broadcast, so the
    servo never replies. A plain WRITE would leave a stray status reply in the RX buffer
    and corrupt the next read.
Anything that needs a reply (setting PID, clearing the acceleration cap) happens before
or after the loop, where 10 ms doesn't matter.

Pacing uses an absolute deadline (t_next += period), not "sleep the remainder", so
jitter doesn't accumulate. A missed reply is recorded as a gap, never retried, because
one timeout already costs a whole period.

BAM CONVENTIONS
---------------
  * position 0 is the arm HANGING DOWN. The recorder subtracts a captured hanging zero
    (capture_zero) and works in raw servo direction, so position, speed and goal share
    one sign. The gravity model is symmetric, so which way is "+" doesn't matter.
  * radians, rad/s, seconds. speed is the CURRENT_SPEED register (sign-magnitude, counts/s).
  * P-only firmware loop: kp is written to the servo's P register with D = I = 0 for the
    duration of the run, then the caller's PID is restored.

The four trajectories below are copied verbatim from bam/trajectory.py
(github.com/Rhoban/bam @ e9a619d, Apache-2.0, (c) 2025 Marc Duclusaud & Gregoire
Passault). They span +-pi/2 or more about the hanging zero, so they are BENCH ONLY:
never run one on a servo that is still in the robot. The in-situ screen uses its own
small triangle instead.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from hardware.serial_bus import SerialBus, SerialBusError
from hardware.st3215.protocol import (
    bytes_to_steps,
    encode_sync_read,
    encode_sync_write,
    unpack_sign_magnitude16,
)
from hardware.st3215.registers import Reg
from hardware.st3215.servo import GOAL_BLOCK_LEN, ST3215, goal_bytes

RAD_PER_STEP = 2.0 * math.pi / 4096.0
STATUS_LEN = Reg.STATUS_LEN          # pos(2) speed(2) load(2) voltage(1) temp(1)


# --- BAM trajectories (verbatim from bam/trajectory.py) ---------------------------------

def _cubic_interpolate(keyframes: list, t: float) -> float:
    if t < keyframes[0][0]:
        return keyframes[0][1]
    if t > keyframes[-1][0]:
        return keyframes[-1][1]
    for i in range(len(keyframes) - 1):
        if keyframes[i][0] <= t <= keyframes[i + 1][0]:
            t0, x0, x0p = keyframes[i]
            t1, x1, x1p = keyframes[i + 1]
            A = [[1, t0, t0**2, t0**3], [0, 1, 2 * t0, 3 * t0**2],
                 [1, t1, t1**2, t1**3], [0, 1, 2 * t1, 3 * t1**2]]
            w = np.linalg.solve(A, [x0, x0p, x1, x1p])
            return float(w[0] + w[1] * t + w[2] * t**2 + w[3] * t**3)
    return keyframes[-1][1]


def _lift_and_drop(t: float) -> tuple[float, bool]:
    return _cubic_interpolate([[0.0, 0.0, 0.0], [2.0, -np.pi / 2, 0.0]], t), t < 2.0


def _sin_time_square(t: float) -> tuple[float, bool]:
    return float(np.sin(t**2)), True


def _up_and_down(t: float) -> tuple[float, bool]:
    kf = [[0.0, 0.0, 0.0], [3.0, np.pi / 2, 0.0], [6.0, 0.8 * np.pi / 2, 0.0]]
    return _cubic_interpolate(kf, t), True


def _sin_sin(t: float) -> tuple[float, bool]:
    return float(np.sin(t) * np.pi / 2 + np.sin(5.0 * t) * 0.5 * np.sin(t * 2.0)), True


BENCH_TRAJECTORIES = {
    "sin_time_square": _sin_time_square,
    "lift_and_drop": _lift_and_drop,
    "up_and_down": _up_and_down,
    "sin_sin": _sin_sin,
}
TRAJECTORY_DURATION_S = 6.0


def triangle(amplitude_rad: float, period_s: float):
    """Slow symmetric triangle about 0, for the in-situ screen (small, safe on the robot)."""
    def f(t: float) -> tuple[float, bool]:
        ph = (t / period_s) % 1.0
        x = 4.0 * ph if ph < 0.25 else (2.0 - 4.0 * ph if ph < 0.75 else 4.0 * ph - 4.0)
        return amplitude_rad * x, True
    return f


# --- recording ---------------------------------------------------------------------------

@dataclass
class SysidJob:
    """One recording. Built by robot/sysid.py, executed on the bus thread via run_exclusive."""
    servo: ST3215
    trajectory: object                 # callable t -> (goal_rad, torque_enable)
    duration_s: float
    kp: int
    zero_steps: int                    # raw encoder count that maps to position 0
    rate_hz: float = 250.0
    restore_pid: tuple[int, int, int] | None = None   # (p, d, i) to put back afterwards
    final_torque: bool = False         # leave torque on at the end (screen) or off (bench)
    abort: threading.Event = field(default_factory=threading.Event)
    progress: list = field(default_factory=list)      # live sample sink, read by the UI

    def run(self, bus: SerialBus) -> dict:
        sid = self.servo.servo_id
        # ---- setup: slow calls that need a reply, done before timing starts ----
        p0, d0, i0 = self.restore_pid or (32, 0, 0)
        self.servo.set_pid(int(self.kp), 0, 0)
        self.servo.write_register(Reg.MAX_ACCELERATION, bytes([0]))
        self.servo.write_register(Reg.ACCELERATION, bytes([0]))

        read_pkt = encode_sync_read(Reg.STATUS_START, STATUS_LEN, [sid])
        torque_pkt = {on: encode_sync_write(Reg.TORQUE_ENABLE, 1, [(sid, bytes([1 if on else 0]))])
                      for on in (True, False)}

        def goal_pkt(goal_rad: float) -> bytes:
            steps = int(round(self.zero_steps + goal_rad / RAD_PER_STEP))
            return encode_sync_write(Reg.TARGET_POS_L, GOAL_BLOCK_LEN,
                                     [(sid, goal_bytes(max(0, min(4095, steps)), 0))])

        entries: list[dict] = []
        gaps = 0
        torque_on = None
        period = 1.0 / self.rate_hz
        aborted = False
        try:
            # First goal = current position, torque on, so the run starts without a jump.
            t0 = time.perf_counter()
            t_next = t0
            while True:
                t = time.perf_counter() - t0
                if t >= self.duration_s:
                    break
                if self.abort.is_set():
                    aborted = True
                    break
                goal, enable = self.trajectory(t)
                enable = bool(enable)
                if enable != torque_on:
                    bus.send_no_reply(torque_pkt[enable])
                    torque_on = enable
                bus.send_no_reply(goal_pkt(goal))
                data = bus.sync_read(read_pkt, [sid], STATUS_LEN).get(sid)
                if data is None:
                    gaps += 1
                else:
                    e = {
                        "timestamp": t,
                        "position": (bytes_to_steps(data, 0) - self.zero_steps) * RAD_PER_STEP,
                        "speed": unpack_sign_magnitude16(data, 2) * RAD_PER_STEP,
                        "load": float(unpack_sign_magnitude16(data, 4)),
                        "input_volts": data[6] * 0.1,
                        "temp": float(data[7]),
                        "goal_position": goal,
                        "torque_enable": enable,
                    }
                    entries.append(e)
                    self.progress.append(e)
                t_next += period
                sleep = t_next - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    t_next = time.perf_counter()     # fell behind: re-anchor, don't burst
        except SerialBusError:
            aborted = True
            raise
        finally:
            # ---- teardown: torque state, then restore the caller's PID ----
            try:
                if aborted or not self.final_torque:
                    bus.send_no_reply(torque_pkt[False])
                self.servo.set_pid(p0, d0, i0)
            except SerialBusError:
                pass

        span = entries[-1]["timestamp"] - entries[0]["timestamp"] if len(entries) > 1 else 0.0
        return {
            "entries": entries,
            "gaps": gaps,
            "aborted": aborted,
            "achieved_hz": (len(entries) - 1) / span if span > 0 else 0.0,
            "mean_volts": float(np.mean([e["input_volts"] for e in entries])) if entries else 0.0,
            "temp_start": entries[0]["temp"] if entries else None,
            "temp_end": entries[-1]["temp"] if entries else None,
        }


def capture_zero(bus: SerialBus, servo: ST3215, samples: int = 25) -> int:
    """Torque off, let the arm hang, average the encoder. Returns raw steps for position 0."""
    servo.disable_torque()
    time.sleep(1.5)                       # let a swinging arm settle
    sid = servo.servo_id
    pkt = encode_sync_read(Reg.STATUS_START, 2, [sid])
    vals = []
    for _ in range(samples):
        d = bus.sync_read(pkt, [sid], 2).get(sid)
        if d is not None:
            vals.append(bytes_to_steps(d, 0))
        time.sleep(0.01)
    if not vals:
        raise SerialBusError(f"servo {sid}: no position replies while capturing zero")
    return int(round(float(np.median(vals))))
