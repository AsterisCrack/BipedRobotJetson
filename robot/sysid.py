"""
robot/sysid.py -- system-identification sessions: bench recordings and the 12-joint screen.

Turns hardware/sysid.py's single-servo recorder into files that Rhoban BAM consumes
directly (bam.process -> bam.fit -> bam.mae, run in BipedRobot/actuator/). It owns:
  * the rig description: loads.json, written by BipedRobot/actuator/rig.py and copied
    to data/sysid/loads.json, which maps each weight hole to the equivalent point
    pendulum's (mass, length);
  * sessions under data/sysid/<session>/. A session is ONE mounting of ONE servo, and
    BAM fits q_offset once across a dataset, so the servo must never be remounted
    mid-session;
  * runs in a background thread, so the HTTP call returns at once and the UI polls
    live samples;
  * the in-situ screen: a small triangle on each robot joint, reporting the hysteresis
    width (backlash + friction deadband). That is what sets the training DR ranges.

Files: each recording is BAM's RAW log format, one JSON per 6 s run, named
YYYY-MM-DD_HHhMMmSS_<trajectory>_kp<kp>_h<hole>.json. Everything else (servo id, gaps,
achieved rate, temperatures, zero) goes in meta/<same name>.json. bam.process globs
*.json in the directory, so the sidecars are kept out of it.

Safety rules enforced here, not in the UI:
  * refuses while the RL policy is anywhere but IDLE, and in simulation mode;
  * bench trajectories swing +-pi/2 about the hanging zero, so they need an explicit
    bench=True and are refused for a servo id that is part of the robot, unless forced;
  * abort() trips the running job's flag; the job turns torque off before returning.
"""
from __future__ import annotations

import io
import json
import logging
import math
import os
import threading
import time
import zipfile
from datetime import datetime
from pathlib import Path

import numpy as np

from hardware.config import ServoConfig
from hardware.st3215.servo import ST3215
from hardware.sysid import (
    BENCH_TRAJECTORIES,
    TRAJECTORY_DURATION_S,
    SysidJob,
    capture_zero,
    triangle,
)

logger = logging.getLogger(__name__)

_REPO = Path(__file__).resolve().parent.parent
DATA_ROOT = _REPO / "data" / "sysid"
LOADS_PATH = DATA_ROOT / "loads.json"


class _ZeroJob:
    def __init__(self, servo: ST3215):
        self.servo = servo

    def run(self, bus):
        return capture_zero(bus, self.servo)


class SysidManager:
    def __init__(self, robot) -> None:
        self._robot = robot
        self._lock = threading.RLock()
        self.session: str | None = None
        self.servo_id: int | None = None
        self.zero_steps: int | None = None
        self._job: SysidJob | None = None
        self._thread: threading.Thread | None = None
        self.state = "idle"               # idle | running | screening
        self.last: dict | None = None     # summary of the last finished run
        self.last_error: str | None = None
        self.screen_result: dict | None = None
        DATA_ROOT.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------ guards
    def _check_can_drive(self) -> None:
        if self._robot.simulation_mode:
            raise RuntimeError("simulation mode is on: servo writes are suppressed")
        pol = getattr(self._robot, "_policy", None)
        if pol is not None and pol.sm.state.value != "idle":
            raise RuntimeError(f"RL policy is {pol.sm.state.value}; disarm it first")
        if self.state != "idle":
            raise RuntimeError(f"a sysid {self.state} job is already in progress")

    def _servo(self, servo_id: int) -> ST3215:
        robot_servo = self._robot._servos_by_id.get(servo_id)
        if robot_servo is not None:
            return robot_servo
        # A bench servo that is not part of robot.yaml (spare, or a temporarily re-IDed one).
        return ST3215(ServoConfig(servo_id=servo_id, joint_name=f"bench_{servo_id}",
                                  zero_offset_steps=2048, direction_sign=1), self._robot._bus)

    # ------------------------------------------------------------------ rig
    def rig(self) -> dict:
        if not LOADS_PATH.exists():
            return {"available": False, "path": str(LOADS_PATH),
                    "hint": "run `python -m actuator.rig` in BipedRobot and copy rig/loads.json here"}
        data = json.loads(LOADS_PATH.read_text())
        return {"available": True, "path": str(LOADS_PATH), **data}

    # ------------------------------------------------------------------ session
    def start_session(self, name: str, servo_id: int) -> dict:
        with self._lock:
            self._check_can_drive()
            safe = "".join(c for c in name if c.isalnum() or c in "-_") or datetime.now().strftime("%Y%m%d_%H%M")
            (DATA_ROOT / safe / "meta").mkdir(parents=True, exist_ok=True)
            self.session, self.servo_id, self.zero_steps = safe, int(servo_id), None
            return self.status()

    def capture_zero(self) -> dict:
        """Torque off, let the arm hang still, record the encoder: that is BAM's position 0."""
        with self._lock:
            self._check_can_drive()
            if self.servo_id is None:
                raise RuntimeError("start a session first")
            self.zero_steps = int(self._robot._bus_manager.run_exclusive(
                _ZeroJob(self._servo(self.servo_id)), timeout=10.0))
            (DATA_ROOT / self.session / "meta" / "zero.json").write_text(json.dumps(
                {"servo_id": self.servo_id, "zero_steps": self.zero_steps,
                 "captured": datetime.now().isoformat()}, indent=2))
            return self.status()

    # ------------------------------------------------------------------ bench run
    def start_run(self, trajectory: str, kp: int, hole: int, rate_hz: float = 250.0,
                  force_robot_servo: bool = False) -> dict:
        with self._lock:
            self._check_can_drive()
            if self.session is None or self.zero_steps is None:
                raise RuntimeError("start a session and capture the hanging zero first")
            if trajectory not in BENCH_TRAJECTORIES:
                raise ValueError(f"unknown trajectory {trajectory!r}; one of {sorted(BENCH_TRAJECTORIES)}")
            if self.servo_id in self._robot._servos_by_id and not force_robot_servo:
                raise RuntimeError(
                    f"servo {self.servo_id} is a robot joint, and bench trajectories swing +-90 deg. "
                    f"Mount it on the rig (or force it if it really is on the rig).")
            rig = self.rig()
            if not rig["available"]:
                raise RuntimeError(rig["hint"])
            if not 0 <= hole < len(rig["loads"]):
                raise ValueError(f"hole index {hole} out of range (0..{len(rig['loads']) - 1})")
            load = rig["loads"][hole]
            servo = self._servo(self.servo_id)
            pid = servo._cfg.pid
            job = SysidJob(servo=servo, trajectory=BENCH_TRAJECTORIES[trajectory],
                           duration_s=TRAJECTORY_DURATION_S, kp=int(kp), zero_steps=self.zero_steps,
                           rate_hz=rate_hz, restore_pid=(pid.p, pid.d, pid.i), final_torque=False)
            meta = {"trajectory": trajectory, "kp": int(kp), "hole": hole, "load": load,
                    "servo_id": self.servo_id, "zero_steps": self.zero_steps, "rate_hz": rate_hz}
            self._launch(job, "running", lambda res: self._save_bench(res, meta))
            return self.status()

    def _save_bench(self, res: dict, meta: dict) -> dict:
        if res["aborted"] or not res["entries"]:
            return {"saved": None, "reason": "aborted" if res["aborted"] else "no samples", **_summary(res)}
        load = meta["load"]
        stamp = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
        name = f"{stamp}_{meta['trajectory']}_kp{meta['kp']}_h{meta['hole']}.json"
        log = {
            "mass": load["mass"], "arm_mass": 0.0, "length": load["length"],
            "kp": meta["kp"], "vin": round(res["mean_volts"], 2),
            "motor": "sts3215", "trajectory": meta["trajectory"],
            "entries": res["entries"],
        }
        folder = DATA_ROOT / self.session
        (folder / name).write_text(json.dumps(log))
        (folder / "meta" / name).write_text(json.dumps({**meta, **_summary(res)}, indent=2))
        return {"saved": name, **_summary(res)}

    # ------------------------------------------------------------------ in-situ screen
    def start_screen(self, amplitude_deg: float = 10.0, period_s: float = 6.0, cycles: int = 2) -> dict:
        """Slow +-amplitude triangle on every robot joint in turn, at the configured PID.

        Hang the robot so every leg is free. Each joint moves alone; the others are left
        as they are. The result is the hysteresis width per joint: up-sweep error minus
        down-sweep error, mid-sweep only. That is backlash plus friction deadband.
        """
        with self._lock:
            self._check_can_drive()
            if not 1.0 <= amplitude_deg <= 20.0:
                raise ValueError("amplitude_deg must be within 1..20 for an in-situ screen")
            servos = list(self._robot._servos_by_id.values())
            self.screen_result = {"started": datetime.now().isoformat(), "amplitude_deg": amplitude_deg,
                                  "joints": {}}

            def work():
                for servo in servos:
                    if self._job is not None and self._job.abort.is_set():
                        break
                    pos_now = self._robot._bus_manager.get_cached_positions().get(servo.joint_name)
                    zero = (servo.deg_to_steps(pos_now) if pos_now is not None
                            else servo._cfg.zero_offset_steps)
                    pid = servo._cfg.pid
                    job = SysidJob(servo=servo, trajectory=triangle(math.radians(amplitude_deg), period_s),
                                   duration_s=period_s * cycles, kp=pid.p, zero_steps=zero,
                                   restore_pid=(pid.p, pid.d, pid.i), final_torque=False)
                    self._job = job
                    res = self._robot._bus_manager.run_exclusive(job, timeout=period_s * cycles + 10)
                    self.screen_result["joints"][servo.joint_name] = {
                        "servo_id": servo.servo_id, **_hysteresis(res["entries"]), **_summary(res)}
                    if res["aborted"]:
                        break
                stamp = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
                (DATA_ROOT / f"screen_{stamp}.json").write_text(json.dumps(self.screen_result, indent=2))
                return {"saved": f"screen_{stamp}.json"}

            self._launch(None, "screening", lambda _: work(), run_inline=True)
            return self.status()

    # ------------------------------------------------------------------ plumbing
    def _launch(self, job, state: str, finish, run_inline: bool = False) -> None:
        self._job, self.state, self.last_error = job, state, None

        def body():
            try:
                res = finish(None) if run_inline else finish(
                    self._robot._bus_manager.run_exclusive(job, timeout=TRAJECTORY_DURATION_S + 15))
                self.last = res
            except Exception as exc:   # surfaced via status(), never kills the server
                logger.exception("sysid job failed")
                self.last_error = str(exc)
            finally:
                self.state = "idle"

        self._thread = threading.Thread(target=body, daemon=True, name=f"sysid_{state}")
        self._thread.start()

    def abort(self) -> dict:
        job = self._job
        if job is not None:
            job.abort.set()
        return self.status()

    def live(self, since: int = 0) -> dict:
        job = self._job
        samples = job.progress[since:] if job is not None else []
        return {"state": self.state, "next": since + len(samples),
                "samples": [{"t": s["timestamp"], "goal": s["goal_position"], "pos": s["position"],
                             "torque": s["torque_enable"]} for s in samples]}

    def status(self) -> dict:
        files = []
        if self.session:
            folder = DATA_ROOT / self.session
            files = sorted(p.name for p in folder.glob("*.json")) if folder.exists() else []
        return {"state": self.state, "session": self.session, "servo_id": self.servo_id,
                "zero_steps": self.zero_steps, "files": files, "last": self.last,
                "last_error": self.last_error, "screen": self.screen_result,
                "trajectories": sorted(BENCH_TRAJECTORIES), "rig": self.rig()}

    def session_zip(self) -> bytes:
        if not self.session:
            raise RuntimeError("no session")
        buf = io.BytesIO()
        folder = DATA_ROOT / self.session
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            for p in folder.rglob("*"):
                if p.is_file():
                    zf.write(p, Path(self.session) / p.relative_to(folder))
        return buf.getvalue()


def _summary(res: dict) -> dict:
    return {"n": len(res["entries"]), "gaps": res["gaps"], "aborted": res["aborted"],
            "achieved_hz": round(res["achieved_hz"], 1), "mean_volts": round(res["mean_volts"], 2),
            "temp_start": res["temp_start"], "temp_end": res["temp_end"]}


def _hysteresis(entries: list[dict]) -> dict:
    """Up-sweep error minus down-sweep error, on the middle half of each sweep only.

    In a slow triangle the servo lags its goal by (deadband + backlash)/2 in each direction,
    with opposite sign. Turnarounds are excluded, since there the lag is dynamic rather
    than the deadband. Returned in degrees.
    """
    if len(entries) < 20:
        return {"hysteresis_deg": None}
    g = np.array([e["goal_position"] for e in entries])
    q = np.array([e["position"] for e in entries])
    dg = np.gradient(g)
    span = np.max(np.abs(g)) or 1.0
    mid = np.abs(g) < 0.5 * span                 # middle half of each sweep
    up, down = mid & (dg > 0), mid & (dg < 0)
    if up.sum() < 5 or down.sum() < 5:
        return {"hysteresis_deg": None}
    err = g - q
    width = float(np.degrees(np.mean(err[up]) - np.mean(err[down])))
    return {"hysteresis_deg": round(width, 3),
            "lag_up_deg": round(float(np.degrees(np.mean(err[up]))), 3),
            "lag_down_deg": round(float(np.degrees(np.mean(err[down]))), 3)}
