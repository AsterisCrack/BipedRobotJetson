"""Policy runner -- ONNX inference driving the robot at the bus loop's rate.

Threading model
---------------
``step()`` is called from the ServoBusManager's 50 Hz thread, in the slot that
thread already measures as "spare time available for NN inference" (~16.9 ms mean
on an Orin Nano; inference here is well under 1 ms). Everything else -- arm,
disarm, load, commands -- arrives from FastAPI request handlers on other threads.

So: all mutable operator-facing state sits behind ``_lock``, and ``step()`` takes
that lock only briefly to snapshot. The heavy work (inference) happens unlocked.

``step()`` must never raise. An exception propagating into the bus thread would
kill the thread that owns the serial port, taking the whole robot down. Everything
is wrapped, and any failure faults rather than throws.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque

import numpy as np

from hardware.st3215.registers import Reg

from .autopilot import AutoPilot
from .command_limit import limit_command
from .deploy_config import DeployConfig, load_bundle
from .observation import (
    JointMapper,
    ObservationBuilder,
    specific_force_from_linear_accel,
)
from .safety import (DEFAULT_MAX_ACTION_RATE, MAX_ACTION_RATE_RANGE, TILT_FAULT_RAD_RANGE,
                     VELOCITY_SCALE_RANGE, FaultReason, SafetyLimits, SafetyMonitor,
                     select_action_rate, select_tilt_fault, sensor_age,
                     tilt_from_projected_gravity)
from .state_machine import PolicyState, StateMachine
from .trace import TraceRecorder

logger = logging.getLogger(__name__)

# Seconds to ramp from wherever the robot is to the standing pose when arming.
ARM_RAMP_S = 2.0
# Servo speed for the arming ramp. Deliberately slow -- this is the one motion that
# happens while a human may still have hands on the robot.
ARM_SPEED = 200
# Servo speed during policy execution. 0 = "as fast as the servo can", which is what
# we want: the policy's own EMA filter is the rate limiter, not the servo's profile.
RUN_SPEED = 0
# A streamed command (gamepad) not refreshed within this many seconds is zeroed: a dropped
# browser tab or WiFi link must not leave the robot walking on the last stick position.
COMMAND_STREAM_TIMEOUT_S = 0.5
# Fallback command ranges for old bundles without command_ranges.
_DEFAULT_COMMAND_RANGES = {"lin_vel_x": (-0.3, 0.5), "lin_vel_y": (-0.3, 0.3),
                           "ang_vel_z": (-0.3, 0.3)}

# Operator-settable limits and their allowed ranges (set_limits refuses anything outside).
_LIMIT_RANGES = {
    "max_action_rate": MAX_ACTION_RATE_RANGE,
    "velocity_scale": VELOCITY_SCALE_RANGE,
    "tilt_fault_rad": TILT_FAULT_RAD_RANGE,
}


class PolicyRunner:
    """Owns the loaded model, the runtime state, and the safety gates."""

    def __init__(self, robot, models_root: str):
        self._robot = robot
        self.models_root = models_root

        self._lock = threading.RLock()
        self.sm = StateMachine()
        self.limits = SafetyLimits()
        self.safety = SafetyMonitor(self.limits)

        self.cfg: DeployConfig | None = None
        self._session = None
        self._mapper: JointMapper | None = None
        self._obs: ObservationBuilder | None = None

        self._command = np.zeros(3, dtype=np.float64)
        self._command_deadline: float | None = None   # set only by streamed commands
        self._filtered_targets: np.ndarray | None = None  # radians, policy order
        self._arm_start: np.ndarray | None = None
        # True for the whole of arm() (audit, goal preload, torque enable, ARMING transition), so
        # the web motion guard covers the window before the state machine reaches ARMING.
        self._arming = False

        # Diagnostics
        self._infer_ms = deque(maxlen=50)
        self._cycle_ms = deque(maxlen=50)
        self._step_count = 0
        self._last_action = np.zeros(12, dtype=np.float64)
        self._last_targets_deg: dict[str, float] = {}
        # Post-mortem ring: the last ~2 s of observations before a fault.
        self._trace = deque(maxlen=100)
        # Full per-step recording of every RUNNING episode, saved to data/traces/.
        self.traces = TraceRecorder(
            os.path.join(os.path.dirname(os.path.abspath(models_root)), "data", "traces"))
        # Opt-in fall recovery (off at boot). disarm() and estop() turn it off.
        self.auto = AutoPilot(self)

    # ------------------------------------------------------------------
    # Model lifecycle
    # ------------------------------------------------------------------

    def load(self, bundle_dir: str) -> dict:
        """Load a bundle. Only permitted while not moving."""
        with self._lock:
            if self.sm.is_moving():
                raise RuntimeError(f"cannot load a model while {self.sm.state.value}")

            cfg = load_bundle(bundle_dir)

            import onnxruntime as ort
            opts = ort.SessionOptions()
            # The bus thread is latency-sensitive and already has the core busy;
            # extra inference threads cause jitter rather than speed on this size
            # of network.
            opts.intra_op_num_threads = 1
            opts.inter_op_num_threads = 1
            opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            session = ort.InferenceSession(
                cfg.onnx_path, sess_options=opts, providers=["CPUExecutionProvider"])

            robot_names = self._robot_joint_names()
            mapper = JointMapper(cfg.joint_names, robot_names,
                                 self._robot_default_offsets())

            obs = ObservationBuilder(cfg)
            obs.validate_layout()

            # Prove the graph runs and is correctly shaped before it can ever be
            # asked to do so mid-gait.
            probe = np.zeros((1, cfg.obs_dim), dtype=np.float32)
            out = session.run(["action"], {"obs": probe})[0]
            if out.shape != (1, cfg.action_dim):
                raise RuntimeError(
                    f"model returned {out.shape}, expected (1, {cfg.action_dim})")

            self.cfg, self._session, self._mapper, self._obs = cfg, session, mapper, obs
            self._apply_bundle_limits(cfg)
            self._filtered_targets = None
            self._last_action = np.zeros(cfg.action_dim)
            self._infer_ms.clear()
            self._cycle_ms.clear()
            self._step_count = 0
            logger.info("Loaded policy %s (obs=%d act=%d @ %.0f Hz)",
                        cfg.name, cfg.obs_dim, cfg.action_dim, cfg.control_hz)
            return self.status()

    def _apply_bundle_limits(self, cfg) -> None:
        """Set the runtime limits from the bundle, always relative to the DEFAULTS.

        Bundle v3+ carries the in-env limiter the policy trained with; it is used only if > 0.
        None (old bundle) or 0 (trained without a limiter) give the default, so a bundle never
        disables the clip, and loading a second bundle forgets the first one's value (the
        previous value is NOT the fallback). The tilt fault only ever tightens: min(bundle, 0.784).
        """
        self.limits.max_action_rate = select_action_rate(cfg.max_action_rate, DEFAULT_MAX_ACTION_RATE)
        self.limits.tilt_fault_rad = select_tilt_fault(cfg.tilt_fault_rad)

    def _robot_joint_names(self) -> list[str]:
        return [s.joint_name for s in self._robot._bus_manager._servos]

    def _robot_default_offsets(self) -> dict[str, float]:
        return {s.joint_name: float(getattr(s, "default_position_deg", 0.0))
                for s in self._robot._bus_manager._servos}

    # ------------------------------------------------------------------
    # Operator commands
    # ------------------------------------------------------------------

    def _audit_servo_registers(self) -> None:
        """Read back, from the SERVOS, the registers the policy's actuator model depends on.

        config/robot.yaml is what Robot.initialize() meant to write, not what the servos hold.
        These are RAM registers: a servo that powered up after the server, or browned out,
        is back at factory values, and the factory acceleration cap (0x55 = 50) makes the
        servo too sluggish to follow the policy, which shows up as a growing oscillation.
          * acceleration cap 0x55 / 0x29 != 0  -> cleared again here, then re-checked
          * P/D/I != the bundle's requires_pid -> refuse
          * TORQUE_LIMIT 0x30 < 1000            -> refuse (training assumed torque_scale 1.0)
        Runs on the request thread with the bus lock taken per transfer, like the startup
        writes. It must not hold the runner lock, or the bus thread would stall behind it.
        """
        req = (self.cfg.actuator or {}).get("requires_pid") if self.cfg and self.cfg.actuator else None
        problems = []
        for servo in self._robot._bus_manager._servos:
            name = servo.joint_name
            try:
                p, d, i = servo.get_pid()
                cap = servo.read_register(Reg.MAX_ACCELERATION, 1)[0]
                acc = servo.read_register(Reg.ACCELERATION, 1)[0]
                if cap or acc:
                    logger.warning("%s: acceleration cap back (0x55=%d, 0x29=%d); clearing", name, cap, acc)
                    servo.write_register(Reg.MAX_ACCELERATION, bytes([0]))   # 0x55 first, then 0x29
                    servo.write_register(Reg.ACCELERATION, bytes([0]))
                    cap = servo.read_register(Reg.MAX_ACCELERATION, 1)[0]
                    acc = servo.read_register(Reg.ACCELERATION, 1)[0]
                lim = int.from_bytes(servo.read_register(Reg.TORQUE_LIMIT_L, 2), "little")
            except Exception as exc:
                problems.append(f"{name}: register read failed ({exc})")
                continue
            if req and (p, d, i) != (req["p"], req["d"], req["i"]):
                problems.append(f"{name}: servo P/D/I = {p}/{d}/{i}, bundle needs {req['p']}/{req['d']}/{req['i']}")
            if cap or acc:
                problems.append(f"{name}: acceleration cap will not clear (0x55={cap}, 0x29={acc})")
            if lim < 1000:
                problems.append(f"{name}: TORQUE_LIMIT = {lim}/1000, but the policy trained on full torque")
        if problems:
            raise RuntimeError("servo register audit failed -- " + "; ".join(problems))

    @property
    def arming_in_progress(self) -> bool:
        """True while arm() is running. Part of Robot.policy_motion_locked()."""
        return self._arming

    def arm(self) -> dict:
        with self._lock:
            if self.cfg is None:
                raise RuntimeError("no model loaded")
            if not self.sm.can(PolicyState.ARMING):
                raise RuntimeError(f"cannot arm from {self.sm.state.value}")
            if getattr(self._robot, "sysid_active", False):
                raise RuntimeError("a system-identification job owns the bus; wait for it or abort it")
            if self._arming:
                raise RuntimeError("arm already in progress")
            # State stays IDLE until the end of the sequence, so this flag is what keeps the web
            # motion guard locked while goals are preloaded and torque comes on. Any failure below
            # leaves the state IDLE and the flag is cleared in the finally.
            self._arming = True
        try:
            return self._arm_sequence()
        finally:
            self._arming = False

    def _arm_sequence(self) -> dict:
        self._audit_servo_registers()

        with self._lock:
            if not self.sm.can(PolicyState.ARMING):
                raise RuntimeError(f"cannot arm from {self.sm.state.value}")
            if not self._robot._bus_manager.has_velocity:
                # Without the speed bytes obs[24:36] would be all zeros -- in-range,
                # plausible, and wrong. Refuse rather than walk on it.
                raise RuntimeError(
                    "BIPED_FAST_MODE=1 omits the speed register, so the policy's joint_vel "
                    "channel would be silently zero. Use BIPED_FAST_MODE=2 (pos+speed) or 0."
                )

            # A BAM-trained policy (bundle format v2) learned against a P-ONLY firmware loop
            # at a specific P register. Arming it on servos running D != 0 or a different P
            # would hand it an actuator it never saw, which is the exact sim2real gap the
            # servo model exists to close. config/robot.yaml is what Robot.initialize()
            # writes to the servos, so check that.
            req = ((self.cfg.actuator or {}).get("requires_pid") if self.cfg.actuator else None)
            if req:
                wrong = []
                for servo in self._robot._bus_manager._servos:
                    pid = servo._cfg.pid
                    if (pid.p, pid.d, pid.i) != (req["p"], req["d"], req["i"]):
                        wrong.append(f"{servo.joint_name} p/d/i={pid.p}/{pid.d}/{pid.i}")
                if wrong:
                    raise RuntimeError(
                        f"bundle was trained on servos with P/D/I = {req['p']}/{req['d']}/{req['i']}, "
                        f"but config/robot.yaml has: {', '.join(wrong)}. Fix the PID there and restart.")

            self.safety.reset()
            self._command[:] = 0.0

            # Capture the pose we are ramping FROM so the ramp is smooth regardless
            # of where the robot happens to be sitting.
            rl = self._robot._bus_manager.get_rl_state()
            if rl["servo_missing"]:
                raise RuntimeError(f"servos never replied: {rl['servo_missing']}; check bus/power")
            # The present positions below come from the bus thread's cache: refuse if it is stale.
            stamp = rl.get("servo_stamp")
            if stamp is None or time.monotonic() - stamp > self.limits.max_sensor_age_s:
                raise RuntimeError(
                    "servo positions are stale (no fresh bus read within "
                    f"{self.limits.max_sensor_age_s * 1000:.0f} ms); is the bus thread running?")
            self._arm_start = self._mapper.robot_deg_to_policy_rad(rl["positions"])

            # The goal registers may still hold a stale goal (a web Home at speed 300, or the last
            # policy target at RUN_SPEED 0 = full speed), and enabling torque makes the servo chase
            # it. So, torque still off, make every servo's goal its PRESENT position at ARM_SPEED
            # first. Any failure refuses the arm: torque must not go on over an unknown goal.
            try:
                self._robot.preload_goal_positions(
                    dict(zip(self._mapper.robot_names, rl["positions"])), ARM_SPEED)
            except Exception as exc:
                raise RuntimeError(
                    f"arm refused: could not set goal = present position before torque enable ({exc})"
                ) from exc

            self._robot.enable_all_torques()
            self.sm.to(PolicyState.ARMING, "operator")
            return self.status()

    def start(self) -> dict:
        """Hand control to the policy. Only from ARMED, never straight from IDLE."""
        with self._lock:
            if not self.sm.can(PolicyState.RUNNING):
                raise RuntimeError(f"cannot start from {self.sm.state.value}")

            # Reset runtime state the way training episodes start: history zero-filled, previous
            # action 0, filtered targets at the current pose. (Seeding the history with copies of
            # the standing frame was a sim2real mismatch; the rate limiter bounds the first steps.)
            rl = self._robot._bus_manager.get_rl_state()
            if rl["servo_missing"]:
                raise RuntimeError(f"servos never replied: {rl['servo_missing']}; check bus/power")
            self._obs.reset()

            pos_rad = self._mapper.robot_deg_to_policy_rad(rl["positions"])
            self._filtered_targets = pos_rad.copy()   # matches Isaac's reset behaviour
            self._command[:] = 0.0
            self.safety.reset()

            self.traces.begin(self._trace_meta())
            self.sm.to(PolicyState.RUNNING, "operator")
            return self.status()

    def _trace_meta(self) -> dict:
        """What is needed to INTERPRET a trace offline (units, joint order, action map, limits, offsets).
        Re-running the policy on it still needs the bundle's ONNX graph and obs normalisation."""
        cfg = self.cfg
        return {
            "model": cfg.name, "model_info": cfg.info,
            "format_version": cfg.format_version,
            "joint_names": list(cfg.joint_names),
            "history_size": cfg.history_size,
            "control_hz": cfg.control_hz,
            "step_dt": cfg.step_dt, "action_map": cfg.action_map,
            "gait_clock": cfg.gait_clock,
            "action_filter_alpha": cfg.action_filter_alpha,
            "action_filter_applications": cfg.action_filter_applications,
            "default_joint_pos": cfg.default_joint_pos.tolist(),          # rad, policy order
            "action_scale_rad": (None if cfg.action_scale_rad is None
                                 else cfg.action_scale_rad.tolist()),     # v6: the a >= 0 scale (action_scale_pos_rad)
            **({} if cfg.action_scale_neg_rad is None else {"action_scale_neg_rad": cfg.action_scale_neg_rad.tolist()}),
            "joint_limits_min": cfg.joint_limits_min.tolist(),            # rad, policy order
            "joint_limits_max": cfg.joint_limits_max.tolist(),
            "command_ranges": cfg.command_ranges,
            "command_limit_shape": cfg.command_limit_shape,
            "actuator": cfg.actuator,
            # URDF-space standing offsets from config/robot.yaml: logical = urdf - this.
            "default_position_deg": self._robot_default_offsets(),
            "limits": {"tilt_fault_rad": self.limits.tilt_fault_rad,
                       "max_action_rate": self.limits.max_action_rate,
                       "velocity_scale": self.limits.velocity_scale},
            "obs_layout": cfg.obs_layout,
        }

    def _end_trace(self, reason: str) -> None:
        s = self.safety.state
        self.traces.end(reason, {"fault": s.fault, "fault_detail": s.fault_detail,
                                 "rate_limited_steps": s.rate_limited_steps})

    def stop(self) -> dict:
        """Back to ARMED: stop stepping the policy but hold torque and pose."""
        with self._lock:
            if self.sm.state == PolicyState.RUNNING:
                self.sm.to(PolicyState.ARMED, "operator stop")
            self._end_trace("stop")
            return self.status()

    def disarm(self) -> dict:
        self.auto.disable()
        with self._lock:
            self._safe_torque_off()
            self.sm.to(PolicyState.IDLE, "operator disarm")
            self._end_trace("disarm")
            return self.status()

    def estop(self) -> dict:
        """Immediate torque cut from any state. Must never fail."""
        try:
            self.auto.disable("e-stop")
        except Exception:
            logger.exception("disabling auto-recover failed during e-stop")
        with self._lock:
            self._safe_torque_off()
            try:
                self._end_trace("estop")
            except Exception:
                logger.exception("ending the trace failed during e-stop")
            self.safety.trigger(FaultReason.ESTOP, "operator e-stop")
            self.sm.fault_now(FaultReason.ESTOP, "operator e-stop")
            logger.warning("E-STOP engaged")
            return self.status()

    def fault(self, reason: str, detail: str = "") -> None:
        """Fault from outside the bus thread (auto-recover): latch, torque off, close the trace."""
        with self._lock:
            self.safety.trigger(reason, detail)
            self.sm.fault_now(reason, detail)
            self._safe_torque_off()
            self._end_trace(f"fault_{reason}")
            logger.warning("FAULT %s: %s", reason, detail)

    def clear_fault(self) -> dict:
        with self._lock:
            self._safe_torque_off()
            self.sm.clear_fault()
            self.safety.reset()
            return self.status()

    def command_ranges(self) -> dict:
        """The trained command box, {key: [lo, hi]}: the bundle's, with defaults for missing keys."""
        r = (self.cfg.command_ranges if self.cfg else None) or {}
        return {k: [float(v) for v in r.get(k, d)] for k, d in _DEFAULT_COMMAND_RANGES.items()}

    def command_limit_shape(self) -> str:
        """'box' (per-axis clamp) or 'ellipse_xy' (vx/vy inside an ellipse); 'box' if the bundle has none."""
        return self.cfg.command_limit_shape if self.cfg else "box"

    def set_command(self, vx: float, vy: float, wz: float, stream: bool = False) -> dict:
        """Velocity command, limited server-side to what the policy was trained on.

        Never trust the client for this: the policy has only ever seen commands
        inside the trained limit, and a slider bug should not be able to ask for
        something it has no idea how to do. The limit is the bundle's ``command_limit``
        (a box, or an ellipse in (vx, vy) with wz on its own box); without one it is the
        per-axis box of ``command_ranges``. See policy/command_limit.py. A non-finite
        command is refused as a zero command (NaN would pass a plain clamp).

        Returns ``command`` (what the policy gets, after velocity_scale) and ``limited``
        (the limited command before velocity_scale, i.e. what the UI should show).

        ``stream=True`` (gamepad) arms a deadman: unless refreshed within
        COMMAND_STREAM_TIMEOUT_S the command is zeroed by step(). A non-stream command
        (slider, keyboard) clears it and holds until changed.
        """
        with self._lock:
            shape = self.command_limit_shape()
            try:
                limited = limit_command((vx, vy, wz), self.command_ranges(), shape)
            except ValueError as exc:
                logger.warning("set_command(%r, %r, %r) refused, command zeroed: %s", vx, vy, wz, exc)
                limited = np.zeros(3, dtype=np.float64)
            scale = self.limits.velocity_scale
            self._command[:] = limited * scale
            self._command_deadline = (time.monotonic() + COMMAND_STREAM_TIMEOUT_S
                                      if stream else None)
            return {"command": self._command.tolist(), "limited": limited.tolist(),
                    "limit_shape": shape}

    def set_limits(self, **kw) -> dict:
        """Update runtime limits. Raises ValueError (nothing applied) if any value is out of range."""
        with self._lock:
            clean = {}
            for k, v in kw.items():
                if v is None or not hasattr(self.limits, k):
                    continue
                v = float(v)
                rng = _LIMIT_RANGES.get(k)
                if rng is not None and not (rng[0] - 1e-9 <= v <= rng[1] + 1e-9):   # also rejects NaN
                    raise ValueError(f"{k}={v:g} outside the allowed range [{rng[0]:g}, {rng[1]:g}]")
                clean[k] = v
            for k, v in clean.items():
                setattr(self.limits, k, v)
            return self.safety_status()

    def _safe_torque_off(self) -> None:
        try:
            self._robot.disable_all_torques()
        except Exception:
            logger.exception("torque disable failed during safety stop")

    # ------------------------------------------------------------------
    # Control step -- called from the bus thread
    # ------------------------------------------------------------------

    def step(self, rl_state: dict) -> None:
        """One control tick. Never raises."""
        t0 = time.monotonic()
        try:
            with self._lock:
                state = self.sm.state
                if state == PolicyState.ARMING:
                    self._step_arming(rl_state)
                    return
                if state != PolicyState.RUNNING:
                    return
                cfg, sess = self.cfg, self._session
                if self._command_deadline is not None and t0 > self._command_deadline:
                    logger.warning("streamed command timed out; zeroing it")
                    self._command[:] = 0.0
                    self._command_deadline = None
                command = self._command.copy()

            self._step_running(rl_state, cfg, sess, command)

        except Exception as exc:
            logger.exception("policy step failed")
            self.sm.fault_now(FaultReason.INFERENCE_ERROR, str(exc)[:200])
            self._safe_torque_off()
            self._end_trace(f"fault_{FaultReason.INFERENCE_ERROR}")
        finally:
            self._cycle_ms.append((time.monotonic() - t0) * 1000.0)

    def _step_arming(self, rl_state: dict) -> None:
        """Ramp to the home pose (bundle default_joint_pos; zero for centered/absolute) over ARM_RAMP_S, then hold."""
        frac = min(1.0, self.sm.seconds_in_state / ARM_RAMP_S)
        # Smoothstep: zero velocity at both ends, so no jerk on entry or arrival.
        s = frac * frac * (3.0 - 2.0 * frac)
        home = self.cfg.default_joint_pos      # policy-space standing pose (crouched home for residual)
        target = home + (self._arm_start - home) * (1.0 - s)
        self._write_targets(target, speed=ARM_SPEED)

        if frac >= 1.0:
            tilt = tilt_from_projected_gravity(rl_state["projected_gravity"])
            if tilt > self.limits.tilt_fault_rad:
                self.sm.fault_now(FaultReason.TILT, f"tilted after arming ramp")
                self._safe_torque_off()
            else:
                self.sm.to(PolicyState.ARMED, "ramp complete")

    def _step_running(self, rl_state, cfg, sess, command) -> None:
        frame = self._assemble_frame(rl_state, advance_phase=True, command=command)

        tilt = tilt_from_projected_gravity(rl_state["projected_gravity"])
        age = sensor_age(time.monotonic(), rl_state["servo_stamp"], rl_state["imu_stamp"])
        fault = self.safety.check_observation(frame, rl_state["projected_gravity"], age)
        if not fault and rl_state["servo_missing"]:
            self.safety.trigger(FaultReason.STALE_SENSOR,
                                f"servos never replied: {rl_state['servo_missing']}")
            fault = self.safety.state.fault
        if fault:
            self._trace.append(frame.copy())
            self.sm.fault_now(fault, self.safety.state.fault_detail)
            self._safe_torque_off()
            logger.warning("FAULT %s: %s", fault, self.safety.state.fault_detail)
            self.traces.record(frame=frame, command=command, tilt_rad=tilt)
            self._end_trace(f"fault_{fault}")
            return

        obs = self._obs.push(frame)
        normalized = cfg.normalize_obs(obs)[None, :]

        t_inf = time.monotonic()
        action = sess.run(["action"], {"obs": normalized})[0][0].astype(np.float64)
        self._infer_ms.append((time.monotonic() - t_inf) * 1000.0)

        if not np.all(np.isfinite(action)):
            self.sm.fault_now(FaultReason.INFERENCE_ERROR, "non-finite action")
            self._safe_torque_off()
            self._end_trace("fault_non_finite_action")
            return

        action_raw = action.copy()
        action = self.safety.limit_action(action, self._obs.previous_action)

        # Affine to joint targets, then the EMA -- applied once per decimation
        # sub-step, exactly as Isaac does inside its physics loop. Applying it once
        # instead of four times leaves the robot measurably laggier than the one the
        # policy trained on (effective alpha 0.4 vs 0.8704).
        targets = cfg.action_to_joint_targets(action)
        a = cfg.action_filter_alpha
        for _ in range(cfg.action_filter_applications):
            self._filtered_targets = a * targets + (1.0 - a) * self._filtered_targets

        self._write_targets(self._filtered_targets, speed=RUN_SPEED)

        self._obs.set_previous_action(action)
        self._last_action = action
        self._step_count += 1
        self._trace.append(frame)
        self.traces.record(
            frame=frame, action_raw=action_raw, action=action, targets_rad=self._filtered_targets,
            rate_limited=float(not np.allclose(np.clip(action_raw, -1, 1), action)),
            command=command, tilt_rad=tilt,
            cycle_ms=self._cycle_ms[-1] if self._cycle_ms else np.nan,
            infer_ms=self._infer_ms[-1] if self._infer_ms else np.nan)

        self.safety.note_cycle_time(
            self._cycle_ms[-1] / 1000.0 if self._cycle_ms else 0.0, cfg.step_dt)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _assemble_frame(self, rl_state, advance_phase: bool,
                        command: np.ndarray | None = None) -> np.ndarray:
        """Build one proprio frame from a get_rl_state() snapshot."""
        cmd = command if command is not None else np.zeros(3)

        # The BNO055 reports gravity-REMOVED linear acceleration; the policy trained
        # on specific force (gravity included, ~+9.81 z upright). Reconstruct it.
        # hardware.imu_raw_accel: the measured specific force (ACC_DATA) is used directly instead.
        raw = rl_state.get("specific_force")
        if raw is not None:
            spec_force = np.asarray(raw, dtype=np.float64)
        else:
            spec_force = specific_force_from_linear_accel(
                rl_state["linear_accel"], rl_state["projected_gravity"])

        pos_rad = self._mapper.robot_deg_to_policy_rad(rl_state["positions"])

        vel_rad = self._mapper.robot_rate_to_policy_rad(rl_state["velocities_deg_s"])

        return self._obs.build_frame(
            specific_force_b=spec_force,
            ang_vel_b=rl_state["angular_vel"],
            projected_gravity_b=rl_state["projected_gravity"],
            commands=cmd,
            joint_pos_rad=pos_rad,
            joint_vel_rad=vel_rad,
            advance_phase=advance_phase,
        )

    def _write_targets(self, targets_rad: np.ndarray, speed: int) -> None:
        """Clamp against the POLICY's joint limits, then hand to the robot.

        We clamp here with the limits from deploy_config rather than relying solely
        on Robot.sync_write_positions: its `_joint_limits_deg` is populated from the
        URDF but the clamp treats it as logical space, so the two disagree whenever
        default_position_deg is non-zero. Clamping with limits we know are correct
        means safety does not depend on resolving that.
        """
        clamped = np.clip(targets_rad, self.cfg.joint_limits_min, self.cfg.joint_limits_max)
        by_name = self._mapper.policy_rad_to_logical_deg(clamped)
        self._last_targets_deg = by_name
        self._robot.sync_write_positions(by_name, speed=speed, raw=False)

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            reason, detail = self.sm.fault
            return {
                "state": self.sm.state.value,
                "seconds_in_state": round(self.sm.seconds_in_state, 2),
                "model": self.cfg.name if self.cfg else None,
                "model_info": self.cfg.info if self.cfg else {},
                "fault": reason,
                "fault_detail": detail,
                "command": self._command.tolist(),
                "command_ranges": self.command_ranges(),
                "command_limit_shape": self.command_limit_shape(),
                "steps": self._step_count,
                "inference_ms": round(float(np.mean(self._infer_ms)), 3) if self._infer_ms else None,
                "inference_ms_max": round(float(np.max(self._infer_ms)), 3) if self._infer_ms else None,
                "cycle_ms": round(float(np.mean(self._cycle_ms)), 3) if self._cycle_ms else None,
                "tilt_deg": round(np.degrees(self.safety.state.last_tilt_rad), 1),
                "budget_ms": round(self.cfg.step_dt * 1000, 1) if self.cfg else None,
                # Runtime limits as the sliders need them (rad / per-step rate / scale).
                "limits": {"tilt_fault_rad": self.limits.tilt_fault_rad,
                           "max_action_rate": self.limits.max_action_rate,
                           "velocity_scale": self.limits.velocity_scale},
                # Loaded bundle, for the control card's one-line summary.
                "format_version": self.cfg.format_version if self.cfg else None,
                "action_map": self.cfg.action_map if self.cfg else None,
                "gait_clock": self.cfg.gait_clock if self.cfg else None,
                "home_deg": ({n: round(float(np.degrees(v)), 1) for n, v in
                              zip(self.cfg.joint_names, self.cfg.default_joint_pos)}
                             if self.cfg else None),
            }

    def safety_status(self) -> dict:
        with self._lock:
            s = self.safety.state
            return {
                "limits": {
                    "tilt_fault_deg": round(np.degrees(self.limits.tilt_fault_rad), 1),
                    "max_action_rate": self.limits.max_action_rate,
                    "velocity_scale": self.limits.velocity_scale,
                    "max_consecutive_overruns": self.limits.max_consecutive_overruns,
                },
                "tilt_deg": round(np.degrees(s.last_tilt_rad), 1),
                "rate_limited_steps": s.rate_limited_steps,
                "consecutive_overruns": s.consecutive_overruns,
                "fault": s.fault,
                "fault_detail": s.fault_detail,
            }

    def telemetry(self) -> dict:
        """Compact block folded into the existing 20 Hz telemetry frame."""
        with self._lock:
            return {
                **self.status(),
                "action": [round(v, 4) for v in self._last_action.tolist()],
                "targets_deg": {k: round(v, 2) for k, v in self._last_targets_deg.items()},
                "gait_phase": round(self._obs.gait_phase, 3) if self._obs else 0.0,
                "auto": self.auto.status(),
            }
