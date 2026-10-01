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
import threading
import time
from collections import deque

import numpy as np

from .deploy_config import DeployConfig, load_bundle
from .observation import (
    JointMapper,
    ObservationBuilder,
    specific_force_from_linear_accel,
)
from .safety import FaultReason, SafetyLimits, SafetyMonitor, tilt_from_projected_gravity
from .state_machine import PolicyState, StateMachine

logger = logging.getLogger(__name__)

# Seconds to ramp from wherever the robot is to the standing pose when arming.
ARM_RAMP_S = 2.0
# Servo speed for the arming ramp. Deliberately slow -- this is the one motion that
# happens while a human may still have hands on the robot.
ARM_SPEED = 200
# Servo speed during policy execution. 0 = "as fast as the servo can", which is what
# we want: the policy's own EMA filter is the rate limiter, not the servo's profile.
RUN_SPEED = 0


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
        self._filtered_targets: np.ndarray | None = None  # radians, policy order
        self._arm_start: np.ndarray | None = None

        # Diagnostics
        self._infer_ms = deque(maxlen=50)
        self._cycle_ms = deque(maxlen=50)
        self._step_count = 0
        self._last_action = np.zeros(12, dtype=np.float64)
        self._last_targets_deg: dict[str, float] = {}
        # Post-mortem ring: the last ~2 s of observations before a fault.
        self._trace = deque(maxlen=100)

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
            self._filtered_targets = None
            self._last_action = np.zeros(cfg.action_dim)
            self._infer_ms.clear()
            self._cycle_ms.clear()
            self._step_count = 0
            logger.info("Loaded policy %s (obs=%d act=%d @ %.0f Hz)",
                        cfg.name, cfg.obs_dim, cfg.action_dim, cfg.control_hz)
            return self.status()

    def _robot_joint_names(self) -> list[str]:
        return [s.joint_name for s in self._robot._bus_manager._servos]

    def _robot_default_offsets(self) -> dict[str, float]:
        return {s.joint_name: float(getattr(s, "default_position_deg", 0.0))
                for s in self._robot._bus_manager._servos}

    # ------------------------------------------------------------------
    # Operator commands
    # ------------------------------------------------------------------

    def arm(self) -> dict:
        with self._lock:
            if self.cfg is None:
                raise RuntimeError("no model loaded")
            if not self.sm.can(PolicyState.ARMING):
                raise RuntimeError(f"cannot arm from {self.sm.state.value}")
            if not self._robot._bus_manager.has_velocity:
                # Without the speed bytes obs[24:36] would be all zeros -- in-range,
                # plausible, and wrong. Refuse rather than walk on it.
                raise RuntimeError(
                    "BIPED_FAST_MODE=1 omits the speed register, so the policy's joint_vel "
                    "channel would be silently zero. Use BIPED_FAST_MODE=2 (pos+speed) or 0."
                )

            self.safety.reset()
            self._command[:] = 0.0

            # Capture the pose we are ramping FROM so the ramp is smooth regardless
            # of where the robot happens to be sitting.
            rl = self._robot._bus_manager.get_rl_state()
            self._arm_start = self._mapper.robot_deg_to_policy_rad(rl["positions"])

            self._robot.enable_all_torques()
            self.sm.to(PolicyState.ARMING, "operator")
            return self.status()

    def start(self) -> dict:
        """Hand control to the policy. Only from ARMED, never straight from IDLE."""
        with self._lock:
            if not self.sm.can(PolicyState.RUNNING):
                raise RuntimeError(f"cannot start from {self.sm.state.value}")

            # Seed every piece of runtime state from the CURRENT pose, so the first
            # policy frame is in-distribution. Isaac zero-fills history at reset and
            # then works around the resulting saturated actions with stochastic
            # sampling; on hardware that would be an unpredictable lurch.
            rl = self._robot._bus_manager.get_rl_state()
            frame = self._assemble_frame(rl, advance_phase=False)
            self._obs.reset(standing_frame=frame)

            pos_rad = self._mapper.robot_deg_to_policy_rad(rl["positions"])
            self._filtered_targets = pos_rad.copy()   # matches Isaac's reset behaviour
            self._command[:] = 0.0
            self.safety.reset()

            self.sm.to(PolicyState.RUNNING, "operator")
            return self.status()

    def stop(self) -> dict:
        """Back to ARMED: stop stepping the policy but hold torque and pose."""
        with self._lock:
            if self.sm.state == PolicyState.RUNNING:
                self.sm.to(PolicyState.ARMED, "operator stop")
            return self.status()

    def disarm(self) -> dict:
        with self._lock:
            self._safe_torque_off()
            self.sm.to(PolicyState.IDLE, "operator disarm")
            return self.status()

    def estop(self) -> dict:
        """Immediate torque cut from any state. Must never fail."""
        with self._lock:
            self._safe_torque_off()
            self.safety.trigger(FaultReason.ESTOP, "operator e-stop")
            self.sm.fault_now(FaultReason.ESTOP, "operator e-stop")
            logger.warning("E-STOP engaged")
            return self.status()

    def clear_fault(self) -> dict:
        with self._lock:
            self._safe_torque_off()
            self.sm.clear_fault()
            self.safety.reset()
            return self.status()

    def set_command(self, vx: float, vy: float, wz: float) -> dict:
        """Velocity command, clamped server-side to the trained ranges.

        Never trust the client for this: the policy has only ever seen commands
        inside these ranges, and a slider bug should not be able to ask for
        something it has no idea how to do.
        """
        with self._lock:
            r = (self.cfg.command_ranges if self.cfg else {})
            def _clamp(v, key, default):
                lo, hi = r.get(key, default)
                return float(np.clip(v, lo, hi))
            scale = self.limits.velocity_scale
            self._command[0] = _clamp(vx, "lin_vel_x", (-0.3, 0.5)) * scale
            self._command[1] = _clamp(vy, "lin_vel_y", (-0.3, 0.3)) * scale
            self._command[2] = _clamp(wz, "ang_vel_z", (-0.3, 0.3)) * scale
            return {"command": self._command.tolist()}

    def set_limits(self, **kw) -> dict:
        with self._lock:
            for k, v in kw.items():
                if v is not None and hasattr(self.limits, k):
                    setattr(self.limits, k, float(v))
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
                command = self._command.copy()

            self._step_running(rl_state, cfg, sess, command)

        except Exception as exc:
            logger.exception("policy step failed")
            self.sm.fault_now(FaultReason.INFERENCE_ERROR, str(exc)[:200])
            self._safe_torque_off()
        finally:
            self._cycle_ms.append((time.monotonic() - t0) * 1000.0)

    def _step_arming(self, rl_state: dict) -> None:
        """Ramp to standing (policy zero) over ARM_RAMP_S, then hold."""
        frac = min(1.0, self.sm.seconds_in_state / ARM_RAMP_S)
        # Smoothstep: zero velocity at both ends, so no jerk on entry or arrival.
        s = frac * frac * (3.0 - 2.0 * frac)
        target = self._arm_start * (1.0 - s)  # policy zero == standing
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

        fault = self.safety.check_observation(frame, rl_state["projected_gravity"])
        if fault:
            self._trace.append(frame.copy())
            self.sm.fault_now(fault, self.safety.state.fault_detail)
            self._safe_torque_off()
            logger.warning("FAULT %s: %s", fault, self.safety.state.fault_detail)
            return

        obs = self._obs.push(frame)
        normalized = cfg.normalize_obs(obs)[None, :]

        t_inf = time.monotonic()
        action = sess.run(["action"], {"obs": normalized})[0][0].astype(np.float64)
        self._infer_ms.append((time.monotonic() - t_inf) * 1000.0)

        if not np.all(np.isfinite(action)):
            self.sm.fault_now(FaultReason.INFERENCE_ERROR, "non-finite action")
            self._safe_torque_off()
            return

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
                "steps": self._step_count,
                "inference_ms": round(float(np.mean(self._infer_ms)), 3) if self._infer_ms else None,
                "inference_ms_max": round(float(np.max(self._infer_ms)), 3) if self._infer_ms else None,
                "cycle_ms": round(float(np.mean(self._cycle_ms)), 3) if self._cycle_ms else None,
                "tilt_deg": round(np.degrees(self.safety.state.last_tilt_rad), 1),
                "budget_ms": round(self.cfg.step_dt * 1000, 1) if self.cfg else None,
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
            }
