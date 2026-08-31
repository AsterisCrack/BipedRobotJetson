from __future__ import annotations

import logging
import math
import sys
import threading
import time
from enum import Enum, auto
from pathlib import Path
from typing import Callable

import numpy as np

from hardware.servo_bus_manager import ServoBusManager
from hardware.st3215.servo import STEPS_PER_DEG

logger = logging.getLogger(__name__)

# Right-leg-first order matching training environment joint indices 0–11
POLICY_JOINT_ORDER = [
    "r_hip_yaw", "r_hip_roll_joint", "r_hip_pitch_joint",
    "r_knee_joint", "r_ankle_roll_joint", "r_ankle_pitch_joint",
    "l_hip_yaw", "l_hip_roll_joint", "l_hip_pitch_joint",
    "l_knee_joint", "l_ankle_roll_joint", "l_ankle_pitch_joint",
]

_OBS_DIM = 48
_ACT_DIM = 12
_HZ = 50.0
_DT = 1.0 / _HZ
_DEG_TO_RAD = math.pi / 180.0

FALL_THRESHOLD_DEG = 40.0
UPRIGHT_THRESHOLD_DEG = 15.0
UPRIGHT_HOLD_S = 3.0
RESUME_DELAY_S = 2.0


class _State(Enum):
    IDLE = auto()
    RUNNING = auto()
    FALLEN = auto()
    RECOVERING = auto()


class PolicyExecutor:
    """
    Runs a trained RL policy at 50 Hz in a dedicated daemon thread.

    Reads state from ServoBusManager caches, runs torch inference,
    and enqueues joint targets via write_fn. Independent of the web layer.

    Fall recovery state machine:
      RUNNING → (tilt > FALL_THRESHOLD) → FALLEN  (motors depowered)
      FALLEN  → (upright for UPRIGHT_HOLD_S)      → RECOVERING (go to default)
      RECOVERING → (RESUME_DELAY_S elapsed)        → RUNNING
    """

    def __init__(
        self,
        bus_manager: ServoBusManager,
        default_offsets: dict[str, float],
        write_fn: Callable,
        disable_torques_fn: Callable,
        enable_torques_fn: Callable,
        weights_path: str,
        submodule_path: str,
        config_path: str | None = None,
        action_scale_deg: float = 40.0,
    ) -> None:
        self._bus_manager = bus_manager
        self._default_offsets = default_offsets
        self._write_fn = write_fn
        self._disable_torques_fn = disable_torques_fn
        self._enable_torques_fn = enable_torques_fn
        self._action_scale_deg = action_scale_deg

        self._state = _State.IDLE
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        self._cmd_lock = threading.Lock()
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        self._prev_actions = np.zeros(_ACT_DIM, dtype=np.float32)

        self._network = _load_network(weights_path, submodule_path, config_path)
        logger.info("PolicyExecutor ready: %s", weights_path)

    # ── Public interface ──────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state.name.lower()

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        with self._cmd_lock:
            self._vx, self._vy, self._wz = vx, vy, wz

    def get_command(self) -> tuple[float, float, float]:
        with self._cmd_lock:
            return self._vx, self._vy, self._wz

    def enable(self) -> None:
        with self._state_lock:
            if self._state != _State.IDLE:
                return
            self._state = _State.RUNNING
        self._prev_actions[:] = 0.0
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="policy")
        self._thread.start()
        logger.info("Policy enabled")

    def disable(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        with self._state_lock:
            self._state = _State.IDLE
        logger.info("Policy disabled")

    # ── Thread loop ───────────────────────────────────────────────────────────

    def _run(self) -> None:
        upright_cycles = 0
        recover_cycles = 0
        upright_hold_cycles = int(UPRIGHT_HOLD_S * _HZ)
        resume_delay_cycles = int(RESUME_DELAY_S * _HZ)

        while not self._stop_event.is_set():
            t_start = time.monotonic()

            imu = self._bus_manager.get_imu_state()
            proj_grav = _projected_gravity(imu.quaternion)
            tilt_deg = math.degrees(math.acos(float(np.clip(-proj_grav[2], -1.0, 1.0))))

            with self._state_lock:
                state = self._state

            if state == _State.RUNNING:
                if tilt_deg > FALL_THRESHOLD_DEG:
                    logger.warning("Fall detected (tilt=%.1f°) — depowering motors", tilt_deg)
                    self._disable_torques_fn()
                    self._prev_actions[:] = 0.0
                    upright_cycles = 0
                    with self._state_lock:
                        self._state = _State.FALLEN
                else:
                    self._inference_step(imu, proj_grav)

            elif state == _State.FALLEN:
                if tilt_deg < UPRIGHT_THRESHOLD_DEG:
                    upright_cycles += 1
                else:
                    upright_cycles = 0
                if upright_cycles >= upright_hold_cycles:
                    logger.info("Robot upright — recovering to default pose")
                    self._enable_torques_fn()
                    default_urdf = {j: self._default_offsets.get(j, 0.0) for j in POLICY_JOINT_ORDER}
                    self._write_fn(default_urdf, speed=150, raw=True)
                    recover_cycles = 0
                    with self._state_lock:
                        self._state = _State.RECOVERING

            elif state == _State.RECOVERING:
                recover_cycles += 1
                if recover_cycles >= resume_delay_cycles:
                    logger.info("Recovery complete — resuming policy")
                    self._prev_actions[:] = 0.0
                    with self._state_lock:
                        self._state = _State.RUNNING

            elapsed = time.monotonic() - t_start
            time.sleep(max(0.0, _DT - elapsed))

    def _inference_step(self, imu, proj_grav: np.ndarray) -> None:
        import torch

        positions = self._bus_manager.get_cached_positions()
        servo_states = self._bus_manager.get_servo_states()
        speed_by_joint = {s.joint_name: s.speed for s in servo_states} if servo_states else {}

        joint_pos_rel = np.array([
            (positions.get(j, self._default_offsets.get(j, 0.0))
             - self._default_offsets.get(j, 0.0)) * _DEG_TO_RAD
            for j in POLICY_JOINT_ORDER
        ], dtype=np.float32)

        # Servo speed is in steps/s; convert to rad/s
        joint_vel = np.array([
            speed_by_joint.get(j, 0) / STEPS_PER_DEG * _DEG_TO_RAD
            for j in POLICY_JOINT_ORDER
        ], dtype=np.float32)

        with self._cmd_lock:
            commands = np.array([self._vx, self._vy, self._wz], dtype=np.float32)

        obs = np.concatenate([
            np.array(imu.accel,  dtype=np.float32),   # 3  lin_acc  (body frame)
            np.array(imu.gyro,   dtype=np.float32),   # 3  ang_vel  (body frame)
            proj_grav.astype(np.float32),              # 3  projected_gravity
            commands,                                   # 3  [vx, vy, wz]
            joint_pos_rel,                              # 12 relative joint positions
            joint_vel,                                  # 12 joint velocities
            self._prev_actions,                         # 12 previous actions
        ])  # 48 total

        obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        with torch.no_grad():
            action_tensor = self._network.actor.get_action(obs_tensor)
        action_np = action_tensor.squeeze(0).cpu().numpy()

        self._prev_actions = action_np.copy()

        # Isaac Lab style: target_urdf = default + scale * action
        urdf_angles = {
            j: self._default_offsets.get(j, 0.0) + self._action_scale_deg * float(action_np[i])
            for i, j in enumerate(POLICY_JOINT_ORDER)
        }
        self._write_fn(urdf_angles, speed=0, raw=True)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _load_network(weights_path: str, submodule_path: str, config_path: str | None):
    import torch
    from gymnasium.spaces import Box, Dict

    resolved = str(Path(submodule_path).resolve())
    if resolved not in sys.path:
        sys.path.insert(0, resolved)

    from models.networks import ActorCriticWithTargets

    obs_space = Dict({
        "actor":  Box(-np.inf, np.inf, (_OBS_DIM,), dtype=np.float32),
        "critic": Box(-np.inf, np.inf, (_OBS_DIM,), dtype=np.float32),
    })
    action_space = Box(-1.0, 1.0, (_ACT_DIM,), dtype=np.float32)

    if config_path is not None:
        from config.schema import Config
        cfg = Config(config_path)
        network = ActorCriticWithTargets(
            obs_space=obs_space,
            action_space=action_space,
            actor_type="deterministic",
            critic_type="distributional",
            config=cfg,
            device=torch.device("cpu"),
        )
    else:
        network = ActorCriticWithTargets(
            obs_space=obs_space,
            action_space=action_space,
            actor_type="deterministic",
            critic_type="distributional",
            actor_sizes=[256, 128, 128],
            critic_sizes=[256, 128, 128],
            device=torch.device("cpu"),
        )

    network.load_state_dict(torch.load(weights_path, map_location="cpu"))
    network.actor.eval()
    return network


def _projected_gravity(quaternion) -> np.ndarray:
    """Rotate world [0, 0, -1] into body frame using the IMU quaternion."""
    w = float(quaternion[0])
    x = float(quaternion[1])
    y = float(quaternion[2])
    z = float(quaternion[3])
    # R^T @ [0, 0, -1] where R rotates body → world
    return np.array([
        -2.0 * (x * z - w * y),
        -2.0 * (y * z + w * x),
        -(w * w - x * x - y * y + z * z),
    ], dtype=np.float64)
