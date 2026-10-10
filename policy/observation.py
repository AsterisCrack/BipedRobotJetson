"""Observation assembly for the deployed policy.

Deliberately pure: numpy only, no hardware imports, no project imports. Everything
here is unit-testable on a laptop, which matters because a silent off-by-one in the
observation vector produces a policy that fails on hardware in a way that is very
hard to debug from the outside.

The 50-dim proprio frame mirrors biped_env._get_observations exactly:

    [0:3]   imu specific force, body frame (INCLUDES gravity: ~+9.81 z when upright)
    [3:6]   gyro, body frame, rad/s
    [6:9]   projected gravity, body frame, UNIT vector (~[0,0,-1] when upright)
    [9:12]  commands [v_x, v_y, w_z]
    [12:24] joint_pos - default_joint_pos, rad   (V2 defaults are all zero)
    [24:36] joint_vel, rad/s
    [36:48] previous action, [-1, 1]
    [48:50] [sin(gait_phase), cos(gait_phase)]

The actor consumes ``history_size`` of these stacked OLDEST-FIRST, so the newest
frame occupies the LAST proprio_dim entries.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.81


class JointMapper:
    """Translates between the robot's joint ordering and the policy's.

    These differ and the difference is silent:

        Isaac  : interleaved   l_hip_yaw, r_hip_yaw, l_hip_roll, r_hip_roll, ...
        Jetson : per-leg       l_hip_yaw, l_hip_roll, ... then all right

    Mapping is built by NAME, never by index. ``get_rl_state()["positions"]`` is
    ordered by config/robot.yaml, which is not an enforced invariant -- reordering
    that file would silently permute the observation if we trusted position.

    Also handles the two frame conventions:
        URDF space    : what the servos report/accept (0 = servo calibration zero)
        logical space : what the policy uses (0 = standing)
        logical = urdf - default_position_deg
    """

    def __init__(
        self,
        policy_joint_names: list[str],
        robot_joint_names: list[str],
        default_position_deg: dict[str, float] | None = None,
    ):
        missing = set(policy_joint_names) - set(robot_joint_names)
        if missing:
            raise ValueError(
                f"robot is missing joints the policy needs: {sorted(missing)}\n"
                f"  policy wants: {policy_joint_names}\n"
                f"  robot has   : {robot_joint_names}"
            )
        self.policy_names = list(policy_joint_names)
        self.robot_names = list(robot_joint_names)
        # robot_to_policy[i] = index into the robot vector for policy slot i
        self._r2p = np.array([robot_joint_names.index(n) for n in policy_joint_names], dtype=int)
        defaults = default_position_deg or {}
        self._default_deg = np.array(
            [defaults.get(n, 0.0) for n in policy_joint_names], dtype=np.float64
        )

    @property
    def default_offsets_deg(self) -> np.ndarray:
        """Standing pose in URDF degrees, in POLICY order."""
        return self._default_deg.copy()

    def robot_deg_to_policy_rad(self, values_deg) -> np.ndarray:
        """URDF degrees in robot order -> logical radians in policy order."""
        v = np.asarray(values_deg, dtype=np.float64)[self._r2p]
        return np.radians(v - self._default_deg)

    def robot_rate_to_policy_rad(self, rates_deg_s) -> np.ndarray:
        """Angular RATES: permute + deg->rad, but no offset (offsets are constant)."""
        return np.radians(np.asarray(rates_deg_s, dtype=np.float64)[self._r2p])

    def policy_rad_to_logical_deg(self, values_rad) -> dict[str, float]:
        """Policy radians -> LOGICAL degrees keyed by joint name.

        No offset is added here. Policy space and the robot's "logical" space are the
        same convention -- both put standing at zero -- so this is purely rad->deg
        plus naming. The URDF offset is applied downstream by
        ``Robot.sync_write_positions(..., raw=False)``, which is also where the
        robot's own joint-limit clamp lives.

        Returns a dict rather than an array so the ordering cannot be lost
        downstream: the consumer indexes by joint name, not position.
        """
        deg = np.degrees(np.asarray(values_rad, dtype=np.float64))
        return {name: float(deg[i]) for i, name in enumerate(self.policy_names)}


def specific_force_from_linear_accel(
    linear_accel_b, projected_gravity_b, gravity: float = GRAVITY
) -> np.ndarray:
    """Reconstruct accelerometer specific force from the BNO055's linear accel.

    The policy was trained on Isaac's ``imu_lin_acc_b``, which is SPECIFIC FORCE --
    proper acceleration including gravity, exactly what a physical accelerometer
    reports (~+9.81 on z when level and still). The checkpoint's own normalizer
    confirms it: mean of that channel is +9.52.

    The BNO055's LIA register (0x28) is the opposite convention -- it has already
    subtracted the fused gravity estimate. Feeding LIA straight in would hand the
    policy a channel that reads ~0 where it expects ~+9.81.

        specific_force = a_body - g_body
                       = linear_accel - 9.81 * projected_gravity

    (At rest and level: linear_accel = 0, projected_gravity = [0,0,-1], so this
    returns [0, 0, +9.81].)

    Alternative if you prefer the unprocessed sensor: read the raw ACC register
    (0x08) instead. Note SMBus block reads cap at 32 bytes, so that needs a second
    I2C transaction rather than widening the existing 26-byte bulk read.
    """
    lin = np.asarray(linear_accel_b, dtype=np.float64)
    pg = np.asarray(projected_gravity_b, dtype=np.float64)
    return lin - gravity * pg


class ObservationBuilder:
    """Builds the stacked observation and owns the state the policy needs remembered.

    Stateful across control steps: history ring, previous action, gait phase. All of
    it must be re-seeded on arm/recovery or the first frames are garbage.
    """

    def __init__(self, cfg):
        self.proprio_dim = cfg.obs_proprio_dim
        self.history_size = cfg.history_size
        self.obs_dim = cfg.obs_dim
        self.action_dim = cfg.action_dim
        self.gait_clock_freq = cfg.gait_clock_freq
        gc = getattr(cfg, "gait_clock", None) or {"mode": "speed_proportional", "freq": cfg.gait_clock_freq}
        self.gait_mode = gc["mode"]
        self.gait_period = float(gc.get("period", 0.0))
        self.gait_stand_threshold = float(gc.get("stand_threshold", 0.05))
        self.step_dt = cfg.step_dt
        self.layout = cfg.obs_layout

        self._history = np.zeros((self.history_size, self.proprio_dim), dtype=np.float64)
        self._prev_action = np.zeros(self.action_dim, dtype=np.float64)
        self._gait_phase = 0.0
        self._primed = False

    # -- state ---------------------------------------------------------------

    @property
    def gait_phase(self) -> float:
        return self._gait_phase

    @property
    def previous_action(self) -> np.ndarray:
        return self._prev_action.copy()

    def set_previous_action(self, action) -> None:
        """Record the action actually sent, for the next frame's obs[36:48].

        Must be the CLIPPED action, matching Isaac: _pre_physics_step clamps to
        [-1,1] before storing, and step() copies that into previous_actions.
        """
        self._prev_action = np.clip(
            np.asarray(action, dtype=np.float64), -1.0, 1.0
        )

    def reset(self, standing_frame: np.ndarray | None = None) -> None:
        """Clear runtime state before a run.

        Isaac zero-fills history at reset, which makes the first ``history_size``
        steps produce saturated actions -- play.py works around it by sampling
        stochastically for the warmup. On hardware that would be an unpredictable
        lurch, so we instead fill every slot with the current standing frame. The
        policy then starts from a consistent, in-distribution view.
        """
        self._prev_action[:] = 0.0
        self._gait_phase = 0.0
        if standing_frame is None:
            self._history[:] = 0.0
            self._primed = False
        else:
            frame = np.asarray(standing_frame, dtype=np.float64)
            if frame.shape != (self.proprio_dim,):
                raise ValueError(
                    f"standing frame is {frame.shape}, expected ({self.proprio_dim},)")
            self._history[:] = frame
            self._primed = True

    # -- gait clock ----------------------------------------------------------

    def _advance_phase(self, cmd: np.ndarray) -> None:
        """Advance the gait clock one control step, exactly as training does.

        Once per control step, before the frame is built (Isaac advances it after physics, so the
        observation shows the post-step phase).

        fixed_period (gait-table policies, bundle v3): constant rate 2*pi/period while the command
            is at or above stand_threshold (|v_xy| or |wz|), held otherwise. Mirrors
            BipedEnv._advance_gait_phase. Phase 0 = right-foot touchdown, [0, 0.5) right stance.
        speed_proportional (v1/v2): rate proportional to commanded xy speed; zero command holds it.
        """
        if self.gait_mode == "fixed_period":
            thr = self.gait_stand_threshold
            moving = math.hypot(float(cmd[0]), float(cmd[1])) >= thr or abs(float(cmd[2])) >= thr
            if moving:
                self._gait_phase = (
                    self._gait_phase + self.step_dt * 2.0 * math.pi / self.gait_period
                ) % (2.0 * math.pi)
        else:
            v_cmd = float(np.linalg.norm(cmd[:2]))
            self._gait_phase = (
                self._gait_phase
                + self.step_dt * 2.0 * math.pi * self.gait_clock_freq * v_cmd
            ) % (2.0 * math.pi)

    # -- assembly ------------------------------------------------------------

    def build_frame(
        self,
        specific_force_b,
        ang_vel_b,
        projected_gravity_b,
        commands,
        joint_pos_rad,
        joint_vel_rad,
        advance_phase: bool = True,
    ) -> np.ndarray:
        """One 50-dim proprio frame. Does NOT touch the history ring."""
        cmd = np.asarray(commands, dtype=np.float64)

        if advance_phase:
            self._advance_phase(cmd)

        frame = np.concatenate([
            np.asarray(specific_force_b, dtype=np.float64),      # 3
            np.asarray(ang_vel_b, dtype=np.float64),             # 3
            np.asarray(projected_gravity_b, dtype=np.float64),   # 3
            cmd,                                                 # 3
            np.asarray(joint_pos_rad, dtype=np.float64),         # 12
            np.asarray(joint_vel_rad, dtype=np.float64),         # 12
            self._prev_action,                                   # 12
            np.array([math.sin(self._gait_phase),
                      math.cos(self._gait_phase)]),              # 2
        ])
        if frame.shape[0] != self.proprio_dim:
            raise ValueError(
                f"assembled frame is {frame.shape[0]} dims, expected {self.proprio_dim}")
        return frame

    def push(self, frame: np.ndarray) -> np.ndarray:
        """Append a frame and return the flattened stacked observation.

        Oldest-first: roll left and write the newest at the end, so the newest frame
        lands in obs[-proprio_dim:]. This matches Isaac's
        ``roll(shifts=-1); buf[:, -1] = obs``.
        """
        if not self._primed:
            # Never ran reset(with standing frame) -- prime from this frame so we
            # don't emit zero-padded history.
            self._history[:] = frame
            self._primed = True
        else:
            self._history[:-1] = self._history[1:]
            self._history[-1] = frame
        return self._history.reshape(-1)

    def validate_layout(self) -> None:
        """Assert the shipped layout actually tiles the frame, no gaps or overlaps."""
        spans = sorted(self.layout.values(), key=lambda s: s[0])
        cursor = 0
        for start, end in spans:
            if start != cursor:
                raise ValueError(f"obs_layout gap/overlap at {start} (expected {cursor})")
            cursor = end
        if cursor != self.proprio_dim:
            raise ValueError(
                f"obs_layout covers {cursor} dims, proprio is {self.proprio_dim}")
