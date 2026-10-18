"""Shared stubs: a fake robot/bus, a bundle on disk and a PolicyRunner loaded from it, with no
hardware and no real ONNX model (onnxruntime.InferenceSession is replaced by a recording fake).

Imported by the test modules as ``from runner_stubs import ...`` (pytest puts tests/ on sys.path).
"""
import json
import time
from types import SimpleNamespace

import numpy as np

from robot.robot import Robot

HOME_DEG = [0, 0, 0, 0, -10, -10, -26, -26, 0, 0, -16, -16]
HOME = np.radians(HOME_DEG)
SCALE = np.repeat([0.6, 0.45, 0.8, 0.6, 0.45, 0.6], 2)
LIM_DEG = [[-45, 45], [-45, 45], [-25, 45], [-25, 45], [-90, 30], [-90, 30],
           [-120, 5], [-120, 5], [-60, 60], [-60, 60], [-35, 50], [-35, 50]]
LO, HI = np.radians([lo for lo, _ in LIM_DEG]), np.radians([hi for _, hi in LIM_DEG])
# Policy order is interleaved L/R ...
NAMES = ["l_hip_yaw", "r_hip_yaw", "l_hip_roll_joint", "r_hip_roll_joint", "l_hip_pitch_joint",
         "r_hip_pitch_joint", "l_knee_joint", "r_knee_joint", "l_ankle_roll_joint",
         "r_ankle_roll_joint", "l_ankle_pitch_joint", "r_ankle_pitch_joint"]
# ... the robot's is per-leg, so a positional mix-up cannot pass unnoticed.
ROBOT_NAMES = [n for n in NAMES if n.startswith("l_")] + [n for n in NAMES if n.startswith("r_")]
# robot.yaml default_position_deg stand-ins (URDF-space offsets), distinct per joint.
OFFSETS = {n: 3.0 * (i + 1) for i, n in enumerate(ROBOT_NAMES)}


def raw_bundle(**over):
    raw = {
        "format_version": 4, "action_map": "residual", "obs_dim": 50, "action_dim": 12,
        "history_size": 1, "obs_proprio_dim": 50,
        "obs_layout": {"imu_lin_acc_b": [0, 3], "imu_ang_vel_b": [3, 6], "projected_gravity_b": [6, 9],
                       "commands": [9, 12], "joint_pos_rel": [12, 24], "joint_vel": [24, 36],
                       "previous_actions": [36, 48], "gait_phase": [48, 50]},
        "history_order": "oldest_first", "joint_names": NAMES,
        "joint_limits_min": LO.tolist(), "joint_limits_max": HI.tolist(),
        "default_joint_pos": HOME.tolist(), "action_scale_rad": SCALE.tolist(),
        "obs_mean": [0.0] * 50, "obs_var": [1.0] * 50, "obs_epsilon": 1e-4,
        "action_filter_alpha": 0.4, "action_filter_applications_per_step": 4, "step_dt": 0.02,
        "control_hz": 50.0, "gait_clock_freq": 5.0,
        "gait_clock": {"mode": "fixed_period", "period": 0.7, "stand_threshold": 0.05},
        "actuator": {"model": "bam", "requires_pid": {"p": 32, "d": 0, "i": 0}},
    }
    raw.update(over)
    return raw


def write_bundle(directory, **over):
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "deploy_config.json").write_text(json.dumps(raw_bundle(**over)))
    (directory / "policy.onnx").write_bytes(b"")
    return str(directory)


class Log(list):
    """Call log; ``hook(kind)`` (if set) runs inside every goal write / torque enable, i.e.
    while arm() is mid-sequence."""
    hook = None


class FakeSession:
    """Stands in for onnxruntime.InferenceSession; records every observation it is fed."""
    feeds: list = []

    def __init__(self, *a, **k):
        pass

    def run(self, names, feed):
        FakeSession.feeds.append(np.array(feed["obs"], copy=True))
        return [np.zeros((1, 12), dtype=np.float32)]


class FakeServo:
    def __init__(self, name, log):
        self.joint_name = name
        self.default_position_deg = OFFSETS[name]
        self._cfg = SimpleNamespace(pid=SimpleNamespace(p=32, d=0, i=0))
        self.log = log
        self.fail_set_position = False

    def set_position(self, deg, speed=0):
        if self.fail_set_position:
            from hardware.serial_bus import SerialBusError
            raise SerialBusError("no reply")
        if getattr(self.log, "hook", None):
            self.log.hook("goal")
        self.log.append(("goal", self.joint_name, round(deg, 6), speed))

    def enable_torque(self):
        if getattr(self.log, "hook", None):
            self.log.hook("torque_on")
        self.log.append(("torque_on", self.joint_name))

    def disable_torque(self):
        self.log.append(("torque_off", self.joint_name))


class FakeBusManager:
    def __init__(self, servos, log):
        self._servos = servos
        self.log = log
        self.has_velocity = True
        # URDF degrees in robot order: a pose 20 deg away from logical zero on every joint.
        self.positions = [OFFSETS[s.joint_name] + 20.0 for s in servos]
        self.stamp_age_s = 0.01          # age of the cached read; None = never stamped

    def get_rl_state(self):
        stamp = None if self.stamp_age_s is None else time.monotonic() - self.stamp_age_s
        return {"positions": list(self.positions), "servo_missing": [], "servo_stamp": stamp,
                "projected_gravity": (0.0, 0.0, -1.0)}

    def set_target_positions(self, angles, speed=300):
        self.log.append(("queue", dict(angles), speed))


class FakeRobot:
    """Real Robot methods bound to fake servos, so the call ORDER is the real code's."""
    preload_goal_positions = Robot.preload_goal_positions
    enable_all_torques = Robot.enable_all_torques
    disable_all_torques = Robot.disable_all_torques
    policy_motion_locked = Robot.policy_motion_locked

    def __init__(self):
        self.log = Log()
        servos = [FakeServo(n, self.log) for n in ROBOT_NAMES]
        self._servos = {s.joint_name: s for s in servos}
        self._bus_manager = FakeBusManager(servos, self.log)
        self._policy = None
        self.sysid_active = False

    @property
    def policy(self):
        return self._policy

    def sync_write_positions(self, joint_angles, speed=300, raw=False):
        self.log.append(("sync_write", dict(joint_angles), speed, raw))


def make_runner(tmp_path, monkeypatch, bundle_name="a", robot=None, **over):
    """A PolicyRunner with a bundle loaded through the real PolicyRunner.load()."""
    import onnxruntime
    from policy.policy_runner import PolicyRunner
    monkeypatch.setattr(onnxruntime, "InferenceSession", FakeSession)
    robot = robot or FakeRobot()
    runner = PolicyRunner(robot, str(tmp_path / "models"))
    robot._policy = runner
    runner.load(write_bundle(tmp_path / "models" / bundle_name, **over))
    return runner, robot
