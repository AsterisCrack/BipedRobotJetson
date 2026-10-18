"""Residual action map on the Jetson side: bundle parsing, action -> target formula (same as
BipedEnv.actions_to_targets), joint_pos_rel = q - home, arming ramp to home. No hardware, no
onnxruntime: deploy_config.py / observation.py are loaded by file path (the `policy` package
imports policy_runner) and policy_runner._step_arming is lifted out with ast.

Run from BipedRobotJetson/:  python -m pytest tests/test_residual_deploy.py -q
"""
import ast
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

POLICY = Path(__file__).resolve().parents[1] / "policy"


def _load(name):
    spec = importlib.util.spec_from_file_location(f"_{name}_under_test", POLICY / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


deploy = _load("deploy_config")
observation = _load("observation")

HOME_DEG = [0, 0, 0, 0, -10, -10, -26, -26, 0, 0, -16, -16]
HOME = np.radians(HOME_DEG)
SCALE = np.repeat([0.6, 0.45, 0.8, 0.6, 0.45, 0.6], 2)
LIM_DEG = [[-45, 45], [-45, 45], [-25, 45], [-25, 45], [-90, 30], [-90, 30],
           [-120, 5], [-120, 5], [-60, 60], [-60, 60], [-35, 50], [-35, 50]]
LO, HI = np.radians([lo for lo, _ in LIM_DEG]), np.radians([hi for _, hi in LIM_DEG])
NAMES = ["l_hip_yaw", "r_hip_yaw", "l_hip_roll_joint", "r_hip_roll_joint", "l_hip_pitch_joint",
         "r_hip_pitch_joint", "l_knee_joint", "r_knee_joint", "l_ankle_roll_joint",
         "r_ankle_roll_joint", "l_ankle_pitch_joint", "r_ankle_pitch_joint"]


def _raw(**over):
    raw = {
        "format_version": 4, "action_map": "residual", "obs_dim": 50, "action_dim": 12, "history_size": 1,
        "obs_proprio_dim": 50,
        "obs_layout": {"imu_lin_acc_b": [0, 3], "imu_ang_vel_b": [3, 6], "projected_gravity_b": [6, 9],
                       "commands": [9, 12], "joint_pos_rel": [12, 24], "joint_vel": [24, 36],
                       "previous_actions": [36, 48], "gait_phase": [48, 50]},
        "history_order": "oldest_first", "joint_names": NAMES,
        "joint_limits_min": LO.tolist(), "joint_limits_max": HI.tolist(),
        "default_joint_pos": HOME.tolist(), "action_scale_rad": SCALE.tolist(),
        "obs_mean": [0.0] * 50, "obs_var": [1.0] * 50, "obs_epsilon": 1e-4,
        "action_filter_alpha": 0.4, "action_filter_applications_per_step": 4, "step_dt": 0.02,
        "gait_clock_freq": 5.0,
        "gait_clock": {"mode": "fixed_period", "period": 0.8, "stand_threshold": 0.05},
    }
    raw.update(over)
    return raw


def _bundle(tmp_path, raw):
    (tmp_path / "deploy_config.json").write_text(json.dumps(raw))
    (tmp_path / "policy.onnx").write_bytes(b"")
    return deploy.load_bundle(str(tmp_path))


def test_parse_v4_bundle(tmp_path):
    cfg = _bundle(tmp_path, _raw())
    assert cfg.action_map == "residual" and cfg.format_version == 4
    assert np.allclose(cfg.default_joint_pos, HOME) and np.allclose(cfg.action_scale_rad, SCALE)


@pytest.mark.parametrize("over, msg", [
    ({"action_scale_rad": SCALE[:11].tolist()}, "action_scale_rad"),
    ({"action_scale_rad": [0.0] + SCALE[1:].tolist()}, "action_scale_rad"),
    ({"action_scale_rad": None}, "action_scale_rad"),
    ({"format_version": 3}, "requires format_version >= 4"),
    ({"format_version": 5}, "newer"),
    ({"default_joint_pos": (HI + 0.1).tolist()}, "strictly inside"),
    ({"default_joint_pos": HOME[:5].tolist()}, "default_joint_pos"),
])
def test_bad_v4_bundles_rejected(tmp_path, over, msg):
    raw = _raw(**over)
    if raw.get("action_scale_rad", 1) is None:
        del raw["action_scale_rad"]
    with pytest.raises(ValueError, match=msg):
        _bundle(tmp_path, raw)


def test_centered_and_absolute_bundles_unchanged(tmp_path):
    for amap, fmt in (("centered", 3), ("absolute", 2)):
        raw = _raw(action_map=amap, format_version=fmt, default_joint_pos=[0.0] * 12)
        del raw["action_scale_rad"]
        cfg = _bundle(tmp_path, raw)
        assert cfg.action_scale_rad is None
        a = np.random.default_rng(0).uniform(-1, 1, 12)
        if amap == "centered":
            span = np.where(a >= 0, HI, -LO)           # default 0
            assert np.allclose(cfg.action_to_joint_targets(a), a * span)
        else:
            assert np.allclose(cfg.action_to_joint_targets(a), LO + (a + 1) * 0.5 * (HI - LO))


def test_action_to_joint_targets_matches_training_formula(tmp_path):
    cfg = _bundle(tmp_path, _raw())
    rng = np.random.default_rng(7)
    for _ in range(200):
        a = rng.uniform(-1.5, 1.5, 12)
        want = np.clip(HOME + SCALE * np.clip(a, -1, 1), LO, HI)   # BipedEnv.actions_to_targets
        assert np.allclose(cfg.action_to_joint_targets(a), want, atol=1e-12)
    assert np.allclose(cfg.action_to_joint_targets(np.zeros(12)), HOME)
    assert np.allclose(cfg.action_to_joint_targets(np.full(12, 9.0)), np.minimum(HOME + SCALE, HI))


def test_joint_pos_rel_is_q_minus_home_and_old_behaviour_for_zero_default(tmp_path):
    cfg = _bundle(tmp_path, _raw())
    ob = observation.ObservationBuilder(cfg)
    ob.reset()
    q = HOME + np.linspace(-0.1, 0.1, 12)
    fr = ob.build_frame(np.zeros(3), np.zeros(3), [0, 0, -1], [0, 0, 0], q, np.zeros(12), advance_phase=False)
    assert np.allclose(fr[12:24], q - HOME)
    assert np.allclose(ob.build_frame(np.zeros(3), np.zeros(3), [0, 0, -1], [0, 0, 0], HOME, np.zeros(12),
                                      advance_phase=False)[12:24], 0.0)
    # centered/absolute bundles (default 0) or a cfg without the field: joint_pos passes through
    for dflt in ([0.0] * 12, None):
        c = SimpleNamespace(obs_proprio_dim=50, history_size=1, obs_dim=50, action_dim=12, gait_clock_freq=5.0,
                            step_dt=0.02, obs_layout=cfg.obs_layout, gait_clock=cfg.gait_clock)
        if dflt is not None:
            c.default_joint_pos = np.array(dflt)
        o2 = observation.ObservationBuilder(c)
        o2.reset()
        assert np.allclose(o2.build_frame(np.zeros(3), np.zeros(3), [0, 0, -1], [0, 0, 0], q, np.zeros(12),
                                          advance_phase=False)[12:24], q)


def test_policy_space_frame_is_training_frame():
    """JointMapper: policy rad = radians(urdf_deg - robot.yaml default). With the servo offset equal to the
    reading at the training-zero pose, a robot sitting at the training home reads q_policy == home, so
    joint_pos_rel == 0 there. (robot.yaml's default_position_deg must be the servo reading at URDF zero.)"""
    offs = {n: 3.0 * (i + 1) for i, n in enumerate(NAMES)}              # arbitrary servo calibration offsets
    robot_names = list(reversed(NAMES))                                  # different order: mapping is by name
    m = observation.JointMapper(NAMES, robot_names, offs)
    urdf_deg_robot = np.array([HOME_DEG[NAMES.index(n)] + offs[n] for n in robot_names])
    assert np.allclose(m.robot_deg_to_policy_rad(urdf_deg_robot), HOME)


# ---------------------------------------------------------------- arming ramp (policy_runner._step_arming)
def _lift_step_arming():
    src = (POLICY / "policy_runner.py").read_text(encoding="utf-8")
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == "_step_arming")
    ns = {"ARM_RAMP_S": 2.0, "ARM_SPEED": 7, "tilt_from_projected_gravity": lambda g: 0.0,
          "PolicyState": SimpleNamespace(ARMED="ARMED"), "FaultReason": SimpleNamespace(TILT="tilt")}
    exec(compile(ast.Module([fn], []), "policy_runner_arming", "exec"), ns)
    return ns["_step_arming"]


def test_arming_ramps_from_current_pose_to_home():
    step = _lift_step_arming()
    start = HOME + np.linspace(0.3, -0.3, 12)
    written, states = [], []
    runner = SimpleNamespace(
        sm=SimpleNamespace(seconds_in_state=0.0, to=lambda s, why: states.append(s),
                           fault_now=lambda *a: states.append("fault")),
        limits=SimpleNamespace(tilt_fault_rad=0.78), cfg=SimpleNamespace(default_joint_pos=HOME),
        _arm_start=start, _write_targets=lambda t, speed: written.append(np.array(t)),
        _safe_torque_off=lambda: None)
    for t in (0.0, 0.5, 1.0, 2.0, 3.0):
        runner.sm.seconds_in_state = t
        step(runner, {"projected_gravity": [0, 0, -1]})
        frac = min(1.0, t / 2.0)
        s = frac * frac * (3.0 - 2.0 * frac)
        assert np.allclose(written[-1], HOME + (start - HOME) * (1 - s))
    assert np.allclose(written[0], start) and np.allclose(written[-1], HOME)
    assert states == ["ARMED", "ARMED"]
    # zero default (centered/absolute) reduces to the old start * (1 - s)
    runner.cfg = SimpleNamespace(default_joint_pos=np.zeros(12))
    runner.sm.seconds_in_state = 1.0
    step(runner, {"projected_gravity": [0, 0, -1]})
    assert np.allclose(written[-1], start * (1 - 0.5))
