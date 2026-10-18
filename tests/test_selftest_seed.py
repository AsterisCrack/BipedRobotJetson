"""policy.selftest: zero-filled history like start(), bundle summary and the measured-vs-home table."""
import sys
from types import SimpleNamespace

import numpy as np

from policy import selftest
from runner_stubs import FakeSession, HOME_DEG, NAMES, OFFSETS, ROBOT_NAMES, write_bundle
from policy.deploy_config import load_bundle


def _run(monkeypatch, tmp_path, argv, **over):
    import onnxruntime
    # history of 2 frames, so "zero-filled" and "filled with the standing frame" differ
    bundle = write_bundle(tmp_path / "b", obs_dim=100, history_size=2,
                          obs_mean=[0.0] * 100, obs_var=[1.0] * 100, **over)
    monkeypatch.setattr(selftest, "load_bundle", lambda _path: load_bundle(bundle))
    monkeypatch.setattr(onnxruntime, "InferenceSession", FakeSession)
    FakeSession.feeds = []
    monkeypatch.setattr(sys, "argv", ["selftest", "--model", "b", "--steps", "3"] + argv)
    return selftest.main()


def test_history_is_zero_filled_not_standing_frame(monkeypatch, tmp_path, capsys):
    _run(monkeypatch, tmp_path, ["--synthetic"])
    first = FakeSession.feeds[0][0]
    assert np.all(first[:50] == 0.0), "older slot must be zero, as after PolicyRunner.start()"
    assert np.any(first[50:] != 0.0)                     # the newest frame is real
    out = capsys.readouterr().out
    assert "format : v4, action_map = residual" in out
    assert "fixed period 0.700 s" in out
    assert "l_knee=-26.0" in out and "l_hip_pitch=-10.0" in out
    assert "skipped (synthetic" in out


def test_live_pose_vs_home_warns_beyond_10_deg(monkeypatch, tmp_path, capsys):
    import robot.config as rcfg
    import robot.robot as rrobot

    class Servo:
        def __init__(self, n):
            self.joint_name, self.default_position_deg = n, OFFSETS[n]

    class Bus:
        has_velocity = True
        _servos = [Servo(n) for n in ROBOT_NAMES]

        def get_rl_state(self):
            # logical = home everywhere except the left knee (home -26, measured -26 + 25)
            pos = []
            for n in ROBOT_NAMES:
                logical = HOME_DEG[NAMES.index(n)] + (25.0 if n == "l_knee_joint" else 2.0)
                pos.append(OFFSETS[n] + logical)
            return {"positions": pos, "velocities": [0] * 12, "velocities_deg_s": [0.0] * 12,
                    "linear_accel": (0.0, 0.0, 0.0), "angular_vel": (0.0, 0.0, 0.0),
                    "projected_gravity": (0.0, 0.0, -1.0)}

    class FakeRobot:
        _bus_manager = Bus()
        def __init__(self, settings): pass
        def initialize(self): pass
        def shutdown(self): pass

    monkeypatch.setattr(rcfg.Settings, "load", staticmethod(lambda: SimpleNamespace()))
    monkeypatch.setattr(rrobot, "Robot", FakeRobot)
    monkeypatch.setattr("time.sleep", lambda s: None)
    _run(monkeypatch, tmp_path, [])
    out = capsys.readouterr().out
    section = out.split("=== measured pose vs bundle home")[1].split("=== inference")[0]
    warn_lines = [l for l in section.splitlines() if "<-- WARN" in l]
    assert len(warn_lines) == 1 and warn_lines[0].startswith("l_knee_joint")
    assert "+25.00" in warn_lines[0] and "1 joint(s) are more than 10 deg" in section
