"""begin() meta must carry what is needed to interpret a trace offline (not the ONNX/normaliser)."""
import json
import threading

import numpy as np
import pytest

from policy.state_machine import PolicyState
from policy.trace import TraceRecorder
from runner_stubs import HOME, HI, LO, NAMES, OFFSETS, SCALE, make_runner

REQUIRED = [
    "format_version", "history_size", "control_hz", "action_filter_alpha",
    "action_filter_applications", "default_joint_pos", "action_scale_rad", "joint_limits_min",
    "joint_limits_max", "default_position_deg", "actuator", "joint_names", "action_map",
    "gait_clock", "step_dt", "limits", "obs_layout", "model",
]


def _wait_for_save():
    for t in threading.enumerate():
        if t.name == "trace-save":
            t.join(timeout=5)


def test_start_records_replay_meta(tmp_path, monkeypatch):
    runner, robot = make_runner(tmp_path, monkeypatch, max_action_rate=0.8)
    runner.traces = TraceRecorder(str(tmp_path / "traces"))
    runner.sm.to(PolicyState.ARMING)
    runner.sm.to(PolicyState.ARMED)
    runner.start()
    assert runner.traces.active
    runner.traces.record(frame=np.zeros(50))
    runner.stop()
    _wait_for_save()

    files = list((tmp_path / "traces").glob("*.npz"))
    assert len(files) == 1
    with np.load(files[0]) as d:
        meta = json.loads(str(d["meta"]))

    for k in REQUIRED:
        assert k in meta, k
    assert meta["format_version"] == 4 and meta["history_size"] == 1 and meta["control_hz"] == 50.0
    assert meta["action_filter_alpha"] == 0.4 and meta["action_filter_applications"] == 4
    assert meta["action_map"] == "residual" and meta["joint_names"] == NAMES
    assert np.allclose(meta["default_joint_pos"], HOME)
    assert np.allclose(meta["action_scale_rad"], SCALE)
    assert np.allclose(meta["joint_limits_min"], LO) and np.allclose(meta["joint_limits_max"], HI)
    assert meta["default_position_deg"] == OFFSETS            # robot.yaml offsets, by joint name
    assert meta["actuator"]["requires_pid"] == {"p": 32, "d": 0, "i": 0}
    assert meta["gait_clock"]["mode"] == "fixed_period"
    assert meta["limits"]["max_action_rate"] == pytest.approx(0.8)
    assert meta["end_reason"] == "stop" and meta["steps"] == 1


def test_non_residual_bundle_has_null_scale(tmp_path, monkeypatch):
    runner, _ = make_runner(
        tmp_path, monkeypatch, format_version=2, action_map="centered", default_joint_pos=[0.0] * 12,
        action_scale_rad=None, gait_clock=None)
    meta = runner._trace_meta()
    assert meta["action_scale_rad"] is None and meta["format_version"] == 2
    assert meta["gait_clock"]["mode"] == "speed_proportional"
    json.dumps(meta, default=str)
