"""Limit handling in PolicyRunner: load-level reset, set_limits bounds, LimitsBody, status() fields.

Uses the real PolicyRunner.load() with a bundle written to tmp_path and a fake ONNX session.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi import HTTPException
from pydantic import ValidationError

from policy.safety import DEFAULT_MAX_ACTION_RATE, DEFAULT_TILT_FAULT_RAD, SafetyLimits, select_tilt_fault
from runner_stubs import HOME_DEG, make_runner
from web.routers import policy as policy_router


def test_defaults_exposed():
    assert DEFAULT_MAX_ACTION_RATE == 0.5 == SafetyLimits().max_action_rate
    assert DEFAULT_TILT_FAULT_RAD == 0.784 == SafetyLimits().tilt_fault_rad


def test_second_bundle_resets_action_rate(tmp_path, monkeypatch):
    runner, _ = make_runner(tmp_path, monkeypatch, "a", max_action_rate=0.8)
    assert runner.limits.max_action_rate == pytest.approx(0.8)
    runner.load(str(tmp_path / "models" / "a"))                    # same bundle: still 0.8
    assert runner.limits.max_action_rate == pytest.approx(0.8)
    # bundle B has no limiter (None): back to the default, NOT the previous 0.8
    from runner_stubs import write_bundle
    runner.load(write_bundle(tmp_path / "models" / "b"))
    assert runner.limits.max_action_rate == DEFAULT_MAX_ACTION_RATE
    # 0 = trained without a limiter: also the default
    runner.load(write_bundle(tmp_path / "models" / "c", max_action_rate=0.0))
    assert runner.limits.max_action_rate == DEFAULT_MAX_ACTION_RATE


def test_load_discards_operator_value(tmp_path, monkeypatch):
    runner, _ = make_runner(tmp_path, monkeypatch)
    runner.set_limits(max_action_rate=1.5)
    runner.load(str(tmp_path / "models" / "a"))
    assert runner.limits.max_action_rate == DEFAULT_MAX_ACTION_RATE


@pytest.mark.parametrize("bundle, expected", [
    (None, 0.784), (0.5, 0.5), (0.784, 0.784), (1.2, 0.784), (0.0, 0.784), (-1.0, 0.784),
    (float("nan"), 0.784),
])
def test_select_tilt_never_loosens(bundle, expected):
    assert select_tilt_fault(bundle) == pytest.approx(expected)


def test_load_applies_bundle_tilt_but_never_loosens(tmp_path, monkeypatch):
    from runner_stubs import write_bundle
    runner, _ = make_runner(tmp_path, monkeypatch, tilt_fault_rad=0.6)
    assert runner.limits.tilt_fault_rad == pytest.approx(0.6)
    runner.load(write_bundle(tmp_path / "models" / "loose", tilt_fault_rad=1.2))
    assert runner.limits.tilt_fault_rad == pytest.approx(0.784)
    runner.load(write_bundle(tmp_path / "models" / "none"))        # key absent -> default
    assert runner.limits.tilt_fault_rad == pytest.approx(0.784)
    assert runner.safety.limits is runner.limits                    # the monitor sees the same object


@pytest.mark.parametrize("kw", [
    {"max_action_rate": 0.0}, {"max_action_rate": 2.5}, {"max_action_rate": float("nan")},
    {"velocity_scale": -0.1}, {"velocity_scale": 1.5},
    {"tilt_fault_rad": math.radians(46)}, {"tilt_fault_rad": math.radians(5)},
])
def test_set_limits_rejects_out_of_range_atomically(tmp_path, monkeypatch, kw):
    runner, _ = make_runner(tmp_path, monkeypatch)
    before = (runner.limits.max_action_rate, runner.limits.velocity_scale, runner.limits.tilt_fault_rad)
    with pytest.raises(ValueError):
        runner.set_limits(**{"velocity_scale": 0.5, **kw})             # a valid field alongside must not apply
    assert before == (runner.limits.max_action_rate, runner.limits.velocity_scale,
                      runner.limits.tilt_fault_rad)


def test_set_limits_accepts_edges(tmp_path, monkeypatch):
    runner, _ = make_runner(tmp_path, monkeypatch)
    out = runner.set_limits(max_action_rate=0.05, velocity_scale=0.0, tilt_fault_rad=math.radians(45))
    assert out["limits"]["max_action_rate"] == 0.05 and out["limits"]["velocity_scale"] == 0.0
    assert out["limits"]["tilt_fault_deg"] == 45.0
    runner.set_limits(max_action_rate=2.0, velocity_scale=1.0, tilt_fault_rad=math.radians(10))
    assert runner.limits.max_action_rate == 2.0


@pytest.mark.parametrize("kw", [
    {"tilt_fault_deg": 9.9}, {"tilt_fault_deg": 45.1}, {"tilt_fault_deg": 60},
    {"max_action_rate": 0.04}, {"max_action_rate": 2.01}, {"max_action_rate": 0},
    {"velocity_scale": -0.01}, {"velocity_scale": 1.01},
])
def test_limits_body_rejects(kw):
    with pytest.raises(ValidationError):
        policy_router.LimitsBody(**kw)


def test_limits_body_accepts_edges_and_none():
    policy_router.LimitsBody()
    policy_router.LimitsBody(tilt_fault_deg=10, max_action_rate=0.05, velocity_scale=0.0)
    policy_router.LimitsBody(tilt_fault_deg=45, max_action_rate=2.0, velocity_scale=1.0)


def test_set_safety_route_applies_and_reports(tmp_path, monkeypatch):
    runner, robot = make_runner(tmp_path, monkeypatch)
    req = SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace(robot=robot)))
    out = policy_router.set_safety(policy_router.LimitsBody(velocity_scale=0.4), req)
    assert out["limits"]["velocity_scale"] == 0.4
    assert runner.limits.max_action_rate == DEFAULT_MAX_ACTION_RATE      # untouched
    # belt and braces: a body that skipped pydantic validation still gets a 422 from the runner
    with pytest.raises(HTTPException) as exc:
        policy_router.set_safety(policy_router.LimitsBody.model_construct(max_action_rate=9.0), req)
    assert exc.value.status_code == 422


def test_status_limits_and_bundle_fields(tmp_path, monkeypatch):
    runner, _ = make_runner(tmp_path, monkeypatch, max_action_rate=0.8)
    s = runner.status()
    assert s["limits"] == {"tilt_fault_rad": runner.limits.tilt_fault_rad,
                           "max_action_rate": 0.8, "velocity_scale": 1.0}
    assert s["format_version"] == 4 and s["action_map"] == "residual"
    assert s["gait_clock"]["mode"] == "fixed_period" and s["gait_clock"]["period"] == pytest.approx(0.7)
    # home in degrees, by policy joint name
    assert s["home_deg"]["l_knee_joint"] == -26.0 and s["home_deg"]["r_ankle_pitch_joint"] == -16.0
    assert [s["home_deg"][n] for n in runner.cfg.joint_names] == [float(v) for v in HOME_DEG]


def test_status_before_any_load(tmp_path):
    from policy.policy_runner import PolicyRunner
    from runner_stubs import FakeRobot
    s = PolicyRunner(FakeRobot(), str(tmp_path / "models")).status()
    assert s["format_version"] is None and s["action_map"] is None
    assert s["gait_clock"] is None and s["home_deg"] is None
    assert s["limits"]["max_action_rate"] == DEFAULT_MAX_ACTION_RATE
