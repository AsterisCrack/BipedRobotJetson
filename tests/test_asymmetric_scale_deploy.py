"""Asymmetric residual action scales on the Jetson side (bundle format v6): parsing, the piecewise
action -> target map, and the guarantee that v4/v5 bundles are untouched. No hardware, no onnxruntime.

Run from BipedRobotJetson/:  python -m pytest tests/test_asymmetric_scale_deploy.py -q
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_residual_deploy import HI, HOME, LO, SCALE, _bundle, _raw  # noqa: E402

POS = np.repeat([0.45, 0.65, 1.00, 0.95, 0.20, 0.55], 2)
NEG = np.repeat([0.65, 0.20, 0.85, 0.90, 0.65, 0.30], 2)


def _v6(**over):
    raw = _raw(format_version=6, action_scale_pos_rad=POS.tolist(), action_scale_neg_rad=NEG.tolist(), **over)
    del raw["action_scale_rad"]
    return raw


def test_v6_parses_and_maps_piecewise(tmp_path):
    cfg = _bundle(tmp_path, _v6())
    assert cfg.format_version == 6 and cfg.action_map == "residual"
    assert np.array_equal(cfg.action_scale_rad, POS) and np.array_equal(cfg.action_scale_neg_rad, NEG)
    a = np.random.default_rng(0).uniform(-1.4, 1.4, (200, 12))
    ac = np.clip(a, -1.0, 1.0)
    want = np.clip(HOME + np.where(ac >= 0.0, POS, NEG) * ac, LO, HI)
    got = np.stack([cfg.action_to_joint_targets(x) for x in a])
    assert np.array_equal(got, want)
    assert np.array_equal(cfg.action_to_joint_targets(np.zeros(12)), HOME)           # a = 0 is still home
    up = cfg.action_to_joint_targets(np.full(12, 1.0))
    dn = cfg.action_to_joint_targets(np.full(12, -1.0))
    assert np.allclose(up, np.minimum(HOME + POS, HI)) and np.allclose(dn, np.maximum(HOME - NEG, LO))
    assert up[6] == pytest.approx(HI[6])                                              # knee a = +1 hits the upper limit (clamp)
    assert not np.allclose(up - HOME, HOME - dn)                                      # really asymmetric


def test_v6_equal_sides_match_symmetric_bundle(tmp_path):
    (tmp_path / "s").mkdir()
    (tmp_path / "a").mkdir()
    sym = _bundle(tmp_path / "s", _raw(format_version=5))
    raw = _raw(format_version=6, action_scale_pos_rad=SCALE.tolist(), action_scale_neg_rad=SCALE.tolist())
    del raw["action_scale_rad"]
    asym = _bundle(tmp_path / "a", raw)
    for a in np.random.default_rng(1).uniform(-1.2, 1.2, (50, 12)):
        assert np.array_equal(sym.action_to_joint_targets(a), asym.action_to_joint_targets(a))


@pytest.mark.parametrize("fmt", [4, 5])
def test_symmetric_bundles_unchanged(tmp_path, fmt):
    cfg = _bundle(tmp_path, _raw(format_version=fmt))
    assert cfg.action_scale_neg_rad is None and np.array_equal(cfg.action_scale_rad, SCALE)
    for a in np.random.default_rng(2).uniform(-1.3, 1.3, (50, 12)):
        old = np.clip(HOME + SCALE * np.clip(a, -1.0, 1.0), LO, HI)              # the v4/v5 expression
        assert np.array_equal(cfg.action_to_joint_targets(a), old)


def test_v6_bundle_may_still_carry_the_symmetric_key(tmp_path):
    cfg = _bundle(tmp_path, _raw(format_version=6))
    assert cfg.action_scale_neg_rad is None and np.array_equal(cfg.action_scale_rad, SCALE)


@pytest.mark.parametrize("over, msg", [
    ({"format_version": 7}, "newer"),
    ({"format_version": 5}, "require format_version >= 6"),
    ({"action_scale_neg_rad": None}, "needs both"),
    ({"action_scale_pos_rad": None}, "needs both"),
    ({"action_scale_neg_rad": NEG[:11].tolist()}, "action_scale_neg_rad must be 12"),
    ({"action_scale_pos_rad": [0.0] + POS[1:].tolist()}, "action_scale_pos_rad must be 12"),
    ({"action_scale_neg_rad": [-0.1] + NEG[1:].tolist()}, "action_scale_neg_rad must be 12"),
    ({"action_scale_rad": SCALE.tolist()}, "both action_scale_rad"),
])
def test_bad_v6_bundles_rejected(tmp_path, over, msg):
    raw = _v6()
    for k, v in over.items():
        if v is None:
            del raw[k]
        else:
            raw[k] = v
    with pytest.raises(ValueError, match=msg):
        _bundle(tmp_path, raw)


def test_v7_refused_even_when_symmetric(tmp_path):
    with pytest.raises(ValueError, match="newer"):
        _bundle(tmp_path, _raw(format_version=7))


def test_trace_meta_carries_neg_scale_only_when_asymmetric(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    runner, _ = make_runner(tmp_path, monkeypatch)
    assert "action_scale_neg_rad" not in runner._trace_meta()
    runner.cfg.action_scale_rad, runner.cfg.action_scale_neg_rad = POS, NEG
    meta = runner._trace_meta()
    json.dumps(meta, default=str)
    assert np.allclose(meta["action_scale_rad"], POS) and np.allclose(meta["action_scale_neg_rad"], NEG)
