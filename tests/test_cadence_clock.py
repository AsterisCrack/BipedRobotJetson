"""Bundle format v5: the cadence-grid gait clock (cadence follows the command, Gait Studio v2
tables). No hardware. Run from BipedRobotJetson/:  python -m pytest tests/test_cadence_clock.py -q

The golden vectors in tests/data/cadence_grid_golden.json are shared with BipedRobot/tests (the
Isaac and MuJoCo clocks are checked against the same file), so all three runtimes agree.
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from policy.cadence import CadenceGrid
from policy.observation import ObservationBuilder
from test_residual_deploy import _bundle, _raw

GOLDEN = json.loads((Path(__file__).parent / "data" / "cadence_grid_golden.json").read_text())
DT = 0.02


def _clock(**over):
    gc = {"mode": "cadence_grid", "grid_vx": GOLDEN["grid_vx"], "grid_vy": GOLDEN["grid_vy"],
          "grid_wz": GOLDEN["grid_wz"], "freq": GOLDEN["freq"], "stand_threshold": 0.05,
          "period": 2.0, "style_digest": "abc"}
    gc.update(over)
    return gc


def test_golden_vectors():
    g = CadenceGrid(GOLDEN["grid_vx"], GOLDEN["grid_vy"], GOLDEN["grid_wz"], GOLDEN["freq"])
    got = [g.frequency(c) for c in GOLDEN["commands"]]
    assert np.allclose(got, GOLDEN["expected"], atol=1e-9)


def test_v5_bundle_loads(tmp_path):
    cfg = _bundle(tmp_path, _raw(format_version=5, gait_clock=_clock()))
    assert cfg.format_version == 5 and cfg.gait_clock["mode"] == "cadence_grid"
    assert cfg.action_map == "residual"            # v5 still carries the v4 action map


@pytest.mark.parametrize("over, gc_over, msg", [
    ({"format_version": 4}, {}, "requires format_version >= 5"),
    ({"format_version": 5}, {"freq": [[[1.0]]]}, "shape"),
    ({"format_version": 5}, {"grid_vx": [0.2, 0.1, 0.0, -0.05, -0.2]}, "strictly increasing"),
    ({"format_version": 5}, {"freq": (np.asarray(GOLDEN["freq"]) * 0 + 9.0).tolist()}, "Hz"),
    ({"format_version": 5}, {"stand_threshold": -1.0}, "stand_threshold"),
])
def test_bad_cadence_bundles_rejected(tmp_path, over, gc_over, msg):
    with pytest.raises(ValueError, match=msg):
        _bundle(tmp_path, _raw(**over, gait_clock=_clock(**gc_over)))


def test_phase_runs_at_freq_of_the_command(tmp_path):
    cfg = _bundle(tmp_path, _raw(format_version=5, gait_clock=_clock()))
    b = ObservationBuilder(cfg)
    grid = CadenceGrid.from_gait_clock(cfg.gait_clock)
    for cmd in ([0.1, 0.0, 0.0], [0.2, 0.1, 0.5], [-0.15, -0.1, -1.0], [0.0, 0.0, 0.3]):
        before = b.gait_phase
        b._advance_phase(np.array(cmd))
        step = (b.gait_phase - before) % (2 * math.pi)
        assert step == pytest.approx(DT * 2 * math.pi * grid.frequency(cmd), abs=1e-12)
    held = b.gait_phase
    b._advance_phase(np.array([0.03, 0.0, 0.0]))     # below stand_threshold: held, as in training
    assert b.gait_phase == held
