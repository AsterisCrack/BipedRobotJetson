"""The (vx, vy, wz) command limit: policy/command_limit.py, the optional bundle key
``command_limit`` and PolicyRunner.set_command. No hardware.
Run from BipedRobotJetson/:  python -m pytest tests/test_command_limit.py -q

tests/data/command_limit_golden.json is shared with BipedRobot/tests (and Gait Studio's copy): the
same inputs go through each implementation and the outputs must agree. Regenerate every copy
together (this one: ``python tests/test_command_limit.py --regen``).
"""
import json
import logging
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from policy.command_limit import SHAPES, limit_command

GOLDEN_PATH = Path(__file__).parent / "data" / "command_limit_golden.json"

R1 = {"lin_vel_x": [-0.2, 0.2], "lin_vel_y": [-0.15, 0.15], "ang_vel_z": [-1.2, 1.2]}
R2 = {"lin_vel_x": [-0.1, 0.3], "lin_vel_y": [-0.15, 0.15], "ang_vel_z": [-1.0, 1.0]}
RANGES = {"R1": R1, "R2": R2}
CMDS = [(0, 0, 0), (0.2, 0, 0), (-0.2, 0, 0), (0, 0.15, 0), (0, -0.15, 0), (0.2, 0.15, 0),
        (0.2, -0.15, 0.5), (-0.2, 0.15, -1.2), (0.1, 0.075, 0.3), (0.3, 0.3, 2.0),
        (0.14, 0.1, 0), (-0.05, 0.14, 0.9), (1e-9, 0, 0)]
SHAPE_ORDER = ["box", "ellipse_xy"]


def _cases():
    out = []
    for rname, ranges in RANGES.items():
        for shape in SHAPE_ORDER:
            for cmd in CMDS:
                res = limit_command(cmd, ranges, shape)
                out.append({"ranges": rname, "shape": shape, "cmd": [float(c) for c in cmd],
                            "out": [float(v) for v in res]})
    return out


def regenerate():
    """Same layout as BipedRobot/tests/data/make_command_limit_golden.py. The file is shared verbatim by
    BipedRobot/tests, Gait Studio and BipedRobotJetson/tests; the canonical generator is BipedRobot's
    (pure python). This numpy one agrees to the last ulp (~1e-17), so its bytes can differ: prefer
    regenerating in BipedRobot and copying the file to all three repos."""
    cases = _cases()
    with open(GOLDEN_PATH, "w", newline="\n") as f:
        f.write('{\n  "R1": ' + json.dumps(R1) + ',\n  "R2": ' + json.dumps(R2) + ',\n  "cases": [\n')
        f.write(",\n".join("    " + json.dumps(c) for c in cases))
        f.write("\n  ]\n}\n")


GOLDEN = json.loads(GOLDEN_PATH.read_text()) if GOLDEN_PATH.exists() else None
if GOLDEN is not None and "ranges" not in GOLDEN:          # shared layout: R1 / R2 at the top level
    GOLDEN["ranges"] = {"R1": GOLDEN["R1"], "R2": GOLDEN["R2"]}


# ---------------------------------------------------------------------------
# golden vectors
# ---------------------------------------------------------------------------

def test_golden_inputs_are_the_agreed_set():
    assert GOLDEN["ranges"] == RANGES
    got = [(c["ranges"], c["shape"], tuple(c["cmd"])) for c in GOLDEN["cases"]]
    want = [(r, s, tuple(float(x) for x in cmd)) for r in RANGES for s in SHAPE_ORDER for cmd in CMDS]
    assert got == want and len(got) == 52


def test_golden_reproduced():
    for c in GOLDEN["cases"]:
        out = limit_command(c["cmd"], GOLDEN["ranges"][c["ranges"]], c["shape"])
        assert np.allclose(out, c["out"], rtol=0.0, atol=1e-12), c


def test_golden_properties():
    for c in GOLDEN["cases"]:
        r = GOLDEN["ranges"][c["ranges"]]
        vx, vy, wz = c["out"]
        assert r["lin_vel_x"][0] <= vx <= r["lin_vel_x"][1]
        assert r["ang_vel_z"][0] <= wz <= r["ang_vel_z"][1]
        if c["shape"] == "ellipse_xy":
            bx = r["lin_vel_x"][1] if vx >= 0 else -r["lin_vel_x"][0]
            by = r["lin_vel_y"][1] if vy >= 0 else -r["lin_vel_y"][0]
            assert (vx / bx) ** 2 + (vy / by) ** 2 <= 1.0 + 1e-12


# ---------------------------------------------------------------------------
# limit_command
# ---------------------------------------------------------------------------

def test_box_is_the_per_axis_clamp():
    assert limit_command((1.0, -1.0, 5.0), R1, "box").tolist() == [0.2, -0.15, 1.2]
    assert limit_command((1.0, -1.0, 5.0), R1).tolist() == [0.2, -0.15, 1.2]      # default = box


def test_ellipse_corner_keeps_direction():
    vx, vy, wz = limit_command((0.2, 0.15, 0.0), R1, "ellipse_xy")
    assert vx == pytest.approx(0.2 / math.sqrt(2)) and vy == pytest.approx(0.15 / math.sqrt(2))
    assert vy / vx == pytest.approx(0.15 / 0.2) and wz == 0.0


def test_ellipse_leaves_axes_inside_and_wz_alone():
    assert limit_command((0.2, 0.0, 1.2), R1, "ellipse_xy").tolist() == [0.2, 0.0, 1.2]
    assert limit_command((0.0, -0.15, -1.2), R1, "ellipse_xy").tolist() == [0.0, -0.15, -1.2]
    assert limit_command((0.1, 0.075, 0.3), R1, "ellipse_xy").tolist() == [0.1, 0.075, 0.3]
    # wz stays at its own box maximum with (vx, vy) at the ellipse edge
    assert limit_command((0.2, 0.15, 9.0), R1, "ellipse_xy")[2] == 1.2


def test_ellipse_uses_the_bound_of_the_side():
    # R2 is asymmetric in vx: forward 0.3, backward 0.1
    fwd = limit_command((0.3, 0.15, 0.0), R2, "ellipse_xy")
    back = limit_command((-0.3, 0.15, 0.0), R2, "ellipse_xy")                     # clamps to -0.1 first
    assert fwd[0] == pytest.approx(0.3 / math.sqrt(2))
    assert back[0] == pytest.approx(-0.1 / math.sqrt(2)) and back[1] == pytest.approx(0.15 / math.sqrt(2))


def test_zero_bound_does_not_divide_by_zero():
    r = {"lin_vel_x": [-0.2, 0.0], "lin_vel_y": [-0.15, 0.15], "ang_vel_z": [-1.0, 1.0]}
    assert limit_command((0.3, 0.1, 0.0), r, "ellipse_xy").tolist() == [0.0, 0.1, 0.0]
    assert limit_command((-0.3, 0.15, 0.0), r, "ellipse_xy")[0] == pytest.approx(-0.2 / math.sqrt(2))


def test_input_not_mutated_and_returns_float_array():
    cmd = [0.2, 0.15, 0.0]
    out = limit_command(cmd, R1, "ellipse_xy")
    assert cmd == [0.2, 0.15, 0.0] and out.dtype == np.float64 and out.shape == (3,)


@pytest.mark.parametrize("bad", [(float("nan"), 0, 0), (0, float("inf"), 0), (0, 0, -float("inf"))])
def test_non_finite_raises(bad):
    with pytest.raises(ValueError, match="non-finite"):
        limit_command(bad, R1, "box")
    with pytest.raises(ValueError, match="non-finite"):
        limit_command(bad, R1, "ellipse_xy")


def test_bad_shape_and_bad_length_raise():
    assert SHAPES == ("box", "ellipse_xy")
    with pytest.raises(ValueError, match="shape"):
        limit_command((0, 0, 0), R1, "circle")
    with pytest.raises(ValueError, match="vx, vy, wz"):
        limit_command((0, 0), R1, "box")
    with pytest.raises(ValueError, match="ranges"):
        limit_command((0, 0, 0), {"lin_vel_x": [-1, 1]}, "box")


# ---------------------------------------------------------------------------
# deploy_config: the optional command_limit key
# ---------------------------------------------------------------------------

def _limit(shape="ellipse_xy", ranges=R1):
    d = {"shape": shape}
    if ranges is not None:
        d.update({k: list(v) for k, v in ranges.items()})
    return d


def _load(tmp_path, name="b", **over):
    from policy.deploy_config import load_bundle
    from runner_stubs import write_bundle
    return load_bundle(write_bundle(tmp_path / "models" / name, **over))


def test_absent_command_limit_is_box(tmp_path):
    cfg = _load(tmp_path, command_ranges=R1)
    assert cfg.command_limit is None and cfg.command_limit_shape == "box"
    assert cfg.command_ranges == R1


def test_ellipse_parsed(tmp_path):
    cfg = _load(tmp_path, command_ranges=R1, command_limit=_limit())
    assert cfg.command_limit_shape == "ellipse_xy"
    assert cfg.command_limit["lin_vel_x"] == [-0.2, 0.2] and cfg.command_limit["ang_vel_z"] == [-1.2, 1.2]
    assert cfg.format_version == 4                                # no format bump


def test_limit_ranges_fall_back_to_command_ranges(tmp_path):
    cfg = _load(tmp_path, command_ranges=R2, command_limit=_limit(ranges=None))
    assert cfg.command_limit["lin_vel_x"] == [-0.1, 0.3] and cfg.command_limit_shape == "ellipse_xy"


def test_limit_ranges_stand_in_for_missing_command_ranges(tmp_path):
    cfg = _load(tmp_path, command_limit=_limit())
    assert cfg.command_ranges == R1


def test_bad_command_limit_rejected(tmp_path):
    with pytest.raises(ValueError, match="shape"):
        _load(tmp_path, command_ranges=R1, command_limit=_limit(shape="circle"))
    with pytest.raises(ValueError, match="shape"):
        _load(tmp_path, command_ranges=R1, command_limit={"lin_vel_x": [-0.2, 0.2]})
    with pytest.raises(ValueError, match="command_limit"):
        _load(tmp_path, command_ranges=R1, command_limit="ellipse_xy")
    with pytest.raises(ValueError, match="lo <= hi"):
        _load(tmp_path, command_ranges=R1, command_limit=_limit(ranges={**R1, "lin_vel_y": [0.2, 0.1]}))
    # a zero-width axis (e.g. turning disabled in training) is a valid bundle
    zero_wz = {**R1, "ang_vel_z": [0.0, 0.0]}
    _load(tmp_path, command_ranges=zero_wz, command_limit=_limit(ranges=zero_wz))
    with pytest.raises(ValueError, match="finite"):
        _load(tmp_path, command_ranges=R1,
              command_limit=_limit(ranges={**R1, "ang_vel_z": [-1.0, float("inf")]}))
    with pytest.raises(ValueError, match="lin_vel_y"):
        _load(tmp_path, command_limit=_limit(ranges={k: v for k, v in R1.items() if k != "lin_vel_y"}))
    with pytest.raises(ValueError, match="no ranges"):
        _load(tmp_path, command_limit=_limit(ranges=None))


def test_inconsistent_ranges_rejected(tmp_path):
    with pytest.raises(ValueError, match="inconsistent"):
        _load(tmp_path, command_ranges=R1, command_limit=_limit(ranges=R2))


# ---------------------------------------------------------------------------
# PolicyRunner.set_command / status
# ---------------------------------------------------------------------------

def test_set_command_ellipse_corner(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    runner, _ = make_runner(tmp_path, monkeypatch, command_ranges=R1, command_limit=_limit())
    out = runner.set_command(0.2, 0.15, 0.0)
    assert out["command"] == pytest.approx([0.2 / math.sqrt(2), 0.15 / math.sqrt(2), 0.0])
    assert out["command"][0] == pytest.approx(0.1414213562)
    assert out["command"][1] == pytest.approx(0.1060660172)
    assert out["limited"] == out["command"] and out["limit_shape"] == "ellipse_xy"
    assert runner._command.tolist() == out["command"]


def test_set_command_box_bundle_unchanged(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    for over in ({"command_ranges": R1}, {"command_ranges": R1, "command_limit": _limit("box")}):
        runner, _ = make_runner(tmp_path, monkeypatch, **over)
        assert runner.set_command(0.2, 0.15, 0.0)["command"] == [0.2, 0.15, 0.0]
        assert runner.set_command(1.0, -1.0, 5.0)["command"] == [0.2, -0.15, 1.2]
        assert runner.status()["command_limit_shape"] == "box"


def test_set_command_without_command_ranges_keeps_fallback_box(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    runner, _ = make_runner(tmp_path, monkeypatch)
    assert runner.set_command(1.0, 1.0, 1.0)["command"] == [0.5, 0.3, 0.3]


def test_velocity_scale_applies_after_the_limit(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    runner, _ = make_runner(tmp_path, monkeypatch, command_ranges=R1, command_limit=_limit())
    runner.set_limits(velocity_scale=0.5)
    out = runner.set_command(0.2, 0.15, 1.2)
    assert out["limited"] == pytest.approx([0.2 / math.sqrt(2), 0.15 / math.sqrt(2), 1.2])
    assert out["command"] == pytest.approx([0.1 / math.sqrt(2), 0.075 / math.sqrt(2), 0.6])
    assert runner._command.tolist() == out["command"]


def test_nan_command_is_zero_with_warning(tmp_path, monkeypatch, caplog):
    from runner_stubs import make_runner
    runner, _ = make_runner(tmp_path, monkeypatch, command_ranges=R1, command_limit=_limit())
    runner.set_command(0.1, 0.05, 0.2)
    with caplog.at_level(logging.WARNING, logger="policy.policy_runner"):
        out = runner.set_command(float("nan"), 0.05, 0.2)
    assert out["command"] == [0.0, 0.0, 0.0] and runner._command.tolist() == [0.0, 0.0, 0.0]
    assert any("non-finite" in r.getMessage() for r in caplog.records)
    # the box path used to pass NaN straight through np.clip
    runner2, _ = make_runner(tmp_path, monkeypatch, "c", command_ranges=R1)
    assert runner2.set_command(0.1, float("inf"), 0.0)["command"] == [0.0, 0.0, 0.0]


def test_status_reports_shape_and_keeps_fields(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    runner, _ = make_runner(tmp_path, monkeypatch, command_ranges=R1, command_limit=_limit())
    st = runner.status()
    assert st["command_limit_shape"] == "ellipse_xy"
    assert st["command_ranges"] == R1 and "command" in st
    assert runner.command_limit_shape() == "ellipse_xy"


def _ws_dispatch(robot, msg):
    import asyncio
    from web.websocket import TelemetryBroadcaster

    class FakeWS:
        def __init__(self):
            self.sent = []

        async def send_text(self, text):
            self.sent.append(json.loads(text))

    ws = FakeWS()
    asyncio.run(TelemetryBroadcaster(robot)._dispatch(ws, json.dumps(msg)))
    return ws.sent


def test_websocket_echoes_the_limited_command_except_for_the_gamepad_stream(tmp_path, monkeypatch):
    from runner_stubs import make_runner
    runner, robot = make_runner(tmp_path, monkeypatch, command_ranges=R1, command_limit=_limit())
    sent = _ws_dispatch(robot, {"type": "set_velocity_command", "vx": 0.2, "vy": 0.15, "wz": 0.0})
    assert len(sent) == 1 and sent[0]["type"] == "command_applied" and sent[0]["shape"] == "ellipse_xy"
    assert sent[0]["command"] == pytest.approx([0.2 / math.sqrt(2), 0.15 / math.sqrt(2), 0.0])
    streamed = _ws_dispatch(robot, {"type": "set_velocity_command", "vx": 0.2, "vy": 0.15, "wz": 0.0,
                                    "stream": True})
    assert streamed == []                                    # no per-tick echo for the gamepad


if __name__ == "__main__":
    if "--regen" in sys.argv:
        regenerate()
        print("wrote", GOLDEN_PATH)
    else:
        sys.exit(pytest.main([__file__, "-q"]))
