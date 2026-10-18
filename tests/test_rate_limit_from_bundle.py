"""Selection of the runtime action-rate clip from a deploy bundle.

safety.py is loaded by file path: importing the ``policy`` package would pull in
policy_runner (onnxruntime / hardware deps).
"""
import importlib.util
import sys
from pathlib import Path

import pytest

_SAFETY = Path(__file__).resolve().parents[1] / "policy" / "safety.py"
_spec = importlib.util.spec_from_file_location("_safety_under_test", _SAFETY)
safety = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = safety  # dataclasses needs this
_spec.loader.exec_module(safety)

DEFAULT = safety.SafetyLimits().max_action_rate


def test_default_is_half():
    assert DEFAULT == 0.5


@pytest.mark.parametrize("bundle, expected", [
    (None, DEFAULT),
    (0, DEFAULT),
    (0.0, DEFAULT),
    (-0.1, DEFAULT),
    (0.3, 0.3),
    (2, 2.0),
    (50, 2.0),        # clamped to the operator maximum: a bundle cannot disable the clip
    (0.01, 0.05),
])
def test_select_action_rate(bundle, expected):
    assert safety.select_action_rate(bundle, DEFAULT) == pytest.approx(expected)
