"""ObservationBuilder._advance_phase fixed_period matches BipedEnv / BipedEnvV2 training clock.
No hardware. Run from BipedRobotJetson/:  python -m pytest tests/test_fixed_period_phase.py -q"""
import math
from types import SimpleNamespace

import numpy as np

from policy.observation import ObservationBuilder

DT = 0.02


def _builder(gait_clock):
    cfg = SimpleNamespace(gait_clock_freq=5.0, gait_clock=gait_clock, step_dt=DT, history_size=1,
                          obs_dim=50, action_dim=12, obs_layout={}, joint_names=[])
    b = ObservationBuilder.__new__(ObservationBuilder)
    b.gait_clock_freq = 5.0
    b.gait_mode = gait_clock["mode"]
    b.gait_period = float(gait_clock.get("period", 0.0))
    b.gait_stand_threshold = float(gait_clock.get("stand_threshold", 0.05))
    b.step_dt = DT
    b._gait_phase = 0.0
    return b


def test_fixed_period_rate_and_hold():
    b = _builder({"mode": "fixed_period", "period": 0.7, "stand_threshold": 0.05})
    b._advance_phase(np.array([0.1, 0.0, 0.0]))
    assert abs(b.gait_phase - 2 * math.pi * DT / 0.7) < 1e-12
    p = b.gait_phase
    b._advance_phase(np.array([0.0, 0.04, 0.0]))     # below threshold: held
    assert b.gait_phase == p
    b._advance_phase(np.array([0.0, 0.0, 0.2]))      # turning counts, as in training
    assert b.gait_phase > p


def test_speed_proportional_unchanged():
    b = _builder({"mode": "speed_proportional", "freq": 5.0})
    b._advance_phase(np.array([0.1, 0.0, 0.0]))
    assert abs(b.gait_phase - DT * 2 * math.pi * 5.0 * 0.1) < 1e-12
