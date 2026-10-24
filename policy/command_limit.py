"""The (vx, vy, wz) command limit: a box, or an ellipse in (vx, vy) with wz on its own box.

One definition shared by training (BipedRobot/utils/command_limit.py), Gait Studio and this
runtime. The three copies are held together by tests/data/command_limit_golden.json (the same
file in each repo). Numpy only, so the Jetson needs nothing extra.

    box          each axis clamped to its [lo, hi] range (what every bundle did before).
    ellipse_xy   the box first, then (vx/vx_b)^2 + (vy/vy_b)^2 <= 1, with vx_b the forward bound
                 (hi) for vx >= 0 and the backward bound (-lo) for vx < 0, likewise vy_b. An
                 over-limit (vx, vy) is scaled down along its own direction. wz is not touched.
"""

from __future__ import annotations

import math

import numpy as np

SHAPES = ("box", "ellipse_xy")
_KEYS = ("lin_vel_x", "lin_vel_y", "ang_vel_z")


def limit_command(cmd, ranges: dict, shape: str = "box") -> np.ndarray:
    """Limit ``cmd`` = (vx, vy, wz) to ``ranges`` ({key: [lo, hi]}) under ``shape``.

    Returns a new float64 array of 3. Raises ValueError for an unknown shape, a command that is
    not 3 finite numbers (NaN would otherwise sail through np.clip), or ranges that are missing.
    """
    if shape not in SHAPES:
        raise ValueError(f"unknown command limit shape {shape!r} (expected one of {SHAPES})")
    c = np.asarray(cmd, dtype=np.float64).reshape(-1)
    if c.shape != (3,):
        raise ValueError(f"command must be (vx, vy, wz), got {c.shape[0]} values")
    if not np.all(np.isfinite(c)):
        raise ValueError(f"non-finite command {c.tolist()}")
    try:
        (xlo, xhi), (ylo, yhi), (wlo, whi) = (ranges[k] for k in _KEYS)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"ranges must have {_KEYS} each as [lo, hi]: {exc!r}") from exc

    vx = float(np.clip(c[0], xlo, xhi))
    vy = float(np.clip(c[1], ylo, yhi))
    wz = float(np.clip(c[2], wlo, whi))
    if shape == "ellipse_xy":
        bx = float(xhi) if vx >= 0.0 else -float(xlo)
        by = float(yhi) if vy >= 0.0 else -float(ylo)
        # After the clamp a component can only sit on a side whose bound is > 0, or be exactly 0
        # (a range that ends at 0). A zero component contributes nothing; guard the division.
        tx = 0.0 if vx == 0.0 else (vx / bx) if bx > 0.0 else math.inf
        ty = 0.0 if vy == 0.0 else (vy / by) if by > 0.0 else math.inf
        s = math.sqrt(tx * tx + ty * ty)
        if s > 1.0:
            if math.isinf(s):
                vx = vy = 0.0
            else:
                vx *= 1.0 / s
                vy *= 1.0 / s
    return np.array([vx, vy, wz], dtype=np.float64)
