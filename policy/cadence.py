"""Cadence-grid gait clock (bundle format v5). Pure numpy.

Gait Studio's v2 gait tables let the gait cycle follow the command, as human cadence does: slow
walking takes long, slow steps, fast walking quicker and longer ones. The policy's clock then
runs at freq(cmd) cycles/s, interpolated trilinearly in (vx, vy, wz) with each axis clamped to
the grid. That is the same rule as training (BipedRobot GaitTableReference.frequency, Isaac) and
sim2sim (BipedRobot utils.motion_reference.CadenceGrid, MuJoCo). Golden vectors shared with
BipedRobot: tests/data/cadence_grid_golden.json.
"""
from __future__ import annotations

import numpy as np

FREQ_RANGE_HZ = (0.2, 5.0)   # same plausibility band as the fixed-period clock (0.2-5 s)


class CadenceGrid:
    def __init__(self, grid_vx, grid_vy, grid_wz, freq) -> None:
        self.axes = [np.asarray(a, dtype=np.float64).reshape(-1) for a in (grid_vx, grid_vy, grid_wz)]
        for name, a in zip(("grid_vx", "grid_vy", "grid_wz"), self.axes):
            if a.size < 1 or not np.all(np.isfinite(a)) or np.any(np.diff(a) <= 0):
                raise ValueError(f"gait_clock.{name} must be finite and strictly increasing")
        shape = tuple(a.size for a in self.axes)
        self.freq = np.asarray(freq, dtype=np.float64)
        if self.freq.shape != shape:
            raise ValueError(f"gait_clock.freq has shape {self.freq.shape}, the grid needs {shape}")
        lo, hi = FREQ_RANGE_HZ
        if not np.all(np.isfinite(self.freq)) or self.freq.min() <= lo or self.freq.max() >= hi:
            raise ValueError(f"gait_clock.freq must be within {lo}-{hi} Hz "
                             f"(got {self.freq.min():.3f}-{self.freq.max():.3f})")

    @classmethod
    def from_gait_clock(cls, gc: dict) -> "CadenceGrid":
        return cls(gc["grid_vx"], gc["grid_vy"], gc["grid_wz"], gc["freq"])

    @staticmethod
    def _locate(grid: np.ndarray, x: float) -> tuple[int, float]:
        n = grid.size
        if n == 1:
            return 0, 0.0
        x = min(max(float(x), float(grid[0])), float(grid[-1]))
        i0 = int(np.clip(np.searchsorted(grid, x, side="right") - 1, 0, n - 2))
        return i0, (x - float(grid[i0])) / float(grid[i0 + 1] - grid[i0])

    def frequency(self, cmd) -> float:
        """Gait clock frequency [cycles/s] for a command (vx, vy, wz)."""
        idx = [self._locate(a, c) for a, c in zip(self.axes, cmd)]
        out = 0.0
        for di in (0, 1):
            for dj in (0, 1):
                for dk in (0, 1):
                    w = 1.0
                    cell = []
                    for d, (i0, a), n in zip((di, dj, dk), idx, self.freq.shape):
                        w *= a if d else 1.0 - a
                        cell.append(min(i0 + d, n - 1))
                    out += w * float(self.freq[tuple(cell)])
        return out

    def period_range(self) -> tuple[float, float]:
        return 1.0 / float(self.freq.max()), 1.0 / float(self.freq.min())
