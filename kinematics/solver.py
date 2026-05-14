from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from kinematics.chain import KinematicChain, rotation_matrix


@dataclass
class IKResult:
    success: bool
    angles_deg: list[float]
    position_error_m: float
    message: str = ""


class KinematicSolver:
    """
    Forward and inverse kinematics for a single leg.

    FK: exact chain computation via homogeneous transforms.
    IK: numerical optimisation (scipy SLSQP) with joint-limit bounds.
        Always warm-start from the provided (or current) joint angles.
    """

    def __init__(self, chain: KinematicChain) -> None:
        self._chain = chain

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def fk(self, leg: str, angles_deg: list[float]) -> dict:
        """
        Compute FK for one leg.

        Returns:
            {
              'position':        [x, y, z]       metres, in base_link frame
              'rotation_matrix': 3×3 ndarray
              'transform':       4×4 ndarray
            }
        """
        angles_rad = [math.radians(a) for a in angles_deg]
        T = self._chain.fk(leg, angles_rad)
        return {
            "position": T[:3, 3].tolist(),
            "rotation_matrix": T[:3, :3].tolist(),
            "transform": T.tolist(),
        }

    # ------------------------------------------------------------------
    # Inverse kinematics
    # ------------------------------------------------------------------

    def ik(
        self,
        leg: str,
        target_pos: np.ndarray,
        target_rot: np.ndarray | None = None,
        initial_angles_deg: list[float] | None = None,
        position_weight: float = 1.0,
        orientation_weight: float = 0.1,
    ) -> IKResult:
        """
        Solve IK for one leg.

        Args:
            leg:                 'left' or 'right'
            target_pos:          desired foot position [x, y, z] in base_link frame (metres)
            target_rot:          desired foot 3×3 rotation matrix (or None for position-only)
            initial_angles_deg:  warm-start joint angles in degrees (defaults to zeros)
            position_weight:     cost weight for position error
            orientation_weight:  cost weight for orientation error (ignored if target_rot=None)

        Returns:
            IKResult with success flag, joint angles (degrees), and residual error.
        """
        limits = self._chain.joint_limits(leg)
        bounds = [(lo, hi) for lo, hi in limits]

        x0 = np.radians(initial_angles_deg) if initial_angles_deg else np.zeros(len(bounds))
        # Clamp initial guess to bounds
        x0 = np.clip(x0, [b[0] for b in bounds], [b[1] for b in bounds])

        def cost(angles: np.ndarray) -> float:
            T = self._chain.fk(leg, angles.tolist())
            pos_err = np.linalg.norm(T[:3, 3] - target_pos)
            if target_rot is not None and orientation_weight > 0:
                R_diff = T[:3, :3] @ target_rot.T
                rot_err = np.linalg.norm(R_diff - np.eye(3))
            else:
                rot_err = 0.0
            return position_weight * pos_err**2 + orientation_weight * rot_err**2

        result = minimize(
            cost,
            x0,
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": 300, "ftol": 1e-8},
        )

        if not result.success:
            # Fallback: L-BFGS-B (box-constrained, no equality constraints)
            result = minimize(
                cost,
                x0,
                method="L-BFGS-B",
                bounds=bounds,
                options={"maxiter": 500, "ftol": 1e-10},
            )

        angles_deg = [round(math.degrees(a), 3) for a in result.x]
        T_final = self._chain.fk(leg, result.x.tolist())
        pos_error = float(np.linalg.norm(T_final[:3, 3] - target_pos))

        return IKResult(
            success=result.success and pos_error < 5e-3,
            angles_deg=angles_deg,
            position_error_m=round(pos_error, 6),
            message=result.message if not result.success else "",
        )
