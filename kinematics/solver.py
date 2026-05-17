from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np

from kinematics.chain import KinematicChain

logger = logging.getLogger(__name__)


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
    IK: Damped Least Squares (DLS) iteration.
        dq = J^T (J J^T + λ²I)^{-1} e
        Robustly handles near-singular Jacobians; always warm-started from
        the provided (or zero) joint angles.
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
    # Inverse kinematics — DLS iteration
    # ------------------------------------------------------------------

    def ik(
        self,
        leg: str,
        target_pos: np.ndarray,
        target_rot: np.ndarray | None = None,
        initial_angles_deg: list[float] | None = None,
        position_weight: float = 1.0,
        orientation_weight: float = 0.1,
        posture_weight: float = 1e-4,
    ) -> IKResult:
        """
        Solve IK for one leg using Damped Least Squares (DLS).

        Args:
            leg:                 'left' or 'right'
            target_pos:          desired foot position [x, y, z] in base_link (metres)
            target_rot:          desired foot 3×3 rotation matrix (None = position-only)
            initial_angles_deg:  warm-start joint angles in degrees (defaults to zeros)
            position_weight:     scales position error rows of the Jacobian
            orientation_weight:  scales orientation error rows (ignored if target_rot=None)
            posture_weight:      pull toward warm-start (null-space regularization)

        Returns:
            IKResult with success flag, joint angles (degrees), and residual error.
        """
        n = len(self._chain.get_chain(leg))

        q = np.radians(initial_angles_deg) if initial_angles_deg else np.zeros(n)
        q = np.clip(q, -math.pi, math.pi)
        x0 = q.copy()   # posture reference (stay near warm-start)

        _LAMBDA_SQ  = 0.01   # DLS damping  (λ = 0.1); increase if oscillation
        _MAX_ITER   = 600
        _MAX_DQ     = 0.15   # radians per iteration (≈ 8.6° max step)
        _POS_TOL    = 5e-7   # early-stop position tolerance
        _SUCCESS_M  = 5e-3   # 5 mm convergence threshold

        for _ in range(_MAX_ITER):
            J_pos, J_omega, T_end = self._chain.full_jacobian(leg, q.tolist())
            pos_err = target_pos - T_end[:3, 3]

            if target_rot is None:
                # Position-only: 3×n Jacobian
                J = position_weight * J_pos
                e = position_weight * pos_err
            else:
                # Full 6-DOF: stack position and orientation rows
                R_e = T_end[:3, :3]
                # Rotation error as axis-angle vector (half of skew-symmetric part)
                R_err = target_rot @ R_e.T
                rot_err = np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ]) * 0.5
                J = np.vstack([position_weight * J_pos,
                               orientation_weight * J_omega])
                e = np.concatenate([position_weight * pos_err,
                                    orientation_weight * rot_err])

            # DLS step: dq = J^T (J J^T + λ²I)^{-1} e
            m = J.shape[0]
            dq = J.T @ np.linalg.solve(J @ J.T + _LAMBDA_SQ * np.eye(m), e)

            # Null-space pull toward warm-start posture
            if posture_weight > 0:
                dq += posture_weight * (x0 - q)

            # Limit step size to prevent large jumps
            dq_norm = np.linalg.norm(dq)
            if dq_norm > _MAX_DQ:
                dq *= _MAX_DQ / dq_norm

            q = np.clip(q + dq, -math.pi, math.pi)

            if np.linalg.norm(pos_err) < _POS_TOL:
                break

        # Final evaluation
        T_final = self._chain.fk(leg, q.tolist())
        pos_error = float(np.linalg.norm(T_final[:3, 3] - target_pos))
        angles_deg = [round(math.degrees(a), 3) for a in q]
        success = pos_error < _SUCCESS_M

        logger.debug(
            "IK %s target=[%.3f, %.3f, %.3f] warm=[%s] → [%s] err=%.4fm %s",
            leg,
            *target_pos,
            ", ".join(f"{a:.1f}" for a in (initial_angles_deg or [0.0] * n)),
            ", ".join(f"{a:.1f}" for a in angles_deg),
            pos_error,
            "OK" if success else "FAIL",
        )

        return IKResult(
            success=success,
            angles_deg=angles_deg,
            position_error_m=round(pos_error, 6),
            message="" if success else "DLS did not converge within 5 mm",
        )
