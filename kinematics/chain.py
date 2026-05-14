"""
Kinematic chain parser and forward kinematics.

Parses joint data from the URDF using stdlib xml.etree.ElementTree —
no external URDF library required, and mesh loading errors are avoided.

URDF flat-file note:
    Robot.urdf has three <xacro:include> lines at the top that reference
    Gazebo and transmission files (non-kinematic).  We strip those lines
    at load time to produce robot_flat.urdf, written atomically so a crash
    during generation cannot leave a corrupt file.
"""
from __future__ import annotations

import logging
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class JointInfo:
    name: str
    axis: np.ndarray        # unit vector [x, y, z]
    origin_xyz: np.ndarray  # translation from parent link
    origin_rpy: np.ndarray  # fixed rotation from parent link (roll, pitch, yaw)
    lower: float            # radians
    upper: float            # radians


class KinematicChain:
    """
    Serial kinematic chain for one leg, built from the robot URDF.

    Coordinate frame: base_link is root; FK returns 4×4 homogeneous
    transform of the foot link in the base_link frame.
    """

    LEFT_JOINTS = [
        "l_hip_yaw",
        "l_hip_roll_joint",
        "l_hip_pitch_joint",
        "l_knee_joint",
        "l_ankle_roll_joint",
        "l_ankle_pitch_joint",
    ]
    RIGHT_JOINTS = [
        "r_hip_yaw",
        "r_hip_roll_joint",
        "r_hip_pitch_joint",
        "r_knee_joint",
        "r_ankle_roll_joint",
        "r_ankle_pitch_joint",
    ]

    def __init__(self, urdf_path: str) -> None:
        flat_path = _ensure_flat_urdf(urdf_path)
        self._joints: dict[str, JointInfo] = _parse_joints(flat_path)

    def get_chain(self, leg: str) -> list[JointInfo]:
        names = self.LEFT_JOINTS if leg == "left" else self.RIGHT_JOINTS
        return [self._joints[n] for n in names]

    def fk(self, leg: str, angles_rad: list[float]) -> np.ndarray:
        """
        Forward kinematics: returns 4×4 homogeneous transform
        from base_link to the foot end-effector.
        """
        chain = self.get_chain(leg)
        T = np.eye(4)
        for joint, angle in zip(chain, angles_rad):
            T = T @ _joint_transform(joint, angle)
        return T

    def joint_limits(self, leg: str) -> list[tuple[float, float]]:
        return [(j.lower, j.upper) for j in self.get_chain(leg)]


# ---------------------------------------------------------------------------
# URDF flat-file preparation
# ---------------------------------------------------------------------------

def _ensure_flat_urdf(urdf_path: str) -> str:
    src = Path(urdf_path)
    if not src.exists():
        # Try relative to cwd
        src = Path.cwd() / urdf_path
    if not src.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    flat = src.parent / "robot_flat.urdf"
    if flat.exists():
        return str(flat)

    content = src.read_text()
    # Remove xacro namespace declaration and include lines
    content = re.sub(r'\s*xmlns:xacro="[^"]*"', "", content)
    content = re.sub(r'[ \t]*<xacro:include[^>]*/>\n?', "", content)

    # Atomic write: write to a temp file then rename so a crash cannot
    # leave a partially-written robot_flat.urdf.
    tmp = flat.parent / (flat.name + ".tmp")
    try:
        tmp.write_text(content)
        os.replace(str(tmp), str(flat))
    except OSError as exc:
        tmp.unlink(missing_ok=True)
        raise RuntimeError(f"Failed to write flat URDF to {flat}: {exc}") from exc

    return str(flat)


# ---------------------------------------------------------------------------
# URDF joint parser
# ---------------------------------------------------------------------------

def _parse_joints(urdf_path: str) -> dict[str, JointInfo]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints: dict[str, JointInfo] = {}

    for joint_el in root.findall("joint"):
        if joint_el.get("type") != "revolute":
            continue
        name = joint_el.get("name", "")

        origin_el = joint_el.find("origin")
        xyz = np.zeros(3)
        rpy = np.zeros(3)
        if origin_el is not None:
            xyz_s = origin_el.get("xyz", "0 0 0")
            rpy_s = origin_el.get("rpy", "0 0 0")
            xyz = np.array([float(v) for v in xyz_s.split()])
            rpy = np.array([float(v) for v in rpy_s.split()])

        axis_el = joint_el.find("axis")
        axis = np.array([0.0, 0.0, 1.0])
        if axis_el is not None:
            axis_s = axis_el.get("xyz", "0 0 1")
            axis = np.array([float(v) for v in axis_s.split()])
        norm = np.linalg.norm(axis)
        if norm > 1e-9:
            axis = axis / norm
        else:
            logger.warning(
                "Joint %r has near-zero axis norm; defaulting to [0, 0, 1]", name
            )
            axis = np.array([0.0, 0.0, 1.0])

        limit_el = joint_el.find("limit")
        lower = upper = 0.0
        if limit_el is not None:
            lower = float(limit_el.get("lower", "0"))
            upper = float(limit_el.get("upper", "0"))

        joints[name] = JointInfo(
            name=name,
            axis=axis,
            origin_xyz=xyz,
            origin_rpy=rpy,
            lower=lower,
            upper=upper,
        )

    return joints


# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------

def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """Roll-Pitch-Yaw (extrinsic XYZ) to 3×3 rotation matrix."""
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' rotation formula → 3×3 matrix."""
    k = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _joint_transform(joint: JointInfo, angle: float) -> np.ndarray:
    """4×4 homogeneous transform for a joint at the given angle (rad)."""
    R_origin = _rpy_to_matrix(joint.origin_rpy)
    R_joint = rotation_matrix(joint.axis, angle)
    R = R_origin @ R_joint

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = joint.origin_xyz
    return T
