"""Loader for an exported policy bundle.

A bundle is a directory produced by BipedRobot/src/isaaclab/export_onnx.py::

    policy.onnx           [1, obs_dim] float32 -> [1, action_dim] float32
    deploy_config.json    normalizer stats, joint limits, timing, layout
    checkpoint_info.json  provenance (optional at load time)

Validation is deliberately strict and happens at load, not at the first control
step -- a missing key discovered mid-gait is a fall.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

import numpy as np

_REQUIRED = (
    "obs_dim", "action_dim", "history_size", "obs_proprio_dim", "obs_layout",
    "history_order", "joint_names", "joint_limits_min", "joint_limits_max",
    "obs_mean", "obs_var", "obs_epsilon", "action_filter_alpha",
    "action_filter_applications_per_step", "step_dt", "gait_clock_freq",
)


@dataclass
class DeployConfig:
    bundle_dir: str
    onnx_path: str

    obs_dim: int
    action_dim: int
    history_size: int
    obs_proprio_dim: int
    obs_layout: dict
    joint_names: list

    joint_limits_min: np.ndarray
    joint_limits_max: np.ndarray
    joint_range: np.ndarray
    default_joint_pos: np.ndarray

    obs_mean: np.ndarray
    obs_scale: np.ndarray          # precomputed 1/sqrt(var + eps)

    action_filter_alpha: float
    action_filter_applications: int
    step_dt: float
    control_hz: float
    gait_clock_freq: float

    command_ranges: dict
    tilt_fault_rad: float
    # "absolute" (format v1, e.g. walk_v1) or "centered" (v2+). See action_to_joint_targets.
    action_map: str = "absolute"
    # Servo model the policy was trained against (v2+), e.g. {"model": "bam", "kp_fw": 32,
    # "requires_pid": {"p": 32, "d": 0, "i": 0}, ...}. None for v1 bundles.
    actuator: dict | None = None
    format_version: int = 1
    info: dict = field(default_factory=dict)

    @property
    def name(self) -> str:
        return os.path.basename(self.bundle_dir.rstrip(os.sep))

    def normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        """Stage-1 normalizer: (x - mean) / sqrt(var + eps).

        Stage 2 lives inside policy.onnx as baked-in constants. BOTH are required --
        applying only one yields a policy that looks plausible and walks into the
        floor. See export_onnx.py's module docstring.
        """
        return ((obs - self.obs_mean) * self.obs_scale).astype(np.float32)

    def action_to_joint_targets(self, action: np.ndarray) -> np.ndarray:
        """[-1,1] -> joint position targets in radians (logical space).

        Must match BipedEnv.actions_to_targets for the bundle's action_map exactly:
          absolute  affine onto the full joint-limit box, so a = 0 is the limit MIDPOINT
          centered  a = 0 is the default pose and a = +-1 reach each limit, piecewise
                    linear on either side
        """
        a = np.clip(action, -1.0, 1.0)
        if self.action_map == "centered":
            span = np.where(a >= 0.0, self.joint_limits_max - self.default_joint_pos,
                            self.default_joint_pos - self.joint_limits_min)
            return self.default_joint_pos + a * span
        return self.joint_limits_min + (a + 1.0) * 0.5 * self.joint_range


def load_bundle(bundle_dir: str) -> DeployConfig:
    bundle_dir = os.path.abspath(bundle_dir)
    cfg_path = os.path.join(bundle_dir, "deploy_config.json")
    onnx_path = os.path.join(bundle_dir, "policy.onnx")

    if not os.path.isfile(cfg_path):
        raise FileNotFoundError(f"no deploy_config.json in {bundle_dir}")
    if not os.path.isfile(onnx_path):
        raise FileNotFoundError(f"no policy.onnx in {bundle_dir}")

    with open(cfg_path, "r", encoding="utf-8") as fh:
        raw = json.load(fh)

    missing = [k for k in _REQUIRED if k not in raw]
    if missing:
        raise ValueError(f"deploy_config.json missing keys: {missing}")

    obs_dim = int(raw["obs_dim"])
    action_dim = int(raw["action_dim"])
    history_size = int(raw["history_size"])
    proprio = int(raw["obs_proprio_dim"])

    if obs_dim != history_size * proprio:
        raise ValueError(
            f"obs_dim {obs_dim} != history_size {history_size} * proprio {proprio}")
    if raw["history_order"] != "oldest_first":
        raise ValueError(
            f"unsupported history_order {raw['history_order']!r}; runtime assumes "
            f"oldest_first (newest frame last)")
    if len(raw["joint_names"]) != action_dim:
        raise ValueError("joint_names length != action_dim")

    obs_mean = np.asarray(raw["obs_mean"], dtype=np.float64)
    obs_var = np.asarray(raw["obs_var"], dtype=np.float64)
    if obs_mean.shape != (obs_dim,) or obs_var.shape != (obs_dim,):
        raise ValueError(
            f"normalizer stats are {obs_mean.shape}/{obs_var.shape}, expected ({obs_dim},)")
    if not np.all(np.isfinite(obs_mean)) or not np.all(np.isfinite(obs_var)):
        raise ValueError("normalizer stats contain non-finite values")
    if np.any(obs_var < 0):
        raise ValueError("normalizer variance has negative entries")

    lo = np.asarray(raw["joint_limits_min"], dtype=np.float64)
    hi = np.asarray(raw["joint_limits_max"], dtype=np.float64)
    if lo.shape != (action_dim,) or hi.shape != (action_dim,):
        raise ValueError("joint limit arrays have the wrong shape")
    if np.any(hi <= lo):
        raise ValueError("joint_limits_max must exceed joint_limits_min for every joint")

    # Format v2 added the action map and servo model. Refuse anything newer than we
    # understand: a runtime that guesses the map would mis-execute every action.
    fmt = int(raw.get("format_version", 1))
    if fmt > 2:
        raise ValueError(f"bundle format_version {fmt} is newer than this runtime understands (2)")
    action_map = raw.get("action_map", "absolute" if fmt == 1 else None)
    if action_map not in ("absolute", "centered"):
        raise ValueError(f"unknown or missing action_map {action_map!r} (format v{fmt})")
    default_pos = np.asarray(raw.get("default_joint_pos", [0.0] * action_dim), dtype=np.float64)
    if action_map == "centered" and (np.any(default_pos >= hi) or np.any(default_pos <= lo)):
        raise ValueError("centered action map needs every default_joint_pos strictly inside its limits")

    eps = float(raw["obs_epsilon"])
    applications = int(raw["action_filter_applications_per_step"])
    if applications < 1:
        raise ValueError("action_filter_applications_per_step must be >= 1")

    info = {}
    info_path = os.path.join(bundle_dir, "checkpoint_info.json")
    if os.path.isfile(info_path):
        with open(info_path, "r", encoding="utf-8") as fh:
            info = json.load(fh)

    step_dt = float(raw["step_dt"])
    return DeployConfig(
        bundle_dir=bundle_dir,
        onnx_path=onnx_path,
        obs_dim=obs_dim,
        action_dim=action_dim,
        history_size=history_size,
        obs_proprio_dim=proprio,
        obs_layout={k: tuple(v) for k, v in raw["obs_layout"].items()},
        joint_names=list(raw["joint_names"]),
        joint_limits_min=lo,
        joint_limits_max=hi,
        joint_range=hi - lo,
        default_joint_pos=default_pos,
        obs_mean=obs_mean,
        obs_scale=1.0 / np.sqrt(obs_var + eps),
        action_filter_alpha=float(raw["action_filter_alpha"]),
        action_filter_applications=applications,
        step_dt=step_dt,
        control_hz=float(raw.get("control_hz", round(1.0 / step_dt))),
        gait_clock_freq=float(raw["gait_clock_freq"]),
        command_ranges=raw.get("command_ranges", {}),
        tilt_fault_rad=float(raw.get("tilt_fault_rad", 0.784)),
        action_map=action_map,
        actuator=raw.get("actuator"),
        format_version=fmt,
        info=info,
    )


def discover_bundles(models_root: str) -> list[dict]:
    """List loadable bundles under a directory, for the UI's model picker."""
    out: list[dict] = []
    if not os.path.isdir(models_root):
        return out
    for name in sorted(os.listdir(models_root)):
        d = os.path.join(models_root, name)
        if not os.path.isdir(d):
            continue
        if not os.path.isfile(os.path.join(d, "policy.onnx")):
            continue
        entry = {"name": name, "path": d, "valid": True, "error": None}
        try:
            cfg = load_bundle(d)
            entry.update(
                obs_dim=cfg.obs_dim, action_dim=cfg.action_dim,
                control_hz=cfg.control_hz, info=cfg.info,
                action_map=cfg.action_map, actuator=cfg.actuator,
            )
        except Exception as exc:  # surface broken bundles rather than hiding them
            entry.update(valid=False, error=str(exc))
        out.append(entry)
    return out
