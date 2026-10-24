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
import math
import os
from dataclasses import dataclass, field

import numpy as np
from policy.cadence import CadenceGrid
from policy.command_limit import SHAPES as COMMAND_LIMIT_SHAPES

_REQUIRED = (
    "obs_dim", "action_dim", "history_size", "obs_proprio_dim", "obs_layout",
    "history_order", "joint_names", "joint_limits_min", "joint_limits_max",
    "obs_mean", "obs_var", "obs_epsilon", "action_filter_alpha",
    "action_filter_applications_per_step", "step_dt", "gait_clock_freq",
)

_COMMAND_AXES = ("lin_vel_x", "lin_vel_y", "ang_vel_z")


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
    # "absolute" (format v1, e.g. walk_v1), "centered" (v2+) or "residual" (v4). See action_to_joint_targets.
    action_map: str = "absolute"
    # residual only (v4): per-joint scale in rad, policy order. default_joint_pos is then the home pose.
    # v6 (asymmetric): action_scale_rad holds the scale for a >= 0 (the bundle's action_scale_pos_rad) and
    # action_scale_neg_rad the scale for a < 0; action_scale_neg_rad is None for a symmetric bundle (v4/v5).
    action_scale_rad: np.ndarray | None = None
    action_scale_neg_rad: np.ndarray | None = None
    # Servo model the policy was trained against (v2+), e.g. {"model": "bam", "kp_fw": 32,
    # "requires_pid": {"p": 32, "d": 0, "i": 0}, ...}. None for v1 bundles.
    actuator: dict | None = None
    format_version: int = 1
    # Gait clock the policy was trained with. {"mode": "speed_proportional", "freq": Hz} advances
    # with commanded speed (v1/v2); {"mode": "fixed_period", "period": s, "stand_threshold": m/s,
    # ...} is the gait-table clock (v3): fixed rate, held while the command is below the threshold;
    # {"mode": "cadence_grid", "grid_vx/vy/wz", "freq": [vx][vy][wz] Hz, "stand_threshold", ...}
    # (v5) runs at freq(cmd), trilinear in the command (policy/cadence.py), held the same way.
    gait_clock: dict = field(default_factory=dict)
    # Per-step action limiter the policy trained with. 0 or None = trained without a limiter
    # (None also = bundle predates it); the runtime then keeps its safety default.
    max_action_rate: float | None = None
    # Optional command limit (bundle key ``command_limit``): {"shape": "box" | "ellipse_xy",
    # "lin_vel_x": [lo, hi], "lin_vel_y": [lo, hi], "ang_vel_z": [lo, hi]}, ranges resolved and
    # validated at load. None = the bundle has none: a per-axis box from command_ranges, as before.
    command_limit: dict | None = None
    info: dict = field(default_factory=dict)

    @property
    def command_limit_shape(self) -> str:
        return self.command_limit["shape"] if self.command_limit else "box"

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
          residual  target = clip(default + action_scale_rad * a, min, max); the default is the
                    crouched home pose (BipedEnv.actions_to_targets, format v4). Format v6 has a
                    separate scale per side: action_scale_rad (= action_scale_pos_rad) for a >= 0 and
                    action_scale_neg_rad for a < 0, same clip.
        """
        a = np.clip(action, -1.0, 1.0)
        if self.action_map == "residual":
            scale = (self.action_scale_rad if self.action_scale_neg_rad is None
                     else np.where(a >= 0.0, self.action_scale_rad, self.action_scale_neg_rad))
            return np.clip(self.default_joint_pos + scale * a,
                           self.joint_limits_min, self.joint_limits_max)
        if self.action_map == "centered":
            span = np.where(a >= 0.0, self.joint_limits_max - self.default_joint_pos,
                            self.default_joint_pos - self.joint_limits_min)
            return self.default_joint_pos + a * span
        return self.joint_limits_min + (a + 1.0) * 0.5 * self.joint_range


def _axis_ranges(d: dict, what: str) -> dict:
    """{lin_vel_x, lin_vel_y, ang_vel_z} -> {key: [lo, hi]} (floats): all three present, finite, lo <= hi (zero width disables an axis)."""
    out = {}
    for k in _COMMAND_AXES:
        if k not in d:
            raise ValueError(f"{what} is missing {k}")
        try:
            lo, hi = (float(v) for v in d[k])
        except (TypeError, ValueError):
            raise ValueError(f"{what}.{k} must be [lo, hi], got {d[k]!r}") from None
        if not (math.isfinite(lo) and math.isfinite(hi)):
            raise ValueError(f"{what}.{k} must be finite, got [{lo}, {hi}]")
        if not lo <= hi:
            raise ValueError(f"{what}.{k} needs lo <= hi, got [{lo}, {hi}]")
        out[k] = [lo, hi]
    return out


def _parse_command_limit(raw_limit, raw_ranges):
    """The optional ``command_limit`` key -> (limit dict or None, command_ranges).

    Absent: (None, raw_ranges) -- nothing changes, a box from command_ranges. Present: the shape is
    validated, its ranges fall back to command_ranges when it has none, and if both exist they must
    agree (a bundle whose limit and command box differ is inconsistent). When it carries ranges they
    also stand in for a missing or partial command_ranges.
    """
    if raw_limit is None:
        return None, raw_ranges
    if not isinstance(raw_limit, dict):
        raise ValueError(f"command_limit must be an object, got {type(raw_limit).__name__}")
    shape = raw_limit.get("shape")
    if shape not in COMMAND_LIMIT_SHAPES:
        raise ValueError(f"command_limit.shape {shape!r} must be one of {COMMAND_LIMIT_SHAPES}")
    has_ranges = any(k in raw_limit for k in _COMMAND_AXES)
    if has_ranges:
        ranges = _axis_ranges(raw_limit, "command_limit")
        for k in _COMMAND_AXES:
            if k in raw_ranges and not np.allclose([float(v) for v in raw_ranges[k]], ranges[k],
                                                   rtol=0.0, atol=1e-9):
                raise ValueError(
                    f"command_limit.{k} {ranges[k]} and command_ranges.{k} {list(raw_ranges[k])} "
                    f"disagree: the bundle is inconsistent")
    elif raw_ranges:
        ranges = _axis_ranges(raw_ranges, "command_ranges")
    else:
        raise ValueError("command_limit has no ranges and the bundle has no command_ranges")
    return {"shape": shape, **ranges}, dict(ranges)


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
    if fmt > 6:
        raise ValueError(f"bundle format_version {fmt} is newer than this runtime understands (6)")
    action_map = raw.get("action_map", "absolute" if fmt == 1 else None)
    if action_map not in ("absolute", "centered", "residual"):
        raise ValueError(f"unknown or missing action_map {action_map!r} (format v{fmt})")
    if action_map == "residual" and fmt < 4:
        raise ValueError("action_map 'residual' requires format_version >= 4")
    default_pos = np.asarray(raw.get("default_joint_pos", [0.0] * action_dim), dtype=np.float64)
    if default_pos.shape != (action_dim,) or not np.all(np.isfinite(default_pos)):
        raise ValueError("default_joint_pos must be action_dim finite values")
    action_scale = action_scale_neg = None
    if action_map == "residual":
        if "action_scale_pos_rad" in raw or "action_scale_neg_rad" in raw:
            # v6: separate scales per side (a >= 0 / a < 0); the symmetric key must not be there as well
            if fmt < 6:
                raise ValueError("action_scale_pos_rad / action_scale_neg_rad require format_version >= 6")
            if "action_scale_rad" in raw:
                raise ValueError("bundle has both action_scale_rad and action_scale_pos_rad / action_scale_neg_rad")
            if "action_scale_pos_rad" not in raw or "action_scale_neg_rad" not in raw:
                raise ValueError("asymmetric residual bundle needs both action_scale_pos_rad and action_scale_neg_rad")
            sides = {}
            for key in ("action_scale_pos_rad", "action_scale_neg_rad"):
                sides[key] = np.asarray(raw[key], dtype=np.float64)
                if sides[key].shape != (action_dim,) or not np.all(np.isfinite(sides[key])) or np.any(sides[key] <= 0):
                    raise ValueError(f"{key} must be {action_dim} finite values > 0")
            action_scale, action_scale_neg = sides["action_scale_pos_rad"], sides["action_scale_neg_rad"]
        else:
            if "action_scale_rad" not in raw:
                raise ValueError("residual bundle has no action_scale_rad")
            action_scale = np.asarray(raw["action_scale_rad"], dtype=np.float64)
            if action_scale.shape != (action_dim,) or not np.all(np.isfinite(action_scale)) or np.any(action_scale <= 0):
                raise ValueError(f"action_scale_rad must be {action_dim} finite values > 0")
        if np.any(default_pos >= hi) or np.any(default_pos <= lo):
            raise ValueError("residual default_joint_pos (home) must be strictly inside the joint limits")
    if action_map == "centered" and (np.any(default_pos >= hi) or np.any(default_pos <= lo)):
        raise ValueError("centered action map needs every default_joint_pos strictly inside its limits")

    # Format v3 added a fixed-period gait clock (gait-table policies). A runtime that fell back to
    # the speed-proportional clock would feed the policy a phase it never saw, so v3 REQUIRES the
    # block and every mode is validated rather than defaulted.
    gait_clock = raw.get("gait_clock")
    if gait_clock is None:
        if fmt >= 3:
            raise ValueError("format v3 bundle has no gait_clock block")
        gait_clock = {"mode": "speed_proportional", "freq": float(raw["gait_clock_freq"])}
    mode = gait_clock.get("mode")
    if mode == "cadence_grid":
        # Format v5: the cadence follows the command. A v3/v4 runtime would run it at one rate.
        if fmt < 5:
            raise ValueError("gait_clock.mode 'cadence_grid' requires format_version >= 5")
        CadenceGrid.from_gait_clock(gait_clock)          # validates axes, shape, plausible rates
        thr = float(gait_clock.get("stand_threshold", 0.05))
        if thr < 0.0:
            raise ValueError("gait_clock.stand_threshold must be >= 0")
        gait_clock = {**gait_clock, "stand_threshold": thr}
    elif mode == "fixed_period":
        period = float(gait_clock.get("period", 0.0))
        if not (0.2 < period < 5.0):
            raise ValueError(f"gait_clock.period {period!r} s is implausible (expected 0.2-5 s)")
        thr = float(gait_clock.get("stand_threshold", 0.05))
        if thr < 0.0:
            raise ValueError("gait_clock.stand_threshold must be >= 0")
        gait_clock = {**gait_clock, "period": period, "stand_threshold": thr}
    elif mode == "speed_proportional":
        gait_clock = {**gait_clock, "freq": float(gait_clock.get("freq", raw["gait_clock_freq"]))}
    else:
        raise ValueError(f"unknown gait_clock.mode {mode!r}")

    command_limit, command_ranges = _parse_command_limit(raw.get("command_limit"),
                                                         raw.get("command_ranges", {}))

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
        command_ranges=command_ranges,
        tilt_fault_rad=float(raw.get("tilt_fault_rad", 0.784)),
        action_map=action_map,
        action_scale_rad=action_scale,
        action_scale_neg_rad=action_scale_neg,
        actuator=raw.get("actuator"),
        format_version=fmt,
        gait_clock=gait_clock,
        max_action_rate=(float(raw["max_action_rate"]) if raw.get("max_action_rate") is not None else None),
        command_limit=command_limit,
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
                gait_clock=cfg.gait_clock,
            )
        except Exception as exc:  # surface broken bundles rather than hiding them
            entry.update(valid=False, error=str(exc))
        out.append(entry)
    return out
