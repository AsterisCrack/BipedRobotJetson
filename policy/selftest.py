"""Dry-run the policy pipeline WITHOUT moving the robot.

Run this on the Jetson before arming for the first time, and again after any
recalibration. It reads real sensors, assembles the observation, runs inference,
and prints the joint targets that WOULD be commanded -- but never writes to the
bus and never enables torque.

    python3 -m policy.selftest --model walk_v1
    python3 -m policy.selftest --model walk_v1 --synthetic   # no hardware needed

What it is actually checking
---------------------------
The policy chain itself is already verified offline (export_onnx.py compares ONNX
against PyTorch; verify_onnx_bundle.py compares against the training repo's own
actor). What CANNOT be verified without hardware is the sensor mapping -- whether
the numbers coming off this robot land in the slots the policy expects. That is
what this tool exists for, and it is the step most likely to catch a silent
sign-flip or unit error before it becomes a fall.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from policy.deploy_config import load_bundle                      # noqa: E402
from policy.observation import (                                  # noqa: E402
    JointMapper, ObservationBuilder, specific_force_from_linear_accel,
)
from policy.safety import tilt_from_projected_gravity             # noqa: E402

OK, BAD, WARN = "  OK ", " FAIL", " WARN"


def _synthetic_state(joint_names):
    """A perfectly upright, motionless robot -- the reference case."""
    return {
        "positions": [0.0] * len(joint_names),
        "velocities": [0] * len(joint_names),
        "velocities_deg_s": [0.0] * len(joint_names),
        "linear_accel": (0.0, 0.0, 0.0),   # BNO055 LIA: gravity already removed
        "angular_vel": (0.0, 0.0, 0.0),
        "projected_gravity": (0.0, 0.0, -1.0),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="bundle name under models/")
    ap.add_argument("--synthetic", action="store_true",
                    help="use a fabricated upright state instead of real sensors")
    # 5 steps is enough for the EMA filter alone (effective alpha ~0.87/step means it
    # tracks a FIXED target within ~5 steps), but the target itself keeps shifting as
    # joint_pos and previous_actions evolve each step, so the whole closed loop needs
    # more like 30 to reach a self-consistent standing pose.
    ap.add_argument("--steps", type=int, default=30)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg = load_bundle(os.path.join(root, "models", args.model))
    print(f"bundle : {cfg.name}")
    print(f"policy : obs={cfg.obs_dim} act={cfg.action_dim} "
          f"hist={cfg.history_size}x{cfg.obs_proprio_dim} @ {cfg.control_hz:.0f} Hz")
    if cfg.info:
        print(f"source : {cfg.info.get('run','?')} step {cfg.info.get('step','?')}")
    gc = cfg.gait_clock
    print("clock  : " + (f"fixed period {gc['period']:.3f} s, holds below {gc['stand_threshold']:.2f} m/s "
                         f"(table {gc.get('style_digest', '?')})" if gc["mode"] == "fixed_period"
                         else f"speed-proportional {gc['freq']:.2f} Hz per m/s"))
    print(f"ranges : {cfg.command_ranges}")
    print()

    # -- sensors -------------------------------------------------------------
    robot = None
    if args.synthetic:
        robot_names = list(cfg.joint_names)
        defaults = {n: 0.0 for n in robot_names}
        rl = _synthetic_state(robot_names)
        print("MODE   : synthetic (upright, motionless)\n")
    else:
        from robot.config import Settings
        from robot.robot import Robot
        robot = Robot(Settings.load())
        robot.initialize()
        bm = robot._bus_manager
        if not bm.has_velocity:
            print("BIPED_FAST_MODE=1 omits the speed register, so the policy's joint_vel "
                  "channel would read zero. Use BIPED_FAST_MODE=2 (pos+speed) or 0.")
            return 1
        robot_names = [s.joint_name for s in bm._servos]
        defaults = {s.joint_name: float(getattr(s, "default_position_deg", 0.0))
                    for s in bm._servos}
        import time
        time.sleep(0.5)  # let the bus thread populate a real reading
        rl = bm.get_rl_state()
        print("MODE   : live sensors (TORQUE STAYS OFF)\n")

    mapper = JointMapper(cfg.joint_names, robot_names, defaults)
    ob = ObservationBuilder(cfg)
    ob.validate_layout()

    failures = []

    # -- sensor sanity -------------------------------------------------------
    print("=== sensors ===")
    pg = np.asarray(rl["projected_gravity"], float)
    n = float(np.linalg.norm(pg))
    tag = OK if abs(n - 1.0) < 0.05 else BAD
    if tag is BAD:
        failures.append("projected_gravity is not a unit vector")
    print(f"{tag} projected_gravity  = [{pg[0]:+.3f} {pg[1]:+.3f} {pg[2]:+.3f}]  |v|={n:.3f}")

    tilt = tilt_from_projected_gravity(pg)
    tag = OK if tilt < cfg.tilt_fault_rad else WARN
    print(f"{tag} tilt               = {math.degrees(tilt):.1f} deg "
          f"(fault at {math.degrees(cfg.tilt_fault_rad):.0f})")

    sf = specific_force_from_linear_accel(rl["linear_accel"], pg)
    mag = float(np.linalg.norm(sf))
    tag = OK if 8.5 < mag < 11.0 else BAD
    if tag is BAD:
        failures.append(
            f"specific force magnitude {mag:.2f} is not ~9.81 -- the IMU accel "
            f"convention or axis mapping is wrong")
    print(f"{tag} specific force     = [{sf[0]:+.2f} {sf[1]:+.2f} {sf[2]:+.2f}]  "
          f"|a|={mag:.2f} m/s2 (want ~9.81)")

    gyro = np.asarray(rl["angular_vel"], float)
    tag = OK if float(np.abs(gyro).max()) < 10.0 else WARN
    print(f"{tag} gyro               = [{gyro[0]:+.3f} {gyro[1]:+.3f} {gyro[2]:+.3f}] rad/s")

    # -- joint mapping -------------------------------------------------------
    print("\n=== joint mapping (robot order -> policy order) ===")
    pos_rad = mapper.robot_deg_to_policy_rad(rl["positions"])
    vel_rad = mapper.robot_rate_to_policy_rad(rl["velocities_deg_s"])
    # Backdrive a joint by hand while this runs: the matching slot should move, with a
    # sign that matches the direction. That is the only check for the sign-magnitude
    # decode and direction_sign, both of which fail silently.
    print(f"joint_vel |max| = {float(np.abs(vel_rad).max()):.3f} rad/s "
          f"(expect ~0 at rest; backdrive a joint to see it move)")
    print(f"{'policy slot':<24}{'URDF deg':>10}{'logical deg':>13}{'limit deg':>16}")
    for i, name in enumerate(cfg.joint_names):
        urdf = rl["positions"][robot_names.index(name)]
        log_deg = math.degrees(pos_rad[i])
        lo, hi = math.degrees(cfg.joint_limits_min[i]), math.degrees(cfg.joint_limits_max[i])
        flag = "" if lo - 1 <= log_deg <= hi + 1 else "  <-- OUTSIDE POLICY LIMIT"
        if flag:
            failures.append(f"{name} at {log_deg:.1f} deg is outside [{lo:.0f}, {hi:.0f}]")
        print(f"{name:<24}{urdf:>10.2f}{log_deg:>13.2f}   [{lo:>5.0f},{hi:>5.0f}]{flag}")

    # -- inference -----------------------------------------------------------
    print("\n=== inference ===")
    import onnxruntime as ort
    sess = ort.InferenceSession(cfg.onnx_path, providers=["CPUExecutionProvider"])

    frame = ob.build_frame(sf, gyro, pg, [0, 0, 0], pos_rad, vel_rad,
                           advance_phase=False)
    ob.reset(standing_frame=frame)

    import time
    filt = pos_rad.copy()
    t_total = 0.0
    for _ in range(args.steps):
        obs = ob.push(frame)
        t0 = time.monotonic()
        action = sess.run(["action"], {"obs": cfg.normalize_obs(obs)[None, :]})[0][0]
        t_total += time.monotonic() - t0
        tgt = cfg.action_to_joint_targets(action.astype(float))
        for _ in range(cfg.action_filter_applications):
            filt = cfg.action_filter_alpha * tgt + (1 - cfg.action_filter_alpha) * filt
        ob.set_previous_action(action)
        # Rebuild the frame for the next iteration. Torque is off, so the real joint
        # positions genuinely do not move -- but `previous_actions` MUST reflect what
        # we just chose, and joint_pos is fed back as the converging EMA target (`filt`)
        # rather than the static measured position, which is what makes this a preview
        # of "where the policy is trying to drive the robot" rather than a repeat of
        # one stale frame. Without this rebuild, previous_actions stays at zero forever
        # and the printed targets are the policy's response to a frame that never
        # changes -- not a converged prediction.
        frame = ob.build_frame(sf, gyro, pg, [0, 0, 0], filt, np.zeros(cfg.action_dim),
                               advance_phase=False)

    ms = t_total / args.steps * 1000
    budget = cfg.step_dt * 1000
    tag = OK if ms < budget * 0.5 else WARN
    print(f"{tag} inference          = {ms:.2f} ms/step (budget {budget:.0f} ms)")

    tag = OK if np.all(np.abs(action) <= 1.0) else BAD
    if tag is BAD:
        failures.append("action escaped [-1, 1]")
    print(f"{tag} action range       = [{action.min():+.3f}, {action.max():+.3f}]")

    print("\n=== targets that WOULD be commanded (nothing was sent) ===")
    print(f"{'joint':<24}{'target deg':>12}{'delta from now':>16}")
    for i, name in enumerate(cfg.joint_names):
        t_deg = math.degrees(filt[i])
        d = t_deg - math.degrees(pos_rad[i])
        mark = "  <-- large" if abs(d) > 30 else ""
        print(f"{name:<24}{t_deg:>12.2f}{d:>16.2f}{mark}")

    if robot is not None:
        robot.shutdown()

    print()
    if failures:
        print("SELFTEST FAILED:")
        for f in failures:
            print(f"  - {f}")
        print("\nDo NOT arm until these are resolved.")
        return 1

    print("SELFTEST PASSED -- sensor mapping and inference look sane.")
    print("Next: suspend the robot, then arm. Keep the e-stop reachable.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
