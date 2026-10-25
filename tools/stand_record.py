"""Record the robot HOLDING the policy's home pose (policy not stepping) while you tap or push it.

What it is for: the hardware traces show the body rocking/giving way with play the encoders do not see.
This measures the passive joint/ground compliance directly: with the servos holding position (P = the
bundle's firmware P, exactly as when the policy runs) a tap makes the body ring; the ring frequency,
its damping and how much of it the encoders see go into the sim's actuator model.

It uses the normal PolicyRunner arming (goal = present position, torque on, 2 s smoothstep ramp to the
bundle's home pose, then hold), but never starts the policy, so the pose is held by the servos alone.

STOP THE WEB APP FIRST (python main.py): only one process may own the serial bus.

    python tools/stand_record.py --bundle models/<bundle_dir> [--seconds 20] [--out data/stand_records]

Procedure (prompts guide you):
  1. Hold the robot upright on the floor, feet flat, roughly in its stand pose. Enter -> it ramps to the home pose.
  2. Let go once it holds. Enter -> recording starts (default 20 s).
  3. During the recording: a few single taps on the torso, letting it settle 2-3 s between them:
     front -> back, back -> front, then left and right sides. Small taps, like knocking on a door,
     NOT pushes that would make it step or fall. Catch it if it goes.
Output: data/stand_records/<UTC>_stand_<bundle>.npz with t, positions_deg (robot order, URDF deg), velocities,
gyro, linear_accel (fused), specific_force (raw accel if hardware.imu_raw_accel), projected_gravity, stamps, meta.
Torque goes off at the end (disarm) or on Ctrl+C / a tilt above 35 deg.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime, timezone

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from dotenv import load_dotenv  # noqa: E402

from robot.config import Settings  # noqa: E402
from robot.robot import Robot  # noqa: E402

TILT_ABORT_DEG = 35.0
PERIOD = 0.02


def _tilt_deg(g) -> float:
    gz = max(-1.0, min(1.0, -float(g[2]) / max(1e-9, math.sqrt(sum(float(v) ** 2 for v in g)))))
    return math.degrees(math.acos(gz))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bundle", required=True, help="exported bundle dir (the one you deploy), e.g. models/walk_v1")
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--out", default=os.path.join(ROOT, "data", "stand_records"))
    args = p.parse_args()

    load_dotenv()
    settings = Settings.load()
    robot = Robot(settings)
    robot.initialize()
    runner = robot.policy
    if runner is None:
        print("policy package unavailable (onnxruntime missing?)")
        return 1
    runner.load(os.path.abspath(args.bundle))
    if not robot.attach_policy_hook():
        print("could not attach the policy hook")
        return 1

    rows: dict[str, list] = {k: [] for k in ("t", "positions_deg", "velocities", "gyro", "linear_accel",
                                             "specific_force", "projected_gravity", "servo_stamp", "imu_stamp")}
    try:
        input("\n1) Hold the robot upright on the floor, feet flat. Press Enter to arm (servos ramp to the home pose over 2 s)...")
        runner.arm()
        t0 = time.monotonic()
        while runner.sm.state.value != "armed":
            if runner.sm.state.value == "fault" or time.monotonic() - t0 > 6.0:
                print(f"arming did not complete (state {runner.sm.state.value}); aborting")
                return 1
            time.sleep(0.05)
        input("2) It now holds the home pose on the servos alone. Let go of it, then press Enter to record "
              f"{args.seconds:.0f} s.\n   During the recording: single small TAPS on the torso (front, back, left, right), "
              "2-3 s apart. Not pushes.\n")
        print("   recording...")
        t_start = time.monotonic()
        nxt = t_start
        while time.monotonic() - t_start < args.seconds:
            rl = robot._bus_manager.get_rl_state()
            rows["t"].append(time.monotonic() - t_start)
            rows["positions_deg"].append(list(rl["positions"]))
            rows["velocities"].append(list(rl["velocities"]))
            rows["gyro"].append(list(rl["angular_vel"]))
            rows["linear_accel"].append(list(rl["linear_accel"]))
            sf = rl.get("specific_force")
            rows["specific_force"].append(list(sf) if sf is not None else [math.nan] * 3)
            rows["projected_gravity"].append(list(rl["projected_gravity"]))
            rows["servo_stamp"].append(rl["servo_stamp"] if rl["servo_stamp"] is not None else math.nan)
            rows["imu_stamp"].append(rl["imu_stamp"] if rl["imu_stamp"] is not None else math.nan)
            if _tilt_deg(rl["projected_gravity"]) > TILT_ABORT_DEG:
                print(f"   tilt above {TILT_ABORT_DEG:.0f} deg: stopping")
                break
            if runner.sm.state.value == "fault":
                print("   policy runner faulted: stopping")
                break
            nxt += PERIOD
            time.sleep(max(0.0, nxt - time.monotonic()))
        print(f"   recorded {len(rows['t'])} samples over {rows['t'][-1] if rows['t'] else 0:.1f} s")
    except KeyboardInterrupt:
        print("\n   interrupted")
    finally:
        try:
            runner.disarm()
        except Exception as exc:  # torque off must still be attempted
            print(f"disarm failed ({exc}); turning torques off directly")
            robot.disable_all_torques()
        robot.detach_policy_hook()

    if not rows["t"]:
        return 1
    os.makedirs(args.out, exist_ok=True)
    name = os.path.basename(os.path.normpath(args.bundle))
    path = os.path.join(args.out, f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_stand_{name}.npz")
    meta = {"bundle": name, "joint_names_robot_order": list(runner._mapper.robot_names),
            "home_pose_policy_rad": [float(v) for v in runner.cfg.default_joint_pos],
            "policy_joint_names": list(runner._mapper.policy_names),
            "imu_mode": settings.hardware.imu_mode, "imu_raw_accel": settings.hardware.imu_raw_accel,
            "period_s": PERIOD}
    np.savez(path, meta=json.dumps(meta), **{k: np.asarray(v, dtype=np.float64) for k, v in rows.items()})
    print(f"saved {path}")
    robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
