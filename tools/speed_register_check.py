#!/usr/bin/env python3
"""
tools/speed_register_check.py — is the servo's CURRENT_SPEED register usable as joint_vel?

Logs, per joint per 50 Hz cycle, the speed register against a finite difference of
position, and reports the four things that decide it:

  sign       decoded velocity must track the slope of position. Catches both a wrong
             sign-magnitude decode and a missing direction_sign -- neither of which
             produces an obviously wrong number on its own.
  scale      regression slope of register vs finite difference. Should be ~1.00 if the
             register really is steps/s.
  lag        cross-correlation peak, in 20 ms control steps. Feeds the sim model.
  dropout    fraction of moving samples where the register reads exactly 0. High here
             means the servo only reports during its internal slew and the channel is
             too bursty to use at 50 Hz.

Modes:
    backdrive  (default) TORQUE OFF. You move the joints by hand. Completely safe,
               and enough on its own to settle sign and scale.
    sweep      TORQUE ON. Drives a sinusoid about the standing pose. Suspend the robot
               first. Needed for lag and dropout, which depend on the servo being
               driven the way the policy drives it.

Usage (from project root):
    venv/bin/python tools/speed_register_check.py --joints l_knee_joint
    venv/bin/python tools/speed_register_check.py --joints l_knee_joint --mode sweep
    venv/bin/python tools/speed_register_check.py --joints l_knee_joint --csv /tmp/speed.csv
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))

from robot.config import Settings
from robot.robot import Robot

DT = 0.02
MOVING_DEG_S = 2.0   # below this the finite difference is mostly quantization noise


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joints", required=True,
                    help="comma-separated joint names to log (and, in sweep mode, to move)")
    ap.add_argument("--mode", choices=["backdrive", "sweep"], default="backdrive")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--amplitude-deg", type=float, default=8.0, help="sweep mode only")
    ap.add_argument("--period-s", type=float, default=2.0, help="sweep mode only")
    ap.add_argument("--csv", type=Path, default=None)
    return ap.parse_args()


def _fit(reg: list[float], fd: list[float]) -> dict:
    """Compare the two velocity estimates over the samples where the joint is moving."""
    pairs = [(r, f) for r, f in zip(reg, fd) if abs(f) > MOVING_DEG_S]
    if not pairs:
        return {"n": 0}
    agree = sum(1 for r, f in pairs if r * f > 0) / len(pairs)
    num = sum(r * f for r, f in pairs)
    den = sum(f * f for f in (p[1] for p in pairs))
    dropout = sum(1 for r, _ in pairs if r == 0.0) / len(pairs)
    return {
        "n": len(pairs),
        "sign_agreement": agree,
        "slope": num / den if den else float("nan"),
        "dropout": dropout,
    }


def _best_lag(reg: list[float], fd: list[float], max_lag: int = 5) -> tuple[int, float]:
    """Shift the register series against the finite difference; return the best shift.

    Positive = the register lags the finite difference by that many control steps.
    """
    best = (0, -2.0)
    for lag in range(-max_lag, max_lag + 1):
        a = reg[lag:] if lag >= 0 else reg[:lag]
        b = fd[:len(a)] if lag >= 0 else fd[-len(a):]
        if len(a) < 10:
            continue
        na = math.sqrt(sum(x * x for x in a))
        nb = math.sqrt(sum(x * x for x in b))
        if na == 0 or nb == 0:
            continue
        corr = sum(x * y for x, y in zip(a, b)) / (na * nb)
        if corr > best[1]:
            best = (lag, corr)
    return best


def main() -> int:
    args = _parse_args()
    names = [n.strip() for n in args.joints.split(",") if n.strip()]

    robot = Robot(Settings.load())
    robot.initialize()
    bm = robot._bus_manager
    if not bm.has_velocity:
        print("BIPED_FAST_MODE=1 — the speed register is not even read. Use 2 or 0.")
        return 1

    order = [s.joint_name for s in bm._servos]
    unknown = [n for n in names if n not in order]
    if unknown:
        print(f"unknown joints: {unknown}\navailable: {order}")
        return 1
    idx = [order.index(n) for n in names]
    defaults = {s.joint_name: float(getattr(s, "default_position_deg", 0.0)) for s in bm._servos}

    if args.mode == "sweep":
        print(f"\nSWEEP MODE WILL MOVE THE ROBOT: {names}, "
              f"+-{args.amplitude_deg:.0f} deg about the standing pose.")
        print("Suspend the robot and keep clear. Ctrl-C aborts.")
        input("Press Enter to enable torque and start: ")
        robot.enable_all_torques()
    else:
        robot.disable_all_torques()
        print(f"\nTORQUE OFF. Move {names} back and forth by hand for {args.seconds:.0f}s.")
        input("Press Enter to start logging: ")

    time.sleep(0.3)
    rows: list[tuple] = []
    prev_pos: list[float] | None = None
    t0 = time.monotonic()
    try:
        while (t := time.monotonic() - t0) < args.seconds:
            rl = bm.get_rl_state()
            pos = [rl["positions"][i] for i in idx]
            reg = [rl["velocities_deg_s"][i] for i in idx]
            raw = [rl["velocities"][i] for i in idx]
            fd = ([(p - q) / DT for p, q in zip(pos, prev_pos)] if prev_pos
                  else [0.0] * len(idx))
            prev_pos = pos
            rows.append((t, pos, reg, fd, raw))

            if args.mode == "sweep":
                off = args.amplitude_deg * math.sin(2 * math.pi * t / args.period_s)
                robot.sync_write_positions(
                    {n: defaults[n] + off for n in names}, speed=0, raw=True)
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if args.mode == "sweep":
            robot.sync_write_positions({n: defaults[n] for n in names}, speed=200, raw=True)
            time.sleep(0.5)
        robot.disable_all_torques()

    rows = rows[1:]   # first finite difference is meaningless
    if not rows:
        print("no samples")
        return 1

    if args.csv:
        with args.csv.open("w") as f:
            f.write("t," + ",".join(
                f"{n}_pos_deg,{n}_reg_deg_s,{n}_fd_deg_s,{n}_raw_counts" for n in names) + "\n")
            for t, pos, reg, fd, raw in rows:
                cells = []
                for k in range(len(names)):
                    cells += [f"{pos[k]:.3f}", f"{reg[k]:.3f}", f"{fd[k]:.3f}", str(raw[k])]
                f.write(f"{t:.4f}," + ",".join(cells) + "\n")
        print(f"\nwrote {len(rows)} samples to {args.csv}")

    print(f"\n{'joint':<22}{'n':>6}{'sign':>8}{'slope':>8}{'lag':>6}{'corr':>7}{'drop':>7}")
    verdict_ok = True
    for k, name in enumerate(names):
        reg = [r[2][k] for r in rows]
        fd = [r[3][k] for r in rows]
        st = _fit(reg, fd)
        if st["n"] == 0:
            print(f"{name:<22}{0:>6}   never moved — nothing to compare")
            verdict_ok = False
            continue
        lag, corr = _best_lag(reg, fd)
        print(f"{name:<22}{st['n']:>6}{st['sign_agreement']:>8.2f}{st['slope']:>8.2f}"
              f"{lag:>6d}{corr:>7.2f}{st['dropout']:>7.2f}")
        if st["sign_agreement"] < 0.9 or not 0.7 < st["slope"] < 1.4:
            verdict_ok = False

    print("\nwant: sign ~1.00, slope ~1.00, lag 0-1 steps, drop ~0.00")
    print("  sign  < 0.9  -> bit-15 decode or direction_sign is wrong for that joint")
    print("  slope far from 1 -> the register is not steps/s after all")
    print("  drop  high   -> too bursty at 50 Hz; fall back to quantized finite differencing")
    print(f"\nverdict: {'register looks usable' if verdict_ok else 'NEEDS INVESTIGATION'}")
    robot.shutdown()
    return 0 if verdict_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
