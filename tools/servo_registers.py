#!/usr/bin/env python3
"""
tools/servo_registers.py -- audit and clear the STS3215 acceleration cap.

WHY THIS EXISTS
---------------
The STS3215 ships with an acceleration limit active. Two registers are involved:

    0x29 (41)  ACCELERATION      documented, unit 100 step/s^2, 0 = unlimited
    0x55 (85)  MAX_ACCELERATION  UNDOCUMENTED, factory value 50

0x55 gates 0x29, and writing 0x55 alone has no effect -- 0x29 must be written
afterwards for the new cap to take. Both live in RAM, so both revert on every
power cycle. (Source: Rhoban BAM PR #20, which hit this while identifying a 12 V
STS3215 and measured the out-of-the-box ceiling at roughly 10-28 rad/s^2.)

Why that matters here: a bang-bang move of distance d in time T needs
a = 4d/T^2. At 50 Hz (T = 20 ms) a 0.1 rad step needs ~1000 rad/s^2. Under a
28 rad/s^2 cap the servo covers 0.0028 rad -- 0.16 deg -- per control step, so
it cannot follow the policy at all, and the phase lag that creates is exactly
how sluggish tracking turns into oscillation.

Usage (from project root, with the main server stopped):
    venv/bin/python tools/servo_registers.py                  # read-only audit
    venv/bin/python tools/servo_registers.py --clear          # write 0x55=0 then 0x29=0
    venv/bin/python tools/servo_registers.py --step-test 10   # prove it: +10 deg step
    venv/bin/python tools/servo_registers.py --servos 2,3 --clear
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any

import yaml

logging.basicConfig(level=logging.WARNING,
                    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))

from hardware.config import HardwareConfig, ServoConfig       # noqa: E402
from hardware.serial_bus import SerialBus, SerialBusError     # noqa: E402
from hardware.st3215.registers import Reg                     # noqa: E402
from hardware.st3215.servo import ST3215, STEPS_PER_DEG       # noqa: E402

# 0x29 is documented as "100 step/s^2" and the encoder is 4096 counts/rev, so one
# unit is 100 * 2*pi/4096 rad/s^2. 0x55 has no published unit, so it is shown raw.
ACC_UNIT_RAD_S2 = 100.0 * (2.0 * math.pi / 4096.0)


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open() as f:
        return yaml.safe_load(f) or {}


def _load_configs(hw_path: Path, robot_path: Path) -> tuple[HardwareConfig, list[ServoConfig]]:
    hw_data = _read_yaml(hw_path)
    robot_data = _read_yaml(robot_path)
    hw = HardwareConfig(**hw_data) if hw_data else HardwareConfig()
    servos = [ServoConfig(**s) for s in robot_data.get("servos", [])]
    return hw, servos


def _audit(servo: ST3215) -> tuple[int, int]:
    """Return (accel_0x29, max_accel_0x55). Raises SerialBusError on failure."""
    accel = servo.read_register(Reg.ACCELERATION, 1)[0]
    max_accel = servo.read_register(Reg.MAX_ACCELERATION, 1)[0]
    return accel, max_accel


def _torque_audit(servo: ST3215) -> tuple[int, int, int]:
    """Return (MAX_TORQUE 0x10, TORQUE_LIMIT 0x30, PROT_CURRENT 0x1C), raw register values.

    MAX_TORQUE (EEPROM) and TORQUE_LIMIT (RAM) are both 0-1000 = 0-100% of the duty the
    firmware may output. The training sim's servo model (BAM) assumes 100%; anything less
    is a real torque ceiling the sim doesn't know about, and it belongs in config.yaml as
    env_config.actuator.torque_scale.
    """
    u16 = lambda b: b[0] | (b[1] << 8)
    max_torque = u16(servo.read_register(Reg.MAX_TORQUE_L, 2))
    torque_limit = u16(servo.read_register(Reg.TORQUE_LIMIT_L, 2))
    prot_current = u16(servo.read_register(Reg.PROT_CURRENT_L, 2))
    return max_torque, torque_limit, prot_current


def _clear(servo: ST3215) -> tuple[int, int]:
    """Write 0x55 = 0 THEN 0x29 = 0 (order matters), return the verified readback."""
    servo.write_register(Reg.MAX_ACCELERATION, bytes([0]))
    time.sleep(0.005)
    servo.write_register(Reg.ACCELERATION, bytes([0]))
    time.sleep(0.005)
    return _audit(servo)


def _step_test(servo: ST3215, amplitude_deg: float, settle_s: float = 1.0) -> None:
    """Command a step and sample position as fast as the bus allows.

    Reports time to 90% of the commanded step. Under the factory cap this runs to
    hundreds of milliseconds; uncapped it should land within a control period or two.
    """
    servo.enable_torque()
    time.sleep(0.2)
    start = int.from_bytes(servo.read_register(Reg.CURRENT_POS_L, 2), "little")
    target = int(round(start + amplitude_deg * STEPS_PER_DEG))

    servo.write_register(Reg.TARGET_POS_L, target.to_bytes(2, "little"))
    t0 = time.perf_counter()
    samples: list[tuple[float, int]] = []
    while (t := time.perf_counter() - t0) < settle_s:
        raw = servo.read_register(Reg.CURRENT_POS_L, 2)
        samples.append((t, int.from_bytes(raw, "little")))

    span = target - start
    if span == 0:
        print("    step of zero -- nothing to measure")
        return
    t90 = next((t for t, p in samples if abs(p - start) >= 0.9 * abs(span)), None)
    final = samples[-1][1] if samples else start
    print(f"    sampled {len(samples)} points at ~{len(samples) / settle_s:.0f} Hz")
    print(f"    start={start} target={target} final={final} "
          f"({(final - start) / STEPS_PER_DEG:+.2f} deg of {amplitude_deg:+.2f} commanded)")
    if t90 is None:
        print("    time to 90%: NOT REACHED in the window  <-- cap is almost certainly active")
    else:
        print(f"    time to 90%: {t90 * 1000:.1f} ms "
              f"({t90 / 0.02:.1f} control periods at 50 Hz)")

    servo.write_register(Reg.TARGET_POS_L, start.to_bytes(2, "little"))
    time.sleep(0.4)
    servo.disable_torque()


def main() -> int:
    ap = argparse.ArgumentParser(description="Audit/clear the STS3215 acceleration cap")
    ap.add_argument("--servos", help="Comma-separated servo IDs (default: all from robot.yaml)")
    ap.add_argument("--clear", action="store_true",
                    help="Write 0 to 0x55 then 0x29 (RAM only -- reverts on power cycle)")
    ap.add_argument("--step-test", type=float, metavar="DEG",
                    help="After auditing, TORQUE ON and command a step of DEG degrees")
    ap.add_argument("--echo", action="store_true", help="Enable half-duplex echo drain")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int)
    args = ap.parse_args()

    hw, servo_cfgs = _load_configs(_REPO_ROOT / "config" / "hardware.yaml",
                                   _REPO_ROOT / "config" / "robot.yaml")
    port = args.port or hw.uart_port
    baud = args.baud or hw.baud_rate

    cfg_map = {c.servo_id: c for c in servo_cfgs}
    if args.servos:
        ids = sorted(int(s.strip()) for s in args.servos.split(","))
    else:
        ids = sorted(cfg_map)
    for sid in ids:
        cfg_map.setdefault(sid, ServoConfig(servo_id=sid, joint_name=f"servo_{sid}",
                                            zero_offset_steps=2048, direction_sign=1))

    print(f"\n  Acceleration cap audit -- {port} @ {baud:,} bps")
    print(f"  Servos: {ids}")
    print(f"  Mode:   {'audit + clear' if args.clear else 'read-only'}\n")
    print(f"  {'servo':<8}{'joint':<22}{'0x29 accel':>12}{'rad/s^2':>10}{'0x55 cap':>10}")

    bus = SerialBus(port, baud, timeout=0.05, expect_echo=args.echo)
    bus.open()
    capped: list[int] = []
    try:
        for sid in ids:
            servo = ST3215(cfg_map[sid], bus)
            try:
                accel, max_accel = _audit(servo)
            except SerialBusError as exc:
                print(f"  {sid:<8}{cfg_map[sid].joint_name:<22}READ FAILED -- {exc}")
                continue
            limited = "" if (accel == 0 and max_accel == 0) else "  <-- LIMITED"
            if limited:
                capped.append(sid)
            print(f"  {sid:<8}{cfg_map[sid].joint_name:<22}{accel:>12}"
                  f"{accel * ACC_UNIT_RAD_S2:>10.1f}{max_accel:>10}{limited}")

        if not capped:
            print("\n  All servos report 0/0 -- no acceleration cap active.")
        else:
            print(f"\n  {len(capped)} servo(s) have a non-zero cap: {capped}")
            print("  At 50 Hz a 0.1 rad step needs ~1000 rad/s^2; the factory cap is ~10-28.")

        # Torque ceiling: what fraction of full duty the firmware is allowed to output.
        print(f"\n  {'servo':<8}{'joint':<22}{'MAX_TORQUE':>11}{'TORQUE_LIM':>11}{'PROT_mA':>9}{'scale':>7}")
        scales: list[float] = []
        for sid in ids:
            try:
                mt, tl, pc = _torque_audit(ST3215(cfg_map[sid], bus))
            except SerialBusError as exc:
                print(f"  {sid:<8}{cfg_map[sid].joint_name:<22}READ FAILED -- {exc}")
                continue
            scale = min(mt, tl) / 1000.0
            scales.append(scale)
            flag = "" if scale >= 0.999 else "  <-- below 100%"
            print(f"  {sid:<8}{cfg_map[sid].joint_name:<22}{mt:>11}{tl:>11}{pc * 6.5:>9.0f}{scale:>7.2f}{flag}")
        if scales:
            lo = min(scales)
            if lo >= 0.999:
                print("\n  Full duty available on every servo -- torque_scale: 1.0 matches.")
            else:
                print(f"\n  At least one servo is capped at {lo:.0%} of full duty. Set "
                      f"env_config.actuator.torque_scale: {lo:.2f} in BipedRobot/config/config.yaml, "
                      f"or raise the limit here.")

        if args.clear and capped:
            print("\n  Clearing (0x55 = 0, then 0x29 = 0)...")
            for sid in capped:
                servo = ST3215(cfg_map[sid], bus)
                try:
                    accel, max_accel = _clear(servo)
                    ok = "OK" if (accel == 0 and max_accel == 0) else "DID NOT TAKE"
                    print(f"    servo {sid:3d}: 0x29={accel} 0x55={max_accel}  {ok}")
                except SerialBusError as exc:
                    print(f"    servo {sid:3d}: WRITE FAILED -- {exc}")
            print("\n  These are RAM registers: they revert on every power cycle.")
            print("  Robot.initialize() re-applies them at startup; this tool is for verifying.")

        if args.step_test is not None:
            for sid in ids:
                print(f"\n  Step test, servo {sid} ({cfg_map[sid].joint_name}), "
                      f"{args.step_test:+.1f} deg -- TORQUE WILL ENGAGE")
                try:
                    _step_test(ST3215(cfg_map[sid], bus), args.step_test)
                except SerialBusError as exc:
                    print(f"    FAILED -- {exc}")
    finally:
        bus.close()
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
