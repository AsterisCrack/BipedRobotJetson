#!/usr/bin/env python3
"""
tools/return_delay.py — Read and optionally zero the RETURN_DELAY register on all servos.

RETURN_DELAY (register 0x07, EEPROM) controls how long a servo waits before
sending its status response.  Each unit = 2 µs; default factory value is often
250 (= 500 µs per servo).  With 12 servos responding sequentially, a non-zero
value adds up to ~N x delay µs of dead time per SYNC_READ cycle.

Usage:
    sudo venv/bin/python tools/return_delay.py              # read-only
    sudo venv/bin/python tools/return_delay.py --set-zero   # read then write 0 to all
    sudo venv/bin/python tools/return_delay.py --servos 2,3,4 --set-zero
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any

import yaml

logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))

from hardware.config import HardwareConfig, ServoConfig
from hardware.serial_bus import SerialBus, SerialBusError
from hardware.st3215.registers import Reg
from hardware.st3215.servo import ST3215


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


def main() -> None:
    parser = argparse.ArgumentParser(description="Read/set RETURN_DELAY on ST3215 servos")
    parser.add_argument("--servos", help="Comma-separated servo IDs (default: all from config)")
    parser.add_argument("--set-zero", action="store_true",
                        help="Write 0 to RETURN_DELAY on each servo (EEPROM write, persists across power cycles)")
    parser.add_argument("--echo", action="store_true",
                        help="Enable half-duplex echo drain (needed for some USB-SCS adapters)")
    parser.add_argument("--port", help="Override UART port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, help="Override baud rate")
    args = parser.parse_args()

    hw_path    = _REPO_ROOT / "config" / "hardware.yaml"
    robot_path = _REPO_ROOT / "config" / "robot.yaml"
    hw, servo_cfgs = _load_configs(hw_path, robot_path)

    port = args.port or hw.uart_port
    baud = args.baud or hw.baud_rate

    if args.servos:
        target_ids = {int(s.strip()) for s in args.servos.split(",")}
    else:
        target_ids = {cfg.servo_id for cfg in servo_cfgs}

    cfg_map = {cfg.servo_id: cfg for cfg in servo_cfgs}
    missing = target_ids - set(cfg_map)
    if missing:
        # Create minimal configs for IDs not in robot.yaml
        for sid in missing:
            cfg_map[sid] = ServoConfig(
                servo_id=sid, joint_name=f"servo_{sid}",
                zero_offset_steps=2048, direction_sign=1,
            )

    ordered_ids = sorted(target_ids)

    print(f"\n  RETURN_DELAY check — {port} @ {baud:,} bps")
    print(f"  Servos: {ordered_ids}")
    print(f"  Mode:   {'read + write 0' if args.set_zero else 'read-only'}")
    print()

    bus = SerialBus(port, baud, timeout=0.05, expect_echo=args.echo)
    bus.open()

    try:
        read_results: dict[int, int] = {}
        for sid in ordered_ids:
            servo = ST3215(cfg_map[sid], bus)
            try:
                data = servo.read_register(Reg.RETURN_DELAY, 1)
                val = data[0]
                read_results[sid] = val
                delay_us = val * 2
                print(f"  Servo {sid:3d}: RETURN_DELAY = {val:3d}  ({delay_us} µs)")
            except SerialBusError as exc:
                print(f"  Servo {sid:3d}: READ FAILED — {exc}")

        if args.set_zero:
            non_zero = {sid: v for sid, v in read_results.items() if v != 0}
            if not non_zero:
                print("\n  All servos already at 0 — no writes needed.")
            else:
                total_saved_us = sum(v * 2 for v in non_zero.values())
                print(f"\n  Writing 0 to {len(non_zero)} servo(s)... "
                      f"(saves ~{total_saved_us} µs = {total_saved_us/1000:.2f} ms per SYNC_READ)")

                write_ok = []
                write_fail = []
                for sid in sorted(non_zero):
                    servo = ST3215(cfg_map[sid], bus)
                    try:
                        servo.write_register(Reg.LOCK_FLAG, bytes([0]))   # unlock EEPROM
                        time.sleep(0.005)
                        servo.write_register(Reg.RETURN_DELAY, bytes([0]))
                        time.sleep(0.005)
                        servo.write_register(Reg.LOCK_FLAG, bytes([1]))   # re-lock EEPROM
                        time.sleep(0.005)

                        # Verify readback
                        data = servo.read_register(Reg.RETURN_DELAY, 1)
                        if data[0] == 0:
                            print(f"  Servo {sid:3d}: ✓  written 0, verified")
                            write_ok.append(sid)
                        else:
                            print(f"  Servo {sid:3d}: ✗  write did not take (read back {data[0]})")
                            write_fail.append(sid)
                    except SerialBusError as exc:
                        print(f"  Servo {sid:3d}: WRITE FAILED — {exc}")
                        write_fail.append(sid)
                        # Try to re-lock on failure path
                        try:
                            servo.write_register(Reg.LOCK_FLAG, bytes([1]))
                        except Exception:
                            pass

                print()
                if write_ok:
                    print(f"  Success: {write_ok}")
                if write_fail:
                    print(f"  Failed:  {write_fail}")
                    print("  Note: EEPROM writes may require torque to be disabled first.")
                    print("        If failure persists, the main server must be stopped.")

        # Summary
        if read_results:
            total_delay_us = sum(v * 2 for v in read_results.values())
            max_val = max(read_results.values())
            print(f"\n  Total RETURN_DELAY overhead: {total_delay_us} µs "
                  f"({total_delay_us/1000:.2f} ms) across {len(read_results)} servos")
            if max_val == 0:
                print("  All servos at 0 ✓ — no extra RX gap overhead")
            else:
                print(f"  Setting all to 0 would recover ~{total_delay_us/1000:.2f} ms per 50 Hz cycle")
    finally:
        bus.close()
        print()


if __name__ == "__main__":
    main()
