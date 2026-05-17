#!/usr/bin/env python3
"""
tools/bus_profiler.py — ServoBusManager harness for bus timing profiling.

Runs the real ServoBusManager (same code path as production: SCHED_FIFO,
dedicated IMU reader thread, SYNC_READ/WRITE) and collects per-phase timing
via built-in profiling hooks.  Reports mean/std/min/max/P5/P95/P99 per phase
plus an ASCII cycle-time histogram.

Usage (from project root):
    sudo venv/bin/python tools/bus_profiler.py               # defaults from config YAML
    sudo venv/bin/python tools/bus_profiler.py --cycles 1000 --write
    sudo venv/bin/python tools/bus_profiler.py --servos 2,3,4,5,6,7,8,9,10,11,12,13
    sudo venv/bin/python tools/bus_profiler.py --echo        # half-duplex echo drain

Run with sudo (or after: sudo setcap cap_sys_nice+eip $(readlink -f venv/bin/python3))
so the bus thread can apply SCHED_FIFO for accurate P99 measurements.
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

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Path: allow running from any directory inside the project
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))

from hardware.config import HardwareConfig, ServoConfig
from hardware.imu.bno055 import BNO055
from hardware.serial_bus import SerialBus
from hardware.servo_bus_manager import ServoBusManager
from hardware.st3215.protocol import encode_sync_read, encode_sync_write, pack_u16, steps_to_bytes
from hardware.st3215.registers import Reg
from hardware.st3215.servo import ST3215


# ---------------------------------------------------------------------------
# YAML loading (inlined — avoids robot/ dependency)
# ---------------------------------------------------------------------------

def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open() as f:
        try:
            return yaml.safe_load(f) or {}
        except yaml.YAMLError as exc:
            raise SystemExit(f"Failed to parse {path}: {exc}") from exc


def _load_configs(hw_path: Path, robot_path: Path) -> tuple[HardwareConfig, list[ServoConfig]]:
    hw_data = _read_yaml(hw_path)
    robot_data = _read_yaml(robot_path)
    hw = HardwareConfig(**hw_data) if hw_data else HardwareConfig()
    servos = [ServoConfig(**s) for s in robot_data.get("servos", [])]
    return hw, servos


# ---------------------------------------------------------------------------
# Statistics — data assumed to be in µs
# ---------------------------------------------------------------------------

def _percentile(sorted_data: list[float], pct: float) -> float:
    if not sorted_data:
        return 0.0
    k = (len(sorted_data) - 1) * pct / 100.0
    lo = int(k)
    hi = lo + 1
    if hi >= len(sorted_data):
        return sorted_data[-1]
    return sorted_data[lo] * (1.0 - (k - lo)) + sorted_data[hi] * (k - lo)


def _stats(values_us: list[float]) -> dict[str, float]:
    """Compute stats from a list of values in µs."""
    if not values_us:
        return {k: 0.0 for k in ("mean", "std", "min", "max", "p5", "p95", "p99")}
    s = sorted(values_us)
    n = len(s)
    mean = sum(s) / n
    std = math.sqrt(sum((x - mean) ** 2 for x in s) / n)
    return {
        "mean": mean, "std": std,
        "min": s[0], "max": s[-1],
        "p5": _percentile(s, 5),
        "p95": _percentile(s, 95),
        "p99": _percentile(s, 99),
    }


def _print_stats(label: str, st: dict[str, float], unit: str) -> None:
    """Print a stats row. unit='us' → display in µs, unit='ms' → display in ms."""
    scale = 1.0 if unit == "us" else 1e-3   # µs→µs  or  µs→ms
    keys = ("mean", "std", "min", "max", "p5", "p95", "p99")
    v = {k: st[k] * scale for k in keys}
    print(f"\n  {label}  [{unit}]")
    print(f"    {'mean':>8} {'std':>8} {'min':>8} {'max':>8} {'p5':>8} {'p95':>8} {'p99':>8}")
    print(
        f"    {v['mean']:8.3f} {v['std']:8.3f} {v['min']:8.3f}"
        f" {v['max']:8.3f} {v['p5']:8.3f} {v['p95']:8.3f} {v['p99']:8.3f}"
    )


# ---------------------------------------------------------------------------
# ASCII histogram — data in µs, axis in ms
# ---------------------------------------------------------------------------

def _histogram(
    values_us: list[float],
    lo_ms: float, hi_ms: float,
    buckets: int,
    label: str,
    budget_ms: float = 0.0,
) -> None:
    lo_us = lo_ms * 1000.0
    hi_us = hi_ms * 1000.0
    width = (hi_us - lo_us) / buckets
    counts = [0] * buckets
    underflow = overflow = 0
    for v in values_us:
        if v < lo_us:
            underflow += 1
        elif v >= hi_us:
            overflow += 1
        else:
            idx = max(0, min(buckets - 1, int((v - lo_us) / width)))
            counts[idx] += 1

    bar_max = max(counts) if any(counts) else 1
    bar_scale = 40 / bar_max
    step = (hi_ms - lo_ms) / buckets

    print(f"\n  {label}  (N={len(values_us)})")
    print(f"  {'Range':>14}   {'Count':>6}   Bar")
    if underflow:
        print(f"  {'< ' + f'{lo_ms:.1f} ms':>14}   {underflow:6d}")
    for i in range(buckets):
        lo_b = lo_ms + i * step
        hi_b = lo_ms + (i + 1) * step
        bar = "█" * int(counts[i] * bar_scale)
        marker = "  ← 20 ms budget" if budget_ms and lo_b < budget_ms <= hi_b else ""
        print(f"  {lo_b:6.3f}–{hi_b:5.3f} ms   {counts[i]:6d}   {bar}{marker}")
    if overflow:
        print(f"  {'>' + f'{hi_ms:.3f} ms':>14}   {overflow:6d}   ← LATE CYCLES")


# ---------------------------------------------------------------------------
# Profiler core
# ---------------------------------------------------------------------------

def run_profiler(
    hw: HardwareConfig,
    servos: list[ServoConfig],
    servo_ids: list[int],
    cycles: int,
    timeout_s: float,
    expect_echo: bool,
    do_write: bool,
) -> None:
    n = len(servo_ids)
    servo_cfg_map = {s.servo_id: s for s in servos}

    missing = [sid for sid in servo_ids if sid not in servo_cfg_map]
    if missing:
        raise SystemExit(f"No config for servo IDs: {missing}")

    sync_read_pkt = encode_sync_read(Reg.STATUS_START, Reg.STATUS_LEN, servo_ids)
    sync_read_rx  = (6 + Reg.STATUS_LEN) * n
    uart_floor_ms = (len(sync_read_pkt) + sync_read_rx) * 10 / 1000.0   # 10 bits/byte @ 1 Mbps
    sync_write_len = 0
    if do_write:
        dummy = [(sid, bytes(4)) for sid in servo_ids]
        sync_write_len = len(encode_sync_write(Reg.TARGET_POS_L, 4, dummy))
        uart_floor_ms += sync_write_len * 10 / 1000.0

    bus = SerialBus(
        port=hw.uart_port,
        baud_rate=hw.baud_rate,
        timeout=timeout_s,
        expect_echo=expect_echo,
    )
    imu = BNO055(i2c_bus=hw.i2c_bus, address=hw.i2c_address)
    servo_objects = [ST3215(servo_cfg_map[sid], bus) for sid in servo_ids]

    manager = ServoBusManager(
        servo_objects, bus, imu,
        rt_scheduling=True,    # request SCHED_FIFO (needs root or setcap)
        fast_mode=False,       # full 8-byte status read (same as production default)
        profiling=True,
    )

    bus.open()
    print(f"  Initializing BNO055 on I2C bus {hw.i2c_bus} addr 0x{hw.i2c_address:02X} (~700 ms)...")
    imu_ok = False
    try:
        imu.initialize()
        imu_ok = True
        print("  BNO055 ready.")
    except Exception as exc:
        print(f"  BNO055 init failed: {exc}")
        print("  Continuing without IMU (IMU thread will return default values).")

    print(f"\n{'=' * 62}")
    print(f"  Bus Profiler (ServoBusManager harness) — {hw.uart_port} @ {hw.baud_rate:,} bps")
    print(f"  Servos:      {n}  IDs: {servo_ids}")
    print(f"  Cycles:      {cycles}   echo: {'yes' if expect_echo else 'no'}")
    print(f"  SYNC_READ:   {len(sync_read_pkt)} B TX  +  {sync_read_rx} B RX  ({n} × {6 + Reg.STATUS_LEN} B)")
    if do_write:
        print(f"  SYNC_WRITE:  {sync_write_len} B TX  (standing positions)")
    print(f"  UART floor:  {uart_floor_ms:.2f} ms  (pure wire time, no gaps)")
    print(f"{'=' * 62}\n")

    manager.start()

    # Warmup: 50 cycles (~1 s) — lets IMU reader populate its cache and bus settle
    standing: dict[str, float] = {}
    if do_write:
        standing = {
            servo_cfg_map[sid].joint_name: servo_cfg_map[sid].default_position_deg
            for sid in servo_ids
        }

    t_wu = time.monotonic()
    while time.monotonic() - t_wu < 50 / 50.0:
        if do_write:
            manager.set_target_positions(standing, speed=300)
        time.sleep(0.010)

    manager.reset_profile_stats()

    last_report = 0
    deadline = time.monotonic() + cycles / 50.0 * 1.5   # 50% headroom
    while manager.profile_cycle_count < cycles and time.monotonic() < deadline:
        if do_write:
            # Post every 10 ms so the bus thread always finds a pending command
            manager.set_target_positions(standing, speed=300)
        time.sleep(0.010)
        n_done = manager.profile_cycle_count
        if n_done - last_report >= 50:
            last_report = n_done
            s = manager.get_profile_stats()
            recent_spare = s["t_spare_us"][-50:]
            recent_cycle = s["t_cycle_us"][-50:]
            spare_ms = sum(recent_spare) / len(recent_spare) / 1000.0 if recent_spare else 0.0
            hz = 1e6 / (sum(recent_cycle) / len(recent_cycle)) if recent_cycle else 0.0
            misses = s["miss_count"]
            print(f"  cycle {n_done:5d}/{cycles}  Hz: {hz:5.1f}  spare: {spare_ms:5.1f} ms  misses: {misses}")

    manager.stop()
    imu.close()
    bus.close()

    stats = manager.get_profile_stats()
    t_uart_us    = stats["t_uart_us"]
    t_write_w_us = stats["t_write_w_us"]
    t_state_us   = stats["t_state_us"]
    t_spare_us   = stats["t_spare_us"]
    t_cycle_us   = stats["t_cycle_us"]
    miss_count   = stats["miss_count"]
    imu_read_us  = stats["imu_read_us"]
    imu_read_hz  = stats["imu_read_hz"]

    # Trim to exactly 'cycles' samples (deadline timeout could give slightly more)
    t_uart_us  = t_uart_us[:cycles]
    t_state_us = t_state_us[:cycles]
    t_spare_us = t_spare_us[:cycles]
    t_cycle_us = t_cycle_us[:cycles]

    n_collected = len(t_cycle_us)

    # -----------------------------------------------------------------------
    # Report
    # -----------------------------------------------------------------------
    sched_ok = manager.rt_scheduling_active
    sched_line = (
        "applied ✓" if sched_ok else
        "⚠ not active  (run as root, or: sudo setcap cap_sys_nice+eip $(readlink -f venv/bin/python3))"
    )

    print(f"\n{'=' * 62}")
    print("  RESULTS")
    print(f"  SCHED_FIFO:  {sched_line}")
    print(f"{'=' * 62}")

    st_uart  = _stats(t_uart_us)
    st_state = _stats(t_state_us)
    st_spare = _stats(t_spare_us)
    st_cycle = _stats(t_cycle_us)

    _print_stats("t_uart_total  SYNC_READ TX+RX wall time", st_uart,  unit="ms")
    if t_write_w_us:
        _print_stats("t_write_w     SYNC_WRITE TX time",       _stats(t_write_w_us), unit="us")
    _print_stats("t_state_fetch  imu cache + state update",   st_state, unit="us")
    _print_stats("t_spare        available for NN / other",   st_spare, unit="ms")
    _print_stats("t_cycle        full cycle wall time",        st_cycle, unit="ms")

    if imu_read_us:
        st_imu = _stats(imu_read_us)
        staleness_ms = 1000.0 / imu_read_hz if imu_read_hz > 0 else 0.0
        print(f"\n  IMU reader thread  (I2C parallel with UART):")
        print(f"    Actual I2C read time  [us]")
        print(f"    {'mean':>8} {'std':>8} {'min':>8} {'max':>8} {'p5':>8} {'p95':>8} {'p99':>8}")
        s = st_imu
        print(
            f"    {s['mean']:8.1f} {s['std']:8.1f} {s['min']:8.1f}"
            f" {s['max']:8.1f} {s['p5']:8.1f} {s['p95']:8.1f} {s['p99']:8.1f}"
        )
        print(f"    Read rate: {imu_read_hz:.1f} Hz  (IMU data staleness ≤ {staleness_ms:.1f} ms)")
        if staleness_ms > 2.0:
            print(f"    ℹ At I2C 400 kHz: ~1 ms reads → ≤1 ms staleness (requires device tree change)")
    else:
        print(f"\n  IMU reader thread: no data (BNO055 not reachable on this run)")

    # Summary
    overhead_ms  = st_uart["mean"] / 1000.0 - (len(sync_read_pkt) + sync_read_rx) * 10 / 1000.0
    miss_pct     = miss_count / max(1, n_collected * n) * 100.0
    mean_hz      = 1e6 / st_cycle["mean"] if st_cycle["mean"] > 0 else 0.0
    p99_ms       = st_cycle["p99"] / 1000.0
    spare_mean   = st_spare["mean"] / 1000.0
    spare_p5     = st_spare["p5"]  / 1000.0
    spare_min    = st_spare["min"] / 1000.0
    imu_stale    = 1000.0 / imu_read_hz if imu_read_hz > 0 else 0.0

    # A "true overrun" is a cycle where work itself took > 20 ms (no spare time).
    # Sleep overshoot pushing total > 20 ms is expected and is NOT an overrun.
    true_overruns = sum(1 for v in t_spare_us if v <= 0)
    budget_ok = true_overruns == 0

    print(f"\n  {'─' * 58}")
    print(f"  Cycles collected:  {n_collected}")
    print(f"  Miss rate:         {miss_count}/{n_collected * n} servo-reads  ({miss_pct:.2f}%)")
    print(f"  UART floor:        {uart_floor_ms:.2f} ms")
    print(f"  RX overhead:       {overhead_ms:+.2f} ms  (inter-servo gaps + OS)")
    print(f"  Mean effective Hz: {mean_hz:.1f} Hz")
    print(f"  P99 cycle:         {p99_ms:.2f} ms  (sleep-paced; all cycles target 20 ms)")
    if budget_ok:
        print(f"  Budget:            ✓ 0 true overruns  (spare time never reached 0)")
    else:
        print(f"  Budget:            ✗ {true_overruns} true overruns  (work exceeded 20 ms — spare hit 0)")
    print(f"  Spare time mean:   {spare_mean:.2f} ms  (P5: {spare_p5:.2f} ms  min: {spare_min:.2f} ms)")
    if imu_stale > 0:
        print(f"  IMU staleness:     ≤ {imu_stale:.1f} ms  (= 1000 / imu_read_hz)")
    if spare_p5 > 0:
        print(f"  → At worst (P5), {spare_p5:.1f} ms available for NN inference per cycle")
    else:
        print(f"  ✗ No spare time at P5 — reduce load or enable BIPED_FAST_MODE=1")

    print()
    if overhead_ms > 1.0:
        print(f"  ⚠ RX overhead {overhead_ms:.1f} ms above floor — check RETURN_DELAY register (should be 0).")
    if miss_pct > 1.0:
        print(f"  ⚠ {miss_pct:.1f}% miss rate — check wiring and UART timeout setting.")
    if true_overruns > 0 and not sched_ok:
        print(f"  ⚠ True overruns detected — enable SCHED_FIFO:")
        print(f"    sudo python main.py")
        print(f"    or: sudo setcap cap_sys_nice+eip $(readlink -f venv/bin/python3)")

    # Wide view: full 0–25 ms range (shows loop is paced at 20 ms)
    _histogram(
        t_cycle_us,
        lo_ms=0.0, hi_ms=25.0, buckets=10,
        label="Cycle time histogram — wide view",
        budget_ms=20.0,
    )
    # Zoomed view: ±1 ms around 20 ms in 0.1 ms steps (shows actual jitter)
    _histogram(
        t_cycle_us,
        lo_ms=19.8, hi_ms=21.0, buckets=12,
        label="Cycle time histogram — zoomed (jitter view)",
        budget_ms=20.0,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="ST3215 bus profiler — measures timing via real ServoBusManager"
    )
    p.add_argument("--port",    default=None, help="UART port (default: from hardware.yaml)")
    p.add_argument("--baud",    type=int, default=None, help="Baud rate (default: from hardware.yaml)")
    p.add_argument("--servos",  default=None,
                   help="Comma-separated servo IDs (default: from robot.yaml)")
    p.add_argument("--cycles",  type=int, default=500, help="Number of cycles to collect (default: 500)")
    p.add_argument("--timeout", type=float, default=None,
                   help="Serial read timeout in seconds (default: from hardware.yaml)")
    p.add_argument("--echo",    action="store_true",
                   help="Enable TX echo drain (for half-duplex loopback adapters)")
    p.add_argument("--write",   action="store_true",
                   help="Send SYNC_WRITE every cycle (holds servos at standing positions)")
    p.add_argument("--hw-config",    default=str(_REPO_ROOT / "config" / "hardware.yaml"))
    p.add_argument("--robot-config", default=str(_REPO_ROOT / "config" / "robot.yaml"))
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    hw, servos = _load_configs(Path(args.hw_config), Path(args.robot_config))

    if args.port:  hw.uart_port = args.port
    if args.baud:  hw.baud_rate = args.baud
    timeout_s = args.timeout if args.timeout is not None else hw.uart_timeout_s

    if args.servos:
        servo_ids = [int(x.strip()) for x in args.servos.split(",")]
    elif servos:
        servo_ids = [s.servo_id for s in servos]
    else:
        raise SystemExit("No servo IDs — pass --servos or provide config/robot.yaml")

    run_profiler(
        hw=hw, servos=servos, servo_ids=servo_ids, cycles=args.cycles,
        timeout_s=timeout_s, expect_echo=args.echo, do_write=args.write,
    )


if __name__ == "__main__":
    main()
