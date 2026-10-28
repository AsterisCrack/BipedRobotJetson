"""Stale-sensor detection: no hardware, no serial port, no ONNX."""
import math
import time
from types import SimpleNamespace

import numpy as np

from hardware.imu.bno055 import BNO055, IMUReading
from hardware.servo_bus_manager import ServoBusManager
from hardware.st3215.servo import ServoStatus
from policy.safety import FaultReason, SafetyLimits, SafetyMonitor, sensor_age

FRAME = np.zeros(50)
UPRIGHT = (0.0, 0.0, -1.0)


def _check(servo_age, imu_age, now=1000.0):
    """Run the same age -> check_observation path _step_running uses."""
    mon = SafetyMonitor(SafetyLimits())
    age = sensor_age(now, None if servo_age is None else now - servo_age,
                     None if imu_age is None else now - imu_age)
    return mon, mon.check_observation(FRAME, UPRIGHT, age)


def test_fresh_stamps_no_fault():
    _, fault = _check(0.02, 0.01)
    assert fault is None


def test_old_servo_stamp_faults():
    mon, fault = _check(0.3, 0.01)
    assert fault == FaultReason.STALE_SENSOR
    assert "300 ms" in mon.state.fault_detail


def test_old_imu_stamp_faults():
    _, fault = _check(0.01, 0.3)
    assert fault == FaultReason.STALE_SENSOR


def test_missing_stamp_is_infinitely_old():
    assert sensor_age(10.0, None, 9.9) == math.inf
    assert sensor_age(10.0, 9.9, None) == math.inf
    assert sensor_age(10.0, 9.9, 9.5) == 10.0 - 9.5
    _, fault = _check(None, 0.01)
    assert fault == FaultReason.STALE_SENSOR


def test_seven_missed_cycles_tolerated_eight_not():
    # 50 Hz: 0.15 s limit tolerates 7 consecutive missed 20 ms cycles (age 0.14 s).
    assert _check(0.14, 0.0)[1] is None
    assert _check(0.16, 0.0)[1] == FaultReason.STALE_SENSOR


# -- servo_bus_manager stamping ----------------------------------------------

def _servo(sid):
    return SimpleNamespace(
        servo_id=sid, joint_name=f"j{sid}", torque_enabled=False,
        steps_per_s_to_deg_per_s=lambda s: float(s))


def _status(sid):
    return ServoStatus(servo_id=sid, joint_name=f"j{sid}", position_deg=1.0 * sid,
                       raw_steps=0, raw_deg=0.0, speed=0, load=0, voltage_v=0.0,
                       temperature_c=0, torque_enabled=False)


def _manager():
    return ServoBusManager([_servo(1), _servo(2)], bus=None, imu=None)


def test_never_replied_servos_reported_missing():
    m = _manager()
    rl = m.get_rl_state()
    assert rl["servo_missing"] == [1, 2]
    assert rl["servo_stamp"] is None
    m._update_state({1: _status(1)}, IMUReading())
    rl = m.get_rl_state()
    assert rl["servo_missing"] == [2]
    assert rl["servo_stamp"] is None


def test_servo_stamp_is_oldest_and_not_refreshed_by_failed_reads():
    m = _manager()
    imu = IMUReading(stamp=time.monotonic())
    m._update_state({1: _status(1), 2: _status(2)}, imu)
    t_first = m.get_rl_state()["servo_stamp"]
    time.sleep(0.02)
    m._update_state({1: _status(1)}, imu)         # servo 2 missed this cycle
    rl = m.get_rl_state()
    assert rl["servo_missing"] == []
    assert rl["servo_stamp"] == t_first            # servo 2's older stamp governs
    assert rl["positions"] == [1.0, 2.0]           # last value still held
    assert rl["imu_stamp"] == imu.stamp
    m._update_state({}, imu)                       # failed SYNC_READ: stamps unchanged
    assert m.get_rl_state()["servo_stamp"] == t_first


# -- IMU stamping ------------------------------------------------------------

class _FakeSMBus:
    def __init__(self):
        self.fail = False

    def read_i2c_block_data(self, addr, reg, n):
        if self.fail:
            raise OSError("i2c")
        return [0] * n

    def read_byte_data(self, addr, reg):
        return 0


def test_imu_stamp_set_on_success_and_kept_on_fallback():
    imu = BNO055()
    assert imu.read().stamp is None                # no bus: never read
    imu._smbus = _FakeSMBus()
    r1 = imu.read()
    assert r1.stamp is not None
    imu._smbus.fail = True
    time.sleep(0.01)
    r2 = imu.read()
    assert r2.stamp == r1.stamp                    # fallback does not refresh


def test_servo_ages_name_the_stale_servo():
    m = _manager()
    assert all(math.isnan(a) for a in m.get_rl_state()["servo_ages_ms"])
    m._update_state({1: _status(1), 2: _status(2)}, IMUReading())
    m._servo_stamps[2] -= 0.2                     # servo 2 last replied 200 ms ago
    ages = m.get_rl_state()["servo_ages_ms"]
    assert ages[0] < 50.0 and 190.0 < ages[1] < 260.0


def test_dropout_logged_once_and_recovery_logged(caplog):
    import logging
    import hardware.servo_bus_manager as sbm
    m = _manager()
    caplog.set_level(logging.WARNING, logger=sbm.logger.name)
    for _ in range(sbm._MISS_LOG_AFTER + 4):      # servo 2 silent
        m._track_misses({1: _status(1)})
    missing = [r for r in caplog.records if "missing from" in r.getMessage()]
    assert len(missing) == 1 and "servo 2 (j2)" in missing[0].getMessage()
    m._track_misses({1: _status(1), 2: _status(2)})
    back = [r for r in caplog.records if "replying again" in r.getMessage()]
    assert len(back) == 1 and f"after {sbm._MISS_LOG_AFTER + 4} missed" in back[0].getMessage()
    assert m._consec_miss[2] == 0
