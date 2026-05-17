"""
BNO055 IMU driver using Adafruit_PureIO.smbus (direct /dev/i2c-N access).

No Blinka / Jetson.GPIO dependency — works on any Linux with I2C support.
BNO055 I2C address: 0x28 (ADDR pin low) or 0x29 (ADDR pin high).
Chip ID register 0x00 must return 0xA0.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class IMUReading:
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
    euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)               # roll, pitch, yaw
    accel: tuple[float, float, float] = (0.0, 0.0, 9.81)                  # m/s² (linear, no gravity)
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)                    # rad/s
    calibration_status: int = 0                                             # 0-3 (sys calibration)


class BNO055:
    """
    BNO055 9-DOF IMU in NDOF fusion mode.

    Reads quaternion, linear acceleration (gravity-free), and gyroscope.
    Uses Adafruit_PureIO.smbus for direct I2C access.

    Usage:
        imu = BNO055(i2c_bus=7, address=0x28)
        imu.initialize()   # ~700 ms for boot + NDOF settle
        reading = imu.read()
        imu.close()
    """

    # Register map (page 0)
    _REG_CHIP_ID    = 0x00
    _REG_PAGE_ID    = 0x07
    _REG_GYR_X_LSB = 0x14   # gyroscope X,Y,Z (6 bytes)
    _REG_EUL_H_LSB = 0x1A   # euler heading/roll/pitch (6 bytes)
    _REG_QUA_W_LSB = 0x20   # quaternion W,X,Y,Z (8 bytes)
    _REG_LIA_X_LSB = 0x28   # linear acceleration X,Y,Z (6 bytes)
    _REG_CALIB_STAT = 0x35
    _REG_OPR_MODE  = 0x3D
    _REG_PWR_MODE  = 0x3E
    _REG_SYS_TRIG  = 0x3F

    # Scale factors (default UNIT_SEL = 0x00)
    _QUAT_SCALE  = 1.0 / 16384.0  # 1/2^14 → dimensionless
    _EULER_SCALE = 1.0 / 16.0     # → degrees
    _LIA_SCALE   = 1.0 / 100.0    # → m/s²
    _GYRO_SCALE  = math.pi / (16.0 * 180.0)  # 1/16 dps → rad/s

    _MODE_CONFIG = 0x00
    _MODE_NDOF   = 0x0C  # full 9-DOF sensor fusion

    # Read calibration status every N calls (~1 Hz at 50 Hz loop) to avoid
    # a separate I2C transaction on every cycle.
    _CALIB_READ_EVERY = 50

    def __init__(self, i2c_bus: int = 7, address: int = 0x28) -> None:
        self._bus_num = i2c_bus
        self._address = address
        self._smbus = None
        self._last_reading = IMUReading()
        self._calib_counter = 0
        self._cached_sys_cal = 0

    def initialize(self) -> None:
        from Adafruit_PureIO import smbus  # deferred: only needed on hardware

        self._smbus = smbus.SMBus(self._bus_num)

        chip_id = self._smbus.read_byte_data(self._address, self._REG_CHIP_ID)
        if chip_id != 0xA0:
            self._smbus.close()
            self._smbus = None
            raise RuntimeError(
                f"BNO055 not found at 0x{self._address:02X} on bus {self._bus_num} "
                f"(chip_id=0x{chip_id:02X}, expected 0xA0)"
            )

        # Config mode → software reset → normal power → NDOF
        self._write(self._REG_OPR_MODE, self._MODE_CONFIG)
        time.sleep(0.025)
        self._write(self._REG_SYS_TRIG, 0x20)  # software reset
        time.sleep(0.65)
        self._write(self._REG_PWR_MODE, 0x00)   # normal power
        time.sleep(0.01)
        self._write(self._REG_PAGE_ID, 0x00)    # page 0
        self._write(self._REG_OPR_MODE, self._MODE_NDOF)
        time.sleep(0.025)

        logger.info(
            "BNO055 initialised on I2C bus %d, addr 0x%02X (NDOF fusion mode)",
            self._bus_num, self._address,
        )

    def read(self) -> IMUReading:
        if self._smbus is None:
            return self._last_reading
        try:
            # One bulk read: 0x14–0x2D = gyro(6) + euler(6) + quat(8) + linear_accel(6) = 26 bytes.
            # Registers are contiguous, so this replaces 3 separate block reads with 1.
            bulk = self._smbus.read_i2c_block_data(self._address, self._REG_GYR_X_LSB, 26)

            # Gyro: offsets 0–5
            gx = _s16(bulk[0], bulk[1]) * self._GYRO_SCALE
            gy = _s16(bulk[2], bulk[3]) * self._GYRO_SCALE
            gz = _s16(bulk[4], bulk[5]) * self._GYRO_SCALE

            # Euler: offsets 6–11 (not used directly — compute from quat for consistency)

            # Quaternion W,X,Y,Z: offsets 12–19
            w = _s16(bulk[12], bulk[13]) * self._QUAT_SCALE
            x = _s16(bulk[14], bulk[15]) * self._QUAT_SCALE
            y = _s16(bulk[16], bulk[17]) * self._QUAT_SCALE
            z = _s16(bulk[18], bulk[19]) * self._QUAT_SCALE
            roll, pitch, yaw = _quat_to_euler(w, x, y, z)

            # Linear accel X,Y,Z: offsets 20–25
            ax = _s16(bulk[20], bulk[21]) * self._LIA_SCALE
            ay = _s16(bulk[22], bulk[23]) * self._LIA_SCALE
            az = _s16(bulk[24], bulk[25]) * self._LIA_SCALE

            # Calibration status — read once every _CALIB_READ_EVERY calls (~1 Hz at 50 Hz)
            self._calib_counter = (self._calib_counter + 1) % self._CALIB_READ_EVERY
            if self._calib_counter == 0:
                cal_byte = self._smbus.read_byte_data(self._address, self._REG_CALIB_STAT)
                self._cached_sys_cal = (cal_byte >> 6) & 0x03

            self._last_reading = IMUReading(
                quaternion=(round(w, 4), round(x, 4), round(y, 4), round(z, 4)),
                euler_deg=(round(roll, 2), round(pitch, 2), round(yaw, 2)),
                accel=(round(ax, 3), round(ay, 3), round(az, 3)),
                gyro=(round(gx, 4), round(gy, 4), round(gz, 4)),
                calibration_status=self._cached_sys_cal,
            )
        except Exception as exc:
            logger.warning("IMU read error: %s", exc)

        return self._last_reading

    def calibrate(self) -> None:
        """BNO055 auto-calibrates in NDOF mode; just log a hint."""
        logger.info(
            "BNO055 calibration: rotate the sensor in multiple orientations "
            "and move in a figure-8 pattern. Monitor CALIB_STAT for 0xFF."
        )

    def close(self) -> None:
        if self._smbus is not None:
            try:
                self._smbus.close()
            except Exception:
                pass
            self._smbus = None

    def _write(self, reg: int, value: int) -> None:
        self._smbus.write_byte_data(self._address, reg, value)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _s16(lsb: int, msb: int) -> int:
    """Combine LSB + MSB bytes into a signed 16-bit integer."""
    val = (msb << 8) | lsb
    return val - 65536 if val >= 32768 else val


def _quat_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert unit quaternion (w,x,y,z) to roll, pitch, yaw in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
