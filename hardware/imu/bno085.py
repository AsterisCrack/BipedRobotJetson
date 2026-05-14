from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class IMUReading:
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
    euler_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)               # roll, pitch, yaw
    accel: tuple[float, float, float] = (0.0, 0.0, 9.81)                  # m/s²
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)                    # rad/s
    calibration_status: int = 0                                             # 0-3


class BNO085:
    """
    Wrapper around the Adafruit BNO08x CircuitPython library.

    Requires adafruit-blinka and adafruit-circuitpython-bno08x.
    On Jetson Orin Nano, i2c_bus=7 maps to /dev/i2c-7 (40-pin header SDA/SCL).
    BNO085 default I2C address: 0x4A (ADDR pin low) or 0x4B (ADDR pin high).

    Usage:
        imu = BNO085(i2c_bus=7, address=0x4A)
        imu.initialize()
        reading = imu.read()
        imu.close()
    """

    def __init__(self, i2c_bus: int = 7, address: int = 0x4A) -> None:
        self._bus = i2c_bus
        self._address = address
        self._device = None
        self._i2c = None
        self._last_reading = IMUReading()

    def initialize(self) -> None:
        import busio
        import board
        from adafruit_bno08x.i2c import BNO08X_I2C
        from adafruit_bno08x import (
            BNO_REPORT_ROTATION_VECTOR,
            BNO_REPORT_ACCELEROMETER,
            BNO_REPORT_GYROSCOPE,
        )

        # Blinka maps board.SCL/SDA to the default I2C bus.
        # For Jetson Orin Nano bus 7, set env var BLINKA_JETSON_INITOVERTMP=1
        # or use the explicit pin names if your Blinka version supports them.
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._device = BNO08X_I2C(self._i2c, address=self._address, reset=None)

        self._device.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self._device.enable_feature(BNO_REPORT_ACCELEROMETER)
        self._device.enable_feature(BNO_REPORT_GYROSCOPE)

        logger.info("BNO085 initialised on I2C bus %d, addr 0x%02X", self._bus, self._address)

    def read(self) -> IMUReading:
        if self._device is None:
            return self._last_reading

        try:
            quat = self._device.quaternion          # (i, j, k, real) — note Adafruit order
            if quat is not None:
                i, j, k, real = quat
                w, x, y, z = real, i, j, k
                roll, pitch, yaw = _quat_to_euler(w, x, y, z)
                accel = self._device.acceleration or (0.0, 0.0, 9.81)
                gyro = self._device.gyro or (0.0, 0.0, 0.0)
                try:
                    cal_status = int(self._device.calibration_status)
                except Exception:
                    logger.debug("calibration_status unavailable, defaulting to 0")
                    cal_status = 0
                self._last_reading = IMUReading(
                    quaternion=(round(w, 4), round(x, 4), round(y, 4), round(z, 4)),
                    euler_deg=(round(roll, 2), round(pitch, 2), round(yaw, 2)),
                    accel=(round(accel[0], 3), round(accel[1], 3), round(accel[2], 3)),
                    gyro=(round(gyro[0], 4), round(gyro[1], 4), round(gyro[2], 4)),
                    calibration_status=cal_status,
                )
        except Exception as exc:
            logger.warning("IMU read error: %s", exc)

        return self._last_reading

    def calibrate(self) -> None:
        if self._device is None:
            return
        try:
            self._device.begin_calibration()
            logger.info("BNO085 dynamic calibration started")
        except Exception as exc:
            logger.warning("Calibration start failed: %s", exc)

    def close(self) -> None:
        if self._i2c is not None:
            try:
                self._i2c.deinit()
            except Exception:
                pass
        self._device = None
        self._i2c = None


def _quat_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert unit quaternion (w,x,y,z) to roll, pitch, yaw in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) — clamped for numerical stability
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
