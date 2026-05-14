"""
hardware — standalone ST3215 servo bus library and BNO085 IMU driver.

Public surface:
  from hardware.config import PIDConfig, ServoConfig, HardwareConfig
  from hardware.serial_bus import SerialBus, SerialBusError
  from hardware.servo_bus_manager import ServoBusManager
  from hardware.st3215.servo import ST3215, ServoStatus
  from hardware.imu.bno085 import BNO085, IMUReading

No dependencies on robot/, kinematics/, or web/.
"""
