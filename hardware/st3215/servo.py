from __future__ import annotations

import logging
from dataclasses import dataclass

from hardware.config import ServoConfig
from hardware.serial_bus import SerialBus, SerialBusError
from hardware.st3215.protocol import (
    bytes_to_steps,
    encode_ping,
    encode_read,
    encode_write,
    pack_u16,
    steps_to_bytes,
    unpack_u16,
)
from hardware.st3215.registers import Reg

logger = logging.getLogger(__name__)

STEPS_PER_DEG: float = 4096.0 / 360.0


@dataclass
class ServoStatus:
    servo_id: int
    joint_name: str
    position_deg: float
    raw_steps: int
    raw_deg: float
    speed: int
    load: int
    voltage_v: float
    temperature_c: int
    torque_enabled: bool


class ST3215:
    """
    Driver for a single ST3215 servo on a shared SerialBus.

    All public methods that communicate with hardware can raise SerialBusError.
    Caller (Robot) is responsible for retry logic.
    """

    def __init__(self, config: ServoConfig, bus: SerialBus) -> None:
        self._cfg = config
        self._bus = bus
        self._torque_enabled = False

    @property
    def servo_id(self) -> int:
        return self._cfg.servo_id

    @property
    def joint_name(self) -> str:
        return self._cfg.joint_name

    @property
    def zero_offset_steps(self) -> int:
        return self._cfg.zero_offset_steps

    @zero_offset_steps.setter
    def zero_offset_steps(self, value: int) -> None:
        self._cfg.zero_offset_steps = value

    @property
    def direction_sign(self) -> int:
        return self._cfg.direction_sign

    @property
    def default_position_deg(self) -> float:
        return self._cfg.default_position_deg

    @default_position_deg.setter
    def default_position_deg(self, value: float) -> None:
        self._cfg.default_position_deg = value

    # ------------------------------------------------------------------
    # Unit conversion
    # ------------------------------------------------------------------

    def deg_to_steps(self, deg: float) -> int:
        if self._cfg.direction_sign == 0:
            return self._cfg.zero_offset_steps
        raw = self._cfg.zero_offset_steps + self._cfg.direction_sign * deg * STEPS_PER_DEG
        return int(round(max(0, min(4095, raw))))

    def steps_to_deg(self, steps: int) -> float:
        if self._cfg.direction_sign == 0:
            return 0.0
        return (steps - self._cfg.zero_offset_steps) / (STEPS_PER_DEG * self._cfg.direction_sign)

    # ------------------------------------------------------------------
    # Raw register I/O
    # ------------------------------------------------------------------

    def read_register(self, reg: int, length: int) -> bytes:
        packet = encode_read(self._cfg.servo_id, reg, length)
        return self._bus.transfer(packet, response_data_len=length)

    def write_register(self, reg: int, data: bytes) -> None:
        packet = encode_write(self._cfg.servo_id, reg, data)
        self._bus.transfer(packet, response_data_len=0)

    # ------------------------------------------------------------------
    # Connectivity
    # ------------------------------------------------------------------

    def ping(self) -> bool:
        try:
            packet = encode_ping(self._cfg.servo_id)
            self._bus.transfer(packet, response_data_len=0)
            return True
        except SerialBusError:
            return False

    # ------------------------------------------------------------------
    # Position
    # ------------------------------------------------------------------

    def set_position(self, deg: float, speed: int = 0) -> None:
        steps = self.deg_to_steps(deg)
        data = steps_to_bytes(steps) + pack_u16(speed)
        # Write TARGET_POS (2B) + TARGET_SPEED (2B) starting at 0x2A
        self.write_register(Reg.TARGET_POS_L, data)

    def get_position(self) -> float:
        data = self.read_register(Reg.CURRENT_POS_L, 2)
        return self.steps_to_deg(bytes_to_steps(data))

    def get_position_steps(self) -> int:
        data = self.read_register(Reg.CURRENT_POS_L, 2)
        return bytes_to_steps(data)

    def steps_to_deg_raw(self, steps: int) -> float:
        return (steps / STEPS_PER_DEG) * 360.0

    def set_position_steps(self, steps: int, speed: int = 0) -> None:
        data = steps_to_bytes(steps) + pack_u16(speed)
        self.write_register(Reg.TARGET_POS_L, data)

    def set_speed(self, speed: int) -> None:
        self.write_register(Reg.TARGET_SPEED_L, pack_u16(speed))

    def set_acceleration(self, accel: int) -> None:
        self.write_register(Reg.ACCELERATION, bytes([max(0, min(254, accel))]))

    def set_torque_limit(self, limit: int) -> None:
        self.write_register(Reg.TORQUE_LIMIT_L, pack_u16(limit))

    # ------------------------------------------------------------------
    # Torque
    # ------------------------------------------------------------------

    def enable_torque(self) -> None:
        self.write_register(Reg.TORQUE_ENABLE, bytes([1]))
        self._torque_enabled = True

    def disable_torque(self) -> None:
        self.write_register(Reg.TORQUE_ENABLE, bytes([0]))
        self._torque_enabled = False

    @property
    def torque_enabled(self) -> bool:
        return self._torque_enabled

    # ------------------------------------------------------------------
    # PID
    # ------------------------------------------------------------------

    def set_pid(self, p: int, d: int, i: int) -> None:
        # PID_P, PID_D, PID_I are consecutive bytes at 0x15
        self.write_register(Reg.PID_P, bytes([p, d, i]))
        self._cfg.pid.p = p
        self._cfg.pid.d = d
        self._cfg.pid.i = i

    def get_pid(self) -> tuple[int, int, int]:
        data = self.read_register(Reg.PID_P, 3)
        return data[0], data[1], data[2]

    # ------------------------------------------------------------------
    # Bulk status read
    # ------------------------------------------------------------------

    def get_status(self) -> ServoStatus:
        # Read 8 bytes starting at 0x38: pos(2) speed(2) load(2) voltage(1) temp(1)
        data = self.read_register(Reg.STATUS_START, Reg.STATUS_LEN)
        pos_steps = bytes_to_steps(data, 0)
        speed = unpack_u16(data, 2)
        load = unpack_u16(data, 4)
        voltage_raw = data[6]
        temp = data[7]
        return ServoStatus(
            servo_id=self._cfg.servo_id,
            joint_name=self._cfg.joint_name,
            position_deg=round(self.steps_to_deg(pos_steps), 2),
            raw_steps=pos_steps,
            raw_deg=round(self.steps_to_deg_raw(pos_steps), 2),
            speed=speed,
            load=load,
            voltage_v=round(voltage_raw * 0.1, 2),
            temperature_c=temp,
            torque_enabled=self._torque_enabled,
        )

    # ------------------------------------------------------------------
    # Configuration (EPROM writes — require torque disabled)
    # ------------------------------------------------------------------

    def set_id(self, new_id: int) -> None:
        if not (0 <= new_id <= 253):
            raise ValueError(f"Servo ID must be 0-253, got {new_id}")
        self.disable_torque()
        self.write_register(Reg.ID, bytes([new_id]))
        logger.info("Servo %d → new ID %d (reconnect to verify)", self._cfg.servo_id, new_id)
        self._cfg.servo_id = new_id

    def set_zero_here(self) -> None:
        """Record current encoder position as the joint zero point."""
        data = self.read_register(Reg.CURRENT_POS_L, 2)
        current_steps = bytes_to_steps(data)
        self._cfg.zero_offset_steps = current_steps
        logger.info(
            "Servo %d (%s): zero offset set to %d steps",
            self._cfg.servo_id, self._cfg.joint_name, current_steps,
        )

    def apply_config_pid(self) -> None:
        self.set_pid(self._cfg.pid.p, self._cfg.pid.d, self._cfg.pid.i)
