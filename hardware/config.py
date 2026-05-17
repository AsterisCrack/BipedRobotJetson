from __future__ import annotations

from pydantic import BaseModel, Field, field_validator


class PIDConfig(BaseModel):
    p: int = 32
    d: int = 16
    i: int = 0


class ServoConfig(BaseModel):
    servo_id: int
    joint_name: str
    direction_sign: int = 1
    zero_offset_steps: int = 2048
    default_position_deg: float = 0.0
    pid: PIDConfig = Field(default_factory=PIDConfig)

    @field_validator("direction_sign")
    @classmethod
    def _validate_direction_sign(cls, v: int) -> int:
        if v not in (-1, 1):
            raise ValueError(f"direction_sign must be -1 or 1, got {v!r}")
        return v


class HardwareConfig(BaseModel):
    uart_port: str = "/dev/ttyTHS1"
    baud_rate: int = 1_000_000
    i2c_bus: int = 7
    i2c_address: int = 0x28  # BNO055 default (ADDR pin low)
    uart_timeout_s: float = 0.05
    max_retries: int = 3
    expect_echo: bool = False
