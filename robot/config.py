from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field
from pydantic_settings import BaseSettings, SettingsConfigDict


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


class HardwareConfig(BaseModel):
    uart_port: str = "/dev/ttyTHS1"
    baud_rate: int = 1_000_000
    i2c_bus: int = 7
    i2c_address: int = 0x4A
    uart_timeout_s: float = 0.05
    max_retries: int = 3


class RobotConfig(BaseModel):
    urdf_path: str = "RobotDescription/urdf/robot_flat.urdf"
    home_pose: str = "stand"
    servos: list[ServoConfig] = Field(default_factory=list)
    poses: dict[str, dict[str, float]] = Field(default_factory=dict)


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    uart_port: str = "/dev/ttyTHS1"
    i2c_bus: int = 7

    hardware: HardwareConfig = Field(default_factory=HardwareConfig)
    robot: RobotConfig = Field(default_factory=RobotConfig)

    @classmethod
    def load(
        cls,
        hardware_yaml: str = "config/hardware.yaml",
        robot_yaml: str = "config/robot.yaml",
    ) -> "Settings":
        hw_data = _read_yaml(hardware_yaml)
        robot_data = _read_yaml(robot_yaml)

        settings = cls()
        hardware = HardwareConfig(**hw_data)
        # .env overrides yaml for port/bus
        hardware.uart_port = settings.uart_port
        hardware.i2c_bus = settings.i2c_bus

        return cls(
            uart_port=settings.uart_port,
            i2c_bus=settings.i2c_bus,
            hardware=hardware,
            robot=RobotConfig(**robot_data),
        )

    def servo_by_id(self, servo_id: int) -> ServoConfig:
        for s in self.robot.servos:
            if s.servo_id == servo_id:
                return s
        raise KeyError(f"No servo with id {servo_id}")

    def servo_by_joint(self, joint_name: str) -> ServoConfig:
        for s in self.robot.servos:
            if s.joint_name == joint_name:
                return s
        raise KeyError(f"No servo for joint {joint_name!r}")


def _read_yaml(path: str) -> dict[str, Any]:
    p = Path(path)
    if not p.exists():
        return {}
    with p.open() as f:
        return yaml.safe_load(f) or {}
