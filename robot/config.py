from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field
from pydantic_settings import BaseSettings, SettingsConfigDict

from hardware.config import HardwareConfig, PIDConfig, ServoConfig  # noqa: F401 (re-exported)


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
        try:
            return yaml.safe_load(f) or {}
        except yaml.YAMLError as exc:
            raise yaml.YAMLError(f"Failed to parse YAML at {p}: {exc}") from exc
