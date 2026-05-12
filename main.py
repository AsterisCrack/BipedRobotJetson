import logging
import sys

import uvicorn

from robot.config import Settings
from robot.robot import Robot
from web.app import create_app

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)

logger = logging.getLogger(__name__)


def main() -> None:
    settings = Settings.load()
    logger.info("UART port: %s  |  I2C bus: %d", settings.hardware.uart_port, settings.hardware.i2c_bus)
    logger.info("Servos configured: %d", len(settings.robot.servos))

    robot = Robot(settings)

    try:
        robot.initialize()
    except Exception as exc:
        logger.error("Robot init failed: %s — running in simulation mode", exc)

    app = create_app(robot)

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8080,
        log_level="info",
        workers=1,  # single process — only one serial bus owner
    )


if __name__ == "__main__":
    main()
