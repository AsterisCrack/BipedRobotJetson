from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles

from web.routers import config, imu, kinematics, servos
from web.websocket import TelemetryBroadcaster

logger = logging.getLogger(__name__)

_STATIC = Path(__file__).parent / "static"
_ROBOT_DESC = Path(__file__).parent.parent / "RobotDescription"


def create_app(robot) -> FastAPI:

    broadcaster = TelemetryBroadcaster(robot)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        loop = asyncio.get_running_loop()
        robot.start_telemetry(broadcaster.queue, loop)
        broadcast_task = asyncio.create_task(broadcaster.broadcast_loop())
        logger.info("Web app started")
        yield
        broadcast_task.cancel()
        robot.shutdown()
        logger.info("Web app stopped")

    app = FastAPI(title="BipedRobot", lifespan=lifespan)
    app.state.robot = robot
    app.state.broadcaster = broadcaster

    app.include_router(servos.router,     prefix="/api/servos",      tags=["servos"])
    app.include_router(imu.router,        prefix="/api/imu",         tags=["imu"])
    app.include_router(kinematics.router, prefix="/api/kinematics",  tags=["kinematics"])
    app.include_router(config.router,     prefix="/api/config",      tags=["config"])

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await broadcaster.handle(ws)

    # Serve robot description assets (URDF + meshes) for the 3D viewer
    if _ROBOT_DESC.exists():
        app.mount("/robot_description", StaticFiles(directory=str(_ROBOT_DESC)), name="robot_desc")

    # Serve the web UI last (catches all remaining routes → index.html)
    app.mount("/", StaticFiles(directory=str(_STATIC), html=True), name="static")

    return app
