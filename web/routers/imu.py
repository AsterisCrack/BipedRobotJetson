from fastapi import APIRouter, Request

router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


@router.get("/")
def get_imu(request: Request) -> dict:
    reading = _robot(request).get_imu()
    return {
        "quaternion": {"w": reading.quaternion[0], "x": reading.quaternion[1],
                       "y": reading.quaternion[2], "z": reading.quaternion[3]},
        "euler_deg":  {"roll": reading.euler_deg[0], "pitch": reading.euler_deg[1],
                       "yaw": reading.euler_deg[2]},
        "accel":      {"x": reading.accel[0], "y": reading.accel[1], "z": reading.accel[2]},
        "gyro":       {"x": reading.gyro[0], "y": reading.gyro[1], "z": reading.gyro[2]},
        "calibration": reading.calibration_status,
    }


@router.post("/calibrate")
def calibrate(request: Request):
    _robot(request).calibrate_imu()
    return {"ok": True, "message": "Dynamic calibration started"}
