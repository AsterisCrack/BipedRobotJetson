"""Motion guard: servo-moving web endpoints answer 409 while the policy is ARMING/ARMED/RUNNING.

No FastAPI TestClient here (it needs httpx, which is not a project dependency): the route functions
and the WebSocket dispatcher are called directly. The one TestClient test is skipped with a stated
reason when httpx is missing.
"""
import asyncio
import json
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

from policy.safety import FaultReason
from policy.state_machine import PolicyState, StateMachine
from robot.robot import Robot
from web.guards import MOTION_LOCK_MESSAGE
from web.routers import kinematics, servos
from web.websocket import TelemetryBroadcaster


def _sm(state):
    sm = StateMachine()
    path = {PolicyState.IDLE: [], PolicyState.ARMING: [PolicyState.ARMING],
            PolicyState.ARMED: [PolicyState.ARMING, PolicyState.ARMED],
            PolicyState.RUNNING: [PolicyState.ARMING, PolicyState.ARMED, PolicyState.RUNNING]}
    if state == PolicyState.FAULT:
        sm.fault_now(FaultReason.TILT, "test")
    else:
        for s in path[state]:
            assert sm.to(s)
    assert sm.state == state
    return sm


class FakeRobot:
    policy_motion_locked = Robot.policy_motion_locked

    def __init__(self, state=None):
        self._policy = None if state is None else SimpleNamespace(sm=_sm(state))
        self.calls = []
        self.policy = self._policy

    def _rec(self, name):
        def f(*a, **k):
            self.calls.append(name)
            return SimpleNamespace(success=True, angles_deg=[0.0] * 6, position_error_m=0.0, message="ok")
        return f

    def __getattr__(self, name):          # any other robot/servo call is recorded
        if name.startswith("_"):
            raise AttributeError(name)
        return self._rec(name)

    def home_pose_name(self):
        return "stand"

    def get_servo_any(self, servo_id):
        return SimpleNamespace(
            set_position=self._rec("servo.set_position"),
            set_position_steps=self._rec("servo.set_position_steps"),
            enable_torque=self._rec("servo.enable_torque"),
            disable_torque=self._rec("servo.disable_torque"),
            set_pid=self._rec("servo.set_pid"), set_zero_here=self._rec("servo.set_zero_here"),
            set_id=self._rec("servo.set_id"), set_speed=self._rec("servo.set_speed"),
            set_acceleration=self._rec("servo.set_acceleration"),
            set_torque_limit=self._rec("servo.set_torque_limit"),
            get_position_steps=lambda: 2048, zero_offset_steps=2048, direction_sign=1)


def _req(robot):
    return SimpleNamespace(app=SimpleNamespace(state=SimpleNamespace(robot=robot)))


@pytest.mark.parametrize("state, locked", [
    (None, False),                    # runner never created
    (PolicyState.IDLE, False),
    (PolicyState.ARMING, True),
    (PolicyState.ARMED, True),
    (PolicyState.RUNNING, True),
    (PolicyState.FAULT, False),
])
def test_predicate(state, locked):
    assert FakeRobot(state).policy_motion_locked() is locked


def test_predicate_does_not_create_the_runner():
    robot = SimpleNamespace()          # no _policy attribute at all
    assert Robot.policy_motion_locked(robot) is False


GATED = [
    ("kin.execute_pose", lambda r: kinematics.execute_pose("stand", _req(r))),
    ("kin.go_home", lambda r: kinematics.go_home(_req(r))),
    ("kin.ik_execute", lambda r: kinematics.inverse_kinematics(
        "left", kinematics.IKCmd(x=0.0, y=0.0, z=-0.2), _req(r))),
    ("servo.position", lambda r: servos.set_position(1, servos.PositionCmd(deg=10.0), _req(r))),
    ("servo.raw_position", lambda r: servos.set_raw_position(1, servos.RawPositionCmd(steps=2000), _req(r))),
    ("servo.torque_enable", lambda r: servos.set_torque(1, servos.TorqueCmd(enable=True), _req(r))),
    ("servo.pid", lambda r: servos.set_pid(1, servos.PIDCmd(p=1, d=0, i=0), _req(r))),
    ("servo.zero", lambda r: servos.set_zero(1, _req(r))),
    ("servo.zero_offset", lambda r: servos.set_zero_offset(1, _req(r))),
    ("servo.direction_sign", lambda r: servos.update_direction_sign(
        1, servos.DirectionSignCmd(direction_sign=-1), _req(r))),
    ("servo.default_position", lambda r: servos.update_default_position(
        1, servos.DefaultPositionCmd(default_position_deg=5.0), _req(r))),
    ("servo.simulation", lambda r: servos.set_simulation_mode(_req(r), {"enabled": True})),
    ("servo.sync_positions", lambda r: servos.sync_positions(
        servos.SyncPositionsCmd(joints={"l_knee_joint": 1.0}), _req(r))),
]


@pytest.mark.parametrize("state", [PolicyState.ARMING, PolicyState.ARMED, PolicyState.RUNNING])
@pytest.mark.parametrize("name, call", GATED, ids=[g[0] for g in GATED])
def test_http_409_while_locked(state, name, call):
    robot = FakeRobot(state)
    with pytest.raises(HTTPException) as exc:
        call(robot)
    assert exc.value.status_code == 409
    assert exc.value.detail == MOTION_LOCK_MESSAGE
    assert robot.calls == []           # nothing reached the robot


@pytest.mark.parametrize("state", [None, PolicyState.IDLE, PolicyState.FAULT])
def test_http_allowed_when_not_locked(state):
    robot = FakeRobot(state)
    assert kinematics.execute_pose("stand", _req(robot))["ok"]
    assert servos.set_position(1, servos.PositionCmd(deg=10.0), _req(robot))["ok"]
    assert "go_to_pose" in robot.calls and "servo.set_position" in robot.calls


@pytest.mark.parametrize("state", [PolicyState.ARMING, PolicyState.ARMED, PolicyState.RUNNING])
def test_torque_disable_and_ik_preview_stay_allowed(state):
    robot = FakeRobot(state)
    assert servos.set_torque(1, servos.TorqueCmd(enable=False), _req(robot))["enabled"] is False
    assert "servo.disable_torque" in robot.calls
    # execute=False only computes the solution, it moves nothing
    out = kinematics.inverse_kinematics(
        "left", kinematics.IKCmd(x=0.0, y=0.0, z=-0.2, execute=False), _req(robot))
    assert out["success"] and "compute_ik" in robot.calls


# -- WebSocket ----------------------------------------------------------------

class FakeWS:
    def __init__(self):
        self.sent = []

    async def send_text(self, text):
        self.sent.append(json.loads(text))


def _dispatch(robot, msg):
    ws = FakeWS()
    asyncio.run(TelemetryBroadcaster(robot)._dispatch(ws, json.dumps(msg)))
    return ws.sent


WS_GATED = [
    {"type": "set_position", "servo_id": 1, "position_deg": 5.0},
    {"type": "set_pose", "pose": "stand"},
    {"type": "set_foot_ik", "leg": "left", "x": 0.0, "y": 0.0, "z": -0.2},
    {"type": "set_joints_fk", "leg": "left", "angles_deg": [0] * 6},
    {"type": "set_pid", "servo_id": 1, "p": 1, "d": 0, "i": 0},
    {"type": "set_torque", "servo_id": 1, "enable": True},
]


@pytest.mark.parametrize("msg", WS_GATED, ids=[m["type"] + str(m.get("enable", "")) for m in WS_GATED])
def test_ws_error_while_locked(msg):
    robot = FakeRobot(PolicyState.RUNNING)
    sent = _dispatch(robot, msg)
    assert sent == [{"type": "error", "cmd": msg["type"], "message": MOTION_LOCK_MESSAGE}]
    assert robot.calls == []


def test_ws_torque_disable_and_estop_allowed_while_locked():
    robot = FakeRobot(PolicyState.RUNNING)
    robot.get_servo_by_id = robot.get_servo_any
    sent = _dispatch(robot, {"type": "set_torque", "servo_id": 1, "enable": False})
    assert sent[0]["type"] == "ack" and "servo.disable_torque" in robot.calls
    robot.policy = None                 # estop path with no runner: bare torque cut
    sent = _dispatch(robot, {"type": "policy_estop"})
    assert sent[0]["type"] == "ack" and "disable_all_torques" in robot.calls


def test_ws_position_allowed_when_idle():
    robot = FakeRobot(PolicyState.IDLE)
    robot.get_servo_by_id = lambda sid: SimpleNamespace(joint_name="l_knee_joint")
    sent = _dispatch(robot, {"type": "set_position", "servo_id": 1, "position_deg": 5.0})
    assert sent[0]["type"] == "ack" and "sync_write_positions" in robot.calls


# -- real HTTP stack (needs httpx for starlette's TestClient) ----------------------

def test_testclient_returns_409():
    pytest.importorskip("httpx", reason="httpx is not installed; TestClient unavailable "
                                        "(route functions are tested directly above)")
    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    app = FastAPI()
    app.include_router(kinematics.router, prefix="/api/kinematics")
    app.include_router(servos.router, prefix="/api/servos")
    app.state.robot = FakeRobot(PolicyState.RUNNING)
    c = TestClient(app)
    for url, body in [("/api/kinematics/poses/stand", None), ("/api/kinematics/home", None),
                      ("/api/servos/1/position", {"deg": 1.0})]:
        r = c.post(url, json=body) if body else c.post(url)
        assert r.status_code == 409 and r.json()["detail"] == MOTION_LOCK_MESSAGE
    assert c.post("/api/servos/1/torque", json={"enable": False}).status_code == 200
    app.state.robot = FakeRobot(PolicyState.IDLE)
    assert c.post("/api/kinematics/home").status_code == 200
