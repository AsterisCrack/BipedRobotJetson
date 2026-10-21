"""Auto-recover (policy/autopilot.py) and the streamed-command deadman.

AutoPilot.tick(now) is driven directly with fake time (the thread is never started). The runner is
the real PolicyRunner on the shared stubs, so arm()/start()/fault() run their real code.
"""
import math
import time

import pytest

from policy.autopilot import ARM_RETRY_S, MAX_ARM_FAILURES
from policy.safety import FaultReason
from policy.state_machine import PolicyState
from runner_stubs import make_runner

UPRIGHT = (0.0, 0.0, -1.0)


def _tilted(deg):
    r = math.radians(deg)
    return (math.sin(r), 0.0, -math.cos(r))


def _auto_runner(tmp_path, monkeypatch):
    runner, robot = make_runner(tmp_path, monkeypatch)
    monkeypatch.setattr(runner, "_audit_servo_registers", lambda: None)   # reads real registers
    runner.auto.enable(start_thread=False)
    return runner, robot


def _fall(runner):
    runner.fault(FaultReason.TILT, "test fall")
    assert runner.sm.state == PolicyState.FAULT


def _rearmed(runner, robot, t=0.0):
    """Fall, stand it up, tick through settle: returns the time after the re-arm."""
    _fall(runner)
    robot._bus_manager.projected_gravity = UPRIGHT
    runner.auto.tick(t)
    runner.auto.tick(t + runner.auto.settings["settle_s"])
    assert runner.sm.state == PolicyState.ARMING
    return t + runner.auto.settings["settle_s"]


def test_rearms_only_after_upright_for_settle(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    _fall(runner)
    bm = robot._bus_manager

    bm.projected_gravity = _tilted(30)          # lying there
    for t in (0.0, 1.0, 5.0):
        runner.auto.tick(t)
    assert runner.sm.state == PolicyState.FAULT and runner.auto.phase == "wait_upright"

    bm.projected_gravity = UPRIGHT
    runner.auto.tick(10.0)
    runner.auto.tick(10.5)
    assert runner.sm.state == PolicyState.FAULT          # not settled yet
    assert runner.auto.status()["upright_s"] == pytest.approx(0.5)
    runner.auto.tick(11.0)
    assert runner.sm.state == PolicyState.ARMING
    assert ("attach_hook",) in robot.log
    assert any(e[0] == "torque_on" for e in robot.log)
    assert runner.auto.phase == "arming"


def test_one_tilted_sample_resets_the_settle_timer(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    _fall(runner)
    bm = robot._bus_manager
    bm.projected_gravity = UPRIGHT
    runner.auto.tick(0.0)
    runner.auto.tick(0.9)
    bm.projected_gravity = _tilted(15)
    runner.auto.tick(0.95)
    bm.projected_gravity = UPRIGHT
    runner.auto.tick(1.0)
    runner.auto.tick(1.5)
    assert runner.sm.state == PolicyState.FAULT
    runner.auto.tick(2.0)
    assert runner.sm.state == PolicyState.ARMING


def test_moving_robot_is_not_upright(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    _fall(runner)
    robot._bus_manager.projected_gravity = UPRIGHT
    robot._bus_manager.angular_vel = (1.5, 0.0, 0.0)     # being swung through vertical
    for t in (0.0, 1.0, 2.0, 5.0):
        runner.auto.tick(t)
    assert runner.sm.state == PolicyState.FAULT


def test_countdown_then_start(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    t = _rearmed(runner, robot)
    runner.sm.to(PolicyState.ARMED, "ramp complete (test)")

    runner.auto.tick(t + 1.0)
    assert runner.auto.phase == "countdown"
    assert runner.auto.status()["countdown_left"] == pytest.approx(5.0)
    runner.auto.tick(t + 5.0)
    assert runner.sm.state == PolicyState.ARMED
    assert runner.auto.status()["countdown_left"] == pytest.approx(1.0)
    runner.auto.tick(t + 6.0)
    assert runner.sm.state == PolicyState.RUNNING
    assert runner.auto.recoveries == 1 and runner.auto.phase == "watching"


def test_operator_armed_is_never_auto_started(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    runner.arm()
    runner.sm.to(PolicyState.ARMED, "ramp complete (test)")
    for t in (0.0, 10.0, 100.0):
        runner.auto.tick(t)
    assert runner.sm.state == PolicyState.ARMED and runner.auto.phase == "watching"


def test_operator_stop_is_not_restarted(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    t = _rearmed(runner, robot)
    runner.sm.to(PolicyState.ARMED, "ramp complete (test)")
    runner.auto.tick(t)
    runner.auto.tick(t + 10.0)
    assert runner.sm.state == PolicyState.RUNNING
    runner.stop()
    for dt in (11.0, 20.0, 50.0):
        runner.auto.tick(t + dt)
    assert runner.sm.state == PolicyState.ARMED


def test_tipped_during_countdown_faults_and_waits_again(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    t = _rearmed(runner, robot)
    runner.sm.to(PolicyState.ARMED, "ramp complete (test)")
    runner.auto.tick(t)
    del robot.log[:]
    robot._bus_manager.projected_gravity = _tilted(60)
    runner.auto.tick(t + 1.0)
    assert runner.sm.state == PolicyState.FAULT and runner.sm.fault[0] == FaultReason.TILT
    assert any(e[0] == "torque_off" for e in robot.log)
    runner.auto.tick(t + 2.0)
    assert runner.auto.enabled and runner.auto.phase == "wait_upright"


@pytest.mark.parametrize("reason", [FaultReason.STALE_SENSOR, FaultReason.INFERENCE_ERROR,
                                    FaultReason.BAD_OBSERVATION, FaultReason.LOOP_OVERRUN])
def test_non_tilt_fault_disables_auto(tmp_path, monkeypatch, reason):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    runner.fault(reason, "test")
    robot._bus_manager.projected_gravity = UPRIGHT
    runner.auto.tick(0.0)
    runner.auto.tick(5.0)
    assert not runner.auto.enabled and reason in runner.auto.last_error
    assert runner.sm.state == PolicyState.FAULT          # still latched for the operator


def test_estop_and_disarm_disable_auto(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    runner.estop()
    assert not runner.auto.enabled and runner.auto.last_error == "e-stop"
    runner.auto.tick(0.0)
    runner.auto.tick(5.0)
    assert runner.sm.fault[0] == FaultReason.ESTOP       # never cleared

    runner.clear_fault()
    runner.auto.enable(start_thread=False)
    runner.disarm()
    assert not runner.auto.enabled


def test_rearm_never_clears_an_estop(tmp_path, monkeypatch):
    """An e-stop landing between the decision and the action must win."""
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    _fall(runner)
    runner.sm._state = PolicyState.IDLE                  # re-fault with the e-stop reason
    runner.estop()
    runner.auto.enabled = True                           # as if decided just before the e-stop
    runner.auto._rearm(0.0)
    assert runner.sm.state == PolicyState.FAULT and runner.sm.fault[0] == FaultReason.ESTOP


def test_arm_failures_back_off_then_give_up(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)

    def boom():
        raise RuntimeError("servos never replied")
    monkeypatch.setattr(runner, "arm", boom)
    _fall(runner)
    robot._bus_manager.projected_gravity = UPRIGHT
    settle = runner.auto.settings["settle_s"]

    t = 0.0
    for i in range(1, MAX_ARM_FAILURES + 1):
        runner.auto.tick(t)
        runner.auto.tick(t + settle)
        if i < MAX_ARM_FAILURES:
            assert runner.auto.enabled and "servos never replied" in runner.auto.last_error
            assert runner.sm.state == PolicyState.FAULT and runner.sm.fault[0] == FaultReason.TILT
            runner.auto.tick(t + settle + ARM_RETRY_S / 2)   # backing off: timer does not run
            assert runner.auto.status()["upright_s"] == 0.0
        t += settle + ARM_RETRY_S
    assert not runner.auto.enabled and "gave up" in runner.auto.last_error


def test_settings_are_range_checked(tmp_path, monkeypatch):
    runner, robot = _auto_runner(tmp_path, monkeypatch)
    before = dict(runner.auto.settings)
    for bad in ({"upright_deg": 50}, {"settle_s": 0.0}, {"countdown_s": float("nan")},
                {"upright_deg": 5, "countdown_s": 99}):
        with pytest.raises(ValueError):
            runner.auto.configure(**bad)
    assert runner.auto.settings == before
    runner.auto.configure(countdown_s=3)
    assert runner.auto.settings["countdown_s"] == 3.0


# ---------------------------------------------------------------------------
# Streamed-command deadman
# ---------------------------------------------------------------------------

def _running(tmp_path, monkeypatch):
    runner, robot = make_runner(tmp_path, monkeypatch)
    monkeypatch.setattr(runner, "_audit_servo_registers", lambda: None)
    runner.arm()
    runner.sm.to(PolicyState.ARMED, "ramp complete (test)")
    runner.start()
    return runner, robot


def test_streamed_command_times_out(tmp_path, monkeypatch):
    runner, robot = _running(tmp_path, monkeypatch)
    runner.set_command(0.1, 0.0, 0.0, stream=True)
    runner.step(robot._bus_manager.get_rl_state())
    assert runner.sm.state == PolicyState.RUNNING
    assert runner._command[0] == pytest.approx(0.1)

    runner._command_deadline = time.monotonic() - 0.01   # the stream stopped arriving
    runner.step(robot._bus_manager.get_rl_state())
    assert runner._command.tolist() == [0.0, 0.0, 0.0]


def test_plain_command_holds(tmp_path, monkeypatch):
    runner, robot = _running(tmp_path, monkeypatch)
    runner.set_command(0.1, 0.0, 0.0, stream=True)
    runner.set_command(0.1, 0.0, 0.0)                    # slider: clears the deadman
    assert runner._command_deadline is None
    runner.step(robot._bus_manager.get_rl_state())
    assert runner._command[0] == pytest.approx(0.1)


def test_status_reports_command_ranges(tmp_path, monkeypatch):
    runner, _ = make_runner(tmp_path, monkeypatch,
                            command_ranges={"lin_vel_x": [-0.2, 0.2], "ang_vel_z": [-1.2, 1.2]})
    r = runner.status()["command_ranges"]
    assert r["lin_vel_x"] == [-0.2, 0.2] and r["ang_vel_z"] == [-1.2, 1.2]
    assert r["lin_vel_y"] == [-0.3, 0.3]                 # missing key: the runtime default
    assert runner.set_command(1.0, 0.0, 5.0)["command"] == [0.2, 0.0, 1.2]
