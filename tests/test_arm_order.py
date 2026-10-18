"""arm(): every goal register is written (present position, ARM_SPEED, torque off) BEFORE any torque
enable. FakeRobot binds the real Robot.preload_goal_positions / enable_all_torques to fake servos,
so the recorded order is the real code's order."""
import pytest

from fastapi import HTTPException

from policy.policy_runner import ARM_SPEED
from policy.state_machine import PolicyState
from runner_stubs import OFFSETS, ROBOT_NAMES, make_runner


def _armable(tmp_path, monkeypatch):
    runner, robot = make_runner(tmp_path, monkeypatch)
    monkeypatch.setattr(runner, "_audit_servo_registers", lambda: None)   # reads real registers
    return runner, robot


def test_goals_written_before_torque_enable(tmp_path, monkeypatch):
    runner, robot = _armable(tmp_path, monkeypatch)
    out = runner.arm()
    assert out["state"] == "arming"

    kinds = [e[0] for e in robot.log]
    assert kinds.count("goal") == 12 and kinds.count("torque_on") == 12
    assert max(i for i, k in enumerate(kinds) if k == "goal") < min(
        i for i, k in enumerate(kinds) if k == "torque_on")

    # goal = PRESENT position (URDF deg, robot order) at ARM_SPEED
    present = dict(zip(ROBOT_NAMES, robot._bus_manager.positions))
    goals = {e[1]: (e[2], e[3]) for e in robot.log if e[0] == "goal"}
    assert goals == {n: (pytest.approx(present[n]), ARM_SPEED) for n in ROBOT_NAMES}
    assert all(present[n] == OFFSETS[n] + 20.0 for n in ROBOT_NAMES)
    # the same targets replace anything stale still queued for the bus thread
    queued = [e for e in robot.log if e[0] == "queue"]
    assert len(queued) == 1 and queued[0][2] == ARM_SPEED and queued[0][1] == present


def test_failed_goal_write_refuses_arm_without_torque(tmp_path, monkeypatch):
    runner, robot = _armable(tmp_path, monkeypatch)
    robot._servos["l_knee_joint"].fail_set_position = True
    with pytest.raises(RuntimeError, match="arm refused.*l_knee_joint"):
        runner.arm()
    assert runner.sm.state == PolicyState.IDLE
    assert not any(e[0] == "torque_on" for e in robot.log)


def test_motion_guard_covers_the_whole_arm_sequence(tmp_path, monkeypatch):
    """The state is IDLE until the end of arm(); the guard must still refuse during the preload
    and the torque enable, and after arm() returns (state ARMING)."""
    from web.guards import MOTION_LOCK_MESSAGE, require_motion_unlocked
    runner, robot = _armable(tmp_path, monkeypatch)
    assert runner.sm.state == PolicyState.IDLE and not robot.policy_motion_locked()
    seen = []

    def hook(kind):
        assert runner.sm.state == PolicyState.IDLE          # mid-sequence: not ARMING yet
        with pytest.raises(HTTPException) as exc:
            require_motion_unlocked(robot)                  # what a web Home / position hits
        assert exc.value.status_code == 409 and exc.value.detail == MOTION_LOCK_MESSAGE
        assert robot.policy_motion_locked()
        if kind == "goal":
            with pytest.raises(RuntimeError, match="already in progress"):
                runner.arm()                                # no concurrent second arm
        seen.append(kind)

    robot.log.hook = hook
    runner.arm()
    assert seen.count("goal") == 12 and seen.count("torque_on") == 12
    assert runner.sm.state == PolicyState.ARMING and robot.policy_motion_locked()   # by state now
    assert not runner.arming_in_progress


def test_lock_released_after_failed_arm(tmp_path, monkeypatch):
    runner, robot = _armable(tmp_path, monkeypatch)
    locked_during = []
    robot.log.hook = lambda kind: locked_during.append(robot.policy_motion_locked())
    robot._servos["r_knee_joint"].fail_set_position = True
    with pytest.raises(RuntimeError, match="arm refused"):
        runner.arm()
    assert locked_during and all(locked_during)
    assert runner.sm.state == PolicyState.IDLE
    assert not runner.arming_in_progress and not robot.policy_motion_locked()
    assert not any(e[0] == "torque_on" for e in robot.log)
    # and arming works again afterwards
    robot.log.hook = None
    robot._servos["r_knee_joint"].fail_set_position = False
    assert runner.arm()["state"] == "arming"


@pytest.mark.parametrize("age", [0.2, None])
def test_stale_cached_positions_refuse_arm(tmp_path, monkeypatch, age):
    runner, robot = _armable(tmp_path, monkeypatch)
    robot._bus_manager.stamp_age_s = age            # > max_sensor_age_s (0.15), or never stamped
    with pytest.raises(RuntimeError, match="stale"):
        runner.arm()
    assert runner.sm.state == PolicyState.IDLE and not runner.arming_in_progress
    assert robot.log == []                          # no goal write, no torque, nothing queued
    robot._bus_manager.stamp_age_s = 0.1            # fresh enough
    assert runner.arm()["state"] == "arming"


def test_existing_refusals_unchanged(tmp_path, monkeypatch):
    runner, robot = _armable(tmp_path, monkeypatch)
    robot._bus_manager.has_velocity = False
    with pytest.raises(RuntimeError, match="BIPED_FAST_MODE"):
        runner.arm()
    assert robot.log == []                                      # nothing written, no torque
