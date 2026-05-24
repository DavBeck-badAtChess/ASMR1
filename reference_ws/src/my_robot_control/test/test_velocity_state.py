import pytest

from my_robot_control.velocity_controller_node import VelocityState


def test_velocity_state_starts_inactive_zero():
    state = VelocityState()
    assert state.active is False
    assert state.linear_x == 0.0
    assert state.angular_z == 0.0


def test_velocity_state_set_goal_activates_with_values():
    state = VelocityState()
    state.set_goal(linear_x=0.3, angular_z=-0.2)
    assert state.active is True
    assert state.linear_x == pytest.approx(0.3)
    assert state.angular_z == pytest.approx(-0.2)


def test_velocity_state_set_goal_replaces_previous():
    state = VelocityState()
    state.set_goal(linear_x=0.3, angular_z=0.0)
    state.set_goal(linear_x=0.0, angular_z=0.5)
    assert state.active is True
    assert state.linear_x == pytest.approx(0.0)
    assert state.angular_z == pytest.approx(0.5)


def test_velocity_state_clear_deactivates_and_zeroes():
    state = VelocityState()
    state.set_goal(linear_x=0.4, angular_z=0.2)
    state.clear()
    assert state.active is False
    assert state.linear_x == 0.0
    assert state.angular_z == 0.0


def test_velocity_state_clear_when_already_inactive_is_safe():
    state = VelocityState()
    state.clear()
    assert state.active is False
    assert state.linear_x == 0.0
    assert state.angular_z == 0.0
