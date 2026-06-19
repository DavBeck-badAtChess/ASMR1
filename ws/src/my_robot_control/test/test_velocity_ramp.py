import math

from my_robot_control.velocity_ramp import step_axis, VelocityRamp


def _close(a, b):
    return abs(a - b) < 1e-9


def test_accelerates_by_accel_limit():
    # current 0, target 0.3, accel 0.5, dt 0.1 -> step 0.05
    assert _close(step_axis(0.0, 0.3, 0.1, 0.5, 1.0), 0.05)


def test_clamps_to_target_when_within_one_step():
    assert _close(step_axis(0.28, 0.3, 0.1, 0.5, 1.0), 0.3)


def test_decelerates_by_decel_limit():
    # slowing toward 0 uses decel limit 1.0 -> step 0.1 -> 0.2
    assert _close(step_axis(0.3, 0.0, 0.1, 0.5, 1.0), 0.2)


def test_negative_target_keeps_sign():
    assert _close(step_axis(0.0, -0.3, 0.1, 0.5, 1.0), -0.05)


def test_ramp_clamps_to_max_velocity():
    r = VelocityRamp(max_vel_x=0.5, max_vel_theta=1.0, max_accel_x=10.0,
                     max_decel_x=10.0, max_accel_theta=10.0, max_decel_theta=10.0)
    vx, _ = r.step(5.0, 0.0, 1.0)
    assert _close(vx, 0.5)


def test_ramp_zero_target_decelerates_axis():
    r = VelocityRamp(max_vel_x=0.5, max_vel_theta=1.0, max_accel_x=0.5,
                     max_decel_x=1.0, max_accel_theta=1.0, max_decel_theta=2.0)
    r.vx = 0.3
    vx, _ = r.step(0.0, 0.0, 0.1)
    assert _close(vx, 0.2)
