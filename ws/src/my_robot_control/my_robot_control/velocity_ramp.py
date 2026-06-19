"""Pure-Python acceleration-limited velocity ramp. No rclpy -> unit-testable.

Used by velocity_smoother_node to turn an instantaneous target velocity into a
smooth, kinematically limited stream of setpoints.
"""
import math


def step_axis(current, target, dt, accel_limit, decel_limit):
    """Advance one axis velocity toward `target` by at most one limited step.

    `accel_limit` caps the change rate while increasing speed magnitude;
    `decel_limit` caps it while decreasing. Both are positive [units/s^2].
    Returns the next velocity.
    """
    delta = target - current
    if delta == 0.0:
        return target
    speeding_up = abs(target) > abs(current) and current * target >= 0.0
    limit = accel_limit if speeding_up else decel_limit
    max_step = limit * dt
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


class VelocityRamp:
    """Holds current (vx, wz) and ramps both toward a target each step."""

    def __init__(self, *, max_vel_x, max_vel_theta, max_accel_x, max_decel_x,
                 max_accel_theta, max_decel_theta):
        self.max_vel_x = max_vel_x
        self.max_vel_theta = max_vel_theta
        self.max_accel_x = max_accel_x
        self.max_decel_x = max_decel_x
        self.max_accel_theta = max_accel_theta
        self.max_decel_theta = max_decel_theta
        self.vx = 0.0
        self.wz = 0.0

    def step(self, target_vx, target_wz, dt):
        target_vx = max(-self.max_vel_x, min(self.max_vel_x, target_vx))
        target_wz = max(-self.max_vel_theta, min(self.max_vel_theta, target_wz))
        self.vx = step_axis(self.vx, target_vx, dt,
                            self.max_accel_x, self.max_decel_x)
        self.wz = step_axis(self.wz, target_wz, dt,
                            self.max_accel_theta, self.max_decel_theta)
        return self.vx, self.wz
