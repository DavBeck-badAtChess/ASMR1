#!/usr/bin/env python3
"""Acceleration-limited velocity smoother (Nav2-style).

Subscribes target velocities as TwistStamped on /cmd_vel, ramps the current
velocity toward the latest target under per-axis accel/decel limits on a fixed
timer, and republishes the smoothed command as TwistStamped on
/cmd_vel_smoothed (bridged to the Gazebo DiffDrive). If no fresh target arrives
within command_timeout_s (compared against the sim clock), the target ramps to
zero -- the reason the interface is TwistStamped.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import TwistStamped

from my_robot_control.velocity_ramp import VelocityRamp


class VelocitySmootherNode(Node):
    def __init__(self):
        super().__init__('velocity_smoother_node')
        self.declare_parameter(
            'rate_hz', 50.0,
            ParameterDescriptor(description='Smoother update rate [Hz].'))
        self.declare_parameter(
            'command_timeout_s', 0.5,
            ParameterDescriptor(description='Ramp to zero if no target within this time [s].'))
        self.declare_parameter(
            'max_vel_x', 0.6,
            ParameterDescriptor(description='Max linear velocity [m/s].'))
        self.declare_parameter(
            'max_vel_theta', 1.0,
            ParameterDescriptor(description='Max angular velocity [rad/s].'))
        self.declare_parameter(
            'max_accel_x', 0.5,
            ParameterDescriptor(description='Max linear acceleration [m/s^2].'))
        self.declare_parameter(
            'max_decel_x', 1.0,
            ParameterDescriptor(description='Max linear deceleration [m/s^2].'))
        self.declare_parameter(
            'max_accel_theta', 1.5,
            ParameterDescriptor(description='Max angular acceleration [rad/s^2].'))
        self.declare_parameter(
            'max_decel_theta', 3.0,
            ParameterDescriptor(description='Max angular deceleration [rad/s^2].'))

        g = self.get_parameter
        self._timeout = g('command_timeout_s').value
        self._ramp = VelocityRamp(
            max_vel_x=g('max_vel_x').value,
            max_vel_theta=g('max_vel_theta').value,
            max_accel_x=g('max_accel_x').value,
            max_decel_x=g('max_decel_x').value,
            max_accel_theta=g('max_accel_theta').value,
            max_decel_theta=g('max_decel_theta').value,
        )
        self._target = (0.0, 0.0)
        self._last_cmd_time = self.get_clock().now()

        # Default (volatile, depth 10) QoS -- a command stream, NOT latched.
        self.create_subscription(TwistStamped, '/cmd_vel', self._on_cmd, 10)
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel_smoothed', 10)

        self._dt = 1.0 / g('rate_hz').value
        self.create_timer(self._dt, self._tick)
        self.get_logger().info('velocity_smoother_node ready')

    def _on_cmd(self, msg):
        self._target = (msg.twist.linear.x, msg.twist.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        age = (now - self._last_cmd_time).nanoseconds * 1e-9
        target = self._target if age <= self._timeout else (0.0, 0.0)
        vx, wz = self._ramp.step(target[0], target[1], self._dt)

        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = vx
        out.twist.angular.z = wz
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySmootherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
