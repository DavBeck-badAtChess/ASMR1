from __future__ import annotations
import sys
import time
sys.dont_write_bytecode = True
import numpy as np 
import math
from enum import Enum
from rclpy.qos import QoSProfile, DurabilityPolicy

import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from my_robot_perception.odom_utils import get_position, get_yaw
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity


class MetaController(Node):

    K_ATT = 0.5
    K_REP = 0.5
    INFLUENCE_DISTANCE = 2.0
    MAX_LINEAR = 0.5
    MAX_ANGULAR = 1.0

    '''
    this is where the action happens. 
    this listens to all the nodes of interest, and commands the output (ie uses nodes to manipulate).
    this is does not directly send any signals, it is just a controller.
    '''

    def __init__(self):
        super().__init__('meta_controller')
        '''
        build the solver to feed the 
        '''
        self._robot_pos: tuple = None
        self._goal_pos: tuple = None
        self._robot_yaw: float = None
        self._att_force = (0, 0)
        self._rep_force = (0, 0)

        time.sleep(10.0) # waiting for rviz and gazebo to load
        # subscribing to lidar
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._on_lidar_data,
            10
        )

        # subscribing to goal_point
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._goal_point_subscription = self.create_subscription(
            PointStamped,
            '/goal_point',
            self._on_goal_data,
            latched_qos
        )

        # creating client for set_velocity
        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')

        # msg = SetVelocity.Goal()
        # msg.linear_x = 0.5
        # msg.angular_z = 0.0

        # self._movement_client.send_goal_async(msg, self._temporary_feedback_function)

        # subscribe to /odom
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom_data,
            10
        )
        
    def _on_odom_data(self, msg):
        self._robot_pos = get_position(msg)
        self._robot_yaw = get_yaw(msg)
        if self._goal_pos != None:
            dx = (self._goal_pos[0] - self._robot_pos[0])
            dy = (self._goal_pos[1] - self._robot_pos[1])
            distance = math.sqrt(dx**2 + dy**2)
            self._att_force = (
                self.K_ATT * dx / distance,
                self.K_ATT * dy / distance
            )
            total_force = (
                self._att_force[0] + self._rep_force[0],
                self._att_force[1] + self._rep_force[1]
            )
            set_vel_msg = self._force_to_velocity(total_force[0], total_force[1], self._robot_yaw)
            self._movement_client.send_goal_async(set_vel_msg, self._temporary_feedback_function)
            self.get_logger().info(f"set force to  {self._att_force}")

    def _on_lidar_data(self, msg):
        # raw_lidar_data = np.array(msg.ranges)
        # self.get_logger().info('getting lidar data')
        fx_robot, fy_robot = 0.0, 0.0
        for i, distance in enumerate(msg.ranges):
            if not (msg.range_min < distance < msg. range_max):
                continue
            if distance > self.INFLUENCE_DISTANCE:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            magnitude = self.K_REP * (1.0/distance - 1.0/self.INFLUENCE_DISTANCE) * (1.0/distance**2)

            fx_robot -= magnitude * math.cos(angle)
            fy_robot -= magnitude * math.sin(angle)

        fx, fy = 0.0, 0.0
        if self._robot_yaw is not None:
            cos_yaw = math.cos(self._robot_yaw)
            sin_yaw = math.sin(self._robot_yaw)

            # rotate into world frame
            fx = fx_robot * cos_yaw - fy_robot * sin_yaw
            fy = fx_robot * sin_yaw + fy_robot * cos_yaw

        self._rep_force = (fx, fy)
                

    def _on_goal_data(self, msg):
        self._goal_pos = point_stamp_to_coordinate(msg)
        self.get_logger().debug(f'Postition of goal set to: {self._goal_pos}')

    def _temporary_feedback_function(self, feedback_msg):
        # self.get_logger().info('calling  the set_velocity-server')
        pass

    def _force_to_velocity(self, fx, fy, robot_yaw):
        target_angle = math.atan2(fy, fx)

        error_angle = target_angle - robot_yaw

        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

        magnitude = math.sqrt(fx**2 + fy**2)

        goal = SetVelocity.Goal()
        goal.linear_x = min(magnitude * 0.5, self.MAX_LINEAR)
        goal.angular_z = max(-self.MAX_ANGULAR, min(error_angle * 1.0, self.MAX_ANGULAR))
        return goal

def point_stamp_to_coordinate(msg) -> tuple:
    """Return (x, y) position from a PointStamp message."""
    return (msg.point.x, msg.point.y)


def main(args=None) -> None:
    rclpy.init(args=args)
    meta_c = MetaController()
    rclpy.spin(meta_c)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

if __name__ == '__main__':
    main()
