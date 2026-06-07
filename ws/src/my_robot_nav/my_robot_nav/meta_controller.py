from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum

import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity


class MetaController(Node):
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

        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self._on_lidar_data,
            10
        )

    def _on_lidar_data(self, msg):
        self.get_logger().info(
            'lidar data caught by meta controller'
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    meta_c = MetaController()
    rclpy.spin(meta_c)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

if __name__ == '__main__':
    main()
