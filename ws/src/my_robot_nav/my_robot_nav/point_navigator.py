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
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity

class PointNavigator(Node):
    '''
    this node is responsible, for driving the robot, do a designated point.
    it subscirbes odom, and uses the provided controller to controll the robot.
    '''

    def __init__(self):
        super().__init__('point_navigator')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')

        # connect to the movement client, and to the 
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')
        
        

        # this is how to use the action
        # g = SetVelocity.Goal()
        # g.linear_x = 1.0
        # g.angular_z = 0.0
        # for i in range(10):
        #     self._movement_client.send_goal_async(g)
        #     rclpy.spin_once(self, timeout_sec=_TICK_SEC)


        while True:
            rclpy.spin_once(self, timeout_sec=1.0)
            try:
                tf = self._tf_buffer.lookup_transform(
                    "odom",   # target frame
                    "base_link",   # source frame
                    rclpy.time.Time()   # time
                )
                self.get_logger().info(tf)
            except tf2_ros.LookupException:
                self.get_logger().info("exeption called :(")


def main(args=None) -> None:
    rclpy.init(args=args)

    point_navigator = PointNavigator()

    rclpy.spin(point_navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


