from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum


import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from asmr_arm_interfaces.action import ExecuteTrajectory
from asmr_arm_interfaces.srv import ComputeFK
from asmr_arm_interfaces.srv import ComputeIK


''' FK
# Request: joint angles (radians)
float64 theta1   # shoulder joint angle [rad]
float64 theta2   # elbow joint angle [rad]
---
# Response: end-effector position in the arm's planar frame (metres)
float64 x        # reach from the shoulder [m]
float64 y        # height relative to the shoulder [m]
bool success     # true if the request was valid
'''




class TajectoryServer(Node):    
    def __init__(self, name:str):
        super().__init__(name)
        self._fk_client = self.create_client(
            ComputeFK,
            "forward_kinematics",
        )
        self._ik_client = self.create_client(
            ComputeIK,
            "inverse_kinematics",
        )

        while not self._fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("FK server not available, trying again")
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK server not available, trying again")
        self.get_logger().info("Successfully connected to FK & IK servers")
        self.req = ComputeFK.Request()
        

        
    def send_fk_request(self, theta1, theta2):
        self.req.theta1 = theta1
        self.req.theta2 = theta2 
        return self._fk_client.call_async(self.req)

    def _callback(self, request, response):
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    server = TajectoryServer('trajectory_server')
    future = server.send_fk_request(float(np.pi * 0.5), 0.0)
    rclpy.spin_until_future_complete(server, future)
    response = future.result()
    server.get_logger().info(
        f"theta1={response.x} ({type(response.x)})"
        f"theta2={response.y} ({type(response.y)})"
    )
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()