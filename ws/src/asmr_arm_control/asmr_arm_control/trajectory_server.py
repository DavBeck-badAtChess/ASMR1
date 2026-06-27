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
from sensor_msgs.msg import JointState
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


NO_INTERPOLATIONS = 100 

class TajectoryServer(Node): 
    def __init__(self, name:str):
        super().__init__(name)
        # Variables
        self.fk_req = ComputeFK.Request()
        self.ik_req = ComputeIK.Request()
        self.current_theta1 = 0.0
        self.current_theta2 = 0.0

        # Create service clients
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

        # Create topic subscriptions
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        

        
    def send_fk_request(self, theta1, theta2):
        self.fk_req.theta1 = theta1
        self.fk_req.theta2 = theta2 
        return self._fk_client.call_async(self.fk_req)


    def send_ik_request(self, x, y):
        self.ik_req.x = x
        self.ik_req.y = y 
        return self._ik_client.call_async(self.ik_req)

    def _callback(self, request, response):
        pass


    def _plan_trajectory(self, end_pos: tuple[float, float]):
        self.get_logger().info("Entered _plan_trajectory")
        future = self.send_fk_request(self.current_theta1, self.current_theta2)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        current_pos = (response.x, response.y)
        x_interpolations = np.linspace(current_pos[0], end_pos[0], NO_INTERPOLATIONS)
        y_interpolations = np.linspace(current_pos[1], end_pos[1], NO_INTERPOLATIONS)
        trajectory_array = zip(x_interpolations, y_interpolations)
        self.get_logger().info("--------------------------------AKSDKFKSJPFJOWEJFWONOFJWOEKFPWJ--------------------------------")
        for i in trajectory_array:
            self.get_logger().info(str(i))



    def joint_callback(self, msg):
        self.current_theta1 = msg.position[0]
        self.current_theta2 = msg.position[1]
        


def main(args=None) -> None:
    rclpy.init(args=args)
    server = TajectoryServer('trajectory_server')
    server._plan_trajectory((0.3, 0.3))
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