from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum


import rclpy
from rclpy.action import ActionServer
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from asmr_arm_interfaces.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from asmr_arm_interfaces.srv import ComputeFK
from asmr_arm_interfaces.srv import ComputeIK
from asmr_arm_interfaces.srv import BSService

import os
from ament_index_python.packages import get_package_share_directory
import yaml



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

class TrajectoryServer(Node): 
    DEBUG = False
    #====================================== get stuff from configs ======================================
    bringup_pkg = get_package_share_directory('asmr_arm_bringup')
    debug_file = os.path.join(bringup_pkg,'config', 'debug.yaml')
    with open (debug_file) as f:
        DEBUG = yaml.safe_load(f)["debug"]
    #====================================================================================================

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

        # Create service
        self._generate_bs_server = self.create_service(
            BSService, 
            'bs_service',
            self.bs_callback
        )

        # Create action server
        self._trajectory_server = ActionServer(
            self,
            ExecuteTrajectory,
            'execute_trajectory',
            self.execute_trajectory
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

    def joint_callback(self, msg):
        self.current_theta1 = msg.position[0]
        self.current_theta2 = msg.position[1]
        
    def bs_callback(self, request, response):
        x_interpolation, y_interpolation = zip(*(self._plan_trajectory((request.x, request.y))))
        response.x_coords = x_interpolation
        response.y_coords = y_interpolation
        self.get_logger().info("exiting bs_callback")


    def _plan_trajectory(self, end_pos: tuple[float, float]) -> list[tuple[float, float]]:
        """
        creates a linear trajectory between the end effector's current position and the desired end position.
        Returns: Array of interpolated points on trajectory as tuples
        """
        self.get_logger().info("Entered _plan_trajectory")
        future = self.send_fk_request(self.current_theta1, self.current_theta2)
        self.get_logger().info("request sent")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("finished spinning")
        response = future.result()
        current_pos = (response.x, response.y)
        x_interpolations = np.linspace(current_pos[0], end_pos[0], NO_INTERPOLATIONS)
        y_interpolations = np.linspace(current_pos[1], end_pos[1], NO_INTERPOLATIONS)
        trajectory_array = list(zip(x_interpolations, y_interpolations))
        self.get_logger().info("exiting _plan_trajectory")
        return trajectory_array

    async def execute_trajectory(self, goal_handle):
        result = ExecuteTrajectory.Result()
        self.get_logger().info("hello from execute_trajectory")
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y

        # check feasibility
        future = self.send_ik_request(goal_x, goal_y)
        await future
        if not future.result().success:
            goal_handle.abort()
            result.success = False
            result.message = "OUT OF REACH"
            return result

        goal = (goal_x, goal_y)
        trajectory = self._plan_trajectory(goal)

        feedback_msg = ExecuteTrajectory.Feedback()
        for i, waypoint in enumerate(trajectory):
            # check if goal is cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.cancelled()
                result.success = False
                return result
            
            # --- TODO ---
            # command arm to waypoint and wait for it to arrive
            
            # sending feedback
            feedback_msg.waypoint_index = i
            feedback_msg.ee_x = waypoint[0]
            feedback_msg.ee_y = waypoint[1]
            self.get_logger().info(f"Waypoint {i}: ({waypoint[0],waypoint[1]})")

        # return result
        goal_handle.succeed()
        result.success = True
        result.theta1 = self.current_theta1
        result.theta2 = self.current_theta2
        return result
            

        # self.get_logger().info('EXECUTING GOAL')
        # result = ExecuteTrajectory.Result()
        # return result

        


def main(args=None) -> None:
    rclpy.init(args=args)
    server = TrajectoryServer('trajectory_server')
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