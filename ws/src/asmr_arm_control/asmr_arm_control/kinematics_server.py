from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 

import os

from ament_index_python.packages import get_package_share_directory
import yaml



import rclpy
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


''' IK
# Request: target end-effector position in the arm's planar frame (metres)
float64 x        # reach from the shoulder [m]
float64 y        # height relative to the shoulder [m]
---
# Response: joint angles (radians) that reach the target
float64 theta1   # shoulder joint angle [rad]
float64 theta2   # elbow joint angle [rad]
bool success     # false if the target is outside the workspace
''' 

class KinematicsServer(Node):
    CLOSENESS_TOLERANCE: float  = 0.01
    STEP_SIZE: float  = 0.01
    DEBUG = False
    bringup_pkg = get_package_share_directory('asmr_arm_bringup')
    debug_file = os.path.join(bringup_pkg,'config', 'debug.yaml')
    with open (debug_file) as f:
        DEBUG = yaml.safe_load(f)
    

    desc_pkg = get_package_share_directory('asmr_arm_description')
    dims_file = os.path.join(desc_pkg,'config', 'arm_dimensions_pedestal.yaml')
    arm_dims = None
    with open (dims_file) as f:
        arm_dims = yaml.safe_load(f)


    def __init__(self, name:str):
        super().__init__(name)
        self.l1 = 0.5
        self.l2 = 0.5
        self._fk_service = self.create_service(
            ComputeFK,
            "forward_kinematics",
            self._compute_forward_kinematics
        )

        self._ik_service = self.create_service(
            ComputeIK,
            "inverse_kinematics",
            self._compute_inverse_kinematics
        )
        self.get_logger().info("KinematicsServer::is now running"+30*"=")
        self.get_logger().info(f"{KinematicsServer.arm_dims}")
        self.get_logger().info(f"{KinematicsServer.DEBUG}")


    def _compute_inverse_kinematics(self, request, response):
        '''
        use the inverse kinematics from week 6
        '''
        target = np.array([request.x, request.y])
        if np.linalg.norm(target) > KinematicsServer.robot_arm_lengths[0] + KinematicsServer.robot_arm_lengths[1]:  # Check if target is within workspace
            response.success = False
            return response

        curr_theta = (0,0)
        curr_pos = self.forward_kinematics(curr_theta)
        
        while np.linalg.norm(target - np.asarray(curr_pos)) > KinematicsServer.CLOSENESS_TOLERANCE:
            J_inv = np.linalg.pinv(self.jacobian(curr_theta))
            curr_theta += KinematicsServer.STEP_SIZE*J_inv@(np.asarray(target) - np.asarray(curr_pos))
            curr_pos = self.forward_kinematics(curr_theta)
        response.theta1 = curr_theta[0]
        response.theta2 = curr_theta[1]
        response.success = True
        return response

    def _compute_inverse_kinematics_alternative(self, request, response):
        x = request.x
        y = request.y

        d = (np.square(x) + np.square(y) - np.square(self.l1) - np.square(self.l2))/(2 * self.l1 * self.l2)
        if abs(d) > 1:
            response.success = False
            return response
        theta2 = np.arctan2(np.sqrt(1-np.square(d)), d) # always use ellbow up
        theta1 = np.arctan2(y, x) - np.arctan2(self.l2 * np.sin(theta2), self.l1 + self.l2 * np.cos(theta2))
        response.theta1 = theta1
        response.theta2 = theta2
        response.success = True 
        return response

    def _compute_forward_kinematics(self, request, response):
        x = self.l1 * np.cos(request.theta1) + self.l2 * np.cos(request.theta1 + request.theta2)
        y = self.l1 * np.sin(request.theta1) + self.l2 * np.sin(request.theta1 + request.theta2)
        response.x = x
        response.y = y
        response.success = True
        return response


    def callback(self,request, response):
        
        self.get_logger().info('msg recieved')
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    server = KinematicsServer('kinematics_server')
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