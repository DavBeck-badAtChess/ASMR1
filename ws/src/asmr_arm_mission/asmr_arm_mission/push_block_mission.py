import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np 
from asmr_arm_interfaces.action import ExecuteTrajectory
from rclpy.action import ActionClient

import os
from ament_index_python.packages import get_package_share_directory
import yaml

class PushBlockMission(Node):
    DEBUG = False
    #====================================== get stuff from configs ======================================
    bringup_pkg = get_package_share_directory('asmr_arm_bringup')
    debug_file = os.path.join(bringup_pkg,'config', 'debug.yaml')
    with open (debug_file) as f:
        DEBUG = yaml.safe_load(f)["mission"]
    #====================================================================================================

    def __init__(self):
        super().__init__('push_block_mission')
        # start -> drive up -> lower -> push -> drive back a bit
        self._waypoints: list[tuple[int,int]] = [(1,0), (0.3,0.5), (0.3,0.0), (0.9,0.0), (0.3,0.0)]

        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        if not self._execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('trajectory client not avalable, trying again')
        if self.__class__.DEBUG: self.get_logger().info(f"{self.__class__}:: now running"+30*"=")


    def execute_mission(self):
        if self.__class__.DEBUG: self.get_logger().info(f"{self.__class__}::mission started o7")
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.x = [point[0] for point in self._waypoints]
        goal_msg.y = [point[1] for point in self._waypoints]

        resp = self._execute_trajectory_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        resp.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"now at point: {feedback.waypoint_index}\n eex: {feedback.ee_x:.2f}\n eey: {feedback.ee_y:.2f}\n") 


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal not reached ;-;")
            return
        self.get_logger().info("goal reached \o/")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"succseed with:\n theta1 {result.theta1} \n theta1 {result.theta2}")
        rclpy.shutdown()



def main(args=None) -> None:
    rclpy.init(args=args)

    node = PushBlockMission()
    node.execute_mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 