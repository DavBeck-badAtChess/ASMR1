import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np 
from asmr_arm_interfaces.action import ExecuteTrajectory
from rclpy.action import ActionClient
from asmr_arm_interfaces.srv import BSService


class PushBlockMission(Node):
    def __init__(self):
        super().__init__('push_block_mission')

        self._execute_trajectory_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        self._bs_client = self.create_client(BSService, 'bs_service')
        while not self._bs_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('bs_service not avalable, trying again')
        self.bs_req = BSService.Request()

    def send_bs_request(self, goal_pos: tuple[float, float]):
        self.bs_req.x = goal_pos[0]
        self.bs_req.y = goal_pos[1]
        return self._bs_client.call_async(self.bs_req)



    def send_goal(self, order: tuple[float, float]):
        future = self.send_bs_request(order)
        rclpy.spin_until_future_complete(self, future)
        x_interpolation = future.result().x_coords
        y_interpolation = future.result().y_coords
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.x = list(x_interpolation)
        goal_msg.y = list(y_interpolation)

        # self._execute_trajectory_client.send_goal_async(goal_msg)

def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = PushBlockMission()
        future = node.send_goal((0.2, 0.2))
        rclpy.spin_until_future_complete(node, future)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()