from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped



class CorridorMission(Node):
    def __init__(self, name:str):
        super().__init__(name)
        self.msg = TwistStamped()


        self._twist_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_smoothed',
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def send_twist(self, linear_x: float, angular_z: float):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.twist.linear.x = linear_x
        self.msg.twist.angular.z = angular_z

        self._twist_publisher.publish(self.msg)

            
    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.5
        msg.twist.angular.z = 0.0
        
        self._twist_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    node = CorridorMission("corridor_mission")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()