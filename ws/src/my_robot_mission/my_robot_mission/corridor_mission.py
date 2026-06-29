from rclpy.node import Node
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

K_P = 10.0
K_D = 0.7
SPEED = 0.2

class CorridorMission(Node):
    def __init__(self, name:str):
        super().__init__(name)
        self.msg = TwistStamped()
        self.f_left = 0
        self.f_right = 0
        self.prev_error = 0
        self.prev_time = self.get_clock().now()

        self._twist_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel_smoothed',
            10
        )

        self.scan_left_subscrption = self.create_subscription(
            LaserScan,
            '/scan_left',
            self.left_scan_callback,
            10
        )
        self.scan_right_subscrption = self.create_subscription(
            LaserScan,
            '/scan_right',
            self.right_scan_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def send_twist(self, linear_x: float, angular_z: float):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.twist.linear.x = linear_x
        self.msg.twist.angular.z = angular_z

        self._twist_publisher.publish(self.msg)

    def left_scan_callback(self, msg):
        self.f_left = np.sum(msg.ranges) / (2 * np.pi /msg.angle_increment)
        # self.get_logger().info(f"f_left = {self.f_left}")

    def right_scan_callback(self, msg):
        self.f_right = np.sum(msg.ranges) / (2 * np.pi /msg.angle_increment)
        # self.get_logger().info(f"f_right = {self.f_right}")

    def pd_controller(self):
        p = self.f_left - self.f_right
        self.get_logger().info(f"p = {p}")
        current_time = self.get_clock().now() 
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if (dt <= 0):
            return
        d = (p - self.prev_error) / dt
        # self.get_logger().info(f"d = {d}")
        angular_z = K_P * p + K_D * d
        self.get_logger().info(f"angular_z = {angular_z}")
        self.prev_error = p
        self.prev_time = current_time
        self.send_twist(SPEED, angular_z)

            
    def timer_callback(self):
        # msg = TwistStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.twist.linear.x = SPEED
        # msg.twist.angular.z = 0.0
        
        # self._twist_publisher.publish(msg)
        self.pd_controller()


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