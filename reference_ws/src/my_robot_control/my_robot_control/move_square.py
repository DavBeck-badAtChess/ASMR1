import time

import rclpy
from geometry_msgs.msg import Twist

from my_robot_control.movement import drive, stop


def main():
    rclpy.init()
    node = rclpy.create_node('move_square')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    time.sleep(2.0)

    node.get_logger().info('Starting square trajectory (open-loop)')

    for i in range(4):
        node.get_logger().info(f'Side {i + 1}: forward')
        drive(pub, 0.3, 0.0, 2.0)

        node.get_logger().info(f'Side {i + 1}: turning')
        drive(pub, 0.0, 0.785, 2.0)

    stop(pub)
    node.get_logger().info('Trajectory complete — observe drift from start position')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
