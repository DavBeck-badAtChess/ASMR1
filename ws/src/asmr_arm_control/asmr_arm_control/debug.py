import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class DebugArmPublisher(Node):
    def __init__(self):
        super().__init__('debug_arm_pub')

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/arm_pid_controller/reference',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_cmd)  # 10 Hz

    def publish_cmd(self):
        msg = Float64MultiArray()

        # DEBUG VALUES (rad)
        msg.data = [
            0.0,   # joint 1
            0.5,   # joint 2
            -0.3,  # joint 3
            1.0,   # joint 4
        ]

        self.pub.publish(msg)
        self.get_logger().info(f'Publishing θ_ref: {msg.data}')


def main():
    rclpy.init()
    node = DebugArmPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()