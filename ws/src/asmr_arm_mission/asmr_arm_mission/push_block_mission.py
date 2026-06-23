import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class PushBlockMission(Node):
    def __init__(self):
        super().__init__('push_block_mission')

def main():
    rclpy.init()

    try:
        node = PushBlockMission()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()