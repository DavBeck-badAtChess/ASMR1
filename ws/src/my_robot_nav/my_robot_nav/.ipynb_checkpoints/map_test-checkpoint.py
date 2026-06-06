import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class OccGrid(Node):
    def __init__(self):
        super().__init__('map_test')
        self.pub = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.info.resolution = 1.0
        msg.info.width = 5
        msg.info.height = 5
        
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        grid = np.zeros((5, 5), dtype=np.int8)
        grid[2, 2] = 100
        grid[1, 3] = 50
        
        msg.data = grid.flatten().tolist()
        
        self.pub.publish(msg)

