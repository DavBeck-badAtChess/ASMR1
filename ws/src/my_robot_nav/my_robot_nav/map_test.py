import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np


class OccGrid(Node):
    def __init__(self, map_dims_in_meter : tuple[float, float]):
        '''
        this class will take a map, and display it.
        this is mainly for debug purpuses, but also because it is kinda cool
        '''

        self._map_dims_in_meter: tuple[float, float] = map_dims_in_meter
        super().__init__('map_test')
        self.pub = self.create_publisher(OccupancyGrid, 'map', 10)



    def display(self, tile_grid: np.ndarray):
        '''
        takes a map of the world, and displays it. thats it.
        '''
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.info.resolution = 1.0
        msg.info.width = self._map_dims_in_meter[0]
        msg.info.height = self._map_dims_in_meter[1]
        
        # i want it to be at the center
        msg.info.origin.position.x = self._map_dims_in_meter[0]/2
        msg.info.origin.position.y = self._map_dims_in_meter[1]/2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # grid = np.zeros((5, 5), dtype=np.int8)
        # grid[2, 2] = 100
        # grid[1, 3] = 50
        
        msg.data = tile_grid.flatten().tolist()
        
        self.pub.publish(msg)

