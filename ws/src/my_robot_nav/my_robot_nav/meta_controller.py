from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum
from my_robot_nav.map_test import OccGrid

from my_robot_nav.maze_solver import Solver
from my_robot_nav.helper import Helper
from my_robot_nav.point_navigator import PointNavigator


import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity


class MetaController(Node):
    '''
    this is where the action happens. 
    this listens to all the nodes of interest, and commands the output (ie uses nodes to manipulate).
    this is does not directly send any signals, it is just a controller.
    '''

    def __init__(self):
        super().__init__('meta_controller')
        '''
        build the solver to feed the 
        '''
        self._goal_tile : int[int,int] = (10,10)# i need the goal thingy to verify this
        self._solver:Solver = Solver(maze_shape=Helper.get_world_arr_shape(), goal_tile= self._goal_tile)
        self._plotter:OccGrid = OccGrid(map_dims_in_meter=Helper.get_total_map_dim_in_meter())
        self._point_navigator:PointNavigator = PointNavigator()
        self._current_tile: int[int,int] = Helper.get_starting_tile()


        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self._on_lidar_data, 
            10)
        self._lidar_subscription # prevent unused variable warning

        i = 0
        while i < 5:
            rclpy.spin_once(self, timeout_sec=1.0)
        # this will kick of the driving.
        self._on_checkpoint_reached()

    def _on_goal_reached(self):
        self._point_navigator.kill()
    
    def _on_checkpoint_reached(self):
        '''
        figure out the next step (this causes the update cascade in the maze).
        this is the loop that keeps everything running. so everything is run of the provided controllers clock.
        # TODO this should realy look two steps ahead...
        '''
        next_waypoint_tile: tuple[int,int] = self._solver.get_next_tile(tile_position=self._current_tile)
        next_waypoint = Helper.tile_to_world_single(tile=next_waypoint_tile)
        self._point_navigator.drive_to(waypoint=next_waypoint, callback= self._on_checkpoint_reached)

    def _replot_map(self):
        '''
        replot_map
        '''
        self._plotter.display(self._solver.informational_map)

    def _on_lidar_data(self,msg):
        '''
        i assume, that everything that is seen here is not needed emidietly. ie the robot can see one tile ahead.
        '''
        new_information = self._solver.account_for_geometry(Helper.get_mask_from_lidar_data_raw(raw_lidar_data=np.array(msg.ranges)))
        if new_information: self._replot_map()



def main(args=None) -> None:
    rclpy.init(args=args)
    meta_c = MetaController()
    rclpy.spin(meta_c)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    meta_c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
