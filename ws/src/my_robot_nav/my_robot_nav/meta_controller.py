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
from rclpy.executors import MultiThreadedExecutor

class MetaController(Node):
    '''
    this is where the action happens. 
    this listens to all the nodes of interest, and commands the output (ie uses nodes to manipulate).
    this is does not directly send any signals, it is just a controller.
    '''
    TICK_HZ = 10.0

    def __init__(self, name:str):
        super().__init__(name)
        self.get_logger().info('meta_controller innit')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)


        self._goal_tile : int[int,int]  = (10,10)# TODO i need the goal thingy to verify this
        self._current_tile  :int[int,int]    = Helper.get_starting_tile()

        self._solver    : Solver     = Solver(maze_shape=Helper.get_world_arr_shape(), goal_tile= self._goal_tile)
        self._plotter   : OccGrid    = OccGrid(map_dims_in_meter=Helper.get_total_map_dim_in_meter())
        self.get_logger().info('before point nav')
        self._point_navigator   :PointNavigator  = PointNavigator()
        self.get_logger().info('after point nav')

        self._goal_msg_recieved:bool = False
        self._goal_subscription = None
        self._create_goal_sub()

        self._latest_lidar_msg = None
        self._lidar_subscription = None
        self.get_logger().info('before lidar sub')
        self._create_lidar_sub()

        self._replot_flag : bool = True

        # movement ------------------------
        
        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')

        self._timer = self.create_timer(
            1.0 / MetaController.TICK_HZ, self._tick
        )

    
    # movement node stuff ======================================================================================================
    def _send_action_goal(self):
        self._movement_client.send_goal_async(self._point_navigator.action_goal)

    # goal node stuff ======================================================================================================
    def _on_goal_reached_send(self, msg):
        '''
        just set the state.
        '''
        self._goal_msg_recieved = True

    def _update_globa_to_local_tf_of_point_nav(self):
        '''
        provides the tf to transform global into local
        '''
        self.get_logger().info('trying to innit tf')
        try:
           tf = self._tf_buffer.lookup_transform(
               "odom",   # source frame
               "map",   # source frame
               #"base_link",   # source frame
                rclpy.time.Time(),   # latest available transform
               timeout=rclpy.duration.Duration(seconds=0.2)
           )
           self._point_navigator.set_globa_to_local_tf(tf)
        except tf2_ros.LookupException:
            return None
        
    
    def _create_goal_sub(self):
        '''
        TODO
        '''
        pass

    # lidar stuff ======================================================================================================
    def _create_lidar_sub(self):
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self._on_lidar_send, 
            5)


    def _on_lidar_send(self, msg):
        '''
        just store the latest msg. this is then used in the tick
        '''
        self._latest_lidar_msg = msg


    def _synch_map(self):
        '''
        call this to synch the vis map.
        use up the flag here
        '''
        if not self._replot_flag: return
        mc = self._solver.informational_map.copy()
       # mc[self._current_tile] = 2
        self._plotter.display(mc)
        self._replot_flag = False


    def _synch_env_with_lidar_data(self):
        '''
        take the lidar data, relan the course, and then determain the next goal.
        use the message up.
        '''
        if self._latest_lidar_msg is None: return
        msg = self._latest_lidar_msg
        self._replot_flag = self._solver.account_for_geometry(Helper.get_tiles_from_lidar_data_raw(raw_lidar_data=np.array(msg.ranges)))
        self._latest_lidar_msg = None


    # tick stuff ======================================================================================================
    @property
    def _ready_to_tick(self)-> bool:
        '''
        many things are assumed in order to tick. this is a quick debug ish function 
        '''
        self.get_logger().info(f'current tile = {self._current_tile}')
        self.get_logger().info(f'goal tile = {self._goal_tile}')
        if self._goal_tile is None: return False
        if self._current_tile is None: return False
        return True

    def _tick(self):
        '''
        here i need to define all the actions, that need to be done in one tick. this needs to be driven by a clock.
        '''
        self.get_logger().info('tick')
        if not self._ready_to_tick: return

        if self._goal_msg_recieved:
            self._point_navigator.kill()
            self.get_logger().info('goal rec')
            return

        self._synch_env_with_lidar_data()
        self._synch_map()

        self._update_globa_to_local_tf_of_point_nav()
        if self._point_navigator.waypoint_reached:
            self._current_tile = self._solver.get_next_tile(tile_position=self._current_tile)
            self.get_logger().info(f'current tile = {self._current_tile}')
            self._point_navigator.drive_to(waypoint= Helper.tile_to_world_single(self._current_tile))
            self._replot_flag = True

        self._point_navigator.tick()
        self._send_action_goal()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetaController('meta_controller')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
