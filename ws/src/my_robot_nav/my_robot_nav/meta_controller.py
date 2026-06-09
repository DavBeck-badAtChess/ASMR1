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
from nav_msgs.msg import Odometry
from my_robot_perception.odom_utils import get_position, get_yaw
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

class MetaController(Node):
    '''
    this is where the action happens. 
    this listens to all the nodes of interest, and commands the output (ie uses nodes to manipulate).
    this is does not directly send any signals, it is just a controller.
    '''
    TICK_HZ = 10.00
    ROTATION_TH = 0.3

    def __init__(self, name:str):
        super().__init__(name)
        self._goal_tile : int[int,int]  =None# Helper.world_to_tile_single(np.array([5,5]))# TODO i need the goal thingy to verify this
        self._current_tile  :int[int,int]    = Helper.get_starting_tile()

        self._solver    : Solver     = None #Solver(maze_shape=Helper.get_world_arr_shape(), goal_tile= self._goal_tile)
        self._plotter   : OccGrid    = OccGrid(map_dims_in_meter=Helper.get_total_map_dim_in_meter())
        self._point_navigator   :PointNavigator  = PointNavigator()

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # subscribe to goal
        self._goal_msg_recieved:bool = False
        self._goal_subscription = None
        self._goal_subscription = self.create_subscription(
            PointStamped,
            '/goal_point',
            self._on_goal_data,
            latched_qos,
        )

        # subscribe to lidar
        self._latest_lidar_msg = None
        self._lidar_subscription = None
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self._on_lidar_send, 
            5)

        # subscribe to /odom
        self._latest_odom_msg = None
        self._odom_subscription = None
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._on_odom_data,
            10
        )
        self._heading_glob:float = None
        self._robot_coord:np.ndarray = None

        self._replot_flag : bool = True
        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')

        self._timer = self.create_timer(
            1 / MetaController.TICK_HZ, self._tick
        )
        self.get_logger().info('meta controller ready')

    
    # movement node stuff ======================================================================================================
    def _send_action_goal(self):
        self._movement_client.send_goal_async(self._point_navigator.action_goal)


    def _on_odom_data(self, msg):
        self._latest_odom_msg = msg
    
    def _update_global_positions(self):
        pos_t = get_position(self._latest_odom_msg)
        self._robot_coord = np.array([pos_t[0], pos_t[1]])
        self._robot_heading = get_yaw(self._latest_odom_msg)


    # goal node stuff ======================================================================================================
    def _on_goal_reached_send(self, msg):
        '''
        just set the state.
        '''
        self._goal_msg_recieved = True


    def _on_goal_data(self, msg: PointStamped):
        if not self._goal_tile is None: return
        point_np = np.array([
            msg.point.x,
            msg.point.y,
            msg.point.z
        ], dtype=np.float64)
        self._goal_tile = Helper.world_to_tile_single(point_np)
        self._solver = Solver(maze_shape=Helper.get_world_arr_shape(), goal_tile= self._goal_tile)


    # lidar stuff ======================================================================================================


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
        #if not self._replot_flag: return
        sm = self._solver.informational_map
        
        
        sm[self._current_tile] += 10
            


        sm[self._current_tile] += 10
        self._plotter.display(sm.T)
        self._replot_flag = False


    def _synch_env_with_lidar_data(self):
        '''
        take the lidar data, relan the course, and then determain the next goal.
        use the message up.
        '''
        if self._latest_lidar_msg is None: return
        msg = self._latest_lidar_msg
        self._replot_flag = self._solver.account_for_geometry(Helper.get_tiles_from_lidar_data_raw(raw_lidar_data=np.array(msg.ranges),
                                                                                                   current_coord= self._robot_coord,
                                                                                                   current_heading=self._robot_heading))


    # tick stuff ======================================================================================================
    @property
    def _ready_to_tick(self)-> bool:
        '''
        many things are assumed in order to tick. this is a quick debug ish function 
        '''
        if self._goal_tile is None: return False
        if self._current_tile is None: return False
        if self._solver is None: return False
        if self._latest_odom_msg is None: return False
        if self._latest_lidar_msg is None: return False
        return True


    def _tick(self):
        '''
        here i need to define all the actions, that need to be done in one tick. this needs to be driven by a clock.
        '''
        self.get_logger().info('tick')
        if not self._ready_to_tick: return

        if self._goal_msg_recieved:
            self._point_navigator.kill()
            return

        self._update_global_positions()
        if self._point_navigator.lidar_data_usable:
            if abs(self._point_navigator.agnular_z) < MetaController.ROTATION_TH:
                '''
                naive check to make shure the current angular v is not to high. this would lead to the lidar/odom connecton to be rubish
                '''
                self._synch_env_with_lidar_data()
        
        self._synch_map()

        #self._update_globa_to_local_tf_of_point_nav()
        
        self._point_navigator.set_global_positions(global_pos=self._robot_coord, heading=self._robot_heading)
        #if self._point_navigator.waypoint_reached:
        self._current_tile = Helper.world_to_tile_single(self._robot_coord)
        self._current_tile = self._solver.get_next_tile(tile_position=self._current_tile)
        self._point_navigator.set_new_waypoint(waypoint= Helper.tile_to_world_single(self._current_tile))
        self._replot_flag = True

        self._point_navigator.tick()
        self._send_action_goal()

        # this prevents old data from being used
        self._latest_lidar_msg = None
        self._point_navigator._goal



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
