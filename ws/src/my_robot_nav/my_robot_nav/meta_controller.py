from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum
from ASMR1.ws.src.my_robot_nav.my_robot_nav.debug_map import OccGrid

from my_robot_nav.maze_solver import Solver
from my_robot_nav.helper import Helper
from my_robot_nav.point_navigator import PointNavigator
from std_msgs.msg import Bool

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
    def __init__(self, name:str):
        super().__init__(name)
        '''
        innit all the necc stuff. 
        build and bind all subscriptions.
        '''
        self._goal_tile : int[int,int]  = None
        self._current_tile  :int[int,int] = None

        self._solver    : Solver     = None
        self._plotter   : OccGrid    = OccGrid(map_dims_in_meter=Helper.get_total_map_dim_in_meter())
        self._point_navigator   :PointNavigator  = PointNavigator()

        latched_qos_goal = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        latched_qos_reached = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # subscribe to goal
        self._goal_subscription = self.create_subscription(
            PointStamped,
            '/goal_point',
            self._on_goal_data,
            latched_qos_goal,
        )
        
        # subscribe to goal rec
        self._goal_msg_recieved:bool = False
        self._done:bool = False
        self._goal_reached_subscription = self.create_subscription(
            Bool,
            '/goal_reached',
            self._on_goal_reached_send,
            latched_qos_reached,
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
        self._innitial_scanning_flag: bool = False

        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')

        self._timer = self.create_timer(
            1 / MetaController.TICK_HZ, self._tick
        )
        self.get_logger().info('meta controller ready')


    def _on_goal_reached_send(self, msg):
        '''
        just set the state.
        '''
        if msg.data:
            self._goal_msg_recieved = True


    def _on_goal_data(self, msg: PointStamped):
        point_np = np.array([
            msg.point.x,
            msg.point.y,
            msg.point.z
        ], dtype=np.float64)
        self._goal_tile = Helper.world_to_tile_single(point_np)
        self._solver = Solver(maze_shape=Helper.get_world_arr_shape(), goal_tile= self._goal_tile)


    def _on_lidar_send(self, msg):
        '''
        just store the latest msg. this is then used in the tick
        '''
        self._latest_lidar_msg = msg


    def _send_action_goal(self):
        self._movement_client.send_goal_async(self._point_navigator.action_goal)


    def _on_odom_data(self, msg):
        self._latest_odom_msg = msg


    def _update_global_positions(self):
        pos_t = get_position(self._latest_odom_msg)
        self._robot_coord = np.array([pos_t[0], pos_t[1]])
        self._robot_heading = get_yaw(self._latest_odom_msg)


    def _synch_map(self):
        '''
        call this to synch the vis map.
        use up the flag here
        '''
        sm = self._solver.informational_map
        sm[self._current_tile] += 30
        self._plotter.display(sm.T)


    def _synch_env_with_lidar_data(self):
        '''
        take the lidar data, relan the course, and then determain the next goal.
        use the message up.
        '''
        msg = self._latest_lidar_msg
        self._solver.account_for_geometry(Helper.get_tiles_from_lidar_data_raw(raw_lidar_data=np.array(msg.ranges),
                                                                                current_coord= self._robot_coord,
                                                                                current_heading=self._robot_heading))


    @property
    def _ready_to_tick(self)-> bool:
        '''
        only run if:
            there is a goal
            the solver exists (really this is eq to the goal tile existing)
            there is a latest odom msg
            there is a latest lidar msg
        without these, there is nothing sensible todo really
        many things are assumed in order to tick. this is a quick debug ish function.
        '''
        if self._goal_tile is None: return False
        if self._solver is None: return False
        if self._latest_odom_msg is None: return False
        if self._latest_lidar_msg is None: return False
        if self._done: return False
        return True


    def _tick(self):
        '''
        here i need to define all the actions, that need to be done in one tick. this needs to be driven by a clock.

            - check if tick is runnable
            - check if the goal is reached 
            - update all global robot position using the latest odom msg
            - if lidar data is usable, update the map with that. this also sets the innitial scan flag
            - synch the visual map with the maze data
            - privide the current position states to the point navigator
            - figure out the next tile on the path, and set that as the next waypoint
            - if the innitial scan has been completed, tick the ppoint nav and send the action goal to the robot
            - reset the latest mesages to prevent usage asynch data
                (tecnically this could go wrong, since this means the controll relies on lidar and odom data arriving at the same 
                meta controller tick, but lidar publishes at 10 hz and odom at 30, so this works just fine in this case)
        '''
        if not self._ready_to_tick: return

        if self._goal_msg_recieved:
            '''
            kill the nav, send the final stop goal, print the reached signal, and set the done flag to early out in furhther iters.
            '''
            self._point_navigator.kill()
            self._send_action_goal()
            self.get_logger().info('ROBOT ARRIVED')
            self._done = True
            return

        self._update_global_positions()
        if self._point_navigator.lidar_data_usable:
            '''
            if the lidar data is usable, use it to update the map. also set the innitial scanning flag
            '''
            self._synch_env_with_lidar_data()
            self._innitial_scanning_flag = True
        
        self._synch_map()

        self._point_navigator.set_global_positions(global_pos=self._robot_coord, heading=self._robot_heading)
        self._current_tile = Helper.world_to_tile_single(self._robot_coord)
        self._current_tile = self._solver.get_next_tile(tile_position=self._current_tile)
        self._point_navigator.set_new_waypoint(waypoint= Helper.tile_to_world_single(self._current_tile))

        if self._innitial_scanning_flag:
            '''
            only start driving if an innitial scan has been made
            '''
            self._point_navigator.tick()
            self._send_action_goal()

        # this prevents old data from being used
        self._latest_lidar_msg = None
        self._latest_odom_msg = None




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
