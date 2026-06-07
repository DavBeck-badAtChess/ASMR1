from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum
from my_robot_nav.map_test import OccGrid

from my_robot_nav.maze_solver import Solver
from my_robot_nav.helper import Helper

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

    CLOSNESS_THRESHOLD = 0.2
    CLOSNESS_ang_renameme = 0.02
    MAX_SPEED = 5.0
    MAX_LIN_ACC = 0.5
    MAX_ROT_ACC = 0.5

    def __init__(self):
        super().__init__('meta_controller')
        '''
        build the solver to feed the 
        '''
        self._goal_tile : int[int,int] = (10,10)# i need the goal thingy to verify this
        self._solver:Solver = Solver(maze_shape=Helper.get_world_arr_shape(), goal_tile= self._goal_tile)
        self._plotter:OccGrid = OccGrid(map_dims_in_meter=Helper.get_total_map_dim_in_meter())
        self._current_tile: int[int,int] = Helper.get_starting_tile()

        # subcribe to lidar scan
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self._on_lidar_data, 
            10)
        self._lidar_subscription # prevent unused variable warning

        # waypoint navigation
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')

        self._current_waypoint: np.ndarray = None
        self._current_waypoint_local: np.ndarray = None
        self._current_heading: float = 0
        self._favor_heading : float = 0

        self._rot_pd:tuple[int,int] = (0.5,0.5)
        self._lin_pd:tuple[int,int] = (0.5,0.5)

        self._current_lin_acc:float = 0.5
        self._current_rot_acc:float = 0.0

        self._velocity_goal = SetVelocity.Goal()
        self._velocity_goal.linear_x = 0.0
        self._velocity_goal.angular_z = 0.0
        self._globa_to_local_tf = None

        self._waypoint_reached = True

        self._curr_callback:callable = None

        # connect to the movement client, and to the 
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')

        while self._globa_to_local_tf is None:
            self.get_logger().info('point_navigator not running ')
            # only start once the tf is usable
            self._update_globa_to_local_tf()
            rclpy.spin_once(self, timeout_sec=1.0)

        self._on_checkpoint_reached()   # initialize main loop
        # i = 0
        # while i < 5:
        #     rclpy.spin_once(self, timeout_sec=1.0)
        # # this will kick of the driving.
        # self._on_checkpoint_reached()
    
    def _on_checkpoint_reached(self):
        '''
        figure out the next step (this causes the update cascade in the maze).
        this is the loop that keeps everything running. so everything is run of the provided controllers clock.
        # TODO this should realy look two steps ahead...
        '''
        self._current_tile = (int(self._current_tile[0]), int(self._current_tile[1]))
        next_waypoint_tile: tuple[int,int] = self._solver.get_next_tile(tile_position=self._current_tile)
        next_waypoint = Helper.tile_to_world_single(tile=next_waypoint_tile)
        self._drive_to(waypoint=next_waypoint, callback= self._on_checkpoint_reached)

    def _replot_map(self):
        '''
        replot_map
        '''
        mc = self._solver.informational_map.copy()
       # mc[self._current_tile] = 2
        self._plotter.display(mc)

    def _on_lidar_data(self,msg):
        '''
        i assume, that everything that is seen here is not needed emidietly. ie the robot can see one tile ahead.
        '''
        self.get_logger().info(
         f"lidar data caught by meta controller"
        )
        new_information = self._solver.account_for_geometry(Helper.get_tiles_from_lidar_data_raw(raw_lidar_data=np.array(msg.ranges)))
        if new_information: self._replot_map()

    def _drive_to(self, coord:np.ndarray, callback:callable):
        '''
        set the state to goal not reached.
        since the privided controller will always call back at 10hrz, this will get picked up in the next update.
        '''
        self._current_waypoint = coord
        self._curr_callback = callback
        self._waypoint_reached = False
        # Initialize action call by starting action feedback loop:
        self._movement_client.send_goal_async(self._velocity_goal, feedback_callback=self._on_setvel_feedback)

    def _on_setvel_feedback(self,feedback_msg):
        '''
        goal is reached -> do nothing
        goal was reached this frame -> callback 
        the assigned states are not reached -> do nothing
        else get new states to assign and do so
        '''
        self.get_logger().info(
         f"velocity feedback loop"
        )
        if self._waypoint_reached: return
        self._update_headings()
        if self._check_if_waypoint_reached():
            self._waypoint_reached = True

            if not self._curr_callback is None:
                self._curr_callback()
            return
        fb = feedback_msg.feedback
        if np.abs(self._velocity_goal.linear_x - fb.current_linear_x) > MetaController.CLOSNESS_ang_renameme:
            return
        if np.abs(self._velocity_goal.angular_z - fb.current_angular_z) > MetaController.CLOSNESS_ang_renameme:
            return
        
        self._update_lin_acc()
        self._update_rot_acc()
        
        self._movement_client.send_goal_async(self._velocity_goal, feedback_callback=self._on_setvel_feedback)

    def _check_if_waypoint_reached(self)->bool:
        return np.linalg.norm(self._current_waypoint_local) < self.CLOSNESS_THRESHOLD

    def _update_headings(self):
        '''
        update the tf and all the stuff the others depend on 
        '''
        self._update_globa_to_local_tf()

        self._current_heading = self._get_global_heading()
        self._current_waypoint_local = self._make_local(self._current_waypoint)
        self._favor_heading = np.arctan2(self._current_waypoint_local[1],self._current_waypoint_local[0])

    def _get_global_heading(self)->float:
        return  self._globa_to_local_tf.transform.rotation.z

    def _update_lin_acc(self)->float:
        '''
        calculate the dot of the favor direction and the actual current direction, 
        use that as teh partial (close to 0 -> orth -> slow down visa versa).
        use the last adjustment (which should now be the current of the gaol since this is only triggered when that is the case) as the d term 
        no term for slowing down when the goal is close. (yet)
        '''
        next_acc = np.clip(np.cos(self._favor_heading-self._current_heading)*self._lin_pd[0] - self._current_rot_acc * self._lin_pd[1], - MetaController.MAX_LIN_ACC,MetaController.MAX_LIN_ACC)
        self._velocity_goal.linear_x += next_acc
        self._current_lin_acc = next_acc

    def _update_rot_acc(self)->float:
        '''
        the same thing as the linear acc.
        '''
        next_acc =np.clip(1-np.cos(self._favor_heading-self._current_heading)*self._rot_pd[0] - self._current_rot_acc * self._rot_pd[1], -MetaController.MAX_ROT_ACC,MetaController.MAX_ROT_ACC)
        self._velocity_goal.angular_z += next_acc
        self._current_rot_acc = next_acc


    def _update_globa_to_local_tf(self):
        '''
        provides the tf to transform global into local
        '''
        try:
            self._globa_to_local_tf = self._tf_buffer.lookup_transform(
                #"base_link",   # target frame
                "odom",   # source frame
                "map",   # source frame
                rclpy.time.Time()   # time
            )
        except tf2_ros.LookupException:
            return None

    def _make_local(self, global_point:np.ndarray)->np.ndarray:
        '''
        takes a global point, and converts into local coords.
        '''
        point_glob = PointStamped()
        point_glob.header.frame_id = "odom"  # this point is global
        point_glob.header.stamp = rclpy.time.Time() # right now
        point_glob.point.x = global_point[0]
        point_glob.point.y = global_point[1]
        point_glob.point.z = 0
        point_local = tf2_geometry_msgs.do_transform_point(point_glob, self._globa_to_local_tf)
        return np.array([point_local.point.x,point_local.point.y])

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

