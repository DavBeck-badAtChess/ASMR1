from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum


import rclpy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity



class PointNavigator:

    MAX_LIN_SPEED:float = 1.0
    MAX_ROT_SPEED:float = 1.0

    @property
    def _ready_to_tick(self)-> bool:
        '''
        many things are assumed in order to tick. this is a quick debug ish function 
        '''
        if self._globa_to_local_tf is None: return False
        if self._current_waypoint_local is None: return False
        return True


    def __init__(self):
        '''
        '''
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._current_waypoint: np.ndarray = None
        self._current_waypoint_local: np.ndarray = None
        self._current_coord_global: np.ndarray = None #TODO USE THIS LATER

        self._rot_acc:float = 0.5
        self._lin_acc:float = 0.5

        self._goal = SetVelocity.Goal()
        self._goal.linear_x = 0.0
        self._goal.angular_z = 0.0
        self._globa_to_local_tf = None

        self._waypoint_reached = False

        # self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')
        # while not self._movement_client.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info('service set velocity not available, waiting again...')


    def _check_if_waypoint_is_reached(self):
        self._waypoint_reached = np.linalg.norm(self._current_waypoint_local) < self.CLOSNESS_THREASHOLD


    def _update_local_waypoint(self):
        '''
        update the current local goal, assume that the transform is local.
        '''
        self._current_waypoint_local = self._make_local(self._current_waypoint)


    def _update_action_goal(self):
        '''
        calculate the next direction, and based on that the sensible speed.
        if the to drive direction is orthogonal, then prioritize rotation, and slow down.

        '''
        favor_heading = np.arctan2(self._current_waypoint_local[1],self._current_waypoint_local[0])
        current_heading = self._globa_to_local_tf.tf.transform.rotation.z
        heading_closeness = np.abs(np.cos(favor_heading - current_heading),0)

        rot_acc = (1 - heading_closeness) * self._rot_acc # if orth, rotate faster
        lin_acc = (heading_closeness-0.5) * self._lin_acc # if orth, slow down
        lin_speed = self._goal.linear_x + lin_acc

        if favor_heading < current_heading:
            rot_speed = self._goal.angular_z - rot_acc
        else:
            rot_speed = self._goal.angular_z + rot_acc

        self._goal.angular_z = np.clip(rot_speed, -PointNavigator.MAX_ROT_SPEED, PointNavigator.MAX_ROT_SPEED)
        self._goal.linear_x = np.clip(lin_speed,0, PointNavigator.MAX_LIN_SPEED)


    def tick(self):
        self.get_logger().info('tick point nav')
        '''
        update everything.
        first update the local tf
        if possible move on. update the lodal waypoint, use that to update the goal action, set the waypoint reached state
        '''
        self._update_globa_to_local_tf()
        if not self._ready_to_tick: self.get_logger().info('tf not ready') 
        if not self._ready_to_tick: return 

        self._update_local_waypoint()
        self._update_action_goal()
        self._send_action_goal()
        self._check_if_goal_is_reached()


    #def _send_action_goal(self):
    #    self._movement_client.send_goal_async(self._goal)


    def set_new_waypoint(self, waypoint:np.ndarray):
        '''
        set a new waypoint to drive to. no callback or anything
        '''
        self.get_logger().info('new waypoint')
        self._current_waypoint = waypoint
        self._waypoint_reached = False


    def kill(self):
        '''
        end it all, sent the final stop message
        '''


    # def _update_globa_to_local_tf(self):
    #     '''
    #     provides the tf to transform global into local
    #     '''
    #     self.get_logger().info('trying to innit tf')
    #     try:
    #         self._globa_to_local_tf = self._tf_buffer.lookup_transform(
    #             "odom",   # source frame
    #             "map",   # source frame
    #             #"base_link",   # source frame
    #              rclpy.time.Time(),   # latest available transform
    #             timeout=rclpy.duration.Duration(seconds=0.2)
    #         )
    #         self.get_logger().info('innited tf')
    #     except tf2_ros.LookupException:
    #         return None
        
    def set_globa_to_local_tf(self, tf ):
        self._globa_to_local_tf = tf 


    @property
    def waypoint_reached(self)-> bool:
        '''
        return if the waypoint was reached
        '''
        return self._waypoint_reached
    
    @property
    def action_goal(self):
        return self._goal