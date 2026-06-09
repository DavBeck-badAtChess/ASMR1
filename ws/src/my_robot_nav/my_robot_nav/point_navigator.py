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

    MAX_LIN_SPEED:float = 0.3
    MAX_ROT_SPEED:float = 2.0
    CLOSNESS_THREASHOLD:float = 0.3
    '''
    if the current rotation rate is to high, lidar data is likely to be offset significantly.
    i do not want to limit the rotation rate of the robot to much though,
    to resolve this conflict the lidar data will only be used if the current rotation rate is below this threashold
    '''
    ROTATION__LIDAR_TH:float = 0.25

    @property
    def _ready_to_tick(self)-> bool:
        '''
        many things are assumed in order to tick. this is a quick debug ish function 
        '''
        if self._current_waypoint is None: return False
        if self._current_global_coord is None: return False
        if self._current_heading is None: return False
        return True



    def __init__(self):
        '''
        innit all the necc stuff.
        '''
        self._current_waypoint: np.ndarray = None
        
        self._current_global_coord: np.ndarray = None
        self._current_heading: float = None 

        self._rot_acc:float = 2.0

        self._goal = SetVelocity.Goal()
        self._goal.linear_x = 0.0
        self._goal.angular_z = 0.0

        self._waypoint_reached = True

    def set_global_positions(self, global_pos: np.ndarray, heading:float):
        self._current_global_coord = global_pos
        self._current_heading = heading

    def _check_if_waypoint_is_reached(self):
        robot_to_waypoint = self._current_waypoint - self._current_global_coord
        self._waypoint_reached = np.linalg.norm(robot_to_waypoint) < PointNavigator.CLOSNESS_THREASHOLD


    def _update_action_goal(self):
        '''
        use the current state info to update the action goal
        '''

        robot_to_waypoint = self._current_waypoint - self._current_global_coord
        target_heading = np.arctan2(
            robot_to_waypoint[1],
            robot_to_waypoint[0]
        )

        heading_error = np.arctan2(
            np.sin(target_heading - self._current_heading),
            np.cos(target_heading - self._current_heading)
        )

        rot_speed = np.clip(
            heading_error * self._rot_acc,
            -PointNavigator.MAX_ROT_SPEED,
            PointNavigator.MAX_ROT_SPEED
        )
        heading_factor = max(np.cos(heading_error), 0.0)

        lin_speed = np.clip(
            heading_factor * PointNavigator.MAX_LIN_SPEED,
            0.0,
            PointNavigator.MAX_LIN_SPEED
        )

        self._goal.angular_z = rot_speed
        self._goal.linear_x = lin_speed


    def tick(self):
       
        '''
        update the action goal, and check if the waypoint is set
        '''
       
        if not self._ready_to_tick: return 
        self._update_action_goal()
        self._check_if_waypoint_is_reached()


    def set_new_waypoint(self, waypoint:np.ndarray):
        '''
        set a new waypoint to drive to. no callback or anything
        '''
        self._current_waypoint = waypoint
        self._waypoint_reached = False


    def kill(self):
        '''
        end it all, sent the final stop message
        '''
        self._goal.linear_x = 0.0
        self._goal.angular_z = 0.0


    @property
    def lidar_data_usable(self)->bool:
        if self._current_global_coord is None: return False
        if self._current_heading is None: return False
        if abs(self._goal.angular_z) > PointNavigator.ROTATION__LIDAR_TH: return False
        return True


    @property
    def action_goal(self):
        return self._goal
