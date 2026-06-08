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
#import tf_transformations



class PointNavigator:

    MAX_LIN_SPEED:float = 0.5
    MAX_ROT_SPEED:float = 2.0
    CLOSNESS_THREASHOLD:float = 0.2

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
        '''
        self._current_waypoint: np.ndarray = None
        #self._current_waypoint_local: np.ndarray = None
        
        self._current_global_coord: np.ndarray = None
        self._current_heading: float = None 

        self._rot_acc:float = 1.0
        self._lin_acc:float = 0.0#0.2

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

    def _update_action_goal_dis(self):
        '''
        calculate the next direction, and based on that the sensible speed.
        if the to drive direction is orthogonal, then prioritize rotation, and slow down.

        '''
        favor_heading = np.arctan2(self._current_waypoint_local[0],self._current_waypoint_local[1])
        
        heading_closeness = max(np.cos(favor_heading - self._current_heading),0)

        rot_acc = (1 - heading_closeness) * self._rot_acc # if orth, rotate faster
        lin_acc = (heading_closeness-0.5) * self._lin_acc # if orth, slow down
        
        lin_speed = self._goal.linear_x + lin_acc

        diff = np.arctan2(
            np.sin(
                np.arctan2(
                    self._current_waypoint_local[1],
                    self._current_waypoint_local[0]
                ) - self._current_heading
            ),
            np.cos(
                np.arctan2(
                    self._current_waypoint_local[1],
                    self._current_waypoint_local[0]
                ) - self._current_heading
            )
        )
        if diff < 0:
            rot_speed = self._goal.angular_z - rot_acc
        else:
            rot_speed = self._goal.angular_z + rot_acc

        self._goal.angular_z = np.clip(rot_speed, -PointNavigator.MAX_ROT_SPEED, PointNavigator.MAX_ROT_SPEED)
        self._goal.linear_x = np.clip(lin_speed,0, PointNavigator.MAX_LIN_SPEED)


    def tick(self):
       
        '''
        update everything.
        first update the local tf
        if possible move on. update the lodal waypoint, use that to update the goal action, set the waypoint reached state
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



    @property
    def lidar_data_usable(self)->bool:
        if self._current_global_coord is None: return False
        if self._current_heading is None: return False
        return True

    @property
    def waypoint_reached(self)-> bool:
        '''
        return if the waypoint was reached
        '''
        return self._waypoint_reached


    @property
    def action_goal(self):
        return self._goal
    
    @property
    def current_global_coord_offset(self)-> np.ndarray:
        return self._current_global_coord
    
    @property
    def current_global_heading(self)-> float:
        return self._current_heading