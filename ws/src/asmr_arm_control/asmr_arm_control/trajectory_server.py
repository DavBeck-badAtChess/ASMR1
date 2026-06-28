from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum
import asyncio
import time

import rclpy
from rclpy.action import ActionServer
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from asmr_arm_interfaces.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from control_msgs.msg import MultiDOFCommand
from asmr_arm_interfaces.srv import ComputeFK
from asmr_arm_interfaces.srv import ComputeIK
from asmr_arm_interfaces.srv import BSService



import os
from ament_index_python.packages import get_package_share_directory
import yaml



''' FK
# Request: joint angles (radians)
float64 theta1   # shoulder joint angle [rad]
float64 theta2   # elbow joint angle [rad]
---
# Response: end-effector position in the arm's planar frame (metres)
float64 x        # reach from the shoulder [m]
float64 y        # height relative to the shoulder [m]
bool success     # true if the request was valid
'''

'''
# Goal: sequence of end-effector waypoints in the arm's planar frame (metres)
float64 x      # reach from the shoulder, per waypoint [m]
float64 y      # height relative to the shoulder, per waypoint [m]
---
# Result
bool success     # true if every waypoint was reached
float64 theta1   # final shoulder angle [rad]
float64 theta2   # final elbow angle [rad]
---
# Feedback: published as each waypoint is reached
int32 waypoint_index   # index of the waypoint just reached
float64 ee_x           # achieved end-effector reach [m]
float64 ee_y           # achieved end-effector height [m]
'''

'''
ros2 topic pub /arm_pid_controller/reference control_msgs/msg/MultiDOFCommand "
dof_names:
  - elbow
  - shoulder
values:
  - 2.0
  - 2.0
values_dot:
  - 0.0
  - 0.0
"
'''


class Trajectory:
    CLOSENESS_THREASHOLD:float = 0.2
    SMOOTHNESS:float = 10.0
    @staticmethod
    def _generate_path(start_wooldcoord:np.ndarray, goal_wooldcoord:np.ndarray) -> np.ndarray:
        '''
        just lerp
        '''
        num = int(np.linalg.norm(start_wooldcoord-goal_wooldcoord)*Trajectory.SMOOTHNESS)
        x = np.linspace(start=start_wooldcoord[0],stop=goal_wooldcoord[0], num =num)
        y = np.linspace(start=start_wooldcoord[1],stop=goal_wooldcoord[1], num =num)
        return np.stack((x,y), axis=1).reshape((-1,2))

    def __init__(self, start_wooldcoord:np.ndarray, goal_wooldcoord:np.ndarray):
        self._completed: bool = False
        self._start:np.ndarray = start_wooldcoord
        self._goal:np.ndarray = goal_wooldcoord
        self._current_idx:int = 0
        self._inbetween:np.ndarray = Trajectory._generate_path(start_wooldcoord = start_wooldcoord, goal_wooldcoord = goal_wooldcoord)
        self._inbetween_angles:np.ndarray = None
    
    async def generate_angles(self, trs:TrajectoryServer):
        self._inbetween_angles = np.zeros(self._inbetween.shape)
        for idx, point in enumerate(self._inbetween):
            future = trs.send_ik_request(x = point[0],
                                            y = point[1])
            await future
            resp = future.result()
            self._inbetween_angles[idx, 0] = resp.theta1
            self._inbetween_angles[idx, 1] = resp.theta2


    def waypoint_reached(self,worldcoord:np.ndarray)->bool:
        return np.linalg.norm(worldcoord-self.next_worldcoord) < Trajectory.CLOSENESS_THREASHOLD

    def angle_reached(self,angles:np.ndarray)->bool:
        return np.linalg.norm(angles-self.next_angle) < Trajectory.CLOSENESS_THREASHOLD


        
    def advance_waypoint(self):
        self._current_idx += 1
        self._completed = (self._current_idx == len(self._inbetween))
    
    @property
    def mission_complete(self)->bool:
        return self._completed

    @property
    def current_idx(self)->int:
        return self._current_idx
    
    @property
    def next_worldcoord(self)->np.ndarray:
        return self._inbetween[self._current_idx]

    @property
    def next_angle(self)->np.ndarray:
        return self._inbetween_angles[self._current_idx]


NO_INTERPOLATIONS = 100
class TrajectoryServer(Node): 
    DEBUG = False
    #====================================== get stuff from configs ======================================
    bringup_pkg = get_package_share_directory('asmr_arm_bringup')
    debug_file = os.path.join(bringup_pkg,'config', 'debug.yaml')
    with open (debug_file) as f:
        DEBUG = yaml.safe_load(f)["trajectory"]
    #====================================================================================================

    def __init__(self, name:str):
        super().__init__(name)
        # Variables
        self._feedback_msg = ExecuteTrajectory.Feedback()
        self.fk_req = ComputeFK.Request()
        self.ik_req = ComputeIK.Request()
        self.mdof_cmd = MultiDOFCommand()
        self.mdof_cmd.dof_names = ["shoulder", "elbow"]

        self._del_me = True

        self.current_theta1:float = 0.0
        self.current_theta2:float = 0.0
        self._current_x:float = 0
        self._current_y:float = 0

        self._current_worldcoord: np.ndarray = np.array([1,0])
        self._current_trajectory: Trajectory = None

        self._current_goal_handle = None
        #self._current_result = None#ExecuteTrajectory.Result()

        # Create service clients=========================================================================
        self._fk_client = self.create_client(
            ComputeFK,
            "forward_kinematics",
        )
        self._ik_client = self.create_client(
            ComputeIK,
            "inverse_kinematics",
        )
        # Create publisher ===============================================================================
        self._multi_dof_cmd_pub = self.create_publisher(MultiDOFCommand, '/arm_pid_controller/reference', 10)

        while not self._fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("FK server not available, trying again")
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("IK server not available, trying again")
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::Successfully connected to FK & IK servers"+30*"-")

        # Create topic subscriptions
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        # Create service
        self._generate_bs_server = self.create_service(
            BSService, 
            'bs_service',
            self.bs_callback
        )

        # Create action server
        self._trajectory_server = ActionServer(
            self,
            ExecuteTrajectory,
            'execute_trajectory',
            self.execute_trajectory
        )
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::now ready "+30*"=")
        


    async def _update_current_worldcoord(self):
        '''
        get the state from the joint states, calculate the curr worldpos from there using the services
        '''
     #   if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update_current_worldcoord "+30*"-")
        future = self.send_fk_request(float(self.current_theta1), float(self.current_theta2))
        await future
        response = future.result()
        self._current_worldcoord = np.array([response.x, response.y])
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update current agnles :{self.current_theta1} :{self.current_theta2}"+30*"-")
        #if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update_current_worldcoord done "+30*"-")


    async def _update(self)-> bool:
        '''
        this will check the current state, and acto accordingly
            - check if there is anything todo
            - if so, 
        '''
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::ender_update "+30*"<")
    
        #await self._update_current_worldcoord()
        #if self._current_trajectory.waypoint_reached(self._current_worldcoord):
        if self._current_trajectory.angle_reached(np.array([self.current_theta1,self.current_theta2])):
            '''
            if the waypoint is reached, advance it to now go for the next one in line
            '''
            feedback = ExecuteTrajectory.Feedback()
            feedback.waypoint_index = self._current_trajectory.current_idx
            feedback.ee_x = self._current_worldcoord[0]
            feedback.ee_y = self._current_worldcoord[1]
            self._current_goal_handle.publish_feedback(feedback)

            if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update waypoint {self._current_trajectory.current_idx} reached \n nexz point{self._current_trajectory.next_worldcoord} E"+30*"Y")
            if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update waypoint {self._current_trajectory.current_idx} reached \n nexz point{self._current_trajectory.next_angle} E"+30*"Y")
            self._current_trajectory.advance_waypoint()
            self._del_me = True
            

        
        if self._current_trajectory.mission_complete:
            '''
            if the mission is completed, remove the trajectory entirely, and set return command
            '''
            self._current_trajectory = None

            self._current_goal_handle.succeed()
            result = ExecuteTrajectory.Result()
            result.success = True
            result.theta1 = self.current_theta1
            result.theta2 = self.current_theta2
            #self._current_goal_handle.set_result(result)
            if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update mission succ"+30*"-")
            return True

        elif self._del_me:
            self._del_me = False
            thetas = self._current_trajectory.next_angle
            self._send_muilti_dof_cmd(thetas[0], thetas[1])
            
            # curren_goal_worldcoords = self._current_trajectory.next_worldcoord

            # # check feasibility
            # future = self.send_ik_request(x = curren_goal_worldcoords[0],
            #                               y = curren_goal_worldcoords[1])
            # await future
            # if not future.result().success:
            #     self._current_goal_handle.abort()
            #     result = ExecuteTrajectory.Result()
            #     result.success = False
            #     self.message = "OUT OF REACH"
            #     self._current_trajectory = None
            #     if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update mission failed"+30*"-")
            #     return True

            # resp = future.result()
            # curren_goal_anglespace = (resp.theta1, resp.theta2)
            # if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update sending new angles {curren_goal_anglespace}"+30*"-")
            # self._send_muilti_dof_cmd(*curren_goal_anglespace)
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::exiz_update "+30*">")

        return False


    async def execute_trajectory(self, goal_handle):
        await self.debug()
        #return goal_handle
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory "+30*"-")
        self._current_goal_handle = goal_handle
        await self._update_current_worldcoord()
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal = np.array([goal_x,goal_y])
        self._current_trajectory = Trajectory(start_wooldcoord=self._current_worldcoord, goal_wooldcoord=goal)
        await self._current_trajectory.generate_angles(self)
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory curr {self._current_trajectory._inbetween}"+30*"-")
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory curr {self._current_trajectory._inbetween_angles}"+30*"-")
        update_compleete = False
        while not update_compleete:
            update_compleete = await self._update()
            #time.sleep(0.1)
        return goal_handle



    def _send_muilti_dof_cmd(self,theta1:float, theta2:float):
        self.mdof_cmd.values = [theta1, theta2]
        return self._multi_dof_cmd_pub.publish(self.mdof_cmd)


    def send_fk_request(self, theta1, theta2):
        self.fk_req.theta1 = float(theta1)
        self.fk_req.theta2 = float(theta2)
        #if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::sending fk request theta1: {theta1}, theta2: {theta2} "+30*"-")
        return self._fk_client.call_async(self.fk_req)


    def send_ik_request(self, x, y):
        self.ik_req.x = float(x)
        self.ik_req.y = float(y)
        #if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::sending ik request x: {self.ik_req.x}, y: {self.ik_req.y} "+30*"-")
        return self._ik_client.call_async(self.ik_req)


    def joint_callback(self, msg):

        idx = {name: i for i, name in enumerate(msg.name)}
        self.current_theta1 = msg.position[idx["shoulder"]]
        self.current_theta2 = msg.position[idx["elbow"]]


    def bs_callback(self, request, response):
        x_interpolation, y_interpolation = zip(*(self._plan_trajectory((request.x, request.y))))
        response.x_coords = x_interpolation
        response.y_coords = y_interpolation
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::exiting bs_callback "+30*"-")


    @property
    def active(self)->bool:
        return not self._current_trajectory is None

    async def debug(self):
        theta1_pre = 0.5
        theta2_pre = 0.6
        
        future = self.send_fk_request(theta1_pre, theta2_pre)
        await future
        response = future.result()
        x =  response.x
        y =  response.y


        future = self.send_ik_request(x = x,
                                        y = y)
        await future
        resp = future.result()


        theta1_post =resp.theta1
        theta2_post =resp.theta2

        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::DEBUG pre:{theta1_pre}, post: {theta1_post} "+30*"TT")
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::DEBUG pre:{theta2_pre}, post: {theta2_post} "+30*"TT")












def main(args=None) -> None:
    rclpy.init(args=args)
    server = TrajectoryServer('trajectory_server')
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()