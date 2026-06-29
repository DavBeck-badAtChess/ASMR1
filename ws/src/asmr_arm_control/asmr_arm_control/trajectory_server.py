from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum
import asyncio
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from asmr_arm_interfaces.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from control_msgs.msg import MultiDOFCommand
from asmr_arm_interfaces.srv import ComputeFK
from asmr_arm_interfaces.srv import ComputeIK


import os
from ament_index_python.packages import get_package_share_directory
import yaml

from enum import Enum

''' TRAJECTORY MISSION
# Goal: sequence of end-effector waypoints in the arm's planar frame (metres)
float64[] x      # reach from the shoulder, per waypoint [m]
float64[] y      # height relative to the shoulder, per waypoint [m]
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

 
class Trajectory:
    CLOSENESS_THREASHOLD:float = 0.2# if i make this any closer, the arm is not reaching its target...i did try many things to fix this....
    SMOOTHNESS:float = 10.0

    @staticmethod
    def _generate_path(start_wooldcoord:np.ndarray, goal_wooldcoord:np.ndarray) -> np.ndarray:
        '''
        the trajectory is just lerp (no fancy checking if the path even robot intersecting or whatever, that will be seen when a coord is acutally driven to)
        the point densety is higher on the start/stop of the traj (i think what i am using is calles cosine distrobution, but i did not look it up to check)
        '''
        num = max(2,int(np.linalg.norm(start_wooldcoord-goal_wooldcoord)*Trajectory.SMOOTHNESS))
        dir = goal_wooldcoord - start_wooldcoord 
        dist = np.linalg.norm(dir, keepdims=True)
        dir /= dist
        ang = np.linspace(start=0, stop= np.pi, num=num)
        dists = (-np.cos(ang)+1)/2 * dist
        x = dists * dir[0] + start_wooldcoord[0]
        y = dists * dir[1] + start_wooldcoord[1]
        #x = np.linspace(start=start_wooldcoord[0],stop=goal_wooldcoord[0], num =num)
        #y = np.linspace(start=start_wooldcoord[1],stop=goal_wooldcoord[1], num =num)
        return np.stack((x,y), axis=1).reshape((-1,2))


    def __init__(self, start_wooldcoord:np.ndarray, goal_wooldcoord:np.ndarray):
        '''
        primitive impl of a basic lerped trajectory.
        give it two points and it will calculate the lerped trajectory.
        this also keepes track of where in the trajectory we are.

        as a debug i also put in a methode to directly work with the angles (ie pre calculate the angles and use them).
        this scenario is almost (the close enough is not in eukl space) the same as using the world coords, but without 
        the annyoing service call delays of the IK and FK. i dont think that is really the intention though so i wont use it.
        '''
        self._completed: bool = False
        self._start:np.ndarray = start_wooldcoord
        self._goal:np.ndarray = goal_wooldcoord
        self._current_idx:int = 0
        self._inbetween:np.ndarray = Trajectory._generate_path(start_wooldcoord = start_wooldcoord, goal_wooldcoord = goal_wooldcoord)
        self._inbetween_angles:np.ndarray = None


    async def generate_angles(self, trs:TrajectoryServer):
        '''
        unused
        precompute all the angles
        '''
        self._inbetween_angles = np.zeros(self._inbetween.shape)
        for idx, point in enumerate(self._inbetween):
            future = trs.send_ik_request(x = point[0],
                                            y = point[1])
            await future
            resp = future.result()
            self._inbetween_angles[idx, 0] = resp.theta1
            self._inbetween_angles[idx, 1] = resp.theta2


    def waypoint_reached(self,worldcoord:np.ndarray)->bool:
        '''
        use eukl distance to eval if the curr pos is already reached
        '''
        return np.linalg.norm(worldcoord-self.next_worldcoord) < Trajectory.CLOSENESS_THREASHOLD


    def angle_reached(self,angles:np.ndarray)->bool:
        '''
        unused
        see if the current angle "matches" the precomputed one
        '''
        return np.linalg.norm(angles-self.next_angle) < Trajectory.CLOSENESS_THREASHOLD


    def advance_waypoint(self):
        '''
        call when a waypoint is reached.
        if that was the last waypoint, the mission is done 
        '''
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
        '''
        unused
        '''
        return self._inbetween_angles[self._current_idx]


class TRAJECTORY_STATE(Enum):
    SUCC = 0
    FAIL = 1
    RUNNING = 2


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
        self._feedback_msg = ExecuteTrajectory.Feedback()
        self.fk_req = ComputeFK.Request()
        self.ik_req = ComputeIK.Request()
        self.mdof_cmd = MultiDOFCommand()
        self.mdof_cmd.dof_names = ["shoulder", "elbow"]

        # Variables
        self.current_theta1:float = 0.0
        self.current_theta2:float = 0.0
        self._current_x:float = 0
        self._current_y:float = 0

        self._current_worldcoord: np.ndarray = np.array([1,0])

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

        # Create topic subscriptions =====================================================================
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )


        # Create action server ===========================================================================
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
        future = self.send_fk_request(float(self.current_theta1), float(self.current_theta2))
        await future
        response = future.result()
        self._current_worldcoord = np.array([response.x, response.y])
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update current agnles :{self.current_theta1} :{self.current_theta2}"+30*"-")


    async def _update(self, trajectory:Trajectory, goal_handle)-> TRAJECTORY_STATE:
        '''
            - update the current position
            - check if the current traj point is reached (the first call garantues this, since the first point is the curr pos)
            - if that was the last point, the whole traj has been walked. done, return true. 
            - if the waypoint was reached, advance to the next one and send the multi dof cmd
        '''
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::enter_update "+30*"<")
        waypoint_has_changed:bool = False
        mission_state:TRAJECTORY_STATE = TRAJECTORY_STATE.RUNNING

        await self._update_current_worldcoord()
        if trajectory.waypoint_reached(self._current_worldcoord):
            '''
            if the waypoint is reached, advance it to now go for the next one in line.
            publish current feedback and set the changed flag
            '''
            feedback = ExecuteTrajectory.Feedback()
            feedback.waypoint_index = trajectory.current_idx
            feedback.ee_x = self._current_worldcoord[0]
            feedback.ee_y = self._current_worldcoord[1]
            goal_handle.publish_feedback(feedback)
            trajectory.advance_waypoint()
            waypoint_has_changed = True
            if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update waypoint {trajectory.current_idx} reached E"+30*"Y")

        if trajectory.mission_complete:
            '''
            if the mission is completed, remove the trajectory, set the goal stuff, return true
            '''
            goal_handle.succeed()
            mission_state = TRAJECTORY_STATE.SUCC

        elif waypoint_has_changed:
            '''
            get the next waypoint, convert it to thetas, send thetas.
            if the next waypoint is not reachable, the mission has falied.
            '''
            waypoint_has_changed = False
            curren_goal_worldcoords = trajectory.next_worldcoord
            # check feasibility
            future = self.send_ik_request(x = curren_goal_worldcoords[0],
                                        y = curren_goal_worldcoords[1])
            await future
            if not future.result().success:
                goal_handle.abort()
                if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::_update mission failed"+30*"-")
                mission_state = TRAJECTORY_STATE.FAIL
            self._send_muilti_dof_cmd(future.result().theta1,future.result().theta2)
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::exit_update "+30*">")

        return mission_state 


    async def _continue_trajectory(self,trajectory:Trajectory)->TRAJECTORY_STATE:
        '''
        do this:
            - update the woldcoords
            - if the next waypoint of the trajectory is reaced
                -> advance the waypoint
                -> check if the traj is compleeted
                -> check if the traj has failed
                -> send next angles 
        '''
        await self._update_current_worldcoord()
        if trajectory.waypoint_reached(self._current_worldcoord):
            trajectory.advance_waypoint()
            
            if trajectory.mission_complete:
                return TRAJECTORY_STATE.SUCC
            future = self.send_ik_request(x = trajectory.next_worldcoord[0],
                                        y = trajectory.next_worldcoord[1])
            await future
            if not future.result().success:
                return TRAJECTORY_STATE.FAIL
            self._send_muilti_dof_cmd(future.result().theta1,future.result().theta2)
        return TRAJECTORY_STATE.RUNNING


    async def execute_trajectory(self, goal_handle):
        '''
        extract the points, generate the trajectories,
        publish feedback on each reached waypoint
        '''
        mission_points = np.asanyarray(zip(goal_handle.request.x,goal_handle.request.y))
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory {mission_points} "+30*"-")
        mission_trajectories = [Trajectory(start_wooldcoord=mission_points[i-1], goal_wooldcoord= mission_points[i-1]) for i in range(1, len(mission_points))]
        result = ExecuteTrajectory.Result()

        for idx, trj in enumerate(mission_trajectories):
            '''
            runn the trajectory untill its done or failed.
            '''
            trj_state = TRAJECTORY_STATE.RUNNING
            while trj_state == TRAJECTORY_STATE.RUNNING:
                self._continue_trajectory(trajectory=trj)

            match trj_state:
                case TRAJECTORY_STATE.SUCC:
                    feedback = goal_handle.Feedback()
                    feedback.waypoint_index = idx
                    feedback.ee_x = self._current_worldcoord[0]
                    feedback.ee_y = self._current_worldcoord[1]
                    goal_handle.publish_feedback(feedback)
                case TRAJECTORY_STATE.FAIL:
                    result.success = False
                    self.message = "OUT OF REACH"
                    return result
        result.success = True
        return result




    async def execute_trajectory_dis(self, goal_handle):
        '''
        build the trajectory, and iterate update untill it is compleeted or aborted.
        depending on the exit, build the result and return it 
        '''
        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory "+30*"-")
        
        await self._update_current_worldcoord()
        goal = np.array([goal_handle.request.x,goal_handle.request.y])
        trajectory = Trajectory(start_wooldcoord=self._current_worldcoord, goal_wooldcoord=goal)

        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory curr {trajectory._inbetween}"+30*"-")

        ms = TRAJECTORY_STATE.RUNNING
        while ms == TRAJECTORY_STATE.RUNNING:
            #time.sleep(0.01) # "for dramatic effect" :1
            ms = await self._update(trajectory, goal_handle)

        if self.__class__.DEBUG:self.get_logger().info(f"{self.__class__}::execute_trajectory done!!!!!!!!!! E"+30*"Y")

        result = ExecuteTrajectory.Result()
        match ms:
            case TRAJECTORY_STATE.SUCC:
                result.success = True
                result.theta1 = self.current_theta1
                result.theta2 = self.current_theta2
            case TRAJECTORY_STATE.FAIL:
                result.success = False
                self.message = "OUT OF REACH"
        return result



    def _send_muilti_dof_cmd(self,theta1:float, theta2:float):
        self.mdof_cmd.values = [theta1, theta2]
        return self._multi_dof_cmd_pub.publish(self.mdof_cmd)


    def send_fk_request(self, theta1, theta2):
        self.fk_req.theta1 = float(theta1)
        self.fk_req.theta2 = float(theta2)
        return self._fk_client.call_async(self.fk_req)


    def send_ik_request(self, x, y):
        self.ik_req.x = float(x)
        self.ik_req.y = float(y)
        return self._ik_client.call_async(self.ik_req)


    def joint_callback(self, msg):
        idx = {name: i for i, name in enumerate(msg.name)}
        self.current_theta1 = msg.position[idx["shoulder"]]
        self.current_theta2 = msg.position[idx["elbow"]]













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