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
from rclpy.node import Node
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity



class PointNavigator(Node):

    '''
    this node is responsible, for driving the robot, do a designated point.
    it subscirbes odom, and uses the provided controller to controll the robot.
    '''
    CLOSNESS_THREASHOLD = 0.2
    CLOSNESS_ang_renameme = 0.02
    MAX_SPEED = 5.0
    MAX_LIN_ACC = 0.5
    MAX_ROT_ACC = 0.5

    def __init__(self):
        super().__init__('point_navigator')
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
    
        self._goal = SetVelocity.Goal()
        self._global_to_local_tf = None

        self._waypoint_reached = True

        self._curr_callback:callable = None

        # connect to the movement client, and to the 
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')

        while self._global_to_local_tf is None:
            # only start once the tf is usable
            self._update_global_to_local_tf()
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('point_navigator now running')
        self._movement_client.send_goal_async(self._goal, feedback_callback=self._on_setvel_feedback)
        rclpy.spin_once(self)

    def kill(self):
        '''
        stop it all. 
        call once the goal is reached
        '''
        self._waypoint_reached = True
        self._curr_callback = None
        self._goal.linear_x = 0
        self._goal.angular_z = 0
        self._movement_client.send_goal_async(self._goal, feedback_callback=self.feedback_callback)

    def drive_to(self, coord:np.ndarray, callback:callable):
        '''
        set the state to goal not reached.
        since the privided controller will always call back at 10hrz, this will get picked up in the next update.
        '''
        self._current_waypoint = coord
        self._curr_callback = callback
        self._waypoint_reached = False
        self._curr_callback = self._curr_callback
        rclpy.spin_once(self)

    def _on_setvel_feedback(self,feedback_msg):
        '''
        goal is reached -> do nothing
        goal was reached this frame -> callback 
        the assigned states are not reached -> do nothing
        else get new states to assign and do so
        '''
        self.get_logger().info(
         f"callback point navigaotr"
        )
        if self._waypoint_reached: return
        if self._check_if_waypoint_reached():
            self._waypoint_reached = True
            if not self._curr_callback is None:
                self._curr_callback()
        fb = feedback_msg.feedback
        if np.abs(self._goal.linear_x -fb.current_linear_x) > PointNavigator.CLOSNESS_ang_renameme:
            return
        if np.abs(self._goal.angular_z -fb.current_angular_z) > PointNavigator.CLOSNESS_ang_renameme:
            return
        
        self._update_headings()
        self._update_lin_acc()
        self._update_rot_acc()
        
        self._movement_client.send_goal_async(self._goal, feedback_callback=self._on_setvel_feedback)
        rclpy.spin_once(self)

    def _update_lin_acc(self)->float:
        '''
        calculate the dot of the favor direction and the actual current direction, 
        use that as teh partial (close to 0 -> orth -> slow down visa versa).
        use the last adjustment (which should now be the current of the gaol since this is only triggered when that is the case) as the d term 
        no term for slowing down when the goal is close. (yet)
        '''
        next_acc = np.clip(np.cos(self._favor_heading-self._current_heading)*self._lin_pd[0] - self._current_rot_acc * self._lin_pd[1], -PointNavigator.MAX_LIN_ACC,PointNavigator.MAX_LIN_ACC)
        self._goal.linear_x += next_acc
        self._current_lin_acc = next_acc

    def _update_rot_acc(self)->float:
        '''
        the same thing as the linear acc.
        '''
        next_acc =np.clip(1-np.cos(self._favor_heading-self._current_heading)*self._rot_pd[0] - self._current_rot_acc * self._rot_pd[1], -PointNavigator.MAX_ROT_ACC,PointNavigator.MAX_ROT_ACC)
        self._goal.angular_z += next_acc
        self._current_rot_acc = next_acc

    def _check_if_waypoint_reached(self)->bool:
        return np.linalg.norm(self._current_waypoint_local) < self.CLOSNESS_THREASHOLD

    def _update_headings(self):
        '''
        update the tf and all the stuff the others depend on 
        '''
        self._update_global_to_local_tf()

        self._current_heading = self._get_global_heading()
        self._current_waypoint_local = self._make_local(self._current_waypoint)
        self._favor_heading = np.arctan2(self._current_waypoint_local[1],self._current_waypoint_local[0])

    def _update_global_to_local_tf(self):
        '''
        provides the tf to transform global into local
        '''
        try:
            self._global_to_local_tf = self._tf_buffer.lookup_transform(
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
        point_local = tf2_geometry_msgs.do_transform_point(point_glob, self._global_to_local_tf)
        return np.array([point_local.point.x,point_local.point.y])

    def _get_global_heading(self)->float:
        return  self._global_to_local_tf.tf.transform.rotation.z


def main(args=None):
    rclpy.init(args=args)

    node = PointNavigator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

