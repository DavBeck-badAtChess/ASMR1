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

    TICK_HZ = 10.0
    CLOSNESS_THREASHOLD = 1.0

    test_mission = {0:np.array([5,5]),
                   1:np.array([-5,5]),
                   2:np.array([-5,-5]),
                   3:np.array([5,-5])}

    def __init__(self):
        super().__init__('point_navigator')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._movement_client = ActionClient(self, SetVelocity, '/set_velocity')

        self.count = 0
        self.mission_counter = 0
        self.fb_there = True

        self._current_waypoint: np.ndarray = None
        self._reached:bool = True
        self._goal = SetVelocity.Goal()
        self._globa_to_local_tf = None


        # connect to the movement client, and to the 
        while not self._movement_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service set velocity not available, waiting again...')
        
        #self.timer = self.create_timer(1.0, self.timer_callback)
        self._timer = self.create_timer(
            1.0 / self.TICK_HZ, self._update
        )
        self.get_logger().info('point_navigator now running')
        

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        #print("current v:", fb.current_linear_x)
        #print("current w:", fb.current_angular_z)

        if np.abs(self._goal.linear_x -fb.current_linear_x) <0.02:
            if np.abs(self._goal.angular_z -fb.current_angular_z) <0.02:
                if not self.fb_there:self.get_logger().info(f'ready_to_move_again')
                self.fb_there = True
        
    def _update_globa_to_local_tf(self):
        '''
        provides the tf to transform global into local
        '''
        try:
            self._globa_to_local_tf = self._tf_buffer.lookup_transform(
                "base_link",   # target frame
                "odom",   # source frame
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
        #return point_local

    def _get_global_heading(self)->float:
        return  self._globa_to_local_tf.tf.transform.rotation.z

    def drive_to(self, waypoint: np.ndarray):
        self._current_waypoint = waypoint
        self._reached = False
      #  self._update() # call this update emedietly for a fast resoponse

    def _send_waypoint_reached_signal(self):
        self.get_logger().info(f'goal reached !!!')
        pass

    def _update(self):
        '''
        get the correct rotation angle, get the global one, make the correct adjustment, and continue driving.
        check wheter the goal is reached (some closenes threashold)
        '''
        self.count += 1
        if self.count == 50:
            self.drive_to(self.test_mission[self.mission_counter])
        elif self.count <50:
            self.get_logger().info(f'count {self.count}')
        else:
            pass
            #self.get_logger().info(f'moving {self._reached}')
        
        if not self.fb_there:
            return 

        self._update_globa_to_local_tf()
        if self._reached or self._globa_to_local_tf is None:
            return

        local_goal = self._make_local(self._current_waypoint)

        if np.linalg.norm(local_goal) < self.CLOSNESS_THREASHOLD:
            '''
            point is reached. 
            stop the movement, set the flag, and send the signal
            '''
            self._reached = True
            #self._goal.linear_x = 0.0
            #self._goal.angular_z = 0.0
            #self._movement_client.send_goal_async(self._goal)
            self._send_waypoint_reached_signal()

            self.mission_counter += 1
            if self.mission_counter > 3:
                self.mission_counter=0
            self.drive_to(self.test_mission[self.mission_counter])
            
            return

        target_angle = np.arctan2(local_goal[1],local_goal[0])
        
        forward_vel = 1.0

        self._goal.linear_x = forward_vel
        self._goal.angular_z = target_angle

        self._movement_client.send_goal_async(self._goal, feedback_callback=self.feedback_callback)
        self.fb_there = False
        #if self.count> 50:self.get_logger().info(f'moving send with local {local_goal}')
        


def main(args=None):
    rclpy.init(args=args)

    node = PointNavigator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

