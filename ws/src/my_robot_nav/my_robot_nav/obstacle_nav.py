'''
- Subscribe to /goal_point (geometry_msgs/PointStamped, latched QoS,
    durability TRANSIENT_LOCAL) to receive the goal.

– SubscribetotheLiDARtopicsyouralgorithmuses(/scan and/or/scan_points)
    and to /odom for the robot pose.

– Use a SetVelocity action client to command velocities. The action is de-
    finedinmy_robot_interfaces; theserverlivesinvelocity_controller_node
    (started by your launch file in part (b)). Your node does not publish
    /cmd_vel directly.

– Publish or log at least one named visualization / debugging signal — e.g.
    an RViz marker for the chosen heading, a throttled log of a key internal
    variable, or a geometry_msgs arrow. The signal must be observable while
    the node runs and named in your submission’s README (one sentence: what
    it is, how to view it).
'''
#================================================================================================

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import tf2_geometry_msgs  # noqa: F401  (registers transform support for PointStamped)
import numpy as np
from my_robot_interfaces.action import SetVelocity # this is the action defined by the provided movement controller, the topic is /set_velocity
"""# Goal — request a constant velocity command. The controller publishes
# /cmd_vel at its tick rate using these values until a new goal preempts
# this one or the client cancels.
float64 linear_x
float64 angular_z
---
# Result — populated on cancel / preempt / shutdown.
bool stopped
---
# Feedback — published at the controller's tick rate.
float64 current_linear_x
float64 current_angular_z
"""
class ObstacleNav(Node):

    def _call_service(self, client, request) -> None:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def __init__(self):
        super().__init__('obstacle_nav')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        

        #movement client test...
        self._client = ActionClient(self, SetVelocity, '/set_velocity')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        _TICK_HZ = 10       # publish rate during an edge (Hz)
        _TICK_SEC = 1.0 / _TICK_HZ
        for i in range(10):
            self._call_service(self._client,SetVelocity.Request(linear_x = 4))
            rclpy.spin_once(self, timeout_sec=_TICK_SEC)
        #/movement client test...


        

        
        
        self._goal_subscription = self.create_subscription(
            PointStamped,
            "/goal_point",
            self._goal_listener_callback,
            10)
        self._goal_subscription  # prevent unused variable warning
        
        self._lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self._lidar_listener_callback, 
            10)
        self._lidar_subscription # prevent unused variable warning

        
    def _goal_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def _lidar_listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        self.remove_me(msg)
    
    #...................................
    def remove_me(self, msg: LaserScan) -> None:
        # TODO 1: find the index of the minimum range value in msg.ranges
        #         (ignore inf and nan values — use math.isfinite())

        arr = np.array(msg.ranges)
        nan_mask = np.isnan(arr)
        inf_mask = np.isinf(arr)
        total_mask = inf_mask | nan_mask
        #arr = arr[~total_mask]
        arr[nan_mask] = np.inf
        self.get_logger().info(f"{type(arr)}")
        self.get_logger().info(f"{arr.shape}")
        
        idx = np.argmin(arr)
        self.get_logger().info(f"{arr[idx]}")

        # TODO 2: compute the angle of that beam
        #         angle = msg.angle_min + idx * msg.angle_increment
        angle = (msg.angle_min + idx * msg.angle_increment)/180 * np.pi

        # TODO 3: convert polar (msg.ranges[idx], angle) to Cartesian (x, y)
        #         in the lidar_link frame  —  x = r*cos(θ),  y = r*sin(θ)
        
        x = np.cos(angle) * arr[idx]
        y = np.sin(angle) * arr[idx]
        

        # TODO 4: fill in the PointStamped message
        point_in_lidar = PointStamped()
        point_in_lidar.header.frame_id = "lidar_link"   # which frame is this point in?
        point_in_lidar.header.stamp = msg.header.stamp       # use the scan message's timestamp
        point_in_lidar.point.x = x
        point_in_lidar.point.y = y
        point_in_lidar.point.z = 0.0

        # TODO 5: look up the transform from lidar_link to base_link
        #         use rclpy.time.Time() to request the latest available transform
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_link",   # target frame
                "lidar_link",   # source frame
                rclpy.time.Time()   # time
            )
        except tf2_ros.LookupException:
            return

        # TODO 6: apply the transform to obtain the point in base_link
        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_lidar, tf)

        self.get_logger().info(
            f'Nearest obstacle — lidar_link: ({x:.3f}, {y:.3f}, 0.000)  '
            f'base_link: ({point_in_base.point.x:.3f}, '
            f'{point_in_base.point.y:.3f}, '
            f'{point_in_base.point.z:.3f})'
        )
    #...................................



def main(args=None) -> None:
    rclpy.init(args=args)

    obstacle_nav = ObstacleNav()

    rclpy.spin(obstacle_nav)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

















#================================================================================================
