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
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs import PointStamped

class ObstacleNav(Node):
    def __init__(self):
        super().__init__('obstacle_nav')
        
        self._subscription = self.create_subscription(
            PointStamped,
            "/goal_poin"t,
            self._goal_listener_callback,
            10)
        self._subscription  # prevent unused variable warning

    def _goal_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)



def main(args=None) -> None:
    """Initialise rclpy, run the node, and shut down cleanly."""
    rclpy.init(args=args)
    node = ObstacleNav()
    try:
        # By the time spin() is called, the star is already drawn.
        # spin() keeps the node alive so turtlesim stays open and the
        # user can see the result. Press Ctrl+C to exit.
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

















#================================================================================================
