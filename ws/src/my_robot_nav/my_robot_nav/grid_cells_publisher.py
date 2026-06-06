import rclpy
from rclpy.node import Node

from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point


class GridCellsPublisher(Node):
    def __init__(self):
        super().__init__('grid_cells_publisher')

        # ✅ correct topic name
        self.pub = self.create_publisher(GridCells, '/grid_cells', 10)

        self.timer = self.create_timer(1.0, self.publish_grid)

        self.get_logger().info("GridCells publisher started")

    def publish_grid(self):
        msg = GridCells()

        # frame in which grid is displayed
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # size of each cell
        msg.cell_width = 0.5
        msg.cell_height = 0.5

        # example grid points (x, y)
        grid = [
            (0, 0),
            (1, 0),
            (2, 0),
            (2, 1),
            (2, 2),
            (3, 2),
            (4, 3),
        ]

        for x, y in grid:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            msg.cells.append(p)

        self.pub.publish(msg)

        self.get_logger().info("Published GridCells")


def main(args=None):
    rclpy.init(args=args)
    node = GridCellsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()