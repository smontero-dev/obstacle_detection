"""Obstacle monitor node.

Periodically calls /fusion/get_closest_obstacle
(obstacle_interfaces/GetClosestObstacle) and raises an alert on
/obstacle_monitor/alert when an obstacle in front is closer than a
configurable distance.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ObstacleMonitorNode(Node):
    """Raise proximity alerts using the fusion service."""

    def __init__(self):
        super().__init__('obstacle_monitor')
        self.get_logger().info('obstacle_monitor started')


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C reaches the node twice (terminal and launch): ignore the second one
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
