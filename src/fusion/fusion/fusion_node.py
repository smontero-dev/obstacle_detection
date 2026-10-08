"""Fusion node.

Matches the LIDAR obstacles (/obstacles/lidar) with the image detections
(/obstacles/ir) by bounding-box overlap in near-IR image pixels, publishes the
labelled obstacles on /obstacles/fused and RViz markers on /obstacles/markers,
and serves ~/get_closest_obstacle (obstacle_interfaces/GetClosestObstacle).
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FusionNode(Node):
    """Fuse LIDAR and image detections."""

    def __init__(self):
        super().__init__('fusion')
        self.get_logger().info('fusion started')


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    node = FusionNode()
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
