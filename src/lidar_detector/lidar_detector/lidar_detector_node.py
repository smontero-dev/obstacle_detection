"""LIDAR obstacle detector node.

Subscribes to the Ouster point cloud (/ouster/points, frame os_lidar), removes
the ground, clusters the remaining points and publishes one Obstacle per
cluster on /obstacles/lidar (obstacle_interfaces/ObstacleArray, frame
os_sensor), including the cluster bounding box in near-IR image pixels.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class LidarDetectorNode(Node):
    """Detect obstacles in the point cloud."""

    def __init__(self):
        super().__init__('lidar_detector')
        self.get_logger().info('lidar_detector started')


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    node = LidarDetectorNode()
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
