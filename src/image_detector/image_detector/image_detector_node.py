"""Image object detector node.

Runs YOLO on an image topic (by default the Ouster near-IR image,
/ouster/nearir_image) and publishes the 2D detections on /obstacles/ir
(obstacle_interfaces/ObstacleArray), plus a debug image with the boxes drawn.
The input topic is a parameter, so a second instance can process the RGB
camera.
"""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ImageDetectorNode(Node):
    """Detect objects in an image with YOLO."""

    def __init__(self):
        super().__init__('image_detector')
        self.get_logger().info('image_detector started')


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args)
    node = ImageDetectorNode()
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
