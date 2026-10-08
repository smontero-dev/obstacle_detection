"""Launch every node of the obstacle detection system.

The bag is played separately with `ros2 bag play`.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the detectors, the fusion node and the obstacle monitor."""
    return LaunchDescription([
        Node(package='lidar_detector', executable='lidar_detector_node',
             name='lidar_detector', output='screen'),
        Node(package='image_detector', executable='image_detector_node',
             name='image_detector', output='screen'),
        Node(package='fusion', executable='fusion_node',
             name='fusion', output='screen'),
        Node(package='obstacle_monitor', executable='obstacle_monitor_node',
             name='obstacle_monitor', output='screen'),
    ])
