"""Launch H264 camera publisher for a single camera (1-camera-per-process mode).

Usage:
  ros2 launch gscam_h264 single_camera.launch.py cam_id:=3 device:=/dev/video3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gscam_h264')
    params_file = os.path.join(pkg_dir, 'cfg', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cam_id', default_value='0',
            description='Camera ID (used for device_index and topic name)'),
        DeclareLaunchArgument(
            'device', default_value='/dev/video0',
            description='Video device path (used directly, not concatenated)'),

        Node(
            package='gscam_h264',
            executable='h264_camera_publisher_main',
            output='screen',
            name=['h264_cam', LaunchConfiguration('cam_id')],
            parameters=[
                params_file,
                {
                    'camera_count': 1,
                    'device': LaunchConfiguration('device'),
                    'device_index': LaunchConfiguration('cam_id'),
                },
            ],
        ),
    ])
