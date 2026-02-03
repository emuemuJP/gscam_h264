"""Launch H264 camera publisher for multiple cameras (1-process mode).

Usage:
  # Sequential devices /dev/video0.../dev/video5:
  ros2 launch gscam_h264 h264_cameras.launch.py

  # Explicit device list with cam IDs:
  ros2 launch gscam_h264 h264_cameras.launch.py \
      devices:="0:/dev/video0,1:/dev/video1,2:/dev/video2,3:/dev/video3,4:/dev/video4,5:/dev/video5"
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
            'camera_count', default_value='6',
            description='Number of cameras (used when devices is empty)'),
        DeclareLaunchArgument(
            'device_base', default_value='/dev/video',
            description='Base path for video devices (used when devices is empty)'),
        DeclareLaunchArgument(
            'device_index', default_value='0',
            description='Starting device index (used when devices is empty)'),
        DeclareLaunchArgument(
            'devices', default_value='',
            description='Comma-separated device list, e.g. "0:/dev/video0,1:/dev/video1"'),

        Node(
            package='gscam_h264',
            executable='h264_camera_publisher_main',
            output='screen',
            name='h264_camera_publisher',
            parameters=[
                params_file,
                {
                    'camera_count': LaunchConfiguration('camera_count'),
                    'device_base': LaunchConfiguration('device_base'),
                    'device_index': LaunchConfiguration('device_index'),
                    'devices': LaunchConfiguration('devices'),
                },
            ],
        ),
    ])
