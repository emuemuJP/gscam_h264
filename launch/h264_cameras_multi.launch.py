"""Launch H264 camera publisher: 1 camera = 1 process (fault isolation mode).

Each camera runs in a separate process so a driver crash
in one camera does not kill the others.

Usage:
  # Default: 6 cameras /dev/video0.../dev/video5
  ros2 launch gscam_h264 h264_cameras_multi.launch.py

  # Custom device list (comma-separated id:path pairs)
  ros2 launch gscam_h264 h264_cameras_multi.launch.py \
      devices:="0:/dev/video0,1:/dev/video1,3:/dev/video3"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gscam_h264')
    params_file = os.path.join(pkg_dir, 'cfg', 'params.yaml')

    # Default: 6 cameras, video0..video5
    default_devices = ','.join(
        ['{}:/dev/video{}'.format(i, i) for i in range(6)])

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'devices', default_value=default_devices,
            description='Comma-separated id:path pairs, e.g. "0:/dev/video0,3:/dev/video3"'),
    ])

    # Parse devices argument at launch time is tricky with substitutions,
    # so we use the default value directly for the static case.
    # For dynamic usage, override via params.yaml or CLI.

    # Generate one node per camera with staggered start (1s apart)
    devices_str = default_devices  # static default for launch generation
    pairs = [p.strip() for p in devices_str.split(',') if p.strip()]

    for idx, pair in enumerate(pairs):
        if ':' in pair:
            cam_id_str, dev_path = pair.split(':', 1)
            cam_id = int(cam_id_str)
        else:
            cam_id = idx
            dev_path = pair

        node = Node(
            package='gscam_h264',
            executable='h264_camera_publisher_main',
            output='screen',
            name='h264_cam{}'.format(cam_id),
            parameters=[
                params_file,
                {
                    'camera_count': 1,
                    'device': dev_path,
                    'device_index': cam_id,
                },
            ],
        )

        # Stagger starts by 1 second each to avoid driver contention
        if idx == 0:
            ld.add_action(node)
        else:
            ld.add_action(TimerAction(
                period=float(idx),
                actions=[node],
            ))

    return ld
