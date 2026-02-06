"""Launch 6 H264 cameras, each in its own process (fault isolation).

Staggered start with 2s intervals to avoid Tegra driver contention.
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def _cam_node(cam_id):
    return Node(
        package='gscam_h264',
        executable='h264_camera_publisher_main',
        output='screen',
        name='h264_cam{}'.format(cam_id),
        parameters=[{
            'camera_count': 1,
            'device': '/dev/video{}'.format(cam_id),
            'device_index': cam_id,
            'width': 1920,
            'height': 1280,
            'fps': 30,
            'bitrate': 30000000,
            'peak_bitrate': 40000000,
            'profile': 4,
            'control_rate': 0,
            'preset_level': 4,
            'maxperf_enable': True,
            'iframeinterval': 5,
            'io_mode': 2,
            'use_pts_stamp': True,
            'topic_prefix': '/cam',
        }],
    )


def generate_launch_description():
    ld = LaunchDescription()

    for i in range(6):
        if i == 0:
            ld.add_action(_cam_node(i))
        else:
            ld.add_action(TimerAction(
                period=float(i * 2),
                actions=[_cam_node(i)],
            ))

    return ld
