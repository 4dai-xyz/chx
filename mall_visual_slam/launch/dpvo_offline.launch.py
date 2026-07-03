#!/usr/bin/env python3
"""
在录制好的商场视频上离线运行 DPVO。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    repo = '/home/ros/ros2_orbslam3'
    return LaunchDescription([
        Node(
            package='dpvo_localization',
            executable='run_dpvo_video',
            name='dpvo_offline',
            output='screen',
            arguments=[
                '--dpvo-root', f'{repo}/Opensource code/DPVO-main',
                '--imagedir', f'{repo}/resources/input_video.mp4_bev.mp4',
                '--calib', f'{repo}/Opensource code/DPVO-main/calib/custom_mall.txt',
                '--name', 'mall_dpvo',
                '--stride', '2',
                '--save_trajectory',
                '--plot',
            ],
        ),
    ])
