#!/usr/bin/env python3
"""Launch ORB-SLAM3 monocular tracking with the local video publisher."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    repo = Path(os.environ.get(
        'ROS2_ORBSLAM3_REPO',
        str(Path(__file__).resolve().parents[1]),
    ))
    orbslam3_dir = Path(os.environ.get(
        'ORB_SLAM3_DIR',
        str(repo / 'Opensource code/ORB_SLAM3-master'),
    ))

    return LaunchDescription([
        Node(
            package='orbslam3_wrapper',
            executable='mono_node',
            name='mono_node',
            output='screen',
            parameters=[{
                'vocab_path': str(orbslam3_dir / 'Vocabulary/ORBvoc.txt'),
                'settings_path': str(repo / 'config/KannalaBrandt8_960x540.yaml'),
                'enable_viewer': True,
                'enable_edge_enhancement': False,
                'enable_dynamic_mask': True,
                'mask_fill_value': 128,
                'publish_map_every_n_frames': 10,
            }],
        ),
        Node(
            package='video_publisher',
            executable='video_publisher_node',
            name='video_publisher_node',
            output='screen',
            parameters=[{
                'video_path': str(repo / 'resources/input_video.mp4'),
                'mask_video_path': str(repo / 'resources/input_video.mp4_bev.mp4'),
                'publish_every_n_frames': 1,
                'filter_enabled': False,
                'output_width': 960,
                'output_height': 540,
                'publish_overlay_mask': True,
                'mask_include_yellow': True,
                'mask_include_green': False,
                'mask_dilation_kernel': 5,
                'debug_mask_save_every_n': 100,
            }],
        ),
    ])
