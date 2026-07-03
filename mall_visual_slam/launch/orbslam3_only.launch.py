#!/usr/bin/env python3
"""只启动 ORB-SLAM3 单目节点，并使用 ORB-SLAM3 自带 Pangolin viewer。"""

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
    ])
