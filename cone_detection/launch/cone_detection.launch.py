#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cone_detection'),
        'config',
        'cone_detection.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    cone_detection_node = Node(
        package='cone_detection',
        executable='cone_detection_node',
        output='screen',
        parameters=[
            config,
            {'use_sim_time': use_sim_time}
        ]
    )

    path_planner_process = ExecuteProcess(
        cmd=[
            'python3',
            '/home/m1_coro1/ros2/src/jemaro_days_2025/jemaro/script/path_planner.py'
        ],
        output='screen'
    )

    return LaunchDescription([
        cone_detection_node,
        path_planner_process
    ])
