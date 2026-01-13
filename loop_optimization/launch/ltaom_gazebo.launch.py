#!/usr/bin/env python3
"""
LTAOM Launch for Gazebo Simulation

Usage:
  ros2 launch loop_optimization ltaom_gazebo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    loop_opt_share = get_package_share_directory('loop_optimization')
    std_loop_share = get_package_share_directory('std_loop')

    save_dir = LaunchConfiguration('save_dir')

    declare_save_dir = DeclareLaunchArgument(
        'save_dir',
        default_value='/tmp/ltaom_logs/',
        description='Directory to save logs and maps'
    )

    # FAST-LIO with Gazebo config (PointCloud2, not CustomMsg)
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_node',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(fast_lio_share, 'config', 'gazebo.yaml'),
            {'use_sim_time': True}
        ],
    )

    # Loop optimization node
    loop_opt_node = Node(
        package='loop_optimization',
        executable='loop_optimization_node',
        name='loop_optimization',
        output='screen',
        parameters=[
            os.path.join(loop_opt_share, 'launch', 'loopopt_config_avia.yaml'),
            {
                'use_sim_time': True,
                'multisession_mode': 0,
                'SaveDir': save_dir,
            }
        ],
    )

    # Loop detection node
    loop_detection_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='std_loop',
                executable='loop_detection_node',
                name='loop_detection',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'lcd_config_path': os.path.join(std_loop_share, 'config', 'config_avia.yaml'),
                    'multisession_mode': 0,
                    'SaveDir': save_dir,
                }],
            )
        ]
    )

    return LaunchDescription([
        declare_save_dir,
        fastlio_node,
        loop_opt_node,
        loop_detection_node,
    ])
