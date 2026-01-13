#!/usr/bin/env python3
"""
LTAOM Launch for Livox MID360

Usage:
  ros2 launch loop_optimization ltaom_mid360.launch.py

With custom save directory:
  ros2 launch loop_optimization ltaom_mid360.launch.py save_dir:=/path/to/logs/
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    fast_lio_share = get_package_share_directory('fast_lio')
    loop_opt_share = get_package_share_directory('loop_optimization')
    std_loop_share = get_package_share_directory('std_loop')

    # Launch arguments
    save_dir = LaunchConfiguration('save_dir')
    multisession_mode = LaunchConfiguration('multisession_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_save_dir = DeclareLaunchArgument(
        'save_dir',
        default_value='/tmp/ltaom_logs/',
        description='Directory to save logs and maps'
    )

    declare_multisession = DeclareLaunchArgument(
        'multisession_mode',
        default_value='0',
        description='Multi-session mode (0: single session, 1: multi-session)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true for Gazebo)'
    )

    # FAST-LIO node with MID360 config
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_node',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(fast_lio_share, 'config', 'mid360.yaml'),
            {
                'use_sim_time': use_sim_time,
                'feature_extract_enable': False,
                'point_filter_num': 3,
                'max_iteration': 4,
                'dense_map_enable': True,
                'filter_size_surf': 0.3,
                'filter_size_map': 0.3,
                'cube_side_length': 1000.0,
            }
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
                'use_sim_time': use_sim_time,
                'multisession_mode': multisession_mode,
                'SaveDir': save_dir,
            }
        ],
    )

    # Loop detection node (delayed start to avoid Eigen alignment issues)
    loop_detection_node = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            Node(
                package='std_loop',
                executable='loop_detection_node',
                name='loop_detection',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'lcd_config_path': os.path.join(std_loop_share, 'config', 'config_avia.yaml'),
                    'multisession_mode': multisession_mode,
                    'SaveDir': save_dir,
                }],
            )
        ]
    )

    return LaunchDescription([
        declare_save_dir,
        declare_multisession,
        declare_use_sim_time,
        fastlio_node,
        loop_opt_node,
        loop_detection_node,
    ])
