#!/usr/bin/env python3

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

    # FAST-LIO node
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_node',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            os.path.join(fast_lio_share, 'config', 'avia.yaml'),
            {
                'feature_extract_enable': False,
                'point_filter_num': 5,
                'max_iteration': 4,
                'dense_map_enable': True,
                'filter_size_surf': 0.3,
                'filter_size_map': 0.3,
                'cube_side_length': 1000.0,
                'correction_ver_thr': 0.20,
                'correction_dis_interval': 200.0,
                'dy_mapretrival_range': 30.0,
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
        fastlio_node,
        loop_opt_node,
        loop_detection_node,
    ])
