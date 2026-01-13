#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('fast_lio')

    config_file = LaunchConfiguration('config_file')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_share_dir, 'config', 'avia.yaml'),
        description='Path to the config file'
    )

    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_node',
        name='fastlio_mapping',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        declare_config_file,
        fastlio_node,
    ])
