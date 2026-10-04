#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        Node(
            package='agos_sim',
            executable='agos_sim_node',
            name='agos_sim_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'),
    ])
