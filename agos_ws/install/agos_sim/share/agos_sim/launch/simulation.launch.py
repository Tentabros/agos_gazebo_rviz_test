#!/usr/bin/env python3
"""AGOS full simulation bring-up: robot_state_publisher, Gazebo (pipe_world.sdf),
robot spawn, ros_gz_bridge topic bridges, and RViz2 -- per DRIFT.md topic/frame
conventions.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agos_sim')

    xacro_file = os.path.join(pkg_share, 'urdf', 'agos.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'pipe_world.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'agos_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock')

    # 1. Process Xacro -> robot_description, publish TF via robot_state_publisher
    robot_description_config = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time,
        }])

    # 2. Launch Gazebo (Harmonic) with pipe_world.sdf
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items())

    # 3. Spawn AGOS into Gazebo at (0, 0, 0.2) once the world is up
    spawn_agos = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_agos',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'agos',
            '-x', '0.0', '-y', '0.0', '-z', '0.2',
        ],
        output='screen')

    spawn_agos_delayed = TimerAction(period=5.0, actions=[spawn_agos])

    # 4. ros_gz_bridge: /cmd_vel, /scan, /camera/depth/points, /imu/data (+ camera
    #    extras) per config/bridge.yaml
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    # 5. RViz2 with the AGOS preset
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        gz_sim,
        spawn_agos_delayed,
        gz_bridge,
        rviz,
    ])
