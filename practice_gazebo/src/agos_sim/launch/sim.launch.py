import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('agos_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg, 'urdf', 'agos.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    default_world = os.path.join(pkg, 'worlds', 'test_world.sdf')
    rviz_cfg = os.path.join(pkg, 'rviz', 'agos.rviz')

    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 with the AGOS config')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items(),
    )

    # Model name must be "agos": the diff-drive TF topic /model/agos/tf_odom_raw depends on it
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'agos',
                   '-x', '0', '-y', '0', '-z', '0.05'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([world_arg, rviz_arg, gz, rsp, spawn, bridge, rviz])