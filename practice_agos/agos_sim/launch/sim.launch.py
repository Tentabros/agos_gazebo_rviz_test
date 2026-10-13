import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('agos_sim')
    xacro_file = os.path.join(pkg_path, 'urdf', 'agos_urdf.xacro')
    bridge_config = os.path.join(pkg_path, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_path, 'rviz', 'agos.rviz')

    robot_desc = xacro.process_file(xacro_file).toxml()

    # ---------- Launch arguments ----------
    args = [
        DeclareLaunchArgument(
            'world', default_value='agos_pipe.sdf',
            description='World file name inside agos_sim/worlds (agos_pipe.sdf or '
                        'agos_world.sdf). It must load the Sensors and IMU systems '
                        'or the sensors stay silent.'),
        # Default spawn pose: middle of the pipe's longest straight, facing +x,
        # tracks just above the curved floor of the 600 mm bore.
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.16'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server only, without its GUI window'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'odom_tf', default_value='true',
            description='Bridge the DiffDrive odom->base_footprint TF onto /tf. '
                        'Set false once an EKF publishes that transform.'),
    ]

    # ---------- Robot description ----------
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ---------- Gazebo ----------
    # Lets Gazebo resolve model://agos_sim/meshes/... (the pipe in the world
    # file, and the robot meshes: package:// URIs in the URDF become model://)
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_path))

    gz_flags = PythonExpression(
        ["'-r -s' if '", LaunchConfiguration('headless'), "' == 'true' else '-r'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': [gz_flags, ' ', PathJoinSubstitution(
                [pkg_path, 'worlds', LaunchConfiguration('world')])],
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'agos',
                   '-x', LaunchConfiguration('spawn_x'),
                   '-y', LaunchConfiguration('spawn_y'),
                   '-z', LaunchConfiguration('spawn_z'),
                   '-Y', LaunchConfiguration('spawn_yaw')],
        output='screen'
    )

    # ---------- Bridges ----------
    # All topics (clock, cmd_vel, odom, joint_states, scan, imu, camera) live in
    # config/bridge.yaml. /clock MUST be bridged: every node here runs with
    # use_sim_time, and without /clock their clocks stay at zero.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # DiffDrive publishes its TF on /model/agos/tf_odom_raw (see agos_tracks.xacro)
    odom_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_tf_bridge',
        arguments=['/model/agos/tf_odom_raw@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/agos/tf_odom_raw', '/tf')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('odom_tf')),
        output='screen'
    )

    # ---------- RViz ----------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription(args + [
        node_robot_state_publisher,
        gz_resource_path,
        gazebo,
        spawn_entity,
        bridge,
        odom_tf_bridge,
        rviz,
    ])