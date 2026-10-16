"""
Gazebo with the AGOS robot, its sensors bridged to ROS, odometry and RViz.

    ros2 launch agos_sim sim.launch.py
    ros2 launch agos_sim sim.launch.py world:=agos_sewer.sdf gz_gui:=false

agos.launch.py starts this together with RTAB-Map, Nav2 and the explorer.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
import xacro


def _robot_state_publisher(context, *args, **kwargs):
    """robot_state_publisher with the URDF built for the chosen camera settings."""
    xacro_file = os.path.join(get_package_share_directory('agos_sim'), 'urdf', 'agos_urdf.xacro')
    mappings = {name: LaunchConfiguration(name).perform(context)
                for name in ('camera_width', 'camera_height', 'camera_rate')}
    robot_desc = xacro.process_file(xacro_file, mappings=mappings).toxml()
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )]


def generate_launch_description():
    pkg_path = get_package_share_directory('agos_sim')
    bridge_config = os.path.join(pkg_path, 'config', 'bridge.yaml')
    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_path, 'rviz', 'agos.rviz')

    # ---------- Launch arguments ----------
    args = [
        DeclareLaunchArgument(
            'world', default_value='agos_sewer_network.sdf',
            description='World file name inside agos_sim/worlds: '
                        'agos_sewer_network.sdf (the default: 248 m of twisting 900 mm '
                        'pipe between 5 manholes), agos_sewer.sdf (the smaller sewer), '
                        'agos_pipe.sdf (your CAD pipe at 900 mm) or agos_world.sdf '
                        '(open ground). It must load the Sensors and IMU systems '
                        'or the sensors stay silent.'),
        # Default spawn pose: the world origin facing +x. That is the middle of
        # manhole M1 in agos_sewer_network.sdf, the middle of chamber A in
        # agos_sewer.sdf and the middle of the longest straight in
        # agos_pipe.sdf. z puts the tracks just above the floor (0.124 m in
        # the chambers, about 0.12 m where the tracks touch the 900 mm pipe).
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.14'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'gz_gui', default_value='true',
            description='Open the Gazebo window. false = run the simulation without '
                        'it (the sensors still work, and everything else runs faster); '
                        'watch the robot in RViz instead'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Old name for gz_gui:=false'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='Odometry from the EKF (track speed + IMU turn rate, '
                        'config/ekf.yaml). false = raw track odometry, which '
                        'over-counts turns in pipe bends.'),
        DeclareLaunchArgument(
            'camera_width', default_value='320',
            description='RGB-D camera image width (640 for the old, sharper camera)'),
        DeclareLaunchArgument(
            'camera_height', default_value='240',
            description='RGB-D camera image height (480 for the old camera)'),
        DeclareLaunchArgument(
            'camera_rate', default_value='10',
            description='RGB-D camera frames per second (15 for the old camera)'),
    ]

    # ---------- Gazebo ----------
    # Lets Gazebo resolve model://agos_sim/meshes/... (the pipe in the world
    # file, and the robot meshes: package:// URIs in the URDF become model://)
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_path))

    # Without the window: server only; --headless-rendering keeps the
    # camera and LiDAR working without a display
    gz_flags = PythonExpression(
        ["'-r -s --headless-rendering' if '", LaunchConfiguration('gz_gui'),
         "'.lower() == 'false' or '", LaunchConfiguration('headless'),
         "'.lower() == 'true' else '-r'"])
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

    # ---------- Odometry ----------
    # The DiffDrive track odometry arrives on /wheel/odom (config/bridge.yaml).
    # ekf:=true  -> the EKF fuses it with the IMU gyro and publishes /odom and
    #               the odom -> base_footprint TF. imu_level gives it the gyro
    #               with the turn rate corrected for the robot's tilt.
    # ekf:=false -> /odom and the TF come straight from DiffDrive.
    use_ekf = IfCondition(LaunchConfiguration('ekf'))
    no_ekf = UnlessCondition(LaunchConfiguration('ekf'))
    imu_level = Node(
        package='agos_sim',
        executable='imu_level',
        parameters=[{'use_sim_time': True}],
        condition=use_ekf,
        output='screen'
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odom')],
        condition=use_ekf,
        output='screen'
    )
    # DiffDrive publishes its TF on /model/agos/tf_odom_raw (see agos_tracks.xacro)
    raw_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='raw_odom_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/model/agos/tf_odom_raw@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/agos/tf_odom_raw', '/tf')],
        parameters=[{'use_sim_time': True}],
        condition=no_ekf,
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
        OpaqueFunction(function=_robot_state_publisher),
        gz_resource_path,
        gazebo,
        spawn_entity,
        bridge,
        imu_level,
        ekf,
        raw_odom_bridge,
        rviz,
    ])
