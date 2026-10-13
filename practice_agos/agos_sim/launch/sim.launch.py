import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            'world', default_value='agos_world.sdf',
            description='World file name inside agos_sim/worlds. It must load the '
                        'Sensors and IMU systems or the sensors stay silent.'),
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
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r ', PathJoinSubstitution(
                [pkg_path, 'worlds', LaunchConfiguration('world')])],
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'agos',
                   '-z', '0.2'],
        output='screen'
    )

    # ---------- Bridges ----------
    # All topics (clock, cmd_vel, odom, joint_states, scan, imu, camera) live in config/bridge.yaml
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # SENSOR BRIDGES ADDED BELOW:
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked', # <-- ADD THIS LINE
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
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
        gazebo,
        spawn_entity,
        bridge,
        odom_tf_bridge,
        rviz,
    ])