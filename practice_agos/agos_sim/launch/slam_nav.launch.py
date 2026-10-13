import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Run this AFTER sim.launch.py is up (it provides /clock, sensors, odom and TF).

# Rectangle around the 0.711 x 0.381 m body in agos_urdf.xacro (base_link
# frame). Nav2's stock params assume a 0.22 m round TurtleBot; a circle around
# AGOS would need a 0.40 m radius, wider than the pipe it drives in.
FOOTPRINT = '[[0.356, 0.191], [0.356, -0.191], [-0.356, -0.191], [-0.356, 0.191]]'

# Camera topics as named in config/bridge.yaml
RTABMAP_REMAPPINGS = [
    ('rgb/image', '/camera/image_raw'),
    ('rgb/camera_info', '/camera/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
    ('scan', '/scan'),
    ('odom', '/odom'),
]

RTABMAP_PARAMS = {
    'frame_id': 'base_footprint',
    'use_sim_time': True,
    'subscribe_depth': True,
    'subscribe_scan': True,
    # Camera (15 Hz), LiDAR (10 Hz) and odom (50 Hz) never share a timestamp,
    # so they must be matched approximately or rtabmap never gets a full set.
    'approx_sync': True,
    'sync_queue_size': 30,
    'topic_queue_size': 10,
    'odom_sensor_sync': True,
    # RTAB-Map's own parameters must be strings
    'Reg/Strategy': '1',               # ICP on the LiDAR scan
    'Reg/Force3DoF': 'true',           # ground robot: x, y, yaw only
    'RGBD/NeighborLinkRefining': 'true',
    'Grid/Sensor': '0',                # occupancy grid from the LiDAR (was Grid/FromDepth)
    'Grid/3D': 'false',
    'Grid/RayTracing': 'true',
    'Grid/RangeMax': '10.0',
    'Optimizer/GravitySigma': '0',     # no IMU constraints, we are already 2D
}


def _write_nav2_params(nav2_bringup_dir):
    """Copy Nav2's stock params with the few values AGOS needs changed."""
    src = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    with open(src, 'r') as f:
        params = yaml.safe_load(f)

    def patch(node):
        if isinstance(node, dict):
            if 'robot_radius' in node:   # a costmap: use the real outline
                node['footprint'] = FOOTPRINT
            for key, value in node.items():
                if key == 'enable_stamped_cmd_vel':
                    node[key] = False
                else:
                    patch(value)

    patch(params)
    # The Gazebo bridge and DiffDrive use plain geometry_msgs/Twist on /cmd_vel.
    # Newer Nav2 publishes TwistStamped by default, which the bridge ignores.
    params.setdefault('/**', {}).setdefault('ros__parameters', {})[
        'enable_stamped_cmd_vel'] = False

    out = os.path.join(tempfile.gettempdir(), 'agos_nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return out


def _launch_nav2(context, *args, **kwargs):
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # navigation_launch.py = planner, controller, behaviors, BT navigator...
    # It does NOT start map_server or AMCL: RTAB-Map already provides
    # /map and the map -> odom transform.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': _write_nav2_params(nav2_bringup_dir),
        }.items()
    )
    # Give RTAB-Map a few seconds to start publishing map -> odom first,
    # otherwise the costmaps sit waiting for that transform during activation.
    return [TimerAction(period=5.0, actions=[nav2])]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('nav2', default_value='true',
                              description='Start Nav2 on top of RTAB-Map'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true',
                              description='Start the RTAB-Map visualizer'),
    ]

    # 1. RTAB-Map SLAM node (publishes /map and map -> odom)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[RTABMAP_PARAMS],
        remappings=RTABMAP_REMAPPINGS,
        arguments=['-d']   # delete the previous database (~/.ros/rtabmap.db) on start
    )

    # 2. RTAB-Map visualizer (same inputs as the SLAM node)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[RTABMAP_PARAMS],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    # 3. Nav2 navigation stack only (lifecycle_manager_navigation)
    nav2 = OpaqueFunction(function=_launch_nav2,
                          condition=IfCondition(LaunchConfiguration('nav2')))

    return LaunchDescription(args + [
        rtabmap_slam,
        rtabmap_viz,
        nav2,
    ])
