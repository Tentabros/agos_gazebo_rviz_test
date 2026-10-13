import os
import tempfile
import xml.etree.ElementTree as ET

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


def _merge(base, extra):
    """Recursively copy the values in extra over base."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


def _bt_without_spin(name):
    """Copy one of Nav2's stock behaviour trees without its Spin recovery.

    Spinning on the spot is fine in a room but not in a pipe: the tracks climb
    the curved wall. The other recoveries (clear costmaps, wait, back up) stay.
    Returns the new file, or None to keep Nav2's default.
    """
    try:
        src = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                           'behavior_trees', name)
        tree = ET.parse(src)
    except (OSError, ET.ParseError, LookupError):
        return None
    for parent in list(tree.iter()):
        for child in list(parent):
            if child.tag == 'Spin':
                parent.remove(child)
    out = os.path.join(tempfile.gettempdir(), 'agos_' + name)
    tree.write(out)
    return out


def _write_nav2_params(nav2_bringup_dir):
    """Nav2's stock params with config/nav2_agos.yaml merged over them."""
    with open(os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'), 'r') as f:
        params = yaml.safe_load(f)
    agos = os.path.join(get_package_share_directory('agos_sim'), 'config', 'nav2_agos.yaml')
    with open(agos, 'r') as f:
        _merge(params, yaml.safe_load(f))

    # The Gazebo bridge and DiffDrive use plain geometry_msgs/Twist on /cmd_vel.
    # Newer Nav2 publishes TwistStamped by default, which the bridge ignores.
    def unstamp(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key == 'enable_stamped_cmd_vel':
                    node[key] = False
                else:
                    unstamp(value)
    unstamp(params)
    params.setdefault('/**', {}).setdefault('ros__parameters', {})[
        'enable_stamped_cmd_vel'] = False

    bt = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})
    for key, name in (('default_nav_to_pose_bt_xml',
                       'navigate_to_pose_w_replanning_and_recovery.xml'),
                      ('default_nav_through_poses_bt_xml',
                       'navigate_through_poses_w_replanning_and_recovery.xml')):
        tree = _bt_without_spin(name)
        if tree:
            bt[key] = tree

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
