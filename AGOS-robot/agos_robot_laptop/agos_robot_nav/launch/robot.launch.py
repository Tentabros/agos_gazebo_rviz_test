"""
The real AGOS test robot, from the laptop: Nav2, RViz and the explorer.

The robot's Raspberry Pi (agos_robot_pi, ./agos_robot start) runs the
Gemini 335, the RPLiDAR C1, the odometry and RTAB-Map, and drives the motors
through the ESP32. This file runs, on the laptop:

    zenoh router   connects to the Pi's router (TCP 7447): over the Ethernet
                   tether by default, robot_host:=<address> for another link
    Nav2           Nav2's stock params, the simulation's tuning over them
                   (config/nav2_base.yaml) and the test robot's size and
                   speeds over that (config/nav2_robot.yaml); it starts by
                   itself once RTAB-Map has a map (nav_supervisor)
    RViz           the map, the scan, the robot, the costmaps, the camera
    explorer       with explore:=true: frontier exploration (full mode by
                   default, which suits a room better than the sewer legs)

    ros2 launch agos_robot_nav robot.launch.py
    ros2 launch agos_robot_nav robot.launch.py robot_host:=192.168.1.42 explore:=true

Everything talks through Zenoh: start it with RMW_IMPLEMENTATION=rmw_zenoh_cpp,
or with ./agos_laptop start, which does that.

Nav2 is started node by node (as in practice_agos's slam_nav.launch.py), so
the lifecycle manager's list is exactly the nodes that run and the velocity
commands take one route to the robot:

    controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> ESP32
    behavior_server   -> /cmd_vel_nav -'
"""
import copy
import os
import socket
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
import yaml

PKG = 'agos_robot_nav'
ZENOH_PORT = 7447                   # the Zenoh router's port, here and on the Pi
TETHER_ADDRESS = '10.77.0.2'        # the Pi on the Ethernet tether (./agos_robot tether)

# The Nav2 servers this file starts, in start-up order. This is the list the
# lifecycle manager configures and activates.
NAV2_NODES = [
    # (package, executable, remappings)
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_smoother', 'smoother_server', []),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_bt_navigator', 'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower', []),
    ('nav2_velocity_smoother', 'velocity_smoother',
     [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
]


def _merge(base, extra):
    """Recursively copy the values in extra over base."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


def _write_nav2_params(pkg):
    """Nav2's stock params with config/nav2_base.yaml, then config/nav2_robot.yaml, over them."""
    stock = os.path.join(get_package_share_directory('nav2_bringup'),
                         'params', 'nav2_params.yaml')
    with open(stock, 'r') as f:
        params = yaml.safe_load(f)
    for name in ('nav2_base.yaml', 'nav2_robot.yaml'):
        with open(os.path.join(pkg, 'config', name), 'r') as f:
            _merge(params, yaml.safe_load(f))
    # The Reverse controller is FollowPath with a few changes (see
    # nav2_base.yaml): copy FollowPath, stock values and all, then apply them.
    controllers = params['controller_server']['ros__parameters']
    if isinstance(controllers.get('Reverse'), dict):
        reverse = copy.deepcopy(controllers['FollowPath'])
        _merge(reverse, controllers['Reverse'])
        controllers['Reverse'] = reverse

    # The ESP32 firmware subscribes to plain geometry_msgs/Twist on /cmd_vel.
    # Newer Nav2 publishes TwistStamped by default, which it would not get.
    # And a real robot runs on the wall clock: the stock file says
    # use_sim_time: True for most nodes, the costmaps' too.
    def real_robot(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key in ('enable_stamped_cmd_vel', 'use_sim_time'):
                    node[key] = False
                else:
                    real_robot(value)
    real_robot(params)
    params.setdefault('/**', {}).setdefault('ros__parameters', {}).update(
        {'enable_stamped_cmd_vel': False, 'use_sim_time': False})

    out = os.path.join(tempfile.gettempdir(), 'agos_robot_nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return out


def _true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _zenoh_endpoint(host):
    """Return the Zenoh endpoint of the Pi's router: an address, or a whole endpoint."""
    if '/' in host:                 # already an endpoint, e.g. tcp/192.168.1.42:7450
        return host
    if ':' in host and not host.startswith('['):
        if host.count(':') == 1:    # address:port
            return 'tcp/' + host
        host = '[%s]' % host        # IPv6
    return 'tcp/%s:%d' % (host, ZENOH_PORT)


def _zenoh_listen(port):
    """Return the endpoint a Zenoh router listens on: every address, IPv6 and IPv4 if it can."""
    # rmw_zenoh's own default is tcp/[::]:7447, which fails outright where
    # IPv6 is switched off
    try:
        with socket.socket(socket.AF_INET6, socket.SOCK_STREAM) as s:
            s.bind(('::', 0))
        return 'tcp/[::]:%d' % port
    except OSError:
        return 'tcp/0.0.0.0:%d' % port


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory(PKG)
    log_level = LaunchConfiguration('log_level')
    host = LaunchConfiguration('robot_host').perform(context).strip() or TETHER_ADDRESS
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('The robot talks through Zenoh: start this with '
                           'RMW_IMPLEMENTATION=rmw_zenoh_cpp (./agos_laptop start does)')

    # 1. The router every node here connects to (tcp/localhost:7447). It also
    #    connects to the Pi's router, and keeps trying until it can.
    override = 'listen/endpoints=["%s"];connect/endpoints=["%s"]' % (
        _zenoh_listen(ZENOH_PORT), _zenoh_endpoint(host))
    actions = [ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        additional_env={'ZENOH_CONFIG_OVERRIDE': override},
        output='screen',
        name='zenoh_router',
    )]

    # 2. RViz
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'robot.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))

    if _true(context, 'nav2'):
        # 3. Nav2 servers and their lifecycle manager. allow_substs: the stock
        #    params contain $(find-pkg-share ...) paths (bt_navigator's
        #    behaviour tree folder), which launch fills in.
        params_file = ParameterFile(_write_nav2_params(pkg), allow_substs=True)
        for package, executable, remappings in NAV2_NODES:
            actions.append(Node(
                package=package,
                executable=executable,
                name=executable,
                output='screen',
                respawn=False,
                parameters=[params_file, {'use_sim_time': False}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ))
        actions.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': False,
                # nav_supervisor (or the Startup button in RViz's Navigation 2
                # panel when autostart:=false) starts Nav2 once the map exists
                'autostart': False,
                'node_names': [executable for _, executable, _ in NAV2_NODES],
                # Wi-Fi or a busy laptop can miss the stock 4 s heartbeat,
                # which makes the lifecycle manager shut every Nav2 node down
                'bond_timeout': 10.0,
            }],
        ))
        # 4. Starts Nav2 once RTAB-Map on the Pi has a map, restarts it if it
        #    goes down
        actions.append(Node(
            package=PKG,
            executable='nav_supervisor',
            output='screen',
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('autostart')),
        ))

    # 5. Exploration, sized for the test robot in a room: smaller steps and
    #    margins than in the sewer (explorer.py has the meaning of each)
    actions.append(Node(
        package=PKG,
        executable='explorer',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'mode': LaunchConfiguration('explore_mode'),
                     'leg_length': ParameterValue(LaunchConfiguration('leg_length'),
                                                  value_type=float),
                     'forward_only': ParameterValue(LaunchConfiguration('forward_only'),
                                                    value_type=bool),
                     'exit_radius': 1.0,
                     'min_frontier_size': 0.2,
                     'clearance': 0.12,
                     'hold_radius': 0.5,
                     'skip_radius': 0.5,
                     'reached_radius': 0.2,
                     'passed_radius': 0.4,
                     'passed_margin': 1.5,
                     'reach': 1.0,
                     'peek': 3.0,
                     'hop_length': 4.0,
                     'report_dir': os.environ.get('AGOS_MISSION_DIR', '~/agos_missions')}],
        condition=IfCondition(LaunchConfiguration('explore')),
    ))
    return [GroupAction([SetParameter('use_sim_time', False)] + actions)]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_host', default_value='',
                              description="The robot's Raspberry Pi: its address (default "
                                          '%s, the Ethernet tether) or a Zenoh endpoint '
                                          'such as tcp/192.168.1.42:7447' % TETHER_ADDRESS),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('nav2', default_value='true', description='Start Nav2'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Start Nav2 by itself once the map exists. '
                                          "false = press Startup in RViz's Navigation 2 panel"),
        DeclareLaunchArgument('explore', default_value='false',
                              description='Run the autonomous exploration'),
        DeclareLaunchArgument('explore_mode', default_value='full',
                              description='full = map everything it can reach, then come back; '
                                          "legs = the proposal's mission (leg_length each way)"),
        DeclareLaunchArgument('leg_length', default_value='5.0',
                              description='legs mode: how far (m) to explore each way'),
        DeclareLaunchArgument('forward_only', default_value='true',
                              description='Drive into unmapped space forwards only (the camera '
                                          'looks ahead)'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level of the Nav2 nodes'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
