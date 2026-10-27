"""
RTAB-Map SLAM and Nav2 for AGOS.

Run it after sim.launch.py, which provides /clock, the sensors, odometry and
TF (or run agos.launch.py, which starts both):

    ros2 launch agos_sim slam_nav.launch.py
    ros2 launch agos_sim slam_nav.launch.py localization:=true

Nav2 is started here node by node instead of through nav2_bringup's
navigation_launch.py, so the lifecycle manager's list is exactly the nodes
that run, and the velocity commands take one fixed route to the robot:

    controller_server  -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> Gazebo
    behavior_server    -> /cmd_vel_nav -'

navigation_launch.py puts collision_monitor between velocity_smoother and
/cmd_vel. Whenever collision_monitor was not active (it is not in the node
list from the Fix NAV notes) nothing reached /cmd_vel, which is why the
topic_tools relay from /cmd_vel_nav to /cmd_vel made the robot move. It is
left out here; the costmaps already keep the robot off the walls.

Nav2 starts once RTAB-Map publishes the map -> odom transform (see
nav_supervisor.py), instead of after a fixed 5 s delay.

With slam:=remote RTAB-Map runs on another computer, a Raspberry Pi
(agos_pi_slam, a separate download), and this computer sends it the sensors:

    ros2 launch agos_sim slam_nav.launch.py slam:=remote slam_host:=192.168.1.42

Everything here then talks through Zenoh (RMW_IMPLEMENTATION=rmw_zenoh_cpp,
which ./agos sets): a Zenoh router started here connects to the one on the
Pi, and only the topics the other side subscribes to cross the network. The
camera images go as one compressed RGB-D message (rgbd_sync below), a few
times a second: about 1 MB/s instead of 6.
"""
import copy
import os
import socket
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile
import yaml

# Camera topics as named in config/bridge.yaml
RTABMAP_REMAPPINGS = [
    ('rgb/image', '/camera/image_raw'),
    ('rgb/camera_info', '/camera/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
    ('scan', '/scan'),
]

RTABMAP_PARAMS = {
    'frame_id': 'base_footprint',
    'subscribe_depth': True,
    'subscribe_scan': True,
    # Odometry from the odom -> base_footprint TF (the EKF: track speed and
    # gyro turn rate), trusted this much for each step between map nodes.
    # Not from the /odom topic: the EKF gives its speed a variance of 1e-9,
    # which makes RTAB-Map so sure of the odometry that it rejects every loop
    # closure ("Rejecting all added loop closures ... graph error ratio") and
    # never corrects the drift.
    'odom_frame_id': 'odom',
    'odom_tf_linear_variance': 0.0002,     # (1.4 cm)^2 per step: track slip
    'odom_tf_angular_variance': 0.00005,   # (0.4 deg)^2 per step: the gyro is good
    # Camera (10 Hz) and LiDAR (10 Hz) never share a timestamp, so they must
    # be matched approximately or rtabmap never gets a full set.
    'approx_sync': True,
    'sync_queue_size': 10,
    'topic_queue_size': 2,
    'odom_sensor_sync': True,
    # RTAB-Map's own parameters must be strings
    'Reg/Strategy': '1',               # loop closures: ICP on the LiDAR scan
    # PCL's ICP, the one all the tests ran with. RTAB-Map built with
    # libpointmatcher (the ROS packages) would use that instead, with other
    # settings, untested here.
    'Icp/Strategy': '0',
    'Reg/Force3DoF': 'true',           # ground robot: x, y, yaw only
    # No LiDAR ICP between neighbouring map nodes or nodes close in space:
    # in a round pipe ICP cannot tell how far the robot moved along a
    # straight or round a bend, and in a test mission it turned the map 6.5
    # deg off while the gyro heading was right to 0.1 deg. Drift is corrected
    # by loop closures when the robot comes back past places it has seen.
    'RGBD/NeighborLinkRefining': 'false',
    'RGBD/ProximityBySpace': 'false',
    'Grid/Sensor': '0',                # occupancy grid from the LiDAR (was Grid/FromDepth)
    'Grid/3D': 'false',
    'Grid/RayTracing': 'true',
    'Grid/RangeMax': '10.0',
    # Where the robot has been is free on the map. Back in a pipe after a
    # long drive, the map can have drifted and RTAB-Map then draws its wall a
    # second time, a little apart; without this, that wall can cross the
    # path just driven, and Nav2 refuses to plan ("Start occupied").
    'GridGlobal/FootprintRadius': '0.25',
    'Optimizer/GravitySigma': '0',     # no IMU constraints, we are already 2D
    # Loop closures only with map nodes within 12 m of where odometry says
    # the robot is (odom_gps gives RTAB-Map the odometry as a GPS fix). The
    # pipes and manholes look alike: matches 33 m and 14 m away bent the map.
    # Back at the start after the whole network (400 m) the odometry was 9 m
    # off, and 12 m leaves room for that.
    'Rtabmap/LoopGPS': 'true',
    'RGBD/LocalRadius': '12',
    # A loop closure is undone if, after it, any link of the map is stretched
    # more than 5 standard deviations (the default is 3). Closing the whole
    # network's loop stretches the odometry links a little more than 3; a
    # wrong match (pipes 6 m apart look alike too) stretches them 20 times.
    'RGBD/OptimizeMaxError': '5',
}

# slam:=remote: the RGB-D images for RTAB-Map on the Pi, as one compressed
# message (JPEG colour, PNG depth). RTAB-Map adds a map node a second, so 5
# images a second are plenty. The Pi's launch file subscribes to
# /camera/rgbd_image/compressed.
RGBD_SYNC_PARAMS = {
    'approx_sync': False,           # Gazebo stamps colour and depth alike
    'compressed_rate': 5.0,         # Hz
}
RGBD_SYNC_REMAPPINGS = [
    ('rgb/image', '/camera/image_raw'),
    ('rgb/camera_info', '/camera/camera_info'),
    ('depth/image', '/camera/depth/image_raw'),
    ('rgbd_image', '/camera/rgbd_image'),
    ('rgbd_image/compressed', '/camera/rgbd_image/compressed'),
]
ZENOH_PORT = 7447                   # the Zenoh router's port, here and on the Pi

# Mapping: start a new map every time (the old database is deleted).
MAPPING_PARAMS = {
    'Mem/IncrementalMemory': 'true',
    'Mem/InitWMWithAllNodes': 'false',
}

# Localization: load the map saved by the last mapping run and only work
# out where the robot is in it. RViz's "2D Pose Estimate" works in this
# mode (in mapping mode RTAB-Map ignores it and says so in the terminal).
LOCALIZATION_PARAMS = {
    'Mem/IncrementalMemory': 'false',
    'Mem/InitWMWithAllNodes': 'true',
}

# The Nav2 servers this launch file starts, in start-up order. This is the
# list the lifecycle manager configures and activates.
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


def _bt_without_spin(name):
    """
    Copy one of Nav2's stock behaviour trees without its Spin recovery.

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


def _write_nav2_params():
    """Nav2's stock params with config/nav2_agos.yaml merged over them."""
    stock = os.path.join(get_package_share_directory('nav2_bringup'),
                         'params', 'nav2_params.yaml')
    with open(stock, 'r') as f:
        params = yaml.safe_load(f)
    agos = os.path.join(get_package_share_directory('agos_sim'), 'config', 'nav2_agos.yaml')
    with open(agos, 'r') as f:
        _merge(params, yaml.safe_load(f))
    # The Reverse controller is FollowPath with a few changes (see
    # nav2_agos.yaml): copy FollowPath, stock values and all, then apply them.
    controllers = params['controller_server']['ros__parameters']
    if isinstance(controllers.get('Reverse'), dict):
        reverse = copy.deepcopy(controllers['FollowPath'])
        _merge(reverse, controllers['Reverse'])
        controllers['Reverse'] = reverse

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
    # IPv6 is switched off (seen in testing; some Raspberry Pi setups do this)
    try:
        with socket.socket(socket.AF_INET6, socket.SOCK_STREAM) as s:
            s.bind(('::', 0))
        return 'tcp/[::]:%d' % port
    except OSError:
        return 'tcp/0.0.0.0:%d' % port


def _remote_slam(context):
    """slam:=remote: the Zenoh router that connects to the Pi, and the images it gets."""
    host = LaunchConfiguration('slam_host').perform(context).strip()
    if not host:
        raise RuntimeError('slam:=remote needs the address of the computer running RTAB-Map: '
                           'slam_host:=<address>')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise RuntimeError('slam:=remote talks to the Pi through Zenoh: start it with '
                           'RMW_IMPLEMENTATION=rmw_zenoh_cpp (./agos and ./agos_pi do this)')
    override = 'listen/endpoints=["%s"];connect/endpoints=["%s"]' % (
        _zenoh_listen(ZENOH_PORT), _zenoh_endpoint(host))
    return [
        # The router every node here connects to (tcp/localhost:7447). It
        # also connects to the Pi's router, and keeps trying until it can.
        ExecuteProcess(
            cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
            additional_env={'ZENOH_CONFIG_OVERRIDE': override},
            output='screen',
            name='zenoh_router',
        ),
        # A new simulation (its clock starts from 0 again): RTAB-Map on the
        # Pi starts a new map. Waits until the Pi is there.
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/agos_pi/new_simulation', 'std_srvs/srv/Trigger'],
            output='screen',
            name='new_map_on_pi',
        ),
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[RGBD_SYNC_PARAMS],
            remappings=RGBD_SYNC_REMAPPINGS,
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    localization = _true(context, 'localization')
    log_level = LaunchConfiguration('log_level')
    slam = LaunchConfiguration('slam').perform(context).strip().lower()
    if slam not in ('local', 'remote', 'none'):
        raise RuntimeError('slam:=%s: use local, remote or none' % slam)
    rtabmap_params = dict(RTABMAP_PARAMS)
    rtabmap_params.update(LOCALIZATION_PARAMS if localization else MAPPING_PARAMS)
    database = LaunchConfiguration('database_path').perform(context)
    if database:
        rtabmap_params['database_path'] = os.path.expanduser(database)

    # 1. RTAB-Map SLAM node (publishes /map and map -> odom)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=RTABMAP_REMAPPINGS,
        # mapping: delete the previous database (~/.ros/rtabmap.db) on start
        arguments=([] if localization else ['-d'])
        + ['--ros-args', '--log-level', log_level],
        # on Ctrl+C it saves the map to the database: give it time
        sigterm_timeout='30',
    )

    # 2. RTAB-Map's own window (same inputs as the SLAM node). Off by
    # default: it draws the full 3D point cloud map, and RViz already shows
    # the map, the robot and the camera.
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_params],
        remappings=RTABMAP_REMAPPINGS,
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    if slam == 'local':
        actions = [rtabmap_slam, rtabmap_viz]
    elif slam == 'remote':
        actions = _remote_slam(context)
    else:                           # none: map -> odom comes from elsewhere
        actions = []
    if slam == 'remote' or (slam == 'local' and not localization):
        # 2b. The odometry as a GPS fix in each new map node (see
        # RTABMAP_PARAMS, Rtabmap/LoopGPS). Wall clock: it only copies the
        # time of the odometry it passes on. (With slam:=remote the Pi
        # decides whether to use it.)
        actions.append(Node(
            package='agos_sim',
            executable='odom_gps',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ))
    if not _true(context, 'nav2'):
        return [GroupAction([SetParameter('use_sim_time', LaunchConfiguration('use_sim_time'))]
                            + actions)]

    # 3. Nav2 servers and their lifecycle manager. allow_substs: the stock
    # params contain $(find-pkg-share ...) paths (bt_navigator's behaviour
    # tree folder), which launch fills in.
    params_file = ParameterFile(_write_nav2_params(), allow_substs=True)
    for package, executable, remappings in NAV2_NODES:
        actions.append(Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            respawn=False,
            parameters=[params_file],
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
            # nav_supervisor (or the Startup button in RViz's Navigation 2
            # panel when autostart:=false) starts Nav2 once the map exists
            'autostart': False,
            'node_names': [executable for _, executable, _ in NAV2_NODES],
            # A busy computer can miss the stock 4 s heartbeat, which makes the
            # lifecycle manager shut every Nav2 node down. Allow 10 s.
            'bond_timeout': 10.0,
        }],
    ))

    # 4. Starts Nav2 when RTAB-Map is ready, restarts it if it goes down. It
    #    runs on the wall clock: following /clock (500 messages a second)
    #    would cost this Python node a sixth of a CPU core, and its 1-5 s
    #    checks do not need simulation time.
    actions.append(Node(
        package='agos_sim',
        executable='nav_supervisor',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('autostart')),
    ))

    return [GroupAction([SetParameter('use_sim_time', LaunchConfiguration('use_sim_time'))]
                        + actions)]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo clock (/clock)'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='Start Nav2 on top of RTAB-Map'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Start Nav2 by itself as soon as RTAB-Map has a map. '
                                          "false = press Startup in RViz's Navigation 2 panel"),
        DeclareLaunchArgument('localization', default_value='false',
                              description='false = build a new map (deletes the old one). '
                                          'true = reuse the last map and only localize in it'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db',
                              description='Where RTAB-Map keeps its map'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description="Also open RTAB-Map's own window (heavy)"),
        DeclareLaunchArgument('slam', default_value='local',
                              description='local = RTAB-Map runs here. remote = on another '
                                          'computer (a Raspberry Pi with agos_pi_slam) at '
                                          'slam_host. none = not started'),
        DeclareLaunchArgument('slam_host', default_value='',
                              description='slam:=remote: address of the Raspberry Pi '
                                          '(or a Zenoh endpoint such as tcp/192.168.1.42:7447)'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level of RTAB-Map and the Nav2 nodes'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
