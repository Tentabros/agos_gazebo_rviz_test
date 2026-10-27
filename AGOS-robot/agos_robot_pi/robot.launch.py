"""
The AGOS test robot's Raspberry Pi 4: sensors, odometry, SLAM and the motors.

Nav2, the explorer and RViz run on the laptop (agos_robot_laptop, started
with ./agos_laptop start). This file runs the rest on the Pi:

    zenoh router     listens on port 7447 on every address (the Ethernet tether's
                     10.77.0.2, and Wi-Fi's); the laptop's router connects to it
    robot_state_publisher   the robot's shape and sensor positions (config/robot.yaml)
    orbbec_camera    the Gemini 335: colour, depth aligned to colour, IMU
    sllidar_node     the RPLiDAR C1 on /scan
    imu_filter_madgwick     the camera's IMU with an orientation, /imu/data
    icp_odometry     odom -> base_footprint from the C1's scans (the TT motors
                     have no encoders), started level by the IMU
    rgbd_sync        colour + depth + calibration as one /camera/rgbd_image
    rtabmap          SLAM: /map and map -> odom, the same settings as the
                     simulation's (practice_agos slam_nav.launch.py)
    odom_gps         the odometry as a GPS fix (Rtabmap/LoopGPS, as in the sim)
    uros_relay x2    /cmd_vel to the ESP32 and its heartbeat back (see uros_relay.py)
    robot_monitor    CPU, temperature, RTAB-Map timing, sensor rates and the ESP32:
                     /agos_robot/status, and a CSV file per run in logs/

The micro-ROS agent, which talks to the ESP32 over USB (or Wi-Fi), runs in
its own Docker container: ./agos_robot start starts it before this file.

Start it with ./agos_robot start, which runs it in Docker when ROS 2 Lyrical
is not installed on the Pi. By hand, with everything installed:

    RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 launch ./robot.launch.py
    RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 launch ./robot.launch.py localization:=true

What crosses the network (only topics the laptop subscribes to):
    Pi -> laptop  /scan, /tf, /tf_static, /odom, /map, /mapPath, /robot_description,
                  and the camera only while RViz shows it
    laptop -> Pi  /cmd_vel (Nav2 or teleop)
"""
import math
import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, \
    OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

# The ROS 2 domain of the micro-ROS side only (the agent and uros_relay's DDS
# half): ROS_DOMAIN_ID in esp32_motor_node/include/config.h. Not 0, so that
# nobody else's ROS 2 on the same Wi-Fi can drive the robot.
UROS_DOMAIN_ID = '77'
RELAY_PORT = 7550                 # localhost UDP, between the two relay halves

# Gemini 335 (gemini_330_series.launch.py), as set up in the sensor guide:
# MJPG colour (the camera's own JPEG also goes out on .../compressed), depth
# aligned to colour inside the camera, the in-camera noise filter only, IMU
# at 200 Hz, camera clock mapped onto the Pi's.
GEMINI_335 = {
    'camera_name': 'camera',
    'enable_color': 'true', 'color_width': '848', 'color_height': '480',
    'color_fps': '15', 'color_format': 'MJPG',
    'enable_depth': 'true', 'depth_width': '848', 'depth_height': '480', 'depth_fps': '15',
    'depth_registration': 'true', 'align_mode': 'HW',
    'enable_point_cloud': 'false', 'enable_colored_point_cloud': 'false',
    'enable_accel': 'true', 'enable_gyro': 'true', 'enable_sync_output_accel_gyro': 'true',
    'accel_rate': '200hz', 'accel_range': '4g', 'gyro_rate': '200hz', 'gyro_range': '1000dps',
    'enable_laser': 'true', 'enable_ldp': 'true',
    'device_preset': 'Default',
    'enable_hardware_noise_removal_filter': 'true', 'enable_noise_removal_filter': 'false',
    'enable_spatial_filter': 'false', 'enable_temporal_filter': 'false',
    'interleave_frame_enable': 'false', 'enable_hdr_merge': 'false',
    'time_domain': 'global', 'enable_frame_sync': 'true',
    'publish_tf': 'true', 'enable_publish_extrinsic': 'true', 'sync_mode': 'standalone',
}
# ir:=true adds both IR cameras (more USB and CPU load; RTAB-Map does not use them)
GEMINI_IR = {
    'enable_left_ir': 'true', 'left_ir_width': '848', 'left_ir_height': '480',
    'left_ir_fps': '15',
    'enable_right_ir': 'true', 'right_ir_width': '848', 'right_ir_height': '480',
    'right_ir_fps': '15',
}

C1 = {
    'channel_type': 'serial', 'serial_port': '/dev/rplidar', 'serial_baudrate': 460800,
    'frame_id': 'laser', 'inverted': False, 'angle_compensate': True, 'scan_mode': 'Standard',
}

# Shared by icp_odometry and rtabmap: ICP tuned for the C1 (sensor guide)
ICP_PARAMS = {
    'Reg/Force3DoF': 'true',
    'Icp/Strategy': '0',
    'Icp/VoxelSize': '0.05',
    'Icp/MaxCorrespondenceDistance': '0.1',
}

# The same as RTABMAP_PARAMS in practice_agos/agos_sim/launch/slam_nav.launch.py
# (see the reasons there), except:
#   - the camera comes as one RGB-D message (rgbd_sync here, on the Pi),
#   - odometry is icp_odometry's /odom topic with its own covariance (the
#     simulation used the EKF's TF with fixed variances, because the EKF
#     reported a variance of 1e-9),
#   - the real clock (no use_sim_time), and the C1's ICP settings above,
#   - Grid/RangeMin: the C1 sees bits of the robot itself.
RTABMAP_PARAMS = {
    'frame_id': 'base_footprint',
    'subscribe_depth': False,
    'subscribe_rgbd': True,
    'rgbd_cameras': 1,
    'subscribe_scan': True,
    'subscribe_odom_info': True,
    'approx_sync': True,
    'sync_queue_size': 10,
    'topic_queue_size': 2,
    'odom_sensor_sync': True,
    'wait_for_transform': 0.2,
    'Reg/Strategy': '1',
    'RGBD/NeighborLinkRefining': 'false',
    'RGBD/ProximityBySpace': 'false',
    'Grid/Sensor': '0',
    'Grid/3D': 'false',
    'Grid/RayTracing': 'true',
    'Grid/RangeMax': '10.0',
    'Grid/RangeMin': '0.2',
    'GridGlobal/FootprintRadius': '0.15',   # the test robot is smaller than AGOS (0.25)
    'Optimizer/GravitySigma': '0',
    'Rtabmap/LoopGPS': 'true',
    'RGBD/LocalRadius': '12',
    'RGBD/OptimizeMaxError': '5',
}
RTABMAP_PARAMS.update(ICP_PARAMS)

# Mapping: start a new map every time (the old database is deleted).
MAPPING_PARAMS = {
    'Mem/IncrementalMemory': 'true',
    'Mem/InitWMWithAllNodes': 'false',
}
# Localization: load the last map and only work out where the robot is.
LOCALIZATION_PARAMS = {
    'Mem/IncrementalMemory': 'false',
    'Mem/InitWMWithAllNodes': 'true',
}
# stationary:=true: add a map node every update even when the robot stands
# still (by default only after it moves 10 cm or turns 0.1 rad)
STATIONARY_PARAMS = {
    'RGBD/LinearUpdate': '0',
    'RGBD/AngularUpdate': '0',
}


def _zenoh_listen(port):
    """Return the endpoint the Zenoh router listens on: every address, IPv6 and IPv4 if it can."""
    # rmw_zenoh's own default is tcp/[::]:7447, which fails outright where
    # IPv6 is switched off (ipv6.disable=1, which some Pi setups use)
    try:
        with socket.socket(socket.AF_INET6, socket.SOCK_STREAM) as s:
            s.bind(('::', 0))
        return 'tcp/[::]:%d' % port
    except OSError:
        return 'tcp/0.0.0.0:%d' % port


def _true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def load_robot(path):
    """Return config/robot.yaml as a dict, with every value checked."""
    with open(path) as f:
        robot = yaml.safe_load(f)
    for key in ('wheel_radius', 'wheel_separation'):
        robot[key] = float(robot[key])
    for part, keys in (('chassis', ('length', 'width', 'height', 'x')),
                       ('laser', ('x', 'y', 'z', 'yaw')),
                       ('camera', ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))):
        robot[part] = {k: float(robot[part].get(k, 0.0)) for k in keys}
    return robot


def robot_urdf(robot):
    """Return the test robot as a URDF: a box, two wheels, the C1 and the camera mount."""
    r, sep, ch = robot['wheel_radius'], robot['wheel_separation'], robot['chassis']
    las, cam = robot['laser'], robot['camera']

    def joint(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
        return ('<joint name="%s" type="fixed"><parent link="%s"/><child link="%s"/>'
                '<origin xyz="%g %g %g" rpy="%g %g %g"/></joint>'
                % ((name, parent, child) + tuple(xyz) + tuple(rpy)))

    def link(name, geometry='', rgba='0.8 0.7 0.1 1', xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
        if not geometry:
            return '<link name="%s"/>' % name
        return ('<link name="%s"><visual><origin xyz="%g %g %g" rpy="%g %g %g"/>'
                '<geometry>%s</geometry><material name="%s_colour"><color rgba="%s"/>'
                '</material></visual></link>'
                % ((name,) + tuple(xyz) + tuple(rpy) + (geometry, name, rgba)))

    wheel = '<cylinder radius="%g" length="0.026"/>' % r
    parts = [
        link('base_footprint'),
        # base_link: on the axle, straight above base_footprint (Nav2 steers
        # it, so it must be the point the robot turns about); the chassis box
        # is drawn ch['x'] ahead of it
        link('base_link', '<box size="%g %g %g"/>' % (ch['length'], ch['width'], ch['height']),
             '0.2 0.4 0.8 1', (ch['x'], 0.0, ch['height'] / 2)),
        joint('base_joint', 'base_footprint', 'base_link', (0.0, 0.0, r)),
        link('left_wheel', wheel, '0.9 0.8 0.1 1'),
        joint('left_wheel_joint', 'base_footprint', 'left_wheel', (0.0, sep / 2, r),
              (-math.pi / 2, 0.0, 0.0)),
        link('right_wheel', wheel, '0.9 0.8 0.1 1'),
        joint('right_wheel_joint', 'base_footprint', 'right_wheel', (0.0, -sep / 2, r),
              (-math.pi / 2, 0.0, 0.0)),
        link('laser', '<cylinder radius="0.028" length="0.04"/>', '0.1 0.1 0.1 1'),
        joint('laser_joint', 'base_footprint', 'laser', (las['x'], las['y'], las['z']),
              (0.0, 0.0, las['yaw'])),
        # camera_link and everything under it come from the camera driver
        link('camera_link'),
        joint('camera_joint', 'base_footprint', 'camera_link', (cam['x'], cam['y'], cam['z']),
              (cam['roll'], cam['pitch'], cam['yaw'])),
    ]
    return '<?xml version="1.0"?><robot name="agos_test_robot">%s</robot>' % ''.join(parts)


def _launch_setup(context, *args, **kwargs):
    localization = _true(context, 'localization')
    camera = _true(context, 'camera')
    log_level = LaunchConfiguration('log_level').perform(context)
    listen = LaunchConfiguration('listen').perform(context).strip()
    if not listen:
        listen = _zenoh_listen(int(LaunchConfiguration('port').perform(context)))
    endpoints = ','.join('"%s"' % e.strip() for e in listen.split(',') if e.strip())
    robot = load_robot(os.path.expanduser(LaunchConfiguration('robot_config').perform(context)))
    rate = LaunchConfiguration('detection_rate').perform(context)
    database = os.path.expanduser(LaunchConfiguration('database_path').perform(context))
    os.makedirs(os.path.dirname(database), exist_ok=True)
    real_time = {'use_sim_time': False}
    actions = []

    # 1. The Zenoh router every node here connects to, and the laptop too
    actions.append(ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        additional_env={'ZENOH_CONFIG_OVERRIDE': 'listen/endpoints=[%s]' % endpoints},
        output='screen',
        name='zenoh_router',
    ))

    # 2. Where everything sits on the robot (static transforms) and its
    #    shape for RViz on the laptop
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[real_time, {'robot_description': robot_urdf(robot)}],
    ))

    # 3. Sensors
    if camera:
        gemini = dict(GEMINI_335)
        if _true(context, 'ir'):
            gemini.update(GEMINI_IR)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orbbec_camera'), 'launch',
                'gemini_330_series.launch.py')),
            launch_arguments=gemini.items()))
    actions.append(Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[real_time, C1],
    ))

    # 4. Odometry from the LiDAR, with the camera's IMU when there is one
    if camera:
        actions.append(Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[real_time, {'use_mag': False, 'world_frame': 'enu',
                                    'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/gyro_accel/sample'),
                        ('imu/data', '/imu/data')],
        ))
    odom_params = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'wait_imu_to_init': camera,
        'wait_for_transform': 0.2,
    }
    odom_params.update(ICP_PARAMS)
    actions.append(Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[real_time, odom_params],
        remappings=[('scan', '/scan'), ('imu', '/imu/data'), ('odom', '/odom')],
        arguments=['--ros-args', '--log-level', log_level],
    ))

    # 5. SLAM
    params = dict(RTABMAP_PARAMS)
    params.update(real_time)
    params.update(LOCALIZATION_PARAMS if localization else MAPPING_PARAMS)
    if _true(context, 'stationary'):
        params.update(STATIONARY_PARAMS)
    params['Rtabmap/DetectionRate'] = rate
    params['database_path'] = database
    remappings = [('scan', '/scan'), ('odom', '/odom'), ('odom_info', '/odom_info')]
    if camera:
        actions.append(Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[real_time, {'approx_sync': True, 'approx_sync_max_interval': 0.05,
                                    'compressed_rate': 5.0}],
            remappings=[('rgb/image', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image', '/camera/depth/image_raw'),
                        ('rgbd_image', '/camera/rgbd_image'),
                        ('rgbd_image/compressed', '/camera/rgbd_image/compressed')],
        ))
        remappings.append(('rgbd_image', '/camera/rgbd_image'))
    else:
        params['subscribe_rgbd'] = False
    if localization:
        # The fixes in a saved map come from that run's odometry: with
        # Rtabmap/LoopGPS they would stop RTAB-Map finding the robot in it.
        remappings.append(('gps/fix', 'gps/fix_unused'))
    else:
        actions.append(ExecuteProcess(
            cmd=['python3', os.path.join(HERE, 'odom_gps.py')],
            output='screen',
            name='odom_gps',
        ))
    actions.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[params],
        remappings=remappings,
        # mapping: delete the previous database on start
        arguments=([] if localization else ['-d']) + ['--ros-args', '--log-level', log_level],
        # on Ctrl+C it saves the map to the database: give it time
        sigterm_timeout='30',
    ))

    # 6. The motors: /cmd_vel from Zenoh to the micro-ROS agent's DDS
    if _true(context, 'motors'):
        relay = os.path.join(HERE, 'uros_relay.py')
        actions.append(ExecuteProcess(
            cmd=['python3', relay, '--side', 'zenoh', '--port', str(RELAY_PORT)],
            output='screen',
            name='uros_relay_zenoh',
        ))
        actions.append(ExecuteProcess(
            cmd=['python3', relay, '--side', 'dds', '--port', str(RELAY_PORT)],
            additional_env={
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'ROS_DOMAIN_ID': UROS_DOMAIN_ID,
                # UDP only: Fast DDS shared memory fails between the agent's
                # container (root) and this one (your user)
                'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
            },
            output='screen',
            name='uros_relay_dds',
        ))

    # 7. How the Pi keeps up
    log_dir = os.path.expanduser(LaunchConfiguration('log_dir').perform(context))
    actions.append(ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'robot_monitor.py'),
             '--ros-args', '-p', 'log_dir:=' + log_dir,
             '-p', 'detection_rate:=%f' % float(rate),
             '-p', 'camera:=%s' % ('true' if camera else 'false')],
        output='screen',
        name='robot_monitor',
        sigterm_timeout='10',
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='false',
                              description='false = build a new map (deletes the old one). '
                                          'true = reuse the last map and only localize in it'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.join(HERE, 'maps', 'rtabmap.db'),
                              description='Where RTAB-Map keeps its map'),
        DeclareLaunchArgument('detection_rate', default_value='1',
                              description='RTAB-Map updates a second (Rtabmap/DetectionRate)'),
        DeclareLaunchArgument('stationary', default_value='false',
                              description='true = keep adding map nodes while the robot '
                                          'stands still'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='false = LiDAR only (no Gemini 335, no IMU)'),
        DeclareLaunchArgument('ir', default_value='false',
                              description='true = also stream both IR cameras'),
        DeclareLaunchArgument('motors', default_value='true',
                              description='false = no relay to the ESP32 (sensors and SLAM only)'),
        DeclareLaunchArgument('robot_config',
                              default_value=os.path.join(HERE, 'config', 'robot.yaml'),
                              description='Where the sensors sit on the robot'),
        DeclareLaunchArgument('port', default_value='7447',
                              description="The Zenoh router's port (the laptop connects to it)"),
        DeclareLaunchArgument('listen', default_value='',
                              description='Zenoh endpoints to listen on instead, comma '
                                          'separated (default tcp/[::]:<port>)'),
        DeclareLaunchArgument('log_dir', default_value=os.path.join(HERE, 'logs'),
                              description="Where robot_monitor writes each run's CSV file"),
        DeclareLaunchArgument('log_level', default_value='info',
                              description="RTAB-Map's log level"),
        OpaqueFunction(function=_launch_setup),
    ])
