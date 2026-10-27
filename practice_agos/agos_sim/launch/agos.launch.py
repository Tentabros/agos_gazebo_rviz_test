"""
The whole AGOS simulation in one command.

    ros2 launch agos_sim agos.launch.py                  # drive it yourself (2D Goal Pose, teleop)
    ros2 launch agos_sim agos.launch.py explore:=true    # the autonomous 25 m mission
    ros2 launch agos_sim agos.launch.py explore:=true explore_mode:=full   # map the whole network
    ros2 launch agos_sim agos.launch.py gz_gui:=false    # no Gazebo window: faster, watch in RViz
    ros2 launch agos_sim agos.launch.py slam:=remote slam_host:=192.168.1.42
        # RTAB-Map on a Raspberry Pi (agos_pi_slam); needs RMW_IMPLEMENTATION=rmw_zenoh_cpp

It starts sim.launch.py (Gazebo, the robot, the sensor bridge, odometry,
RViz), slam_nav.launch.py (RTAB-Map and Nav2, which starts itself once the
map exists) and, with explore:=true, the explorer (agos_sim/explorer.py).
Every argument of those two files can be given here too.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, description) of the arguments passed on to the other files
SIM_ARGS = [
    ('world', 'agos_sewer_network.sdf', 'World in agos_sim/worlds'),
    ('gz_gui', 'true', 'Open the Gazebo window (false = faster; watch in RViz)'),
    ('rviz', 'true', 'Open RViz'),
    ('spawn_x', '0.0', 'Robot start position x (m)'),
    ('spawn_y', '0.0', 'Robot start position y (m)'),
    ('spawn_yaw', '0.0', 'Robot start heading (rad)'),
    ('camera_width', '320', 'RGB-D camera image width'),
    ('camera_height', '240', 'RGB-D camera image height'),
    ('camera_rate', '10', 'RGB-D camera frames per second'),
]
SLAM_ARGS = [
    ('localization', 'false', 'true = reuse the last map instead of making a new one'),
    ('rtabmap_viz', 'false', "Also open RTAB-Map's own window (heavy)"),
    ('nav2', 'true', 'Start Nav2'),
    ('autostart', 'true', 'Start Nav2 by itself once the map exists'),
    ('slam', 'local', 'local = RTAB-Map here; remote = on a Raspberry Pi (agos_pi_slam)'),
    ('slam_host', '', 'slam:=remote: address of the Raspberry Pi'),
]


def generate_launch_description():
    pkg = get_package_share_directory('agos_sim')
    args = [DeclareLaunchArgument(name, default_value=default, description=text)
            for name, default, text in SIM_ARGS + SLAM_ARGS]
    args += [
        DeclareLaunchArgument('explore', default_value='false',
                              description='Run the autonomous exploration mission'),
        DeclareLaunchArgument('leg_length', default_value='25.0',
                              description='How far (m along the pipe) to explore each way'),
        DeclareLaunchArgument('explore_mode', default_value='legs',
                              description="legs = the proposal's mission (leg_length each way "
                                          'from the start); full = map the whole network, '
                                          'then come back'),
        DeclareLaunchArgument('forward_only', default_value='true',
                              description='Drive into unmapped pipe forwards only, turning '
                                          'round in manholes (false = the old behaviour, '
                                          'which may reverse into new pipe)'),
        DeclareLaunchArgument('tour', default_value='true',
                              description='full mode: plan a tour through all the open '
                                          'pipe ends instead of taking the nearest'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={name: LaunchConfiguration(name)
                          for name, _, _ in SIM_ARGS}.items())
    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam_nav.launch.py')),
        launch_arguments={name: LaunchConfiguration(name)
                          for name, _, _ in SLAM_ARGS}.items())
    # The explorer runs on the wall clock and takes the simulation time from
    # /tf instead (see now_s in explorer.py): following /clock would cost it a
    # sixth of a CPU core.
    explorer = Node(
        package='agos_sim',
        executable='explorer',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'mode': LaunchConfiguration('explore_mode'),
                     'leg_length': ParameterValue(LaunchConfiguration('leg_length'),
                                                  value_type=float),
                     'forward_only': ParameterValue(LaunchConfiguration('forward_only'),
                                                    value_type=bool),
                     'tour': ParameterValue(LaunchConfiguration('tour'), value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('explore')),
    )
    return LaunchDescription(args + [sim, slam_nav, explorer])
