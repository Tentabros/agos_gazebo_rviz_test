import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 1. RTAB-Map SLAM Node
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'use_sim_time': True,
            'Grid/3D': 'false',
            'Grid/FromDepth': 'false',
            'Grid/RayTracing': 'true',
            'Grid/RangeMax': '10.0',
            'Reg/Strategy': '1',
            'RGBD/NeighborLinkRefining': 'true',
        }],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth_image'),
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('grid_map', '/map')
        ],
        arguments=['-d']
    )

    # 2. RTAB-Map Visualizer
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'use_sim_time': True,
            'Grid/3D': 'false',
        }],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth_image'),
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
    )

    # 3. Full Nav2 Bringup (Launches lifecycle_manager_navigation)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': '',                             # SLAM provides the map dynamically
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        }.items()
    )

    return LaunchDescription([
        rtabmap_slam,
        rtabmap_viz,
        nav2_bringup
    ])