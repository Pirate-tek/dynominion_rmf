import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('dynominion_rmf_bringup')
    config_file = os.path.join(pkg_bringup, 'config', 'rmf_config.yaml')

    pkg_maps = get_package_share_directory('dynominion_rmf_maps')
    building_map_file = os.path.join(pkg_maps, 'building', 'new_env.building.yaml')

    return LaunchDescription([
        # RMF Building Map Server
        Node(
            package='rmf_building_map_tools',
            executable='building_map_server',
            name='building_map_server',
            output='screen',
            arguments=[building_map_file],
            parameters=[config_file]
        ),

        # RMF Traffic Schedule
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule',
            output='screen',
            parameters=[config_file]
        ),
        
        # RMF Task Dispatcher
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            name='rmf_task_dispatcher',
            output='screen',
            parameters=[config_file]
        ),

        # RMF Traffic Blockade
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_blockade',
            name='rmf_traffic_blockade',
            output='screen',
            parameters=[config_file]
        )
    ])
