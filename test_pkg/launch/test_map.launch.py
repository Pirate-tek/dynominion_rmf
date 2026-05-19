import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_test = get_package_share_directory('test_pkg')
    building_map_file = os.path.join(pkg_test, 'building', 'new_env.building.yaml')
    rviz_config_file = os.path.join(pkg_test, 'rviz', 'test_map.rviz')

    return LaunchDescription([
        Node(
            package='rmf_building_map_tools',
            executable='building_map_server',
            name='building_map_server',
            output='screen',
            arguments=[building_map_file],
            remappings=[
                ('/map', '/floorplan')
            ]
        ),
        Node(
            package='test_pkg',
            executable='nav_graph_bridge.py',
            name='nav_graph_bridge',
            output='screen'
        ),
        Node(
            package='rmf_visualization_navgraphs',
            executable='navgraph_visualizer_node',
            name='navgraph_visualizer_node',
            output='screen',
            remappings=[
                ('/map', '/floorplan'),
                ('/nav_graphs', '/nav_graphs')
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
