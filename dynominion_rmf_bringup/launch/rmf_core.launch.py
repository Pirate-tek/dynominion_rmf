import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    pkg_bringup = get_package_share_directory('dynominion_rmf_bringup')
    config_file = os.path.join(pkg_bringup, 'config', 'rmf_config.yaml')

    pkg_maps = get_package_share_directory('dynominion_rmf_maps')
    building_map_file = os.path.join(pkg_maps, 'building', 'new_env.building.yaml')
    nav_graph_path = os.path.join(pkg_maps, 'nav_graphs', '0.yaml')

    pkg_adapter = get_package_share_directory('dynominion_fleet_adapter')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # RMF Visualization
    rmf_visualization_group = GroupAction([
        SetRemap(src='/map', dst='/floorplan'),
        SetRemap(src='/floorplan', dst='/floorplan_grid'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rmf_visualization'), 'visualization.launch.xml')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map_name': 'L1',
                'viz_config_file': os.path.join(pkg_bringup, 'rviz', 'fleet_view.rviz')
            }.items()
        )
    ])

    fleets = [
        {
            'name': 'dynominion_fleet1',
            'params': os.path.join(pkg_adapter, 'config', 'fleet1_node_params.yaml'),
            'config': os.path.join(pkg_adapter, 'config', 'fleet1_integration_config.yaml')
        },
        {
            'name': 'dynominion_fleet2',
            'params': os.path.join(pkg_adapter, 'config', 'fleet2_node_params.yaml'),
            'config': os.path.join(pkg_adapter, 'config', 'fleet2_integration_config.yaml')
        },
        {
            'name': 'dynominion_fleet3',
            'params': os.path.join(pkg_adapter, 'config', 'fleet3_node_params.yaml'),
            'config': os.path.join(pkg_adapter, 'config', 'fleet3_integration_config.yaml')
        }
    ]

    fleet_actions = []
    for f in fleets:
        f_manager = Node(
            package='dynominion_fleet_adapter',
            executable='fleet_manager_node',
            name=f['name'] + '_manager',
            output='screen',
            parameters=[
                f['params'],
                {
                    'config_file': f['config'],
                    'use_sim_time': use_sim_time,
                }
            ]
        )
        f_adapter = Node(
            package='dynominion_fleet_adapter',
            executable='fleet_adapter_node',
            name=f['name'] + '_adapter',
            output='screen',
            parameters=[
                f['params'],
                {
                    'config_file': f['config'],
                    'nav_graph_path': nav_graph_path,
                    'use_sim_time': use_sim_time,
                }
            ]
        )
        fleet_actions.append(TimerAction(period=13.0, actions=[f_manager]))
        fleet_actions.append(TimerAction(period=15.0, actions=[f_adapter]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        # RMF Building Map Server (Patched)
        Node(
            package='dynominion_rmf_bringup',
            executable='building_map_server_patched.py',
            arguments=[building_map_file],
            name='building_map_server',
            output='screen',
            remappings=[('/map', '/floorplan')],
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # RMF Traffic Schedule
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),
        
        # RMF Task Dispatcher
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            name='rmf_task_dispatcher',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),

        # RMF Traffic Blockade
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_blockade',
            name='rmf_traffic_blockade',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),
        
        *fleet_actions,
        rmf_visualization_group
    ])
