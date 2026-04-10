import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            RegisterEventHandler, TimerAction, GroupAction,
                            ExecuteProcess)
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('dynominion_rmf_gazebo')
    pkg_nav = get_package_share_directory('dynominion_rmf_navigation')
    pkg_bringup = get_package_share_directory('dynominion_rmf_bringup')
    pkg_adapter = get_package_share_directory('dynominion_fleet_adapter')
    pkg_maps = get_package_share_directory('dynominion_rmf_maps')

    # Launch Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_file = os.path.join(pkg_maps, 'maps', 'dynominion_map.yaml')
    params_file = os.path.join(pkg_nav, 'config', 'nav_param.yaml')
    
    # 1. Gazebo & Spawns
    # Using existing multi_robot_gazebo.launch.py
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'multi_robot_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Navigation instances per robot
    robots = ['dynominion1', 'dynominion2', 'dynominion3', 'dynominion4', 'dynominion5']
    nav_instances = []
    for robot in robots:
        nav_instances.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, 'launch', 'dynominion_rmf_nav_bringup.launch.py')
                ),
                launch_arguments={
                    'namespace': robot,
                    'use_namespace': 'True',
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'True',
                    'use_localization': 'True',
                    'use_rviz': 'False'
                }.items()
            )
        )

    # 3. RMF Core Nodes
    # We define them here to use the handles for event registration
    schedule_node = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        name='rmf_traffic_schedule',
        output='screen'
    )
    
    dispatcher_node = Node(
        package='rmf_task_ros2',
        executable='rmf_task_dispatcher',
        name='rmf_task_dispatcher',
        output='screen'
    )
    
    blockade_node = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_blockade',
        name='rmf_traffic_blockade',
        output='screen'
    )

    # 4. Fleet Adapter
    # Fleet config path
    fleet_config_file = os.path.join(pkg_adapter, 'config', 'fleet_config.yaml')
    # Nav graph path (usually 0.yaml in maps package)
    nav_graph_path = os.path.join(pkg_maps, 'nav_graphs', '0.yaml')

    fleet_adapter_node = Node(
        package='dynominion_fleet_adapter',
        executable='fleet_adapter_node',
        name='dynominion_fleet_adapter',
        output='screen',
        parameters=[{
            'config_file': fleet_config_file,
            'nav_graph_path': nav_graph_path,
            'use_sim_time': use_sim_time
        }]
    )

    # RMF event handling: start adapter only after schedule is up
    adapter_with_trigger = RegisterEventHandler(
        OnProcessStart(
            target_action=schedule_node,
            on_start=[
                TimerAction(
                    period=5.0, # extra buffer
                    actions=[fleet_adapter_node]
                )
            ]
        )
    )

    # 5. RViz
    # We could start a new RViz instance with a fleet view
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fleet',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'fleet_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(sim_gazebo)
    for nav in nav_instances:
        ld.add_action(nav)
    ld.add_action(schedule_node)
    ld.add_action(dispatcher_node)
    ld.add_action(blockade_node)
    ld.add_action(adapter_with_trigger)
    ld.add_action(rviz_node)

    return ld
