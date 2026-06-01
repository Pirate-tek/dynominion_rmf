import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            RegisterEventHandler, TimerAction, GroupAction,
                            ExecuteProcess)
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

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
    rmf_config_file = os.path.join(pkg_bringup, 'config', 'rmf_config.yaml')
    
    # 0. Global Map Server (Fix 1: Consolidate Map Server)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': use_sim_time}]
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )


    door_supervisor_node = Node(
        package='rmf_fleet_adapter',
        executable='door_supervisor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    lift_supervisor_node = Node(
        package='rmf_fleet_adapter',
        executable='lift_supervisor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 1. Gazebo & Spawns
    # Using existing multi_robot_gazebo.launch.py
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'multi_robot_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Navigation instances per robot
    # Positions match multi_robot_gazebo.launch.py spawns exactly
    robots = [
        {'name': 'dynominion1', 'x': 1.48, 'y': -9.51, 'yaw': 0.0},
        {'name': 'dynominion2', 'x': 1.48, 'y': -15.95, 'yaw': 0.0},
        {'name': 'dynominion3', 'x': 4.7, 'y': -5.55, 'yaw': 0.0},
        {'name': 'dynominion4', 'x': 7.89, 'y': -9.51, 'yaw': 0.0},
        {'name': 'dynominion5', 'x': 7.89, 'y': -15.95, 'yaw': 0.0},
        {'name': 'dynominion6', 'x': 6.4, 'y': -5.55, 'yaw': 0.0},
        {'name': 'dynominion7', 'x': 6.4, 'y': -19.4, 'yaw': 0.0},
    ]


    nav_instances = []
    # Stagger matches Gazebo spawn: robot i spawns at i*5s.
    # Add a 20s base buffer on top to let TF, controllers and sensors settle.
    NAV_BASE_DELAY = 20.0   # seconds after launch before first robot's Nav2 starts
    NAV_STAGGER    = 10.0   # seconds between each subsequent robot

    for i, robot in enumerate(robots):
        # Fix: correctly namespace parameters using RewrittenYaml so nodes find their config
        param_substitutions = {
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'base_frame_id': [robot['name'], '/base_footprint'],
            'odom_frame_id': [robot['name'], '/odom'],
        }

        robot_params_file = RewrittenYaml(
            source_file=params_file,
            root_key=robot['name'],
            param_rewrites=param_substitutions,
            convert_types=True
        )

        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'dynominion_rmf_nav_bringup.launch.py')
            ),
            launch_arguments={
                'namespace':        robot['name'],
                'use_namespace':    'True',
                'map':              map_file,
                'use_sim_time':     use_sim_time,
                'params_file':      robot_params_file,
                'autostart':        'True',
                'use_localization': 'True',
                'use_rviz':         'False',
                'launch_map_server': 'False', # Global map server is used instead
                'initial_pose_x':   str(robot['x']),
                'initial_pose_y':   str(robot['y']),
                'initial_pose_yaw': str(robot['yaw']),
            }.items()
        )
        nav_instances.append(
            TimerAction(
                period=NAV_BASE_DELAY + i * NAV_STAGGER,
                actions=[nav_launch]
            )
        )


    # 4. Fleet Adapter (Process 1 — Goal 6)
    # High-level: RMF task bidding, traffic scheduling, status polling.
    fleet_config_file = os.path.join(pkg_adapter, 'config', 'integration_config.yaml')
    nav_graph_path = os.path.join(pkg_maps, 'nav_graphs', '0.yaml')
    fleet_node_params = os.path.join(pkg_adapter, 'config', 'fleet_node_params.yaml')




    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)
    ld.add_action(door_supervisor_node)
    ld.add_action(lift_supervisor_node)
    ld.add_action(sim_gazebo)
    for nav in nav_instances:
        ld.add_action(nav)

    return ld
