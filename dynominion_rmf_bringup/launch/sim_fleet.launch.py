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
        {'name': 'dynominion1', 'x': 1.48, 'y': -5.6, 'yaw': 0.0},
        {'name': 'dynominion2', 'x': 1.48, 'y': -9.5, 'yaw': 0.0},
        {'name': 'dynominion3', 'x': 7.89, 'y': -9.5, 'yaw': 0.0},
        {'name': 'dynominion4', 'x': 7.92, 'y': -15.9, 'yaw': 0.0},
        {'name': 'dynominion5', 'x': 1.63, 'y': -16.6, 'yaw': 0.0},
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

    # 3. RMF Core Nodes
    # We define them here to use the handles for event registration
    schedule_node = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_schedule',
        name='rmf_traffic_schedule',
        output='screen',
        parameters=[rmf_config_file] # Fix 3: Sync Time/Parameters
    )
    
    dispatcher_node = Node(
        package='rmf_task_ros2',
        executable='rmf_task_dispatcher',
        name='rmf_task_dispatcher',
        output='screen',
        parameters=[rmf_config_file] # Fix 3: Sync Time/Parameters
    )
    
    blockade_node = Node(
        package='rmf_traffic_ros2',
        executable='rmf_traffic_blockade',
        name='rmf_traffic_blockade',
        output='screen',
        parameters=[rmf_config_file] # Fix 3: Sync Time/Parameters
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
        }],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Fleet Adapter with a delay to ensure schedule is up
    fleet_adapter_timer = TimerAction(
        period=15.0,
        actions=[fleet_adapter_node]
    )

    # 5. RMF Visualization
    schedule_visualizer = Node(
        package='rmf_visualization_schedule',
        executable='schedule_visualizer_node',
        name='schedule_visualizer',
        output='screen',
        parameters=[rmf_config_file]
    )

    nav_graph_visualizer = Node(
        package='rmf_visualization_navgraphs',
        executable='navgraph_visualizer_node',
        name='navgraph_visualizer',
        output='screen',
        parameters=[{
            'nav_graph_file': nav_graph_path,
            'use_sim_time': use_sim_time
        }]
    )
    
    # 6. RViz
    # We could start a new RViz instance with a fleet view
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fleet',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'fleet_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 7. Initial Pose Publisher (Fix 2: Automatic Initialization)
    initial_pose_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(os.path.dirname(os.path.dirname(pkg_bringup)), 'initialpose_rmf.py')],
        output='screen'
    )
    # Give robots plenty of time to spawn and Nav2/AMCL to start (Base delay is 20s + stagger)
    initial_pose_timer = TimerAction(
        period=NAV_BASE_DELAY + len(robots) * NAV_STAGGER + 15.0,
        actions=[initial_pose_publisher]
    )

    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)
    ld.add_action(sim_gazebo)
    for nav in nav_instances:
        ld.add_action(nav)
    ld.add_action(schedule_node)
    ld.add_action(dispatcher_node)
    ld.add_action(blockade_node)
    ld.add_action(fleet_adapter_timer)
    ld.add_action(schedule_visualizer)
    ld.add_action(nav_graph_visualizer)
    ld.add_action(rviz_node)
    ld.add_action(initial_pose_timer)

    return ld
