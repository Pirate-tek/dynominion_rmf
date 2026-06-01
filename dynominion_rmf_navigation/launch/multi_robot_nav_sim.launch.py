import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('dynominion_rmf_gazebo')
    pkg_nav = get_package_share_directory('dynominion_rmf_navigation')
    pkg_bringup = get_package_share_directory('dynominion_rmf_bringup')
    pkg_maps = get_package_share_directory('dynominion_rmf_maps')

    # Launch Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_file = LaunchConfiguration(
        'map', 
        default=os.path.join(pkg_maps, 'maps', 'dynominion_map.yaml')
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_nav, 'config', 'nav_param.yaml')
    )

    # 1. Global Map Server
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

    # 2. Gazebo & Spawns
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'multi_robot_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Navigation instances per robot
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
    
    # Stagger navigation launches to reduce startup load
    for i, robot in enumerate(robots):
        param_substitutions = {
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'base_frame_id': [robot['name'], '/base_footprint'],
            'robot_base_frame': [robot['name'], '/base_footprint'],
            'base_frame': [robot['name'], '/base_footprint'],
            'odom_frame_id': [robot['name'], '/odom'],
            'local_frame': [robot['name'], '/odom'],
            'fixed_frame': [robot['name'], '/odom'],
            'set_initial_pose': 'True',
            'initial_pose_x': str(float(robot.get('x', 0.0))),
            'initial_pose_y': str(float(robot.get('y', 0.0))),
            'initial_pose_z': str(float(robot.get('z', 0.0))),
            'initial_pose_yaw': str(float(robot.get('yaw', 0.0)))
        }

        namespaced_params_file = ReplaceString(
            source_file=os.path.join(pkg_nav, 'config', 'nav_param.yaml'),
            replacements={'<robot_namespace>': robot['name']}
        )

        robot_params_file = RewrittenYaml(
            source_file=namespaced_params_file,
            root_key=robot['name'],
            param_rewrites=param_substitutions,
            convert_types=True
        )
        
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'dynominion_rmf_nav_bringup.launch.py')
            ),
            launch_arguments={
                'namespace': robot['name'],
                'use_namespace': 'True',
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': robot_params_file,
                'autostart': 'True',
                'use_localization': 'True',
                'use_rviz': 'False',
                'launch_map_server': 'False',
                'initial_pose_x': str(float(robot.get('x', 0.0))),
                'initial_pose_y': str(float(robot.get('y', 0.0))),
                'initial_pose_yaw': str(float(robot.get('yaw', 0.0)))
            }.items()
        )
        
        # Start navigation slightly after Gazebo spawner for each to avoid tf issues during init
        staggered_nav = TimerAction(
            period=float(10.0 + i * 10.0),
            actions=[nav_launch]
        )
        nav_instances.append(staggered_nav)

    # 3. RViz showing fleet view
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
    
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('map', default_value=os.path.join(pkg_maps, 'maps', 'dynominion_map.yaml')))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_nav, 'config', 'nav_param.yaml')))

    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_node)
    ld.add_action(sim_gazebo)
    for nav in nav_instances:
        ld.add_action(nav)
    ld.add_action(rviz_node)

    return ld
