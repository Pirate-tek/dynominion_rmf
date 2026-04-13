import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            TimerAction, GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import tempfile

def create_robot_nav_params(robot_name, template_path):
    with open(template_path, 'r') as f:
        nav_params = yaml.safe_load(f)
        
    def replace_frames(d):
        if isinstance(d, dict):
            if 'ros__parameters' in d:
                d['ros__parameters']['use_sim_time'] = True
                d['ros__parameters']['autostart'] = True
                
            for k, v in d.items():
                if k in ['base_frame_id', 'robot_base_frame', 'base_frame']:
                    d[k] = f"{robot_name}/base_footprint"
                elif k in ['odom_frame_id', 'local_frame', 'fixed_frame']:
                    d[k] = f"{robot_name}/odom"
                elif k == 'global_frame' and v == 'odom':
                    d[k] = f"{robot_name}/odom"
                elif isinstance(v, dict):
                    replace_frames(v)

    for node_config in nav_params.values():
        replace_frames(node_config)

    namespaced_params = {robot_name: nav_params}
    
    out_path = os.path.join(tempfile.gettempdir(), f"{robot_name}_nav_param.yaml")
    with open(out_path, 'w') as f:
        yaml.dump(namespaced_params, f)
        
    return out_path

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

    # 1. Gazebo & Spawns
    sim_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'multi_robot_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Navigation instances per robot
    robots = ['dynominion1', 'dynominion2', 'dynominion3', 'dynominion4', 'dynominion5']
    nav_instances = []
    
    # Stagger navigation launches to reduce startup load
    for i, robot in enumerate(robots):
        robot_params_file = create_robot_nav_params(
            robot, 
            os.path.join(pkg_nav, 'config', 'nav_param.yaml')
        )
        
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'dynominion_rmf_nav_bringup.launch.py')
            ),
            launch_arguments={
                'namespace': robot,
                'use_namespace': 'True',
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': robot_params_file,
                'autostart': 'True',
                'use_localization': 'True',
                'use_rviz': 'False'
            }.items()
        )
        
        # Start navigation slightly after Gazebo spawner for each to avoid tf issues during init
        staggered_nav = TimerAction(
            period=float(5.0 + i * 5.0),
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

    ld.add_action(sim_gazebo)
    for nav in nav_instances:
        ld.add_action(nav)
    ld.add_action(rviz_node)

    return ld
