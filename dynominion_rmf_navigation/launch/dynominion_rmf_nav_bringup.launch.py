# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    working_dir = get_package_share_directory('dynominion_rmf_navigation')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')
    use_rviz = LaunchConfiguration('use_rviz', default='True')
    initial_pose_x   = LaunchConfiguration('initial_pose_x',   default='0.0')
    initial_pose_y   = LaunchConfiguration('initial_pose_y',   default='0.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.0')

    # Keep TF on the global topics so the isolated Nav2 container shares the
    # same TF tree as the rest of the robot stack.
    remappings = []

    # Replace the placeholder in the params file with the actual namespace
    params_file_substituted = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': namespace}
    )

    configured_params = ParameterFile(
        params_file_substituted,
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='dynominion1', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(working_dir, 'maps', 'dynominion_rmf_map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='True',
        description='Whether to enable localization or not'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(working_dir, 'config', 'nav_param.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(working_dir, 'rviz', 'nav2_view.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )

    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0', description='Initial pose X for AMCL'
    )
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0', description='Initial pose Y for AMCL'
    )
    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0', description='Initial pose Yaw for AMCL'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'slam_launch.py')
                ),
                condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_respawn': use_respawn,
                    'params_file': params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(working_dir, 'launch', 'localization_launch.py')
                ),
                condition=IfCondition(PythonExpression(['not ', slam, ' and ', use_localization])),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file_substituted,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                    'launch_map_server': LaunchConfiguration('launch_map_server'),
                    'initial_pose_x':   initial_pose_x,
                    'initial_pose_y':   initial_pose_y,
                    'initial_pose_yaw': initial_pose_yaw,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(working_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file_substituted,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)
    ld.add_action(DeclareLaunchArgument(
        'launch_map_server', default_value='True',
        description='Whether to launch the map server'
    ))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    ld.add_action(start_rviz_cmd)

    return ld
