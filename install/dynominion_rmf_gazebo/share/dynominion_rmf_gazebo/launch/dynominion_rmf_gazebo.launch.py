# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, GroupAction, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([FindPackageShare('dynominion_rmf_gazebo'), 'models'])
    )
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    world = LaunchConfiguration('world', default="cafe.world")
    robot_name = LaunchConfiguration('robot_name', default="dynominion1")
    config_file = LaunchConfiguration('config', default="gz_bridge.yaml")
    
    x_pose = LaunchConfiguration('x', default='0.0')
    y_pose = LaunchConfiguration('y', default='0.0')
    z_pose = LaunchConfiguration('z', default='0.1')

    world_path = PathJoinSubstitution([
        FindPackageShare("dynominion_rmf_gazebo"),
        "worlds",
        world
    ])
    
    # Path to the bridge config
    config_path = PathJoinSubstitution([
        FindPackageShare("dynominion_rmf_gazebo"),
        "config",
        config_file
    ])

    # Replace <robot_name> in bridge config if it exists, or handle prefixing
    # For simplicity, we assume the bridge config topics are relative and we push the node to a namespace.
    # But Gazebo topics need absolute paths. We use ReplaceString to handle the GZ topics.
    # Note: The user said "generate per-robot in the launch file".
    bridge_params = ReplaceString(
        source_file=config_path,
        replacements={'GZ_SCAN': ['/', robot_name, '/scan'], 'GZ_IMU': ['/', robot_name, '/imu']}
    )

    configured_controllers = PathJoinSubstitution(
        [FindPackageShare('dynominion_rmf_gazebo'), 'config', 'multi_robot_controllers.yaml']
    )

    # Robot State Publisher
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare('dynominion_rmf_gazebo'),
        'urdf',
        'dynominion_rmf.urdf.xacro'
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file_path,
        ' robot_name:=', robot_name,
        ' controller_config:=', configured_controllers
    ])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', 
                   '-name', robot_name,
                   '-allow_renaming', 'true',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', z_pose
                   ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-ros-args', '-r /joint_states:=joint_states'],
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', 
                   '--controller-ros-args', '-r diff_drive_base_controller/cmd_vel:=cmd_vel',
                   '--controller-ros-args', '-r diff_drive_base_controller/odom:=odom'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params, "use_sim_time": use_sim_time}],
        output='screen'
    )

    odom_modifier = Node(
        package='dynominion_rmf_gazebo',
        executable='odom_modifier.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    joint_state_republisher = Node(
        package='dynominion_rmf_gazebo',
        executable='joint_state_republisher.py',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Group all robot-specific nodes into a namespace
    robot_group = GroupAction([
        PushROSNamespace(robot_name),
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        odom_modifier,
        joint_state_republisher,
        TimerAction(
            period=10.0,
            actions=[
                joint_state_broadcaster_spawner,
                diff_drive_base_controller_spawner,
            ]
        ),
    ])

    launch_simulation = LaunchConfiguration('launch_simulation', default='true')

    return LaunchDescription([
        stdout_linebuf_envvar,
        gz_resource_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='cafe.world'),
        DeclareLaunchArgument('robot_name', default_value='dynominion1'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.5'),
        DeclareLaunchArgument('launch_simulation', default_value='true'),
        
        # Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
            condition=IfCondition(launch_simulation),
            launch_arguments=[('gz_args', [' -r -v 1 ', world_path])]),
            
        robot_group
    ])
