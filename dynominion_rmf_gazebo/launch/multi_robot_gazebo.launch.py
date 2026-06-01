import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    pkg_dynominion_gazebo = get_package_share_directory('dynominion_rmf_gazebo')
    
    # Define robots to spawn
    robots = [
        {'name': 'dynominion1', 'x': '1.48', 'y': '-9.51', 'z': '0.5'},
        {'name': 'dynominion2', 'x': '1.48', 'y': '-15.95', 'z': '0.5'},
        {'name': 'dynominion3', 'x': '4.7', 'y': '-5.55', 'z': '0.5'},
        {'name': 'dynominion4', 'x': '7.89', 'y': '-9.51', 'z': '0.5'},
        {'name': 'dynominion5', 'x': '7.89', 'y': '-15.95', 'z': '0.5'},
        {'name': 'dynominion6', 'x': '6.4', 'y': '-5.55', 'z': '0.5'},
        {'name': 'dynominion7', 'x': '6.4', 'y': '-19.4', 'z': '0.5'},
    ]


    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    #world = LaunchConfiguration('world', default='cafe.world')
    world = LaunchConfiguration('world', default='new_env.world')

    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    #ld.add_action(DeclareLaunchArgument('world', default_value='cafe.world'))
    ld.add_action(DeclareLaunchArgument('world', default_value='new_env.world'))

    # Global bridge for clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/new_env/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    ld.add_action(clock_bridge)

    # Loop through the robots and add spawn actions
    for i, robot in enumerate(robots):
        # We only launch simulation for the first robot (or we could launch it separately)
        # Here, we set launch_simulation to true only for the first one.
        launch_sim = 'True' if i == 0 else 'False'
        
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dynominion_gazebo, 'launch', 'dynominion_rmf_gazebo.launch.py')
            ),
            launch_arguments={
                'robot_name': robot['name'],
                'x': robot['x'],
                'y': robot['y'],
                'z': robot['z'],
                'use_sim_time': use_sim_time,
                'world': world,
                'launch_simulation': launch_sim
            }.items()
        )
        
        # Stagger the launches by 15 seconds each
        staggered_spawn = TimerAction(
            period=float(i * 15.0),
            actions=[spawn_robot]
        )
        ld.add_action(staggered_spawn)

    return ld
