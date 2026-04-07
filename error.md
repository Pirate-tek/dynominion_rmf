# Launch File Errors and Resolutions

This document records investigation and fixes for errors encountered during the namespacing and multi-robot setup of the `dynominion_rmf` package.

---

## Error 1: `NameError: name 'ReplaceString' is not defined`

### **Symptoms:**
Running `ros2 launch dynominion_rmf_gazebo multi_robot_gazebo.launch.py` resulted in an exception indicating `ReplaceString` was not defined.

### **Root Cause:**
The `ReplaceString` utility from `nav2_common.launch` was being used to dynamically generate the bridge configuration, but it was not imported in the `multi_robot_gazebo.launch.py` file.

### **Fix:**
Added the missing import at the top of the launch file:
```python
from nav2_common.launch import ReplaceString
```

---

## Error 2: `UnboundLocalError: cannot access local variable 'configured_controllers' where it is not associated with a value`

### **Symptoms:**
The launch system failed when trying to generate the `robot_description` for simulation.

### **Root Cause:**
In Python launch files, the `generate_launch_description()` function is executed sequentially. The `configured_controllers` variable (result of `RewrittenYaml`) was being passed into the `xacro` command at a line earlier than its actual assignment. 

```python
# BROKEN ORDER:
robot_description_content = Command([..., 'controller_config:=', configured_controllers])
...
configured_controllers = RewrittenYaml(...)
```

### **Fix:**
Reordered the assignments so that `RewrittenYaml` generates the configuration file path first, ensuring the variable is populated before it is passed to the `xacro` command.

---

## Error 3: `TypeError: Failed to normalize given item of type '<class 'NoneType'>'`

### **Symptoms:**
Caught exception when trying to launch after initializing `configured_controllers = None`.

### **Root Cause:**
Initializing the variable to `None` satisfied the scope check, but the `launch` system's `Command` substitution cannot handle `NoneType`. It requires either a Raw string or a `Substitution` object.

### **Fix:**
Resolved by properly reordering the code (see Error 2 fix) to provide a valid `Substitution` instead of `None`.

---

## Summary of Missing Dependencies
During the refactor, several imports were missing that resulted in runtime failures:
- `RewrittenYaml` (nav2_common)
- `GroupAction` (launch)
- `PushROSNamespace` (launch_ros)
- `IfCondition` (launch)

These have all been added to the `dynominion_rmf_gazebo.launch.py` script.

---

## Error 4: `Could not contact service /controller_manager/list_controllers`

### **Symptoms:**
During multi-robot launch, the `spawner` nodes for `joint_state_broadcaster` and `diff_drive_base_controller` timed out with warnings about not being able to contact the controller manager service.

### **Root Cause:**
While the spawner node itself was namespaced via `PushROSNamespace`, the internal service calls were resolving to the global root (`/controller_manager/list_controllers`). This is because the `--controller-manager` argument was passed as a simple relative string `controller_manager`, which some versions of the spawner tool do not correctly resolve under a group namespace.

### **Fix:**
Fully qualified the `--controller-manager` argument using the `robot_name` in the launch file:
```python
arguments=['joint_state_broadcaster', '--controller-manager', [robot_name, '/controller_manager']]
```
This forces the spawner to look specifically in the robot's namespaced controller manager (e.g., `/dynominion1/controller_manager`).

---

## Error 5: `Timeout when getting world names` (Gazebo Sim Exit Code 14)

### **Symptoms:**
The `gz_spawn_entity` node timed out after 30 seconds with an error: `[create]: Requesting list of world names... [ERROR] [create]: Service /world/cafe/create not available`.

### **Root Cause:**
Gazebo was failing to load models specified in the `.world` file (e.g., `model://Cafe`) because the `GZ_SIM_RESOURCE_PATH` environment variable was not configured. This caused the Gazebo server to exit immediately upon startup with `Error Code 14`, preventing the spawning services from ever becoming available.

### **Fix:**
Injected the correct model path into the launch environment using `AppendEnvironmentVariable`:
```python
gz_resource_path = AppendEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=PathJoinSubstitution([FindPackageShare('dynominion_rmf_gazebo'), 'models'])
)
```

---

## Error 6: `LaunchConfiguration` Scoping Bleed & "Waiting for service /controller_manager/..."

### **Symptoms:**
Spawners for robots 1-4 would wait indefinitely for the `controller_manager` of robot 5.

### **Root Cause:**
Using `RegisterEventHandler` with `OnProcessExit` caused the `LaunchConfiguration('robot_name')` inside the event handler's `on_exit` actions to be evaluated in the *global* context after the loop had finished, rather than the local context of each robot. This meant all spawners defaulted to the last robot's name.

### **Fix:**
Removed asynchronous event handlers for spawners. Instead, defining the `Node` actions directly inside the `GroupAction` with `PushROSNamespace` ensures they are correctly encapsulated and executed with the appropriate local namespace context.

---

## Error 7: `RTPS_TRANSPORT_SHM Error` and "The 'type' param was not defined"

### **Symptoms:**
Some robots (typically 2 and 5) would fail to load their controllers, reporting that the `type` parameter was missing, despite it being present in the YAML. `RTPS_TRANSPORT_SHM` errors were also observed in the logs.

### **Root Cause:**
Starting 5 independent robot launch sequences simultaneously caused race conditions in both parameter rewriting (`RewrittenYaml`) and shared memory port allocation for ROS 2 communications.

### **Fix:**
1. **Static Multi-Robot Config**: Replaced dynamic `RewrittenYaml` with a static `multi_robot_controllers.yaml` mapping every robot to its specific parameters and frame IDs.
2. **Staggered Launch**: Implemented a 5-second delay between robot spawns using `TimerAction` to ensure the system has adequate time to allocate resources for each instance sequentially.
3. **Global Clock Bridge**: Added a dedicated global `/clock` bridge to prevent "No clock received" warnings in namespaced `controller_manager` nodes.

---

## Error 8: `spawner: error: unrecognized arguments` (ROS 2 Jazzy)

### **Symptoms:**
The controller spawner failed to start with the error: `spawner: error: unrecognized arguments: -r diff_drive_base_controller/odom:=odom`.

### **Root Cause:**
In ROS 2 Jazzy, the `controller_manager` spawner requires each individual remapping argument to be preceded by its own `--controller-ros-args` flag. Grouping multiple remappings under a single flag or passing them as separate list items without individual flags causes parsing failures.

### **Fix:**
Updated the spawner `arguments` in the launch file to prefix every remapping with its own flag:
```python
arguments=[
    'diff_drive_base_controller', 
    '--controller-ros-args', '-r diff_drive_base_controller/cmd_vel:=cmd_vel',
    '--controller-ros-args', '-r diff_drive_base_controller/odom:=odom'
]
```

---

## Error 9: Double Prefixing in `robot_state_publisher`

### **Symptoms:**
TF frames were appearing with a doubled name prefix, e.g., `dynominion1/dynominion1/base_link`.

### **Root Cause:**
The `robot_state_publisher` node was being passed a `frame_prefix` parameter (`dynominionX/`) while also being placed inside a ROS namespace (`/dynominionX`). In ROS 2, `robot_state_publisher` automatically applies the namespace as a prefix to the URDF frames, leading to redundant doubling when `frame_prefix` is also set.

### **Fix:**
Removed the explicit `frame_prefix` parameter from the `robot_state_publisher` node configuration in the launch file.

---

## Error 10: `XML Element[gz_frame_id]... not defined in SDF`

### **Symptoms:**
Gazebo logs showed warnings about `gz_frame_id` not being a valid SDF element for sensors.

### **Root Cause:**
The `<gz_frame_id>` tag is a deprecated or non-standard element for modern Gazebo Sim (Harmonic/Jazzy). The standard element for defining the coordinate frame of a sensor is `<frame_id>`.

### **Fix:**
Renamed all `<gz_frame_id>` tags to `<frame_id>` in the `gazebo_sensor_plugin.xacro` file and ensured they were correctly placed within the `<sensor>` block.

---

## Error 11: `Executor is not available during hardware component initialization`

### **Symptoms:**
Warnings appeared during launch indicating that the `controller_manager` could not create nodes because the executor was not yet available. This often led to controllers failing to load or "Waiting for RM" indefinitely.

### **Root Cause:**
A race condition where the `spawner` nodes were attempting to load controllers before the Gazebo `ros2_control` plugin had finished initializing the hardware interface and its internal executor.

### **Fix:**
1. **Timed Delay**: Wrapped the spawner nodes in a `TimerAction` to delay their execution by 5 seconds, giving Gazebo enough time to fully initialize.
2. **Inactive Loading**: Added the `--inactive` flag to the spawners so they load the controllers into a dormant state, allowing the system to stabilize before activation.

---

## Error 12: `Desired controller update period is slower than the gazebo simulation period`

### **Symptoms:**
Warnings in Gazebo: `[WARN] [dynominion1.gz_ros_control]: Desired controller update period (0.0333333 s) is slower than the gazebo simulation period (0.001 s).`

### **Root Cause:**
The `controller_manager` update rate was set to 30 Hz (0.033s), which is significantly slower than the 1000 Hz (0.001s) simulation step. This can cause jittery control and integration issues.

### **Fix:**
Raised the `update_rate` to **100 Hz** in the `multi_robot_controllers.yaml` file to provide more frequent control updates and reduce the rate mismatch warning.

