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

