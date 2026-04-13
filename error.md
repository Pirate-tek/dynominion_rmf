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

---

## Error 13: `Can't accept new commands. subscriber is inactive`

### **Symptoms:**
Robots were stationary in Gazebo despite receiving navigation goals. The console was flooded with warnings from `diff_drive_base_controller` stating "Can't accept new commands. subscriber is inactive."

### **Root Cause:**
The controller spawner nodes in the launch file were configured with the `--inactive` flag. This loaded the controllers but did not move them to the `ACTIVE` state. In ROS 2 Control (specifically Jazzy/Humble), an inactive controller does not activate its `cmd_vel` subscriber, leading to the warning and complete lack of movement.

### **Fix:**
1. **Removed `--inactive` flag**: Allowed the spawner to automatically transition controllers to the `ACTIVE` state upon successful loading.
2. **Extended Spawner Delay**: Increased the `TimerAction` delay from 5 to 10 seconds to ensure the Gazebo simulation and hardware interfaces are fully initialized before the spawner attempts to communicate with the `controller_manager`.

---

## Error 14: Odom -> Base Link Transform Pipeline Failure

### **Symptoms:**
Robots were unable to localize or plan paths. The TF tree showed disconnected transforms, and the console was flooded with `dynominion3/dynominion3/odom` double namespacing.

### **Three-Way Namespace Conflict:**

1.  **Double-Prepending Paradox (Controller Config)**:
    - **Issue**: `multi_robot_controllers.yaml` explicitly used `dynominion1/odom`.
    - **Effect**: Since the controller was already namespaced (`/dynominion1`), it automatically prepended its namespace, creating **`dynominion1/dynominion1/odom`**.
    - **Fix**: Removed prefixes from YAML, setting frames as `odom` and `base_footprint`.

2.  **Global Topic Lock-in (Nav2)**:
    - **Issue**: `nav_param.yaml` used leading slashes on topics (e.g., `/wheelodom`).
    - **Effect**: Nav2 nodes (AMCL, Local Planner) ignored local namespaced data and waited for a global root-level topic that did not exist.
    - **Fix**: Removed leading slashes to make topics relative (e.g., `wheelodom`).

3.  **Frame Identity Mismatch (Odom Modifier)**:
    - **Issue**: `odom_modifier.py` script hardcoded static frame names (`odom`).
    - **Effect**: TF could not link incoming odometry data to the actual robot model (whose links are namespaced in URDF like `dynominion1/base_link`).
    - **Fix**: Updated the script to programmatically detect the node namespace and prefix the frames accordingly.

---

## Error 15: `RCLError: failed to shutdown: rcl_shutdown already called` 

### **Symptoms:**
Python scripts like `odom_modifier.py` reported a traceback pointing to `rclpy.shutdown()` upon terminating the launch session.

### **Root Cause:**
Attempting to call `rclpy.shutdown()` when the ROS 2 context has already been terminated. In ROS 2 Jazzy, this raises an explicit `RCLError`.

### **Fix:**
Updated scripts to check `if rclpy.ok(): rclpy.shutdown()` to ensure a clean exit.

# Build Errors and Resolutions — Dynominion Fleet Adapter

This document tracks the issues encountered during the implementation and build of the `dynominion_fleet_adapter` package and how they were resolved.

## 1. File Naming Inconsistency
- **Error**: `CMake Error: Cannot find source file: src/FleetAdapter.cpp`
- **Why it occurred**: The source files were intermittently renamed to `fleetadapter.cpp` and `nav2robotHandle.cpp` on the filesystem. Since Linux filesystems are case-sensitive, CMake could not locate the files specified in `add_executable`.
- **Fix**: Standardized all filenames to PascalCase (`FleetAdapter.cpp`, `Nav2RobotHandle.cpp`) to match the `CMakeLists.txt` configuration.

## 2. Missing Link Target (`rmf_fleet_adapter::read_only`)
- **Error**: `Target "fleet_adapter_node" links to: rmf_fleet_adapter::read_only but the target was not found.`
- **Why it occurred**: In ROS 2 Jazzy, `rmf_fleet_adapter` exports several executable targets (like `read_only`). `ament_target_dependencies` sometimes attempts to link against all exported targets, causing failures if the internal dependency tree is not perfectly resolved or if specific components are expected as libraries.
- **Fix**: Replaced the high-level ament dependency for RMF with explicit `target_link_libraries` pointing to the core libraries: `rmf_fleet_adapter::rmf_fleet_adapter`, `rmf_utils::rmf_utils`, and `rmf_traffic::rmf_traffic`.

## 3. RMF API Version Mismatch (EasyFullControl)
- **Error**: `no matching member function for call to 'add_robot'` and type mismatches in navigation callbacks.
- **Why it occurred**: The initial implementation used a legacy prototype of the `EasyFullControl` API. The version in ROS 2 Jazzy is more structured and requires explicit `RobotConfiguration`, `RobotCallbacks`, and `RobotState` objects instead of direct lambda arguments.
- **Fix**: Refactored the registration logic in `FleetAdapter.cpp` to use the new Jazzy API structures.

## 4. `update_position` Signature Change
- **Error**: `no matching member function for call to 'update_position'`
- **Why it occurred**: The `update_position` method in Jazzy's `RobotUpdateHandle` requires a map name and an `Eigen::Vector3d` object, whereas the legacy code passed a `std::vector` and a timestamp.
- **Fix**: Updated `Nav2RobotHandle.cpp` to use the `update()` method (or appropriate `update_position` overload) with `Eigen::Vector3d` and the map name `"L1"`.

## 5. `VehicleTraits` Constructor Parameters
- **Error**: `no matching constructor for initialization of 'rmf_traffic::agv::VehicleTraits'`
- **Why it occurred**: The constructor for `VehicleTraits` now requires a `rmf_traffic::Profile` object as a mandatory third argument to define the robot's physical footprint.
- **Fix**: Created an `rmf_traffic::Profile` from the robot's footprint radius and used it to initialize the traits.

## 6. Invalid Use of `shared_from_this()`
- **Error**: Logical/Potential runtime crash.
- **Why it occurred**: The registration logic was initially placed in the `FleetAdapterNode` constructor. Calling `shared_from_this()` inside a constructor is illegal because the object is not yet managed by a `std::shared_ptr`.
- **Fix**: Moved the fleet and robot registration logic into a separate `init()` method, which is called in `main()` after the node has been safely wrapped in a `std::shared_ptr`.

## 7. Eigen3 Dependency and Includes
- **Error**: Compilation errors regarding Eigen types or missing headers.
- **Why it occurred**: RMF depends heavily on Eigen for geometry calculations, but `CMakeLists.txt` did not explicitly include the Eigen headers or link against the target.
- **Fix**: Added `find_package(Eigen3 REQUIRED)` to `CMakeLists.txt` and added `Eigen3::Eigen` to the link libraries and include paths.

---

## 16. `name 'true' is not defined` in Launch Files
- **Error**: `[ERROR] [launch]: Caught exception in launch: name 'true' is not defined`.
- **Why it occurred**: In ROS 2 Python launch files, boolean values passed in `launch_arguments` that are later evaluated inside a `PythonExpression` (e.g., in Nav2) must follow Python syntax (`True`/`False`). Lowercase `'true'` causes Python to attempt a variable lookup for `true`, which fails.
- **Fix**: Capitalized `'true'`/`'false'` to `'True'`/`'False'` project-wide in launch scripts.

## 17. Missing `nav2_delay_gate` Plugin
- **Error**: `Failed to create behavior delay_gate of type nav2_delay_gate/DelayGate`.
- **Why it occurred**: The `nav2_delay_gate` plugin was referenced in the navigation configuration but is not available in the current Nav2 distribution or environment.
- **Fix**: Removed the plugin reference from `nav_param.yaml`.
# Multi-Robot TF Tree and AMCL Pipeline Debugging Guide

## Issue Overview
When launching a multi-robot Nav2 simulation using Gazebo and ROS 2, the `amcl` localization pipeline (which calculates the `map -> odom` transform) may fail to publish its transform. This results in errors such as `Tf has two or more unconnected trees` or local costmap timeouts:

```text
Timed out waiting for transform from dynominion1/base_footprint to dynominion1/odom to become available, tf error: Invalid frame ID "dynominion1/odom" passed to canTransform argument target_frame - frame does not exist
```

## Root Cause Analysis
The failure of AMCL to broadcast the `map -> odom` transform is often treated as an AMCL problem. In multi-robot setups, however, **the actual root cause is usually a break lower in the TF tree—specifically the missing `odom -> base_footprint` transform.**

AMCL works by calculating the robot's pose in the `map` frame (i.e. finding the `map -> base_footprint` transform) and then mathematically *subtracting* the `odom -> base_footprint` transform to publish the resulting offset as the `map -> odom` transform. If the odometry frame `odom -> base_footprint` is missing, AMCL physically cannot calculate the offset and will silently drop the publication of `map -> odom`.

### Why the `odom -> base_footprint` Transform Was Missing
In this architecture, `odom_modifier.py` subscribes to the odometry topic to broadcast the missing TF frame. However:
1. The `diff_drive_base_controller` was publishing odometry to the nested topic `diff_drive_base_controller/odom` instead of the expected `odom` topic.
2. The user attempted to resolve this by passing remapping arguments (`--ros-args -r diff_drive_base_controller/odom:=odom`) to the `controller_manager` spawner node inside the Python launch file.
3. **The ROS 2 `spawner` executable ignores runtime argument remappings for controllers.** Controllers do not launch as independent nodes; they run as dynamic plugins directly inside the `controller_manager` process, which in this case is owned by the Gazebo Simulation itself (`gz_ros2_control-system` plugin).

Because the topic remained `diff_drive_base_controller/odom`, the `odom_modifier.py` script (which was listening for `odom`) never received a single Odometry message, and therefore never broadcast the `odom -> base_footprint` TF transform. Because `odom` was missing, AMCL collapsed.

## The Solution
To successfully remap topics for a controller running inside Gazebo's `ign_ros2_control` or `gz_ros2_control` ecosystem, **you must apply the remapping parameters at the URDF/XACRO level where the Gazebo plugin is instantiated.**

### 1. Modify the `gazebo_ros2_control.xacro`
Locate the `<plugin>` definition for `GazeboSimROS2ControlPlugin`. Inside the `<ros>` tag, explicitly declare the `<remapping>` parameters.

**Before:**
```xml
    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
          <parameters>$(arg controller_config)</parameters>
          <ros>
            <namespace>$(arg robot_name)</namespace>
          </ros>
        </plugin>
    </gazebo>
```

**After (The Fix):**
```xml
    <gazebo>
        <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
          <parameters>$(arg controller_config)</parameters>
          <ros>
            <namespace>$(arg robot_name)</namespace>
            <!-- Explicitly force the controller to publish directly to the root namespace odom/cmd_vel -->
            <remapping>diff_drive_base_controller/cmd_vel:=cmd_vel</remapping>
            <remapping>diff_drive_base_controller/odom:=odom</remapping>
          </ros>
        </plugin>
    </gazebo>
```

### 2. Verify Fixes
By configuring the URDF to remap the topics natively on startup:
1. The odometry publishes perfectly to `/dynominion1/odom`.
2. `odom_modifier.py` successfully consumes the data and broadcasts `/dynominion1/odom -> /dynominion1/base_footprint`.
3. AMCL now has access to the full base transform chain, allowing it to calculate the position offset and broadcast the `/map -> /dynominion1/odom` transform perfectly.

*Note: Upon booting the simulation, AMCL will log `unconnected trees` for the first 10-25 seconds of initialization before emitting its first valid `map -> odom` transform. This transient warning is normal in Nav2 bringups.*
