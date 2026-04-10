# RMF Traffic Map Verification & Corrections Guide (Phase 4)

This document details the architectural corrections and implementation steps performed to align the `dynominion_rmf_maps` package with system requirements. This serves as a study guide for understanding the relationship between ROS 2 Navigation (Nav2) and Open-RMF map coordination.

---

## 1. Package Infrastructure Alignment
### Correction: Missing ROS 2 Build System Files
- **Change**: Created `package.xml` and `CMakeLists.txt` in the `dynominion_rmf_maps` directory.
- **Why**: The directory was previously a plain folder. Open-RMF requires these files to recognize the directory as a package, allowing it to be built using `colcon` and referenced by other nodes (like `building_map_server`).
- **Study Tip**: `rmf_building_map_msgs` is a critical dependency for any RMF map package to correctly export navigation data.

---

## 2. Nav2 & RMF Map Synchronization
### Correction: Map Naming and Source Consistency
- **Change**: Copied `dynominion_rmf_map.pgm` and `.yaml` from `dynominion_rmf_navigation` and renamed them to `dynominion_map.pgm` and `dynominion_map.yaml`. 
- **Why**: Consistency between the navigation stack and the RMF traffic layer is vital. If Nav2 uses a different map than RMF, the robot's physical position will drift or collide with invisible "traffic obstacles" defined in RMF but not in the physical world.

### Correction: Image Reference in Yaml
- **Change**: Updated internal reference in `dynominion_map.yaml` from `image: dynominion_rmf_map.pgm` to `image: dynominion_map.pgm`.
- **Why**: Prevents "File Not Found" errors when the ROS 2 Map Server attempts to load the map after the rename.

---

## 3. Coordinate System & Scale (CRITICAL)
### Correction: Pixel-to-Meter Transformation
- **Change**: Adjusted vertex coordinates in `dynominion.building.yaml` to use calculated meter values instead of pixel indices.
- **Why**: 
    - **Original State**: The traffic-editor was using 1:1 pixel scale, meaning a vertex at `[160, 180]` pixels was being exported to the navigation graph as `[160m, 180m]`. This made the map 20x larger than the real cafe.
    - **Corrected State**: Using the formula `Meter = (Pixel * Resolution) + Origin_Offset`, we shifted coordinates to match Gazebo's physical ground truth.
- **Formula Used**:
    - $X_{world} = (X_{pixel} \times 0.05) - 5.016$
    - $Y_{world} = ((Height_{pixel} - Y_{pixel, TE}) \times 0.05) - 11.099$
- **Study Tip**: Open-RMF's `building_map_generator` negates the Y-axis during export because typical image formats count Y from the top-down, while ROS 2 counts from the bottom-up. To get a POSITIVE meter value in the output `0.yaml`, the input Y in `building.yaml` must be NEGATIVE.

---

## 4. Semantic Waypoint Naming
### Correction: Abstract vs. Functional Names
- **Change**: Renamed waypoints from generic names like `station1`, `station4` to semantic names like `charger_1`, `table_1`, `entry_point`.
- **Why**: RMF task dispatchers rely on string names to assign tasks (e.g., "Go to Table 1"). Clear naming prevents human error and allows the Fleet Adapter to correctly identify parking spots and charging stations.
- **Study Tip**: Always ensure `is_charger: true` is tagged for waypoints named `charger_X` if you want the robot to perform autonomous charging.

---

## 5. Navigation Graph Verification
### Correction: Scale Determination
- **Change**: Added a `measurement` field to the `building.yaml` between two vertices with a calculated real-world distance of `7.0418m`.
- **Why**: Without a measurement, the generator tool issues a warning and defaults to pixel units. Adding the measurement forces the generator to use the correct scale factor (1.0 in our case, since we pre-metered the vertices).

---

## Final Package Structure
- `building/dynominion.building.yaml`: Source of truth for designers.
- `maps/dynominion_map.pgm`: Physical occupancy grid.
- `nav_graphs/0.yaml`: Logical graph consumed by RMF Fleet Adapters.
- `CMakeLists.txt` & `package.xml`: Installation and build instructions.
