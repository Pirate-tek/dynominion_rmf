# Dynominion_rmf

A ROS 2–based open-source Autonomous Mobile Robot platform featuring 80 kg payload capacity, robotic arm integration, and modular hardware for robotics education and applied research.

![RobotoAI Dyno Minion](dynominion_rmf_x/robotoai_dyno_minion.jpeg)

## Features

- **High Payload Capacity**: Supports up to 80 kg for versatile applications.
- **Modular Hardware**: Designed for easy customization and research.
- **ROS 2 Integration**: Built on the latest ROS 2 framework for robust performance.
- **Simulation Support**: Includes Gazebo worlds and virtual robot model for simulation.

## Installation

1.  **Clone the repository**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/TeamRobotoAI/dynominion_rmf.git
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build**:
    ```bash
    colcon build
    source install/setup.bash
    ```

## Docker Compose

### Compose Command
```bash
docker compose up -d --build dynominion_rmf
```

### Check Container
```bash
docker ps
```
Look for `dynominion_rmf` in the `NAMES` column.

### Exec Command
```bash
docker exec -it dynominion_rmf bash
```


### Real Robot
For operating the physical robot, refer to the [Dynominion_rmf X Real Robot Guide](dynominion_rmf_x/REAL_README.md).

### Simulation
For simulation usage, refer to the [Dynominion_rmf X Simulation Guide](dynominion_rmf_x/SIM_README.md).

## Structure

This repository contains the following packages:

| Package | Description |
|---------|-------------|
| [`dynominion_rmf_description`](dynominion_rmf_description/README.md) | Robot URDF/Xacro models and visualization. |
| [`dynominion_rmf_gazebo`](dynominion_rmf_gazebo/README.md) | Gazebo simulation environments and plugins. |
| [`dynominion_rmf_navigation`](dynominion_rmf_navigation/README.md) | Navigation2 stack configuration. |
| [`dynominion_rmf_slam`](dynominion_rmf_slam/README.md) | SLAM (Simultaneous Localization and Mapping) setup. |
| [`dynominion_rmf_x`](dynominion_rmf_x/README.md) | Entry point for real robot and simulation documentation. |
| [`teleop_robot`](teleop_robot/README.md) | Teleoperation nodes for manual control. |

### File Tree
```
dynominion_rmf  
├── dynominion_rmf_description  
├── dynominion_rmf_gazebo   
├── dynominion_rmf_navigation   
├── dynominion_rmf_slam   
├── dynominion_rmf_x        
├── LICENSE     
├── README.md   
└── teleop_robot    
```

## License

This project is licensed under the terms found in the [LICENSE](LICENSE) file.
