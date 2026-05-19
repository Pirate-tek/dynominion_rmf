import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState
import yaml
import math
import os

class RobotDistanceMonitor(Node):
    def __init__(self, graph_path):
        super().__init__('robot_distance_monitor')
        
        # Load the graph
        self.waypoints = []
        if os.path.exists(graph_path):
            with open(graph_path, 'r') as f:
                graph_data = yaml.safe_load(f)
                # In RMF graphs, waypoints are under levels -> [LevelName] -> vertices
                for level_name, level_data in graph_data.get('levels', {}).items():
                    for i, vertex in enumerate(level_data.get('vertices', [])):
                        # vertex is [x, y, {params}]
                        x, y, params = vertex
                        name = params.get('name', f'wp_{i}')
                        self.waypoints.append({'name': name, 'x': x, 'y': y, 'level': level_name})
        
        if not self.waypoints:
            self.get_logger().error(f"Failed to load waypoints from {graph_path}")
            return

        self.subscription = self.create_subscription(
            FleetState,
            '/fleet_states',
            self.listener_callback,
            10)
        
        self.get_logger().info(f"Monitoring distances for {len(self.waypoints)} waypoints.")

    def listener_callback(self, msg):
        # Clear screen for live feel
        print("\033[H\033[J", end="")
        print(f"{'Robot':<15} | {'Waypoint':<20} | {'Distance (m)':<15}")
        print("-" * 55)

        for robot in msg.robots:
            r_x = robot.location.x
            r_y = robot.location.y
            r_level = robot.location.level_name
            
            for wp in self.waypoints:
                # Only compare if on the same level
                if wp['level'] == r_level:
                    dist = math.sqrt((r_x - wp['x'])**2 + (r_y - wp['y'])**2)
                    # Highlight very close waypoints
                    color = "\033[92m" if dist < 0.5 else ""
                    reset = "\033[0m" if color else ""
                    print(f"{robot.name:<15} | {wp['name']:<20} | {color}{dist:>12.3f}{reset}")
            print("-" * 55)

def main():
    rclpy.init()
    # Path found in previous steps
    graph_path = "/home/jazzy/dynominion_rmf/install/dynominion_rmf_maps/share/dynominion_rmf_maps/nav_graphs/0.yaml"
    node = RobotDistanceMonitor(graph_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
