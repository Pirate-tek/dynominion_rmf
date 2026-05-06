#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rmf_building_map_msgs.msg import BuildingMap, Graph

class NavGraphBridge(Node):
    def __init__(self):
        super().__init__('nav_graph_bridge')
        
        self.subscription = self.create_subscription(
            BuildingMap,
            '/floorplan',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            Graph,
            '/nav_graphs',
            10)
        
        self.get_logger().info('Nav Graph Bridge started. Listening on /floorplan and publishing to /nav_graphs')

    def listener_callback(self, msg):
        if msg.nav_graphs:
            # We publish the first graph found in the building map
            self.publisher.publish(msg.nav_graphs[0])
            # self.get_logger().info(f'Published graph: {msg.nav_graphs[0].name}')

def main(args=None):
    rclpy.init(args=args)
    nav_graph_bridge = NavGraphBridge()
    rclpy.spin(nav_graph_bridge)
    nav_graph_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
