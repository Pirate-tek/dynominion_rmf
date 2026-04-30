
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class MapListener(Node):
    def __init__(self):
        super().__init__('map_listener_tmp')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            qos)

    def listener_callback(self, msg):
        print(f"Map Received!")
        print(f"Frame ID: {msg.header.frame_id}")
        print(f"Resolution: {msg.info.resolution}")
        print(f"Origin: {msg.info.origin}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = MapListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
