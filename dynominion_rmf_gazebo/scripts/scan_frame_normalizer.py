#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameNormalizer(Node):
    def __init__(self):
        super().__init__('scan_frame_normalizer')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_corrected')
        self.declare_parameter('output_frame_id', 'laser')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        output_frame_id = self.get_parameter('output_frame_id').get_parameter_value().string_value

        self._output_frame_id = output_frame_id

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos,
        )
        self.publisher = self.create_publisher(LaserScan, output_topic, qos)

        self.get_logger().info(
            f"Subscribed to {input_topic}, republishing normalized scans to {output_topic}"
        )

    def scan_callback(self, msg: LaserScan):
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max
        scan.ranges = msg.ranges
        scan.intensities = msg.intensities

        ns = self.get_namespace().lstrip('/')
        scan.header.frame_id = f"{ns}/{self._output_frame_id}" if ns else self._output_frame_id

        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
