#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy



class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')

        # Parameters
        self.declare_parameter('input_topic', 'diff_drive_base_controller/odom')
        self.declare_parameter('output_topic', 'wheelodom')
        self.declare_parameter('new_frame_id', 'odom') # Note: Frames will be prefixed in controller or here if needed
        self.declare_parameter('new_child_frame_id', 'base_footprint')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscriber and publisher
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(f"Subscribed to {input_topic}, republishing to {output_topic}")

    def odom_callback(self, msg: Odometry):
        # Create a new message based on the received one
        new_msg = msg  # msg is already an Odometry object

        # Get namespace (remove leading /)
        ns = self.get_namespace().lstrip('/')
        
        # Original frames from parameters
        frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value
        child_frame_id = self.get_parameter('new_child_frame_id').get_parameter_value().string_value

        # Prefix frames if namespace exists
        if ns:
            new_msg.header.frame_id = f"{ns}/{frame_id}"
            new_msg.child_frame_id = f"{ns}/{child_frame_id}"
        else:
            new_msg.header.frame_id = frame_id
            new_msg.child_frame_id = child_frame_id

        # Optionally update timestamp to current time
        new_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish modified message
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
