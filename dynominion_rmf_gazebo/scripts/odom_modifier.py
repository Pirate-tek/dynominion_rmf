#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster



class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')

        # Parameters
        self.declare_parameter('input_topic', 'odom')
        self.declare_parameter('output_topic', 'wheelodom')
        self.declare_parameter('new_frame_id', 'odom') # Note: Frames will be prefixed in controller or here if needed
        self.declare_parameter('new_child_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

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
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Subscribed to {input_topic}, republishing to {output_topic}")

    @staticmethod
    def _qualify_frame(ns: str, frame_id: str) -> str:
        if not ns or not frame_id:
            return frame_id

        if frame_id.startswith('/'):
            frame_id = frame_id.lstrip('/')

        if frame_id.startswith(f"{ns}/"):
            return frame_id

        return f"{ns}/{frame_id}"

    def odom_callback(self, msg: Odometry):
        # Create a new message based on the received one
        new_msg = msg  # msg is already an Odometry object

        # Get namespace (remove leading /)
        ns = self.get_namespace().lstrip('/')
        
        # Original frames from parameters
        frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value
        child_frame_id = self.get_parameter('new_child_frame_id').get_parameter_value().string_value

        # Prefix frames if namespace exists
        new_msg.header.frame_id = self._qualify_frame(ns, frame_id)
        new_msg.child_frame_id = self._qualify_frame(ns, child_frame_id)

        # Optionally update timestamp to current time
        new_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish modified message
        self.publisher.publish(new_msg)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = new_msg.header.stamp
            transform.header.frame_id = new_msg.header.frame_id
            transform.child_frame_id = new_msg.child_frame_id
            transform.transform.translation.x = new_msg.pose.pose.position.x
            transform.transform.translation.y = new_msg.pose.pose.position.y
            transform.transform.translation.z = new_msg.pose.pose.position.z
            transform.transform.rotation = new_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
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
