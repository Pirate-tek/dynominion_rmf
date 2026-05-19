#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class MultiInitialPosePublisher(Node):

    def __init__(self):
        super().__init__('multi_initial_pose_publisher')

        self.robots = [
            {'name': 'dynominion1', 'x': 1.487486, 'y': -5.617650, 'yaw': 0.0},
            {'name': 'dynominion2', 'x': 1.487486, 'y': -9.518767, 'yaw': 0.0},
            {'name': 'dynominion3', 'x': 7.890055, 'y': -9.518767, 'yaw': 0.0},
            {'name': 'dynominion4', 'x': 7.711421, 'y': -5.558105, 'yaw': 0.0},
            {'name': 'dynominion5', 'x': 2.112862, 'y': -11.662898, 'yaw': 0.0},
        ]

        # ✅ FIX: avoid conflict with Node internals
        self.pose_publishers = []

        for robot in self.robots:
            topic = f"/{robot['name']}/initialpose"
            pub = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
            self.pose_publishers.append((pub, robot))

        # Wait a bit so AMCL is ready
        self.timer = self.create_timer(2.0, self.publish_initial_poses)
        self.published = False

    def yaw_to_quaternion(self, yaw):
        """Convert yaw (rad) to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def publish_initial_poses(self):
        if self.published:
            return

        for pub, robot in self.pose_publishers:
            msg = PoseWithCovarianceStamped()

            # Header
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()

            # Position
            msg.pose.pose.position.x = robot['x']
            msg.pose.pose.position.y = robot['y']
            msg.pose.pose.position.z = 0.0

            # Orientation
            qz, qw = self.yaw_to_quaternion(robot['yaw'])
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            # Covariance (important for AMCL)
            msg.pose.covariance = [
                0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0.25, 0, 0, 0,
                0, 0, 0, 0.0685, 0, 0,
                0, 0, 0, 0, 0.0685, 0,
                0, 0, 0, 0, 0, 0.0685
            ]

            pub.publish(msg)
            self.get_logger().info(f"✅ Published initial pose for {robot['name']}")

        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = MultiInitialPosePublisher()

    # Spin just long enough to publish once
    rclpy.spin_once(node, timeout_sec=3.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
