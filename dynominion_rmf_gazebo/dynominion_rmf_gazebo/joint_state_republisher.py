#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')
        # match the broadcaster QoS (reliable + transient_local)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub = self.create_subscription(
            JointState, 'joint_states', self.cb, 10)

        # republish with same QoS as joint_state_broadcaster so RSP receives it
        self.pub = self.create_publisher(JointState, 'joint_states_republished', qos)

    def cb(self, msg: JointState):
        # create a cleaned copy
        fixed = JointState()
        # stamp the message with the current ROS time (simulation time if use_sim_time=True)
        now = self.get_clock().now().to_msg()
        fixed.header.stamp = now
        # ensure frame_id empty (normal for joint_states)
        fixed.header.frame_id = ''
        fixed.name = msg.name
        fixed.position = [round(p, 4) for p in msg.position]
        fixed.velocity = [round(v, 4) for v in msg.velocity]
        fixed.effort   = [0.0 if not v == v else round(v, 4) for v in msg.effort]
        # publish corrected topic
        self.pub.publish(fixed)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
