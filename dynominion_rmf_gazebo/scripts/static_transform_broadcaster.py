#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import xml.etree.ElementTree as ET
import math

class StaticJointBroadcaster(Node):
    def __init__(self):
        super().__init__('static_transform_broadcaster')
        
        self.declare_parameter('robot_description', '')
        self.declare_parameter('use_sim_time', True)
        
        self.broadcaster = StaticTransformBroadcaster(self)
        
        robot_desc = self.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_desc:
            self.get_logger().error("No robot_description provided!")
            return

        self.publish_static_transforms(robot_desc)

    def publish_static_transforms(self, xml_string):
        try:
            root = ET.fromstring(xml_string)
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
            return

        static_transforms = []
        
        # We look for joints of type "static" or with the special relevance tag
        for joint in root.findall('joint'):
            joint_type = joint.get('type')
            is_static = (joint_type == 'static')
            
            # Also check for relevance tag or special property
            relevance = joint.find('relevance')
            if relevance is not None and relevance.text == 'static':
                is_static = True
            
            if is_static:
                parent = joint.find('parent').get('link')
                child = joint.find('child').get('link')
                
                # Parse origin
                origin = joint.find('origin')
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]
                
                if origin is not None:
                    if origin.get('xyz'):
                        xyz = [float(x) for x in origin.get('xyz').split()]
                    if origin.get('rpy'):
                        rpy = [float(x) for x in origin.get('rpy').split()]

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = parent
                t.child_frame_id = child
                
                t.transform.translation.x = xyz[0]
                t.transform.translation.y = xyz[1]
                t.transform.translation.z = xyz[2]
                
                # Convert RPY to Quaternion
                q = self.euler_to_quaternion(rpy[0], rpy[1], rpy[2])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                
                static_transforms.append(t)
                self.get_logger().info(f"Broadcasting static transform: {parent} -> {child}")

        if static_transforms:
            self.broadcaster.sendTransform(static_transforms)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = StaticJointBroadcaster()
    # No need to spin forever if we only publish once and use the latched broadcaster?
    # Actually, for StaticTransformBroadcaster in Python, you often need to keep the node alive?
    # Yes, rclpy.spin() keeps the node alive so the latched topic works.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
