
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import time

class TFStaticChecker(Node):
    def __init__(self):
        super().__init__('tf_static_checker_tmp')
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=100
        )
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.listener_callback,
            qos)
        self.found_frames = set()

    def listener_callback(self, msg):
        for transform in msg.transforms:
            frame_id = transform.header.frame_id
            child_frame_id = transform.child_frame_id
            pair = (frame_id, child_frame_id)
            if pair not in self.found_frames:
                self.found_frames.add(pair)
                print(f"Static Transform: {frame_id} -> {child_frame_id}")

def main():
    rclpy.init()
    node = TFStaticChecker()
    print("Listening for Static TF transforms for 3 seconds...")
    start_time = time.time()
    while time.time() - start_time < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\nSummary of all detected Static transform pairs:")
    for f, c in sorted(list(node.found_frames)):
        print(f"  {f} -> {c}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
