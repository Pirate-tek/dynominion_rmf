
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_checker_tmp')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.found_frames = set()

    def listener_callback(self, msg):
        for transform in msg.transforms:
            frame_id = transform.header.frame_id
            child_frame_id = transform.child_frame_id
            pair = (frame_id, child_frame_id)
            if pair not in self.found_frames:
                self.found_frames.add(pair)
                print(f"Transform: {frame_id} -> {child_frame_id}")

def main():
    rclpy.init()
    node = TFChecker()
    print("Listening for TF transforms for 5 seconds...")
    start_time = time.time()
    while time.time() - start_time < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("\nSummary of all detected transform pairs:")
    for f, c in sorted(list(node.found_frames)):
        print(f"  {f} -> {c}")
    rclpy.shutdown()

import time
if __name__ == '__main__':
    main()
