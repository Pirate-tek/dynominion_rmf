
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import time

class TFScanner(Node):
    def __init__(self):
        super().__init__('tf_scanner_tmp')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan(self):
        time.sleep(2) # Wait for some TFs to come in
        frames = self.tf_buffer.all_frames_as_yaml()
        print("TF Frames (YAML format):")
        print(frames)

def main():
    rclpy.init()
    node = TFScanner()
    node.scan()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
