#!/usr/bin/env python3

import sys
import json
import uuid
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rmf_task_msgs.msg import ApiRequest, ApiResponse

class TaskSender(Node):
    def __init__(self, waypoint):
        super().__init__('task_sender')
        
        # Enable simulation time
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.waypoint = waypoint
        self.response_future = rclpy.task.Future()
        
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.pub = self.create_publisher(ApiRequest, '/task_api_requests', transient_qos)
        self.sub = self.create_subscription(ApiResponse, '/task_api_responses', self.response_callback, 10)
        
        self.get_logger().info("TaskSender initialized. Waiting for 2 seconds to synchronize clock and DDS discovery...")
        
    def response_callback(self, msg):
        if msg.request_id == self.request_id:
            self.get_logger().info("Received response from dispatcher!")
            self.response_future.set_result(json.loads(msg.json_msg))
            
    def send_task(self):
        # Calculate time with synchronized sim clock
        now = self.get_clock().now().to_msg()
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        
        self.request_id = 'direct_' + str(uuid.uuid4())
        
        payload = {
            "type": "dispatch_task_request",
            "request": {
                "category": "compose",
                "description": {
                    "category": "go_to_place",
                    "phases": [
                        {
                            "activity": {
                                "category": "go_to_place",
                                "description": {
                                    "one_of": [
                                        {
                                            "waypoint": self.waypoint
                                        }
                                    ]
                                }
                            }
                        }
                    ]
                },
                "unix_millis_earliest_start_time": start_time
            }
        }
        
        msg = ApiRequest()
        msg.request_id = self.request_id
        msg.json_msg = json.dumps(payload)
        
        self.get_logger().info(f"Publishing task request for waypoint '{self.waypoint}' at sim time {now.sec}.{now.nanosec:09d}...")
        self.get_logger().info(f"Payload: {json.dumps(payload, indent=2)}")
        
        # Publish
        self.pub.publish(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--place', required=True, type=str, help='Place to go to')
    args = parser.parse_args()
    
    rclpy.init()
    sender = TaskSender(args.place)
    
    # Spin to allow discovery and clock synchronization
    start_wait = time.time()
    while time.time() - start_wait < 2.0:
        rclpy.spin_once(sender, timeout_sec=0.1)
        
    # Send the task
    sender.send_task()
    
    # Spin until we receive response or timeout (10 seconds)
    rclpy.spin_until_future_complete(sender, sender.response_future, timeout_sec=10.0)
    
    if sender.response_future.done():
        res = sender.response_future.result()
        print(f"\nGot response:\n{json.dumps(res, indent=2)}")
    else:
        print("\nTimeout waiting for response. The task might still have been queued by the dispatcher.")
        
    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
