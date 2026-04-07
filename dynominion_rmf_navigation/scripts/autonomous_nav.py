#!/usr/bin/env python3

import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Patrol Autonomous Navigation Script for Dynominion_rmf
This script navigates the robot through a series of waypoints in the Cafe world
and returns it to the origin.
"""

class AutonomousPatrol(Node):
    def __init__(self):
        super().__init__('autonomous_patrol')
        self.declare_parameter('namespace', 'dynominion1')
        self.declare_parameter('map_frame', 'map')
        
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.navigator = BasicNavigator(namespace=namespace)

        # Set our demo's initial pose (optional but good practice)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = map_frame
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0
        # self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        waypoint_coords = [
            [3.5, 0.0], [3.5, -10.0], [-4.0, -10.0], [-4.0, 0.0],
            [-4.0, 10.0], [3.5, 10.0], [0.0, 0.0]
        ]

        goal_poses = []
        for coords in waypoint_coords:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = map_frame
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = coords[0]
            goal_pose.pose.position.y = coords[1]
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)

        self.get_logger().info("Starting Autonomous Patrol...")
        self.navigator.goThroughPoses(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Estimated arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Patrol successful!')
        else:
            self.get_logger().info('Patrol failed or canceled.')

def main():
    rclpy.init()
    node = AutonomousPatrol()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
