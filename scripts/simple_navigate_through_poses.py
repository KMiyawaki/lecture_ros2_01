#!/usr/bin/env python3
# -*- coding: utf_8 -*-

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from oit_robot_utils.pose_conversions import (pose2d_from_amcl,
                                              pose_stamped_from)
from rclpy.action import ActionClient
from rclpy.node import Node


class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_commander')
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)

        # アクション名を変更
        self.nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

        self.pose = None
        self.waypoints = self.create_waypoint_list()
        self.goal_handle = None
        self.timer = self.create_timer(2.0, self.start_navigation)

    def create_waypoint_list(self):
        poses = []
        for p in [(2.67, 0.369), (7.32, 0.394), (7.05, 1.91), (4.3, 2.6)]:
            poses.append(pose_stamped_from(
                p[0], p[1], 0.0, self.get_clock().now().to_msg()))
        return poses

    def amcl_pose_callback(self, msg):
        self.pose = pose2d_from_amcl(msg)

    def start_navigation(self):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_through_poses action server not available.')
            raise KeyboardInterrupt()

        self.send_goals()  # 一括送信
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(0.05, self.control_loop)

    def send_goals(self):
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.waypoints  # リストを丸ごと渡す

        self.get_logger().info('Sending all waypoints to Nav2.')
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goals rejected!')
            raise KeyboardInterrupt()

        self.get_logger().info('Goals accepted!')
        self.goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, msg):
        # NavigateThroughPosesのフィードバックには distance_remaining は直接ない場合があります
        # 現在の進捗を確認するには、必要に応じて log を調整してください
        self.get_logger().info('Navigating through poses...', throttle_duration_sec=2.0)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished!')
        raise KeyboardInterrupt()  # 完了したら終了

    def control_loop(self):
        if self.pose:
            text = f'Current pos: ({self.pose.x:.2f}, {self.pose.y:.2f})'
            self.get_logger().info(text, throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = NavCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
