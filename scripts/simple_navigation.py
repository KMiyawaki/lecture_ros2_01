#!/usr/bin/env python3
# -*- coding: utf_8 -*-

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from oit_robot_utils.pose_conversions import (pose2d_from_amcl,
                                              pose_stamped_from)
from rclpy.action import ActionClient
from rclpy.node import Node


class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_commander')
        # パブリッシャー。ロボットに速度の指令を送る道具。
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # サブスクライバー。センサの情報の受信器。データが届くと、callback関数が呼ばれる。
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)

        # Nav2のアクションクライアント生成
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.pose = None
        self.waypoints = self.create_waypoint_list()  # 巡回地点リスト
        self.crnt_waypoint_index = 0
        self.goal_handle = None

        self.timer = self.create_timer(
            2.0, self.start_navigation)  # 2秒後にナビゲーション開始

    def create_waypoint_list(self):
        # 巡回したいウェイポイントの定義
        poses = []
        for p in [(2.67, 0.369), (7.32, 0.394), (7.05, 1.91), (4.3, 2.6)]:
            poses.append(pose_stamped_from(
                p[0], p[1], 0.0, self.get_clock().now().to_msg()))
        return poses

    def amcl_pose_callback(self, msg):
        self.pose = pose2d_from_amcl(msg)

    def start_navigation(self):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available.')
            raise KeyboardInterrupt()  # 終了
            return

        self.send_goal()  # 最初の目標を送る
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(
            0.05, self.control_loop)  # コントロールループに切り替える

    def send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[self.crnt_waypoint_index]
        self.get_logger().info('Sending goal to Nav2.')
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()  # ClientGoalHandle
        if not self.goal_handle.accepted:
            self.get_logger().error(
                f'Goal {self.crnt_waypoint_index} rejected!')
            self.goal_handle = None
            if self.crnt_waypoint_index == len(self.waypoints) - 1:
                self.get_logger().info('All goals completed. Exiting.')
                raise KeyboardInterrupt()  # 終了
            else:
                self.crnt_waypoint_index = self.crnt_waypoint_index + 1  # 次の目標に切り替え
                self.send_goal()  # 次の目標を送る
            return

        self.get_logger().info(f'Goal {self.crnt_waypoint_index} accepted!')
        future = self.goal_handle.get_result_async()
        future.add_done_callback(self.result_callback)

    def feedback_callback(self, msg):
        self.get_logger().info(
            f'distance_remaining: {msg.feedback.distance_remaining:.2f}m remaining', throttle_duration_sec=1.0)

    def result_callback(self, result_msg):
        self.get_logger().info(f'Goal result: {result_msg}')
        if self.crnt_waypoint_index == len(self.waypoints) - 1:
            self.get_logger().info('All goals completed. Exiting.')
            raise KeyboardInterrupt()  # 終了
        else:
            self.crnt_waypoint_index = self.crnt_waypoint_index + 1  # 次の目標に切り替え
            self.send_goal()  # 次の目標を送る

    def control_loop(self):
        # タイマーで呼ばれる関数。ここにロボットの制御のコードを書く。
        text = f'Goal index: {self.crnt_waypoint_index}'
        if self.pose:
            text += f', Current position: ({self.pose.x:.2f}, {self.pose.y:.2f}, {self.pose.theta:.2f})'
        self.get_logger().info(text, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavCommander()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
