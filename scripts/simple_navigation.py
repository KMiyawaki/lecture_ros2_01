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
        # パブリッシャー。ロボットに速度の指令を送る道具。
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # サブスクライバー。センサの情報の受信器。データが届くと、callback関数が呼ばれる。
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)

        # Nav2のアクションクライアント生成
        self.nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

        self.pose = None
        self.waypoints = self.create_waypoint_list()  # 巡回地点リスト
        self.number_of_poses_remaining = -1  # 残りのウェイポイント数

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
            self.get_logger().error("navigate_through_poses action server not available.")
            raise KeyboardInterrupt()  # 終了
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints
        self.get_logger().info("Sending waypoints to Nav2.")
        self.future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(
            0.05, self.control_loop)  # コントロールループに切り替える

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            raise KeyboardInterrupt()  # 終了
            return

        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # 現在通過中のウェイポイントのインデックスを保存する
        self.number_of_poses_remaining = feedback_msg.feedback.number_of_poses_remaining

    def result_callback(self, result_msg):
        # ナビゲーションの結果を受け取るコールバック
        if result_msg.status == 4:  # SUCCEEDED
            self.get_logger().info("Navigation succeeded!")
        else:
            self.get_logger().warn(
                f"Navigation failed with status code: {result_msg.status}")
        raise KeyboardInterrupt()  # 終了

    def control_loop(self):
        # タイマーで呼ばれる関数。ここにロボットの制御のコードを書く。
        text = f"Poses remaining: {self.number_of_poses_remaining}"
        if self.pose:
            text += f", Current position: ({self.pose.x:.2f}, {self.pose.y:.2f}, {self.pose.theta:.2f})"
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
