#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from oit_robot_utils.pose_conversions import pose_stamped_from
from rclpy.action import ActionClient
from rclpy.node import Node


class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_commander')

        # FollowPathのアクションクライアント
        self.nav_client = ActionClient(self, FollowPath, 'follow_path')
        self.current_path_index = 0

        # 経路データ（Path型）を生成
        self.path = self.create_path()

        # 準備ができたらナビゲーション開始
        self.timer = self.create_timer(1.0, self.start_path_following)

    def create_path(self):
        """ウェイポイントからPathメッセージを生成する関数"""
        path = Path()
        path.header.frame_id = 'map'
        # 目的地リスト
        waypoints = [(2.67, 0.369), (7.32, 0.394), (7.05, 1.91)]

        for x, y in waypoints:
            path.poses.append(pose_stamped_from(
                x, y, 0.0, self.get_clock().now().to_msg()))
        return path

    def start_path_following(self):
        self.destroy_timer(self.timer)  # タイマーは1回のみ実行

        goal = FollowPath.Goal()
        goal.path = self.path
        self.get_logger().info('Sending path to FollowPath action server.')

        # アクションの送信
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, msg):
        """アクション実行中のフィードバック受信"""
        feedback = msg.feedback
        self.current_path_index = feedback.path_index
        # 進捗をログ出力（throttleで間引き）
        self.get_logger().info(f'Following path... Current index: {self.current_path_index}',
                               throttle_duration_sec=1.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Path rejected by server!')
            return

        self.get_logger().info('Path accepted!')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Path following completed!')


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
