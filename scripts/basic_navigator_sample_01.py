#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from threading import Thread

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
# https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/robot_navigator.py
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from oit_robot_utils.pose_conversions import (pose2d_from_amcl,
                                              pose_stamped_from)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class NavCommander(Node):
    def __init__(self):
        super().__init__('nav_commander')
        # ナビゲーションを簡単に扱うライブラリ。マルチスレッド推奨。
        self.nav = BasicNavigator()
        self.scan = None  # レーザースキャン
        self.amcl_pose = None  # 地図中の位置姿勢
        self.odom = None  # オドメトリの位置姿勢と速度

        # パブリッシャー。ロボットに速度の指令を送る道具。
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # サブスクライバー。センサの情報の受信器。データが届くと、callback関数が呼ばれる。
        # 今回は別スレッドでspinするので、ReentrantCallbackGroupを使う。
        # 同じグループに所属するコールバック関数の並列実行を許可する。
        self.cp_group = ReentrantCallbackGroup()
        self.sub_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10, callback_group=self.cp_group)
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10, callback_group=self.cp_group)
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10, callback_group=self.cp_group)

    # コールバックではメッセージ保存のような軽い処理しかしない。
    # 参照の代入だけなのでスレッドセーフ
    def scan_callback(self, msg):
        self.scan = msg

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def odom_callback(self, msg):
        self.odom = msg

    def loop(self):
        '''
        メインスレッドで動作するブロッキングループ
        ここではシーケンシャルな処理を記述できる。
        '''
        self.get_logger().info('ユーザー処理ループ開始')
        # 10Hz で管理するレートオブジェクトを作成
        rate = self.create_rate(10)
        # ナビゲーションが準備完了になるまで無限待ちする。タイムアウトが無いのでここは要検討。
        self.nav.waitUntilNav2Active()
        # サブスクライバーが接続されるのを待つループ。接続されるまでは何もしない。
        while self.pub_cmd_vel.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber...', throttle_duration_sec=1.0)
            rate.sleep()

        self.get_logger().info('目標を送信')
        self.nav.goToPose(pose_stamped_from(2.67, 0.369, 0.0,
                          self.get_clock().now().to_msg()))  # 最初の目標を送る
        self.get_logger().info('送信完了。ナビゲーション開始')

        # 自律移動させる。
        while not self.nav.isTaskComplete() and rclpy.ok():
            # 参照の読み取りだけなのでスレッドセーフ
            if self.amcl_pose:
                pose_2d = pose2d_from_amcl(self.amcl_pose)
                self.get_logger().info(
                    f'現在位置: x={pose_2d.x:.2f}, y={pose_2d.y:.2f}, theta={pose_2d.theta:.2f}')
            rate.sleep()
        self.get_logger().info('送信完了。ナビゲーション終了')

        # 終わった後にすこしだけ前進してみる。
        self.get_logger().info('前進')
        start_time = self.get_clock().now()
        duration = Duration(seconds=5)
        # 10秒経過したらループを抜ける
        while (self.get_clock().now() - start_time) <= duration and rclpy.ok():
            twist = Twist()
            twist.linear.x = 0.2
            self.pub_cmd_vel.publish(twist)
        self.pub_cmd_vel.publish(Twist())  # 止まる
        self.get_logger().info('ユーザー処理ループ終了')
        raise KeyboardInterrupt()  # 終了


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NavCommander()
        # センサー受信用のスレッド（通信）
        # 内部のコールバックが増えたとき、スレッド数を増やす。
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        Thread(target=executor.spin, daemon=True).start()
        # メインスレッドでユーザーループを実行
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
