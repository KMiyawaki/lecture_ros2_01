#!/usr/bin/env python3
# -*- coding: utf_8 -*-

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from oit_robot_utils.pose_conversions import pose2d_from_amcl
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-publisher-node


class SimpleMove(Node):
    def __init__(self):
        super().__init__('simple_move')

        # パブリッシャー。ロボットに速度の指令を送る道具。
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        # サブスクライバー。センサの情報の受信器。データが届くと、callback関数が呼ばれる。
        self.sub_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        # コールバックで受信したデータの入れ物
        self.scan = None  # レーザースキャン
        self.pose = None  # 地図中の位置姿勢
        self.odom = None  # オドメトリの位置姿勢と速度

        # タイマー（20Hz: 0.05秒間隔）。一定周期で指定された関数を呼ぶ。
        self.timer = self.create_timer(0.05, self.waiting_loop)  # 最初は待ちループ

        # 状態と変数
        self.start_time = None  # プログラムの開始時刻。

    def scan_callback(self, msg):
        # コールバックではメッセージ保存のような軽い処理しかしない。
        self.scan = msg

    def amcl_pose_callback(self, msg):
        self.pose = pose2d_from_amcl(msg)

    def odom_callback(self, msg):
        self.odom = msg

    def waiting_loop(self):
        # サブスクライバーが接続されるのを待つループ。接続されるまでは何もしない。
        if self.pub_cmd_vel.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber...', throttle_duration_sec=1.0)
            return
        self.destroy_timer(self.timer)
        self.get_logger().info('Subscriber connected. Starting control loop.')
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(
            0.05, self.control_loop)  # コントロールループに切り替える

    def control_loop(self):
        # タイマーで呼ばれる関数。ここにロボットの制御のコードを書く。
        tm = self.get_clock().now()
        from_start = (tm - self.start_time).nanoseconds / 1e9
        if self.pose:
            # 地図中の現在位置の表示
            self.get_logger().info(
                f'{from_start:.2f}sec, (x, y, theta): {self.pose.x:.2f}, {self.pose.y:.2f}, {math.degrees(self.pose.theta):.2f}')

        msg = Twist()
        # 動作の継続時間や速度を色々変更してみましょう。
        # 速度に負の数を入れてみましょう。
        # elif節を増やして様々な動きをさせましょう。例：正方形を描かせる。
        if from_start < 5.0:  # 最初の5秒は前進
            msg.linear.x = 0.2  # 並進速度(m/s)
        elif from_start < 8.0:  # 次の3秒は旋回
            msg.angular.z = math.radians(30)  # 旋回角速度(ラジアン/s)
        # elif from_start < 11.0:  # 次の3秒の動作。並進速度と旋回角速度を両方与えることもできます。
        #    msg.linear.x = 0.2  # 並進速度(m/s)
        #    msg.angular.z = math.radians(30)  # 旋回角速度(ラジアン/s)
        else:
            self.get_logger().info('Target time reached. Stopping.')
            self.pub_cmd_vel.publish(msg)  # 停止指令を送る
            raise KeyboardInterrupt()     # 終了
        self.pub_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimpleMove()
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
