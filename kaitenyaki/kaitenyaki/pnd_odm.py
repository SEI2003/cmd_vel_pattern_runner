#!/usr/bin/env python3
# ライブラリ導入
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

# QuaternionからYaw角（2D姿勢角）を計算
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# 角度を-π〜πに正規化する関数
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# ROS2のノードを作成
class SquareOdom(Node):
    # 初期化設定
    def __init__(self):
        super().__init__('square_odom')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 速度命令を送信

        # 受信設定
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',    # 自己位置推定
            self.odom_callback, # odom受信時に呼ばれる関数
            10
        )

        # 現在位置を保存する変数
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False # オドメトリが届いたかを判定するためのフラグ

        # --- 制御用パラメータ（調整値） ---
        self.linear_speed = 0.2       # 前進速度 [m/s]
        self.straight_distance = 1.0  # 直線距離 [m]
        self.turn_angle = math.pi/2   # 曲がる角度 [rad]（90度）
        self.curve_radius = 0.3       # 曲率半径 [m]

    # odomを受信する関数（バックグラウンドで常に動く）
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_ready = True

    # 速度を送信する関数
    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    # 停止動作を行う関数
    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)
        time.sleep(0.5)

    # 直進運動を行う関数
    def move_straight(self, distance):
        start_x = self.current_x
        start_y = self.current_y

        # ループが超高速で空回りするのを防ぐため、50Hz（0.02秒に1回）の周期を設定
        loop_rate = self.create_rate(50)

        while rclpy.ok():
            # 最新のodom情報を反映させる
            rclpy.spin_once(self, timeout_sec=0.0)

            dx = self.current_x - start_x      # X方向の移動量
            dy = self.current_y - start_y      # Y方向の移動量
            traveled = math.sqrt(dx * dx + dy * dy) # 三平方の定理で直線距離を計算

            # 指定距離進んだらループ終了
            if traveled >= distance:
                break

            # 前進命令を送信
            self.publish_velocity(self.linear_speed, 0.0)

            # 50Hzの周期を保つために待機
            loop_rate.sleep()

        self.stop_robot()

    # 右旋回（カーブ）を行う関数
    def rotate_right_curve(self, angle):
        start_yaw = self.current_yaw

        # 円運動の式から角速度を計算（0.2 / 0.3 = 約 0.67 rad/s）
        curve_angular = self.linear_speed / self.curve_radius

        # 行き過ぎ（オーバーシュート）を防ぐためのブレーキしきい値（約5.7度手前）
        threshold = 0.1

        # 回転した角度の合計を安全に計算するための変数
        total_rotated = 0.0
        last_yaw = start_yaw

        # 50Hz（0.02秒に1回）の周期を設定
        loop_rate = self.create_rate(50)

        while rclpy.ok():
            # 最新のodom情報を反映させる
            rclpy.spin_once(self, timeout_sec=0.0)

            # 前回チェック時からの微小な角度変化を計算
            yaw_diff = normalize_angle(self.current_yaw - last_yaw)

            # 右回転（マイナス方向）の変化量を絶対値にして累積足し算する
            total_rotated += abs(yaw_diff)
            last_yaw = self.current_yaw

            # 目標角度（手前のしきい値）に達したらループ終了
            if total_rotated >= angle - threshold:
                break

            # 右旋回命令（マイナス符号）を送信
            self.publish_velocity(self.linear_speed, -curve_angular)

            # 50Hzの周期を保つために待機
            loop_rate.sleep()

        self.stop_robot()

    # －－－メイン動作－－－
    def run_experiment(self):
        for i in range(4):
            self.get_logger().info(f"Lap {i+1}/4")
            self.move_straight(self.straight_distance)  # 直進
            self.rotate_right_curve(self.turn_angle)    # 右カーブ

        self.get_logger().info("Square odom experiment complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SquareOdom()

    # 1. 最初にオドメトリのデータが最低1回届くまで待つ
    node.get_logger().info("Waiting for initial odom...")
    while rclpy.ok() and not node.odom_ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 2. データを無事受信したらメイン動作を開始
    node.get_logger().info("Starting square odom experiment...")
    node.run_experiment()

    # 3. 終わったらノードをシャットダウン
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()