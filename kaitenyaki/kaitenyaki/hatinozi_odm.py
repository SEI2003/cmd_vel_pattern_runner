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

# 角度の差分を -pi から pi の範囲に収める関数
def normalize_angle_diff(diff):
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff

# ROS2のノードを作成
class FigureEightOdomFixed(Node):
    def __init__(self):
        super().__init__('figure_eight_odom')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 速度命令を送信

        # 受信設定
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',    # 自己位置推定
            self.odom_callback, # odom受信時に呼ばれる関数
            10
        )

        # 現在位置を保存
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False # オドメトリが届いたかを判定するためのフラグ

        # 前進速度 [m/s]
        self.linear_speed = 0.1
        # 曲率半径 [m] (滑りを防ぐため、少し大きめの0.3m〜0.4mを推奨)
        self.curve_radius = 0.1
        
        # 旋回角度の定義
        self.full_circle_angle = 2.0 * math.pi  # 1周 (360度)
        self.half_circle_angle = math.pi        # 半周 (180度)
        self.quarter_circle_angle = math.pi / 2 # 4分の1周 (90度)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_ready = True

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)
        time.sleep(0.5)

    # 左旋回を行う関数（累積角度をオドメトリで監視）
    def rotate_left_curve(self, target_angle):
        curve_angular = self.linear_speed / self.curve_radius
        
        # 開始時の角度を保存
        last_yaw = self.current_yaw
        total_turned_angle = 0.0  # 実際に回った累積角度

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # 前回のループからの微小な角度変化を計算
            yaw_diff = self.current_yaw - last_yaw
            yaw_diff = normalize_angle_diff(yaw_diff) # -pi〜piを跨いだ時の補正

            # 左旋回（反時計回り）の移動量を累積
            total_turned_angle += yaw_diff
            last_yaw = self.current_yaw

            # 目標角度に達したら終了 (左旋回なのでtotal_turned_angleは正に増える)
            if total_turned_angle >= target_angle:
                break

            # 速度命令を送信（左旋回はプラス）
            self.publish_velocity(self.linear_speed, curve_angular)

        self.stop_robot()

    # 右旋回を行う関数（累積角度をオドメトリで監視）
    def rotate_right_curve(self, target_angle):
        curve_angular = self.linear_speed / self.curve_radius
        
        # 開始時の角度を保存
        last_yaw = self.current_yaw
        total_turned_angle = 0.0  # 実際に回った累積角度

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # 前回のループからの微小な角度変化を計算
            yaw_diff = self.current_yaw - last_yaw
            yaw_diff = normalize_angle_diff(yaw_diff)

            # 右旋回（時計回り）は角度が減少するので、マイナスをかけて正の移動量として累積
            total_turned_angle += (-yaw_diff)
            last_yaw = self.current_yaw

            # 目標角度に達したら終了
            if total_turned_angle >= target_angle:
                break

            # 速度命令を送信（右旋回はマイナス）
            self.publish_velocity(self.linear_speed, -curve_angular)

        self.stop_robot()

    # 直進運動を行う関数
    def move_straight(self, distance):
        start_x = self.current_x
        start_y = self.current_y

        while rclpy.ok():
            # 【重要】ループの先頭で最新のodom情報を反映させる
            rclpy.spin_once(self, timeout_sec=0.01)

            dx = self.current_x - start_x      # 移動量計算
            dy = self.current_y - start_y      # 移動量計算
            traveled = math.sqrt(dx * dx + dy * dy) # 三平方の定理

            # 指定距離進んだら終了
            if traveled >= distance:
                break

            self.publish_velocity(self.linear_speed, 0.0)
        self.stop_robot()



    # －－－メイン動作－－－
    def run_experiment(self):
        # 1. スタート地点のオドメトリを読み込む
        self.get_logger().info(f"スタート位置: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.1f}°")

        # 1. まずは1m直進
        self.get_logger().info("ステップ0: 1m直進中...")
        self.move_straight(0.2) # 0.4m直進してから旋回に入る（距離は調整可能）


        # 2. 半周(180度)回る（左旋回）
        for i in range(3):    
            self.get_logger().info(f"左回転中... (Lap {i+1}/3)")
            self.move_straight(0.3)
            self.rotate_left_curve(self.quarter_circle_angle)

        #   3m直進
        self.move_straight(0.6) # 1.2m直進してから次の旋回に入る（距離は調整可能）

        # 4 migi旋回）
        for i in range(3):
            self.get_logger().info(f"右回転中... (Lap {i+1}/3)")
            self.move_straight(0.3)
            self.rotate_right_curve(self.quarter_circle_angle)

        self.move_straight(0.4)     #後に少し前進してから停止

        # 初期位置に到達して停止
        self.stop_robot()
        self.get_logger().info(f"最終位置: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.1f}°")
        self.get_logger().info("8の字走行実験が完了し、安全に停止しました。")


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightOdomFixed()

    # 1. 最初にオドメトリのデータが1回届くまでスピンして待つ
    node.get_logger().info("Waiting for initial odom...")
    while rclpy.ok() and not node.odom_ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 2. データを無事受信したら実験（メイン動作）を開始
    node.get_logger().info("Starting figure eight experiment...")
    node.run_experiment()

    # 3. 実験が終わったらノードを綺麗にシャットダウン
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
