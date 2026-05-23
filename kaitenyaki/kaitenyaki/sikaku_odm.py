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

# 角度を-π〜πに正規化
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

        # 現在位置を保存
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False # オドメトリが届いたかを判定するためのフラグ

        # 前進速度 [m/s]
        self.linear_speed = 0.5
        # 直線距離 [m]
        self.straight_distance = 0.5
        # 曲がる角度 [rad]（90度）
        self.turn_angle = math.pi/2
        # 曲率半径 [m]
        self.curve_radius = 0.5
        # ※ タイマーは廃止しました

    # odomを受信する関数
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

        #self.stop_robot()

    # 半径5mで90度右旋回を行う関数
    """
    def rotate_right_curve(self, angle):
        start_yaw = self.current_yaw

        # 円運動の式から角速度を計算
        curve_angular = self.linear_speed / self.curve_radius

        while rclpy.ok():
            # 【重要】ループの先頭で最新のodom情報を反映させる
            rclpy.spin_once(self, timeout_sec=0.01)

            delta = normalize_angle(start_yaw - self.current_yaw)   # 角度の変化量

            if abs(delta) >= angle:     # 指定の角度を曲がったら終了
                break

            self.publish_velocity(self.linear_speed, -curve_angular)    # 右旋回（マイナス）

        self.stop_robot()   # ロボットを停止させる
    """

    def rotate_right_curve(self, angle):
        start_yaw = self.current_yaw

        # ① 曲率半径を大きくして角速度を下げる（例: 0.3m）
        curve_radius = 0.5
        #curve_angular = self.linear_speed / curve_radius  # = 0.67 rad/s
        curve_angular = 0.157
        # ② 手前で止まるためのしきい値を設ける
        threshold = 0.1  # rad（約6度の余裕）

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # ③ abs() で符号問題を排除
            diff = self.current_yaw - start_yaw
            while diff >  math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            delta = abs(diff)

            # ④ しきい値分手前で終了
            if delta >= angle - threshold:
                break

            self.publish_velocity(self.linear_speed, -curve_angular)

        self.stop_robot()

    # －－－メイン動作－－－
    def run_experiment(self):
        for i in range(4):
            self.get_logger().info(f"x {self.current_x}, y {self.current_y}, yaw {math.degrees(self.current_yaw):.1f}°")    # 現在の位置と角度をログに出力
            self.get_logger().info(f"Lap {i+1}/4")

            self.move_straight(self.straight_distance)  # 直進
            self.rotate_right_curve(self.turn_angle)    # 右カーブ
        self.get_logger().info("Square odom experiment complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SquareOdom()

    # 1. 最初にオドメトリのデータが1回届くまでスピンして待つ
    node.get_logger().info("Waiting for initial odom...")
    while rclpy.ok() and not node.odom_ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 2. データを無事受信したら実験（メイン動作）を開始
    node.get_logger().info("Starting square odom experiment...")
    node.run_experiment()

    # 3. 実験が終わったらノードを綺麗にシャットダウン
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()