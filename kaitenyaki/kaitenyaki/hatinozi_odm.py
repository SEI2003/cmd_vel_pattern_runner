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

# ROS2のノードを作成
class FigureEightOdom(Node):
    # 初期化設定
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
        self.linear_speed = 0.2
        # 曲率半径 [m] (円の大きさ)
        self.curve_radius = 0.35
        # 旋回する角度 [rad] (1周 = 360度)
        self.circle_angle = 2.0 * math.pi
        # 手前で止まるためのしきい値 [rad] (オーバーシュート対策)
        self.threshold = 0.15 

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

    # 左旋回を行う関数
    def rotate_left_curve(self, angle):
        start_yaw = self.current_yaw
        curve_angular = self.linear_speed / self.curve_radius

        self.get_logger().info("左旋回を開始...")
        self.get_logger().info(f"Initial Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.1f}°")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # 角度の差分計算（-π〜πに正規化）
            diff = self.current_yaw - start_yaw
            while diff >  math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            
            # 左旋回は反時計回り（正の方向）に角度が増える
            delta = diff
            if delta < 0:
                delta += 2 * math.pi  # 累積角度を正の数で扱う

            # しきい値分手前で終了
            if delta >= angle - self.threshold:
                break

            self.publish_velocity(self.linear_speed, curve_angular)  # 左旋回（プラス）

        self.stop_robot()

    # 右旋回を行う関数
    def rotate_right_curve(self, angle):
        start_yaw = self.current_yaw
        curve_angular = self.linear_speed / self.curve_radius

        self.get_logger().info("右旋回を開始...")
        self.get_logger().info(f"Initial Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.1f}°")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # 角度の差分計算（-π〜πに正規化）
            diff = start_yaw - self.current_yaw
            while diff >  math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            
            # 右旋回は時計回り（負の方向）に角度が減るので、差分を反転させて正の数にする
            delta = diff
            if delta < 0:
                delta += 2 * math.pi

            # しきい値分手前で終了
            if delta >= angle - self.threshold:
                break

            self.publish_velocity(self.linear_speed, -curve_angular) # 右旋回（マイナス）

        self.stop_robot()

    # －－－メイン動作－－－
    def run_experiment(self):
        self.get_logger().info(f"Initial Pose: x={self.current_x:.2f}, y={self.current_y:.2f}, yaw={math.degrees(self.current_yaw):.1f}°")
        
        # 1. 左回りに1回転
        self.rotate_left_curve(self.circle_angle)
        
        # 2. 右回りに1回転
        self.rotate_right_curve(self.circle_angle)
        
        self.get_logger().info("Figure eight experiment complete.")


def main(args=None):
    rclpy.init(args=args)
    node = FigureEightOdom()

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