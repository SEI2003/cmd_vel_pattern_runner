#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class SquareOdom(Node):
    def __init__(self):
        super().__init__('square_odom')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',    
            self.odom_callback, 
            10
        )

        self.current_yaw = 0.0
        self.odom_ready = False 

        # --- パラメータ設定 ---
        self.linear_speed = 0.2          # 前進速度 [m/s]
        self.straight_distance = 0.5     # 基本の直線距離
        self.turn_angle = math.pi / 2    # 曲がる角度 (90度)

    def odom_callback(self, msg):
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

    # キューに溜まった古いオドメトリを吐き出す共通処理
    def flush_odom(self):
        flush_start = self.get_clock().now()
        while (self.get_clock().now() - flush_start).nanoseconds / 1e9 < 0.05:
            rclpy.spin_once(self, timeout_sec=0.001)

    def move_straight(self, distance):
        self.flush_odom() # 古いデータをリセット
        self.get_logger().info(f"直進を開始: 目標 {distance}m")
        start_time = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            current_time = self.get_clock().now()
            dt = (current_time - start_time).nanoseconds / 1e9
            traveled_distance = self.linear_speed * dt

            if traveled_distance >= distance:
                break

            self.publish_velocity(self.linear_speed, 0.0)

        self.stop_robot()

    # 右カーブ（時計回り：角速度はマイナス）
    def rotate_right_curve(self, angle):
        self.flush_odom() # 古いデータをリセット
        self.get_logger().info("右カーブを開始")
        start_yaw = self.current_yaw
        
        curve_radius = 0.5
        curve_angular = self.linear_speed / curve_radius

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            diff = self.current_yaw - start_yaw
            while diff >  math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            delta = abs(diff)

            if delta >= angle:
                break

            self.publish_velocity(self.linear_speed, -curve_angular) # マイナス

        self.stop_robot()

    # 【新規追加】左カーブ（反時計回り：角速度はプラス）
    def rotate_left_curve(self, angle):
        self.flush_odom() # 古いデータをリセット
        self.get_logger().info("左カーブを開始")
        start_yaw = self.current_yaw
        
        curve_radius = 0.5
        curve_angular = self.linear_speed / curve_radius

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            diff = self.current_yaw - start_yaw
            while diff >  math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            delta = abs(diff)

            if delta >= angle:
                break

            self.publish_velocity(self.linear_speed, curve_angular) # プラス

        self.stop_robot()

    # －－－ 8の字実験のメイン動作 －－－
    def run_experiment(self):
        # === 前半：右側の四角形（3回ループ） ===
        for i in range(3):
            self.get_logger().info(f"[右エリア] 辺 {i+1}/3 | 現在のYaw: {math.degrees(self.current_yaw):.1f}°")
            self.move_straight(self.straight_distance)    # 0.5m 直進
            self.rotate_right_curve(self.turn_angle)      # 右カーブ 90度

        # === 中間：交差点を2倍の距離で直進 ===
        self.get_logger().info(f"[交差点] 2倍の直進をして左エリアへ移行")
        self.move_straight(self.straight_distance * 2)   # 1.0m 直進（0.5m の 2倍）

        # === 後半：左側の四角形（4回ループして戻ってくる） ===
        for i in range(4):
            self.get_logger().info(f"[左エリア] 辺 {i+1}/4 | 現在のYaw: {math.degrees(self.current_yaw):.1f}°")
            self.rotate_left_curve(self.turn_angle)       # 左カーブ 90度
            
            # 最後のステップだけは、曲がった時点でスタート地点に戻っているので直進は不要
            if i < 3:
                self.move_straight(self.straight_distance) # 0.5m 直進
            
        self.get_logger().info("8の字（ツイン・スクエア）走行実験が完了しました！")

def main(args=None):
    rclpy.init(args=args)
    node = SquareOdom()

    node.get_logger().info("C++からのIMUオドメトリ(/odom)を待っています...")
    while rclpy.ok() and not node.odom_ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("オドメトリを確認！8の字自律走行を開始します。")
    node.run_experiment()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()