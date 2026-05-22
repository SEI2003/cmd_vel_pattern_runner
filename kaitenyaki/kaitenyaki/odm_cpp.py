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

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class SquareOdom(Node):
    def __init__(self):
        super().__init__('square_odom')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # C++ノード（/odom）からのトピックを受信
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
        self.straight_distance = 0.5     # ★ 直線距離 
        self.turn_angle = math.pi / 2    # 曲がる角度 (90度)
        self.curve_radius = 0.5          # ★ 曲率半径 

    def odom_callback(self, msg):
        # C++側で計算された正確なIMUのYaw角を受け取る
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

    # 【変更】エンコーダなしに対応：時間経過から進んだ距離を計算する
    def move_straight(self, distance):
        self.get_logger().info(f"直進を開始: 目標 {distance}m")
        start_time = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # 経過時間から疑似的に移動距離を算出 (距離 = 速度 × 時間)
            current_time = self.get_clock().now()
            dt = (current_time - start_time).nanoseconds / 1e9
            traveled_distance = self.linear_speed * dt

            if traveled_distance >= distance:
                break

            self.publish_velocity(self.linear_speed, 0.0)

        self.stop_robot() # 次の旋回に移る前に一度ピタッと止める

    # 【調整】半径1mのなめらかなカーブを行う
    def rotate_right_curve(self, angle):
        # ⬇️ 【追加】溜まった古いオドメトリをすべて処理して、最新の角度に追いつかせる
        # 0.05秒間、届いているデータを一気にさばきます
        flush_start = self.get_clock().now()
        while (self.get_clock().now() - flush_start).nanoseconds / 1e9 < 0.05:
            rclpy.spin_once(self, timeout_sec=0.001)

        # 最新の状態に追いついた上で、現在の正しい角度をスタート地点にする
        start_yaw = self.current_yaw
        
        curve_radius = 0.3
        curve_angular = self.linear_speed / curve_radius
        threshold = 0.0 

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            diff = self.current_yaw - start_yaw
            while diff >  math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            delta = abs(diff)

            if delta >= angle - threshold:
                break

            self.publish_velocity(self.linear_speed, -curve_angular)

        self.stop_robot()

    def run_experiment(self):
        for i in range(4):
            self.get_logger().info(f"Lap {i+1}/4 | 現在のYaw: {math.degrees(self.current_yaw):.1f}°")
            
            self.move_straight(self.straight_distance)  # 1m直進
            self.rotate_right_curve(self.turn_angle)    # 半径1mで90度カーブ
            
        self.get_logger().info("四角形走行実験が完了しました！")

def main(args=None):
    rclpy.init(args=args)
    node = SquareOdom()

    node.get_logger().info("C++からのIMUオドメトリ(/odom)を待っています...")
    while rclpy.ok() and not node.odom_ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("オドメトリを確認！自律走行を開始します。")
    node.run_experiment()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()