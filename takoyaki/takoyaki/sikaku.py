#!/usr/bin/env python3
#秒数で□描くやつ
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SquareCmdVel(Node):
    def __init__(self):
        super().__init__('square_cmdvel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # 前進速度 [m/s]
        self.linear_speed = 0.2
        # 角速度 [rad/s]（現在未使用）
        self.angular_speed = 0.5
        # 直線移動距離 [m]
        self.straight_distance = 1.0
        # 曲がる角度 [rad]（90度）
        self.turn_angle = math.pi
        # 曲線旋回半径 [m]
        self.curve_radius = 0.1

        self.run_experiment()

    def publish_cmd(self, linear, angular, duration):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        start = time.time()
        while time.time() - start < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)  # 20Hz相当
        stop = Twist()
        self.publisher_.publish(stop)
        time.sleep(0.5)

    def run_experiment(self):
        for _ in range(4):
            # 直進時間 = 距離 / 速度
            straight_time = self.straight_distance / self.linear_speed

            # 曲率半径から必要角速度を計算
            # R = v / ω → ω = v / R
            curve_angular = self.linear_speed / self.curve_radius

            # 90度旋回に必要な時間
            curve_time = self.turn_angle / curve_angular
            self.get_logger().info('直線移動に入ります')
            self.publish_cmd(self.linear_speed, 0.0, straight_time)
            self.get_logger().info('カーブに入ります')
            self.publish_cmd(self.linear_speed, curve_angular, curve_time)
        self.get_logger().info("Square experiment complete.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SquareCmdVel()
    rclpy.spin(node)

if __name__ == '__main__':
    main()