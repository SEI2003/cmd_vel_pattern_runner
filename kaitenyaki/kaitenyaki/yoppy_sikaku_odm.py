import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from enum import Enum

class State(Enum): # 名前管理
    WAITING  = 0
    DRIVING  = 1
    TURNING  = 2
    DONE     = 3

class SquareNode(Node):
    def __init__(self):
        super().__init__('square_node')

        # Publisher / Subscriber
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # パラメータ
        self.side_length   = 1.0   # 辺の長さ (m)
        self.linear_speed  = 0.2   # 直進速度 (m/s)
        self.angular_speed = 1.57  # 回転速度 (rad/s)
        self.turn_thresh   = 0.15  # 回転完了の許容誤差 (rad) ≒ 1度
        self.dist_thresh   = 0.02  # 直進完了の許容誤差 (m)
        self.total_sides   = 4     # 正方形の辺数

        # odom現在値
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # ステップ開始時の基準値
        self.ref_x   = 0.0
        self.ref_y   = 0.0
        self.ref_yaw = 0.0

        # 状態
        self.state = State.WAITING
        self.sides_done = 0

        self.create_timer(0.05, self.control_loop)  # 20Hz
        self.get_logger().info('起動しました。odom待機中...')

    # ------------------------------------------------------------------
    # odomコールバック
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.x   = pos.x
        self.y   = pos.y
        self.yaw = self._quat_to_yaw(msg.pose.pose.orientation)

        if self.state == State.WAITING:
            self._set_ref()
            self.state = State.DRIVING
            self.get_logger().info('odom受信。走行開始!')

    # ------------------------------------------------------------------
    # メイン制御ループ
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.state == State.WAITING:
            return

        if self.state == State.DONE:
            self._publish(0.0, 0.0)
            return

        if self.state == State.DRIVING:
            self._drive()
        elif self.state == State.TURNING:
            self._turn()

    def _drive(self):
        dist = self._distance()
        if dist < self.side_length - self.dist_thresh:
            self._publish(linear=self.linear_speed)
        else:
            self.get_logger().info(f'辺{self.sides_done + 1} 完了 (dist={dist:.3f}m)')
            self._publish(0.0, 0.0)
            self._set_ref()
            self.state = State.TURNING

    def _turn(self):
        angle = self._delta_yaw()
        if angle < math.pi / 2 - self.turn_thresh:
            self._publish(angular=self.angular_speed)
        else:
            self.sides_done += 1
            self.get_logger().info(f'回転完了 (angle={math.degrees(angle):.1f}°) [{self.sides_done}/{self.total_sides}]')
            self._publish(0.0, 0.0)

            if self.sides_done >= self.total_sides:
                self.get_logger().info('完走しました！')
                self.destroy_timer(self.timer)
                self.state = State.DONE
            else:
                self._set_ref()
                self.state = State.DRIVING

    # ------------------------------------------------------------------
    # ユーティリティ
    # ------------------------------------------------------------------
    def _set_ref(self):
        """現在のodom値を基準点として記録"""
        self.ref_x   = self.x
        self.ref_y   = self.y
        self.ref_yaw = self.yaw

    def _distance(self):
        """基準点からの移動距離"""
        return math.hypot(self.x - self.ref_x, self.y - self.ref_y)
        # math.hypot(dx, dy) は √(dx²+dy²) と同じ.基準点からの直線距離を返します。

    def _delta_yaw(self):
        """基準点からの回転量（0〜π に正規化）"""
        diff = self.yaw - self.ref_yaw
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return abs(diff)

    def _publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    @staticmethod # @staticmethodはselfを使わないメソッドに付けるもので、クラスに関連する関数をまとめる目的です。
    def _quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)


def main(args=None):
    rclpy.init(args=args)
    node = SquareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()