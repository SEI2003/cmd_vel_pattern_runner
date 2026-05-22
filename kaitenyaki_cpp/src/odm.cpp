#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>

class OdmNode : public rclcpp::Node
{
public:
    OdmNode() : Node("odm_node")
    {
        // ROS2の時間（タイムスタンプ）を初期化
        last_time_ = this->now();

        // KobukiのIMUトピック（/imu/data）を購読するサブスクライバの設定
        // キューサイズは最新のデータを逃さないよう10に設定
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu_data", 10,
            std::bind(&OdmNode::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "IMUオドメトリノードが起動しました。データ待機中...");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        
        // 初回コールバック時は経過時間の計算をスキップ
        if (!initialized_) {
            last_time_ = current_time;
            initialized_ = true;
            return;
        }

        // ① 前回のデータ受信からの経過時間（秒）を計算 (Δt)
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // ② IMUからZ軸まわりの角速度（ラジアン/秒）を取得
        double angular_velocity_z = msg->angular_velocity.z;

        // ③ 角速度 × 経過時間 を足し合わせて Yaw角 を更新（積分）
        yaw_ += angular_velocity_z * dt;

        // 必要に応じて、角度を -PI 〜 +PI の範囲に正規化
        while (yaw_ > M_PI)  yaw_ -= 2.0 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

        // ラジアンから「度（degree）」に変換して表示しやすくする
        double yaw_deg = yaw_ * 180.0 / M_PI;

        // ターミナルに計算した角度を表示
        RCLCPP_INFO(this->get_logger(), "角速度: %6.3f rad/s | 現在のYaw: %6.1f deg (%6.3f rad)", 
            angular_velocity_z, yaw_deg, yaw_);
    }

    // メンバー変数
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Time last_time_;
    double yaw_ = 0.0;       // 推定された現在のYaw角（ラジアン）
    bool initialized_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdmNode>());
    rclcpp::shutdown();
    return 0;
}