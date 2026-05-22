#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp" // オドメトリメッセージ
#include <cmath>

// クラス名をファイル名に合わせて OdmPyNode に
class OdmPyNode : public rclcpp::Node
{
public:
    OdmPyNode() : Node("odm_py_node")
    {
        last_time_ = this->now();

        // IMUのサブスクライバ
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu_data", 10,
            std::bind(&OdmPyNode::imu_callback, this, std::placeholders::_1));

        // Pythonノードに向けて、計算したオドメトリを配信するパブリッシャ
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        RCLCPP_INFO(this->get_logger(), "Python連携用 IMUオドメトリ配信ノードが起動しました。");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        
        if (!initialized_) {
            last_time_ = current_time;
            initialized_ = true;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double angular_velocity_z = msg->angular_velocity.z;
        yaw_ += angular_velocity_z * dt;

        while (yaw_ > M_PI)  yaw_ -= 2.0 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

        // 計算したYaw角を、ROS2の標準オドメトリ型に変換してパブリッシュ
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // 位置(X, Y)はひとまず0固定（Python側で時間制御するため）
        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;

        // オイラー角(Yaw)からクォータニオン(Z, W)への変換
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
        odom_msg.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Time last_time_;
    double yaw_ = 0.0;
    bool initialized_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdmPyNode>());
    rclcpp::shutdown();
    return 0;
}