#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <iostream>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
#define PI 3.141592653589793238
using namespace std::chrono_literals;

class imu_convert : public rclcpp::Node
{
public:
  imu_convert()
      : Node("imu_cov"),
        best_effort(rclcpp::KeepLast(10))
  {
    RCLCPP_INFO(this->get_logger(), "imu_cov started!\n");

    // define covariance matrix
    // x  variance
    cov[0] = 0.1;
    // y variance
    cov[4] = 0.1;
    // z variance
    cov[8] = 0.1;

    auto imu_callback = [this](const sensor_msgs::msg::Imu::SharedPtr msg)
    {

      rclcpp::Time current_time = this->get_clock()->now();
      imu_message = *msg;

      imu_message.header.stamp = current_time;
      imu_message.orientation_covariance = cov;
      imu_message.angular_velocity_covariance = cov;
      imu_message.linear_acceleration_covariance = cov;

      publisher_->publish(imu_message);
    };

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", best_effort.reliability(be), imu_callback);
  }

private:
  sensor_msgs::msg::Imu imu_message;

  std::array<double, 9> cov;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::QoS best_effort;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_convert>());
  rclcpp::shutdown();
  return 0;
}
