#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "labview_r2interface/msg/lvodom.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
#define PI 3.141592653589793238
using namespace std::chrono_literals;

class odom_convert : public rclcpp::Node
{
public:
  odom_convert()
      : Node("unfiltered_odom_convert"),
        best_effort(rclcpp::KeepLast(10))
  {
    RCLCPP_INFO(this->get_logger(), "unfiltered_odom_converter started!\n");

    // define empty initials
    odom_message = nav_msgs::msg::Odometry();
    odom_message.header.frame_id = "odom";
    odom_message.child_frame_id = "base_footprint";

    // define covariance matrix
    // x variance
    cov[0] = 0.1;
    // y variance
    cov[7] = 1e-9;
    // z variance
    cov[14] = 1e5;
    // rx variance
    cov[21] = 1e5;
    // ry variance
    cov[28] = 1e5;
    // rz variance
    cov[35] = 10.0;

    odom_message.pose.covariance = cov;
    odom_message.twist.covariance = cov;

    auto odom_callback = [this](const labview_r2interface::msg::Lvodom::SharedPtr msg)
    {
      odom_message.pose.pose.position.x = msg->bot_x / 1000.0;
      odom_message.pose.pose.position.y = msg->bot_y / 1000.0;
      odom_message.pose.pose.position.z = 0;

      // convert from theta in radians to a quaternion
      tf2::Quaternion q;
      q.setRPY(0, 0, msg->bot_theta);

      odom_message.pose.pose.orientation.x = q.x();
      odom_message.pose.pose.orientation.y = q.y();
      odom_message.pose.pose.orientation.z = q.z();
      odom_message.pose.pose.orientation.w = q.w();

      odom_message.twist.twist.linear.x = msg->bot_linear / 1000.0;
      odom_message.twist.twist.angular.z = msg->bot_angular * (PI / 180.0);
    };

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_unfiltered", 10);
    subscription_ = this->create_subscription<labview_r2interface::msg::Lvodom>("bot_odom", best_effort.reliability(be), odom_callback);

    timer_ = this->create_wall_timer(
        10ms, [this]()
        {
      rclcpp::Time current_time = this->get_clock()->now();

      odom_message.header.stamp = current_time;
 

      publisher_->publish(odom_message); });
  }

private:
  nav_msgs::msg::Odometry odom_message;

  std::array<double, 36> cov;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<labview_r2interface::msg::Lvodom>::SharedPtr subscription_;
  rclcpp::QoS best_effort;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_convert>());
  rclcpp::shutdown();
  return 0;
}
