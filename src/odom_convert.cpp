#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "labview_r2interface/msg/lvodom.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
#define PI 3.141592653589793238
using namespace std::chrono_literals;

class odom_convert : public rclcpp::Node
{
public:
  odom_convert()
      : Node("odom_convert"),
        best_effort(rclcpp::KeepLast(10))
  {
    tbr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "odom_converter started!\n");
    // rotations = vector<double>(4, 0);
    // translations = vector<double>(4, 0);

    // define empty initials
    odom_message = nav_msgs::msg::Odometry();

    auto odom_callback = [this](const labview_r2interface::msg::Lvodom::SharedPtr msg)
    {
      // std::cout<<msg->bot_x<<" "<<msg->bot_y<<" "<<msg->bot_theta<<" "<<msg->bot_linear<<" "<<msg->bot_angular<<"\n";
      odom_message = nav_msgs::msg::Odometry();
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
      odom_message.child_frame_id = "base_footprint";
      // get current time and fill up the header
      rclcpp::Time time_now = rclcpp::Clock().now();
      // odom_message.header.stamp = time_now;
      odom_message.header.stamp = this->now();
      odom_message.header.frame_id = "odom";

      // also send transform data at the same rate
      // traslations[0] = msg->bot_x / 1000.0;
      // translations[1] = msg->bot_y / 1000.0;
      // translations[2] = 0.0;

      // rotations[0] = q.x();
      // rotations[1] = q.y();
      // rotations[2] = q.z();
      // rotations[3] = q.w();

      transformStamped.transform.translation.x = msg->bot_x / 1000.0;
      transformStamped.transform.translation.y = msg->bot_y / 1000.0;
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
    };

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    subscription_ = this->create_subscription<labview_r2interface::msg::Lvodom>("bot_odom", best_effort.reliability(be), odom_callback);

    timer_ = this->create_wall_timer(
        1ms, [this]()
        {

      // rclcpp::Time current_time = rclcpp::Clock().now();
      rclcpp::Time current_time = this->get_clock()->now();

      transformStamped.header.stamp = current_time;
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = "base_footprint";

          publisher_->publish(odom_message);
          tbr->sendTransform(transformStamped); });
  }

private:
  nav_msgs::msg::Odometry odom_message;
  geometry_msgs::msg::TransformStamped transformStamped;
  // vector<double> rotations;
  // vector<double> translations;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<labview_r2interface::msg::Lvodom>::SharedPtr subscription_;
  rclcpp::QoS best_effort;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tbr;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_convert>());
  rclcpp::shutdown();
  return 0;
}
