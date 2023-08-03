#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
#define PI 3.141592653589793238
using namespace std::chrono_literals;

class pose_reset : public rclcpp::Node
{
public:
    pose_reset()
        : Node("pose_reset"),
          best_effort(rclcpp::KeepLast(10))
    {
        // tbr = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // RCLCPP_INFO(this->get_logger(), "pose_reset started!\n");

        ismoving = false;
        auto cmdvel_callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            if (msg->linear.x == 0 && msg->angular.z == 0)
                ismoving = false;
            else
                ismoving = true;
        };

        // publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", best_effort.reliability(be), cmdvel_callback);

        timer_ = this->create_wall_timer(
            100ms, [this]()
            { std::cout << ismoving << "\n"; });
    };

private:
    // geometry_msgs::msg::TransformStamped transformStamped;
    bool ismoving;

    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    rclcpp::QoS best_effort;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tbr;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_reset>());
    rclcpp::shutdown();
    return 0;
}
