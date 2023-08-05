#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.h>

#include "nav_msgs/msg/odometry.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
using namespace std::chrono_literals;

class pose_reset : public rclcpp::Node
{
public:
    pose_reset()
        : Node("pose_reset"),
          best_effort(rclcpp::KeepLast(10))
    {
        RCLCPP_INFO(this->get_logger(), "pose_reset started!\n");

        ismoving = false;
        fired = false;

        // define covariance matrix
        // x variance
        cov[0] = 1e-9;
        // y variance
        cov[7] = 1e-9;
        // z variance
        cov[14] = 1e-9;
        // rx variance
        cov[21] = 1e-9;
        // ry variance
        cov[28] = 1e-9;
        // rz variance
        cov[35] = 1e-9;

        auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            double linearv = msg->twist.twist.linear.x;
            double angularv = msg->twist.twist.angular.z;
            if (abs(angularv) <= 0.01 && abs(linearv) < 0.01)
                ismoving = false;
            else
                ismoving = true;

            std::cout << ismoving << "\n";
        };

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/set_pose", 10);
        odomsub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", best_effort.reliability(be), odom_callback);

        timer_ = this->create_wall_timer(
            10ms, [this]()
            {
                if (ismoving)
                {
                    fired = false;
                    this->one_off_timer->cancel();
                }
                else if (fired == false)
                {
                    this->one_off_timer->reset();
                    fired = true;
                } });

        one_off_timer = this->create_wall_timer(600s, [this]()
                                                {
                                                    // geometry_msgs::msg::PoseWithCovarianceStamped amci;
                                                    // amci.header.frame_id= "odom";
                                                    // amci.header.stamp = this->get_clock()->now();
                                                    // amci.pose = latchedOdom.pose;

                                                    // publisher_->publish(amci);

                                                    this->one_off_timer->reset(); });
        // cancel to prevent running at the start
        one_off_timer->cancel();
    };

private:
    bool ismoving, fired;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub_;

    rclcpp::QoS best_effort;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr one_off_timer;

    nav_msgs::msg::Odometry latchedOdom;

    std::array<double, 36> cov;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_reset>());
    rclcpp::shutdown();
    return 0;
}
