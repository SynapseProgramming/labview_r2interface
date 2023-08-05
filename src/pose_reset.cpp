#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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
        cov[0] = 0.25;
        // y variance
        cov[7] = 0.25;
        // z variance
        cov[14] = 0.0;
        // rx variance
        cov[21] = 0.0;
        // ry variance
        cov[28] = 0.0;
        // rz variance
        cov[35] = 0.06853891909122467;

        auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            double linearv = msg->twist.twist.linear.x;
            double angularv = msg->twist.twist.angular.z;
            if (abs(angularv) <= 0.01 && abs(linearv) <= 0.01)
                ismoving = false;
            else
                ismoving = true;
        };

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        odomsub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", best_effort.reliability(be), odom_callback);
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            100ms, [this]()
            {
                if (ismoving)
                {
                    fired = false;
                    this->one_off_timer->cancel();
                }
                else if (fired == false)
                {
                    this->one_off_timer->reset();
                    // get the robots current position in space
                    try {
                        transformStamped = tf_buffer_->lookupTransform(
                            fromFrameRel,toFrameRel,
                            tf2::TimePointZero,100ms);
                        } catch (const tf2::TransformException & ex) {
                        RCLCPP_INFO(
                            this->get_logger(), "Could not transform %s to %s: %s",
                            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                        return;
                        }
                    fired = true;
                } });

        one_off_timer = this->create_wall_timer(1s, [this]()
                                                {
                                                    geometry_msgs::msg::PoseWithCovarianceStamped amci;
                                                    amci.header.frame_id = "map";
                                                    amci.header.stamp = this->get_clock()->now();

                                                    amci.pose.pose.position.x = transformStamped.transform.translation.x;
                                                    amci.pose.pose.position.y = transformStamped.transform.translation.y;
                                                    amci.pose.pose.orientation = transformStamped.transform.rotation;
                                                    amci.pose.covariance = cov;

                                                    publisher_->publish(amci);

                                                    this->one_off_timer->reset(); });
        // cancel to prevent running at the start
        one_off_timer->cancel();
    };

private:
    bool ismoving, fired;
    geometry_msgs::msg::TransformStamped transformStamped;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::string toFrameRel = "base_footprint";
    std::string fromFrameRel = "map";

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
