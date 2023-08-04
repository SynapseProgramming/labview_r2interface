#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        ismoving = false;
        fired = false;
        auto cmdvel_callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            if (msg->linear.x == 0 && msg->angular.z == 0)
                ismoving = false;
            else
                ismoving = true;
        };

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", best_effort.reliability(be), cmdvel_callback);

        timer_ = this->create_wall_timer(
            100ms, [this]()
            {
                std::cout << ismoving << "\n";
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
                            toFrameRel, fromFrameRel,
                            tf2::TimePointZero);
                        } catch (const tf2::TransformException & ex) {
                        RCLCPP_INFO(
                            this->get_logger(), "Could not transform %s to %s: %s",
                            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                        return;
                        }
                    std::cout<<"POSE SAVED\n";
                    fired = true;
                } });

        one_off_timer = this->create_wall_timer(1s, [this]()
                                                {
                    printf("in one_off_timer callback\n");

                    std::cout << transformStamped.transform.translation.x << "\n";
                    std::cout << transformStamped.transform.translation.y << "\n";

                    geometry_msgs::msg::PoseWithCovarianceStamped amci;
                    // fill up header here

                    amci.pose.pose.position.x = transformStamped.transform.translation.x;
                    amci.pose.pose.position.y = transformStamped.transform.translation.y;
                    amci.pose.pose.orientation = transformStamped.transform.rotation;

                    this->one_off_timer->reset(); });
        // cancel to prevent running at the start
        one_off_timer->cancel();
    };

private:
    geometry_msgs::msg::TransformStamped transformStamped;
    bool ismoving, fired;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    std::string toFrameRel = "base_footprint";
    std::string fromFrameRel = "map";

    rclcpp::QoS best_effort;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr one_off_timer;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_reset>());
    rclcpp::shutdown();
    return 0;
}
