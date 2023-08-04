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
        auto cmdvel_callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            if (msg->linear.x == 0 && msg->angular.z == 0)
                ismoving = false;
            else
                ismoving = true;
        };

        auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->initOdom = *msg;
        };

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/set_pose", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", best_effort.reliability(be), cmdvel_callback);
        odomsub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", best_effort.reliability(be), odom_callback);

        timer_ = this->create_wall_timer(
            10ms, [this]()
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
                    //   only call timer when there is odom data
                    if(initOdom.pose.pose.position.x==0) return;
                    latchedOdom = initOdom;
                    fired = true;
                } });

        one_off_timer = this->create_wall_timer(60s, [this]()
                                                {
                    printf("in one_off_timer callback\n");

                    std::cout<<"bot pose: "<<latchedOdom.pose.pose.position.x<<"\n";


                    geometry_msgs::msg::PoseWithCovarianceStamped amci;
                    amci.header.frame_id= "odom";
                    amci.header.stamp = this->get_clock()->now();
                    amci.pose = latchedOdom.pose;

                    // amci.pose.pose.position.x = transformStamped.transform.translation.x;
                    // amci.pose.pose.position.y = transformStamped.transform.translation.y;
                    // amci.pose.pose.orientation = transformStamped.transform.rotation;
                    publisher_->publish(amci);

                    this->one_off_timer->reset(); });
        // cancel to prevent running at the start
        one_off_timer->cancel();
    };

private:
    bool ismoving, fired;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomsub_;

    std::string toFrameRel = "base_footprint";
    std::string fromFrameRel = "odom";

    rclcpp::QoS best_effort;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr one_off_timer;

    nav_msgs::msg::Odometry initOdom;
    nav_msgs::msg::Odometry latchedOdom;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_reset>());
    rclcpp::shutdown();
    return 0;
}
