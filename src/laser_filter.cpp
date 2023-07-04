#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>

using namespace std::chrono_literals;

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class laser_filter : public rclcpp::Node
{
public:
    laser_filter()
        : Node("laser_filter"),
          best_effort(rclcpp::KeepLast(10))
    {

        laserscan_message = sensor_msgs::msg::LaserScan();
        data = std::vector<float>(811, 3.4556);

        RCLCPP_INFO(this->get_logger(), "laser filter started!\n");
        auto laser_callback = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            laserscan_message.header.frame_id = msg->header.frame_id;

            laserscan_message.angle_min = msg->angle_min;
            laserscan_message.angle_max = msg->angle_max;
            // std::cout << "Tolerance" << static_cast<double>(msg->ranges.size() - 1) * static_cast<double>(msg->time_increment) << "\n";
            // std::cout << "Array Size" << static_cast<double>(msg->ranges.size() - 1) << "\n";

            laserscan_message.angle_increment = msg->angle_increment;
            laserscan_message.time_increment = msg->time_increment;
            laserscan_message.scan_time = msg->scan_time;

            laserscan_message.time_increment = 0.0;
            laserscan_message.scan_time = 0.0;

            laserscan_message.range_min = msg->range_min;
            laserscan_message.range_max = msg->range_max;

            laserscan_message.ranges = msg->ranges;
            laserscan_message.intensities = msg->intensities;

            // std::cout << "received time" << msg->header.stamp.nanosec << "\n";
            // std::cout << "current time" << laserscan_message.header.stamp.nanosec << "\n";

            // rclcpp::Time now = this->now();
            // try
            // {
            //     tf_buffer_->lookupTransform(
            //         toFrame, fromFrame, tf2::TimePointZero);
            // }
            // catch (const tf2::TransformException &ex)
            // {
            //     RCLCPP_INFO(
            //         this->get_logger(), "Could not transform %s to %s: %s",
            //         toFrame.c_str(), fromFrame.c_str(), ex.what());
            //     return;
            // }

            // // laserscan_message.header.stamp = now;
            laserscan_message.header.stamp = rclcpp::Clock().now();

            publisher_->publish(laserscan_message);
        };

        // timer_ = this->create_wall_timer(
        //     50ms, [this]()
        //     {
        //         // rclcpp::Time current_time = rclcpp::Clock().now();
        //         rclcpp::Time current_time = this->get_clock()->now();

        //         laserscan_message.header.frame_id = "base_laser";
        //         laserscan_message.angle_min = -2.356194496154785;
        //         laserscan_message.angle_max = 2.3557233810424805;

        //         laserscan_message.angle_increment = 0.00581718236207962;
        //         // laserscan_message.time_increment = 6.172839493956417e-05;
        //         laserscan_message.time_increment = 0.0;

        //         laserscan_message.ranges = data;
        //         laserscan_message.intensities = data;
        //         //  laserscan_message.scan_time = 0.05;
        //          laserscan_message.scan_time = 0.0;

        //         laserscan_message.range_min = 0.0;
        //         laserscan_message.range_max = 100.0;

        //         laserscan_message.header.stamp = rclcpp::Clock().now();

        //         publisher_->publish(laserscan_message); });

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_unfiltered", best_effort.reliability(be), laser_callback);

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    sensor_msgs::msg::LaserScan laserscan_message;
    std::vector<float> data;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::string toFrame = "odom";
    std::string fromFrame = "base_laser";
    // rclcpp::TimerBase::SharedPtr timer_;
    // const float laser_min = 0.022;
    // const float laser_max = 20;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<laser_filter>());
    rclcpp::shutdown();
    return 0;
}