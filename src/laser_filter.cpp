#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class laser_filter : public rclcpp::Node
{
public:
    laser_filter()
        : Node("laser_filter"),
          best_effort(rclcpp::KeepLast(10))
    {

        RCLCPP_INFO(this->get_logger(), "laser filter started!\n");
        auto laser_callback = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            auto laserscan_message = sensor_msgs::msg::LaserScan();

            laserscan_message.header.stamp = rclcpp::Clock().now();
            // laserscan_message.header.seq = msg->header.seq;
            laserscan_message.header.frame_id = msg->header.frame_id;

            laserscan_message.angle_min = msg->angle_min;
            laserscan_message.angle_max = msg->angle_max;

            laserscan_message.angle_increment = msg->angle_increment;
            laserscan_message.time_increment = msg->time_increment;
            laserscan_message.scan_time = msg->scan_time;
            
            laserscan_message.range_min = msg->range_min;
            laserscan_message.range_max = msg->range_max;

            laserscan_message.ranges = msg->ranges;
            laserscan_message.intensities = msg->intensities;



            // std::cout << "received time" << msg->header.stamp.nanosec << "\n";
            // std::cout << "current time" << laserscan_message.header.stamp.nanosec << "\n";

            publisher_->publish(laserscan_message);
        };

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_unfiltered", best_effort.reliability(be), laser_callback);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::QoS best_effort;
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