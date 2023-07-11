#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

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
            sensor_msgs::msg::LaserScan laserscan_message = *msg;

            laserscan_message.header.stamp = rclcpp::Clock().now();
            laserscan_message.time_increment = 0.0;
            laserscan_message.scan_time = 0.0;

            unfilteredpub_->publish(laserscan_message);
        };

        unfilteredpub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_unfiltered", best_effort.reliability(be), laser_callback);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr unfilteredpub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    rclcpp::QoS best_effort;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<laser_filter>());
    rclcpp::shutdown();
    return 0;
}