#include <chrono>
#include <memory>
#include <iostream>
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
        // get parameters
        this->declare_parameter("max_range", rclcpp::PARAMETER_DOUBLE);
        this->get_parameter_or("max_range", max_range, 2.0);

        RCLCPP_INFO(this->get_logger(), "The max range is: %f", max_range);

        auto laser_callback = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            sensor_msgs::msg::LaserScan unfiltered_laserscan_message = *msg;
            sensor_msgs::msg::LaserScan filtered_laserscan_message = *msg;

            // unfiltered message properties
            unfiltered_laserscan_message.header.stamp = rclcpp::Clock().now();
            unfiltered_laserscan_message.time_increment = 0.0;
            unfiltered_laserscan_message.scan_time = 0.0;

            // filtered scan properties

            filtered_laserscan_message.header.stamp = rclcpp::Clock().now();
            filtered_laserscan_message.time_increment = 0.0;
            filtered_laserscan_message.scan_time = 0.0;
            std::vector<bool> valid_ranges(filtered_laserscan_message.ranges.size(), false);
            for (size_t idx = 0; idx < filtered_laserscan_message.ranges.size() - filter_window + 1; ++idx)
            {
                bool window_valid = checkWindowValid(
                    filtered_laserscan_message, idx, filter_window, max_range_difference);

                // Actually set the valid ranges (do not set to false if it was already valid or out of range)
                for (size_t neighbor_idx_or_self_nr = 0; neighbor_idx_or_self_nr < filter_window; ++neighbor_idx_or_self_nr)
                {
                    size_t neighbor_idx_or_self = idx + neighbor_idx_or_self_nr;
                    if (neighbor_idx_or_self < filtered_laserscan_message.ranges.size()) // Out of bound check
                    {
                        bool out_of_range = filtered_laserscan_message.ranges[neighbor_idx_or_self] > max_range;
                        valid_ranges[neighbor_idx_or_self] = valid_ranges[neighbor_idx_or_self] || window_valid || out_of_range;
                    }
                }
            }

            // set invalid points to nan

            for (size_t idx = 0; idx < valid_ranges.size(); ++idx)
            {
                if (!valid_ranges[idx])
                {
                    filtered_laserscan_message.ranges[idx] = std::numeric_limits<float>::quiet_NaN();
                }
            }

            unfilteredpub_->publish(unfiltered_laserscan_message);
            filteredpub_->publish(filtered_laserscan_message);
        };

        // TODO: change the topic names
        unfilteredpub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        filteredpub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_unfiltered", best_effort.reliability(be), laser_callback);
    }

    bool
    checkWindowValid(const sensor_msgs::msg::LaserScan &scan, size_t idx, size_t window, double max_range_difference)
    {
        const float &range = scan.ranges[idx];
        if (range != range)
        {
            return false;
        }

        for (size_t neighbor_idx_nr = 1; neighbor_idx_nr < window; ++neighbor_idx_nr)
        {
            size_t neighbor_idx = idx + neighbor_idx_nr;
            if (neighbor_idx < scan.ranges.size()) // Out of bound check
            {
                const float &neighbor_range = scan.ranges[neighbor_idx];
                if (neighbor_range != neighbor_range || fabs(neighbor_range - range) > max_range_difference)
                {
                    return false;
                }
            }
        }
        return true;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr unfilteredpub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filteredpub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    // TODO: parameterize these values

    // only datapoints with distances smaller than this range are taken into account (m)
    double max_range = 0.0;
    // maximum distance between two consecutive points (m)
    double max_range_difference = 0.04;
    // maximum number of neighbour points to consider
    unsigned int filter_window = 2;

    rclcpp::QoS best_effort;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<laser_filter>());
    rclcpp::shutdown();
    return 0;
}