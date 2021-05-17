#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "labview_r2interface/msg/laserarray.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <iostream>

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class laser_convert : public rclcpp::Node
{
public:

  laser_convert()
  : Node("laserscan_convert"),
    best_effort(rclcpp::KeepLast(10))
  {

           RCLCPP_INFO(this->get_logger(), "laserscan_converter started!\n");
    auto laser_callback= [this](const labview_r2interface::msg::Laserarray::SharedPtr msg){
            auto laserscan_message=sensor_msgs::msg::LaserScan();
          // RCLCPP_INFO(this->get_logger(), "message_received\n");

           //acquire distance points
           std::vector<float> distance_points;
           auto raw_points=msg->points;
           for(long unsigned int i=0;i<raw_points.max_size();i++){
            //convert from mm to m
            float point=raw_points[i]/1000.0;
            //limit between laser limits
            point=std::max(point,laser_min);
            point=std::min(point,laser_max);
            distance_points.push_back(point);
           }
           //fill up the other parameters
           laserscan_message.angle_min=-2.347467844;
           laserscan_message.angle_max=2.364921136;
           laserscan_message.angle_increment=0.00872664626;
           laserscan_message.time_increment=0.0001851851852;
           laserscan_message.scan_time=0.1;
           laserscan_message.range_min=laser_min;
           laserscan_message.range_max=laser_max;
           // get current time and fill up the header
           rclcpp::Time time_now = rclcpp::Clock().now();
           laserscan_message.header.stamp=time_now;
           laserscan_message.header.frame_id="base_laser";
           laserscan_message.ranges=distance_points;

           distance_points.clear();

           publisher_->publish(laserscan_message);
    };

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    subscription_ = this->create_subscription<labview_r2interface::msg::Laserarray>("raw_laser", best_effort.reliability(be),laser_callback);

  }

private:

  //shared ptr of a timer
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<labview_r2interface::msg::Laserarray>::SharedPtr subscription_;
  rclcpp::QoS best_effort;
   const float laser_min=0.022;
   const float laser_max=20;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_convert>());
  rclcpp::shutdown();
  return 0;
}
