#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/laser_scan.hpp"
// #include "/opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/laser_scan.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());


        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // auto message = std_msgs::msg::String();
        // message.data = "Hello, world! " + std::to_string(count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // publisher_->publish(message);

        // LaserScan scan;//


        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

        // scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
        // scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
        // scan_msg->header.frame_id = frame_id;
        // scan_msg->angle_min = scan.config.min_angle;
        // scan_msg->angle_max = scan.config.max_angle;
        // scan_msg->angle_increment = scan.config.angle_increment;
        // scan_msg->scan_time = scan.config.scan_time;
        // scan_msg->time_increment = scan.config.time_increment;
        // scan_msg->range_min = scan.config.min_range;
        // scan_msg->range_max = scan.config.max_range;

        // int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
        // scan_msg->ranges.resize(size);
        // scan_msg->intensities.resize(size);
        // for(size_t i=0; i < scan.points.size(); i++) {
        //     int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        //     if(index >=0 && index < size) {
        //         scan_msg->ranges[index] = scan.points[i].range;
        //         scan_msg->intensities[index] = scan.points[i].intensity;
        //     }
        // }        

        publisher_->publish(*scan_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}