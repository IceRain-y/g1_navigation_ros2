#ifndef LIDAR_SIMULATOR_HPP_
#define LIDAR_SIMULATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarSimulator : public rclcpp::Node
{
public:
    LidarSimulator();

private:
    void timer_callback();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

#endif  // LIDAR_SIMULATOR_HPP_