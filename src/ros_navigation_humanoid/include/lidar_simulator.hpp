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


// ########################################################3

// #ifndef LIDAR_SIMULATOR_HPP
// #define LIDAR_SIMULATOR_HPP

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2/LinearMath/Transform.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "tf2/utils.h"
// #include <vector>

// struct Obstacle {
//     double x;  // 障碍物在地图坐标系中的x坐标
//     double y;  // 障碍物在地图坐标系中的y坐标
// };

// class LidarSimulator : public rclcpp::Node {
// public:
//     LidarSimulator();
//     ~LidarSimulator() = default;

// private:
//     void timer_callback();
//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//     std::vector<Obstacle> obstacles_;  // 环境中的固定障碍物
// };

// #endif  // LIDAR_SIMULATOR_HPP
