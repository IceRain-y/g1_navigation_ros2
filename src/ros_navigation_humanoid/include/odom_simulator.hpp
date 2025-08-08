// #ifndef ODOM_SIMULATOR_HPP_
// #define ODOM_SIMULATOR_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "tf2_ros/transform_broadcaster.h"

// class OdomSimulator : public rclcpp::Node
// {
// public:
//     OdomSimulator();

// private:
//     void timer_callback();
    
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
//     // 里程计状态
//     double x_;
//     double y_;
//     double theta_;
// };

// #endif  // ODOM_SIMULATOR_HPP_


#ifndef SIMPLE_ODOM_NODE_HPP_
#define SIMPLE_ODOM_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

class SimpleOdomNode : public rclcpp::Node
{
public:
  SimpleOdomNode();
  ~SimpleOdomNode() = default;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void timerCallback();

  void publishOdom();
  void publishTf();

  // parameters
  std::string base_frame_;
  std::string odom_frame_;
  bool use_twist_stamped_;

  // state
  double x_, y_, yaw_;
  geometry_msgs::msg::Twist current_twist_;

  // ros infra
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex state_mutex_;
};

#endif  // SIMPLE_ODOM_NODE_HPP_