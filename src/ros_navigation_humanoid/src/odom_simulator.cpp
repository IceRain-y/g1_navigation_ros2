// #include "odom_simulator.hpp"
// #include "tf2/LinearMath/Quaternion.h"

// OdomSimulator::OdomSimulator() 
//     : Node("odom_simulator"), x_(0.0), y_(0.0), theta_(0.0)
// {
//     // 创建里程计数据发布者
//     publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/utlidar/robot_odom", 10);
    
//     // 初始化TF广播器
//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
//     // 创建定时器，50Hz发布数据
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(20),  // 50Hz
//         std::bind(&OdomSimulator::timer_callback, this)
//     );
// }

// void OdomSimulator::timer_callback()
// {
//     auto odom_msg = nav_msgs::msg::Odometry();
//     auto transform_msg = geometry_msgs::msg::TransformStamped();
    
//     // 更新位置（简单直线运动模型）
//     x_ += 0.01;  // 每步前进1cm
//     theta_ += 0.001;  // 每步旋转0.001rad
    
//     // 设置header
//     odom_msg.header.stamp = this->now();
//     odom_msg.header.frame_id = "odom";
//     odom_msg.child_frame_id = "base_link";
    
//     // 设置位置
//     odom_msg.pose.pose.position.x = x_;
//     odom_msg.pose.pose.position.y = y_;
    
//     // 设置方向（四元数）
//     tf2::Quaternion q;
//     q.setRPY(0, 0, theta_);
//     odom_msg.pose.pose.orientation.x = q.x();
//     odom_msg.pose.pose.orientation.y = q.y();
//     odom_msg.pose.pose.orientation.z = q.z();
//     odom_msg.pose.pose.orientation.w = q.w();
    
//     // 设置速度（恒定速度）
//     odom_msg.twist.twist.linear.x = 0.05;  // 5cm/s
//     odom_msg.twist.twist.angular.z = 0.05; // 0.05rad/s
    
//     // 发布里程计消息
//     publisher_->publish(odom_msg);
    
//     // 发布TF变换
//     transform_msg.header = odom_msg.header;
//     transform_msg.child_frame_id = "base_link";
//     transform_msg.transform.translation.x = x_;
//     transform_msg.transform.translation.y = y_;
//     transform_msg.transform.rotation = odom_msg.pose.pose.orientation;
//     tf_broadcaster_->sendTransform(transform_msg);
// }

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<OdomSimulator>());
//     rclcpp::shutdown();
//     return 0;
// }

#include "odom_simulator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SimpleOdomNode::SimpleOdomNode()
: Node("simple_odom"),
  base_frame_("base_link"),
  odom_frame_("odom"),
  use_twist_stamped_(false),
  x_(0.0), y_(0.0), yaw_(0.0)
{
  // ---- parameters ----
  this->declare_parameter<std::string>("base_frame", base_frame_);
  this->declare_parameter<std::string>("odom_frame", odom_frame_);
  this->declare_parameter<bool>("use_twist_stamped", use_twist_stamped_);

  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("use_twist_stamped", use_twist_stamped_);

  // ---- publishers ----
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // ---- subscriptions ----
  if (use_twist_stamped_) {
    cmd_vel_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_stamped", 10,
      std::bind(&SimpleOdomNode::cmdVelStampedCallback, this, std::placeholders::_1));
  } else {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SimpleOdomNode::cmdVelCallback, this, std::placeholders::_1));
  }

  // ---- 100 Hz timer ----
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&SimpleOdomNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "SimpleOdomNode initialized.");
}

void SimpleOdomNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_twist_ = *msg;
}

void SimpleOdomNode::cmdVelStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_twist_ = msg->twist;
}

void SimpleOdomNode::timerCallback()
{
  const double dt = 0.01;  // 100 Hz

  std::lock_guard<std::mutex> lock(state_mutex_);
  const double vx = current_twist_.linear.x;
  const double vy = current_twist_.linear.y;
  const double wz = current_twist_.angular.z;

  x_ += (vx * cos(yaw_) - vy * sin(yaw_)) * dt;
  y_ += (vx * sin(yaw_) + vy * cos(yaw_)) * dt;
  yaw_ += wz * dt;

  publishOdom();
  publishTf();
}

void SimpleOdomNode::publishOdom()
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist = current_twist_;
  odom_pub_->publish(odom);
}

void SimpleOdomNode::publishTf()
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;

  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  tf.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleOdomNode>());
  rclcpp::shutdown();
  return 0;
}