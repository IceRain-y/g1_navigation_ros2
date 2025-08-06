// #ifndef RVIZ_SIM_NODE_HPP_
// #define RVIZ_SIM_NODE_HPP_

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <chrono>

// class RvizSimNode : public rclcpp::Node
// {
// public:
//     RvizSimNode();
//     ~RvizSimNode();

// private:
//     // 订阅者
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    
//     // TF广播器
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
//     // 计时器
//     rclcpp::TimerBase::SharedPtr timer_;
    
//     // 位姿变量
//     double th_;
//     double x_;
//     double y_;
//     double z_;
    
//     // 时间变量
//     rclcpp::Time last_time_;
    
//     // 参数
//     float pelvis_to_foot_heigth_;
    
//     // 回调函数
//     void DealCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    
//     // 发布TF变换
//     void publishTransform();
// };

// #endif  // RVIZ_SIM_NODE_HPP_


#ifndef RVIZ_SIM_NODE_HPP_
#define RVIZ_SIM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>  // 新增里程计消息头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <memory>  // 确保包含智能指针支持

class RvizSimNode : public rclcpp::Node
{
public:
    RvizSimNode();
    ~RvizSimNode();

private:
    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    
    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 计时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 新增：里程计发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    // 位姿变量
    double th_;
    double x_;
    double y_;
    double z_;
    
    // 时间变量
    rclcpp::Time last_time_;
    
    // 新增：存储最后接收到的速度命令
    geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
    rclcpp::Time last_cmd_time_;
    
    // 参数
    float pelvis_to_foot_heigth_;
    
    // 回调函数
    void DealCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    
    // 发布TF变换
    void publishTransform();
};

#endif  // RVIZ_SIM_NODE_HPP_
