#ifndef ROS_NAVIGATION_HUMANOID__BAG_PLAY_NODE_HPP_
#define ROS_NAVIGATION_HUMANOID__BAG_PLAY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "queue"
#include "mutex"

namespace ros_navigation_humanoid
{

class BagPlayNode : public rclcpp::Node
{
public:
    BagPlayNode();
    ~BagPlayNode() override = default;

    void timer_callback();

private:
    // 回调函数
    void localization_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 订阅者和发布者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr loc_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    
    // 数据存储
    std::queue<geometry_msgs::msg::PoseStamped> loc_queue_;
    std::mutex loc_queue_mutex_;
    geometry_msgs::msg::PoseStamped global_pose_;
    nav_msgs::msg::Path trajectory_path_;
    
    // 参数
    float pelvis_to_foot_heigth_;
    std::string loc_topic_;
    std::string trajectory_topic_;
    
    // 定时器（替代ROS1的spinOnce循环）
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros_navigation_humanoid

#endif  // ROS_NAVIGATION_HUMANOID__BAG_PLAY_NODE_HPP_
