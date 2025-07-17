#include "bag_play.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/rclcpp.hpp"

namespace ros_navigation_humanoid
{

BagPlayNode::BagPlayNode() : Node("bag_play_node")
{
    // 声明并获取参数
    this->declare_parameter<float>("pelvis_to_foot_heigth", 0.8);
    this->declare_parameter<std::string>("loc_topic", "/localization_3d");
    this->declare_parameter<std::string>("trajectory_topic", "/trajectory_path");
    
    this->get_parameter("pelvis_to_foot_heigth", pelvis_to_foot_heigth_);
    this->get_parameter("loc_topic", loc_topic_);
    this->get_parameter("trajectory_topic", trajectory_topic_);
    
    // 初始化TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 初始化订阅者和发布者
    loc_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        loc_topic_, 1, 
        std::bind(&BagPlayNode::localization_callback, this, std::placeholders::_1)
    );
    
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        trajectory_topic_, 1
    );
    
    // 初始化位姿
    global_pose_.pose.position.x = 0.0;
    global_pose_.pose.position.y = 0.0;
    global_pose_.pose.position.z = 0.0;
    global_pose_.pose.orientation.w = 1.0;
    global_pose_.pose.orientation.x = 0.0;
    global_pose_.pose.orientation.y = 0.0;
    global_pose_.pose.orientation.z = 0.0;
    
    // 初始化轨迹
    trajectory_path_.header.frame_id = "map";
    
    // 创建定时器（50Hz循环，替代ROS1的spinOnce）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz = 20ms
        std::bind(&BagPlayNode::timer_callback, this)
    );
}

void BagPlayNode::localization_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // 保存最新位姿并加入队列
    global_pose_ = *msg;
    std::lock_guard<std::mutex> lock(loc_queue_mutex_);
    loc_queue_.push(*msg);
    
    // 限制队列大小
    while (loc_queue_.size() > 500)
    {
        loc_queue_.pop();
    }
}

void BagPlayNode::timer_callback()
{
    // 发布TF变换
    geometry_msgs::msg::TransformStamped map_trans;
    map_trans.header.stamp = this->get_clock()->now();
    map_trans.header.frame_id = "map";
    map_trans.child_frame_id = "pelvis";
    
    map_trans.transform.translation.x = global_pose_.pose.position.x;
    map_trans.transform.translation.y = global_pose_.pose.position.y;
    map_trans.transform.translation.z = pelvis_to_foot_heigth_;  // 从腰关节到脚的高度
    map_trans.transform.rotation = global_pose_.pose.orientation;
    
    tf_broadcaster_->sendTransform(map_trans);
    
    // 发布轨迹
    trajectory_path_.header.stamp = map_trans.header.stamp;
    trajectory_path_.poses.clear();
    
    // 复制队列数据（避免锁阻塞）
    std::queue<geometry_msgs::msg::PoseStamped> loc_que_copyed;
    {
        std::lock_guard<std::mutex> lock(loc_queue_mutex_);
        loc_que_copyed = loc_queue_;
    }
    
    // 填充轨迹数据
    while (!loc_que_copyed.empty())
    {
        trajectory_path_.poses.emplace_back(loc_que_copyed.front());
        loc_que_copyed.pop();
    }
    
    trajectory_pub_->publish(trajectory_path_);
}

}  // namespace ros_navigation_humanoid

// 注册节点
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros_navigation_humanoid::BagPlayNode>());
    rclcpp::shutdown();
    return 0;
}
