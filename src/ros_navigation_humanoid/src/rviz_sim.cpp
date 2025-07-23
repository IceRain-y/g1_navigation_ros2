#include "rviz_sim.hpp"
#include <cmath>

RvizSimNode::RvizSimNode() : Node("rviz_sim_node"), th_(0.0), x_(0.0), y_(0.0), z_(0.0)
{
    // 声明并获取参数
    this->declare_parameter<float>("pelvis_to_foot_heigth", 0.8);
    this->get_parameter("pelvis_to_foot_heigth", pelvis_to_foot_heigth_);
    
    // 初始化时间
    last_time_ = this->now();
    
    // 创建订阅者
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, std::bind(&RvizSimNode::DealCmdVel, this, std::placeholders::_1));
    
    // 初始化TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 创建计时器，定期发布TF变换
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz
        std::bind(&RvizSimNode::publishTransform, this));
    
    RCLCPP_INFO(this->get_logger(), "Rviz simulation node initialized");
}

RvizSimNode::~RvizSimNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down rviz simulation node");
}

void RvizSimNode::DealCmdVel(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    
    if (dt > 1.0) {
        RCLCPP_WARN(this->get_logger(), "[Debug: skip] dt = %f", dt);
        return;
    }

    // 读取速度
    double v_x = cmd_vel->linear.x;
    double v_y = cmd_vel->linear.y;  // 适用于全向移动
    double omega = cmd_vel->angular.z;

    // 计算新的位姿 (基于里程计积分)
    x_ += (v_x * cos(th_) - v_y * sin(th_)) * dt;
    y_ += (v_x * sin(th_) + v_y * cos(th_)) * dt;
    th_ += omega * dt;

    // 归一化角度到 [-π, π]
    th_ = atan2(sin(th_), cos(th_));
}

void RvizSimNode::publishTransform()
{
    // 创建TF变换消息
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "pelvis";
    
    // 设置平移
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = pelvis_to_foot_heigth_;
    
    // 设置旋转
    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    // 发布TF变换
    tf_broadcaster_->sendTransform(transform);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RvizSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
