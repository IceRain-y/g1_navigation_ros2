#ifndef FOOTSTEP_SIM_NODE_HPP_
#define FOOTSTEP_SIM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <memory>

class FootstepSimNode : public rclcpp::Node
{
public:
    FootstepSimNode();
    ~FootstepSimNode();

private:
    // 回调函数
    void DealInitializePoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void DealMoveGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void DealJointStatesSub(const sensor_msgs::msg::JointState::SharedPtr msg);
    void DealVisionliationMarker(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    // 辅助函数
    geometry_msgs::msg::Pose CalculatePelvisPose(const geometry_msgs::msg::Pose& left_ankle_pose, 
                                                const geometry_msgs::msg::Pose& right_ankle_pose);
    geometry_msgs::msg::PoseStamped ConvertPose1ToPose2Frame(const geometry_msgs::msg::PoseStamped& pose1, 
                                                           const geometry_msgs::msg::PoseStamped& pose2);
    void UpdateJointStates(const geometry_msgs::msg::PoseStamped& left_ankle_pose, 
                          const geometry_msgs::msg::PoseStamped& pelive_pose, 
                          const geometry_msgs::msg::PoseStamped& right_ankle_pose);
    
    // 关节运动计算线程函数
    void JointsMotionCalculateThreadFunc();
    
    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr footstep_marker_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    
    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr footplanner_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialize_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    // TF相关
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 成员变量
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped current_goal_;
    visualization_msgs::msg::MarkerArray planned_footstep_marker_;
    sensor_msgs::msg::JointState current_joint_states_;
    geometry_msgs::msg::PoseStamped left_ankle_pose_;
    geometry_msgs::msg::PoseStamped right_ankle_pose_;
    
    float pelvis_to_foot_heigth_;
    bool is_plann_update_;
    std::thread joint_calc_thread_;
    std::atomic<bool> thread_running_;
};

#endif  // FOOTSTEP_SIM_NODE_HPP_
