#include "footstep_sim.hpp"
#include <cmath>
#include <tf2/impl/utils.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

FootstepSimNode::FootstepSimNode() : Node("footstep_sim_node"), is_plann_update_(false), thread_running_(true)
{
    // 初始化TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 声明并获取参数
    this->declare_parameter<float>("pelvis_to_foot_heigth", 0.8);
    this->get_parameter("pelvis_to_foot_heigth", pelvis_to_foot_heigth_);
    
    // 初始化订阅者
    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, 
        std::bind(&FootstepSimNode::DealInitializePoseCallback, this, std::placeholders::_1)
    );
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10, 
        std::bind(&FootstepSimNode::DealMoveGoalCallback, this, std::placeholders::_1)
    );
    
    footstep_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/footstep_planner/footsteps_array", 10, 
        std::bind(&FootstepSimNode::DealVisionliationMarker, this, std::placeholders::_1)
    );
    
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, 
        std::bind(&FootstepSimNode::DealJointStatesSub, this, std::placeholders::_1)
    );
    
    // 初始化发布者
    footplanner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);
    initialize_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // 初始化姿态
    current_pose_.header.frame_id = "map";
    current_pose_.pose.position.x = 0.0;
    current_pose_.pose.position.y = 0.0;
    current_pose_.pose.position.z = 0.0;
    current_pose_.pose.orientation.w = 1.0;
    
    left_ankle_pose_.header.frame_id = "map";
    right_ankle_pose_.header.frame_id = "map";
    
    // 启动关节计算线程
    joint_calc_thread_ = std::thread(&FootstepSimNode::JointsMotionCalculateThreadFunc, this);
    
    RCLCPP_INFO(this->get_logger(), "Footstep simulation node initialized.");
}

FootstepSimNode::~FootstepSimNode()
{
    thread_running_ = false;
    if (joint_calc_thread_.joinable()) {
        joint_calc_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Footstep simulation node shutdown.");
}

void FootstepSimNode::DealInitializePoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), 
        "[DealInitializePoseCallback]: Received initialize pose at position=[%.3lf, %.3lf, %.3lf] quat=[%.3lf, %.3lf, %.3lf, %.3lf]",
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w
    );
    
    current_pose_.pose = msg->pose.pose;
    RCLCPP_WARN(this->get_logger(), "[DealInitializePoseCallback]: Robot localization reset done.");
}

void FootstepSimNode::DealMoveGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), 
        "[DealMoveGoalCallback]: Received Goal pose at position=[%.3lf, %.3lf, %.3lf], quat=[%.3lf, %.3lf, %.3lf, %.3lf]",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.x, msg->pose.orientation.y, 
        msg->pose.orientation.z, msg->pose.orientation.w
    );
    
    // 发布起点话题
    auto init_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    init_pose.header.frame_id = "map";
    init_pose.header.stamp = this->now();
    init_pose.pose.pose = current_pose_.pose;
    initialize_pose_pub_->publish(init_pose);
    
    // 短暂休眠
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 发布终点话题
    current_goal_ = *msg;
    footplanner_goal_pub_->publish(*msg);
}

geometry_msgs::msg::Pose FootstepSimNode::CalculatePelvisPose(const geometry_msgs::msg::Pose& left_ankle_pose, 
                                                            const geometry_msgs::msg::Pose& right_ankle_pose)
{
    // Step1. 计算位置，取平均值
    geometry_msgs::msg::Pose pelvis_pose;
    pelvis_pose.position.x = (left_ankle_pose.position.x + right_ankle_pose.position.x) / 2.0;
    pelvis_pose.position.y = (left_ankle_pose.position.y + right_ankle_pose.position.y) / 2.0;
    pelvis_pose.position.z = pelvis_to_foot_heigth_;  // 使用配置的高度
    
    // Step2. 计算yaw姿态，取两个yaw角的中间角度值
    tf2::Quaternion left_quat, right_quat;
    tf2::fromMsg(left_ankle_pose.orientation, left_quat);
    tf2::fromMsg(right_ankle_pose.orientation, right_quat);
    
    double left_yaw = tf2::impl::getYaw(left_quat);
    double right_yaw = tf2::impl::getYaw(right_quat);
    
    double x_dir_m = (std::cos(left_yaw) + std::cos(right_yaw)) / 2.0;
    double y_dir_m = (std::sin(left_yaw) + std::sin(right_yaw)) / 2.0;
    
    tf2::Quaternion pelvis_quat;
    pelvis_quat.setRPY(0, 0, std::atan2(y_dir_m, x_dir_m));
    pelvis_pose.orientation = tf2::toMsg(pelvis_quat);
    
    return pelvis_pose;
}

geometry_msgs::msg::PoseStamped FootstepSimNode::ConvertPose1ToPose2Frame(const geometry_msgs::msg::PoseStamped& pose1, 
                                                                         const geometry_msgs::msg::PoseStamped& pose2)
{
    geometry_msgs::msg::PoseStamped converted_pose;
    converted_pose.header.frame_id = pose2.header.frame_id;
    converted_pose.header.stamp = this->now();
    
    tf2::Transform pose1_trans, pose2_trans;
    tf2::fromMsg(pose1.pose, pose1_trans);
    tf2::fromMsg(pose2.pose, pose2_trans);
    
    tf2::Transform trans_C = pose2_trans.inverse() * pose1_trans;
    // converted_pose.pose = tf2::toMsg(trans_C);

    converted_pose.pose.position.x = trans_C.getOrigin().x();
    converted_pose.pose.position.y = trans_C.getOrigin().y();
    converted_pose.pose.position.z = trans_C.getOrigin().z();
    converted_pose.pose.orientation = tf2::toMsg(trans_C.getRotation());
    
    return converted_pose;
}

void FootstepSimNode::UpdateJointStates(const geometry_msgs::msg::PoseStamped& left_ankle_pose, 
                                       const geometry_msgs::msg::PoseStamped& pelive_pose, 
                                       const geometry_msgs::msg::PoseStamped& right_ankle_pose)
{
    // 计算两只脚在腰关节的相对位置
    geometry_msgs::msg::PoseStamped left_in_pelive_pose = ConvertPose1ToPose2Frame(left_ankle_pose, pelive_pose);
    geometry_msgs::msg::PoseStamped right_in_pelive_pose = ConvertPose1ToPose2Frame(right_ankle_pose, pelive_pose);

    double left_ankle_angle = std::atan2(fabs(left_in_pelive_pose.pose.position.x), pelvis_to_foot_heigth_);
    double right_ankle_angle = std::atan2(fabs(right_in_pelive_pose.pose.position.x), pelvis_to_foot_heigth_);

    if (left_in_pelive_pose.pose.position.x > 0.0) {
        // 左脚在前，右脚在后 -> 左负，右正
        left_ankle_angle = -1.0 * left_ankle_angle;
    } else {
        // 右脚在前，左脚在后 -> 左正，右负
        right_ankle_angle = -1.0 * right_ankle_angle;
    }

    // 更新到joint_state话题中
    int left_hip_index = -1;
    int right_hip_index = -1;
    
    for (size_t i = 0; i < current_joint_states_.name.size(); ++i) {
        if (current_joint_states_.name[i] == "left_hip_pitch_joint") {
            left_hip_index = i;
        }
        if (current_joint_states_.name[i] == "right_hip_pitch_joint") {
            right_hip_index = i;
        }
    }
    
    if (left_hip_index == -1 || right_hip_index == -1) {
        RCLCPP_WARN(this->get_logger(), "Hip joint indices not found!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[Angle]: left_value=%.3lf, right_value=%.3lf", 
               left_ankle_angle, right_ankle_angle);

    sensor_msgs::msg::JointState new_states = current_joint_states_;
    new_states.position[left_hip_index] = left_ankle_angle;
    new_states.position[right_hip_index] = right_ankle_angle;
    new_states.header.stamp = this->now();
    
    joint_states_pub_->publish(new_states);
}

void FootstepSimNode::JointsMotionCalculateThreadFunc()
{
    RCLCPP_WARN(this->get_logger(), "[JointsMotionCalculateThreadFunc]: Enter joints motion calculate thread func.");
    
    while (thread_running_ && rclcpp::ok()) {
        // 如果当前目标点没有更新，则以2Hz的频率待命
        if (!is_plann_update_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        tf2::Quaternion tf_quat;
        tf2::fromMsg(current_goal_.pose.orientation, tf_quat);
        
        RCLCPP_INFO(this->get_logger(), 
            "[JointsMotionCalculateThreadFunc]: Goal updated at [%.3lf, %.3lf, %.3lf]-[%.3lf]", 
            current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z,
            // tf2::impl::getYaw(current_goal_.pose.orientation)
            tf2::impl::getYaw(tf_quat)
        );
        
        is_plann_update_ = false;  // 重置标志位

        while (!is_plann_update_ && thread_running_ && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Simulating robot joints...");
            
            for (size_t cal_index = 0; cal_index < planned_footstep_marker_.markers.size() - 1; ++cal_index) {
                // 这里为了方便只算第一步是左脚的，如果第一步是右脚则直接跳过
                if (cal_index == 0 && planned_footstep_marker_.markers[cal_index].color.g == 1.0) {
                    RCLCPP_WARN(this->get_logger(), "First step is right, skip this frame...");
                    continue;
                }
                
                // 检查是否有新目标
                if (is_plann_update_) {
                    RCLCPP_WARN(this->get_logger(), "Goal has changed.");
                    break;
                }
                
                // 更新左右脚位置
                if (planned_footstep_marker_.markers[cal_index].color.r == 1.0) {  // 左脚
                    left_ankle_pose_.pose = planned_footstep_marker_.markers[cal_index].pose;
                }
                if (planned_footstep_marker_.markers[cal_index].color.g == 1.0) {  // 右脚
                    if (cal_index + 1 < planned_footstep_marker_.markers.size()) {
                        right_ankle_pose_.pose = planned_footstep_marker_.markers[cal_index + 1].pose;
                    }
                }
                
                // 更新腰应该在的位置
                current_pose_.pose = CalculatePelvisPose(left_ankle_pose_.pose, right_ankle_pose_.pose);
                current_pose_.header.stamp = this->now();
                
                // 更新关节信息
                UpdateJointStates(left_ankle_pose_, current_pose_, right_ankle_pose_);
                
                // 发布TF变换
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = this->now();
                transform.header.frame_id = "map";
                transform.child_frame_id = "pelvis";
                transform.transform.translation.x = current_pose_.pose.position.x;
                transform.transform.translation.y = current_pose_.pose.position.y;
                transform.transform.translation.z = pelvis_to_foot_heigth_;
                transform.transform.rotation = current_pose_.pose.orientation;
                
                tf_broadcaster_->sendTransform(transform);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            RCLCPP_INFO(this->get_logger(), "Current goal sim done.");
            break;
        }
        
        RCLCPP_WARN(this->get_logger(), "[JointsMotionCalculateThreadFunc]: Detected goal changed, break this sim circle.");
    }
    
    RCLCPP_WARN(this->get_logger(), "[JointsMotionCalculateThreadFunc]: Leave joints motion calculate thread func.");
}

void FootstepSimNode::DealJointStatesSub(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_states_ = *msg;
}

void FootstepSimNode::DealVisionliationMarker(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{

    tf2::Quaternion tf_quat_goal;
    tf2::fromMsg(current_goal_.pose.orientation, tf_quat_goal);

    RCLCPP_INFO(this->get_logger(), "================================================================================");
    RCLCPP_WARN(this->get_logger(), 
        "[DealVisionliationMarker]: Received planned footsteps array for [%.3lf, %.3lf, %.3lf]-[%.3lf], size=[%zu]", 
        current_goal_.pose.position.x, current_goal_.pose.position.y, current_goal_.pose.position.z,
        // tf2::impl::getYaw(current_goal_.pose.orientation),
        tf2::impl::getYaw(tf_quat_goal),
        msg->markers.size()
    );
    
    // 打印脚步信息
    size_t step_count = 1;
    for (const auto& marker : msg->markers) {
        tf2::Quaternion tf_quat_maker;
        tf2::fromMsg(marker.pose.orientation, tf_quat_maker);
        if (marker.color.r == 1.0) {  // 左脚
            RCLCPP_INFO(this->get_logger(), 
                "[%zu/%zu]: Left at pos=[%.3lf, %.3lf, %.3lf], yaw=[%.3lf]", 
                step_count, msg->markers.size(),
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, 
                // tf2::impl::getYaw(marker.pose.orientation)
                tf2::impl::getYaw(tf_quat_maker)
            );
        } else if (marker.color.g == 1.0) {  // 右脚
            RCLCPP_INFO(this->get_logger(), 
                "[%zu/%zu]: Right at pos=[%.3lf, %.3lf, %.3lf], yaw=[%.3lf]", 
                step_count, msg->markers.size(),
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z, 
                // tf2::impl::getYaw(marker.pose.orientation)
                tf2::impl::getYaw(tf_quat_maker)
            );
        }
        step_count++;
    }
    
    is_plann_update_ = true;
    planned_footstep_marker_ = *msg;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FootstepSimNode>();
    
    rclcpp::Rate loop_rate(50);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
