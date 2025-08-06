#include "lidar_simulator.hpp"

LidarSimulator::LidarSimulator() 
    : Node("lidar_simulator")
{
    // 创建激光雷达数据发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    
    // 创建定时器，20Hz发布数据
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LidarSimulator::timer_callback, this)
    );
    
    // 声明参数
    this->declare_parameter("frame_id", "laser_link");
    this->declare_parameter("angle_min", -3.14159);
    this->declare_parameter("angle_max", 3.14159);
    this->declare_parameter("angle_increment", 0.0174533);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 10.0);
}

void LidarSimulator::timer_callback()
{
    auto scan = sensor_msgs::msg::LaserScan();
    
    // 获取参数
    // scan.header.frame_id = this->get_parameter("frame_id").as_string();
    scan.header.frame_id = "laser_link"; 
    scan.angle_min = this->get_parameter("angle_min").as_double();
    scan.angle_max = this->get_parameter("angle_max").as_double();
    scan.angle_increment = this->get_parameter("angle_increment").as_double();
    scan.range_min = this->get_parameter("range_min").as_double();
    scan.range_max = this->get_parameter("range_max").as_double();
    
    scan.header.stamp = this->now();
    
    // 计算扫描点数量
    int num_points = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_points, scan.range_max);  // 默认设置为最大距离
    
    //添加模拟障碍物（正前方1米处）
    // int center_index = num_points / 2;
    // scan.ranges[center_index] = 1.0;
    // int center = num_points / 2;
    // for (int i = center - 4; i <= center + 4; ++i) {
    //     if (i >= 0 && i < num_points) {
    //         scan.ranges[i] = 4.0;  // 正前方 5 米有障碍物
    //     }
    // }
    
    // 发布扫描数据
    publisher_->publish(scan);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulator>());
    rclcpp::shutdown();
    return 0;
}


// ########################################################################################

// #include "lidar_simulator.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <cmath>

// LidarSimulator::LidarSimulator() 
//     : Node("lidar_simulator")
// {
//     // 创建激光雷达数据发布者
//     publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    
//     // 创建TF缓冲区和监听器（用于获取机器人在地图中的位置）
//     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
//     // 创建定时器，20Hz发布数据
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(50),
//         std::bind(&LidarSimulator::timer_callback, this)
//     );
    
//     // 声明参数（修正frame_id为激光雷达坐标系）
//     this->declare_parameter("frame_id", "laser");  // 关键：使用激光雷达自身坐标系
//     this->declare_parameter("angle_min", -M_PI);   // -π到π（全向扫描）
//     this->declare_parameter("angle_max", M_PI);
//     this->declare_parameter("angle_increment", 0.0174533);  // 约1度/点
//     this->declare_parameter("range_min", 0.1);
//     this->declare_parameter("range_max", 10.0);
    
//     // 定义环境中固定的障碍物（地图坐标系）
//     // 示例：在地图(2, 0)和(0, 2)处各放一个障碍物
//     obstacles_ = {
//         {2.0, 0.0},   // 障碍物1：x=2m, y=0m（机器人前方2米）
//         {0.0, 2.0}    // 障碍物2：x=0m, y=2m（机器人右侧2米）
//     };
// }

// void LidarSimulator::timer_callback()
// {
//     auto scan = sensor_msgs::msg::LaserScan();
    
//     // 获取参数
//     scan.header.frame_id = this->get_parameter("frame_id").as_string();  // 使用激光坐标系
//     scan.angle_min = this->get_parameter("angle_min").as_double();
//     scan.angle_max = this->get_parameter("angle_max").as_double();
//     scan.angle_increment = this->get_parameter("angle_increment").as_double();
//     scan.range_min = this->get_parameter("range_min").as_double();
//     scan.range_max = this->get_parameter("range_max").as_double();
//     scan.header.stamp = this->now();
    
//     // 计算扫描点数量
//     int num_points = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
//     scan.ranges.resize(num_points, scan.range_max);  // 默认无障碍物（最大距离）
    
//     // 获取激光雷达在地图坐标系中的位置（通过TF变换）
//     try {
//         // 查找激光雷达(laser)相对于地图(map)的变换
//         auto transform = tf_buffer_->lookupTransform(
//             "map",          // 目标坐标系（地图）
//             scan.header.frame_id,  // 源坐标系（激光雷达）
//             tf2::TimePointZero    // 获取最新变换
//         );
        
//         // 激光雷达在地图中的坐标 (x_laser, y_laser)
//         double x_laser = transform.transform.translation.x;
//         double y_laser = transform.transform.translation.y;
//         // 激光雷达在地图中的朝向（弧度）
//         double yaw_laser = tf2::getYaw(transform.transform.rotation);
        
//         // 遍历所有障碍物，计算每个障碍物相对于激光雷达的距离和角度
//         for (const auto& obs : obstacles_) {
//             // 障碍物在地图中的坐标 (x_obs, y_obs)
//             double x_obs = obs.x;
//             double y_obs = obs.y;
            
//             // 计算障碍物相对于激光雷达的坐标（x_rel, y_rel）
//             double dx = x_obs - x_laser;
//             double dy = y_obs - y_laser;
//             double distance = std::hypot(dx, dy);  // 直线距离
            
//             // 若障碍物在激光量程外，跳过
//             if (distance < scan.range_min || distance > scan.range_max) {
//                 continue;
//             }
            
//             // 计算障碍物相对于激光雷达的角度（弧度）
//             double angle_abs = std::atan2(dy, dx);  // 绝对角度（相对于地图）
//             double angle_rel = angle_abs - yaw_laser;  // 相对激光雷达的角度（激光坐标系）
            
//             // 角度归一化到[-π, π]
//             angle_rel = std::atan2(std::sin(angle_rel), std::cos(angle_rel));
            
//             // 计算该角度对应的激光扫描点索引
//             int index = static_cast<int>((angle_rel - scan.angle_min) / scan.angle_increment);
            
//             // 确保索引在有效范围内
//             if (index >= 0 && index < num_points) {
//                 // 若该角度的当前距离大于障碍物距离，则更新为障碍物距离
//                 // （模拟“最近障碍物优先”）
//                 if (distance < scan.ranges[index]) {
//                     scan.ranges[index] = distance;
//                     // 为障碍物添加一定的角度宽度（模拟障碍物有体积）
//                     int width = 3;  // 障碍物占据3个激光点（约3度）
//                     for (int i = 1; i <= width; ++i) {
//                         if (index + i < num_points) {
//                             scan.ranges[index + i] = std::min(scan.ranges[index + i], static_cast<float>(distance));
//                         }
//                         if (index - i >= 0) {
//                             scan.ranges[index - i] = std::min(scan.ranges[index - i], static_cast<float>(distance));
//                         }
//                     }
//                 }
//             }
//         }
//     } catch (const tf2::TransformException& ex) {
//         RCLCPP_WARN(this->get_logger(), "TF变换失败: %s", ex.what());
//         return;
//     }
    
//     // 发布扫描数据
//     publisher_->publish(scan);
// }

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<LidarSimulator>());
//     rclcpp::shutdown();
//     return 0;
// }
