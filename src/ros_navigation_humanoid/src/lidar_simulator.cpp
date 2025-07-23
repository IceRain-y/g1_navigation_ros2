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
    this->declare_parameter("frame_id", "base_link");
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
    scan.header.frame_id = this->get_parameter("frame_id").as_string();
    scan.angle_min = this->get_parameter("angle_min").as_double();
    scan.angle_max = this->get_parameter("angle_max").as_double();
    scan.angle_increment = this->get_parameter("angle_increment").as_double();
    scan.range_min = this->get_parameter("range_min").as_double();
    scan.range_max = this->get_parameter("range_max").as_double();
    
    scan.header.stamp = this->now();
    scan.header.frame_id = "base_link"; 
    
    // 计算扫描点数量
    int num_points = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_points, scan.range_max);  // 默认设置为最大距离
    
    // 添加模拟障碍物（正前方1米处）
    // int center_index = num_points / 2;
    // scan.ranges[center_index] = 1.0;
    int center = num_points / 2;
    for (int i = center - 5; i <= center + 5; ++i) {
        if (i >= 0 && i < num_points) {
            scan.ranges[i] = 5.0;  // 正前方 5 米有障碍物
        }
    }
    
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