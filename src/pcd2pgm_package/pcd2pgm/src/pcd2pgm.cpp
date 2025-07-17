#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/conditional_removal.h>         // 条件滤波器头文件
#include <pcl/filters/passthrough.h>                 // 直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      // 半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> // 统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  // 体素滤波器头文件
#include <pcl/point_types.h>

// 全局参数变量
std::string file_directory;
std::string file_name;
std::string pcd_file;
std::string map_topic_name;
const std::string pcd_format = ".pcd";

// 地图消息
nav_msgs::msg::OccupancyGrid map_topic_msg;

// 滤波参数
double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;
double map_resolution = 0.05;
double thre_radius = 0.1;
int thres_point_count = 10;

// 点云指针
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

// 函数声明
void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in);
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count);
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg, rclcpp::Node::SharedPtr node);

int main(int argc, char **argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pcl_filters");

    // 声明并获取参数
    node->declare_parameter("file_directory", std::string("/home/"));
    node->declare_parameter("file_name", std::string("map"));
    node->declare_parameter("map_topic_name", std::string("map"));
    node->declare_parameter("thre_z_min", 0.3);
    node->declare_parameter("thre_z_max", 2.0);
    node->declare_parameter("flag_pass_through", 0);
    node->declare_parameter("map_resolution", 0.05);
    node->declare_parameter("thre_radius", 0.1);
    node->declare_parameter("thres_point_count", 10);

    node->get_parameter("file_directory", file_directory);
    node->get_parameter("file_name", file_name);
    node->get_parameter("map_topic_name", map_topic_name);
    node->get_parameter("thre_z_min", thre_z_min);
    node->get_parameter("thre_z_max", thre_z_max);
    node->get_parameter("flag_pass_through", flag_pass_through);
    node->get_parameter("map_resolution", map_resolution);
    node->get_parameter("thre_radius", thre_radius);
    node->get_parameter("thres_point_count", thres_point_count);

    pcd_file = file_directory + file_name + pcd_format;

    // 创建地图发布者
    auto map_topic_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_name, 10);

    // 加载PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
        RCLCPP_ERROR(node->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
        return (-1);
    }
    RCLCPP_INFO(node->get_logger(), "初始点云数据点数：%zu", pcd_cloud->points.size());

    // 执行滤波
    PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));
    RadiusOutlierFilter(cloud_after_PassThrough, thre_radius, thres_point_count);
    SetMapTopicMsg(cloud_after_Radius, map_topic_msg, node);

    // 发布循环
    rclcpp::Rate rate(1.0);  // 1Hz频率
    while (rclcpp::ok()) {
        map_topic_pub->publish(map_topic_msg);
        rate.sleep();
        rclcpp::spin_some(node);  // 处理回调（本节点无回调，可省略但保留规范）
    }

    rclcpp::shutdown();
    return 0;
}

// 直通滤波实现
void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in) {
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(thre_low, thre_high);
    passthrough.setNegative(flag_in);
    passthrough.filter(*cloud_after_PassThrough);

    // 保存滤波结果
    pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_filter.pcd", *cloud_after_PassThrough);
    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
}

// 半径滤波实现
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
    radiusoutlier.setInputCloud(pcd_cloud);
    radiusoutlier.setRadiusSearch(radius);
    radiusoutlier.setMinNeighborsInRadius(thre_count);
    radiusoutlier.filter(*cloud_after_Radius);

    // 保存滤波结果
    pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd", *cloud_after_Radius);
    std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
}

// 转换为栅格地图并设置消息
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::msg::OccupancyGrid &msg, rclcpp::Node::SharedPtr node) {
    msg.header.stamp = node->get_clock()->now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = node->get_clock()->now();
    msg.info.resolution = map_resolution;

    // 计算点云边界
    if (cloud->points.empty()) {
        RCLCPP_WARN(node->get_logger(), "点云为空！");
        return;
    }

    float x_min = cloud->points[0].x, x_max = cloud->points[0].x;
    float y_min = cloud->points[0].y, y_max = cloud->points[0].y;

    for (const auto &point : cloud->points) {
        x_min = std::min(x_min, point.x);
        x_max = std::max(x_max, point.x);
        y_min = std::min(y_min, point.y);
        y_max = std::max(y_max, point.y);
    }

    // 设置地图原点和尺寸
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;  // 单位四元数

    msg.info.width = static_cast<uint32_t>((x_max - x_min) / map_resolution);
    msg.info.height = static_cast<uint32_t>((y_max - y_min) / map_resolution);

    // 初始化地图数据（0表示未知，100表示占据）
    msg.data.resize(msg.info.width * msg.info.height, 0);

    RCLCPP_INFO(node->get_logger(), "地图数据大小：%zu", msg.data.size());

    // 填充地图数据
    for (const auto &point : cloud->points) {
        int i = static_cast<int>((point.x - x_min) / map_resolution);
        int j = static_cast<int>((point.y - y_min) / map_resolution);

        // 检查索引有效性
        if (i >= 0 && i < static_cast<int>(msg.info.width) &&
            j >= 0 && j < static_cast<int>(msg.info.height)) {
            msg.data[i + j * msg.info.width] = 100;  // 标记为占据
        }
    }
}
