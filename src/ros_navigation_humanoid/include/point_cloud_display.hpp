#ifndef POINT_CLOUD_DISPLAY_NODE_HPP_
#define POINT_CLOUD_DISPLAY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudDisplayNode : public rclcpp::Node
{
public:
    PointCloudDisplayNode();
    ~PointCloudDisplayNode();

private:
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    
    // 参数
    std::string frame_id_;
    std::string publish_topic_;
    std::string file_path_;
    float z_min_;
    float z_max_;
    float voxel_filte_size_;
    float pub_rate_;
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud_;
    sensor_msgs::msg::PointCloud2 cloud_msg_;
    
    // 计时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 加载和处理点云
    bool loadPointCloud();
    void filterPointCloud();
    void convertToROSMessage();
    
    // 发布回调函数
    void publishCallback();
};

#endif  // POINT_CLOUD_DISPLAY_NODE_HPP_
