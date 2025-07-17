#include "point_cloud_display.hpp"

PointCloudDisplayNode::PointCloudDisplayNode() : Node("pointcloud_loader")
{
    // 声明并获取参数
    this->declare_parameter<std::string>("publish_topic", "map_point_cloud");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("file_path", "");
    this->declare_parameter<float>("z_min", 0.0);
    this->declare_parameter<float>("z_max", 2.0);
    this->declare_parameter<float>("voxel_filte_size", 0.05);
    this->declare_parameter<float>("pub_rate", 0.5);
    
    this->get_parameter("publish_topic", publish_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("file_path", file_path_);
    this->get_parameter("z_min", z_min_);
    this->get_parameter("z_max", z_max_);
    this->get_parameter("voxel_filte_size", voxel_filte_size_);
    this->get_parameter("pub_rate", pub_rate_);
    
    // 初始化点云指针
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    filtered_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filtered_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    
    // 创建发布者
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(publish_topic_, 1);
    
    // 加载和处理点云
    if (loadPointCloud()) {
        filterPointCloud();
        convertToROSMessage();
        
        // 创建计时器，定期发布点云
        auto period = std::chrono::duration<double>(1.0 / pub_rate_);
        timer_ = this->create_wall_timer(period, std::bind(&PointCloudDisplayNode::publishCallback, this));
    }
}

PointCloudDisplayNode::~PointCloudDisplayNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down point cloud display node");
}

bool PointCloudDisplayNode::loadPointCloud()
{
    if (file_path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "File path is not specified, do not display.");
        return false;
    }
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path_, *cloud_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read point cloud file [%s]", file_path_.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %zu points", cloud_->points.size());
    return true;
}

void PointCloudDisplayNode::filterPointCloud()
{
    // Z轴范围过滤
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*filtered_cloud_);
    
    RCLCPP_INFO(this->get_logger(), "After Z filter: %zu points", filtered_cloud_->points.size());
    
    // 体素滤波降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered_cloud_);
    voxel.setLeafSize(voxel_filte_size_, voxel_filte_size_, voxel_filte_size_);
    voxel.filter(*voxel_filtered_cloud_);
    
    RCLCPP_INFO(this->get_logger(), "After VoxelGrid filter: %zu points", voxel_filtered_cloud_->points.size());
}

void PointCloudDisplayNode::convertToROSMessage()
{
    // 转换为ROS消息
    pcl::toROSMsg(*voxel_filtered_cloud_, cloud_msg_);
    cloud_msg_.header.frame_id = frame_id_;
    
    RCLCPP_INFO(this->get_logger(), "Point cloud converted to ROS message");
}

void PointCloudDisplayNode::publishCallback()
{
    cloud_msg_.header.stamp = this->now();
    pub_->publish(cloud_msg_);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
