cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)

# 查找PCL库
find_package(PCL 1.10 REQUIRED)

# 添加可执行文件
add_executable(pcd2pgm src/pcd2pgm.cpp)

# 设置包含目录
target_include_directories(pcd2pgm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# 设置编译选项
target_compile_features(pcd2pgm PUBLIC c_std_99 cxx_std_17)

# 链接库
target_link_libraries(pcd2pgm
  ${PCL_LIBRARIES}
)

# 链接ROS2依赖
ament_target_dependencies(
  pcd2pgm
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "pcl_conversions"
  "pcl_ros"
  "std_msgs"
)

# 安装可执行文件
install(TARGETS pcd2pgm
  DESTINATION lib/${PROJECT_NAME})

# # 安装头文件
# install(DIRECTORY include/
#   DESTINATION include/)

# install(
#   DIRECTORY include/  
#   DESTINATION share/${PROJECT_NAME}
# )

# 如果有launch文件也需要安装
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 导出包信息
# ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(std_msgs)
ament_package()
    