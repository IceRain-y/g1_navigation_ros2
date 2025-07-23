cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(ros_navigation_humanoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# 添加自定义依赖包
find_package(pcd2pgm REQUIRED)

# 查找PCL库
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加头文件目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(rviz_sim src/rviz_sim.cpp)
ament_target_dependencies(rviz_sim
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  pcd2pgm
)

add_executable(bag_play src/bag_play.cpp)
ament_target_dependencies(bag_play
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  pcd2pgm
)

add_executable(footstep_sim src/footstep_sim.cpp)
ament_target_dependencies(footstep_sim
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  pcd2pgm
)

add_executable(point_cloud_display src/point_cloud_display.cpp)
ament_target_dependencies(point_cloud_display
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  pcd2pgm
)
target_link_libraries(point_cloud_display ${PCL_LIBRARIES})

ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  pcd2pgm
  geometry_msgs
)

add_executable(lidar_simulator src/lidar_simulator.cpp)
ament_target_dependencies(lidar_simulator
  rclcpp
  sensor_msgs
)

# 安装可执行文件
install(TARGETS
  rviz_sim
  bag_play
  footstep_sim
  point_cloud_display
  lidar_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )
install(DIRECTORY include/ DESTINATION include/)


# 安装其他资源文件
install(DIRECTORY
  launch
  maps
  move_base_config
  rviz
  robot_descriptions
  resources
  DESTINATION share/${PROJECT_NAME}
)

# 注册包
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
    