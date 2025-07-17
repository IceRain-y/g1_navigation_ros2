import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取当前包和依赖包的路径
    ros_nav_hum_pkg = get_package_share_directory('ros_navigation_humanoid')
    footstep_planner_pkg = get_package_share_directory('footstep_planner')
    
    return LaunchDescription([
        # 包含地图服务器launch文件（已转换为ROS2格式）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_nav_hum_pkg, 'launch', 'map_server.launch.py')
            )
        ),
        
        # 包含机器人描述launch文件（已转换为ROS2格式）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_nav_hum_pkg, 'launch', 'robot_desc.launch.py')
            )
        ),
        
        # 包含脚步规划器launch文件（已转换为ROS2格式）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(footstep_planner_pkg, 'launch', 'footstep_planner.launch.py')
            )
        ),
        
        # RViz2节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_sim',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([ros_nav_hum_pkg, 'rviz', 'foot_plan.rviz'])
            ]
        ),
        
        # 导航仿真节点
        Node(
            package='ros_navigation_humanoid',
            executable='footstep_sim_node',
            name='footstep_sim_node',
            output='screen',
            parameters=[
                {'pelvis_to_foot_heigth': 0.8}  # 节点参数
            ]
        )
    ])
