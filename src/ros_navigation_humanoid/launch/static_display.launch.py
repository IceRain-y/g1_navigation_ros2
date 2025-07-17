import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')
    
    # 声明启动参数
    pelvis_height_arg = DeclareLaunchArgument(
        'pelvis_to_foot_height',
        default_value='0.8',
        description='Height from pelvis to foot'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='g1_29dof_with_hand',
        description='Name of the robot model'
    )
    
    # 包含机器人描述启动文件
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, 'launch', 'robot_desc.launch.py'
            ])
        )
    )
    
    # RViz节点配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_sim',
        output='screen',
        arguments=[
            '-d', 
            PathJoinSubstitution([
                pkg_share, 'rviz', 'static_display.rviz'
            ])
        ]
    )
    
    # 创建Z高度参数（带负号）
    z_height = Command(['echo ', TextSubstitution(text='-'), LaunchConfiguration('pelvis_to_foot_height')])
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pelvis_2_map',
        output='screen',
        arguments=[
            '--x', '0',          # X偏移
            '--y', '0',          # Y偏移
            '--z', z_height,     # Z偏移（带负号）
            '--yaw', '0',        # 偏航角
            '--pitch', '0',      # 俯仰角
            '--roll', '0',       # 翻滚角
            '--frame-id', 'map', # 父坐标系
            '--child-frame-id', 'pelvis'  # 子坐标系
        ]
    )
    
    return LaunchDescription([
        pelvis_height_arg,
        robot_name_arg,
        robot_desc_launch,
        rviz_node,
        static_tf_node
    ])