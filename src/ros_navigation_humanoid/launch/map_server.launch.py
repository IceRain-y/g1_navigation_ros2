import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')
    
    pgm_map_name_arg = DeclareLaunchArgument(
        'pgm_map_name',
        default_value='map',
        description='PGM地图文件名（不含扩展名）'
    )
    
    display_point_cloud_arg = DeclareLaunchArgument(
        'display_point_cloud',
        default_value='true',
        description='是否显示点云'
    )
    
    # 使用PathJoinSubstitution正确构建路径，避免字符串拼接
    map_yaml_file = PathJoinSubstitution([
        pkg_share,
        'maps',
        # LaunchConfiguration('pgm_map_name'),
        'map.yaml'  # 作为单独的路径元素
    ])
    
    pcd_file_path = PathJoinSubstitution([
        pkg_share,
        'maps',
        # LaunchConfiguration('pgm_map_name'),
        'map.pcd'  # 作为单独的路径元素
    ])
    
    # 使用navigation2的map_server节点
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file
        }]
    )
    
    # 点云显示节点配置
    point_cloud_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('display_point_cloud')),
        actions=[
            Node(
                package='ros_navigation_humanoid',
                executable='point_cloud_display',
                name='point_cloud_display',
                output='screen',
                parameters=[{
                    'publish_topic': 'map_point_cloud',
                    'frame_id': 'map',
                    'file_path': pcd_file_path,
                    'z_min': 0.0,
                    'z_max': 2.0,
                    'voxel_filter_size': 0.05,
                    'pub_rate': 0.5
                }]
            )
        ]
    )
    
    return LaunchDescription([
        pgm_map_name_arg,
        display_point_cloud_arg,
        map_server_node,
        point_cloud_group
    ])    