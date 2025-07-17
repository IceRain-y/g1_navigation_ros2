import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 添加了 IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 添加了 PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    bag_path = LaunchConfiguration('bag_path', 
        default=PathJoinSubstitution([pkg_share, 'resources', 'localization_3d.bag']))
    
    # 包含其他launch文件
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'map_server.launch.py')
        )
    )
    
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_desc.launch.py')
        )
    )
    
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_sim',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'bag_play.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ROS2 bag节点
    rosbag_node = Node(
        package='rosbag2_transport',  # 修改为rosbag2_transport
        executable='ros2 bag play',    # 修改为正确的可执行文件
        name='player',
        output='screen',
        arguments=[bag_path, '--clock'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 自定义节点
    bag_play_node = Node(
        package='ros_navigation_humanoid',
        executable='bag_play_node',
        name='bag_play_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'loc_topic': '/localization_3d'},
            {'trajectory_topic': '/trajectory_path'},
            {'pelvis_to_foot_heigth': 0.8}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value=PathJoinSubstitution([pkg_share, 'resources', 'localization_3d.bag'])),
        
        map_server_launch,
        robot_desc_launch,
        rviz_node,
        rosbag_node,
        bag_play_node
    ])
