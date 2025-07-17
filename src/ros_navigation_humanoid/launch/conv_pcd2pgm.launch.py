import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction  # 新增延迟启动

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')
    
    # pcd2pgm节点
    pcd2pgm_node = Node(
        package='pcd2pgm',
        executable='pcd2pgm',  
        name='pcd2pgm',
        output='screen',
        parameters=[
            {'file_directory': PathJoinSubstitution([pkg_share, 'maps/'])},
            {'file_name': 'map'},
            {'qos_policy': 2},  # 设置 QoS 为 Transient Local
            {'map_topic_name': 'map'},
            {'thre_z_min': 0.2},
            {'thre_z_max': 0.8},
            {'flag_pass_through': 0},
            {'thre_radius': 0.5},
            {'thres_point_count': 10},
            {'map_resolution': 0.05}
        ]
    )
    
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_sim',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'conv_pcd2pgm.rviz'])]
    )
    
    return LaunchDescription([
        pcd2pgm_node,
        TimerAction(period=0.0, actions=[rviz_node])
    ])    