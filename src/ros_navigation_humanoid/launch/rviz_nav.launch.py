import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml  # 动态参数加载
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')
    
    # 参数动态配置
    configured_params = RewrittenYaml(
        source_file=os.path.join(pkg_share, 'move_base_config', 'nav2_params.yaml'),
        root_key='',
        param_rewrites={},
        convert_types=True
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Navigation2完整栈
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'),
            launch_arguments={
                'params_file': configured_params,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),
        
        # 地图服务
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # os.path.join(pkg_share, 'launch', 'map_server.launch.py')
                PathJoinSubstitution([pkg_share, 'launch', 'map_server.launch.py'])
        )),
        
        # 机器人描述
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # os.path.join(pkg_share, 'launch', 'robot_desc.launch.py')
                PathJoinSubstitution([pkg_share, 'launch', 'robot_desc.launch.py'])
        )),
        
        # RViz可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_sim',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'rviz_sim.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        
        # 自定义节点
        Node(
            package='ros_navigation_humanoid',
            executable='bag_play',
            name='bag_play_node',
            parameters=[{
                'loc_topic': '/localization_3d',
                'trajectory_topic': '/trajectory_path',
                'pelvis_to_foot_height': 0.8  
            }]
        )
    ])