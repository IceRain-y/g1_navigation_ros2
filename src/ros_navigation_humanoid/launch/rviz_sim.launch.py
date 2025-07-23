import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml  

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')
    
    # 声明参数（使用标准声明方式）
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 动态参数处理（解决路径问题）
    nav_params_file = PathJoinSubstitution([
        pkg_share, 'move_base_config', 'nav2_params.yaml'
    ])
    configured_params = RewrittenYaml(
        source_file=nav_params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # 包含其他launch文件（传递参数）
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'map_server.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_desc.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_sim',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'rviz_sim.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 导航栈集成（使用标准导航启动）
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': configured_params,  # 使用动态参数
            'use_sim_time': use_sim_time,
            'autostart': 'true'  
        }.items()
    )
    
    # 自定义节点（修正参数名）
    bag_play_node = Node(
        package='ros_navigation_humanoid',
        executable='bag_play',
        name='bag_play_node',
        output='screen',
        parameters=[
            {'loc_topic': '/localization_3d'},
            {'trajectory_topic': '/trajectory_path'},
            {'pelvis_to_foot_height': 0.8},  
            {'use_sim_time': use_sim_time}
        ]
    )

    # # 静态TF广播
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],  # 发布map->odom变换
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # # 地图服务器生命周期管理
    # lifecycle_activation = ExecuteProcess(
    #     cmd=["ros2 lifecycle set /map_server configure && "
    #          "ros2 lifecycle set /map_server activate"],
    #     shell=True,
    #     output='screen'
    # )

    lidar_sim_node = Node(
        package='ros_navigation_humanoid',
        executable='lidar_simulator',
        name='lidar_simulator',
        parameters=[{
            'frame_id': 'laser',
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'range_min': 0.1,
            'range_max': 10.0
        }],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'global_costmap',
                'local_costmap'
            ]}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_server_launch,
        # lifecycle_activation,
        lifecycle_manager,
        lidar_sim_node,
        robot_desc_launch,
        static_tf_node,
        rviz_node,
        nav2_bringup,
        bag_play_node
    ])