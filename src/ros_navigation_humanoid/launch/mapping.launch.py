import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    slam_enable     = LaunchConfiguration('slam_enable')
    nav2_params_file= LaunchConfiguration('nav2_params_file')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    # slam_toolbox= LaunchConfiguration('slam_toolbox')
    
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',          
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch rviz'
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            get_package_share_directory('ros_navigation_humanoid'),
            'move_base_config',
            'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file to use.'
    )
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_slam_enable_cmd = DeclareLaunchArgument(
        'slam_enable',
        default_value='True',
        description='bool value to choose between mapping and navigation'
    )
    slam_enable = LaunchConfiguration('slam_enable')
    
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_navigation_humanoid'),
            'launch/'), 'robot_desc.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    static_map_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_navigation_humanoid'),
            'launch/'), 'map_server.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('ros_navigation_humanoid'),
            'rviz', 'rviz_sim.rviz')],
        condition=IfCondition(rviz)
    )
    
    lidar_simulator_node = Node(
        package='ros_navigation_humanoid',
        executable='lidar_simulator',
        name='lidar_simulator',
        parameters=[{
            'frame_id': 'laser_link',
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'range_min': 0.15,
            'range_max': 10.0,
            'use_sim_time': use_sim_time 
        }],
        output='screen'
    )

    slam_toolbox_config_path = os.path.join(
        get_package_share_directory('ros_navigation_humanoid'),
        'move_base_config',
        'slam_config.yaml' 
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', 
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config_path,
            {'scan_topic': '/laser'}, 
            {'use_sim_time': use_sim_time}, 
            {'map_frame': 'map'},       
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
        ],
        # condition=IfCondition(PythonExpression([slam_toolbox]))
        condition=IfCondition(PythonExpression([slam_enable]))
    )

    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch/'), 'navigation_launch.py']),
        launch_arguments={
            'params_file': nav2_params_file, 
            'use_sim_time': use_sim_time    
            # 'autostart': 'true'              
        }.items()
        # condition=IfCondition(PythonExpression([rviz]))
    )
    
    # statics_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )

    odom_simulator_node = Node(
        package='ros_navigation_humanoid',  
        executable='odom_simulator',
        name='odom_simulator',
        # parameters=[{
        #     'use_sim_time': use_sim_time 
        # }],
        output='screen'
    )
    
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )


    ld = LaunchDescription()
    ld.add_action(clock_bridge_node)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_slam_enable_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(lidar_simulator_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(static_map_server_cmd)
    # ld.add_action(statics_tf_node)
    ld.add_action(nav_node)
    ld.add_action(odom_simulator_node)
    
    return ld
