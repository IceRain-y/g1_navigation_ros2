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
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',          
        description='Use simulation (Gazebo) clock if true'
    )
    

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch rviz'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='/home/p30021405276/code_pnc/humanoid_navigation/src/ros_navigation_humanoid/maps/map.yaml', 
        description='Full path to map yaml file to load for localization.'
    )
    map_file = LaunchConfiguration('map_file')

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
            'launch/'), 'robot_desc.launch.py'])
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
            'range_max': 10.0
        }],
        output='screen'
    )

    # nav_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('ros_navigation_humanoid'),
    #         'launch/'), 'nav2_navigation_launch.py']),
    #     launch_arguments={
    #         'params_file': nav2_params_file, # <--- Qui passiamo il tuo file di configurazione
    #         'use_sim_time': use_sim_time    # Passa use_sim_time anche a Nav2
    #         # 'autostart': 'true'              # Generalmente utile per Nav2
    #     }.items()
    #     # condition=IfCondition(PythonExpression([rviz]))
    # )
    
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
    
    odom_simulator_node = Node(
        package='ros_navigation_humanoid',  
        executable='odom_simulator',
        name='odom_simulator',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,          
            {'use_sim_time': use_sim_time},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id':  'odom'},
            {'global_frame_id': 'map'},
            {'scan_topic': '/laser'},
        ],
        condition=UnlessCondition(slam_enable)  
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_slam_enable_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(lidar_simulator_node)
    ld.add_action(static_map_server_cmd)
    ld.add_action(amcl_node)
    ld.add_action(nav_node)
    ld.add_action(odom_simulator_node)
    
    return ld
