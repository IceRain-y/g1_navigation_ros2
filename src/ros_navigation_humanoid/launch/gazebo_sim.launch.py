import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_navigation_humanoid')

    # Xacro 文件路径
    xacro_file = os.path.join(
        pkg_share, 'robot_descriptions', 'unitree_g1', 'g1_29dof_with_hand_gazebo.xacro'
    )

    # 使用 xacro 实时展开 URDF
    robot_description = Command(['xacro ', xacro_file])

    # 手动启动 Gazebo，显式加载插件
    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            os.path.join(pkg_share, 'worlds', 'room.world')
        ],
        output='screen'
    )

    # 启动 Gazebo GUI（可选）
    start_gazebo_gui_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 使用 URDF 字符串传递给 spawn_entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '1.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )

    # 关节控制器（可选，需配置 ros2_control）
    joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller'],
        output='screen'
    )

    # 导航系统（使用仿真时间）
    navigation_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'sim.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # 设置 Gazebo 模型路径
        ExecuteProcess(
            cmd=['bash', '-c', f'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{os.path.join(pkg_share, "models")}'],
            shell=True
        ),

        start_gazebo_cmd,
        start_gazebo_gui_cmd,  
        robot_state_publisher,
        spawn_entity,
        joint_state_controller,
        navigation_launch
    ])