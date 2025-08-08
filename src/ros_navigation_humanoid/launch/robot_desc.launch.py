import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='g1_29dof_with_hand',
        description='Name of the robot URDF file'
    )
    
    # 使用OpaqueFunction解决上下文相关路径问题
    def launch_nodes_with_context(context):
        # 获取包路径
        pkg_share = get_package_share_directory('ros_navigation_humanoid')
        
        # 构建URDF文件路径
        urdf_path = os.path.join(
            pkg_share,
            'robot_descriptions',
            'unitree_g1',
            context.launch_configurations['robot_name'] + '.urdf'
        )
        
        # 读取URDF文件内容
        try:
            with open(urdf_path, 'r') as f:
                robot_description = f.read()
        except Exception as e:
            print(f"Error reading URDF file: {e}")
            robot_description = ''
        
        # 机器人状态发布节点
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'publish_fixed_frame': False,  
                         'ignore_fixed_joints': True }]
        )
        
        #关节状态发布节点
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
        
        # 关节状态发布GUI节点
        # joint_state_publisher_gui_node = Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # )
        
        return [robot_state_publisher_node, joint_state_publisher_node]
    
    return LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function=launch_nodes_with_context)
    ])