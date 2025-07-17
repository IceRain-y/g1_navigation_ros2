from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd2pgm',
            executable='pcd2pgm',
            name='pcd2pgm',
            output='screen',
            parameters=[
                {'file_directory': '/home/robot/map/'},
                {'file_name': 'map'},
                {'thre_z_min': 0.1},
                {'thre_z_max': 1.5},
                {'flag_pass_through': 0},
                {'thre_radius': 0.5},
                {'thres_point_count': 10},
                {'map_resolution': 0.05},
                {'map_topic_name': 'map'}
            ]
        )
    ])