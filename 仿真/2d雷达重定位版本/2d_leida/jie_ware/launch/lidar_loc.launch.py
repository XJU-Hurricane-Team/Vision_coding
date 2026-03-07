# 示例 launch/lidar_loc.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jie_ware',
            executable='lidar_loc',
            name='lidar_loc',
            output='screen',
            parameters=[{
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'laser_frame': 'laser',
                'laser_topic': 'scan'
            }]
        )
    ])