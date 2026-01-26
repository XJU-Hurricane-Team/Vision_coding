from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='要订阅的里程计话题名称'
        ),
        
        Node(
            package='odometry',  # 修正为实际包名
            executable='odometry_listener',
            name='odometry_listener',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic')
            }]
        )
    ])
