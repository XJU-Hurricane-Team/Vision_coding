from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='要订阅的里程计话题名称'
        ),
        
        # 【新增】XYZ 偏移参数
        DeclareLaunchArgument('x_offset', default_value='0.0', description='X轴坐标补偿偏移量'),
        DeclareLaunchArgument('y_offset', default_value='0.0', description='Y轴坐标补偿偏移量'),
        DeclareLaunchArgument('z_offset', default_value='0.091', description='Z轴坐标补偿偏移量'), # 默认增加 0.091
        
        Node(
            package='odometry',
            executable='odometry_listener',
            name='odometry_listener',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                # 传递参数
                'x_offset': LaunchConfiguration('x_offset'),
                'y_offset': LaunchConfiguration('y_offset'),
                'z_offset': LaunchConfiguration('z_offset')
            }]
        )
    ])