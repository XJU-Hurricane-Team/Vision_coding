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
        
        # 新增：声明误差补偿参数，允许外部传入
        DeclareLaunchArgument('offset_x', default_value='-0.001', description='X轴误差补偿'),
        DeclareLaunchArgument('offset_y', default_value='-0.02', description='Y轴误差补偿'),
        DeclareLaunchArgument('offset_z', default_value='0.0', description='Z轴误差补偿'),
        DeclareLaunchArgument('offset_w', default_value='0.0', description='W轴误差补偿'),
        
        Node(
            package='odometry',  # 修正为实际包名
            executable='odometry_listener',
            name='odometry_listener',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                # 将 Launch 接收到的参数传递给 C++ 节点内部
                'offset_x': LaunchConfiguration('offset_x'),
                'offset_y': LaunchConfiguration('offset_y'),
                'offset_z': LaunchConfiguration('offset_z'),
                'offset_w': LaunchConfiguration('offset_w')
            }]
        )
    ])