from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 里程计话题参数
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='要订阅的里程计话题名称'
        ),
        
        # 机械臂基座标偏移参数（相对于里程计）
        DeclareLaunchArgument('base_x', default_value='12.0', description='基座标X偏移'),
        DeclareLaunchArgument('base_y', default_value='12.0', description='基座标Y偏移'),
        DeclareLaunchArgument('base_z', default_value='12.0', description='基座标Z偏移'),
        DeclareLaunchArgument('base_qx', default_value='0.0', description='基座标四元数X'),
        DeclareLaunchArgument('base_qy', default_value='0.0', description='基座标四元数Y'),
        DeclareLaunchArgument('base_qz', default_value='0.0', description='基座标四元数Z'),
        DeclareLaunchArgument('base_qw', default_value='1.0', description='基座标四元数W'),
        
        # 机械臂TCP偏移参数（相对于基座标）
        DeclareLaunchArgument('tcp_x', default_value='12.0', description='TCP X偏移'),
        DeclareLaunchArgument('tcp_y', default_value='12.0', description='TCP Y偏移'),
        DeclareLaunchArgument('tcp_z', default_value='12.0', description='TCP Z偏移'),
        DeclareLaunchArgument('tcp_qx', default_value='0.0', description='TCP四元数X'),
        DeclareLaunchArgument('tcp_qy', default_value='0.0', description='TCP四元数Y'),
        DeclareLaunchArgument('tcp_qz', default_value='0.0', description='TCP四元数Z'),
        DeclareLaunchArgument('tcp_qw', default_value='1.0', description='TCP四元数W'),
        
        # 坐标转换节点
        Node(
            package='odometry',  # 替换为实际包名
            executable='odometry_transform',
            name='odometry_transform',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                # 基座标参数
                'base_x': LaunchConfiguration('base_x'),
                'base_y': LaunchConfiguration('base_y'),
                'base_z': LaunchConfiguration('base_z'),
                'base_qx': LaunchConfiguration('base_qx'),
                'base_qy': LaunchConfiguration('base_qy'),
                'base_qz': LaunchConfiguration('base_qz'),
                'base_qw': LaunchConfiguration('base_qw'),
                # TCP参数
                'tcp_x': LaunchConfiguration('tcp_x'),
                'tcp_y': LaunchConfiguration('tcp_y'),
                'tcp_z': LaunchConfiguration('tcp_z'),
                'tcp_qx': LaunchConfiguration('tcp_qx'),
                'tcp_qy': LaunchConfiguration('tcp_qy'),
                'tcp_qz': LaunchConfiguration('tcp_qz'),
                'tcp_qw': LaunchConfiguration('tcp_qw'),
            }]
        )
    ])
