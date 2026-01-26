from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    declare_yaml_path = DeclareLaunchArgument(
        'yaml_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('pcd2pgm'),
            'save',
            'scans.yaml'  # 对应scans.pgm的YAML描述文件
        ]),
        description='YAML地图描述文件路径'
    )
    
    declare_map_topic = DeclareLaunchArgument(
        'map_topic',
        default_value='scans_map',  # 地图发布话题
        description='地图发布话题名称'
    )

    # 定义pgm_publisher节点（发布scans.pgm）
    pgm_publisher_node = Node(
        package='pcd2pgm',
        executable='pgm_publisher',
        name='pgm_publisher_node',
        output='screen',
        parameters=[{
            'yaml_path': LaunchConfiguration('yaml_path'),
            'map_topic': LaunchConfiguration('map_topic')
        }]
    )

    # 启动RViz2并加载地图配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('pcd2pgm'),
            'rviz',
            'map_view.rviz'  # 确保该文件存在且配置正确
        ])]
    )

    return LaunchDescription([
        declare_yaml_path,
        declare_map_topic,
        pgm_publisher_node,
        rviz_node  # 启动RViz2
    ])
