import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取与拼接默认路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
    get_package_share_directory('fishbot_navigation2'),
    'config',
    'rviz',
    'display_model.rviz' # 确保文件名和路径对得上
)
    # 创建 launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration('map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration('params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    # 定义 pointcloud_to_laserscan 节点 (3D转2D)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'laser_link',
            'transform_tolerance': 0.01,
            'min_height': -0.1,  # 扫描高度范围下限
            'max_height': 1.0,   # 扫描高度范围上限
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.0087, # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'qos_reliability': 2,
            'use_sim_time': use_sim_time
        }],
        # 将输入重映射到 Gazebo 输出的 3D 点云话题
        remappings=[('cloud_in', '/scan/point_cloud_PointCloud2'),
                    ('scan', '/scan_2d')] # 输出回标准的 /scan 供 AMCL 使用
    )

    return launch.LaunchDescription([
        # 声明新的 launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load'),
        
        # 启动转换节点
        pointcloud_to_laserscan_node,

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 launch 参数替换原有参数
            launch_arguments={'map': map_yaml_path, 'use_sim_time': use_sim_time, 'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])