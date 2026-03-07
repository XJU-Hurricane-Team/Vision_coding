import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 定义路径变量
    fishbot_nav2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    map_yaml_path = os.path.join(fishbot_nav2_dir, 'maps', 'room.yaml')
    nav2_param_path = os.path.join(fishbot_nav2_dir, 'config', 'nav2_params.yaml')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 2. 启动 Gazebo 仿真
    gazebo_sim_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(fishbot_nav2_dir, 'launch', 'gazebo_sim.launch.py')]
        ),
    )

    # 3. 启动 Map Server
    map_server_node = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_yaml_path}]
    )

    lifecycle_manager_map = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 4. 启动 jie_ware 的 lidar_loc (核心定位节点)
    # [修复] 添加 remappings 解决服务连接不上的问题
    lidar_loc_node = launch_ros.actions.Node(
        package='jie_ware',
        executable='lidar_loc',
        name='lidar_loc',
        output='screen',
        parameters=[{
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'laser_frame': 'laser_link',
            'laser_topic': '/scan',
            'use_sim_time': True
        }],
        remappings=[
            # jie_ware 默认找 /local_costmap/...，Nav2 实际在 /controller_server/...
            ('/local_costmap/clear_entirely_costmap', '/controller_server/clear_entirely_costmap')
        ]
    )

    # 5. 启动 jie_ware 的 costmap_cleaner (辅助工具)
    # [修复] 添加 remappings 解决服务连接不上的问题
    costmap_cleaner_node = launch_ros.actions.Node(
        package='jie_ware',
        executable='costmap_cleaner',
        name='costmap_cleaner',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # 将 jie_ware 的服务名映射到 Nav2 标准节点名
            ('/local_costmap/clear_entirely_costmap', '/controller_server/clear_entirely_costmap'),
            ('/global_costmap/clear_entirely_costmap', '/planner_server/clear_entirely_costmap')
        ]
    )

    # 6. 启动 Nav2 导航栈
    nav2_navigation_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')]
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_param_path,
            'autostart': 'True',
            'map_subscribe_transient_local': 'True'
        }.items()
    )

    # 7. 启动 RViz2
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo_sim_launch,
        map_server_node,
        lifecycle_manager_map,
        lidar_loc_node,
        costmap_cleaner_node,
        nav2_navigation_launch,
        rviz_node
    ])