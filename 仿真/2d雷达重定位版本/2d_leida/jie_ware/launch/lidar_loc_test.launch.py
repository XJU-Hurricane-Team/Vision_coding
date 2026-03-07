from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 声明参数 (使用 LaunchConfiguration 替代 ROS 1 的 <arg> )
    pkg_name = 'jie_ware'
    
    # 假设 wpr_simulation/wpb_home_tutorials 存在 ROS 2 版本
    wpr_sim_dir = get_package_share_directory('wpr_simulation')
    wpb_home_bringup_dir = get_package_share_directory('wpb_home_bringup')
    nav2_tutorial_dir = get_package_share_directory('wpb_home_tutorials')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    world_name = LaunchConfiguration('world_name', default=os.path.join(wpr_sim_dir, 'worlds', 'robocup_home.world'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    model_path = LaunchConfiguration('model', default=os.path.join(wpb_home_bringup_dir, 'urdf', 'wpb_home.urdf'))
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(wpr_sim_dir, 'maps', 'map.yaml'))
    rviz_config_file = LaunchConfiguration('rvizconfig', default=os.path.join(wpr_sim_dir, 'rviz', 'nav.rviz'))
    nav2_params_file = os.path.join(nav2_tutorial_dir, 'nav_lidar', 'nav2_params.yaml') # 假设 Nav2 参数文件

    # 2. Gazebo 启动 (替换 ROS 1 empty_world.launch)
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_name, 'paused': 'false', 'use_sim_time': use_sim_time}.items()
    )

    # 3. 机器人模型和状态发布 (Spawn the objects/robot into Gazebo)
    # **注意: 机器人模型和物体模型的 ROS 2 路径和 spawning 方式可能需要根据实际情况调整**
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', model_path, '-entity', 'wpb_home', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0', '-robot_namespace', ''],
        output='screen'
    )
    
    # 4. 机器人状态发布 (robot_state_publisher)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': PythonExpression(['"', ExecuteProcess(cmd=['xacro', model_path], output='screen'), '"'])},]
    )
    
    # 5. 地图服务器 (Nav2 map_server)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )
    
    # 6. 地图服务器生命周期管理
    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_server']}],
    )

    # 7. 用户 Lidar Loc 节点 (替换 ROS 1 的 <node pkg="jie_ware" type="lidar_loc">)
    lidar_loc_node = Node(
        package=pkg_name,
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

    # 8. 导航 (替换 ROS 1 的 move_base)
    # 使用 Nav2 的 navigation_launch.py 启动所有 Nav2 组件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file, # Placeholder, 需要用户创建 Nav2 兼容的参数文件
            'map_subscribe_transient_local': 'True',
            'autostart': 'False', # 不自动启动 Nav2 节点，由用户或另一个 Lifecycle Manager 控制
            'default_bt_xml_filename': os.path.join(nav2_bringup_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
        }.items(),
    )

    # 9. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value=world_name, description='Gazebo world file'),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gui', default_value=gui, description='Flag to enable Rviz2 GUI'),
        DeclareLaunchArgument('model', default_value=model_path, description='URDF file path'),
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to map file to load'),
        DeclareLaunchArgument('rvizconfig', default_value=rviz_config_file, description='Full path to the RVIZ config file'),
        
        gazebo_launch,
        spawn_entity,
        robot_state_publisher,
        
        # Map server
        map_server_node,
        lifecycle_manager_map_server,

        # User's Node
        lidar_loc_node,
        
        # Nav2
        nav2_launch,
        
        # RViz
        rviz_node,
    ])