from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 声明参数
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
    
    # AMCL 参数 - 假设用户将 ROS 1 的参数转换成了 Nav2 兼容的 YAML 文件
    amcl_params_file = os.path.join(nav2_tutorial_dir, 'nav_lidar', 'amcl_omni.yaml') 
    nav2_params_file = os.path.join(nav2_tutorial_dir, 'nav_lidar', 'nav2_params.yaml')

    # 2. Gazebo 启动
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_name, 'paused': 'false', 'use_sim_time': use_sim_time}.items()
    )

    # 3. 机器人模型和状态发布 
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', model_path, '-entity', 'wpb_home', '-x', '0.0', '-y', '0.0', '-z', '0.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0', '-robot_namespace', ''],
        output='screen'
    )
    
    # 4. 机器人状态发布 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': PythonExpression(['"', ExecuteProcess(cmd=['xacro', model_path], output='screen'), '"'])},]
    )
    
    # 5. 地图服务器 
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )

    # 6. AMCL 节点 (替换 ROS 1 的 <include file="$(find wpb_home_tutorials)/nav_lidar/amcl_omni.launch" />)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}] 
    )

    # 7. 地图服务器和 AMCL 的生命周期管理
    lifecycle_manager_map_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_amcl',
        output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['map_server', 'amcl']}],
    )
    
    # 8. 导航 (替换 ROS 1 的 move_base)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map_subscribe_transient_local': 'True',
            'autostart': 'False' # 让自定义的 lifecycle manager 管理 AMCL
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
        
        # Map server and AMCL (using Nav2 components)
        map_server_node,
        amcl_node,
        lifecycle_manager_map_amcl,

        # Nav2
        nav2_launch,
        
        # RViz
        rviz_node,
    ])