import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions

# =======================================================
# 🚗 [快捷修改区] 请在这里直接修改小车初始化的坐标数值！
# =======================================================
INIT_X = -5.293      # 小车在 X 轴的坐标 (米)
INIT_Y = -4.090      # 小车在 Y 轴的坐标 (米)
INIT_Z = 0.000       # 车头朝向 (四元数 z)
INIT_W = 1.000       # 车头朝向 (四元数 w)
# =======================================================

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')
    livox_ros_driver2_dir = get_package_share_directory('livox_ros_driver2')

    # 所有路径现在都指向 fishbot_navigation2 包内部
    rviz_config_dir = os.path.join(fishbot_navigation2_dir, 'config', 'rviz', 'nav2_default_view.rviz')
    map_yaml_path = os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml')
    pcd_file_path = os.path.join(fishbot_navigation2_dir, 'PCD', 'test.pcd')
    nav2_param_path = os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
    urdf_path = os.path.join(fishbot_navigation2_dir, 'urdf', 'my_robot', 'my_robot.xacro')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    init_pose_str = '{header: {frame_id: "map"}, pose: {pose: {position: {x: %s, y: %s, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: %s, w: %s}}}}' % (INIT_X, INIT_Y, INIT_Z, INIT_W)

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),

        # 0. 启动真实 Livox 雷达驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([livox_ros_driver2_dir, '/launch_ROS2', '/msg_MID360_launch.py']),
        ),

        # 0.5 直接发布机器人的 TF 和 URDF
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 1. 启动 Fast-LIO
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([fast_lio_dir, '/launch', '/start.launch.py']),
            launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items(),
        ),

        # 2. TF 坐标系桥接
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher', name='odom_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init']
        ),
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher', name='body_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_footprint']
        ),

        # 3. 启动 pointcloud_to_laserscan
        launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/cloud_registered_body'), ('scan', '/scan')], 
            parameters=[{
                'target_frame': 'base_footprint', 'transform_tolerance': 0.01,
                'min_height': 0.1, 'max_height': 1.0, 'angle_min': -3.14159, 'angle_max': 3.14159,
                'angle_increment': 0.0087, 'scan_time': 0.1, 'range_min': 0.3, 'range_max': 30.0,
                'use_inf': True, 'inf_epsilon': 1.0, 'use_sim_time': use_sim_time
            }]
        ),

        # 4. 启动 GICP 定位节点
        launch_ros.actions.Node(
            package="small_gicp_relocalization",
            executable="small_gicp_relocalization_node",
            name="small_gicp_relocalization",
            remappings=[("registered_scan", "/cloud_registered")],
            parameters=[{
                "num_threads": 4, "num_neighbors": 20, "global_leaf_size": 0.5,
                "registered_leaf_size": 0.5, "max_dist_sq": 0.25, "map_frame": "map",
                "odom_frame": "odom", "base_frame": "odom", "lidar_frame": "odom",
                "robot_base_frame": "base_footprint", "prior_pcd_file": pcd_file_path,
                "use_sim_time": use_sim_time
            }]
        ),

        # 5. 启动 Nav2 Map Server
        launch_ros.actions.Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': use_sim_time}]
        ),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]
        ),

        # 6. 启动 Nav2 核心导航框架
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_param_path}.items(),
        ),

        # 7. 启动 RViz2
        launch_ros.actions.Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 8. 延迟 5 秒后，自动发布初始位姿给 GICP
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped', init_pose_str],
                    output='screen'
                )
            ]
        ),
    ])