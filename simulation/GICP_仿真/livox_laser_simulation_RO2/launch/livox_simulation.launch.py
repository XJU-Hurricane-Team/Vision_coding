import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # ============================================================================
    package_name = 'ros2_livox_simulation'
    robot_name = 'my_robot'
    world_file_name = 'custom_room.world'
    use_sim_time = True
    gui = True
    # ============================================================================

    # ============================================================================
    # bash传参配置
    use_sim_time_arg = LaunchConfiguration('use_sim_time', default=use_sim_time) 
    gui_arg = LaunchConfiguration('gui', default=gui)

    x_pose = LaunchConfiguration('x', default='3.5')
    y_pose = LaunchConfiguration('y', default='1.0')
    z_pose = LaunchConfiguration('z', default='0.0')
    yaw_pose = LaunchConfiguration('yaw', default='90.0')
    # ============================================================================

    # ============================================================================
    # 包路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros2_livox_simulation = get_package_share_directory(package_name)
    pkg_share = get_package_share_directory(package_name)[:-len(package_name)]
    # ============================================================================

    # 给GAZEBO_MODEL_PATH添加路径
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share
    
    # ============================================================================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=str(use_sim_time),
        description='Use simulation (Gazebo) clock if true')
    
    declare_x_cmd = DeclareLaunchArgument(
        'x', default_value='2', description='Initial X position')
    declare_y_cmd = DeclareLaunchArgument(
        'y', default_value='1.0', description='Initial Y position')
    declare_z_cmd = DeclareLaunchArgument(
        'z', default_value='0.0', description='Initial Z position')
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='3.14', description='Initial Yaw orientation')
    # ============================================================================

    # gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_ros2_livox_simulation, 'worlds', world_file_name),
            'gui': gui_arg,
            'verbose': 'true'
        }.items()
    )
    
    xacro_file = os.path.join(pkg_ros2_livox_simulation, 'urdf', robot_name, robot_name +'.xacro')
    robot_description = Command([f'xacro {xacro_file}'])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},{'use_sim_time': use_sim_time_arg}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time_arg}]
    )

    # ============================================================================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mid360_platform', 
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )
    # ============================================================================
    
    bringup_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', '/livox/lidar/pointcloud'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',   
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 0.1,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )


    
    rviz_path = os.path.join(pkg_ros2_livox_simulation, 'rviz', robot_name + '.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_path],
        parameters=[{'use_sim_time': use_sim_time_arg}]
    )

    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'myrobot_joint_state_broadcaster'],
        output='screen'
    )

    load_controller_event = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(bringup_pointcloud_to_laserscan_node)
    ld.add_action(rviz) 
    ld.add_action(load_controller_event)
    return ld