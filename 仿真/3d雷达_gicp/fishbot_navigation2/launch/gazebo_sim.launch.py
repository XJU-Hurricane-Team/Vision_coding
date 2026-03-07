import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # ================= 1. 获取各个包的路径 =================
    pkg_livox_sim = get_package_share_directory("ros2_livox_simulation")
    pkg_rc26_world = get_package_share_directory("RC26_world")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    # ================= 2. 配置文件路径 =================
    default_model_path = os.path.join(pkg_livox_sim, 'urdf', 'my_robot', 'my_robot.xacro')
    default_world_path = os.path.join(pkg_rc26_world, 'my_scene.world')

    # ================= 3. 注入环境变量 =================
    livox_share_parent = pkg_livox_sim[:-len('ros2_livox_simulation')]
    rc26_share_parent = pkg_rc26_world[:-len('RC26_world')]
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += f":{livox_share_parent}:{rc26_share_parent}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = f"{livox_share_parent}:{rc26_share_parent}"

    # ================= 4. 定义位置相关的 Launch 参数 =================
    x_pose = LaunchConfiguration('x', default='-5.245')
    y_pose = LaunchConfiguration('y', default='2.021')
    z_pose = LaunchConfiguration('z', default='0.225') # 默认略微抬高，防止卡在地面
    yaw_pose = LaunchConfiguration('yaw', default='0.0')

    declare_x_cmd = DeclareLaunchArgument('x', default_value='-5.245', description='Initial X position')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='2.021', description='Initial Y position')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.225', description='Initial Z position')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw orientation')

    # ================= 5. 节点定义 =================
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', default_model_path]),
        value_type=str
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )

    # 在 Gazebo 中生成小车实体，并传入位置参数
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'mid360_robot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )

    # ================= 6. 加载控制器 =================
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'myrobot_joint_state_broadcaster'],
        output='screen'
    )
    
    # 请确保此处的驱动控制器名称与你的 yaml 配置一致
    load_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'myrobot_diff_drive_controller'],
        output='screen'
    )

    # ================= 7. 返回 LaunchDescription =================
    ld = launch.LaunchDescription()

    # 添加声明参数的 action
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)

    # 添加事件处理器以保证节点启动顺序
    ld.add_action(launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[load_joint_state_controller],
        )
    ))
    ld.add_action(launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_drive_controller],
        )
    ))

    # 添加常规节点
    ld.add_action(robot_state_publisher_node)
    ld.add_action(launch_gazebo)
    ld.add_action(spawn_entity_node)

    return ld