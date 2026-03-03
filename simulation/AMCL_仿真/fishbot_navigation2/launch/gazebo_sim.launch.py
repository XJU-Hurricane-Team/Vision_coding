import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name_in_model = "fishbot"
    
    # 获取 fishbot_navigation2 的路径 (用于加载机器人模型)
    fishbot_pkg_path = get_package_share_directory("fishbot_navigation2")
    
    # 获取 RC26_world 的路径 (用于加载新地图)
    # 注意：确保你已经编译并source了 RC26_world 包，且该包的 CMakeLists.txt 已正确安装了 my_scene.world 文件
    rc26_world_pkg_path = get_package_share_directory("RC26_world")
    
    # 设置模型文件路径 (保持不变)
    default_model_path = fishbot_pkg_path + '/urdf/fishbot/fishbot.urdf.xacro'
    
    # 设置世界文件路径 (修改为 RC26_world 下的 my_scene.world)
    # 假设编译安装后 my_scene.world 位于 share/RC26_world/ 根目录或同名目录下
    # 如果你的安装路径包含 world 子文件夹，请改为: rc26_world_pkg_path + '/world/my_scene.world'
    default_world_path = rc26_world_pkg_path + '/my_scene.world'

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF的绝对路径')
    
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
            value_type=str)
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
            launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_joint_state_broadcaster'],
            output='screen'
    )
    
    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_effort_controller'],
        output='screen'
    )
    
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, 
                   '-x', '-5.380453',  # X坐标，根据需要修改
                   '-y', '1.839768',  # Y坐标，根据需要修改
                   '-z', '0.050971',  # Z坐标，抬高机器人 (例如0.2米)，防止卡在地下
                   '-Y', '0.004870'   
                   ])
    
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'fishbot_diff_drive_controller'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller],
            )
        ),
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])