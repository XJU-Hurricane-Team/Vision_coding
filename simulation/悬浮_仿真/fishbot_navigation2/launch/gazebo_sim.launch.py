import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # --- 关键修改 1: 配置环境变量确保 Gazebo 能找到模型 ---
    # 获取 chassis 包的 share 目录 (例如 install/chassis/share/chassis)
    chassis_pkg_dir = get_package_share_directory("chassis")
    # 获取上一级目录 (install/chassis/share)，这是 Gazebo 查找 package:// 前缀的根目录
    chassis_model_path = os.path.dirname(chassis_pkg_dir)
    
    # 将其添加到 GAZEBO_MODEL_PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + chassis_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = chassis_model_path

    print(f"Set GAZEBO_MODEL_PATH to include: {chassis_model_path}")

    # --------------------------------------------------------

    robot_name_in_model = "chassis"
    
    # 获取各包路径
    fishbot_pkg_path = get_package_share_directory("fishbot_navigation2")
    chassis_pkg_path = get_package_share_directory("chassis")
    rc26_world_pkg_path = get_package_share_directory("RC26_world")
    
    # 设置模型文件路径
    default_model_path = chassis_pkg_path + '/urdf/chassis.urdf'
    
    # 设置世界文件路径
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
            # ========================================================
            # 👉 这里的 extra_gazebo_args 是核心！它强制 Gazebo 加载底层服务插件
            # ========================================================
            launch_arguments=[
                ('world', default_world_path), 
                ('verbose', 'true')
            ]
    )
    
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model, 
                   # --- 参数位置按原来的来，但 Z 轴微调 ---
                   '-x', '-5.245',
                   '-y', '2.021',
                   '-z', '0.225',  
                   '-Y', '0.0'
                   ])

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])