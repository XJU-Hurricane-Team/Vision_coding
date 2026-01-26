from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare  # 新增

def generate_launch_description():
    # 声明启动参数（可选，用于在命令行或上级launch文件中传递参数）
    declare_file_directory = DeclareLaunchArgument(
        'file_directory',
        default_value='/home/chairman/working/ros2/leida/ws_livox/src/fast_lio/PCD/',
        description='PCD文件所在目录'
    )
    declare_file_name = DeclareLaunchArgument(
        'file_name',
        default_value='scans',
        description='PCD文件名（不含扩展名）'
    )
    declare_thre_z_min = DeclareLaunchArgument(
        'thre_z_min',
        default_value='0.2',
        description='Z轴最小值（直通滤波）'
    )
    declare_thre_z_max = DeclareLaunchArgument(
        'thre_z_max',
        default_value='2.0',
        description='Z轴最大值（直通滤波）'
    )
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.05',
        description='地图分辨率（米/格）'
    )
    # 新增：声明输出目录参数，默认指向包内的save文件夹
    declare_output_directory = DeclareLaunchArgument(
        'output_directory',
        default_value='/home/chairman/working/ros2/leida/ws_livox/src/pcd2pgm/save',
        description='生成文件的保存目录'
    )

    # 定义pcd2pgm节点
    pcd2pgm_node = Node(
        package='pcd2pgm',
        executable='pcd2pgm',
        name='pcd2pgm_node',
        output='screen',
        parameters=[{
            'file_directory': LaunchConfiguration('file_directory'),
            'file_name': LaunchConfiguration('file_name'),
            'thre_z_min': LaunchConfiguration('thre_z_min'),
            'thre_z_max': LaunchConfiguration('thre_z_max'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'flag_pass_through': 0,
            'thre_radius': 0.5,
            'thres_point_count': 10,
            'map_topic_name': 'map',
            'output_directory': LaunchConfiguration('output_directory')  # 新增：添加输出目录参数
        }]
    )

    # 构建启动描述
    return LaunchDescription([
        declare_file_directory,
        declare_file_name,
        declare_thre_z_min,
        declare_thre_z_max,
        declare_map_resolution,
        declare_output_directory,  # 新增：添加输出目录参数声明
        pcd2pgm_node
    ])
