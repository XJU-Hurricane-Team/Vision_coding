from launch import LaunchDescription
from launch_ros.actions import Node

# =======================================================
# 🚗 [快捷修改区] 请在这里选择预设的 Map 和 Lidar 位置编号！
# =======================================================
# 1. 选择 Map 原点编号 (填 1 或 2)
SELECTED_MAP = 1

# 2. 选择 Lidar 初始位置编号 (填 1, 2, 3 或 4)
SELECTED_LIDAR = 1

# --- 预设数据字典 ---
# 定义 2 个【统一地图原点】
PRESET_MAPS = {
    1: {'map_origin_x': -5.543, 'map_origin_y': -5.998, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0},
    2: {'map_origin_x': -5.548, 'map_origin_y': -0.007, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0}
}

# 定义 4 个【雷达真实起步点】
PRESET_LIDARS = {
    1: {'lidar_init_x': 2.472, 'lidar_init_y': -5.402, 'lidar_init_z': 0.0, 'lidar_init_qz': -0.714, 'lidar_init_qw': 0.701},
    2: {'lidar_init_x': 1.500, 'lidar_init_y':  2.000, 'lidar_init_z': 0.0, 'lidar_init_qz':  0.000, 'lidar_init_qw': 1.000},
    3: {'lidar_init_x': 10.000, 'lidar_init_y': 5.000, 'lidar_init_z': 0.0, 'lidar_init_qz':  0.000, 'lidar_init_qw': 1.000},
    4: {'lidar_init_x': 0.000, 'lidar_init_y':  0.000, 'lidar_init_z': 0.0, 'lidar_init_qz':  0.000, 'lidar_init_qw': 1.000}
}
# =======================================================


def generate_launch_description():
    
    # 自动合并选中的地图和雷达参数
    current_params = {}
    current_params.update(PRESET_MAPS[SELECTED_MAP])
    current_params.update(PRESET_LIDARS[SELECTED_LIDAR])

    # 调用里程计数学转换节点
    odometry_transform_math_node = Node(
        package='odometry',
        executable='odometry_transform_math', 
        name='odometry_transform_math_node',
        output='screen',
        parameters=[current_params], 
        remappings=[
            ('odom_raw', '/Odometry'), 
            ('odom_map', '/odom_map')  
        ]
    )

    return LaunchDescription([
        odometry_transform_math_node
    ])