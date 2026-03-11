from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 定义 2 个【统一地图原点】
    map_origins = {
        'map_1': { # 场地一的地图原点
            'map_origin_x': -5.543, 'map_origin_y': -5.998, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
        },
        'map_2': { # 默认全零地图原点
            'map_origin_x': -5.548, 'map_origin_y': -0.007, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
        }
    }

    # 2. 定义 4 个【雷达真实起步点】
    lidar_inits = {
        'lidar_A': { # 起步点 A
            'lidar_init_x': -5.293, 'lidar_init_y': -4.090, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        },
        'lidar_B': { # 起步点 B
            'lidar_init_x': 1.500, 'lidar_init_y': 2.000, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        },
        'lidar_C': { # 起步点 C
            'lidar_init_x': 10.000, 'lidar_init_y': 5.000, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        },
        'lidar_D': { # 默认全零起步点
            'lidar_init_x': 0.000, 'lidar_init_y': 0.000, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        }
    }

    # ==========================================================
    # 3. 【核心控制开关】：在这里自由搭配你的原点和起步点！
    # ==========================================================
    selected_map = 'map_1'      # 从 'map_1', 'map_2' 中选
    selected_lidar = 'lidar_A'  # 从 'lidar_A', 'lidar_B', 'lidar_C', 'lidar_D' 中选
    
    
    # 将选中的地图原点和雷达起步点合并成一个完整的参数字典
    current_params = {}
    current_params.update(map_origins[selected_map])
    current_params.update(lidar_inits[selected_lidar])

    odometry_transform_node = Node(
        package='odometry',
        executable='odometry_transform',
        name='odometry_transform_node',
        output='screen',
        # 4. 直接将合并后的字典传给节点
        parameters=[current_params], 
        remappings=[
            ('odom_raw', '/Odometry'), 
            ('odom_map', '/odom_map')  
        ]
    )

    return LaunchDescription([
        odometry_transform_node
    ])