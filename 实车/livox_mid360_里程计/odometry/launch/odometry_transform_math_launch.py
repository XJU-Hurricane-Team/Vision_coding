from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 定义 2 个【统一地图原点】
    map_origins = {
        'map_1': { 
            'map_origin_x': -5.543, 'map_origin_y': -5.998, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
        },
        'map_2': { 
            'map_origin_x': -5.548, 'map_origin_y': -0.007, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
        }
    }

    # 2. 定义 4 个【雷达真实起步点】
    lidar_inits = {
        'lidar_A': { 
            'lidar_init_x': 2.472, 'lidar_init_y': -5.402, 'lidar_init_z': 0.0, 'lidar_init_qz': -0.714, 'lidar_init_qw': 0.701,
        },
        'lidar_B': { 
            'lidar_init_x': 1.500, 'lidar_init_y': 2.000, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        },
        'lidar_C': { 
            'lidar_init_x': 10.000, 'lidar_init_y': 5.000, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        },
        'lidar_D': { 
            'lidar_init_x': 0.000, 'lidar_init_y': 0.000, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
        }
    }

    selected_map = 'map_1'      
    selected_lidar = 'lidar_A'  
    
    current_params = {}
    current_params.update(map_origins[selected_map])
    current_params.update(lidar_inits[selected_lidar])

    # 这里调用的是新的 math 节点
    odometry_transform_math_node = Node(
        package='odometry',
        executable='odometry_transform_math', # <--- 注意这里改了
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