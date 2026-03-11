import os.path
import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# -------------------------- LiDAR 驱动核心配置 --------------------------
xfer_format   = 1    # 0=PointCloud2格式(PointXYZRTL), 1=Livox自定义格式
multi_topic   = 0    # 0=所有LiDAR共享1个话题, 1=每个LiDAR独立话题
data_src      = 0    # 0=数据来源于真实LiDAR（必须保持0，其他值无效）
publish_freq  = 10.0 # 点云发布频率（可选：5.0/10.0/20.0/50.0等）
output_type   = 0    # 输出类型（默认0，无需修改）
frame_id      = 'livox_frame'  # LiDAR对应的坐标帧ID（需与后续建图/可视化保持一致）

# （可选）若用LVX文件回放，需填写路径；真实LiDAR可保留但不影响（驱动会优先用真实设备）
lvx_file_path = '/home/hyz/project/ws_livox/src/Indoor_sampledata.lvx2'

# LiDAR设备编码（默认livox0000000001，若多设备需改；单设备无需修改）
cmdline_bd_code = 'livox0000000001'

# 关键：LiDAR配置文件路径（需确保路径与你的MID360配置文件完全一致）
user_config_path = '/home/chairman/working/ros2/leida/ws_livox/src/livox_ros_driver2/config/MID360_config.json'


# -------------------------- 驱动节点参数列表 --------------------------
livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


# -------------------------- 生成Launch描述（包含驱动节点和point_lio节点） --------------------------
def generate_launch_description():
    # 定义LiDAR驱动节点
    livox_driver_node = Node(
        package='livox_ros_driver2',  # 驱动包名（固定）
        executable='livox_ros_driver2_node',  # 驱动可执行文件名（固定）
        name='livox_lidar_publisher',  # 节点名（自定义，便于识别）
        output='screen',  # 日志输出到终端（便于调试）
        parameters=livox_ros2_params  # 传入上述驱动配置参数
    )

    # 获取point_lio包的路径
    point_lio_dir = get_package_share_directory('point_lio')
    # 定义要包含的point_lio launch文件
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'point_lio.launch.py')
        ),
        # 可根据需要传递参数，例如禁用rviz或指定配置文件
        # launch_arguments={
        #     'rviz': 'False',
        #     'point_lio_cfg_dir': os.path.join(point_lio_dir, 'config', 'mid360.yaml')
        # }.items()
    )

    # 添加所有节点和launch文件到Launch描述中
    ld = LaunchDescription()
    ld.add_action(livox_driver_node)
    ld.add_action(point_lio_launch)

    return ld
