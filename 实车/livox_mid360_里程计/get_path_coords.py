import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# --- 纯数学工具函数（替代Transform矩阵类） ---
def quat_to_yaw(qz, qw):
    """从四元数的z/w分量计算偏航角（yaw），单位：弧度"""
    return 2.0 * math.atan2(qz, qw)

def yaw_to_quat(yaw):
    """将偏航角转换回四元数（仅z/w分量，x/y为0）"""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)

def rotate_point(x, y, yaw):
    """绕原点旋转点(x,y)，旋转角为yaw"""
    cos_ = math.cos(yaw)
    sin_ = math.sin(yaw)
    rx = x * cos_ - y * sin_
    ry = x * sin_ + y * cos_
    return (rx, ry)

class LocalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('local_pose_subscriber')
        
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
                'lidar_init_x': -5.293, 'lidar_init_y': -4.090, 'lidar_init_z': 0.0, 'lidar_init_qz': 0.0, 'lidar_init_qw': 1.0,
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

        # 3. 【核心控制开关】：自由搭配原点和起步点
        selected_map = 'map_1'      
        selected_lidar = 'lidar_A'  
        
        curr_map = map_origins[selected_map]
        curr_lidar = lidar_inits[selected_lidar]

        # 4. 声明并获取参数
        self.declare_parameter("map_origin_x", curr_map['map_origin_x'])
        self.declare_parameter("map_origin_y", curr_map['map_origin_y'])
        self.declare_parameter("map_origin_z", curr_map['map_origin_z']) 
        self.declare_parameter("map_origin_qz", curr_map['map_origin_qz'])
        self.declare_parameter("map_origin_qw", curr_map['map_origin_qw'])

        self.declare_parameter("lidar_init_x", curr_lidar['lidar_init_x'])
        self.declare_parameter("lidar_init_y", curr_lidar['lidar_init_y'])
        self.declare_parameter("lidar_init_z", curr_lidar['lidar_init_z']) 
        self.declare_parameter("lidar_init_qz", curr_lidar['lidar_init_qz'])
        self.declare_parameter("lidar_init_qw", curr_lidar['lidar_init_qw'])

        # 读取参数值
        mx = self.get_parameter("map_origin_x").value
        my = self.get_parameter("map_origin_y").value
        mz = self.get_parameter("map_origin_z").value
        mqz = self.get_parameter("map_origin_qz").value
        mqw = self.get_parameter("map_origin_qw").value

        lx = self.get_parameter("lidar_init_x").value
        ly = self.get_parameter("lidar_init_y").value
        lz = self.get_parameter("lidar_init_z").value
        lqz = self.get_parameter("lidar_init_qz").value
        lqw = self.get_parameter("lidar_init_qw").value

        # 5. 【核心纯数学预计算】（与C++ math版本完全对齐）
        # 5.1 计算地图原点和雷达起步点的偏航角
        map_yaw = quat_to_yaw(mqz, mqw)
        lidar_yaw = quat_to_yaw(lqz, lqw)

        # 5.2 计算雷达起步点与地图原点的直线差值
        dx = lx - mx
        dy = ly - my
        
        # 5.3 旋转到地图原点的角度系下，得到固定偏移量
        self.offset_x = dx * math.cos(map_yaw) - dy * math.sin(map_yaw)
        self.offset_y = dx * math.sin(map_yaw) + dy * math.cos(map_yaw)
        self.offset_z = lz - mz

        # 5.4 预计算偏航角的固定差值
        self.offset_yaw = lidar_yaw - map_yaw

        # 6. 监听局部规划路径 /local_plan
        self.subscription = self.create_subscription(
            Path,
            '/local_plan', 
            self.listener_callback,
            10)
            
        # 7. 发布变换后的目标点
        self.target_pose_pub = self.create_publisher(PoseStamped, '/next_target_pose', 10)
            
        self.get_logger().info(f"纯数学版坐标变换节点已启动 | 预计算偏移：X={self.offset_x:.3f}, Y={self.offset_y:.3f}, Yaw={self.offset_yaw:.3f}")
          
        # 计数器，用于降低终端打印的频率
        self.print_counter = 0

    def listener_callback(self, msg):
        if not msg.poses:
            return

        total_points = len(msg.poses)
        # poses[0] 是小车当前位置，poses[1] 是前方的预测点
        target_index = 1 if total_points > 1 else 0
        next_pose = msg.poses[target_index]
        
        # 读取原始局部路径点的坐标和姿态
        raw_x = next_pose.pose.position.x
        raw_y = next_pose.pose.position.y
        raw_z = next_pose.pose.position.z
        raw_qz = next_pose.pose.orientation.z
        raw_qw = next_pose.pose.orientation.w

        # 6. 【核心计算】纯数学坐标变换（与C++ math版本完全对齐）
        # 6.1 计算原始点的偏航角
        raw_yaw = quat_to_yaw(raw_qz, raw_qw)

        # 6.2 坐标旋转+平移（核心公式）
        final_x = self.offset_x + (raw_x * math.cos(self.offset_yaw) - raw_y * math.sin(self.offset_yaw))
        final_y = self.offset_y + (raw_x * math.sin(self.offset_yaw) + raw_y * math.cos(self.offset_yaw))
        final_z = raw_z

        # 6.3 计算最终偏航角并转换回四元数
        final_yaw = self.offset_yaw + raw_yaw
        _, _, final_qz, final_qw = yaw_to_quat(final_yaw)

        # 7. 构造并发布变换后的PoseStamped消息
        target_msg = PoseStamped()
        target_msg.header = msg.header
        target_msg.header.frame_id = "unified_map"  # 统一地图坐标系
        
        target_msg.pose.position.x = float(final_x)
        target_msg.pose.position.y = float(final_y)
        target_msg.pose.position.z = float(final_z)
        
        # 四元数x/y固定为0（仅偏航角）
        target_msg.pose.orientation.x = 0.0
        target_msg.pose.orientation.y = 0.0
        target_msg.pose.orientation.z = float(final_qz)
        target_msg.pose.orientation.w = float(final_qw)
        
        self.target_pose_pub.publish(target_msg)
        
        # 8. 低频打印（避免刷屏）
        self.print_counter += 1
        if self.print_counter % 10 != 0:
            return
        
        self.get_logger().info(
            f"X={final_x:.3f}, Y={final_y:.3f}, Z={final_z:.3f} | qz={final_qz:.3f}, qw={final_qw:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = LocalPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
