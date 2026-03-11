import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped  # <--- 新增：引入 PoseStamped 消息类型

# --- 手写轻量级几何变换库，杜绝缺少依赖的问题 ---
def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quat_apply(q, v):
    q_v = (v[0], v[1], v[2], 0.0)
    q_inv = (-q[0], -q[1], -q[2], q[3])
    res = quat_mult(quat_mult(q, q_v), q_inv)
    return (res[0], res[1], res[2])

class Transform:
    def __init__(self, x, y, z, qx, qy, qz, qw):
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw

    def inverse(self):
        iqx, iqy, iqz, iqw = -self.qx, -self.qy, -self.qz, self.qw
        ix, iy, iz = quat_apply((iqx, iqy, iqz, iqw), (-self.x, -self.y, -self.z))
        return Transform(ix, iy, iz, iqx, iqy, iqz, iqw)

    def __mul__(self, other):
        rqx, rqy, rqz, rqw = quat_mult(
            (self.qx, self.qy, self.qz, self.qw),
            (other.qx, other.qy, other.qz, other.qw)
        )
        tx, ty, tz = quat_apply(
            (self.qx, self.qy, self.qz, self.qw), 
            (other.x, other.y, other.z)
        )
        return Transform(self.x + tx, self.y + ty, self.z + tz, rqx, rqy, rqz, rqw)
# ------------------------------------------------

class LocalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('local_pose_subscriber')
        
        # ==========================================================
        # 1. 定义 2 个【统一地图原点】
        # ==========================================================
        map_origins = {
            'map_1': { 
                'map_origin_x': -5.543, 'map_origin_y': -5.998, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
            },
            'map_2': { 
                'map_origin_x': -5.548, 'map_origin_y': -0.007, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
            }
        }

        # ==========================================================
        # 2. 定义 4 个【雷达真实起步点】
        # ==========================================================
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

        # ==========================================================
        # 3. 【核心控制开关】：在这里自由搭配你的原点和起步点！
        # ==========================================================
        selected_map = 'map_1'      
        selected_lidar = 'lidar_A'  
        
        curr_map = map_origins[selected_map]
        curr_lidar = lidar_inits[selected_lidar]

        # 4. 声明参数时，直接将上面选中的字典值作为默认值
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

        # 5. 构建这两个点的全局位姿变换矩阵 (x, y, z, qx, qy, qz, qw)
        T_Global_MapOrigin = Transform(mx, my, mz, 0.0, 0.0, mqz, mqw)
        T_Global_LidarInit = Transform(lx, ly, lz, 0.0, 0.0, lqz, lqw)

        # 6. 【核心一步】预计算：起步点相对于地图原点的固定偏移量
        self.T_offset = T_Global_MapOrigin.inverse() * T_Global_LidarInit
        
        # 7. 监听局部规划路径 /local_plan
        self.subscription = self.create_subscription(
            Path,
            '/local_plan', 
            self.listener_callback,
            10)
            
        # ==========================================================
        # 8. 新增发布者：发布变换后的目标点
        # ==========================================================
        self.target_pose_pub = self.create_publisher(PoseStamped, '/next_target_pose', 10)
            
        self.get_logger().info("已启动局部路径监听，并将发布变换后的点到 /next_target_pose 话题...")
          
        # 计数器，用于降低终端打印的频率
        self.print_counter = 0

    def listener_callback(self, msg):
        if not msg.poses:
            return

        total_points = len(msg.poses)
        
        # poses[0] 是小车当前位置，poses[1] 是前方的预测点
        target_index = 1 if total_points > 1 else 0
        next_pose = msg.poses[target_index]
        
        pos = next_pose.pose.position
        ori = next_pose.pose.orientation
        
        # A. 将局部路径中原生传来的预测点构建为一个矩阵
        T_raw = Transform(pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)
        
        # B. 坐标修正：固定偏移量 * 当前原生坐标
        T_final = self.T_offset * T_raw
        
        # C. 【绝对关键】强制还原 Z 轴数据！
        T_final.z = pos.z
        
        # ==========================================================
        # D. 构造并发布 PoseStamped 消息（高频发布，不被打印计数器拦截）
        # ==========================================================
        target_msg = PoseStamped()
        target_msg.header = msg.header
        target_msg.header.frame_id = "unified_map" # 坐标系改为统一地图坐标系
        
        target_msg.pose.position.x = float(T_final.x)
        target_msg.pose.position.y = float(T_final.y)
        target_msg.pose.position.z = float(T_final.z)
        
        target_msg.pose.orientation.x = float(T_final.qx)
        target_msg.pose.orientation.y = float(T_final.qy)
        target_msg.pose.orientation.z = float(T_final.qz)
        target_msg.pose.orientation.w = float(T_final.qw)
        
        self.target_pose_pub.publish(target_msg)
        
        # ==========================================================
        # E. 控制终端打印频率（低频打印，避免刷屏）
        # ==========================================================
        self.print_counter += 1
        if self.print_counter % 10 != 0:
            return
        
        self.get_logger().info(
            f"马上要去的下一个点: "
            f"X: {T_final.x:.3f}, Y: {T_final.y:.3f}, Z: {T_final.z:.3f} "
            f"qz: {T_final.qz:.3f}, qw: {T_final.qw:.3f})"
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