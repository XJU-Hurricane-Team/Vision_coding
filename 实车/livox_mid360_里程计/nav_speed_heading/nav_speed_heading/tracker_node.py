import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
# 【核心修改】引入我们将要自定义的消息类型
from custom_nav_msgs.msg import SpeedHeading

# =======================================================
# 🚗 [快捷修改区] 请在这里选择预设的 Map 和 Lidar 位置编号！
# =======================================================
SELECTED_MAP = 1
SELECTED_LIDAR = 1

PRESET_MAPS = {
    1: [0.0, 1.0],         
    2: [0.707, 0.707],     
}

PRESET_LIDARS = {
    1: [0.0, 1.0],         
    2: [0.707, 0.707],     
    3: [1.0, 0.0],         
    4: [-0.707, 0.707],    
}
# =======================================================

def quat_to_yaw(qz, qw):
    """从四元数的z/w分量计算偏航角（yaw），单位：弧度"""
    return 2.0 * math.atan2(qz, qw)


class VelocityHeadingNode(Node):
    def __init__(self):
        super().__init__('velocity_heading_node')
        
        curr_map_qz, curr_map_qw = PRESET_MAPS[SELECTED_MAP]
        curr_lidar_qz, curr_lidar_qw = PRESET_LIDARS[SELECTED_LIDAR]

        self.declare_parameter("map_origin_qz", curr_map_qz)
        self.declare_parameter("map_origin_qw", curr_map_qw)
        self.declare_parameter("lidar_init_qz", curr_lidar_qz)
        self.declare_parameter("lidar_init_qw", curr_lidar_qw)

        mqz = self.get_parameter("map_origin_qz").value
        mqw = self.get_parameter("map_origin_qw").value
        lqz = self.get_parameter("lidar_init_qz").value
        lqw = self.get_parameter("lidar_init_qw").value

        map_yaw = quat_to_yaw(mqz, mqw)
        lidar_yaw = quat_to_yaw(lqz, lqw)
        self.offset_yaw = lidar_yaw - map_yaw
        self.latest_yaw = 0.0

        # === 【修改】使用我们自定义的 SpeedHeading 类型 ===
        self.data_pub = self.create_publisher(SpeedHeading, '/nav_speed_heading_data', 10)

        self.plan_sub = self.create_subscription(Path, '/local_plan', self.plan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
            
        self.get_logger().info(f"速度与方向提取节点已启动 | Map={SELECTED_MAP}, Lidar={SELECTED_LIDAR}")
        self.get_logger().info("数据将同时发布至话题: /nav_speed_heading_data (类型: custom_nav_msgs/SpeedHeading)")

    def plan_callback(self, msg):
        if not msg.poses:
            return

        target_index = 1 if len(msg.poses) > 1 else 0
        next_pose = msg.poses[target_index]
        
        raw_qz = next_pose.pose.orientation.z
        raw_qw = next_pose.pose.orientation.w

        raw_yaw = quat_to_yaw(raw_qz, raw_qw)
        self.latest_yaw = self.offset_yaw + raw_yaw

    def cmd_callback(self, msg):
        linear_v = msg.linear.x
        angular_w = msg.angular.z
        
        pub_msg = SpeedHeading()
        pub_msg.linear_velocity = float(linear_v)   # 线速度
        pub_msg.angular_velocity = float(angular_w) # 角速度
        pub_msg.yaw = float(self.latest_yaw)        # 偏航角
        
        self.data_pub.publish(pub_msg)
            
        self.get_logger().info(
            f"{linear_v: .3f} m/s | {angular_w: .3f} rad/s | {self.latest_yaw: .3f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VelocityHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()