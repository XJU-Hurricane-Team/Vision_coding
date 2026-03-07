import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState, SpawnEntity, DeleteEntity
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import sys, select, termios, tty
import threading

msg = """
=============================================
      🛰️ Livox MID360 小车全向控制中心 🛰️
=============================================
【全向移动】 (麦克纳姆轮模式)
   u    i    o      (左前) (前进) (右前)
   j    k    l      (左移) (停止) (右移)
   m    ,    .      (左后) (后退) (右后)

【原地旋转】
   a : 逆时针旋转 (左转)
   d : 顺时针旋转 (右转)

【系统控制】
   Space 键 : 🛑 紧急停止
   r 键     : 🔄 原地重载 (销毁并重建满血模型)
   w / s    : 速度调高 / 调低 (当前步长 0.1)

CTRL-C 退出
=============================================
"""

# 方向映射：(x速度, y速度, 角速度)
moveBindings = {
    'i': (1.0, 0.0, 0.0),   # 前进
    ',': (-1.0, 0.0, 0.0),  # 后退
    'j': (0.0, 1.0, 0.0),   # 左平移
    'l': (0.0, -1.0, 0.0),  # 右平移
    'u': (1.0, 1.0, 0.0),   # 左前
    'o': (1.0, -1.0, 0.0),  # 右前
    'm': (-1.0, 1.0, 0.0),  # 左后
    '.': (-1.0, -1.0, 0.0), # 右后
    'a': (0.0, 0.0, 1.0),   # 原地左转
    'd': (0.0, 0.0, -1.0),  # 原地右转
}

class LivoxTeleop(Node):
    def __init__(self):
        super().__init__('livox_teleop')
        
        # 对应 gazebo_control_plugin.xacro 中的映射
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 基础服务客户端
        self.get_state_cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        # 自动获取当前小车的 URDF 模型
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.robot_urdf = ""
        self.urdf_sub = self.create_subscription(String, '/robot_description', self.urdf_cb, qos)

        self.speed = 0.6  # 初始速度
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

        # 启动动力发布定时器
        self.timer = self.create_timer(0.1, self.publish_commands)

    def urdf_cb(self, msg):
        self.robot_urdf = msg.data

    def publish_commands(self):
        twist = Twist()
        twist.linear.x = self.target_linear_x
        twist.linear.y = self.target_linear_y
        twist.angular.z = self.target_angular_z
        self.vel_pub.publish(twist)

    def stop(self):
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

    def reset_model(self):
        """原地重载逻辑"""
        if not self.robot_urdf:
            print("\n[错误] 未获取到模型数据！")
            return
        
        req = GetEntityState.Request()
        req.name = 'my_robot'  # 对应你 launch 中的 entity 名字
        
        future = self.get_state_cli.call_async(req)
        def cb(f):
            res = f.result()
            if res.success:
                pos = res.state.pose.position
                print(f"\n[记录] 位置: X={pos.x:.2f}, Y={pos.y:.2f}。正在重载...")
                
                # 删除
                del_req = DeleteEntity.Request()
                del_req.name = 'my_robot'
                self.delete_cli.call_async(del_req)
                
                # 延迟重建
                spawn_req = SpawnEntity.Request()
                spawn_req.name = 'my_robot'
                spawn_req.xml = self.robot_urdf
                spawn_req.initial_pose.position = pos
                spawn_req.initial_pose.orientation.w = 1.0
                self.spawn_cli.call_async(spawn_req)
        
        future.add_done_callback(cb)

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = LivoxTeleop()
    
    # 在后台运行 ROS 线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    print(msg)
    try:
        while rclpy.ok():
            key = getKey(settings)
            if key in moveBindings:
                x, y, z = moveBindings[key]
                node.target_linear_x = x * node.speed
                node.target_linear_y = y * node.speed
                node.target_angular_z = z * 1.2 # 旋转速度系数
            elif key == ' ':
                node.stop()
            elif key == 'w':
                node.speed += 0.1
                print(f"\r当前速度: {node.speed:.1f}", end='')
            elif key == 's':
                node.speed = max(0.1, node.speed - 0.1)
                print(f"\r当前速度: {node.speed:.1f}", end='')
            elif key == 'r':
                node.reset_model()
            elif key == '\x03':
                break
    finally:
        node.stop()
        node.publish_commands()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()