import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Wrench
from gazebo_msgs.srv import GetEntityState, SpawnEntity, DeleteEntity
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import sys, select, termios, tty
import threading

msg = """
=============================================
         🚗 小车全能统一控制中心 🚗
=============================================

【平面移动】(长按生效，切勿按住不放)
   u    i    o      (左前) (前进) (右前)
   j    k    l      (左转) (停止) (右转)
   m    ,    .      (左后) (后退) (右后)
   a    d           (向左平移) (向右平移)

【上下悬浮】
   w : 增加向上推力 (上升)
   s : 减少向上推力 (下降)

【系统控制】
   Space 键 或 k : 🛑 紧急停止 (全部动力、推力瞬间归零)
   r 键          : 🔄 原地重载 (在当前三维位置原地重新加载满血模型)

CTRL-C 退出
=============================================
"""

moveBindings = {
    'i': (1, 0, 0),
    ',': (-1, 0, 0),
    'j': (0, 0, 1),
    'l': (0, 0, -1),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'u': (1, 0, 1),
    'o': (1, 0, -1),
    'm': (-1, 0, -1),
    '.': (-1, 0, 1),
}

class UnifiedTeleop(Node):
    def __init__(self):
        super().__init__('unified_teleop')
        
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.force_pub = self.create_publisher(Wrench, '/force_cmd', 10)
        
        # 状态读取客户端
        self.get_state_cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        
        # 生成和删除模型的客户端
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        # 订阅 robot_description 话题来保存模型 XML，QoS 策略设为 TransientLocal 以便获取历史消息
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.robot_urdf = ""
        self.urdf_sub = self.create_subscription(String, '/robot_description', self.urdf_cb, qos_profile)

        self.speed = 0.5       
        self.turn = 1.0        
        self.force_step = 1.5  

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.target_force_z = 0.0

        self.timer = self.create_timer(0.1, self.publish_commands)

    def urdf_cb(self, msg):
        self.robot_urdf = msg.data

    def publish_commands(self):
        twist = Twist()
        twist.linear.x = float(self.target_linear_x)
        twist.linear.y = float(self.target_linear_y)
        twist.angular.z = float(self.target_angular_z)
        self.vel_pub.publish(twist)

        wrench = Wrench()
        wrench.force.z = float(self.target_force_z)
        self.force_pub.publish(wrench)

    def stop_all(self):
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.target_force_z = 0.0

    def reset_pose(self):
        self.stop_all() 
        
        if not self.robot_urdf:
            print("\n[错误] 尚未获取到模型文件，无法重载。请确保 robot_state_publisher 正在运行。")
            return

        if not self.get_state_cli.wait_for_service(timeout_sec=1.0):
            print("\n[错误] 无法连接到 Gazebo 的状态服务。")
            return

        req = GetEntityState.Request()
        req.name = 'chassis'
        future = self.get_state_cli.call_async(req)
        future.add_done_callback(self.get_state_cb)

    def get_state_cb(self, future):
        try:
            res = future.result()
            if res.success:
                state = res.state
                x = state.pose.position.x
                y = state.pose.position.y
                z = state.pose.position.z
                
                # 输出当前的三维坐标
                print(f"\n[坐标记录] 重载位置 -> X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")
                print("[模型重载] 正在销毁错乱模型并重新生成...")

                # 步骤 1: 发起删除现有车体的请求
                del_req = DeleteEntity.Request()
                del_req.name = 'chassis'
                del_fut = self.delete_cli.call_async(del_req)

                # 步骤 2: 删除完成后，在原坐标生成全新的车体
                def spawn_cb(f):
                    spawn_req = SpawnEntity.Request()
                    spawn_req.name = 'chassis'
                    spawn_req.xml = self.robot_urdf
                    
                    # 仅保留坐标，朝向 (orientation) 默认设为正向 (w=1.0)
                    spawn_req.initial_pose.position.x = x
                    spawn_req.initial_pose.position.y = y
                    spawn_req.initial_pose.position.z = z
                    spawn_req.initial_pose.orientation.x = 0.0
                    spawn_req.initial_pose.orientation.y = 0.0
                    spawn_req.initial_pose.orientation.z = 0.0
                    spawn_req.initial_pose.orientation.w = 1.0 
                    
                    self.spawn_cli.call_async(spawn_req)
                    print("[成功] 小车已在该位置原地满血复活！\n")

                del_fut.add_done_callback(spawn_cb)

            else:
                print("\n[警告] 未能在 Gazebo 中找到 'chassis' 模型。")
        except Exception as e:
            print(f"\n[错误] 重载位姿失败: {e}")

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    
    node = UnifiedTeleop()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(msg)
    try:
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x, y, th = moveBindings[key]
                node.target_linear_x = x * node.speed
                node.target_linear_y = y * node.speed
                node.target_angular_z = th * node.turn
                print(f"\r[移动] x: {node.target_linear_x}, y: {node.target_linear_y}, 旋转: {node.target_angular_z}      ", end='')
                
            elif key == 'w':
                node.target_force_z += node.force_step
                print(f"\r[推力] 当前向上悬浮力: {node.target_force_z}                   ", end='')
                
            elif key == 's':
                node.target_force_z -= node.force_step
                print(f"\r[推力] 当前向上悬浮力: {node.target_force_z}                   ", end='')
                
            elif key == ' ' or key == 'k':
                node.stop_all()
                print("\r[🛑紧急停止] 动力归零！                                       ", end='')
                
            elif key == 'r':
                print("\r[🔄原地重载] 正在获取当前位置并重新加载模型...                   ", end='')
                node.reset_pose()
                
            elif key == '\x03':  
                break

    except Exception as e:
        print(e)
    finally:
        node.stop_all()
        node.publish_commands()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()