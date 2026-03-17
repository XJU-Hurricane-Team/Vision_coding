import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# =======================================================
# 🚗 [快捷修改区] 请在这里选择你要去的目标点编号！
# =======================================================
# 选择你要前往的目标点 (填 1, 2, 3 或 4)
SELECTED_GOAL = 1

# 定义 4 个预设目标点，格式为: {编号: [X, Y, 姿态qz, 姿态qw]}
PRESET_GOALS = {
    1: [ -4.105,  -2.867, -0.005, 1.000],  # 目标点 1 (⚠️ 请替换为实际值)
    2: [-2.000,  3.500, 0.707, 0.707],  # 目标点 2 (⚠️ 请替换为实际值)
    3: [ 4.200, -1.000, 1.000, 0.000],  # 目标点 3 (⚠️ 请替换为实际值)
    4: [ 0.000,  0.000, 0.000, 1.000],  # 目标点 4 (⚠️ 请替换为实际值)
}
# =======================================================

class PresetNavNode(Node):
    def __init__(self):
        super().__init__('preset_nav_node')
        
        # 1. 创建 NavigateToPose 动作客户端
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. 读取快捷修改区的目标点
        self.target_x, self.target_y, self.target_qz, self.target_qw = PRESET_GOALS[SELECTED_GOAL]
        self.get_logger().info(f"📍 已选择目标点 {SELECTED_GOAL}: X={self.target_x}, Y={self.target_y}")

    def send_goal(self):
        """发送导航目标点"""
        self.get_logger().info('⏳ 等待 Nav2 导航服务就绪...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(self.target_x)
        goal_msg.pose.pose.position.y = float(self.target_y)
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(self.target_qz)
        goal_msg.pose.pose.orientation.w = float(self.target_qw)

        self.get_logger().info('🚀 正在发送目标点，小车开始移动...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'📉 距离目标点还有: {feedback.distance_remaining:.2f} 米', throttle_duration_sec=2.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标点被 Nav2 拒绝！(可能是目标点在障碍物上或路径不可达)')
            return

        self.get_logger().info('✅ 目标点已被接受，正在前往...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4: # SUCCEEDED
            self.get_logger().info('🎉 成功到达目标点！')
        else:
            self.get_logger().warn(f'⚠️ 导航未成功完成，状态码: {status}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PresetNavNode()
    node.send_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('被用户中断')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()