from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
def main():
    rclpy.init()
    navigator = BasicNavigator()
    # Wait for navigation to start
    navigator.waitUntilNav2Active()
    # Set the goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    # Send the goal and receive feedback
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(
            f'  Estimated time remaining: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s'
        )
        # Timeout to cancel
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()
            # Final result judgement
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('  Navigation result: Succeeded')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('  Navigation result: Canceled')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('  Navigation result: Failed')
    else:
        navigator.get_logger().error('  Navigation result: Invalid status')
        rclpy.shutdown()
