from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    # create waypoint set 252
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id ='map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id ='map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id ='map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 2.0
    goal_pose3.pose.position.y = 2.0
    goal_pose3.pose.orientation.w = 1.0
    goal_poses.append(goal_pose3)
    # call waypoint navigation service
    navigator.followWaypoints(goal_poses)
    # check for completion and get feedback
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f' Current waypoint: {feedback.current_waypoint}')
    # final result determination
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(' Navigation result: succeeded')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn(' Navigation result: canceled')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error(' Navigation result: failed')
    else:
        navigator.get_logger().error(' Navigation result: invalid return status')

