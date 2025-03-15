#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait until the user provides an initial pose
    print("Waiting for initial pose estimate...")
    while not navigator.initial_pose_received:
        rclpy.spin_once(navigator)

    print("Initial pose received. Proceeding with navigation.")
    
    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()

    # Define new goal waypoints
    goal_poses = []

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.5757035941455941
    goal_pose1.pose.position.y = 1.6022682954095302
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.7535552300505937
    goal_pose2.pose.position.y = -0.20064095013680686
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -0.5654100470395151
    goal_pose3.pose.position.y = -0.18586310960695732
    goal_pose3.pose.orientation.w = 1.0
    goal_poses.append(goal_pose3)

    # Start waypoint navigation
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}')
            now = navigator.get_clock().now()

            # Timeout logic
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

    # Check task result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
