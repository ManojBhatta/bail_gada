#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math

def distance(pose1, pose2):
    """Calculate Euclidean distance between two poses"""
    return math.sqrt((pose1.position.x - pose2.position.x) ** 2 + 
                    (pose1.position.y - pose2.position.y) ** 2)

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    
    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()
    
    # Define goal waypoints
    goal_poses = []
    target_positions = [
        (1.5757, 1.6023),
        (1.7536, -0.2006),
        (-0.5654, -0.1859)
    ]
    
    for x, y in target_positions:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        goal_poses.append(goal_pose)
    
    # For periodic status updates
    print_count = 0
    
    # Navigate through each waypoint individually
    current_waypoint = 0
    while current_waypoint < len(goal_poses):
        goal_pose = goal_poses[current_waypoint]
        print(f'Navigating to waypoint {current_waypoint+1}/{len(goal_poses)}: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})')
        
        # Navigate to the current waypoint
        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            # Get feedback which includes current pose
            feedback = navigator.getFeedback()
            
            if feedback:
                # Check if we're close enough to the goal
                current_pose = feedback.current_pose.pose
                dist = distance(current_pose, goal_pose.pose)
                
                # If we're within 0.05m of the goal, move to the next waypoint
                if dist < 0.05:
                    print(f'Reached waypoint {current_waypoint+1} (distance: {dist:.4f}m)')
                    navigator.cancelTask()
                    break
                
                # Print distance update every ~5 iterations instead of using Duration
                print_count += 1
                if print_count % 5 == 0:
                    print(f'Current distance to goal: {dist:.4f}m')
        
        # Check result and proceed to next waypoint if successful or close enough
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED or dist < 0.05:
            print(f'Waypoint {current_waypoint+1} completed successfully!')
            current_waypoint += 1
        else:
            print(f'Failed to reach waypoint {current_waypoint+1}, retrying...')
            # Optional: add retry limit here
    
    print('All waypoints completed!')
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()