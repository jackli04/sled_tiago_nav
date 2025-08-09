#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def move_to_goal(x_goal, y_goal, yaw_goal):
    # Initialize the action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")

    # Wait until the action server has started up
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Create a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()

    # Set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Define the goal position
    goal.target_pose.pose.position.x = x_goal
    goal.target_pose.pose.position.y = y_goal
    goal.target_pose.pose.position.z = 0.0

    # Convert yaw angle to quaternion
    quaternion = quaternion_from_euler(0.0, 0.0, yaw_goal)

    # Define the goal orientation
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Sending goal location ...")
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()

    # Check the result
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("You have reached the destination")
    else:
        rospy.loginfo("The robot failed to reach the destination")

if __name__ == '__main__':
    try:
        # roslaunch tiago_2dnav_gazebo tiago_mapping.launch public_sim:=true
        # rotate 360 degrees to get more occupancy grid
        # then run this script
        rospy.init_node('move_base_client_py', anonymous=False)
        print("Enter goal point:\n")
        while not rospy.is_shutdown():
            try: 
                sx = input("x = ").strip()
                if sx == "": break
                x = float(sx)
                sy = input("y = ").strip()
                if sy == "": break
                y = float(sy)
                st = input("theta = ").strip()
                if st == "": break
                theta = float(st)
            except ValueError:
                print("Please enter again")
                continue
            
            move_to_goal(x, y, theta)
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")