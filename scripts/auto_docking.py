#!/usr/bin/env python3

import rospy
import numpy as np
import actionlib
import tf2_ros
from math import atan2
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatus


class Auto_Docking(object):
    def __init__(self, topic="/goal_finding/nearest_point"):
        self.target_dist = rospy.get_param("~target_dist", 0.60)
        self.frame_id    = rospy.get_param("~frame_id", "map")
        
        self.tf_buf = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
        self.sub = rospy.Subscriber(topic, Point, self._cb, queue_size=1)
        rospy.loginfo("Listening to %s ..." % topic)

        rospy.loginfo("Waiting for move_base...")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("move_base connected.")


    def _get_robot_pose_in_map(self):
        try:
            trans = self.tf_buf.lookup_transform("map", "base_link",
                                                 rospy.Time(0), rospy.Duration(1.0))
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            return robot_x, robot_y, yaw
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return
    

    def _cb(self, msg):
        rospy.loginfo("Got point: x=%.3f, y=%.3f, z=%.3f", msg.x, msg.y, msg.z)
        ox, oy = msg.x, msg.y
        rx, ry, q = self._get_robot_pose_in_map()

        v = np.array([ox - rx, oy - ry], dtype=float)
        dist = np.linalg.norm(v)
        if dist < 1e-6:
            rospy.logwarn("Obstacle point coincides with robot pose; skipping.")
            return
        u = v / dist

        # Calculate docking position
        dstar = float(self.target_dist)
        gx, gy = (ox - dstar * u[0], oy - dstar * u[1])

        yaw = atan2(oy - gy, ox - gx)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

        rospy.loginfo("Nearest obstacle:(%.3f,%.3f)  Robot:(%.3f,%.3f)  dist=%.2f",
                      ox, oy, rx, ry, dist)
        rospy.loginfo("Dock goal:(%.3f,%.3f) d*=%.2f yaw=%.1f",
                      gx, gy, dstar, np.degrees(yaw))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        rospy.loginfo("Sending goal to move_base...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached docking position!!")
        else:
            rospy.logwarn("Docking failed!")




if __name__ == "__main__":
    rospy.init_node("auto_docking")
    listener = Auto_Docking("/goal_finding/nearest_point")
    rospy.spin()