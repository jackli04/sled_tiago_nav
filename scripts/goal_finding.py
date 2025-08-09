#!/usr/bin/env python


import rospy
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped


class GoalFinderSimple(object):
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/local_costmap/image")
        self.costmap_topic = rospy.get_param("~costmap_topic", "/move_base/local_costmap/costmap")
        self.wait_for_image = rospy.get_param("~wait_for_image", True)
        self.wait_timeout_sec = rospy.get_param("~wait_timeout_sec", 60.0)
        self.fov_deg = rospy.get_param("~fov_deg", 120.0)

        # TF
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # Obstacle publisher
        self.nearest_pub = rospy.Publisher("/goal_finding/nearest_point", Point, queue_size=1)

    def wait_costmap(self):
        if self.wait_for_image:
            try:
                rospy.wait_for_message(self.image_topic, Image, timeout=self.wait_timeout_sec)
                rospy.loginfo("Costmap image received, starting obstacle finding...")
            except rospy.ROSException:
                rospy.logwarn("goal_finding: Timeout waiting for costmap image")

    def find_goal(self):
        # Get costmap
        try:
            grid = rospy.wait_for_message(self.costmap_topic, OccupancyGrid, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("No costmap received.")
            return

        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = np.array(grid.data, dtype=np.int16).reshape(h, w)

        # Get robot tf
        try:
            trans = self.tf_buf.lookup_transform("map", "base_link",
                                                 rospy.Time(0), rospy.Duration(1.0))
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return
        
        # # ------------------- Debug -------------------
        # print("------------------- Debug -------------------\n")
        # print("Yaw = ", np.degrees(yaw),"\n")
        # print("x = ", robot_x, "y = ", robot_y, "\n")
        # print("------------------- Debug -------------------\n")
        # # ------------------- Debug -------------------


        # Convert costmap obsticals to tf coords
        occ_idx = np.argwhere(data >= 100)  # Only obstacles
        if occ_idx.size == 0:
            rospy.logerr("No obstacles near the robot!!!")
            return

        oq = grid.info.origin.orientation
        _, _, origin_yaw = euler_from_quaternion([oq.x, oq.y, oq.z, oq.w])
        cos_o = np.cos(origin_yaw)
        sin_o = np.sin(origin_yaw)

        costmap_obs = []
        for r, c in occ_idx:
            x_local = (c + 0.5) * res
            y_local = (r + 0.5) * res
            x_rot = cos_o * x_local - sin_o * y_local
            y_rot = sin_o * x_local + cos_o * y_local
            wx = origin_x + x_rot
            wy = origin_y + y_rot
            costmap_obs.append([wx, wy])
        costmap_obs = np.array(costmap_obs)

        costmap_frame = grid.header.frame_id
        if costmap_frame != "map":
            try:
                t = self.tf_buf.lookup_transform("map", costmap_frame,
                                                rospy.Time(0), rospy.Duration(1.0))
                tq = t.transform.rotation
                _, _, yaw_tf = euler_from_quaternion([tq.x, tq.y, tq.z, tq.w])
                tx = t.transform.translation.x
                ty = t.transform.translation.y

                cos_tf = np.cos(yaw_tf)
                sin_tf = np.sin(yaw_tf)

                x_old = costmap_obs[:, 0]
                y_old = costmap_obs[:, 1]
                x_new = cos_tf * x_old - sin_tf * y_old + tx
                y_new = sin_tf * x_old + cos_tf * y_old + ty
                costmap_obs = np.stack([x_new, y_new], axis=1)

            except Exception as e:
                rospy.logwarn("TF map from %s failed: %s", costmap_frame, e)
                return

        # Tf to robot
        dx = costmap_obs[:, 0] - robot_x
        dy = costmap_obs[:, 1] - robot_y

        # Rotation matrix
        rel_x = np.cos(-yaw) * dx - np.sin(-yaw) * dy
        rel_y = np.sin(-yaw) * dx + np.cos(-yaw) * dy

        # FOV filtering
        angles = np.degrees(np.arctan2(rel_y, rel_x))
        fov_mask_obstacles = (np.abs(angles) <= self.fov_deg / 2.0) & (rel_x > 0)
        available_obstacles = costmap_obs[fov_mask_obstacles]

        if available_obstacles.shape[0] == 0:
            rospy.logerr("No obstacles in valid FOV!!!")
            return

        # Find nearest obstacle
        dists = np.linalg.norm(available_obstacles - np.array([robot_x, robot_y]), axis=1)  # Euclidean distance
        nearest_idx = np.argmin(dists)
        nearest_pt = available_obstacles[nearest_idx]

        # Publish nearest obstacle
        self.nearest_pub = rospy.Publisher("/goal_finding/nearest_point", Point, queue_size=1, latch=True)
        point = Point()
        point.x = nearest_pt[0]
        point.y = nearest_pt[1]
        point.z = 0.0
        self.nearest_pub.publish(point)

        rospy.loginfo("Nearest obstacle published at (%.2f, %.2f)", nearest_pt[0], nearest_pt[1])
        rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("goal_finding")
    gf = GoalFinderSimple()
    gf.wait_costmap()
    gf.find_goal()
    rospy.loginfo("Goal founded.")
