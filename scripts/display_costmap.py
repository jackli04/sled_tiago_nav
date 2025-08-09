#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class CostmapPresenter(object):
    def __init__(self):
        self.costmap_topic = rospy.get_param("~costmap_topic", "/move_base/local_costmap/costmap")
        self.wait_after_sec = rospy.get_param("~wait_after_sec", 0.5)
        self.trigger_immediately = rospy.get_param("~trigger_immediately", False)

        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/local_costmap/image", Image, queue_size=1, latch=True)

        if self.trigger_immediately:
            rospy.loginfo("Arrived at the goal. Capturing costmap...")
            rospy.Timer(rospy.Duration(self.wait_after_sec), self._capture_costmap, oneshot=True)
        else:
            rospy.Subscriber("/move_base/result", MoveBaseActionResult, self._result_callback)
            # rospy.loginfo("Moving base...")

    def _result_callback(self, msg):
        if msg.status.status == 3:  # SUCCEEDED
            rospy.loginfo("Arrived at the goal. Capturing costmap...")
            rospy.Timer(rospy.Duration(self.wait_after_sec), self._capture_costmap, oneshot=True)

    def _capture_costmap(self, _evt):
        try:
            grid = rospy.wait_for_message(self.costmap_topic, OccupancyGrid, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("Timeout waiting for costmap!")
            return

        w, h = grid.info.width, grid.info.height
        data = np.array(grid.data, dtype=np.int16).reshape(h, w)

        img = np.full((h, w), 127, dtype=np.uint8)
        img[data == 0] = 255
        img[data >= 100] = 0

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = grid.header.frame_id
        msg_img = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        msg_img.header = header
        self.img_pub.publish(msg_img)

        rospy.loginfo("Costmap published on /local_costmap/image")
        rospy.signal_shutdown("done")

if __name__ == "__main__":
    rospy.init_node("costmap_present")
    CostmapPresenter()

    rospy.signal_shutdown("done")
