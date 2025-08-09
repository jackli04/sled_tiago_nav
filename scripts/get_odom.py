#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

def odometry_callback(msg):
    # Extract pose information
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Extract twist (velocity) information
    linear = msg.twist.twist.linear
    angular = msg.twist.twist.angular

    # # Print the data
    # rospy.loginfo(f"Position:\n x={position.x}\n y={position.y}\n z={position.z}")
    # rospy.loginfo(f"Orientation:\n x={orientation.x}\n y={orientation.y}\n z={orientation.z}\n w={orientation.w}")
    # rospy.loginfo(f"Linear Velocity:\n x={linear.x}\n y={linear.y}\n z={linear.z}")
    # rospy.loginfo(f"Angular Velocity:\n x={angular.x}\n y={angular.y}\n z={angular.z}")
    text = (f"Position:\n x={position.x}\n y={position.y}\n z={position.z}\n"
            f"Orientation:\n x={orientation.x}\n y={orientation.y}\n z={orientation.z}\n w={orientation.w}\n"
            f"Linear Velocity:\n x={linear.x}\n y={linear.y}\n z={linear.z}\n"
            f"Angular Velocity:\n x={angular.x}\n y={angular.y}\n z={angular.z}\n")
    rospy.loginfo_throttle(5,text)

def listener():
    rospy.init_node('odometry_listener', anonymous=True)

    # Subscribe to the odometry topic (replace '/odom' with your specific topic if different)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
