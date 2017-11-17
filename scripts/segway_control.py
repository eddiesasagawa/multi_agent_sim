#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def pose_callback(msg):
    pose = msg.pose
    rospy.loginfo("Received: {}, {}, {}".format(pose.position.x, pose.position.y, pose.position.z))

def odom_callback(msg):
    twist = msg.twist.twist
    rospy.loginfo("Received twist: {}, {}, {}".format(twist.linear.x, twist.linear.y, twist.linear.z))

def position_controller(namespace):
    rospy.init_node('controller')

    rospy.Subscriber("pose", PoseStamped, pose_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)

    rospy.spin()

if __name__ == "__main__":
    position_controller("seggy1")
