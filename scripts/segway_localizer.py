#!/usr/bin/env python
import sys
import numpy as np
import rospy

import tf
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry

from abstractions.abstract_node import AbstractNode

class Localizer(AbstractNode):
    def __init__(self):
        super(Localizer, self).__init__('localizer')

        self.robot_name = rospy.get_param('robot_name')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.callback_pose)

        self.frame_baselink = 'base_link_{}'.format(self.robot_name)
        self.frame_odom = 'odom_{}'.format(self.robot_name)

    def callback_pose(self, msg):
        try:
            tf_baselink_odom = self.tf_buffer.lookup_transform(self.frame_baselink, self.frame_odom, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        q = Quaternion()
        q.x = msg.pose.orientation.x - tf_baselink_odom.transform.rotation.x
        q.y = msg.pose.orientation.y - tf_baselink_odom.transform.rotation.y
        q.z = msg.pose.orientation.z - tf_baselink_odom.transform.rotation.z
        q.w = msg.pose.orientation.w - tf_baselink_odom.transform.rotation.w
        
        d = np.linalg.norm([q.x, q.y, q.z, q.w])

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = self.frame_odom
        t.transform.translation.x = msg.pose.position.x - tf_baselink_odom.transform.translation.x
        t.transform.translation.y = msg.pose.position.y - tf_baselink_odom.transform.translation.y
        t.transform.translation.z = msg.pose.position.z - tf_baselink_odom.transform.translation.z
        t.transform.rotation.x = q.x / d if d != 0 else 0.0
        t.transform.rotation.y = q.y / d if d != 0 else 0.0
        t.transform.rotation.z = q.z / d if d != 0 else 0.0
        t.transform.rotation.w = q.w / d if d != 0 else 0.0

        # need to normalize quaternion

        self.tf_broadcaster.sendTransform(t)

if __name__ == "__main__":
    localizer = Localizer()
    localizer.spin()
