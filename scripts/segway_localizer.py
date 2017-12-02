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

        self.pose_sub = rospy.Subscriber('pose_truth', PoseStamped, self.callback_pose)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom)

        self.frame_baselink = 'base_link_{}'.format(self.robot_name)
        self.frame_basefootprint = 'base_footprint_{}'.format(self.robot_name)
        self.frame_basestabilized = 'base_stabilized_{}'.format(self.robot_name)
        self.frame_odom = 'odom_{}'.format(self.robot_name)
        self.frame_map = 'map_{}'.format(self.robot_name)

    def unpack_quaternion_msg(self, msg):
        return (msg.x, msg.y, msg.z, msg.w)

    def callback_pose(self, msg):
        '''
        eventually provide ground truth transformation between maps, but ignore for now..
        '''
        pass
        # try:
        #     tf_baselink_odom = self.tf_buffer.lookup_transform(self.frame_baselink, self.frame_odom, rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     return

        # # q_tf * q_o = q_bl --> q_tf = q_bl * q_o.inv
        # # tf.transformations.quaternion_multiply()

        # q = Quaternion()
        # q.x = msg.pose.orientation.x - tf_baselink_odom.transform.rotation.x
        # q.y = msg.pose.orientation.y - tf_baselink_odom.transform.rotation.y
        # q.z = msg.pose.orientation.z - tf_baselink_odom.transform.rotation.z
        # q.w = msg.pose.orientation.w - tf_baselink_odom.transform.rotation.w
        # # q.normalize()
        # d = np.linalg.norm([q.x, q.y, q.z, q.w])

        # t = TransformStamped()
        # t.header.stamp = msg.header.stamp
        # t.header.frame_id = 'map'
        # t.child_frame_id = self.frame_map
        # t.transform.translation.x = msg.pose.position.x - tf_baselink_odom.transform.translation.x
        # t.transform.translation.y = msg.pose.position.y - tf_baselink_odom.transform.translation.y
        # t.transform.translation.z = msg.pose.position.z - tf_baselink_odom.transform.translation.z
        # t.transform.rotation.x = q.x / d if d != 0 else 0.0
        # t.transform.rotation.y = q.y / d if d != 0 else 0.0
        # t.transform.rotation.z = q.z / d if d != 0 else 0.0
        # t.transform.rotation.w = q.w / d if d != 0 else 0.0

        # self.tf_broadcaster.sendTransform(t)

    def callback_odom(self, msg):
        '''
        Augment the Odom TF publishing by publishing TFs to base_footprint and base_stabilized, per REP105
        '''
        rpy = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        # base_footprint in odom frame
        q_o2bf = tf.transformations.quaternion_from_euler(0.0, 0.0, rpy[2])

        t_o2bf = TransformStamped()
        t_o2bf.header.stamp = msg.header.stamp
        t_o2bf.header.frame_id = self.frame_odom
        t_o2bf.child_frame_id = self.frame_basefootprint
        t_o2bf.transform.translation.x = msg.pose.pose.position.x
        t_o2bf.transform.translation.y = msg.pose.pose.position.y
        t_o2bf.transform.translation.z = 0
        t_o2bf.transform.rotation.x = q_o2bf[0]
        t_o2bf.transform.rotation.y = q_o2bf[1]
        t_o2bf.transform.rotation.z = q_o2bf[2]
        t_o2bf.transform.rotation.w = q_o2bf[3]
        self.tf_broadcaster.sendTransform(t_o2bf)

        # base_stabilized in base_footprint frame
        # no rotation, just translation to CG
        t_bf2bs = TransformStamped()
        t_bf2bs.header.stamp = msg.header.stamp
        t_bf2bs.header.frame_id = self.frame_basefootprint
        t_bf2bs.child_frame_id = self.frame_basestabilized
        t_bf2bs.transform.translation.x = 0
        t_bf2bs.transform.translation.y = 0
        t_bf2bs.transform.translation.z = msg.pose.pose.position.z
        t_bf2bs.transform.rotation.x = 0
        t_bf2bs.transform.rotation.y = 0
        t_bf2bs.transform.rotation.z = 0
        t_bf2bs.transform.rotation.w = 1
        self.tf_broadcaster.sendTransform(t_bf2bs)

        # base_link in base_stabilized frame
        # only contain roll and pitch information in this transformation
        q_bs2bl = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], 0.0)

        t_bs2bl = TransformStamped()
        t_bs2bl.header.stamp = msg.header.stamp
        t_bs2bl.header.frame_id = self.frame_basestabilized
        t_bs2bl.child_frame_id = self.frame_baselink
        t_bs2bl.transform.translation.x = 0
        t_bs2bl.transform.translation.y = 0
        t_bs2bl.transform.translation.z = 0
        t_bs2bl.transform.rotation.x = q_bs2bl[0]
        t_bs2bl.transform.rotation.y = q_bs2bl[1]
        t_bs2bl.transform.rotation.z = q_bs2bl[2]
        t_bs2bl.transform.rotation.w = q_bs2bl[3]
        self.tf_broadcaster.sendTransform(t_bs2bl)

if __name__ == "__main__":
    localizer = Localizer()
    localizer.spin()
