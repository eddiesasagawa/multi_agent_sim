#!/usr/bin/env python
import sys
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from abstractions.abstract_node import AbstractNode

class LeaderControl(AbstractNode):
    '''
    Published topics:
    * /rosout [rosgraph_msgs/Log] 2 publishers
    * /seggy1/cmd_vel [geometry_msgs/Twist] 1 publisher
    * /rosout_agg [rosgraph_msgs/Log] 1 publisher
    * /tf [tf/tfMessage] 1 publisher
    * /seggy1/base_scan [sensor_msgs/LaserScan] 1 publisher
    * /seggy1/pose [geometry_msgs/PoseStamped] 1 publisher
    * /seggy1/odom [nav_msgs/Odometry] 1 publisher

    Subscribed topics:
    * /seggy1/cmd_vel [geometry_msgs/Twist] 1 subscriber
    * /rosout [rosgraph_msgs/Log] 1 subscriber
    * /seggy1/pose [geometry_msgs/PoseStamped] 1 subscriber
    * /seggy1/odom [nav_msgs/Odometry] 1 subscriber

    '''
    def __init__(self):
        super(LeaderControl, self).__init__('leader')

        self.do_pub = do_pub

        ### ROS Stuff ###
        self.robot_name = rospy.get_param('robot_name')

        # publish twist commands to move bot
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # get TF data?\
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # get frontier points
        self.frontier_sub = rospy.Subscriber('frontiers', Marker, self.callback_frontiers)

        # processing rate
        self.rate = rospy.Rate(10) #10Hz

        # frame names
        self.frame_baselink = 'base_link_{}'.format(self.robot_name)
        self.frame_basefootprint = 'base_footprint_{}'.format(self.robot_name)
        self.frame_basestabilized = 'base_stabilized_{}'.format(self.robot_name)
        self.frame_odom = 'odom_{}'.format(self.robot_name)
        self.frame_map = 'map_{}'.format(self.robot_name)

        ### Internal ###
        self.frontier_pts = []
        self.active_goal = None
        self.current_path = []
        self.pose = PoseStamped()

    def control(self):
        # try:
        #     tf_baselink_odom = self.tf_buffer.lookup_transform(self.frame_baselink, self.frame_odom, rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     return
        if self.active_goal is None:
            # stay still until we have a destination
            self.publish_motion_cmd(0, 0)
            return

        # get current pose in map
        try:
            tf_baselink_map = self.tf_buffer.lookup_transform(self.frame_baselink, self.frame_map, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # skip 
            rospy.logwarn("Couldn't find TF from baselink to map")
            return

        # compare to current way point
        

    def publish_motion_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def callback_frontiers(self, msg):
        '''
        For now, just save out current list of frontiers.
        Eventually, I should do something a little more robust such as comparing frontiers with blacklisted areas, etc.
        '''
        self.frontier_pts = msg.points.copy()

        # also update active goal? 
        if self.active_goal is None:
            # pick the closest 

    def spin(self):
        try:
            while not rospy.is_shutdown():
                # do something with pub here
                # self.cmd_pub.publish(something)
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5

                if self.do_pub:
                    self.cmd_pub.publish(cmd)

                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def callback_pose(self, msg):
        pass

    def callback_odom(self, msg):
        pass

if __name__ == "__main__":
    ctrl = LeaderControl()
    ctrl.spin()
