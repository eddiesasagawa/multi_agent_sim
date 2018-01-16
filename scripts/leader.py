#!/usr/bin/env python
import sys
import json
import numpy as np

import rospy

import tf
import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from abstractions.abstract_node import AbstractNode

from multi_agent_sim.srv import *

class LeaderControl(AbstractNode):
    '''
    @note use context manager to utilize log

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

        ### ROS Stuff ###
        # Load from parameter server
        self.robot_name                 = rospy.get_param('robot_name')

        self.laserscan_topic            = rospy.get_param('~laserscan_topic')
        self.proc_rate                  = rospy.get_param('~processing_rate') #10Hz
        self.logfile_name               = rospy.get_param('~logfile_name', default='leader.log')

        self.max_vel                    = rospy.get_param('~max_velocity', default=10.0) # m/s
        self.max_angular_vel            = rospy.get_param('~max_ang_vel', default=10.0) # m/s
        self.waypoint_transition_radius = rospy.get_param('~waypoint_transition_radius', default=1.0) # m
        self.kp_turning                 = rospy.get_param('~kp_turning', default=1.0)
        self.kp_straight                = rospy.get_param('~kp_straight', default=1.0)

        # publish twist commands to move bot
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('current_path', Marker, queue_size=10, latch=True)

        # get TF data?\
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # get frontier points
        self.frontier_sub = rospy.Subscriber('frontiers', Marker, self.callback_frontiers)

        # get access to Service
        rospy.wait_for_service('RequestPath')
        self.request_path = rospy.ServiceProxy('RequestPath', RequestPath)

        # processing rateW
        rospy.loginfo("setting processing rate to {}".format(self.proc_rate))
        self.rate = rospy.Rate(self.proc_rate) 

        # frame names
        self.frame_baselink = 'base_link_{}'.format(self.robot_name)
        self.frame_odom = 'odom_{}'.format(self.robot_name)
        self.frame_map = 'map_{}'.format(self.robot_name)

        ### Internal ###
        self.log = None

        self.frontier_pts = []
        self.active_goal = None
        self.current_path = []
        self.state = [0,0,0]

    def __enter__(self):
        rospy.loginfo("Starting log file at {}".format(self.logfile_name))
        self.log = open(self.logfile_name, 'w')
        self.log.write("{'data': [")
        return self

    def __exit__(self, exc_type, exc_val, traceback):
        self.log.write("]}")
        self.log.close()
        rospy.loginfo("Log file closed")

    def log_entry(self, entry):
        if self.log is not None:
            self.log.write(json.dumps(entry))
            self.log.write(',')

    def control(self):
        '''
        Control action for determining motion command. 

        Currently just a simple turn-n-drive controller, but ultimately would like to generate a proper pos/vel trajectory for the robot to follow
        '''
        if self.active_goal is None:
            # stay still until we have a destination
            self.publish_motion_cmd(0, 0)
            self.pick_new_goal()
            return

        # get current pose in map
        try:
            tf_baselink_map = self.tf_buffer.lookup_transform(self.frame_baselink, self.frame_map, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # skip 
            rospy.logwarn("Couldn't find TF from baselink to map")
            return

        # compare to current way point
        rpy = tf.transformations.euler_from_quaternion([tf_baselink_map.transform.rotation.x, tf_baselink_map.transform.rotation.y, tf_baselink_map.transform.rotation.z, tf_baselink_map.transform.rotation.w])

        self.state = np.array([tf_baselink_map.transform.translation.x, tf_baselink_map.transform.translation.y, rpy[-1]])

        current_pos = self.state[:2]
        next_pt = np.array(self.current_path[-1]) #path is reversed
        dist_to_pt = np.linalg.norm(next_pt-current_pos)

        while dist_to_pt < self.waypoint_transition_radius:
            # pop last point on path
            del self.current_path[-1]
            # if path is empty, we've reached the goal
            if len(self.current_path) == 0:
                rospy.loginfo("reached goal")
                self.active_goal = None
                self.publish_motion_cmd(0,0)
                self.pick_new_goal()
                return
            else:
                next_pt = np.array(self.current_path[-1]) #path is reversed
                dist_to_pt = np.linalg.norm(next_pt-current_pos)

        ### CONTROL LAW ###
        # EXTREMELY NAIVE -- needs update later.
        # also needs some local avoidance rules..

        # For now, just turn first before driving forwards
        tgt_heading = np.arctan2((next_pt[1]), (next_pt[0]))

        rpy = tf.transformations.euler_from_quaternion([tf_baselink_map.transform.rotation.x, tf_baselink_map.transform.rotation.y, tf_baselink_map.transform.rotation.z, tf_baselink_map.transform.rotation.w])
        current_heading = self.state[-1]

        e_theta = tgt_heading - current_heading # NEED TO CHECK RANGES MATCH ([PI, -PI] or [0, 2PI])
        e_2d = dist_to_pt # want to drive distance to 0

        if e_theta > 0.1:
            # turn in place
            self.publish_motion_cmd(0, e_theta * self.proc_rate * self.kp_turning)

        elif e_2d > self.waypoint_transition_radius:
            # drive forwards
            self.publish_motion_cmd(e_2d * (self.proc_rate) * self.kp_straight, 0)

        else:
            # we've reached our goal, so stop and mark
            self.active_goal = None
            self.publish_motion_cmd(0,0)
            self.pick_new_goal()

        self.publish_path_markers()

        ### Logging ###
        self.log_entry({
            'control': {
                'rostime': rospy.get_time(),
                'simtime': {'secs': tf_baselink_map.header.stamp.secs, 'nsecs': tf_baselink_map.header.stamp.nsecs},
                'tf_baselink_map': {
                    'x': current_pos[0],
                    'y': current_pos[1],
                    'theta': current_heading
                },
                'tf_target_map': {
                    'x': next_pt[0],
                    'y': next_pt[1],
                    'theta': tgt_heading
                }
            }
        })
        
    def publish_motion_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        # self.cmd_pub.publish(cmd)

    def publish_path_markers(self):
        wp_marker = Marker()
        wp_marker.header.frame_id = self.frame_map
        wp_marker.header.stamp = rospy.Time.now()
        wp_marker.ns = self.robot_name + "_active_path"
        wp_marker.id = 1
        wp_marker.frame_locked = True
        wp_marker.type = wp_marker.SPHERE
        wp_marker.action = wp_marker.ADD
        wp_marker.pose.position.x = self.current_path[-1][0]
        wp_marker.pose.position.y = self.current_path[-1][1]
        wp_marker.pose.orientation.w = 1.0
        wp_marker.scale.x = 0.4
        wp_marker.scale.y = 0.4
        wp_marker.scale.z = 0.4
        wp_marker.color.a = 0.5
        wp_marker.color.r = 1.0
        wp_marker.lifetime = rospy.Duration()
        self.path_pub.publish(wp_marker)

        marker = Marker()
        marker.header.frame_id = self.frame_map
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.robot_name + "_active_path"
        marker.id = 0
        marker.frame_locked = True
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        marker.points = [Point(x=x, y=y) for x, y in self.current_path[::-1]]

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.color.a = 0.5
        marker.color.g = 1.0

        marker.lifetime = rospy.Duration()
        
        self.path_pub.publish(marker)

    def pick_new_goal(self):
        if self.frontier_pts:
            # pick the closest
            dist_to_pts = [([p.x, p.y], np.linalg.norm([p.x-self.state[0], p.y-self.state[1]])) for p in self.frontier_pts]
            self.active_goal = min(dist_to_pts, key=lambda p: p[-1])[0]
            rospy.loginfo("set new goal: {}".format(self.active_goal))

            path_req = RequestPathRequest()
            path_req.start.position.x = self.state[0]
            path_req.start.position.y = self.state[1]
            path_req.end.position.x = self.active_goal[0]
            path_req.end.position.y = self.active_goal[1]
            path_to_goal = self.request_path(path_req)

            if path_to_goal.is_found:
                self.current_path = [(p.pose.position.x, p.pose.position.y) for p in path_to_goal.path.poses]
                self.current_path = self.current_path[::-1] # reverse the path so goal is at 0
                rospy.loginfo("found new path")

    def callback_frontiers(self, msg):
        '''
        For now, just save out current list of frontiers.
        Eventually, I should do something a little more robust such as comparing frontiers with blacklisted areas, etc.
        '''
        self.frontier_pts = msg.points

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.control()
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass

    def callback_pose(self, msg):
        pass

    def callback_odom(self, msg):
        pass

if __name__ == "__main__":
    # with LeaderControl() as ctrl:
    #     ctrl.spin()
    ctrl = LeaderControl()
    ctrl.spin()
