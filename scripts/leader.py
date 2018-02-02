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

class Path(object):
    def __init__(self, pathlist, waypoint_radius):
        '''
        @param pathlist list of points along path, with 0 being at start
        '''
        self.wp_radius = waypoint_radius

        self.start = pathlist[0]
        self.end = pathlist[-1]
        self.path = pathlist
        self.wp_count = len(pathlist)
        
        self.link_lengths = self.calculate_link_lengths(self.path)
        
        self.set_waypoint(1) # waypoint index, starting at one (skip starting point)

        self.finished = False
        
    def calculate_link_lengths(self, pathlist):
        ''' 
        Pre-calculate inter-waypoint distances
        Each element will be the distance to the next waypoint, ie. dist(wp_i, wp_i+1)
        To maintain same length as the path, there will be a zero element at the end
        '''
        return [np.linalg.norm(wp_nx - wp_i) for wp_i, wp_nx in zip(pathlist[:-1], pathlist[1:])] + [0.0]

    def set_waypoint(self, idx):
        '''
        Calculate waypoint metrics associated with waypoint #<idx> 

        If idx is out-of-bounds, let it throw an exception as we process it
        '''
        self.wp_idx = idx
        self.wp_current = self.path[self.wp_idx]
        self.wp_prev = self.path[self.wp_idx-1]
        self.wp_next = None if self.wp_idx >= (self.wp_count - 1) else self.path[self.wp_idx + 1]
        self.next_link_length = self.link_lengths[self.wp_idx]

    def advance_waypoint(self):
        '''
        wrapper for set_waypoint
        '''
        self.set_waypoint(self.wp_idx + 1)
        
    def prune(self, pos):
        '''
        @brief Remove path points within radius of pos

        We want to advance the waypoint if we pass it even a little bit
        To achieve this, we advance the waypoint when:
        1. We are within acceptance radius of current waypoint
        2. (TBD) Closer to the next waypoint than the previous waypoint
            - Idea is that if we overshoot the current waypoint, we might as well start navigating to the next one
                However, this is not robust to needing to navigate around walls or corners.
                Also if the next waypoint is just that much closer to the current than the previous, we might advance too early.
                    This could be remedied by dividing by the link length of waypoints to get percentage comparisons instead of distance.

        Since this is intended to be called regularly, we just advance only once even if we could technically skip a bunch of waypoints.      
        '''
        if np.linalg.norm(self.wp_current - pos) < self.wp_radius:
            if self.wp_current == self.end:
                self.finished = True
            else:
                self.advance_waypoint()


    def centerline_error(self, pos):
        '''
        Calculate the shortest distance from pos to the line to current waypoint from previous.
        ie. deviation from ideal path

        Method:
        line from point A to B (line AB), find shortest dist to point C, treat as infinite line (ie. don't clamp to just line ssegment)
        parameterize line as P = A + t * (B - A), where t is some scalar value
        Then project line AC to AB to get value of t
        t = [AC . AB] / ||AB||^2

        '''
        pA = np.array(self.path[self.wp_idx-1])
        pB = np.array(self.path[self.wp_idx])
        pC = np.array(pos)

        AB = pB - pA
        AC = pC - pA

        projAC = np.dot(AB, AC) / np.dot(AB, AB) # square of norm is just sum of squares
        pP = pA + projAC * (AB)

        return np.linalg.norm(pP - pC)

    def update(self, state):
        '''
        given current state (x, y, theta), calculate errors in heading and centerline
        Also update current waypoint

        state is a 3-element array of x, y, theta in map frame
        '''
        pos = state[:2]
        heading = state[-1]

        ## First, prune the waypoints to make sure we are navigating to the right one
        self.prune(pos)

        ## Get centerline error
        e_centerline = self.centerline_error(pos)

        ## Get heading error
        tgt_heading = np.arctan2((self.wp_current[1]), (self.wp_current[0]))
        e_heading = tgt_heading - heading

        ## Get distance to waypoint
        e_dist = np.linalg.norm(self.wp_current-pos)

        return e_centerline, e_heading, e_dist

    @property
    def current_waypoint(self):
        '''
        return current waypoint
        '''
        return self.wp_current
    
    @property
    def reached_goal(self):
        '''
        return true if end is reached.
        '''
        return self.finished

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
        self.path = None
        self.state = [0,0,0]
        self.cmd = Twist()

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

        e_centerline, e_heading, e_dist = self.path.update(self.state)

        if self.path.reached_goal:
            self.active_goal = None
            self.publish_motion_cmd(0,0)
            self.pick_new_goal()

        else:
            ### CONTROL LAW ###
            # EXTREMELY NAIVE -- needs update later.
            # also needs some local avoidance rules..

            # for now, just turn first before driving straight
            if e_heading > 0.1:
                # turn in place
                self.publish_motion_cmd(0, e_heading * self.proc_rate * self.kp_turning)

            else:
                # drive straight
                self.publish_motion_cmd(e_dist * self.proc_rate * self.kp_straight, 0)

        ### Visualization / Logging ###
        self.publish_path_markers()

        self.log_entry({
            'control': {
                'rostime': rospy.get_time(),
                'simtime': {'secs': tf_baselink_map.header.stamp.secs, 'nsecs': tf_baselink_map.header.stamp.nsecs},
                'tf_baselink_map': {
                    'x': self.state[0],
                    'y': self.state[1],
                    'theta': self.state[2]
                },
                'tf_target_map': {
                    'x': self.path.current_waypoint[0],
                    'y': self.path.current_waypoint[1],
                },
                'e_centerline': e_centerline,
                'e_heading': e_heading,
                'e_dist': e_dist,
                'command': {
                    'v': self.cmd.linear.x,
                    'w': self.cmd.angular.z,
                },
            }
        })
        
    def publish_motion_cmd(self, v, w):
        self.cmd = Twist()
        self.cmd.linear.x = v
        self.cmd.angular.z = w
        # self.cmd_pub.publish(self.cmd)

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
                # self.current_path = self.current_path[::-1] # reverse the path so goal is at 0
                self.path = Path(self.current_path, self.waypoint_transition_radius)
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
