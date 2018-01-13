#!/usr/bin/env python
import sys
import json

import rospy

import tf
import tf2_ros

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from abstractions.abstract_node import AbstractNode

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

        self.do_pub = do_pub

        ### ROS Stuff ###
        # Load from parameter server
        self.robot_name                 = rospy.get_param('robot_name')
        self.logfile_name               = rospy.get_param('logfile_name', default='leader.log')
        self.proc_rate                  = rospy.get_param('processing_rate', default=10) #10Hz

        self.max_vel                    = rospy.get_param('max_velocity', default=10.0) # m/s
        self.max_angular_vel            = rospy.get_param('max_ang_vel', default=10.0) # m/s
        self.waypoint_transition_radius = rospy.get_param('waypoint_transition_radius', default=1.0) # m
        self.kp_turning                 = rospy.get_param('kp_turning', default=1.0)
        self.kp_straight                = rospy.get_param('kp_straight', default=1.0)
        

        # publish twist commands to move bot
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # get TF data?\
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # get frontier points
        self.frontier_sub = rospy.Subscriber('frontiers', Marker, self.callback_frontiers)

        # processing rate
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
        self.pose = PoseStamped()

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
            return

        # get current pose in map
        try:
            tf_baselink_map = self.tf_buffer.lookup_transform(self.frame_baselink, self.frame_map, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # skip 
            rospy.logwarn("Couldn't find TF from baselink to map")
            return

        # compare to current way point
        current_pos = np.array([tf_baselink_map.transform.translation.x, tf_baseline_map.transform.translation.y])
        next_pt = np.array(self.current_path[-1]) #path is reversed
        dist_to_pt = np.linalg.norm(next_pt-current_pos)

        while dist_to_pt < self.waypoint_transition_radius:
            # pop last point on path
            del self.current_path[-1]
            # if path is empty, we've reached the goal
            if not self.current_path:
                self.active_goal = None
                self.publish_motion_cmd(0,0)
                return
            else:
                next_pt = np.array(self.current_path[-1]) #path is reversed
                dist_to_pt = np.linalg.norm(next_pt-current_pos)

        ### CONTROL LAW ###
        # For now, just turn first before driving forwards
        tgt_heading = np.arctan2((next_pt[1]), (next_pt[0]))

        rpy = tf.transformations.euler_from_quaternion([tf_baselink_map.transform.rotation.x, tf_baselink_map.transform.rotation.y, tf_baselink_map.transform.rotation.z, tf_baselink_map.transform.rotation.w])
        current_heading = rpy[-1] # yaw for 2d 

        e_theta = tgt_heading - current_heading # NEED TO CHECK RANGES MATCH ([PI, -PI] or [0, 2PI])
        e_2d = dist_to_pt # want to drive distance to 0

        


        ### Logging ###
        self.log_entry({
            'control': {
                'rostime': rospy.get_time(),
                'simtime': {'secs': tf_baselink_map.header.stamp.secs, 'nsecs': tf_baselink_map.header.stamp.nsecs},
                'tf_baselink_map': 
                    'x': current_pos[0],
                    'y': current_pos[1]
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
    with LeaderControl() as ctrl:
        ctrl.spin()
