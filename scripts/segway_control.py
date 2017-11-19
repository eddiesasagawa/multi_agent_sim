#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

class PositionController(object):
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
    def __init__(self, do_pub = True):
        rospy.init_node('controller')

        self.do_pub = do_pub

        ### ROS Stuff ###
        if self.do_pub:
            self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        else:
            self.cmd_pub = None

        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.callback_pose)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.callback_odom)

        self.rate = rospy.Rate(10) #10Hz

        ### Internal ###
        self.pose = None
        self.odom = None

        

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


# def pose_callback(msg):
#     pose = msg.pose
#     rospy.loginfo("Received: {}, {}, {}".format(pose.position.x, pose.position.y, pose.position.z))

# def odom_callback(msg):
#     twist = msg.twist.twist
#     rospy.loginfo("Received twist: {}, {}, {}".format(twist.linear.x, twist.linear.y, twist.linear.z))

# def position_controller(namespace):
#     rospy.init_node('controller')

#     rospy.Subscriber("pose", PoseStamped, pose_callback)
#     rospy.Subscriber("odom", Odometry, odom_callback)

#     rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == '--no-pub':
        publish_cmds = False
    else:
        publish_cmds = True

    ctrl = PositionController(publish_cmds)
    ctrl.spin()
