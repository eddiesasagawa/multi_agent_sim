#!/usr/bin/env python
import sys
import numpy as np

import cv2
import rospy

import tf
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData

from abstractions.abstract_node import AbstractNode

class MapGrid2D(object):
    '''
    From nav_msgs/OccupancyGrid message:
    # The map data, in row-major order, starting with (0,0).  Occupancy
    # probabilities are in the range [0,100].  Unknown is -1.


    '''
    def __init__(self):
        self.raw_msg    = OccupancyGrid()     # OccupancyGrid message goes here
        self.map_meta   = MapMetaData()     # MapMetaData
        self.map = np.array([])
        self.map_is_loaded = False

        self.occupancy_thresh = 50 # % probability of occupancy before considering as occupied.

        cv2.namedWindow('map_im', cv2.WINDOW_NORMAL)

    def update(self, map_msg):
        rospy.loginfo('update map')
        self.raw_msg = map_msg
        self.map_meta = map_msg.info
        self.map = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height, 1))
        self.map_is_loaded = True

        self.map[self.map == -1] = 50

        self.map = self.map.astype(np.uint8)
        # temp = np.array(self.map.copy())
        
        cv2.normalize(self.map, dst = self.map, alpha = 0, beta = 255, norm_type=cv2.NORM_MINMAX)
        
    def display(self):
        if self.map_is_loaded:
            cv2.imshow('map_im', self.map)
            cv2.waitKey(10)

    def find_frontiers(self):
        pass

        # apply Sobel operator to find edges

        # iterate over edge pixels and filter for open-unknown transitions only

        # group frontier pixels
    

class FrontierExpander(AbstractNode):
    def __init__(self):
        super(FrontierExpander, self).__init__('frontier_expansion')

        self.robot_name = rospy.get_param('robot_name')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_grid = MapGrid2D()

        self.frame_baselink = 'base_link_{}'.format(self.robot_name)
        self.frame_odom = 'odom_{}'.format(self.robot_name)
        self.frame_map = 'map_{}'.format(self.robot_name)

        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.callback_map)

        self.rate = rospy.Rate(1) #Hz

    def callback_map(self, msg):
        self.map_grid.update(msg)

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self.map_grid.display()
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    frontier_expander = FrontierExpander()
    frontier_expander.spin()
