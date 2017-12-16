#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

import cv2
import rospy

import tf
import tf2_ros

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TransformStamped, Quaternion
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
        self.map_im = np.array([])
        self.roi = np.array([])
        self.map_is_loaded = False
        self.roi_origin = Pose()
        self.roi_max = Point()
        self.roi_min = Point()

        self.occupancy_thresh = 50 # % probability of occupancy before considering as occupied.
        self.map_roi_boundary = 5 # extra cells to keep around known map 

        cv2.namedWindow('map_im', cv2.WINDOW_NORMAL)
        # self.fig = plt.figure(1)
        # plt.ion()
        # plt.show()

    def update(self, map_msg):
        rospy.loginfo('update map')
        self.raw_msg = map_msg
        self.map_meta = map_msg.info
        self.map = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height, 1))

        self.map[self.map == -1] = 50
        self.map = self.map.astype(np.uint8)
        cv2.normalize(self.map, dst = self.map, alpha = 0, beta = 255, norm_type=cv2.NORM_MINMAX)

        self.extract_map_roi()
        self.find_frontiers()

        self.map_is_loaded = True
        
    def display(self):
        if self.map_is_loaded:
            self.map_im = cv2.cvtColor(self.roi, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(self.map_im, [self.outline], 0, (0, 255, 0), 1)
            self.map_im[self.frontiers > 0] = [0, 0, 255]
            cv2.imshow('map_im', self.map_im)
            cv2.waitKey(10)

    def extract_map_roi(self):
        # find map contour to zoom in on only region of interest
        temp = self.map.copy()
        ret, thresh = cv2.threshold(temp, 127, 255, cv2.THRESH_BINARY_INV)
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # find largest contour
        areas = [cv2.contourArea(c) for c in contours]
        max_idx = np.argmax(areas)
        self.outline = contours[max_idx]    # keep this for later
        # rospy.loginfo('\n\routline ({}): {}'.format(self.outline.shape, self.outline))
        x, y, w, h = cv2.boundingRect(self.outline)

        # now filter for ROI
        self.roi_width = w
        self.roi_height = h
        self.roi_min.x = (x - self.map_roi_boundary) if (x-self.map_roi_boundary) > 0 else 0
        self.roi_max.x = (x + w + self.map_roi_boundary) if (x+w+self.map_roi_boundary) < self.map_meta.width else self.map_meta.width
        self.roi_min.y = (y - self.map_roi_boundary) if (y-self.map_roi_boundary) > 0 else 0
        self.roi_max.y = (y + h + self.map_roi_boundary) if (y+h+self.map_roi_boundary) < self.map_meta.height else self.map_meta.height

        # get new map parameters
        self.roi = self.map[self.roi_min.y:self.roi_max.y, self.roi_min.x:self.roi_max.x]
        self.roi_origin.position.x = self.roi_min.x - self.map_meta.origin.position.x
        self.roi_origin.position.y = self.roi_min.y - self.map_meta.origin.position.y
        self.roi_origin.orientation.x = self.map_meta.origin.orientation.x
        self.roi_origin.orientation.y = self.map_meta.origin.orientation.y
        self.roi_origin.orientation.z = self.map_meta.origin.orientation.z
        self.roi_origin.orientation.w = self.map_meta.origin.orientation.w

        #convert contours to ROI
        self.outline[:, :, 0] = self.outline[:, :, 0] - self.roi_min.x
        self.outline[:, :, 1] = self.outline[:, :, 1] - self.roi_min.y

    def is_open_cell(self, c):
        return c < 128

    def is_unknown_cell(self, c):
        return c == 128

    def is_occupied_cell(self, c):
        return c > 128

    def pos2cell(self, x, y):
        if self.map_is_loaded:
            x_col = int((x - self.roi_origin.position.x) / self.map_meta.resolution)
            y_row = int((y - self.roi_origin.position.y) / self.map_meta.resolution)
            return x_col, y_row
        else:
            return 0, 0

    def cell2pos(self, row, col):
        pass

    def is_cell_possible_frontier(self, r, c):
        '''
        this function assumes that there is at least 1 cell in all directions at the given r(ow) c(olumn)
        TODO: make this function work over the whole array somehow...
        '''
        if self.is_unknown_cell(self.roi[r, c]):
            # get a grid of indexes around the point of interest
            nx, ny = np.meshgrid([-1, 0, 1], [-1, 0, 1])

            # if any of the 8 neighboring cells are open, this is a potential frontier
            return 255 if np.any(self.is_open_cell(self.roi[nx+r, ny+c])) else 0
        return 0

    def find_frontiers(self):
        ### first go through the ROI and identify possible frontier cells
        # create a new numpy array with unoccupied grids = 0x01 and occupied grids = 0x10, and unknown = 0x00
        cpy = self.roi.copy()
        cpy[self.is_open_cell(cpy)] = 0x01
        cpy[self.is_occupied_cell(cpy)] = 0x10
        cpy[self.is_unknown_cell(cpy)] = 0x00

        # apply convolution with a 3x3 kernel of 1s (sum all elements) 
        raw_frontiers = cv2.filter2D(cpy, -1, np.ones((3,3)), cv2.BORDER_REPLICATE)
        # use the original as a mask to get values for only unknown cells
        ret, mask = cv2.threshold(cpy, 0, 255, cv2.THRESH_BINARY_INV)
        self.frontiers = cv2.bitwise_and(raw_frontiers, raw_frontiers, mask=mask)
        # since kernel is just 3x3, a pixel is next to empty cell if lower nibble is >0
        self.frontiers = cv2.bitwise_and(self.frontiers, 0x0F)


        # get a grid of indices of the ROI (ignore the outermost edges)
        # rospy.loginfo('start search {}'.format(self.roi.shape))
        # xx, yy = np.meshgrid(np.arange(1, self.roi.shape[0]-1), np.arange(1, self.roi.shape[1]-1))
        # rospy.loginfo('sizes: {}, {}'.format(xx.shape, yy.shape))
        # self.frontiers = np.vectorize(self.is_cell_possible_frontier)(xx, yy)
        # rospy.loginfo('found frontiers')

        ### group the frontier cells into regions


        ### filter based on width of regions (make sure robot can get through it)


        
    

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
