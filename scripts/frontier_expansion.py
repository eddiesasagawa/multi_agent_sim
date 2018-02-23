#!/usr/bin/env python
import time
import sys
import numpy as np
# import matplotlib.pyplot as plt
# import seaborn as sns

import hdbscan

import cv2
import rospy

import tf
import tf2_ros

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, TransformStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from visualization_msgs.msg import Marker

from abstractions.abstract_node import AbstractNode
from support.path_finder import PathFinderJPS

from multi_agent_sim.srv import *

class MapGrid2D(object):
    '''
    From nav_msgs/OccupancyGrid message:
    # The map data, in row-major order, starting with (0,0).  Occupancy
    # probabilities are in the range [0,100].  Unknown is -1.


    '''
    def __init__(self, map_name):
        self.name = map_name
        self.raw_msg    = OccupancyGrid()     # OccupancyGrid message goes here
        self.map_meta   = MapMetaData()     # MapMetaData
        self.map = np.array([])
        self.map_im = np.array([])
        self.roi = np.array([])
        self.frontiers = []
        self.poi = []
        self.debug_markers = []

        self.map_is_loaded = False

        # ROI
        self.roi_origin = Pose()
        self.roi_max = Point()
        self.roi_min = Point()

        # HDBSCAN
        self.clusterer = hdbscan.HDBSCAN(min_cluster_size=50)

        # parameters
        self.occupancy_thresh = 50 # % probability of occupancy before considering as occupied.
        self.map_roi_boundary = 5 # extra cells to keep around known map 

        # Plotting stuff
        # sns.set_context('poster')
        # sns.set_color_codes()
        # main map window 
        cv2.namedWindow('map_im', cv2.WINDOW_NORMAL)
        # auxiliary matplotlib plots
        # self.fig = plt.figure(1)
        # plt.ion()
        # plt.grid()
        # plt.show()

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
        if self.map_is_loaded:
            x = col*self.map_meta.resolution + self.roi_origin.position.x
            y = row*self.map_meta.resolution + self.roi_origin.position.y
            return x, y
        else:
            return 0, 0

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
        self.roi_origin.position.x = (self.roi_min.x * self.map_meta.resolution) + self.map_meta.origin.position.x
        self.roi_origin.position.y = (self.roi_min.y * self.map_meta.resolution) + self.map_meta.origin.position.y
        self.roi_origin.orientation.x = self.map_meta.origin.orientation.x
        self.roi_origin.orientation.y = self.map_meta.origin.orientation.y
        self.roi_origin.orientation.z = self.map_meta.origin.orientation.z
        self.roi_origin.orientation.w = self.map_meta.origin.orientation.w
        # rospy.loginfo('Map Origin: ({}, {}), ROI origin: ({}, {})'.format(self.map_meta.origin.position.x, self.map_meta.origin.position.y, self.roi_origin.position.x, self.roi_origin.position.y))

        #convert contours to ROI
        self.outline[:, :, 0] = self.outline[:, :, 0] - self.roi_min.x
        self.outline[:, :, 1] = self.outline[:, :, 1] - self.roi_min.y

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
        self.frontier_mask = cv2.bitwise_and(raw_frontiers, raw_frontiers, mask=mask)
        # since kernel is just 3x3, a pixel is next to empty cell if lower nibble is >0
        self.frontier_mask = cv2.bitwise_and(self.frontier_mask, 0x0F)
        self.frontier_mask[self.frontier_mask > 0] = 255 # make it binary

        ### Group frontier cells
        self.frontier_pts = np.transpose(np.nonzero(self.frontier_mask))
        # Run HDBSCAN to do cluster analysis
        self.clusterer.fit(self.frontier_pts)
        rospy.loginfo('clustered: {}'.format(self.clusterer.labels_.max()+1))
        self.clusters = [self.frontier_pts[self.clusterer.labels_ == c] for c in range(self.clusterer.labels_.max()+1)]
        self.frontiers = [{'mean': np.mean(cluster, axis=0).astype(int), 'std': np.std(cluster, axis=0).astype(int), 'count': cluster.shape[0]} for cluster in self.clusters]

    @property
    def markers(self):
        '''
        This property creates a Marker message with Sphere List for all detected frontiers.
        Since this is mostly just for visualization (?), I went with Sphere List instead of a MarkerArray
        '''
        marker = Marker()
        marker.header.frame_id = self.name
        marker.header.stamp = self.map_meta.map_load_time
        marker.ns = self.name + '_frontiers'
        marker.id = 0
        marker.frame_locked = True
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD

        positions = [self.cell2pos(f['mean'][0], f['mean'][1]) for f in self.frontiers]
        marker.points = [Point(x=x, y=y) for x, y in positions]

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.1
        marker.color.a = 0.5
        marker.color.b = 1.0

        marker.lifetime = rospy.Duration()
        return marker

    def mark_map(self, point):
        self.poi.append((self.pos2cell(point[0], point[1])))

    def update(self, map_msg):
        rospy.loginfo('update map')
        self.raw_msg = map_msg
        self.map_meta = map_msg.info
        self.map = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height, 1))

        self.map[self.map == -1] = 50
        self.map = self.map.astype(np.uint8)
        cv2.normalize(self.map, dst = self.map, alpha = 0, beta = 255, norm_type=cv2.NORM_MINMAX)

        t_1 = time.time()
        self.extract_map_roi()
        t_2 = time.time()
        self.find_frontiers()
        t_3 = time.time()

        rospy.loginfo("\n\rMap: extract={} s, find={} s".format(t_2-t_1, t_3-t_2))
        self.map_is_loaded = True

    def find_path(self, start, goal):
        '''
        Uses a Path Finder to figure out a path from start to goal.
        start is a tuple of (x,y)
        goal is a tuple of (x,y)
        '''
        ret, thresh_im = cv2.threshold(self.roi, 128, 255, cv2.THRESH_BINARY)
        pathfinder = PathFinderJPS(thresh_im)
        # convert start / goal to cells
        s = self.pos2cell(*start)
        g = self.pos2cell(*goal)
        path, came_from, cost_so_far = pathfinder(s, g)
        self.debug_markers = path
        
        return [(self.cell2pos(p[1],p[0])) for p in path]

    def display(self):
        if self.map_is_loaded:
            self.map_im = cv2.cvtColor(self.roi, cv2.COLOR_GRAY2BGR)
            # cv2.drawContours(self.map_im, [self.outline], 0, (0, 255, 0), 1)
            self.map_im[self.frontier_mask > 0] = [0, 0, 255]

            for frontier in self.frontiers:
                self.map_im = cv2.ellipse(self.map_im, tuple(frontier['mean'][::-1]), tuple(frontier['std'][::-1]), 0.0, 0.0, 360.0, (0, 255, 0))

            for p1, p2 in zip(self.poi[1:-1], self.poi[2:]):
                self.map_im = cv2.arrowedLine(self.map_im, p1, p2, (255, 0, 0))

            for p in self.debug_markers:
                self.map_im = cv2.circle(self.map_im, p, 3, (255, 0, 0))

            cv2.imshow('map_im', cv2.flip(self.map_im, 0))
            cv2.waitKey(10)

class FrontierExpander(AbstractNode):
    def __init__(self):
        super(FrontierExpander, self).__init__('frontier_expansion')

        self.robot_name = rospy.get_param('robot_name')

        self.frame_baselink = 'base_link_{}'.format(self.robot_name)
        self.frame_odom = 'odom_{}'.format(self.robot_name)
        self.frame_map = 'map_{}'.format(self.robot_name)

        self.map_grid = MapGrid2D(self.frame_map)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.callback_map)
        self.frontier_pub = rospy.Publisher('frontiers', Marker, queue_size=10, latch=True)

        self.path_req_srv = rospy.Service('RequestPath', RequestPath, self.callback_request_path)

        self.rate = rospy.Rate(2) #Hz

    def callback_map(self, msg):
        self.map_grid.update(msg)
        self.frontier_pub.publish(self.map_grid.markers)

    def callback_request_path(self, req):
        resp = RequestPathResponse()
        s, g = (req.start.position.x, req.start.position.y), (req.end.position.x, req.end.position.y)
        rospy.loginfo('Serving RequestPath: {} -> {}'.format(s, g))
        path = self.map_grid.find_path(s, g)

        resp.is_found = bool(path)
        resp.path.poses = [PoseStamped() for i in range(len(path))]
        for i, p in enumerate(path):
            self.map_grid.mark_map(p)
            resp.path.poses[i].pose.position.x = p[0]
            resp.path.poses[i].pose.position.y = p[1]

        return resp

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
