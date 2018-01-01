import abc
import itertools
import heapq
from collections import deque
from Queue import PriorityQueue

import numpy as np
from scipy.spatial import distance

from scripts.support.graphs import SquareGrid

class FastPriorityQueue(object):
    '''
    Implementation from 
    @see https://www.redblobgames.com/pathfinding/a-star/implementation.html
    @see https://github.com/c2huc2hu/jps/blob/master/jps.py also for use of counter to break ties in priorities

    Could use PriorityQueue but it has additional overhead due to locking semantics for multiple concurrent users
    '''
    def __init__(self):
        self.elements = []
        self.counter = itertools.count()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, next(self.counter), item))
    
    def get(self):
        return heapq.heappop(self.elements)[-1]
    
    def flush(self):
        self.__init__()

class GraphNode(object):
    def __init__(self, location, f, g_value=0, h_value=0):
        self.location = location
        self.f = f
        self.prev = None
        self.g = g_value
        self.h = h_value

    def __cmp__(self, other):
        if type(other) == GraphNode:
            return cmp(self.f, other.f)
        else:
            raise TypeError('Cannot compare GraphNode to {!r}'.format(type(other)))

##################################################################################################3
class PathFinderAbstract(object):
    __metaclass__ = abc.ABCMeta
    '''
    Implements a Path Finding algorithm to get from A to B given a map
    The map should be a discretized map with 0 for (potentially) traversable cells and 0xFF for walls

    Any constraints on nearness to an obstacle should already be applied to the map before passing to this
    '''
    NAME = "Abstract"
    def __init__(self):
        pass

    @abc.abstractmethod
    def __call__(self, map, start, goal):
        '''
        Implementations can inherit this class and override this function
        '''
        return []

class PathFinderAStar(PathFinderAbstract):
    '''
    A* implementation of path finding.
    TODO include orientation into algorithm (maybe as a movement cost, and carrying along vectors with each node?)
    '''
    NAME = "A*"
    def __init__(self, grid_map, weight_map=None):
        '''
        Current implementation is such that this class is tied to a particular map.
        You could override it manually by assigning to graph directly, though...
        '''
        self.graph = SquareGrid(grid_map, weight_map)
        self.frontiers = FastPriorityQueue()
        
    def __call__(self, start, goal):
        '''
        @todo convert to numpy array usage, and use scipy library
        '''
        # self.frontiers.put((0, start))
        self.frontiers.flush()
        self.frontiers.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        path = []
        path_found = False

        while not self.frontiers.empty():
            current = self.frontiers.get()
            # for current_f, current in self.frontiers.queue:
            if current == goal:
                path_found = True

            if path_found:
                break

            neighbors = self.graph.neighbors(current)

            for nbr in neighbors:
                g = cost_so_far[current] + self.graph.cost(current, nbr)
                if nbr not in cost_so_far or g < cost_so_far[nbr]:
                    cost_so_far[nbr] = g
                    # f() = g() + h(), where the heuristic h() is just the manhattan distance to goal
                    f = g + self.graph.manhattan_distance(goal, nbr)
                    # f = g + distance.cityblock(nbr, goal)
                    # self.frontiers.put((f, nbr))
                    self.frontiers.put(nbr, f)
                    came_from[nbr] = current

        if path_found:
            prev = came_from[goal]
            while prev is not None:
                path.append(tuple(prev))
                prev = came_from[prev]

        return path[::-1], came_from, cost_so_far

class PathFinderJPS(PathFinderAbstract):
    '''
    implementing Jump-Point Search algorithm for path finding.

    It sounds like this algorithm is much faster than A* and is intended to work well with Grids
    '''
    NAME = "A* + JPS"

    def __init__(self, grid_map, weight_map=None):
        '''
        Current implementation is such that this class is to be instantiated at every new search
        '''
        self.graph = SquareGrid(grid_map, weight_map)
        self.frontiers = FastPriorityQueue()
        self.set_goal = (0, 0)

    def __call__(self, start, goal):
        '''
        Main search algorithm -> A* + JPS
        '''
        self.frontiers.flush()
        self.frontiers.put(start, 0)
        self.set_goal = goal # set goal up for recursive searches to use
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        path = []
        path_found = False

        while not self.frontiers.empty():
            current = self.frontiers.get()
            if current == goal:
                path_found = True
                break

            


    def find_jump_point(self, loc):
        '''
        Recursive function intended to prune the indicated location's neighbors.

        First apply straight pruning rule in vertical and horizontal directions
        Then apply diagonal pruning rule if jump point not found.
        '''
        pass

    def recursive_jps_cardinal(self, loc, loc_g, dir_vec):
        '''
        straight pruning rule
        dir_vec should be a 2-element vector in the set {-1, 0, 1} and point N/S/E/W (i.e., 1 element is 0)
        '''
        (x, y) = loc
        (dx, dy) = dir_vec

        # check for any forced neighbors
        # just need to check the two cells orthogonal to dir_vec:
        #  |   | x |   |        |   | v |   |
        #   --- --- ---          --- -+- ---
        #  | v +>  |   |   OR   | x | v | x |
        #   --- --- ---          --- --- ---
        #  |   | x |   |        |   |   |   |
        #
        # where center cell is cell being checked, v is the cell we came from, and x marks the two cells
        # that should be checked for obstacles.
        # Note that due to symmetry, it doesn't matter whether the vector is up/down or left/right
        # Also, since this function is limited to horizontal and vertical unit vectors, one element will be +/- 1 while
        # the other is going to 0
        forced_nbrs_dirs = []
        if (not self.graph.is_unoccupied((x+dy, y+dx))) and self.graph.is_unoccupied((x+dx+dy, y+dy+dx)):
            forced_nbrs_dirs.append((dx+dy, dy+dx))
        if (not self.graph.is_unoccupied((x-dy, y-dx))) and self.graph.is_unoccupied((x+dx-dy, y+dy-dx)):
            forced_nbrs_dirs.append((dx-dy, dy-dx))

        # Exit conditions
        if forced_nbrs_dirs:
            # found a forced neighbor, so return this node with its cost and the directions of the forced neighbors
            return loc, loc_g, forced_nbrs_dirs
        elif (x+dx, y+dy) == self.set_goal:
            # found the goal, but just stick it on the stack for proper processing (should I?)
            return loc, loc_g, [dir_vec]
        elif self.graph.is_traversable((x+dx, y+dy)):
            # next node in straight line is valid, so recurse into it
            return self.recursive_jps_cardinal((x+dx, y+dy), loc_g+1, dir_vec)
        else:
            # hit a wall, so return empty
            return None, None, []

    def recursive_jps_diagonal(self, loc, loc_g, dir_vec):
        '''
        diagonal pruning rule
        dir_vec should be a 2-element vector in the set {-1, 1}, and point to any of the four corners

        The diagonal pruning rule basically checks the horizontal/vertical directions before recursing to next

        if direction is (1,1), then                 if direction is (1, -1), then
         --- --- ---                                 --- --- ---
        |===|   |   |                               |===|===|===|
         --- --- ---                                 --- --- ---
        |===| / |   |                               |===| \ |   |
         --- --- ---                                 --- --- ---
        |===|===|===|                               |===|   |   |
         --- --- ---                                 --- --- ---

        horizontal dir = (1, 0)                     horizontal dir = (1, 0)
        vertical dir = (0, 1)                       vertical dir = (0, -1)

        So when checking cardinal directions, just take individual components of (dx, 0), (0, dy)

        @return list of tuples containing jump points (if any). Each tuple contains (location, location_cost, list of direction vectors to take)
        '''
        (x, y) = loc
        (dx, dy) = dir_vec
        jump_points = []

        # check horizontal direction first
        horiz_jp = self.recursive_jps_cardinal((x+dx, y), loc_g+1, (dx, 0))

        # check vertical
        vert_jp = self.recursive_jps_cardinal((x, y+dy), loc_g+1, (0, dy))

        if horiz_jp is not None:
            jump_points.append(horiz_jp)
        if vert_jp is not None:
            jump_points.append(vert_jp)

        # Exit conditions
        if jump_points or (x+dx, y+dy) == self.set_goal:
            # found either jump points or the goal
            jump_points.append((loc, loc_g, [dir_vec]))
            return jump_points
        elif self.graph.is_traversable((x+dx, y+dy)):
            # no jump points detected, so move to next in diagonal
            return self.recursive_jps_diagonal((x+dx, y+dy), loc_g+1, dir_vec)
        else:
            # hit a wall, so return empty
            return []


class PathFinderRRT(PathFinderAbstract):
    '''
    Placeholder TODO for implementing Rapidly-exploring Random Trees for path finding..

    I also noticed there were stuff like A*-RRT that further improved performance.
    '''
    pass