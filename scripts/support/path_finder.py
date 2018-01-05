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

    Any constraints on nearness to an obstacle should alreadj be applied to the map before passing to this
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

    def extract_path(self, goal, came_from, cost_so_far):
        if goal not in came_from:
            return []

        path = [goal]
        prev = came_from[goal]
        while prev is not None:
            path.append(tuple(prev))
            prev = came_from[prev]

        return path[::-1], came_from, cost_so_far

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

        return self.extract_path(goal, came_from, cost_so_far)

class PathFinderJPS(PathFinderAbstract):
    '''
    implementing Jump-Point Search algorithm for path finding.

    It sounds like this algorithm is much faster than A* and is intended to work well with Grids
    '''
    NAME = "A* + JPS"

    class JumpPointResult(object):
        '''
        A little helper class to hold jump point information
        '''
        def __init__(self, loc=None, cost=None, directions=[], prev_loc=None):
            self.location = loc
            self.prev_loc = prev_loc
            self.cost = cost
            self.directions = directions

        def __repr__(self):
            return "JumpPointResult: {}, {}, {}".format(self.location, self.cost, self.directions)

    def __init__(self, grid_map, weight_map=None):
        '''
        Current implementation is such that this class is to be instantiated at every new search
        '''
        self.graph = SquareGrid(grid_map, weight_map)
        self.frontiers = FastPriorityQueue()
        self.set_goal = (0, 0)
        self.cardinal_directions = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1)
        ]
        self.diagonal_directions = [
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1)
        ]
        self.jumped_from = {}
        self.cost_so_far = {}

    def __call__(self, start, goal):
        '''
        Main search algorithm -> A* + JPS
        '''
        self.frontiers.flush()
        self.frontiers.put(start, 0)
        self.set_goal = goal # set goal up for recursive searches to use
        self.jumped_from = {}
        self.cost_so_far = {}
        directions_to_check = {}
        path = []
        path_found = False

        self.jumped_from[start] = None
        self.cost_so_far[start] = 0
        directions_to_check[start] = self.cardinal_directions + self.diagonal_directions

        while not self.frontiers.empty():
            current = self.frontiers.get()
            if current == goal:
                path_found = True
                break

            (i, j) = current

            # find all valid directions to check (ie, not an obstacle)
            cardinal_dirs_to_check = [d for d in self.cardinal_directions if (self.graph.is_traversable((i+d[0], j+d[1])) and d in directions_to_check[current])]
            diagonal_dirs_to_check = [d for d in self.diagonal_directions if (self.graph.is_traversable((i+d[0], j+d[1])) and d in directions_to_check[current])]

            jp_list = [
                self.search_cardinal_jump_point((i+dir_vec[0],j+dir_vec[1]), self.cost_so_far[current]+1, dir_vec, current) for dir_vec in cardinal_dirs_to_check
            ] 
            for dir_vec in diagonal_dirs_to_check:
                jp_list.extend(self.search_diagonal_jump_point((i+dir_vec[0],j+dir_vec[1]), self.cost_so_far[current]+1, dir_vec, current))

            print('processed {} -> cdir: {}, ddir: {}, jp list: {}'.format(current, cardinal_dirs_to_check, diagonal_dirs_to_check, jp_list))
            # process all found jump points
            for jp in jp_list:
                if (jp.location is not None) and (jp.location not in self.cost_so_far or jp.cost < self.cost_so_far[jp.location]):
                    self.cost_so_far[jp.location] = jp.cost
                    self.jumped_from[jp.location] = jp.prev_loc
                    directions_to_check[jp.location] = jp.directions

                    f = jp.cost + self.graph.manhattan_distance(jp.location, goal)
                    self.frontiers.put(jp.location, f)

        return self.extract_path(goal, self.jumped_from, self.cost_so_far)

    def search_cardinal_jump_point(self, start_loc, start_cost, dir_vec, source_loc):
        '''
        Apply straight pruning rule along a cardinal direction
        dir_vec should be a 2-element vector in the set {-1, 0, 1} and point N/S/E/W (i.e., 1 element is 0)

        @return a tuple of (jump_point, cost_to_jump_point, [directions_of_interest_from_this_jump_point])
        '''
        (i0, j0) = start_loc
        (di, dj) = dir_vec

        i = i0
        j = j0
        cost = start_cost

        while True:
            # quick goal check
            if (i,j) == self.set_goal:
                return self.JumpPointResult((i,j), cost, [dir_vec], source_loc)
            elif not self.graph.is_traversable((i,j)):
                return self.JumpPointResult(None, None, [dir_vec], None)

            # check for any forced neighbors
            # just need to check the two cells orthogonal to dir_vec:
            #  |   | i |   |        |   | v |   |
            #   --- --- ---          --- -+- ---
            #  | v +>  |   |   OR   | i | v | i |
            #   --- --- ---          --- --- ---
            #  |   | i |   |        |   |   |   |
            #
            # where center cell is cell being checked, v is the cell we came from, and i marks the two cells
            # that should be checked for obstacles.
            # Note that due to symmetry, it doesn't matter whether the vector is up/down or left/right
            # Also, since this function is limited to horizontal and vertical unit vectors, one element will be +/- 1 while
            # the other is going to 0
            forced_nbrs_dirs = []
            if (not self.graph.is_traversable((i+dj, j+di))) and self.graph.is_traversable((i+di+dj, j+dj+di)):
                forced_nbrs_dirs.append((di+dj, dj+di))
            if (not self.graph.is_traversable((i-dj, j-di))) and self.graph.is_traversable((i+di-dj, j+dj-di)):
                forced_nbrs_dirs.append((di-dj, dj-di))

            # Exit conditions
            if forced_nbrs_dirs:
                # found a forced neighbor, so return this node with its cost and the directions of the forced neighbors
                return self.JumpPointResult((i,j), cost, forced_nbrs_dirs, source_loc)
            else:
                # keep going
                i = i+di
                j = j+dj
                cost += 1

    def search_diagonal_jump_point(self, start_loc, start_cost, dir_vec, source_loc):
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

        So when checking cardinal directions, just take individual components of (di, 0), (0, dj)

        @return list of tuples containing jump points (if any). Each tuple contains (location, location_cost, list of direction vectors to take)
        '''
        (i0, j0) = start_loc
        (di, dj) = dir_vec
        
        i = i0
        j = j0
        cost = start_cost

        while True:
            # quick goal check
            if (i,j) == self.set_goal:
                return [self.JumpPointResult((i,j), cost, [dir_vec], source_loc)]
            elif not self.graph.is_traversable((i,j)):
                return []

            jump_points = []
            # check horizontal direction first
            horiz_jp = self.search_cardinal_jump_point((i+di, j), cost+1, (di, 0), (i,j))
            if horiz_jp.location is not None:
                jump_points.append(horiz_jp)

            # check vertical
            vert_jp = self.search_cardinal_jump_point((i, j+dj), cost+1, (0, dj), (i,j))
            if vert_jp.location is not None:
                jump_points.append(vert_jp)

            # Exit conditions
            if jump_points:
                # found either jump points or the goal
                jump_points.append(self.JumpPointResult((i, j), cost, [dir_vec], source_loc))
                return jump_points
            else:
                i = i+di
                j = j+dj
                cost += 1


class PathFinderRRT(PathFinderAbstract):
    '''
    Placeholder TODO for implementing Rapidly-exploring Random Trees for path finding..

    I also noticed there were stuff like A*-RRT that further improved performance.
    '''
    pass