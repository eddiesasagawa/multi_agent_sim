import numpy as np
import math

class Graph(object):
    '''
    Not sure if this is needed, but intended to be a way to convert a grid based map to something more reduced/efficient for path finding.
    '''

    def __init__(self):
        pass

    def neighbors(self, loc):
        raise NotImplementedError

    def cost(self, loc, next):
        raise NotImplementedError

class SquareGrid(Graph):
    '''
    Graph wrapper for 2d maps

    NOTE: maps are assumed to be numpy arrays.
    
    '''
    def __init__(self, grid_map, weight_map=None):
        self.map_traversable = 0
        self.map_occupied = 255
        
        if weight_map is None:
            weight_map = np.ones(grid_map.shape)

        if grid_map.shape[:2] != weight_map.shape[:2]:
            raise ValueError("Grid map shape of {} does not match weight map shape of {}".format(grid_map.shape, weight_map.shape))
        self.height, self.width = grid_map.shape[:2]
        self.y_off = self.height - 1
        self.map = grid_map.copy()
        self.map[self.map < self.map_occupied] = self.map_traversable
        
        self.weights = weight_map.copy()

    def manhattan_distance(self, p1, p2):
        (i1, j1) = p1
        (i2, j2) = p2
        return abs(i2-i1) + abs(j2-j1)

    def euclidean_distance(self, p1, p2):
        (i1, j1) = p1
        (i2, j2) = p2
        return math.sqrt((i2-i1)**2 + (j2-j1)**2)

    def is_in_bounds(self, loc):
        (i, j) = loc
        return 0 <= j < self.width and 0 <= i < self.height
    
    def is_unoccupied(self, loc):
        return self.map[*loc] == self.map_traversable

    def is_traversable(self, loc):
        return self.is_in_bounds(loc) and self.is_unoccupied(loc)

    def neighbors(self, loc):
        (i, j) = loc
        nbrs = [
            # just construct it manually since it is only 8 neighbors (diagonals too)
            # list the 4 cardinal directions before diagonals
            (i-1, j),
            (i,   j-1),
            (i,   j+1),
            (i+1, j),
            (i-1, j-1),
            (i-1, j+1),
            (i+1, j-1),
            (i+1, j+1)
        ]
        nbrs = filter(self.is_traversable, nbrs)
        return nbrs

    def cost(self, loc, dest):
        '''
        Cost of moving to destination.
        For now, this is only dependent on the destination, and is simply 1
        Since the map contains values between 0 and 255, we can scale the values as weights
        '''
        return self.weights[*dest]

