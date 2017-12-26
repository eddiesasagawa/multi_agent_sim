import numpy as np

class Graph(object):
    '''
    Not sure if this is needed, but intended to be a way to convert a grid based map to something more reduced/efficient for path finding.
    '''
    pass

class PathFinder(object):
    '''
    Implements a Path Finding algorithm to get from A to B given a map
    The map should be a discretized map with 0 for (potentially) traversable cells and 0xFF for walls

    '''
    def __init__(self, min_cells_radius=1):
        self.min_cells_radius = min_cells_radius

    def __call__(self, map, start, goal):
        '''
        Default implementation will be A*

        Future implementations can inherit this class and override this function
        '''
        pass

    def heuristic(self, p1, p2):
        '''
        Heuristic for algorithm
        For A*, this will be a measure of distance to goal (euclidean)
        '''
        pass

class PathFinderJPS(PathFinder):
    '''
    Placeholder TODO for implementing Jump-Point Search algorithm for path finding.

    It sounds like this algorithm is much faster than A* and is intended to work well with Grids
    '''
    pass

class PathFinderRRT(PathFinder):
    '''
    Placeholder TODO for implementing Rapidly-exploring Random Trees for path finding..

    I also noticed there were stuff like A*-RRT that further improved performance.
    '''
    pass