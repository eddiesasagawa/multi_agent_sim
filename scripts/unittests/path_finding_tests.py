import numpy as np
from scripts.support.graphs import SquareGrid
from scripts.support.path_finder import PathFinderAStar

def draw_grid(graph, start, goal, path=[], costs=[], point_to=[], spacing=2):
    '''
    graph should be of SquareGrid type
    path is a list of locations comprise the path
    '''
    locations = [[(x, y) for x in range(graph.width)] for y in range(graph.height)]
    print(''.join(['-']*(graph.width*spacing)))
    for row in locations:
        row_disp = [
            "A" if loc == start else
            "Z" if loc == goal else
            "@" if loc in path else
            ">" if loc in point_to and point_to[loc][0] == loc[0]+1 else
            "<" if loc in point_to and point_to[loc][0] == loc[0]-1 else
            "v" if loc in point_to and point_to[loc][1] == loc[1]+1 else
            "^" if loc in point_to and point_to[loc][1] == loc[1]-1 else
            "#" if graph.map[loc[1], loc[0]] == graph.map_occupied else
            "."
            for loc in row
        ]
        print(''.join([' ']*spacing).join(row_disp))    

    print(''.join(['-']*(graph.width*spacing)))
    if costs:
        print("Costs:")
        for row in locations:
            row_disp = [
                "  A" if loc == start else
                "  Z" if loc == goal else
                "{:3}".format(int(costs[loc])) if loc in costs else
                "  #" if graph.map[loc[1], loc[0]] == graph.map_occupied else
                "  ."
                for loc in row
            ]
            print(''.join(row_disp))    
        print(''.join(['-']*(graph.width*spacing)))

def evaluate(pathfinder, test_grid, start, goal):
    '''
    Test grid should be an instance of SquareGrid
    '''
    print(">>> Testing < {} > search for path: {} -> {}".format(pathfinder.NAME, start, goal))
    finder = pathfinder(test_grid.map, test_grid.weights)
    paths, came_from, costs_so_far = finder(start, goal)
    print("search: {}".format('SUCCEEDED' if paths else 'FAILED'))
    print("paths: {}".format(paths))
    draw_grid(test_grid, start, goal, paths, costs=costs_so_far, point_to=came_from)
    print(' ')

### Red Blob Games example ###
def red_blob_test():
    diagram4 = np.zeros((10, 10))
    diagram4[7:9, 1:4] = 255
    weighted_locations = [(3, 4), (3, 5), (4, 1), (4, 2),
                            (4, 3), (4, 4), (4, 5), (4, 6), 
                            (4, 7), (4, 8), (5, 1), (5, 2),
                            (5, 3), (5, 4), (5, 5), (5, 6), 
                            (5, 7), (5, 8), (6, 2), (6, 3), 
                            (6, 4), (6, 5), (6, 6), (6, 7), 
                            (7, 3), (7, 4), (7, 5)]
    diagram4_weights = np.ones(diagram4.shape)
    for loc in weighted_locations:
        diagram4_weights[loc[1], loc[0]] = 5

    test_grid = SquareGrid(diagram4, weight_map=diagram4_weights)
    test_sets = (
        ((4,1), (8,7)),
        ((0,9), (3,3)),
        ((0,0), (3,9)),
        ((0,0), (8,8)),
        ((1,4), (7,8))
    )

    print(test_grid.weights)

    for start, goal in test_sets:
        evaluate(PathFinderAStar, test_grid, start, goal)

if __name__ == "__main__":
    red_blob_test()