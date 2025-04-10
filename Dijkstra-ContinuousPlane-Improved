import numpy as np
import matplotlib.pyplot as plt
import time
import bisect

def bresenham_line(x0, y0, x1, y1):
    """
    Returns a list of tuples representing the coordinates of the pixels along the line between (x0, y0) and (x1, y1).
    """
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    line = []
    while True:
        line.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return line

def eucledian_distance(pos1, pos2):
    return np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)

def calcutalte_path_length(path):
    length = 0
    for p_i in range(len(path)-1):
        if path[p_i][0] == path[p_i+1][0] or path[p_i][1] == path[p_i+1][1]:
            length += 1
        else:
            length += 1.41
    return length

def generate_problem(
        dim_x = 1000, 
        dim_y = 1000, 
        min_obs_count = 10, 
        max_obs_count = 15, 
        min_obs_side = 50, 
        max_obs_side = 150, 
        manual_onbstacle = None, 
        start_pos = None,
        goal_pos = None
    ) -> np.array:
    '''
    Generate a random problem
    returns:
        obstacle_map: numpy.array((DIM_X, DIM_Y)) - 2D array representing the obstacle map
        start_pos: tuple - (x, y) coordinates of the start position
        goal_pos: tuple - (x, y) coordinates of the goal position
    '''
    obstacle_map = np.zeros((dim_x, dim_y), dtype=int)
    if manual_onbstacle is None:
        for _ in range(np.random.randint(min_obs_count, max_obs_count)):
            obstacle_dim = np.random.randint(min_obs_side, max_obs_side, 2)
            obstacle_pos = (np.random.randint(0, dim_x), np.random.randint(0, dim_y))
            obstacle_map[obstacle_pos[0]:obstacle_pos[0]+obstacle_dim[0], obstacle_pos[1]:obstacle_pos[1]+obstacle_dim[1]] = 1
    else:
        for obs in manual_onbstacle:
            obstacle_map[obs[0]:obs[0]+obs[2], obs[1]:obs[1]+obs[3]] = 1
    if start_pos is None:
        while True:
            start_pos = (np.random.randint(0, dim_x), np.random.randint(0, dim_y))
            if obstacle_map[start_pos[0], start_pos[1]] == 0:
                break
    else:
        if obstacle_map[start_pos[0], start_pos[1]] == 1:
            raise ValueError("Start position is in an obstacle")
    if goal_pos is None:
        while True:
            goal_pos = (np.random.randint(0, dim_x), np.random.randint(0, dim_y))
            if obstacle_map[goal_pos[0], goal_pos[1]] == 0:
                break
    else:
        if obstacle_map[goal_pos[0], goal_pos[1]] == 1:
            raise ValueError("Goal position is in an obstacle")
        
    return obstacle_map, start_pos, goal_pos

def identify_corners(obstacle_map) -> dict:
    '''
    Identify corners of the obstacles in the obstacle_map
    args:
        obstacle_map: np.array((DIM_X, DIM_Y)) - 2D array representing the obstacle map
    returns:
        corners: dict - dictionary containing the corners of the obstacles in the obstacle_map
    '''
    DIM_X = obstacle_map.shape[0]
    DIM_Y = obstacle_map.shape[1]
    CORNER_TYPES={1: [[0, 0], [0, 1]], 2: [[0, 0], [1, 0]], 3: [[1, 0], [0, 0]], 4: [[0, 1], [0, 0]], 5: [[1, 1], [1, 0]], 6: [[1, 1], [0, 1]], 7: [[0, 1], [1, 1]], 8: [[1, 0], [1, 1]]}
    CORNER_CHECK_INDEX = {1: [-1, -1], 2: [-1, 0], 3: [0, 0], 4: [0, -1], 5: [0, 0], 6: [0, 0], 7: [0, -1], 8: [0, 0]}
    '''
    CORNER_TYPES [t1, t2, t3, t4, t5, t6, t7, t8]
    ---------------------   
    ----t1-----------t2--   Obstacle : [W]
    ----WWWWWWWWWWWWWW---   Free : [-]
    ----WWWWWWWWWWWWWW---   
    ------t6 WWWWWWWWW---   
    ---------WWWW t5-----   
    ------t7 WWWW--------   
    ----WWWWWWWWW t8-----   
    ----WWWWWWWWWWWWWW---   
    ----WWWWWWWWWWWWWW---   
    ----t4-----------t3--   
    ---------------------        
    '''
    corners = {1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: [], 8: []}
    for x in range(1, DIM_X-1):
        for y in range(1, DIM_Y-1):
            if obstacle_map[x, y] == 1:
                for i in corners:
                    x2=x+CORNER_CHECK_INDEX[i][0]
                    y2=y+CORNER_CHECK_INDEX[i][1]
                    if x2 < 0 or y2 < 0 or x2+1 >= DIM_X or y2+1 >= DIM_Y:
                        continue
                    temp_submatrix = obstacle_map[x2:x2+2, y2:y2+2] 
                    if np.array_equal(temp_submatrix, CORNER_TYPES[i]):
                        corners[i].append([x, y])
                        break
    return corners

def extract_obstacle_edge_vertices(corners, dim_x, dim_y) -> tuple:
    ob_x, ob_y = list(), list()
    for i in corners:
        for corner in corners[i]:
            bisect.insort(ob_x, corner[0])
            bisect.insort(ob_y, corner[1])
    if 0 not in ob_x:
        ob_x.insert(0, 0)
    if 0 not in ob_y:
        ob_y.insert(0, 0)
    if dim_x not in ob_x:
        ob_x.append(dim_x)
    if dim_y not in ob_y:
        ob_y.append(dim_y)
    ob_x = list(dict.fromkeys(ob_x))
    ob_y = list(dict.fromkeys(ob_y))
    return ob_x, ob_y

def fill_free_space_with_rectangles(obstacle_map, ob_x, ob_y) -> list:
    '''
    Fill the map with rectangles of free space
    args:
        obstacle_map: np.array((DIM_X, DIM_Y)) - 2D array representing the obstacle map
        ob_x: list - list of x coordinates of the obstacle borders
        ob_y: list - list of y coordinates of the obstacle borders
    returns:
        rects: list [region_number, x_begin, x_end, y_begin, y_end] - list of rectangles of free space
    '''
    region_map = np.copy(obstacle_map) # Copy the obstacle_map to region_map
    rects = [] # list of rectangles of free space
    region_graph = {}
    '''
    {
        region_number: {
            neighbor_region_1: [(coordinates)*],
        }
    }
    '''
    region = 2 # 1 is reserved for obstacles and 0 is reserved for unexplored space
    offsets = [np.argwhere(region_map == 0)[0]]
    while True: # Loop until all free space is filled
        # find first occurance of 0 in obstacle_map
        if len(offsets) == 0:
            p = np.argwhere(region_map == 0)
            if len(p) == 0:
                break
            x, y = p[0]
        else:
            x, y = offsets.pop(0)
        # get greater and smaller closest value from ob_x and ob_y
        if x in ob_x:
            x1 = x
            x2 = ob_x[np.searchsorted(ob_x, x)+1]
        else:
            x1 = ob_x[np.searchsorted(ob_x, x)-1]
            x2 = ob_x[np.searchsorted(ob_x, x)]
        if y in ob_y:
            y1 = y
            y2 = ob_y[np.searchsorted(ob_y, y)+1]
        else:
            y1 = ob_y[np.searchsorted(ob_y, y)-1]
            y2 = ob_y[np.searchsorted(ob_y, y)]

        for x_i in range(x1, x2):
            for y_i in range(y1, y2):
                region_map[x_i, y_i] = region
        
        rects.append([region, x1, x2-1, y1, y2-1])
        if y2<dim_y and obstacle_map[x1, y2] == 0:
            offsets.append([x1, y2])
        if obstacle_map[x1, y1] == 1:
            raise ValueError("Unexpected obstacle")
        # find the neighbors of the region
        region_graph[region] = {}
        for x_i in range(max(0, x1-1), min(x2+1, obstacle_map.shape[0])):
            nr = region_map[x_i, y1-1]
            if nr != 0 and nr != 1 and nr != region and y1 != 0:
                region_graph[region][nr].append((x_i, y1-1)) if nr in region_graph[region] else region_graph[region].update({nr: [(x_i, y1-1)]})
                if region_map[x_i, y1] != 0 and region_map[x_i, y1] != 1 and region_map[x_i, y1] != region:
                    region_graph[nr][region].append((x_i, y1)) if region in region_graph[nr] else region_graph[nr].update({region: [(x_i, y1)]})
        for y_i in range(max(0, y1-1), min(y2+1, obstacle_map.shape[1])):
            nr = region_map[x1-1, y_i]
            if nr != 0 and nr != 1 and nr != region and x1 != 0:
                region_graph[region][nr].append((x1-1, y_i)) if nr in region_graph[region] else region_graph[region].update({nr: [(x1-1, y_i)]})
                if region_map[x1, y_i] != 0 and region_map[x1, y_i] != 1 and region_map[x1, y_i] != region:
                    region_graph[nr][region].append((x1, y_i)) if region in region_graph[nr] else region_graph[nr].update({region: [(x1, y_i)]})
        region += 1
    #goal_region = region_map[goal_pos[0], goal_pos[1]]
    #region_map[goal_pos[0], goal_pos[1]] = region
    #region_graph[goal_region][region] = [(goal_pos[0], goal_pos[1])]
    return rects, region_map, region_graph

def merge_rectangles(rects):
    '''
    Merge the rectangles of free space to form larger rectangles
    '''
    pass

def find_partial_path_bu(region_map, region_graph, start_pos, goal_pos):
    start_region = region_map[start_pos[0], start_pos[1]]
    goal_region = region_map[goal_pos[0], goal_pos[1]]
    neighbor_regions = {}
    for region in region_graph:
        neighbor_regions[region] = []
        for nr in region_graph[region]:
            neighbor_regions[region].append(nr)
    move = dict()
    explored = set()
    frontier = [(start_region, start_region,0)]
    while len(frontier) > 0:
        frontier.sort(key=lambda x: x[2])
        pos = frontier.pop(0)
        explored.add(pos[0])
        move[pos[0]] = pos[1]
        if pos[0] == goal_region:
            break
        for nr in neighbor_regions[pos[0]]:
            if nr not in explored:
                frontier.append((nr, pos[0], eucledian_distance([pos[0], 0], [goal_region, 0])))
    path = []
    reg = goal_region
    while reg != start_region:
        path.append(reg)
        reg = move[reg]
    path.append(start_region)
    for i in range(len(path)-1):
        possible_paths = region_graph[path[i]][path[i+1]]
        # select the one with the least distance to the goal
        samples = [possible_paths[0], possible_paths[-1]]
        if samples[0][0] == samples[1][0]:
            if samples[0][1] < goal_pos[1] and samples[1][1] > goal_pos[1]:
                path[i]=(samples[0][0], goal_pos[1])
            else:
                if abs(samples[0][1]-goal_pos[1]) < abs(samples[1][1]-goal_pos[1]):
                    path[i]=samples[0]
                else:
                    path[i]=samples[1]
        elif samples[0][1] == samples[1][1]:
            if samples[0][0] < goal_pos[0] and samples[1][0] > goal_pos[0]:
                path[i]=(goal_pos[0], samples[0][1])
            else:
                if abs(samples[0][0]-goal_pos[0]) < abs(samples[1][0]-goal_pos[0]):
                    path[i]=samples[1]
                else:
                    path[i]=samples[0]
        #path[i]=region_graph[path[i]][path[i+1]][0]
    path[-1] = goal_pos
    return path

def find_partial_path(region_map, region_graph, start_pos, goal_pos):
    start_region = region_map[start_pos[0], start_pos[1]]
    goal_region = region_map[goal_pos[0], goal_pos[1]]
    neighbor_regions = {}
    for region in region_graph:
        neighbor_regions[region] = []
        for nr in region_graph[region]:
            neighbor_regions[region].append(nr)
    neighbor_coords = {}
    for region in region_graph:
        neighbor_coords[region] = []
        for nr in region_graph[region]:
            for coord in region_graph[region][nr]:
                neighbor_coords[region].append(coord)
    move = dict()
    explored = set()
    frontier = [(start_pos, start_pos, 0, start_region, start_region)]
    while len(frontier) > 0:
        #frontier.sort(key=lambda x: x[2])
        head = frontier.pop(0)
        pos = head[1]
        explored.add(pos)
        frontier = [x for x in frontier if x[1] != pos]
        move[pos] = head[0]
        if region_map[pos[0], pos[1]] == goal_region:
            move[goal_pos] = pos
            break
        for coord in neighbor_coords[region_map[pos[0], pos[1]]]:
            if coord not in explored and region_map[coord[0], coord[1]] != head[3]:
                #frontier.append((pos, coord, head[2]+eucledian_distance(coord, pos), head[4], region_map[coord[0], coord[1]]))
                bisect.insort(frontier, (pos, coord, head[2]+eucledian_distance(coord, pos), head[4], region_map[coord[0], coord[1]]), key=lambda x: x[2])
    else:
        raise ValueError("No path found")
    path = []
    pos = goal_pos
    while pos != start_pos:
        path.append(pos)
        pos = move[pos]
    path.append(start_pos)
    return path

def complete_partial_path(path):
    path_complete = []
    for i in range(len(path)-1):
        part_path = bresenham_line(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
        path_complete.extend(part_path)
    return path_complete

def show_path(obstacle_map, start_pos, goal_pos, path, title="", caption="", plt=plt, rects=None):
    plt.imshow(obstacle_map)
    plt.set_title(title)
    plt.set_xlabel(caption)
    for i in range(len(path)-1, 0, -1):
        if path[i][0] == path[i-1][0] or path[i][1] == path[i-1][1]:
            path.pop(i)
    if rects is not None:
        pass
        #for r in rects:   
    # draw the path line
    for i in range(len(path)-1):
        plt.plot([path[i][1], path[i+1][1]], [path[i][0], path[i+1][0]], 'w')
    plt.plot(start_pos[1], start_pos[0], 'ro')
    plt.plot(goal_pos[1], goal_pos[0], 'go')
    #plt.gca().set_xlim([0, obstacle_map.shape[0]])
    #plt.gca().set_ylim([0, obstacle_map.shape[1]])
    plt.set_xlim([0, obstacle_map.shape[1]])
    plt.set_ylim([0, obstacle_map.shape[0]])
    #plt.show()

def dijkstra(obstacle_map, start_pos, goal_pos):
    strategy_name = "Dijkstra's Algorithm"
    print(strategy_name)
    start_time_temp = time.time()
    DIM_X, DIM_Y= obstacle_map.shape
    MOVE_DICT = {1: [0, -1], 2: [0, 1], 3: [-1, 0], 4: [1, 0], 5: [-1, -1], 6: [-1, 1], 7: [1, -1], 8: [1, 1]}
    explored = np.zeros((DIM_X, DIM_Y), dtype=bool)
    for x in range(DIM_X):
        for y in range(DIM_Y):
            if obstacle_map[x, y] == 1:
                explored[x, y] = True
    move = np.zeros((DIM_X, DIM_Y), dtype=int)
    frontier = [(start_pos,start_pos,0)]
    pos = frontier.pop(0)
    explored[pos[1][0], pos[1][1]] = True
    x, y = pos[1]
    for i in MOVE_DICT:
        x_mov, y_mov = MOVE_DICT[i]
        new_x, new_y = x+x_mov, y+y_mov
        if new_x >= 0 and new_x < DIM_X and new_y >= 0 and new_y < DIM_Y and not explored[new_x, new_y]:
            #frontier.append((pos[1], (new_x, new_y), pos[2]+(1 if x_mov==0 or y_mov==0 else 1.41)))
            bisect.insort(frontier, (pos[1], (new_x, new_y), pos[2]+(1 if x_mov==0 or y_mov==0 else 1.41)), key=lambda x: x[2])
    
    while len(frontier) > 0:
        #frontier.sort(key=lambda x: x[2])
        pos = frontier.pop(0)
        explored[pos[1][0], pos[1][1]] = True
        move[pos[1][0], pos[1][1]] = [k for k in MOVE_DICT if MOVE_DICT[k] == [pos[1][0]-pos[0][0], pos[1][1]-pos[0][1]]][0]
        frontier = [x for x in frontier if x[1] != pos[1]]
        x, y = pos[1]
        if pos[1] == goal_pos:
            move[pos[1][0], pos[1][1]] = [k for k in MOVE_DICT if MOVE_DICT[k] == [pos[1][0]-pos[0][0], pos[1][1]-pos[0][1]]][0]
            break
        for i in MOVE_DICT:
            x_mov, y_mov = MOVE_DICT[i]
            new_x, new_y = x+x_mov, y+y_mov
            if new_x >= 0 and new_x < DIM_X and new_y >= 0 and new_y < DIM_Y and not explored[new_x, new_y]:
                #frontier.append((pos[1], (new_x, new_y), pos[2]+(1 if x_mov==0 or y_mov==0 else 1.41)))
                bisect.insort(frontier, (pos[1], (new_x, new_y), pos[2]+(1 if x_mov==0 or y_mov==0 else 1.41)), key=lambda x: x[2])
    del frontier

    # Backtrack to find the path
    path = []
    pos = goal_pos
    while pos != start_pos:
        path.append(pos)
        x, y = pos
        move_num = move[x, y]
        x_mov, y_mov = MOVE_DICT[move_num]
        pos = (x-x_mov, y-y_mov)
    path.append(start_pos)
    time_taken_temp = time.time() - start_time_temp
    print("Total: %s seconds" % time_taken_temp)
    print()
    return strategy_name, path[::-1], time_taken_temp

def region_border_method(obstacle_map, start_pos, goal_pos, manual_onbstacle = None):
    strategy_name = "Boundary Points Method"
    print(strategy_name)
    if manual_onbstacle is None:
        corners = identify_corners(obstacle_map)
        ob_x, ob_y = extract_obstacle_edge_vertices(corners, obstacle_map.shape[0], obstacle_map.shape[1])
    else:
        ob_x, ob_y = [], []
        for obs in manual_onbstacle:
            bisect.insort(ob_x, obs[0])
            bisect.insort(ob_x, obs[0]+obs[2])
            bisect.insort(ob_y, obs[1])
            bisect.insort(ob_y, obs[1]+obs[3])
        if 0 not in ob_x:
            ob_x.insert(0, 0)
        if 0 not in ob_y:
            ob_y.insert(0, 0)
        if obstacle_map.shape[0] not in ob_x:
            ob_x.append(obstacle_map.shape[0])
        if obstacle_map.shape[1] not in ob_y:
            ob_y.append(obstacle_map.shape[1])
        # remove duplicates
        ob_x = list(dict.fromkeys(ob_x))
        ob_y = list(dict.fromkeys(ob_y))
    start_time_temp = time.time()
    print("Filling free space with rectangles")
    rects, region_map, region_graph = fill_free_space_with_rectangles(obstacle_map, ob_x, ob_y)
    #merge_rectangles(rects)
    print("Finding partial path")
    path = find_partial_path(region_map, region_graph, start_pos, goal_pos)
    print("Completing partial path")
    path_complete = complete_partial_path(path)
    time_taken_temp = time.time() - start_time_temp
    print("Total: %s seconds" % time_taken_temp)
    print()
    return strategy_name, path_complete, time_taken_temp, rects

def region_corner_method(obstacle_map, start_pos, goal_pos, manual_onbstacle = None):
    strategy_name = "Corner Points Method"
    print(strategy_name)
    if manual_onbstacle is None:
        corners = identify_corners(obstacle_map)
        ob_x, ob_y = extract_obstacle_edge_vertices(corners, obstacle_map.shape[0], obstacle_map.shape[1])
    else:
        ob_x, ob_y = [], []
        for obs in manual_onbstacle:
            bisect.insort(ob_x, obs[0])
            bisect.insort(ob_x, obs[0]+obs[2])
            bisect.insort(ob_y, obs[1])
            bisect.insort(ob_y, obs[1]+obs[3])
        if 0 not in ob_x:
            ob_x.insert(0, 0)
        if 0 not in ob_y:
            ob_y.insert(0, 0)
        if obstacle_map.shape[0] not in ob_x:
            ob_x.append(obstacle_map.shape[0])
        if obstacle_map.shape[1] not in ob_y:
            ob_y.append(obstacle_map.shape[1])
        # remove duplicates
        ob_x = list(dict.fromkeys(ob_x))
        ob_y = list(dict.fromkeys(ob_y))
    start_time_temp = time.time()
    print("Filling free space with rectangles")
    rects, region_map, region_graph = fill_free_space_with_rectangles(obstacle_map, ob_x, ob_y)
    #merge_rectangles(rects)
    print("Finding partial path")
    start_region = region_map[start_pos[0], start_pos[1]]
    goal_region = region_map[goal_pos[0], goal_pos[1]]
    neighbor_regions = {}
    for region in region_graph:
        neighbor_regions[region] = []
        for nr in region_graph[region]:
            neighbor_regions[region].append(nr)
    neighbor_coords = {}
    '''
    for region in region_graph:
        neighbor_coords[region] = []
        for nr in region_graph[region]:
            neighbor_coords[region].append(((rects[nr-2][1]+rects[nr-2][2])//2, (rects[nr-2][3]+rects[nr-2][4])//2))
    '''
    for region in region_graph:
        neighbor_coords[region] = []
        for nr in region_graph[region]:
            neighbor_coords[region].append(region_graph[region][nr][0])
            neighbor_coords[region].append(region_graph[region][nr][-1])

    move = dict()
    explored = set()
    frontier = [(start_pos, start_pos, 0, start_region, start_region)]
    while len(frontier) > 0:
        #frontier.sort(key=lambda x: x[2])
        head = frontier.pop(0)
        pos = head[1]
        explored.add(pos)
        frontier = [x for x in frontier if x[1] != pos]
        move[pos] = head[0]
        if region_map[pos[0], pos[1]] == goal_region:
            move[goal_pos] = pos
            break
        for coord in neighbor_coords[region_map[pos[0], pos[1]]]:
            if coord not in explored and region_map[coord[0], coord[1]] != head[3]:
                #frontier.append((pos, coord, head[2]+eucledian_distance(coord, pos), head[4], region_map[coord[0], coord[1]]))
                bisect.insort(frontier, (pos, coord, head[2]+eucledian_distance(coord, pos), head[4], region_map[coord[0], coord[1]]), key=lambda x: x[2])
    else:
        raise ValueError("No path found")
    path = []
    pos = goal_pos
    while pos != start_pos:
        path.append(pos)
        pos = move[pos]
    path.append(start_pos)

    print("Completing partial path")
    path_complete = complete_partial_path(path)
    time_taken_temp = time.time() - start_time_temp
    print("Total: %s seconds" % time_taken_temp)
    print()
    return strategy_name, path_complete, time_taken_temp, rects


def region_corner_plus_center_method(obstacle_map, start_pos, goal_pos, manual_onbstacle = None):
    strategy_name = "Corner cum Centre Points Method"
    print(strategy_name)
    if manual_onbstacle is None:
        corners = identify_corners(obstacle_map)
        ob_x, ob_y = extract_obstacle_edge_vertices(corners, obstacle_map.shape[0], obstacle_map.shape[1])
    else:
        ob_x, ob_y = [], []
        for obs in manual_onbstacle:
            bisect.insort(ob_x, obs[0])
            bisect.insort(ob_x, obs[0]+obs[2])
            bisect.insort(ob_y, obs[1])
            bisect.insort(ob_y, obs[1]+obs[3])
        if 0 not in ob_x:
            ob_x.insert(0, 0)
        if 0 not in ob_y:
            ob_y.insert(0, 0)
        if obstacle_map.shape[0] not in ob_x:
            ob_x.append(obstacle_map.shape[0])
        if obstacle_map.shape[1] not in ob_y:
            ob_y.append(obstacle_map.shape[1])
        # remove duplicates
        ob_x = list(dict.fromkeys(ob_x))
        ob_y = list(dict.fromkeys(ob_y))
    start_time_temp = time.time()
    print("Filling free space with rectangles")
    rects, region_map, region_graph = fill_free_space_with_rectangles(obstacle_map, ob_x, ob_y)
    #merge_rectangles(rects)
    print("Finding partial path")
    start_region = region_map[start_pos[0], start_pos[1]]
    goal_region = region_map[goal_pos[0], goal_pos[1]]
    neighbor_regions = {}
    for region in region_graph:
        neighbor_regions[region] = []
        for nr in region_graph[region]:
            neighbor_regions[region].append(nr)
    neighbor_coords = {}
    for region in region_graph:
        neighbor_coords[region] = []
        for nr in region_graph[region]:
            neighbor_coords[region].append(((rects[nr-2][1]+rects[nr-2][2])//2, (rects[nr-2][3]+rects[nr-2][4])//2))

    for region in region_graph:
        #neighbor_coords[region] = []
        for nr in region_graph[region]:
            neighbor_coords[region].append(region_graph[region][nr][0])
            neighbor_coords[region].append(region_graph[region][nr][-1])

    move = dict()
    explored = set()
    frontier = [(start_pos, start_pos, 0, start_region, start_region)]
    while len(frontier) > 0:
        #frontier.sort(key=lambda x: x[2])
        head = frontier.pop(0)
        pos = head[1]
        explored.add(pos)
        frontier = [x for x in frontier if x[1] != pos]
        move[pos] = head[0]
        if region_map[pos[0], pos[1]] == goal_region:
            move[goal_pos] = pos
            break
        for coord in neighbor_coords[region_map[pos[0], pos[1]]]:
            if coord not in explored and region_map[coord[0], coord[1]] != head[3]:
                #frontier.append((pos, coord, head[2]+eucledian_distance(coord, pos), head[4], region_map[coord[0], coord[1]]))
                bisect.insort(frontier, (pos, coord, head[2]+eucledian_distance(coord, pos), head[4], region_map[coord[0], coord[1]]), key=lambda x: x[2])
    else:
        raise ValueError("No path found")
    path = []
    pos = goal_pos
    while pos != start_pos:
        path.append(pos)
        pos = move[pos]
    path.append(start_pos)

    print("Completing partial path")
    path_complete = complete_partial_path(path)
    time_taken_temp = time.time() - start_time_temp
    print("Total: %s seconds" % time_taken_temp)
    print()
    return strategy_name, path_complete, time_taken_temp, rects

def show_regions(obstacle_map, start_pos, goal_pos, rects):
    plt.imshow(obstacle_map)
    for r in rects:
        plt.fill_between([r[3], r[4]], r[1], r[1]+1, color='red')
        plt.fill_between([r[3], r[4]], r[2]-1, r[2], color='red')
        plt.fill_betweenx([r[1], r[2]], r[3], r[3]+1, color='red')
        plt.fill_betweenx([r[1], r[2]], r[4]-1, r[4], color='red')
    plt.plot(start_pos[1], start_pos[0], 'ro')
    plt.plot(goal_pos[1], goal_pos[0], 'go')
    plt.gca().set_xlim([0, obstacle_map.shape[0]])
    plt.gca().set_ylim([0, obstacle_map.shape[1]])
    plt.show()


#manual_obstacle = [[np.random.randint(0, 998), np.random.randint(0, 998), np.random.randint(50, 80), np.random.randint(50, 80)] for _ in range(90)]
#print(manual_obstacle)
manual_obstacle = [[576, 87, 68, 51], [533, 504, 77, 73], [892, 633, 75, 72], [228, 457, 62, 65], [931, 151, 60, 76], [40, 985, 75, 71], [318, 911, 52, 56], [821, 244, 62, 59], [895, 581, 68, 60], [576, 876, 66, 65], [180, 679, 53, 65], [794, 168, 58, 70], [213, 849, 58, 79], [807, 812, 56, 64], [53, 785, 53, 70], [290, 924, 62, 71], [395, 488, 67, 63], [781, 707, 52, 61], [198, 723, 65, 69], [32, 478, 67, 54], [119, 889, 74, 54], [68, 104, 62, 76], [346, 648, 69, 72], [890, 562, 67, 70], [450, 777, 79, 50], [485, 596, 71, 79], [388, 10, 58, 72], [698, 809, 56, 52], [745, 60, 69, 55], [994, 401, 56, 52], [724, 993, 71, 55], [253, 609, 65, 51], [107, 273, 56, 55], [267, 848, 57, 78], [505, 10, 72, 56], [93, 492, 77, 52], [799, 984, 52, 79], [704, 794, 72, 77], [213, 397, 58, 62], [855, 662, 50, 66], [954, 923, 72, 77], [780, 74, 68, 66], [579, 779, 63, 55], [105, 452, 76, 58], [988, 693, 61, 79], [317, 385, 70, 55], [473, 641, 66, 74], [972, 877, 76, 61], [520, 916, 65, 62], [174, 208, 77, 71], [315, 25, 62, 66], [121, 996, 56, 69], [465, 365, 51, 64], [882, 335, 52, 56], [26, 661, 55, 60], [288, 728, 64, 62], [111, 341, 57, 61], [276, 130, 79, 78], [285, 269, 59, 55], [17, 32, 79, 60], [914, 613, 67, 50], [148, 366, 51, 79], [499, 965, 73, 52], [427, 890, 64, 65], [219, 451, 73, 78], [957, 344, 74, 57], [388, 523, 55, 72], [409, 757, 71, 66], [556, 576, 57, 50], [757, 202, 53, 70], [372, 776, 67, 60], [786, 473, 72, 59], [988, 469, 52, 72], [996, 667, 67, 69], [600, 660, 56, 63], [834, 50, 54, 61], [921, 424, 55, 58], [211, 379, 57, 71], [273, 480, 70, 78], [635, 452, 51, 50], [346, 686, 59, 79], [143, 136, 76, 68], [281, 499, 78, 63], [1, 978, 76, 62], [908, 594, 58, 77], [547, 961, 69, 67], [441, 959, 51, 63], [37, 704, 65, 52], [465, 521, 68, 69], [822, 699, 50, 71]]

fig, axs = plt.subplots(2, 4)

dim_x = 750
dim_y = 750
start_pos = (10, 10)
goal_pos = (650, 650)
manual_obstacle = [x for x in manual_obstacle if x[0] < dim_x and x[1] < dim_y and x[0]+x[2] < dim_x and x[1]+x[3] < dim_y]
obstacle_map, start_pos, goal_pos = generate_problem(dim_x= dim_x, dim_y=dim_y, manual_onbstacle=manual_obstacle,start_pos=start_pos, goal_pos=goal_pos)
strategy_name1, path1, time_taken_temp1  = dijkstra(obstacle_map, start_pos, goal_pos)
strategy_name2, path2, time_taken_temp2, rects = region_border_method(obstacle_map, start_pos, goal_pos, manual_onbstacle=manual_obstacle)
strategy_name3, path3, time_taken_temp3, rects = region_corner_method(obstacle_map, start_pos, goal_pos, manual_onbstacle=manual_obstacle)
strategy_name4, path4, time_taken_temp4, rects = region_corner_plus_center_method(obstacle_map, start_pos, goal_pos, manual_onbstacle=manual_obstacle)
length1 = calcutalte_path_length(path1)
length2 = calcutalte_path_length(path2)
length3 = calcutalte_path_length(path3)
length4 = calcutalte_path_length(path4)
show_path(obstacle_map, start_pos, goal_pos, path1, title=strategy_name1, caption="Time taken: "+str(round(time_taken_temp1,2))+" seconds\nDistance: "+str(round(length1,2)), plt=axs[0][0])
show_path(obstacle_map, start_pos, goal_pos, path2, title=strategy_name2, caption="TIme taken: "+str(round(time_taken_temp2,2))+" seconds\nDistance: "+str(round(length2,2)), plt=axs[0][1], rects=rects)
show_path(obstacle_map, start_pos, goal_pos, path3, title=strategy_name3, caption="Time taken: "+str(round(time_taken_temp3,2))+" seconds\nDistance: "+str(round(length3,2)), plt=axs[0][2], rects=rects)
show_path(obstacle_map, start_pos, goal_pos, path4, title=strategy_name4, caption="Time taken: "+str(round(time_taken_temp4,2))+" seconds\nDistance: "+str(round(length4,2)), plt=axs[0][3], rects=rects)

dim_x = 500
dim_y = 500
start_pos = (10, 10)
goal_pos = (450, 450)
manual_obstacle = [x for x in manual_obstacle if x[0] < dim_x and x[1] < dim_y and x[0]+x[2] < dim_x and x[1]+x[3] < dim_y]
obstacle_map, start_pos, goal_pos = generate_problem(dim_x= dim_x, dim_y=dim_y, manual_onbstacle=manual_obstacle,start_pos=start_pos, goal_pos=goal_pos)
strategy_name1, path1, time_taken_temp1  = dijkstra(obstacle_map, start_pos, goal_pos)
strategy_name2, path2, time_taken_temp2, rects = region_border_method(obstacle_map, start_pos, goal_pos, manual_onbstacle=manual_obstacle)
strategy_name3, path3, time_taken_temp3, rects = region_corner_method(obstacle_map, start_pos, goal_pos, manual_onbstacle=manual_obstacle)
strategy_name4, path4, time_taken_temp4, rects = region_corner_plus_center_method(obstacle_map, start_pos, goal_pos, manual_onbstacle=manual_obstacle)
length1 = calcutalte_path_length(path1)
length2 = calcutalte_path_length(path2)
length3 = calcutalte_path_length(path3)
length4 = calcutalte_path_length(path4)
show_path(obstacle_map, start_pos, goal_pos, path1, title=strategy_name1, caption="Time taken: "+str(round(time_taken_temp1,2))+" seconds\nDistance: "+str(round(length1,2)), plt=axs[1][0])
show_path(obstacle_map, start_pos, goal_pos, path2, title=strategy_name2, caption="TIme taken: "+str(round(time_taken_temp2,2))+" seconds\nDistance: "+str(round(length2,2)), plt=axs[1][1], rects=rects)
show_path(obstacle_map, start_pos, goal_pos, path3, title=strategy_name3, caption="Time taken: "+str(round(time_taken_temp3,2))+" seconds\nDistance: "+str(round(length3,2)), plt=axs[1][2], rects=rects)
show_path(obstacle_map, start_pos, goal_pos, path4, title=strategy_name4, caption="Time taken: "+str(round(time_taken_temp4,2))+" seconds\nDistance: "+str(round(length4,2)), plt=axs[1][3], rects=rects)
show_regions(obstacle_map, start_pos, goal_pos, rects)
fig.tight_layout()
plt.show()
