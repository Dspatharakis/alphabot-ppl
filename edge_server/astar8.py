# A* Shortest Path Algorithm
# http://en.wikipedia.org/wiki/A*
# FB - 201012256
from heapq import heappush, heappop # for priority queue
import math
import time
import random
import numpy as np 


class node:
    xPos = 0 # x position
    yPos = 0 # y position
    distance = 0 # total distance already travelled to reach the node
    priority = 0 # priority = distance + remaining distance estimate
    def __init__(self, xPos, yPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        self.distance = distance
        self.priority = priority
    def __lt__(self, other): # comparison method for priority queue
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*
    # give higher priority to going straight instead of diagonally
    def nextMove(self, dirs, d): # d: direction to move
        if dirs == 8 and d % 2 != 0:
            self.distance += math.sqrt(2)
        else:
            self.distance += 1
    # Estimation function for the remaining distance to the goal.
    def estimate(self, xDest, yDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        # Euclidian Distance
        d = math.sqrt(xd * xd + yd * yd)
        # Manhattan distance
        # d = abs(xd) + abs(yd)
        # Chebyshev distance
        # d = max(abs(xd), abs(yd))
        return(d)

# A-star algorithm.
# The path returned will be a string of digits of directions.
def pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB):
    closed_nodes_map = [] # map of closed (tried-out) nodes
    open_nodes_map = [] # map of open (not-yet-tried) nodes
    dir_map = [] # map of dirs
    row = [0] * n
    for i in range(m): # create 2d arrays
        closed_nodes_map.append(list(row))
        open_nodes_map.append(list(row))
        dir_map.append(list(row))

    pq = [[], []] # priority queues of open (not-yet-tried) nodes
    pqi = 0 # priority queue index
    # create the start node and push into list of open nodes
    n0 = node(xA, yA, 0, 0)
    n0.updatePriority(xB, yB)
    heappush(pq[pqi], n0)
    open_nodes_map[yA][xA] = n0.priority # mark it on the open nodes map

    # A* search
    while len(pq[pqi]) > 0:
        # get the current node w/ the highest priority
        # from the list of open nodes
        n1 = pq[pqi][0] # top node
        n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
        x = n0.xPos
        y = n0.yPos
        heappop(pq[pqi]) # remove the node from the open list
        open_nodes_map[y][x] = 0
        closed_nodes_map[y][x] = 1 # mark it on the closed nodes map

        # quit searching when the goal is reached
        # if n0.estimate(xB, yB) == 0:
        if x == xB and y == yB:
            # generate the path from finish to start
            # by following the dirs
            path = ''
            while not (x == xA and y == yA):
                j = dir_map[y][x]
                c = str((j + dirs / 2) % dirs)
                path = c + path
                x += dx[j]
                y += dy[j]
            return path,n1.distance

        # generate moves (child nodes) in all possible dirs
        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m - 1
                    or the_map[ydy][xdx] == 1 or closed_nodes_map[ydy][xdx] == 1):
                # generate a child node
                m0 = node(xdx, ydy, n0.distance, n0.priority)
                m0.nextMove(dirs, i)
                m0.updatePriority(xB, yB)
                # if it is not in the open list then add into that
                if open_nodes_map[ydy][xdx] == 0:
                    open_nodes_map[ydy][xdx] = m0.priority
                    heappush(pq[pqi], m0)
                    # mark its parent node direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                elif open_nodes_map[ydy][xdx] > m0.priority:
                    # update the priority
                    open_nodes_map[ydy][xdx] = m0.priority
                    # update the parent direction
                    dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                    # replace the node
                    # by emptying one pq to the other one
                    # except the node to be replaced will be ignored
                    # and the new node will be pushed in instead
                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1 - pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi]) # remove the target node
                    # empty the larger size priority queue to the smaller one
                    if len(pq[pqi]) > len(pq[1 - pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])       
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0) # add the better node instead
    return '' # if no route found

# MAIN
def Astar8(n,start,goal):
    xA = start[1]
    yA = start[0]
    xB = goal[1]
    yB = goal[0]
    obstacles=[[[0,90],[99,110]],[[176,0],[249,99]]]#,[[290,567],[420,678]],[[790,67],[920,150]],[[600,607],[780,780]],[[760,67],[870,150]]]#,[[560,900],[660,990]],[[1600,1607],[1780,1780]],[[1760,1907],[1870,1500]],[[2060,200],[2160,2390]]]
    CELL_SIZE = 5
    s = (n,n) 
    lineofsightmatrix =np.zeros((s),dtype=int)
    for rect in obstacles:
        #print rect
        minobi = rect[0][1]/CELL_SIZE
        minobj = rect[0][0]/CELL_SIZE
        maxobi = rect[1][1]/CELL_SIZE
        maxobj = rect[1][0]/CELL_SIZE
        for j in range (minobj,maxobj+1):
            for i in range (minobi,maxobi+1):
                lineofsightmatrix[i][j]=1

    dirs = 8 # number of possible directions to move on the map
    if dirs == 4:
        dx = [1, 0, -1, 0]
        dy = [0, 1, 0, -1]
    elif dirs == 8:
        dx = [1, 1, 0, -1, -1, -1, 0, 1]
        dy = [0, 1, 1, 1, 0, -1, -1, -1]

    n = n # horizontal size of the map
    m = n # vertical size of the map
    the_map = []
    row = [0] * n
    for i in range(m): # create empty map
        the_map.append(list(row))

    '''
    # fillout the map with a '+' pattern
    for x in range(n / 8, n * 7 / 8):
        the_map[m / 2][x] = 1
    for y in range(m/8, m * 7 / 8):
        the_map[y][n / 2] = 1
    '''
    the_map=lineofsightmatrix
    # randomly select start and finish locations from a list
    '''
    sf = []
    sf.append((0, 0, n - 1, m - 1))
    sf.append((0, m - 1, n - 1, 0))
    sf.append((n / 2 - 1, m / 2 - 1, n / 2 + 1, m / 2 + 1))
    sf.append((n / 2 - 1, m / 2 + 1, n / 2 + 1, m / 2 - 1))
    sf.append((n / 2 - 1, 0, n / 2 + 1, m - 1))
    sf.append((n / 2 + 1, m - 1, n / 2 - 1, 0))
    sf.append((0, m / 2 - 1, n - 1, m / 2 + 1))
    sf.append((n - 1, m / 2 + 1, 0, m / 2 - 1))
    (xA, yA, xB, yB) = random.choice(sf)
    '''
    print 'Map size (X,Y): ', n, m
    print 'Start: ', xA, yA
    print 'Finish: ', xB, yB
    t = time.time()
    route, cost = pathFind(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB)
    print 'Time to generate the route (seconds): ', time.time() - t
    print 'Route:'
    print route
    print len(route)
    print cost

    return route,cost
    # mark the route on the map
    '''
    if len(route) > 0:
        x = xA
        y = yA
        the_map[y][x] = 2
        for i in range(len(route)):
            j = int(route[i])
            x += dx[j]
            y += dy[j]
            the_map[y][x] = 3
        the_map[y][x] = 4

    # display the map with the route added
    
    print 'Map:'
    for y in range(m):
        for x in range(n):
            xy = the_map[y][x]
            if xy == 0:
                print '.', # space
            elif xy == 1:
                print 'O', # obstacle
            elif xy == 2:
                print 'S', # start
            elif xy == 3:
                print 'R', # route
            elif xy == 4:
                print 'F', # finish
        print

    raw_input('Press Enter...')
    '''
