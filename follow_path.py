from simple_grid_nav import *
import os
import subprocess
from self_locate import *

DIAGRAM1 = GridWithWeights(10, 10)
DIAGRAM1.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
DIAGRAM1.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6), 
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6), 
                                       (5, 7), (5, 8), (6, 2), (6, 3), 
                                       (6, 4), (6, 5), (6, 6), (6, 7), 
                                       (7, 3), (7, 4), (7, 5)]}
# (Column, Row)
GRID_COLUMNS = 5
GRID_ROWS = 5

BEACON_COLORS = [0, 1, 2, 3, 4]
BEACON_COLUMNS = [2, 4, 0, 4, 0]
BEACON_ROWS = [4, 2, 4, 4, 2]
CELL_SIZE = 50

DIAGRAM1_S = (1,4)
DIAGRAM1_G = (7,8)

DIAGRAM2 = GridWithWeights(GRID_COLUMNS, GRID_ROWS)
DIAGRAM2_S = (2,0)
DIAGRAM2_G = (2,3)

RIGHT = 4
LEFT = 2
DOWN = 3
UP = 1

def change_orientation(co, no):
    diff = no - co
    if (diff == 1 or diff == 3):
        # rotate left
        print("Rotating LEFT!")
        with open(os.devnull, 'wb') as devnull:
            subprocess.check_call(['python', 'turn_left.py'], stdout=devnull, stderr=subprocess.STDOUT)
    elif (diff == -1 or diff == -3):
        print("Rotating RIGHT!")
        with open(os.devnull, 'wb') as devnull:
            subprocess.check_call(['python', 'turn_right.py'], stdout=devnull, stderr=subprocess.STDOUT)
    # Can only happen during first move
    elif (diff == 2 or diff == -2):
        print("Rotating BACKWARDS!")
        with open(os.devnull, 'wb') as devnull:
            subprocess.check_call(['python', 'turn_left.py'], stdout=devnull, stderr=subprocess.STDOUT)           
            subprocess.check_call(['python', 'turn_left.py'], stdout=devnull, stderr=subprocess.STDOUT)
    else:
        print("No need for rotation...")

def move_forward():
    print("Moving FORWARD!")
    with open(os.devnull, 'wb') as devnull:
        subprocess.check_call(['python', 'velocity.py', '-51', '45'], stdout=devnull, stderr=subprocess.STDOUT)

def self_localize(self_locator):
    print("Self Localising...")
    distance, angle, color = self_locator.dna_from_beacons()
    c, r = detect_position_in_grid(distance, color)
    print c,r
    return c, r

def detect_position_in_grid(distance, color):
    r0 = distance[0]
    print("R0 = " + str(r0))
    r1 = distance[1]
    print("R1 = " + str(r1))
    x0 = BEACON_ROWS[color[0]] * CELL_SIZE + CELL_SIZE / 2 # because it sits in the cell centre
    print("x0 = " + str(x0))
    y0 = BEACON_COLUMNS[color[0]] * CELL_SIZE + CELL_SIZE / 2
    print("y0 = " + str(y0))
    x1 = BEACON_ROWS[color[1]] * CELL_SIZE + CELL_SIZE / 2
    print("x1 = " + str(x1))  
    y1 = BEACON_COLUMNS[color[1]] * CELL_SIZE + CELL_SIZE / 2
    print("y1 = " + str(y1))
    
    # Circle - Circle Intersection calculation
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(r0 ** 2 - a ** 2)
    x2 = x0 + a * (x1 - x0) / d   
    y2 = y0 + a * (y1 - y0) / d   
    x3a = x2 + h * (y1 - y0) / d     
    print("IP1 x = " + str(x3a))
    x3b = x2 - h * (y1 - y0) / d
    print("IP2 x = " + str(x3b))
    y3a = y2 - h * (x1 - x0) / d
    print("IP1 y = " + str(y3a))
    y3b = y2 + h * (x1 - x0) / d
    print("IP2 y = " + str(y3b))

    if ((x3a > GRID_ROWS*CELL_SIZE) or (y3a > GRID_COLUMNS*CELL_SIZE)):   
        if ((x3b <= GRID_ROWS*CELL_SIZE) or (y3b <= GRID_COLUMNS*CELL_SIZE)):  
            x3 = x3b 
            y3 = y3b
        else:
            print("Invalid Intersection Point found!")
            
    elif ((x3a < GRID_ROWS*CELL_SIZE) and (y3a < GRID_COLUMNS*CELL_SIZE)):   
        x3 = x3a
        y3 = y3a
    else:
        print("Invalid Intersection Point found")

    print("IP x = " + str(x3))
    print("IP y = " + str(y3))
    row = x3 // CELL_SIZE
    column = y3 // CELL_SIZE
    
    return column, row

def main():
    sl = SelfLocator(300)
    # Solve A* and visualise path
    came_from, cost_so_far = a_star_search(DIAGRAM2, DIAGRAM2_S, DIAGRAM2_G)
    draw_grid(DIAGRAM2, width=3, point_to=came_from, start=DIAGRAM2_S, goal=DIAGRAM2_G)
    print("")
    draw_grid(DIAGRAM2, width=3, number=cost_so_far, start=DIAGRAM2_S, goal=DIAGRAM2_G)
    print("")
    draw_grid(DIAGRAM2, width=3, path=reconstruct_path(came_from, start = DIAGRAM2_S, goal = DIAGRAM2_G))
    path = reconstruct_path(came_from, start = DIAGRAM2_S, goal = DIAGRAM2_G)
    print(path)

    # Start following path
    print("Start following path...")
    cp = path.pop(0) # Get starting position
    co = DOWN # Assume that starting orientation is down
    move = 0
    while path: # While path_list not empty
        move += 1 
        #print(path)
	np = path.pop(0) # Get next position
	# Define new orientation
        if (np[0] - cp[0] == 1): # => no = RIGHT
            print("RIGHT")
            no = RIGHT # new orientation
        elif (np[0] - cp[0] == -1): # => no = LEFT
            print("LEFT")
            no = LEFT
        elif (np[1] - cp[1] == 1):
            print("DOWN")
            no = DOWN
        elif (np[1] - cp[1] == -1):
            print("UP")
            no = UP
        else:
            print("Invalid Next Move")
        change_orientation(co, no)
        cp = np
        co = no
        # move_forward()
        self_localize(sl)
        print("_________________________________")
        print("End of Step")
        print("---------------------------------")
    print("Goal Reached")

if __name__ == "__main__":
    main()
