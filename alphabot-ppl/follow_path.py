from grid_nav import *
import os
import subprocess
from self_locator import *
from micro_controller import *
import yaml
import math
import json
import time
import signal
import sys
import requests
import numpy as numpy  
CONFIG = yaml.load(open("../config.yaml"))

# Example Diagram with obstacles
# DIAGRAM1_S = (1,4)
# DIAGRAM1_G = (7,8)
# DIAGRAM1 = GridWithWeights(10, 10)
# DIAGRAM1.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
# DIAGRAM1.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
#                                        (4, 3), (4, 4), (4, 5), (4, 6), 
#                                        (4, 7), (4, 8), (5, 1), (5, 2),
#                                        (5, 3), (5, 4), (5, 5), (5, 6), 
#                                        (5, 7), (5, 8), (6, 2), (6, 3), 
#                                        (6, 4), (6, 5), (6, 6), (6, 7), 
#                                        (7, 3), (7, 4), (7, 5)]}

# (Column, Row)
GRID_COLUMNS = CONFIG["grid"]["columns"]
GRID_ROWS = CONFIG["grid"]["columns"]

# BEACON_COLORS = [0, 1, 2, 3, 4]
BEACON_COLUMNS = CONFIG["grid"]["beacons_columns"]
BEACON_ROWS = CONFIG["grid"]["beacons_rows"]
CELL_SIZE = CONFIG["grid"]["cell_size"]

DIAGRAM2 = GridWithWeights(GRID_COLUMNS, GRID_ROWS)
#TODO put walls at config file
DIAGRAM2.walls = [(1, 0), (1, 1), (2, 0), (2, 1), (0, 3), (0, 4),(1,3),(1,4)]
DIAGRAM2_S = CONFIG["grid"]["start"] # Start
DIAGRAM2_G = CONFIG["grid"]["goal"] # Goal
ORIENT_REF_ROW = CONFIG["grid"]["orientation_reference_row"] # Red Beacon
offload_dijkstra = CONFIG["offload_path_planning"]
S1 = str(CONFIG["camera"]["vertical_servo_pin"])


# Rotate AlphaBot according to its new desired orientation
def change_orientation(mc, co, no):
    print("Orientation should be: " + str(co- no) + "deg, but is: " + str(co) + "deg.")
    deg_diff = no 
    # Rotational movements below 10deg are imprecise
    if (math.fabs(deg_diff) > 10):
        print("Fix: Rotating " + str(int(deg_diff)) + "deg...\n")
        print("---------- Micro Controller Logs -------------")
        r = mc.move_and_control([0, 0, 0, 0, 0, deg_diff])
        return_orientation = r[2]
        print("----------------------------------------------\n")
    else:
        print("Error too small. Ignoring...")
        return_orientation = deg_diff 
    return return_orientation
    
# Move AlphaBot one tile forward
def move_forward(mc, distance):
    print("Moving forward for " + str(round(distance, 2)) + "m...\n")
    pos_arry = [0, 0, 0, distance, 0, 0]
    print("---------- Micro Controller Logs -------------")
    r = mc.move_and_control(pos_arry)
    print("----------------------------------------------\n")
    return r

# Return estimated AlphaBot's position in grid (column, row, orientation)
def self_localize(self_locator):
    print("Self Localising...")
    start_time = time.time()
    b_distance, b_angle, b_color = self_locator.dna_from_beacons()
    end_time = time.time() -start_time
    print "Computational time "+str(end_time)
    x, y, column, row = detect_position_in_grid(b_distance, b_color)
    # angle from the first beacon is enough
    orientation = detect_orientation(x, y, b_distance[0], b_angle[0], b_color[0]) 
    # print x, y, column, row, orientation
    return x, y, column, row, orientation

# Detect AlphaBot's orientation (degrees) relevant to a reference point in grid
def detect_orientation(ax, ay, distance, theta_beac, color):
    rx = ORIENT_REF_ROW * CELL_SIZE + CELL_SIZE / 2
    ry = ay
    bx = BEACON_ROWS[color] * CELL_SIZE + CELL_SIZE / 2
    by = BEACON_COLUMNS[color] * CELL_SIZE + CELL_SIZE / 2 

    # beacon - alphabot distance
    bad = round(distance)
    print("Beacon - AlphaBot distance: " + str(bad) + "cm.")
    # beacon - ref distance
    brd = round(math.sqrt((bx - rx) ** 2 + (by - ry) ** 2))
    print("Beacon - Reference Point distance: " + str(brd) + "cm.")
    # alphabot - ref distance
    ard = round(math.sqrt((ax - rx) ** 2 + (ay - ry) ** 2))
    print("AlphaBot - Reference Point distance: " + str(ard) + "cm.")

    # Cosine Rule
    a = round(math.degrees(math.acos((bad ** 2 + ard ** 2 - brd ** 2) / (2 * bad * ard))), 2)
    print("Cosine Rule Angle: " + str(a) + "deg.")
    
    # theta_or < 0 if the reference point is on the right of the alphabot, > 0 otherwise 
    # if ((by < ry) and (theta_beac > 0)):
    #     theta_or = theta_beac + a
    # elif ((by < ry) and (theta_beac < 0)):
    #     theta_or = theta_beac + a 
    # elif ((by > ry) and (theta_beac > 0)):
    #     theta_or = theta_beac - a
    # elif ((by > ry) and (theta_beac < 0)):
    #     theta_or = theta_beac - a
    theta_or = theta_beac - ((by-ry)/math.fabs(by-ry)) * a
    print("Orientation Angle: " + str(-theta_or) + "deg.")

    return theta_or

# Detect AlphaBot's position in grid (columns, rows)
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
    x3a = round(x2 + h * (y1 - y0) / d, 2)     
    print("IP1 x = " + str(x3a))
    x3b = round(x2 - h * (y1 - y0) / d, 2)
    print("IP2 x = " + str(x3b))
    y3a = round(y2 - h * (x1 - x0) / d, 2)
    print("IP1 y = " + str(y3a))
    y3b = round(y2 + h * (x1 - x0) / d, 2)
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
    
    return x3, y3, column, row

# A dp path planning algorithm (either dijkstra or a*)
def plan_the_path(start, goal):
    came_from, cost_so_far = a_star_search(DIAGRAM2, start, goal)
    draw_grid(DIAGRAM2, width=3, point_to=came_from, start=start, goal=goal)
    print("")
    draw_grid(DIAGRAM2, width=3, number=cost_so_far, start=start, goal=goal)
    print("")
    draw_grid(DIAGRAM2, width=3, path=reconstruct_path(came_from, start = start, goal = goal))
    path = reconstruct_path(came_from, start = start, goal = goal)
    print(path)

    return path

def signal_handler(sig, frame):
    print(' was pressed! Fixing camera position and terminating...\n')
    with open(os.devnull, 'wb') as devnull:
        subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', S1, '-w', '1600'], 
            stdout=devnull, stderr=subprocess.STDOUT)
    sys.exit(0)

def main():
    os.system('clear')
    signal.signal(signal.SIGINT, signal_handler)
    print("-----------------------------------------------------------------------------------")
    print("--------------------------------- ALPHABOT-PPL ------------------------------------") 
    print("-----------------------------------------------------------------------------------\n")
    print("Initiating...\n")
    sl = SelfLocator(300)
    mc = MicroControler()
    try:
        x, y, i, j, co = self_localize(sl)
    except InsufficientLocalizationInfoError:
        print("I have no information regarding where I am. Quiting...\n")
        quit()
    if offload_dijkstra == 1 :
        path = plan_the_path((i,j), DIAGRAM2_G)
    else:
        start_time = time.time()
        post_url =  "http://192.168.1.114:8000/dijkstra"
        payload = [{ "i" : int(i) , "j": int(j), "x" : x, "y":y, "or": co}]
        co = -co
        r = requests.post(post_url,json=payload)
        a,b = json.loads(r.text)
        temp = [int(a),int(b)]
        path = []
        start_position = [int(y/CELL_SIZE), int (x/CELL_SIZE) ]
        path.append(start_position)
        path.append(temp)
        print "Total time for dijkstra: "+ str(time.time()-start_time)

    # Start following path
    print("Starting to follow path...")
    cp = path.pop(0) # Get starting position
    if offload_dijkstra == 0:
        np = path.pop(0)
    while True : # While current tile is different from target's tile:
        # 1: check if re-calculation of the path is needed
        print ("Thinking that I am in position: (" + str(int(cp[0])) + ", " + str(int(cp[1])) + ")")
        print ("I actually am at: (" + str(int(i)) + ", " + str(int(j)) + "), or (" + 
            str(int(x)) + ", " + str(int(y)) + ") in cm. ")
        if (((i != cp[0]) or (j != cp[1])) and offload_dijkstra == 1):
            print "So, I need to plan the path again.\n"
            path = plan_the_path((i,j), DIAGRAM2_G)
            cp = path.pop(0)
            continue
        else:
            if offload_dijkstra == 1:
                print "So, no need for planning the path again.\n"
                np = path.pop(0)
        
        # 2: check about extra orientation turn
        print "Checking for extra turn (a change in path orientation)..."
        
        # 3: change the orientation; use the Pythagorean Theorem for calculating the new required distance 
        # and angle to move to the next tile
        a = x - (np[1] * CELL_SIZE + CELL_SIZE / 2)
        b = y - (np[0] * CELL_SIZE + CELL_SIZE / 2)
        distance = math.sqrt(a ** 2 + b ** 2) / 100 # distance is needed in m.
        # Find new Point in the line of current orientation.
	x1 = x + 10*cos(co*pi/180) 
	y1 = y + 10*sin(co*pi/180)
	# x goes for j , y goes for i 
	xtarget = np[1] * CELL_SIZE + CELL_SIZE/2
	ytarget = np[0] * CELL_SIZE + CELL_SIZE/2
	# A = (x,y) , B = (x1,y1) , C= (xtarget,ytarget)
	# find distance of the sides a_side = BC , b_side = AC , c_side = AB
	a_side = math.sqrt((x1-xtarget)**2 + (y1-ytarget)**2) 
	b_side = math.sqrt((x-xtarget)**2 + (y-ytarget)**2)
	c_side = math.sqrt((x-x1)**2 + (y-y1)**2)
	# Law of cosines a**2 = c**2 + b**2 - 2cbcosw , where w is the angle of A
	w = math.acos(float(-(a_side**2) + c_side**2 + b_side**2)/(float(2*c_side*b_side))) 
    	# We need also the slope to determine to whick way the Robot must rotate
	a = numpy.array([x1-x,y1-y])
        b = numpy.array([xtarget-x,ytarget-y])
        sign =  numpy.cross(a,b) # cross product of the two vectors
        no =  w * 180/pi* sign/abs(sign)
        
        #raw_input("Press Enter to continue...\n")
        temp_ori = change_orientation(mc, int(co), int(no))
        # 4: move a tile forward and at the same time measure the distance and angle reported by the light 
        # sensors (encoders) during the last step. Use this information to get an estimation on the grid
        #raw_input("Press Enter to continue...\n")
        x_enc, y_enc, theta_enc = move_forward(mc, distance)
        #raw_input("Press Enter to continue...\n")
        # 5: acquire estimated position on the grid
        try:
            x, y, i, j, co = self_localize(sl)
            co=-co
        except (InsufficientLocalizationInfoError, ValueError):
            # At this stage, the (blind) AlphaBot has no environmental information regarding its position.
            # It now relys solely on the information coming from the encoders. 
            print("Beacon information not sufficient for localizing. Now using encoders' output...")
            print("Last known beacon-based position estimation: Row = "+ str(int(i)) + ", Column = " + 
                    str(int(j)) + ", x = " + str(int(x)) + "cm , y = " + str(int(y)) + 
                    "cm and Orientation = " + str(int(co)) + "deg")
            # rotation of axis so we need to compute the real x,y diff
            co += (temp_ori + theta_enc)
            theta = math.radians(co)
            print theta
            print x_enc,y_enc,theta_enc
            print  str(int(x)) , str(int(x_enc)*math.cos(theta)) , str(int(y_enc)*math.sin(theta) )
            x = int(x) + int(x_enc)*math.cos(theta) - int(y_enc)*math.sin(theta) 
            y = int(y) + int(x_enc)*math.sin(theta) + int(y_enc)*math.cos(theta)
            i = y // CELL_SIZE
            j = x // CELL_SIZE
            print("Current encoder-based position estimation: Row = "+ str(int(i)) + ", Column = " + 
                    str(int(j)) + ", x = " + str(int(x)) + "cm , y = " + str(int(y)) + 
                    "cm and orientation = " + str(int(co)) + "deg")
            # Current position in grid is last step's next position
        if offload_dijkstra !=1 :
            start_time = time.time()    
            post_url =  "http://192.168.1.114:8000/dijkstra"
            payload = [{ "i" : int(i) , "j": int(j), "x" : x, "y":y, "or": co}]
            r = requests.post(post_url,json=payload)
            a,b = json.loads(r.text)
            temp = [int(a),int(b)]
            path = []
            start_position = [int(y/CELL_SIZE), int (x/CELL_SIZE) ]
            path.append(start_position)
            path.append(temp)
            print "Total time for dijkstra: "+ str(time.time()-start_time)
            if start_position == temp:
                break
            print path
            cp = path.pop(0)
            np = path.pop(0)

        else : # local execution of dijkstra
            cp = np
            if cp==np :
                break
        

        print("\n---------------------------------")
        print("End of Step")
        print("---------------------------------\n")
    print("\n\n\n ---------------------- GOAL REACHED (or at least I think so) ----------------------\n\n\n ")

if __name__ == "__main__":
    main()
