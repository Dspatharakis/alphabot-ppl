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
import csv

CONFIG = yaml.load(open("../config.yaml"))

INITIAL_TIME = time.time()
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
DIAGRAM2.walls = [(9, 7), (10, 7), (11, 7), (12, 7), (13, 7), (14, 7),(15,7),(16,7), 
(9, 8), (10, 8), (11, 8), (12, 8), (13, 8), (14, 8),(15,8),(16,8),
(9, 9), (10, 9), (11, 9), (12, 9), (13, 9), (14, 9),(15,9),(16,9),
(9, 10), (10, 10), (11, 10), (12, 10), (13, 10), (14, 10),(15,10),(16,10),
(11, 18), (12, 18), (13, 18), (14, 18),
(11, 19), (12, 19), (13, 19), (14, 19),
(11, 20), (12, 20), (13, 20), (14, 20),
(11, 21), (12, 21), (13, 21), (14, 21),
(11, 22), (12, 22), (13, 22), (14, 22),
(11, 23), (12, 23), (13, 23), (14, 23)]
DIAGRAM2_G = CONFIG["grid"]["goal"] # Goal
ORIENT_REF_ROW = CONFIG["grid"]["orientation_reference_row"] # Red Beacon
S1 = str(CONFIG["camera"]["vertical_servo_pin"])
obstacles = DIAGRAM2.walls

# Rotate AlphaBot according to its new desired orientation
def change_orientation(mc, co, no, x, y):
    #print("Orientation should be: " + str(co- no) + "deg, but is: " + str(co) + "deg.")
    deg_diff = no 
    # Rotational movements below 10deg are imprecise
    if (math.fabs(deg_diff) > 10):
        #print("Fix: Rotating " + str(int(deg_diff)) + "deg...\n")
        print("---------- Micro Controller Logs -------------")
        r = mc.move_and_control([0, 0, 0, 0, 0, deg_diff])
        return_orientation = r[2]
	f_list = []
	f_list = [f*(180/math.pi) for f in r[9]]
	filename = "./stats/localcontroller.csv"
	R = [0] * len(r[11])
	F = []
	for i in range (len(r[11])):
	    F.append(f_list[i] - no)
        with open(filename, 'a') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
	    
            # If opened for the first time, insert header row
            if os.path.getsize(filename) == 0:
                wr.writerow(["timestamp","wr","wl","ur","ul","x1","x2","x3","duration","R","F"])
	    for i in range (len(r[11])):
                wr.writerow(["{0:0.5f}".format(r[11][i]-INITIAL_TIME),"{0:0.5f}".format(r[3][i]),"{0:0.5f}".format(r[4][i]),"{0:0.5f}".format(r[5][i]),"{0:0.5f}".format(r[6][i]),"{0:0.5f}".format(x),"{0:0.5f}".format(y),"{0:0.5f}".format(f_list[i]),"{0:0.5f}".format(r[10][i]),"{0:0.5f}".format(R[i]),"{0:0.5f}".format(F[i])])

        print("----------------------------------------------\n")
    else:
        print("Error too small. Ignoring...")
        return_orientation = deg_diff 
    return return_orientation
    
# Move AlphaBot one tile forward
def move_forward(mc, distance,temp_ori, co, x ,y):
    print("Moving forward for " + str(round(distance, 2)) + "m...\n")
    pos_arry = [0, 0, 0, distance, 0, 0]
    print("---------- Micro Controller Logs -------------")
    r = mc.move_and_control(pos_arry)
    co += (temp_ori + r[2])
    theta = math.radians(co)
    x_enc = r[0]
    y_enc = r[1]
    x_list = []
    y_list = []
    f_list = []
    x_list= [int(x) + int(x_enc)*math.cos(theta) - int(y_enc)*math.sin(theta) for x in r[7]]
    y_list = [int(y) + int(x_enc)*math.sin(theta) + int(y_enc)*math.cos(theta) for y in r[8]]    	
    f_list= [f*(180/math.pi) for f in r[9]]
    R = []
    F = []
    for i in range (len(r[7])):
        R.append(math.sqrt( (x_list[i] - x)**2 + (y_list[i] - y)**2 ))
        F.append(f_list[i])
	 
    filename = "./stats/localcontroller.csv"
    with open(filename, 'a') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        # If opened for the first time, insert header row
        if os.path.getsize(filename) == 0:
	    wr.writerow(["timestamp","wr","wl","ur","ul","x1","x2","x3","duration","R","F"])
	for i in range (len(r[11])):
            wr.writerow(["{0:0.5f}".format(r[11][i]-INITIAL_TIME),"{0:0.5f}".format(r[3][i]),"{0:0.5f}".format(r[4][i]),"{0:0.5f}".format(r[5][i]),"{0:0.5f}".format(r[6][i]),"{0:0.5f}".format(x_list[i]),"{0:0.5f}".format(y_list[i]),"{0:0.5f}".format(f_list[i]),"{0:0.5f}".format(r[10][i]),"{0:0.5f}".format(R[i]),"{0:0.5f}".format(F[i])])


    print("----------------------------------------------\n")
    return r[0],r[1],r[2]

# Return estimated AlphaBot's position in grid (column, row, orientation)
def self_localize(self_locator,offload):
    print("Self Localising...")
    start_time = time.time()
    b_distance, b_angle, b_color, z0, x0, cores, comput_time = self_locator.dna_from_beacons(offload)
    end_time = time.time() -start_time
    print "Computational time for Total Image Processing "+str(end_time)
    x, y, column, row = detect_position_in_grid(b_distance, b_color)
    # angle from the first beacon is enough
    orientation = detect_orientation(x, y, b_distance[0], b_angle[0], b_color[0]) 
    # print x, y, column, row, orientation
    return x, y, column, row, orientation, z0, x0, cores, comput_time 

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

def switch_image_processing(x0):
    t_server = -1.34* float(x0) + 1.675
    if t_server < 1.5: 
        S2 = True  # True goes to server 
    else:
	S2 = False
    filename = "./stats/switch2.csv"
    with open(filename, 'a') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL) 
        # If opened for the first time, insert header row
        if os.path.getsize(filename) == 0:
	    wr.writerow(["timestamp","S2","Kalman_Estimaton_of_cores"])
        wr.writerow(["{0:0.5f}".format(time.time()-INITIAL_TIME),S2,"{0:0.3f}".format(int(x0))])
    #TODO 
    print "To S2 einai : " + str(S2)
    #S2= False
    return S2	

def switch_path_planning(x,y,path):
    #S3 = False
    #return S3
    D = 0
    Jstar = 1 
    for item in obstacles :
    	D = D + math.exp(-math.sqrt((x-(item[0]*CELL_SIZE+CELL_SIZE/2))**2 + (y-(item[1]*CELL_SIZE+CELL_SIZE/2))**2) )
    J = 0
    for i in range(0,len(path)-1):
	J = J + math.sqrt( (path[i][0] - path[i+1][0])**2 + (path[i][1] - path[i+1][1])**2) 
    J = J  -  math.sqrt( (path[0][0] - path[len(path)-1][0])**2 + (path[0][1] - path[len(path)-1][1])**2) 
    if math.fabs(D - J) < Jstar :
	S3 = False
    else : 
	S3 = True 
    filename = "./stats/switch3.csv"
    with open(filename, 'a') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL) 
        # If opened for the first time, insert header row
        if os.path.getsize(filename) == 0:
	    wr.writerow(["timestamp","S3","D","J"])
        wr.writerow(["{0:0.5f}".format(time.time()-INITIAL_TIME),S3,"{0:0.5f}".format(D),"{0:0.5f}".format(J)])
    #TODO
    return S3
    

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
    derror = 7 
    d = 1 
    x0 = 1
    if True :
        try:
	    duration_start = time.time()
            offload_ip = switch_image_processing(x0)
            x, y, i, j, co ,z0, x0,cores, comput_time = self_localize(sl,offload_ip)
	    co = - co
	    filename = "./stats/localization.csv"
            with open(filename, 'a') as myfile:
                wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                # If opened for the first time, insert header row
                if os.path.getsize(filename) == 0:
                    wr.writerow(["timestamp","x","y","i","j","or","time_image_processing","comput_time","z0","x0","real_cores_of_server","uncertainty_d","offload_image_processing"])
                wr.writerow(["{0:0.5f}".format(time.time()-INITIAL_TIME),"{0:0.5f}".format(int(x)),"{0:0.5f}".format(int(y)),i,j,co,"{0:0.5f}".format(time.time()-duration_start),comput_time,z0,x0,cores,d,offload_ip])

        except InsufficientLocalizationInfoError:
            print("I have no information regarding where I am. Quiting...\n")
            quit()
    else :
	x = 35 
    	y = 145
    	i = 14 
    	j = 3
    	co = 90 
    index_astar = 0
    ########################################
    start_time = time.time()
    path = plan_the_path((i,j), tuple(DIAGRAM2_G[0]))
    path_plan_time = time.time()- start_time
    cost_path = 1
    path_astar = path
    S3 = switch_path_planning(x,y,path)
    if S3:
        start_time = time.time()
        post_url =  "http://192.168.88.245:8000/dijkstra"
        payload = [{ "i" : int(i) , "j": int(j), "x" : x, "y":y, "or": co, "dest" : index_astar }]
        #co = -co
        r = requests.post(post_url,json=payload)
        a,b,cost_path = json.loads(r.text)
        temp = [int(a),int(b)]
        path = []
        start_position = [int(y/CELL_SIZE), int (x/CELL_SIZE) ]
        path.append(start_position)
        path.append(temp)
	path_plan_time = time.time()- start_time
        print "Total time for dijkstra: "+ str(time.time()-start_time)
	cp = path.pop(0)
	np = path.pop(0)
    print S3
    # Start following path
    print("Starting to follow path...")
    cp = path_astar.pop(0) # Get starting position
    
    while True : # While current tile is different from target's tile:
        # 1: check if re-calculation of the path is needed
        print ("Thinking that I am in position: (" + str(int(cp[0])) + ", " + str(int(cp[1])) + ")")
        print ("I actually am at: (" + str(int(i)) + ", " + str(int(j)) + "), or (" + 
            str(int(x)) + ", " + str(int(y)) + ") in cm. ")
        #if (((i != int(cp[0])) or (j != int(cp[1])))):
        #    print "So, I need to plan the path again.\n"
	path_time = time.time()
        path_astar = plan_the_path((i,j), tuple(DIAGRAM2_G[index_astar]))
	path_plan_time = time.time()- path_time
        cp = path_astar.pop(0)
        cost_path = 1
            #continue
        #else:
        if S3 == False: 
            print "So, no need for planning the path again.\n"
            try:
		np = path_astar.pop(0)
	    except IndexError:
		np = cp 

	
        print np , cp  
        # 2: check about extra orientation turn
	print co 
        print "Checking for extra turn (a change in path orientation)..."
        #time.sleep(2) 
        # 3: change the orientation; use the Pythagorean Theorem for calculating the new required distance 
        # and angle to move to the next tile
        a = x - (np[1] * CELL_SIZE + CELL_SIZE / 2)
        b = y - (np[0] * CELL_SIZE + CELL_SIZE / 2)
        distance = math.sqrt(a ** 2 + b ** 2) / 100 # distance is needed in m.
        # Find new Point in the line of current orientation.
	x1 = x + 1*cos(co*pi/180) 
	y1 = y + 1*sin(co*pi/180)
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
	print sign
	if sign != 0 :
            no =  w * 180/pi* sign/abs(sign)
	else: 
	    no = 0 
	print "Rotating for: "+str(no)+ " while my current orientation is: " + str(co) 
	###################################################        

	filename = "./stats/path_planning.csv"
        with open(filename, 'a') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            # If opened for the first time, insert header row
            if os.path.getsize(filename) == 0:
                wr.writerow(["timestamp","S3","time_of_path_planning","cost_path_planing","curent_position", "next_position","current_orientation","next_orientation","distance"])
            wr.writerow(["{0:0.5f}".format(time.time()-INITIAL_TIME),S3,"{0:0.5f}".format(path_plan_time),"{0:0.5f}".format(cost_path),cp,np,co,no,"{0:0.5f}".format(distance)])


	##################################################
        #raw_input("Press Enter to continue...\n")
        temp_ori = change_orientation(mc, int(co), int(no),x,y)
        # 4: move a tile forward and at the same time measure the distance and angle reported by the light 
        # sensors (encoders) during the last step. Use this information to get an estimation on the grid
        #raw_input("Press Enter to continue...\n")
        x_enc, y_enc, theta_enc = move_forward(mc, distance,temp_ori,co,x,y)
        #raw_input("Press Enter to continue...\n")
        # 5: acquire estimated position on the grid
	# rotation of axis so we need to compute the real x,y diff
    	co += (temp_ori + theta_enc)
    	theta = math.radians(co)
    	#print theta
    	#print x_enc,y_enc,theta_enc
    	#print  str(int(x)) , str(int(x_enc)*math.cos(theta)) , str(int(y_enc)*math.sin(theta) )
    	x = int(x) + int(x_enc)*math.cos(theta) - int(y_enc)*math.sin(theta) 
    	y = int(y) + int(x_enc)*math.sin(theta) + int(y_enc)*math.cos(theta)
    	i = y // CELL_SIZE
    	j = x // CELL_SIZE
    	print("Current encoder-based position estimation: Row = "+ str(int(i)) + ", Column = " + 
        	str(int(j)) + ", x = " + str(int(x)) + "cm , y = " + str(int(y)) + 
                "cm and orientation = " + str(int(co)) + "deg")
	S1 = False
	b0 = 1 
	b1 = 0.2
	d = d + b0 + b1 * math.fabs(100*distance-x_enc)
	print "Uncertainty is: " + str(d)
	print "Distance error is : " + str(100*distance-x_enc)
	if d > derror : 
	    derror = 7 
	    S1 = True
	    try:
	    	duration_start = time.time()
		if x0 =="none" : x0 = 1 
                offload_ip = switch_image_processing(float(x0))
		x, y, i, j, co ,z0, x0,cores, comput_time = self_localize(sl,offload_ip)
		d = 0 
                co=-co
		filename = "./stats/localization.csv"
                with open(filename, 'a') as myfile:
                    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                    # If opened for the first time, insert header row
                    if os.path.getsize(filename) == 0:
                        wr.writerow(["timestamp","x","y","i","j","or","time_image_processing","comput_time","z0","x0","real_cores_of_server","uncertainty_d","offload_image_processing"])
                    wr.writerow(["{0:0.5f}".format(time.time()-INITIAL_TIME),"{0:0.5f}".format(int(x)),"{0:0.5f}".format(int(y)),i,j,co,"{0:0.5f}".format(time.time()-duration_start),comput_time,z0,x0,cores,d,offload_ip])

            except (InsufficientLocalizationInfoError, ValueError):
            	# At this stage, the (blind) AlphaBot has no environmental information regarding its position.
            	# It now relys solely on the information coming from the encoders. 
            	print("Beacon information not sufficient for localizing. Now using encoders' output...")

        filename = "./stats/switch1.csv"
        with open(filename, 'a') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL) 
            # If opened for the first time, insert header row
            if os.path.getsize(filename) == 0:
                wr.writerow(["timestamp","S1","d"])
            wr.writerow([time.time()-INITIAL_TIME,S1, d])
	
	S3 = switch_path_planning(x,y,path_astar)
	print S3
        if S3:
            start_time = time.time()    
            post_url =  "http://192.168.88.245:8000/dijkstra"
            payload = [{ "i" : int(i) , "j": int(j), "x" : x, "y":y, "or": co, "dest" : index_astar} ]
            r = requests.post(post_url,json=payload)
	    path_plan_time = time.time()- start_time
            a,b,cost_path = json.loads(r.text)
            temp = [int(a),int(b)]
            path = []
            start_position = [int(y/CELL_SIZE), int (x/CELL_SIZE) ]
            path.append(start_position)
            path.append(temp)
            print "Total time for dijkstra: "+ str(time.time()-start_time)
            if start_position == temp:
                if start_position == DIAGRAM2_G[-1]:
		    break
		else :
		    index_astar += 1	
	    ### multiple destinations!! 
            cp = path.pop(0)
            np = path.pop(0)

        else : # local execution of dijkstra
            cp = np
            if cp==DIAGRAM2_G[index_astar]:
		print cp
		index_astar +=1
		if index_astar > len(DIAGRAM2_G):
                    break
        

        print("\n---------------------------------")
        print("End of Step")
        print("---------------------------------\n")
    print("\n\n\n ---------------------- GOAL REACHED (or at least I think so) ----------------------\n\n\n ")
    total_duration = time.time()-INITIAL_TIME
    filename = "./stats/total_time.csv"
    with open(filename, 'a') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL) 
        if os.path.getsize(filename) == 0:
            wr.writerow(["Total_time"])
        wr.writerow(["{0:0.5f}".format(total_duration)])
	
	
if __name__ == "__main__":
    main()
