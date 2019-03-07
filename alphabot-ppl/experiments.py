import os
import subprocess
from self_locator import *
import yaml
import math
import json
import time
import signal
import sys
import csv
import requests
import numpy as numpy  
CONFIG = yaml.load(open("../config.yaml"))

# (Column, Row)
GRID_COLUMNS = CONFIG["grid"]["columns"]
GRID_ROWS = CONFIG["grid"]["columns"]

# BEACON_COLORS = [0, 1, 2, 3, 4]
BEACON_COLUMNS = CONFIG["grid"]["beacons_columns"]
BEACON_ROWS = CONFIG["grid"]["beacons_rows"]
CELL_SIZE = CONFIG["grid"]["cell_size"]

ORIENT_REF_ROW = CONFIG["grid"]["orientation_reference_row"] # Red Beacon
offload_dijkstra = CONFIG["offload_path_planning"]
S1 = str(CONFIG["camera"]["vertical_servo_pin"])


# Return estimated AlphaBot's position in grid (column, row, orientation)
def self_localize(self_locator):
    b_distance, b_angle, b_color = self_locator.dna_from_beacons()
    x, y, column, row = detect_position_in_grid(b_distance, b_color)
    # angle from the first beacon is enough
    orientation = detect_orientation(x, y, b_distance[0], b_angle[0], b_color[0]) 
    # print x, y, column, row, orientation
    return x, y, column, row, orientation, b_color

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

def signal_handler(sig, frame):
    print(' was pressed! Fixing camera position and terminating...\n')
    with open(os.devnull, 'wb') as devnull:
        subprocess.check_call(['sudo', 'python', 'turn_head.py', '-s', S1, '-w', '1600'], 
            stdout=devnull, stderr=subprocess.STDOUT)
    sys.exit(0)

def main():
    os.system('clear')
    signal.signal(signal.SIGINT, signal_handler)
    sl = SelfLocator(300)
    c = 0
    
    ## dummy values
    xreal = 150
    yreal = 150 
    orreal = 0
    while True : # While current tile is different from target's tile:
        c += 1
        if c==5:
            break;
        #raw_input("Press Enter to continue...\n")
        
        try:
            x, y, i, j, co, beacons = self_localize(sl)
            co=-co
            filename = "./statsclient"
            with open(filename, 'a') as myfile:
                wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                # If opened for the first time, insert header row
                if os.path.getsize(filename) == 0: 
                    wr.writerow(["xreal","yreal","orreal","xmeas","ymeas","ormeas","beacons"])
                wr.writerow([xreal,yreal,orreal,x,y,co,beacons])

        except (InsufficientLocalizationInfoError, ValueError):
            print("I have no information regarding where I am. Quiting...\n")
            continue
        


if __name__ == "__main__":
    main()
