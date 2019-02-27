#!/usr/bin/env python
import requests
import json
import time
import sys
from Astar import *

GRID_SIZE = 50
CELL_SIZE = 5
goal = (48,49)
def main():
    a = [0,0]
    start = (a[0],a[1])
    time_astar = time.time()
    DIAGRAM2 = GridWithWeights(GRID_SIZE, GRID_SIZE)
    came_from, cost_so_far = a_star_search(DIAGRAM2, start, goal)
    path = reconstruct_path(came_from, start = start, goal = goal)
    time_astar = time.time() - time_astar
    print path
    cost = 0
    path2 = []
    temp = [a[0],a[1]]
    path2.append(temp)
    post_url = "http://192.168.1.114:8000/dijkstra"
    time_improved_dijkstra = time.time()
    step_counter = 0 
    step_timer = 0 
    while True: 
        i = int(a[0])
        j = int(a[1])
        x = j * CELL_SIZE
        y = i * CELL_SIZE
        payload = [ {
                "i": int(i) ,# 6 
                "j": int(j) , # 7 
                "x": int(x) , # 150
                "y": int(y) , # 175
                "or": 170
                }]
        step_counter += 1
        c = time.time()
        r = requests.post(post_url, json=payload )# , data=json)
        step_timer += time.time() - c
        a = json.loads(r.text)
        #print (a[0])
        #print (a[1])
        #print (a[2])
        cost += a[2]
        temp = [int(a[0]),int(a[1])]
        path2.append(temp)
        if int(a[0]) == int(goal[0]) and int(a[1]) == int(goal[1]):
            print "target"
            time_improved_dijkstra =time.time() - time_improved_dijkstra
            print "time improved dijkstra: " + str(time_improved_dijkstra)
            print "time astar: " + str(time_astar)
            print "average time for each step of improved dijkstra : "+str(step_timer/step_counter)
            print "cost improved dijkstra : "+str(cost) 
            print "cost astar : "+str(len(path))
            print "path improved dijkstra" +str(path)
            print "path astar"+ str(path2)
            break

    #a = json.loads(r.text)
    #print a[2] 
if __name__ == "__main__":
        main()
