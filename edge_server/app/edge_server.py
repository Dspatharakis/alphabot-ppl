#!flask/bin/python
import os 
import json
import ast
import math
import time
import sqlite3
import yaml
from sympy import Poly, Symbol
from sympy.solvers.inequalities import reduce_rational_inequalities
from flask import Flask
from flask import request, url_for
from flask import jsonify, abort
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from dna import Dna
from werkzeug.utils import secure_filename
from alphabot_exceptions import *
from dijkstra_shortest_paths import *
CONFIG = yaml.load(open("./config.yaml"))

d = Dna()
GRID_SIZE = CONFIG["grid"]["grid_size"]
CELL_SIZE = CONFIG["grid"]["cell_size"]
DISTANCE_TO_NEIGHBOURS = CONFIG["grid"]["distance_to_neighbours"]
DESTINATION = CONFIG["grid"]["destination"]
obstacles = CONFIG["grid"]["obstacles"]

app = Flask(__name__)
conn = sqlite3.connect(':memory:', check_same_thread=False)
c = conn.cursor()

def feed_db() :
    start_time = time.time()
    graph = GraphShortestPaths(GRID_SIZE,DESTINATION)
    graph_time = time.time()
    print "Time for  Graph: "+str(graph_time - start_time)
    source,path,cost = graph.shortest_paths(DESTINATION)
    dijkstra_time = time.time()
    print "Time for Dijkstra: "+str(dijkstra_time - graph_time)
    c.execute('''CREATE TABLE Path (source text, path text, cost text) ''')
    for i in range (len(source)):
        if source [i] == DESTINATION : 
            temp_path = source[i]
        elif not path[i]:
            a = 0 
        else:
            temp_path = path [i][1]
        values = [str(source[i]),str(temp_path),str(cost[i]),]
        c.execute("INSERT INTO Path VALUES (?,?,?)",values)
        conn.commit()
        db_time = time.time()
    print "Time for db: "+str(db_time - dijkstra_time)
    

@app.route('/', methods = ['GET', 'POST'])
def post_image():
    if request.method == 'GET':
        return "GET \n"
    if request.method == 'POST':
        start_time = time.time()
        temp_json = json.load(request.files['time'])
        communication_time = temp_json['time']
        transmission_time = start_time-int(communication_time)
        #print "Transmission time: "+ str(transmission_time)
        file = request.files['file']
        filename = secure_filename(file.filename)
        file.save(filename)
        dirr = os.getcwd()
        osname = os.path.join(dirr, '')
        dest_img = osname + filename
        try: 
            results = d.find_distance_and_angle(dest_img)  ### pairnei path
            os.remove(dest_img)
            end_time = time.time()-start_time
            print "Computational time for Image Recognition :"+str(end_time)
            return jsonify(results)
        except BeaconNotFoundError:
            os.remove(dest_img)
            end_time = time.time()-start_time
            print "Computational time for Image Recognition :"+str(end_time)
            return abort (404)


@app.route('/dijkstra', methods = ['GET', 'POST'])
def path_planning():
    start_time = time.time()
    a = (request.get_json())[0] 
    print a
    for key, value in a.items() :
        if key == "i":
            iref = value
        elif key == "j" :
            jref = value
        elif key == "x" :
            x = value
        elif key == "y" :
            y = value 
        else :
            orientation = value
    path_to_node, cost_to_node  = reconstruct(x,y,iref,jref,GRID_SIZE)
    return_path = []
    cost = float("inf")
    target_node = 0 
    for candidate_path in path_to_node :
        if not candidate_path:
            continue ;
        search = candidate_path[0]
        c.execute('SELECT * FROM Path WHERE source LIKE ?' , (str(search)+'%',))
        temp1 = c.fetchall()
        templist = []
        for x in temp1: 
            templist.append(map(str, x))
        for temp in templist :
            temp_source = temp[0]
            temp_path = temp[1]
            temp_cost = int(temp[2])
            #print temp_source
            if temp_source == DESTINATION:
                print "Target eliminitated"
                target_node = temp.source
                almost = target_node.rsplit(":",1)[0]
                a = almost.rsplit(":",1)[0]
                b = almost.rsplit(":",1)[1]
                return_list = []
                return_listeappend(a)
                return_list.append(b)
                ##
                cost = temp_cost + cost_to_node[path_to_node.index(candidate_path)]
                cost_for_move = cost_to_node[path_to_node.index(candidate_path)]
                #return_list.append(cost_for_move) 
                #print cost_for_move
                ##
                print "Computational time for dijkstra: " + str(time.time()-start_time)
                return jsonify(return_list)
            #print str(temp.cost + cost_to_node[path_to_node.index(candidate_path)]) 
            if temp_cost + cost_to_node[path_to_node.index(candidate_path)] < cost : 
                return_path = temp_path
                cost = temp_cost + cost_to_node[path_to_node.index(candidate_path)]
                cost_for_move = cost_to_node[path_to_node.index(candidate_path)]
                target_node = temp_source
                #print cost_for_move
    #print return_path
    #print target_node
    if target_node.rsplit(":",1)[0] == str(iref)+str(":")+str(jref):
        print "I am already at this node looking for next step from db"
        while True:  
            target_node = return_path
            #print target_node
            if target_node.rsplit(":",1)[0] != (str(iref)+str(":")+str(jref)):
                cost_for_move = 1
                break 
            search = return_path
            c.execute('SELECT * FROM Path WHERE source LIKE ?' , (str(return_path),))
            temp1 = c.fetchall()
            temp = map(str,temp1)
            target_node = temp[1]
            #temp = Path.query.filter(Path.source.startswith(str(return_path))).all()
            #target_node = temp_path
            
    print "Next step is: "+ str(target_node)
    print "The cost to reach target is: "+ str(cost)
    almost = target_node.rsplit(":",1)[0]
    a = almost.rsplit(":",1)[0]
    b = almost.rsplit(":",1)[1]
    return_list = []
    return_list.append(a)
    return_list.append(b)
    ##
    #return_list.append(cost_for_move) 
    #print cost_for_move
    print "Computational time for dijkstra: " + str(time.time()-start_time)
    return jsonify(return_list)


def reconstruct(x,y,i,j,GRID_SIZE):
    neighbours = [] 
    costs = []
    imax = (i * CELL_SIZE + CELL_SIZE/2 + DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    imin = (i * CELL_SIZE + CELL_SIZE/2 - DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    jmax = (j * CELL_SIZE + CELL_SIZE/2 + DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    jmin = (j * CELL_SIZE + CELL_SIZE/2 - DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    iref = i
    jref = j 
    if imin < 0 : imin = 0
    if jmin < 0 : jmin = 0
    if imax > GRID_SIZE - 1: imax = GRID_SIZE - 1
    if jmax > GRID_SIZE - 1: jmax = GRID_SIZE - 1
    for i in range (imin,imax+1):
        for j in range (jmin,jmax+1):
            #print i,j
	    candx= j*CELL_SIZE + CELL_SIZE /2 
	    candy= i*CELL_SIZE + CELL_SIZE/2
            #print (x,candx,y,candy)
            obstacle = line_of_sight(x,candx,y,candy)  
            if obstacle == False:
                distance = math.sqrt( ((x-(j*CELL_SIZE+CELL_SIZE/2))**2)+((y-(i*CELL_SIZE+CELL_SIZE/2))**2))
                #print i,j
                #print ("I am connecting with: "+str(i)+str(j))
                #print ("I am connecting from i= "+str(iref)+" j: "+str(jref)+" to i: "+str(i)+" and j: "+str(j))
                temp = []
                temp.append(str(i)+":"+str(j))
                costs.append(distance/CELL_SIZE)
                neighbours.append(temp)
            #else:
                #print ("I tried to connect i= "+str(iref)+" j: "+str(jref)+" with i: "+str(i)+" and j: "+str(j)+" but there is an obstacle")
    return neighbours, costs 

def line_of_sight(xa,xb,ya,yb) :
    lineofsight = False
    for rect in obstacles:
    # box has down left and up right corner of the rectangle
        box = [rect[0][0],rect[0][1],rect[1][0],rect[1][1]]		
        xmin = box [1]
        ymin=box[0]
        xmax= box[3]
        ymax = box[2]
        l = Symbol('l', real=True)
        #print box , xa ,xb ,ya ,yb 
        if (xa-xb==0) and (ya-yb==0):
            if xmin<xa<ya and ymin<ya<ymax :
                print "obstacle"
        elif (xa-xb == 0) :
        # check if obstacle is in the vertical line between the two points
            if xmin<=xa<=xmax :
                a = reduce_rational_inequalities([[ l <= 1, l >= 0 , l<= (ymax-yb)/(ya-yb), l>= (ymin-yb)/(ya-yb)]], l)
                if a != False :
                    return True
        elif (ya-yb == 0) :
        # check if obstacle is in the horizontal line between the two points
            if ymin<=ya<=ymax :
                a = reduce_rational_inequalities([[ l <= 1, l >= 0 , l<= (xmax-xb)/(xa-xb) , l >= (xmin-xb)/(xa-xb)]], l)
                if a != False :
                    return True 
        else :
            a = reduce_rational_inequalities([[ l <= 1, l >= 0 , l<= (xmax-xb)/(xa-xb) , l >= (xmin-xb)/(xa-xb), l<= (ymax-yb)/(ya-yb), l>= (ymin-yb)/(ya-yb)]], l)
            if a != False :
                return True
    return lineofsight

if __name__ == '__main__':
    feed_db()
    #app.run(host='192.168.1.114', port=8000)
    app.run(host='0.0.0.0', port=8000)
