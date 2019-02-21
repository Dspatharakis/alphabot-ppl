#!flask/bin/python
import os 
import json
import ast
import math
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
from config import Config
from models import Path, db

d = Dna()
#####
GRID_SIZE = 10 #5 #10
CELL_SIZE = 25 #50#25
DISTANCE_TO_NEIGHBOURS = CELL_SIZE * 2 
DESTINATION = ("6:7:W")  # 2:3:W, 6:7:W
obstacles=[[[0,0],[50,0],[75,50],[75,0]],[[55,78],[55,67],[80,67],[80,78]],[[90,67],[90,78],[120,78],[120,67]]]

app = Flask(__name__)
app.config.from_object(Config)
db.init_app(app)


def feed_db() : 
    graph = GraphShortestPaths(GRID_SIZE)
    source,path,cost = graph.shortest_paths(DESTINATION)
    #print dest, a , b
    for i in range (len(source)):
        #print source[i]
        #print path[i]
        if source [i] == DESTINATION : 
            temp_path = source[i]
        elif not path[i]:
            a = 0 
            #print "Path does not exist"
        else:
            temp_path = path [i][1]
        u = Path(source = str(source[i]), path = str(temp_path), cost=str(cost[i]))
        db.session.add(u)
        db.session.commit()


@app.route('/', methods = ['GET', 'POST'])
def post_image():
    if request.method == 'GET':
        return "GET \n"
    if request.method == 'POST':
        
        file = request.files['file']
        filename = secure_filename(file.filename)
        file.save(filename)
        dirr = os.getcwd()
        osname = os.path.join(dirr, '')
        dest_img = osname + filename
        try: 
            results = d.find_distance_and_angle(dest_img)  ### pairnei path
            os.remove(dest_img)
            return jsonify(results)
        except BeaconNotFoundError:
            os.remove(dest_img)
            return abort (404)


@app.route('/dijkstra', methods = ['GET', 'POST'])
def path_planning():
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
    #print path_to_node , cost_to_node 
    return_path = []
    cost = float("inf")
    target_node = 0 
    for candidate_path in path_to_node :
        if not candidate_path:
            continue ;
        temp =Path.query.filter(Path.source.startswith(str(candidate_path[0]))).all()
        #print candidate_path
        for temp in temp :
            #print temp.source
            if temp.source == DESTINATION:
                print "Target eliminitated"
                target_node = temp.source
                almost = target_node.rsplit(":",1)[0]
                a = almost.rsplit(":",1)[0]
                b = almost.rsplit(":",1)[1]
                return_list = []
                return_list.append(a)
                return_list.append(b)
                return jsonify(return_list)
            #print str(temp.cost + cost_to_node[path_to_node.index(candidate_path)]) 
            if temp.cost + cost_to_node[path_to_node.index(candidate_path)] < cost : 
                #return_path = []
                #return_path.append(ast.literal_eval(temp.path))
                return_path = temp.path
                cost = temp.cost + cost_to_node[path_to_node.index(candidate_path)]
                target_node = temp.source
                #print cost 
    #print return_path
    #print target_node
    if target_node.rsplit(":",1)[0] == str(iref)+str(":")+str(jref):
        print "eimai hdh se auto ton kombo koitaw to epomeno bhma"
        #for i in range (len(return_path[0])) :
        while True:  
            target_node = return_path
            print target_node
            if target_node.rsplit(":",1)[0] != (str(iref)+str(":")+str(jref)):
                break 
            temp = Path.query.filter(Path.source.startswith(str(return_path))).all()
            target_node = temp.path
            
    print "Next step is: "+ str(target_node)
    print "The cost to reach target is: "+ str(cost)
    almost = target_node.rsplit(":",1)[0]
    a = almost.rsplit(":",1)[0]
    b = almost.rsplit(":",1)[1]
    return_list = []
    return_list.append(a)
    return_list.append(b)
    return jsonify(return_list)


def reconstruct(x,y,i,j,GRID_SIZE):
    neighbours = [] 
    costs = []
    print i,j, DISTANCE_TO_NEIGHBOURS
    imax = (i * CELL_SIZE + CELL_SIZE/2 + DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    imin = (i * CELL_SIZE + CELL_SIZE/2 - DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    jmax = (j * CELL_SIZE + CELL_SIZE/2 + DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    jmin = (j * CELL_SIZE + CELL_SIZE/2 - DISTANCE_TO_NEIGHBOURS)/CELL_SIZE
    ix = i
    jy = j 
    if imin < 0 : imin = 0
    if jmin < 0 : jmin = 0
    if imax > GRID_SIZE - 1: imax = GRID_SIZE - 1
    if jmax > GRID_SIZE - 1: jmax = GRID_SIZE - 1
    print imax , imin, jmax , jmin
    for i in range (imin,imax+1):
        for j in range (jmin,jmax+1):
            print i,j
	    ## if line of sight
	    candx= j*CELL_SIZE + CELL_SIZE /2 
	    candy= i*CELL_SIZE + CELL_SIZE/2
            print (x,candx,y,candy)
            obstacle = line_of_sight(x,candx,y,candy)  
            print obstacle
            if obstacle == False:
                distance = math.sqrt( ((x-((j+1)*CELL_SIZE-CELL_SIZE/2))**2)+((y-((i+1)*CELL_SIZE-CELL_SIZE/2))**2) )
                #print i,j
                #print distance
                #print ("I am connecting with: "+str(i)+str(j))
                print ("I am connecting from x= "+str(ix)+" y: "+str(jy)+" to x: "+str(i)+" and y: "+str(j))
                temp = []
                temp.append(str(i)+":"+str(j))
                costs.append(distance/CELL_SIZE)
                neighbours.append(temp)
            else:
                print ("I tried to connect x= "+str(ix)+" y: "+str(jy)+" with x: "+str(i)+" and y: "+str(j)+" but there is an obstacle")
    return neighbours, costs 

def line_of_sight(xa,xb,ya,yb) :
    lineofsight = False
    for rect in obstacles:
    # box has down left and up right corner of the rectangle
        box = [rect[1][0],rect[1][1],rect[3][0],rect[3][1]]		
        xmin = box [0]
        ymin=box[1]
        xmax= box[2]
        ymax = box[3]
        l = Symbol('l', real=True)
        print box 
        if (xa-xb == 0) :
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
    print lineofsight
    return lineofsight

if __name__ == '__main__':
    db.create_all()
    feed_db()
    app.run(host='192.168.1.114', port=8000)
