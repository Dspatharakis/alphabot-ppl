#!flask/bin/python
import os 
import json
import ast
import math
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
GRID_SIZE = 5
DESTINATION = ("2:3:W")  # 2:3:W

app = Flask(__name__)
app.config.from_object(Config)
db.init_app(app)


def feed_db() : 
    graph = GraphShortestPaths(GRID_SIZE)
    source,path,cost = graph.shortest_paths(DESTINATION)
    #print dest, a , b
    for i in range (GRID_SIZE*GRID_SIZE*4):
        u = Path(source = str(source[i]), path = str(path[i]), cost=str(cost[i]))
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
    path_to_node, cost_to_node  = reconstruct(x,y,GRID_SIZE)
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
                #print target_node
                almost = target_node.rsplit(":",1)[0]
                a = almost.rsplit(":",1)[0]
                b = almost.rsplit(":",1)[1]
                return_list = []
                return_list.append(a)
                return_list.append(b)
                return jsonify(return_list)
            if temp.cost + cost_to_node[path_to_node.index(candidate_path)] < cost : 
                return_path = []
                return_path.append(ast.literal_eval(temp.path))
                cost = temp.cost + cost_to_node[path_to_node.index(candidate_path)]
                target_node = temp.source
                #print cost 
    #print return_path
    #print target_node
    if target_node.rsplit(":",1)[0] == str(iref)+str(":")+str(jref):
        print "eimai hdh se auto ton kombo koitaw to epomeno bhma"
        for i in range (len(return_path[0])) :
            target_node = return_path[0][i]
            if target_node.rsplit(":",1)[0] != (str(iref)+str(":")+str(jref)):
                target_node = return_path[0][i]
                break
    print target_node
    almost = target_node.rsplit(":",1)[0]
    a = almost.rsplit(":",1)[0]
    b = almost.rsplit(":",1)[1]
    return_list = []
    return_list.append(a)
    return_list.append(b)
    return jsonify(return_list)


def reconstruct(x,y,GRID_SIZE):
    neighbours = [] 
    costs = []
    for i in range (GRID_SIZE):
        for j in range (GRID_SIZE):
	    distance = math.sqrt( ((x-((j+1)*CELL_SIZE-CELL_SIZE/2))**2)+((y-((i+1)*CELL_SIZE-CELL_SIZE/2))**2) )
            #print i,j
            #print distance
	    if distance < 280 :
		#print ("I am connecting with: "+str(i)+str(j))
		temp = []
		temp.append(str(i)+":"+str(j))
		costs.append(1)
		neighbours.append(temp)
    return neighbours, costs 
         
if __name__ == '__main__':
    db.create_all()
    feed_db()
    app.run(host='192.168.1.114', port=8000)
