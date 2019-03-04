from collections import deque, namedtuple
import time
import numpy as np 
import yaml

CONFIG = yaml.load(open("./config.yaml"))
# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')
CELL_SIZE = CONFIG["grid"]["cell_size"]
obstacles = CONFIG["grid"]["obstacles"]

def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class GraphShortestPaths:
    def __init__(self, GRID_SIZE, DESTINATION):
        s = (GRID_SIZE,GRID_SIZE)
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
        #r = DESTINATION.rsplit(":",1)[0]
        #lineofsightmatrix[int(r.rsplit(":",1)[0])][int(r.rsplit(":",1)[1])] = 5
        print lineofsightmatrix

        list = [] 
        temp = []
        strings = [":N",":E",":S",":W"]
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                for k in range (0,len(strings)):
                    ######################### for orientation
                    if lineofsightmatrix[i][j] == 0 :
                        if k == 0:
                            anticlockwise = len(strings) - 1
                        else : 
                            anticlockwise = k - 1 
                        if k == len(strings)-1:
                            clockwise = 0
                        else: 
                            clockwise = k + 1
                        temp = []
                        temp.append(str(i)+":"+str(j)+strings[k])
                        temp.append(str(i)+":"+str(j)+strings[anticlockwise])
                        temp.append(1)
                        list.append(temp) 

                        temp = [] 
                        temp.append(str(i)+":"+str(j)+strings[k])
                        temp.append(str(i)+":"+str(j)+strings[clockwise])
                        temp.append(1)
                        list.append(temp)
                    ########################################################### for E the edges are reversed  
                    if k == 1 :
                        if j == 0:
                            continue
                        else: 
                            right = j - 1 
                        if lineofsightmatrix[i][j] == 0 and lineofsightmatrix[i][right]==0 :
                            temp = [] 
                            temp.append(str(i)+":"+str(j)+strings[k])
                            temp.append(str(i)+":"+str(right)+strings[k])
                            temp.append(1)
                            list.append(temp)
                    ########################################################### for W the edges are reversed  

                    if k == 3 :
                        
                        if j == GRID_SIZE - 1 :
                            continue
                        else : 
                            left = j + 1 
                        if lineofsightmatrix[i][j] == 0 and lineofsightmatrix[i][left]==0 :
                            temp = [] 
                            temp.append(str(i)+":"+str(j)+strings[k])
                            temp.append(str(i)+":"+str(left)+strings[k])
                            temp.append(1)
                            list.append(temp) 

                    ################## for N
                    if k == 0 :
                        if i == GRID_SIZE -1 :
                            continue
                        else : 
                            up = i + 1 
                        if lineofsightmatrix[i][j] == 0 and lineofsightmatrix[up][j]==0 :
                            temp = [] 
                            temp.append(str(i)+":"+str(j)+strings[k])
                            temp.append(str(up)+":"+str(j)+strings[k])
                            temp.append(1)
                            list.append(temp) 
                    ################## for S 
                    if k == 2 :
                        if i == 0:
                            continue
                        else: 
                            down = i - 1 
                        if lineofsightmatrix[i][j] == 0 and lineofsightmatrix[down][j]==0 :
                            temp = [] 
                            temp.append(str(i)+":"+str(j)+strings[k])
                            temp.append(str(down)+":"+str(j)+strings[k])
                            temp.append(1)
                            list.append(temp)
        #print list
        print "Arithmos akmwn: " + str(len(list)) 
        edges = list 
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]
    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=False):
        print both_ends
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=False):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=False):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source):
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
   
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                #print neighbour , current_vertex
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex
                   
        paths = []
        total_costs= []
        source = []
        #print len(self.vertices)
        for dest in self.vertices :
            source.append(dest)
            path = deque()
            current_vertex = dest
            total_cost = distances[current_vertex]
            while previous_vertices[current_vertex] is not None:
                path.appendleft(current_vertex)
                current_vertex = previous_vertices[current_vertex]
            if path:
                path.appendleft(current_vertex)
            paths.append(list(reversed(path)))
            total_costs.append(total_cost)
            #print dest, path,total_cost
        return source , paths , total_costs
   

    def shortest_paths(self,DESTINATION):
        source , paths , total_costs=(self.dijkstra(DESTINATION))
	return_path = []
	return_source=[]
	return_cost = []
	return  source , paths, total_costs









