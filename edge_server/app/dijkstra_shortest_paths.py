from collections import deque, namedtuple
import time


CELL_SIZE = 50 # cm MUST GO TO CONFIG.YAML 

# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class GraphShortestPaths:
    def __init__(self, GRID_SIZE):
        list = [] 
        temp = []
        strings = [":N",":E",":S",":W"]
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                for k in range (0,len(strings)):
                    ######################### for orientation
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
                    ########################################################### for x 
                    if k == 1 :
                        if j == GRID_SIZE -1:
                            continue
                        else: 
                            right = j + 1 
                        temp = [] 
                        temp.append(str(i)+":"+str(j)+strings[k])
                        temp.append(str(i)+":"+str(right)+strings[k])
                        temp.append(1)
                        list.append(temp)


                    if k == 3 :
                        
                        if j == 0:
                            continue
                        else : 
                            left = j - 1 
                        temp = [] 
                        temp.append(str(i)+":"+str(j)+strings[k])
                        temp.append(str(i)+":"+str(left)+strings[k])
                        temp.append(1)
                        list.append(temp) 

                    ################## for y 
                    if k == 0 :
                        if i == 0:
                            continue
                        else : 
                            up = i - 1 
                        temp = [] 
                        temp.append(str(i)+":"+str(j)+strings[k])
                        temp.append(str(up)+":"+str(j)+strings[k])
                        temp.append(1)
                        list.append(temp) 

                    if k == 2 :
                        if i == GRID_SIZE - 1:
                            continue
                        else: 
                            down = i + 1 
                        temp = [] 
                        temp.append(str(i)+":"+str(j)+strings[k])
                        temp.append(str(down)+":"+str(j)+strings[k])
                        temp.append(1)
                        list.append(temp)
                    #print list
        
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
	for path in paths :
	    temp = []
	    for x in path :
		#print x
		cell_id =  x.rsplit(':',1)[0]
		ori = x.rsplit(':',1)[1]
                if ori == "W":
                    ori = "E"
                elif ori == "E":
                    ori = "W"
                elif ori == "N":
                    ori = "S"
                elif ori == "S":
                    ori = "N"
		temp.append(cell_id+":"+ori)
		if path.index(x) == 0 :
                    return_source.append(cell_id+":"+ori)
	            return_cost.append(total_costs[paths.index(path)])
	    if temp : 
		return_path.append(temp)


	return_source.insert(0,DESTINATION)
	return_path.insert(0,DESTINATION)
        return_cost.insert(0,0)
	return  return_source , return_path, return_cost









