from collections import deque, namedtuple
import time


CELL_SIZE = 50 # cm MUST GO TO CONFIG.YAML 

# we'll use infinity as a default distance to nodes.
inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class Graph:
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
                    temp.append(str(i)+str(j)+strings[k])
                    temp.append(str(i)+str(j)+strings[anticlockwise])
                    temp.append(1)
                    list.append(temp) 

                    temp = [] 
                    temp.append(str(i)+str(j)+strings[k])
                    temp.append(str(i)+str(j)+strings[clockwise])
                    temp.append(1)
                    list.append(temp)
                    ########################################################### for x 
                    if k == 1 :
                        if j == GRID_SIZE -1:
                            continue
                        else: 
                            right = j + 1 
                        temp = [] 
                        temp.append(str(i)+str(j)+strings[k])
                        temp.append(str(i)+str(right)+strings[k])
                        temp.append(1)
                        list.append(temp)


                    if k == 3 :
                        
                        if j == 0:
                            continue
                        else : 
                            left = j - 1 
                        temp = [] 
                        temp.append(str(i)+str(j)+strings[k])
                        temp.append(str(i)+str(left)+strings[k])
                        temp.append(1)
                        list.append(temp) 

                    ################## for y 
                    if k == 0 :
                        if i == 0:
                            continue
                        else : 
                            up = i - 1 
                        temp = [] 
                        temp.append(str(i)+str(j)+strings[k])
                        temp.append(str(up)+str(j)+strings[k])
                        temp.append(1)
                        list.append(temp) 

                    if k == 2 :
                        if i == GRID_SIZE - 1:
                            continue
                        else: 
                            down = i + 1 
                        temp = [] 
                        temp.append(str(i)+str(j)+strings[k])
                        temp.append(str(down)+str(j)+strings[k])
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

    def get_node_pairs(self, n1, n2, both_ends=True):
        print both_ends
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
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

    def dijkstra(self, source, dest):
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
                
        path, current_vertex = deque(), dest
        total_cost = distances[current_vertex]
        
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
         
        return path , total_cost






    def calculation_of_path(self,i,j,x,y,orientation,dest):
        DESTINATION = dest
        possible_orientation = ["N","E","S","W"]
        if -90<=orientation<0 : 
            actualor1 = "S"
            actualor2 = "W" 
        elif 0<=orientation<90 : 
            actualor1 = "S"
            actualor2 = "E" 
        elif 90<=orientation<180 :
            actualor1 = "E"
            actualor2 = "N" 
        else : #  - 180 < orientation < - 90 
            actualor1 = "W"
            actualor2 = "N" 

        if (x//(CELL_SIZE/2))> (2*j) :
            # briskomai sthn deksia meria tou koutiou 
            if (y//(CELL_SIZE/2))> (2*i) :
            # briskomai sthn katw deksia meria tou koutiou 
                print "katw deksia" 
                or1 = "S"
                or2 = "E" 
            else : 
                print "panw deksia" 
                or1 = "N"
                or2 = "E" 
        else: 
            # briskomai sthn aristeri meria tou koutiou 
            if (y//(CELL_SIZE/2))> (2*i) :
            # briskomai sthn katw aristeri meria tou koutiou 
                print "katw aristera"
                or1 = "S"
                or2 = "W" 
            else: 
                print "panw aristera" 
                or1 = "N"
                or2 = "W"  

        print "where am i at the cell: " +or1,or2 
        print "actual orientation: " + actualor1, actualor2
        neighbour_cells = []
        for x in list(self.neighbours[str(i)+str(j)+":"+or1]):  
            neighbour_cells.append(x)
        for x in list(self.neighbours[str(i)+str(j)+":"+or2]):  
            neighbour_cells.append(x)
        neighbour_cells_list = []
        for neighbour_cells in neighbour_cells: neighbour_cells_list.append((list(neighbour_cells)[0].rsplit(':',1)[0]))
        neighbour_cells_list = list(set(neighbour_cells_list))
        print "my neighbour cells including my cell are :  "+ str(neighbour_cells_list)
        print "my cell is :  "+ str(i),str(j)
                
        paths = []
        cost = inf
        for neighbours in neighbour_cells_list:
            print neighbours
            neighbouri = str(neighbours)[0]
            neighbourj = str(neighbours)[1]
            print "the neighbour's i,j I am examining now! :  "+ neighbouri, neighbourj
            print neighbouri , i , neighbourj , j
            if ((neighbouri == str(i)) and (neighbourj == str(j))) :
                print "examining all possible orientations from my cell"
                for orient in range (4): 
                    a, b = self.dijkstra(neighbouri+neighbourj+":"+possible_orientation[orient],DESTINATION)
                    if b < cost : 
                        cost = b 
                        paths = a
                    print "path :  " + str(a)

            else : 
                a, b = self.dijkstra(neighbouri+neighbourj+":"+actualor1,DESTINATION)
                if b < cost : 
                    cost = b 
                    paths = a
                print "path :  " + str(a)
                a, b = self.dijkstra(neighbouri+neighbourj+":"+actualor2,DESTINATION)
                if b < cost : 
                    cost = b 
                    paths = a
                print "path :  "+ str(a)

        
        return paths 






