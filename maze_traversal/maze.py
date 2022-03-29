# 
# 
# copied from https://www.bogotobogo.com/python/python_graph_data_structures.php

from msilib import type_short
import numpy as np


class Vertex:
    def __init__(self, xpos, ypos, intersect):
        self.xpos = xpos
        self.ypos = ypos
        self.intersect = intersect          # will be an interger value with the directions coming out of the vertex: 0-start, 1-dead end, 3-3way int, 4-4way int, 5-finish line
        self.direc_verts=np.empty(4,dtype=Vertex)
        self.direc_weights=np.empty(4,dtype=int)

    #fix this
    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

    def add_neighbor(self,direction, neighbor, weight):
        self.direc_verts[direction]=neighbor
        self.direc_weights[direction]=weight

    def get_connections(self):
        return self.direc_verts

    def get_id(self):
        return self.id

    def get_direc_weight(self, neighbor):
        # fix for if there are two paths from self to neighbor
        (direc_index,)=(np.where(self.direc_verts==neighbor))
        return (direc_index[0],self.direc_weights[direc_index[0]])

class Graph:
    def __init__(self):
        # vertex dictionary
        # vertexs are named with a tuple for xpos and ypos
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, pos, intersect):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex( pos[0], pos[1], intersect)
        self.vert_dict[pos] = new_vertex
        return new_vertex

    def get_vertex(self, pos):
        if pos in self.vert_dict:
            return self.vert_dict[pos]
        else:
            return None

    def add_edge(self, frm, to,direc,end_direc ,cost ):
        # following needs to be done outside of add edge
        # if frm not in self.vert_dict:
        #     self.add_vertex(frm)
        # if to not in self.vert_dict:
        #     self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(direc,self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor((end_direc+2)%4,self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

if __name__ == '__main__':

    g = Graph()

    g.add_vertex((11,4), 2)
    g.add_vertex((11,11))
    g.add_vertex((2,4))
    g.add_vertex((2,11))
    g.add_vertex((2,17))
    g.add_vertex((25,4))

    g.add_edge((11,4), (11,11),0,0, 7)  
    g.add_edge((11,4), (2,4),3,3, 9)
    g.add_edge((11,4), (25,4),1,1, 14)
    g.add_edge((11,11), (2,11),3,3, 10)
    g.add_edge((2,11), (2,4),2,2, 15)
    g.add_edge((2,11), (2,17),0,0, 6)
    g.add_edge((25,4), (2,37),0,0,22)
    g.add_edge((2,17), (2,37),1,1,20)

    for v in g:
        for w in v.get_connections():
            if w!=None:
                vid = v.get_id()
                wid = w.get_id()
                print ( vid, wid, v.get_direc_weight(w))

    # for v in g:
    #     print 'g.vert_dict[%s]=%s' %(v.get_id(), g.vert_dict[v.get_id()])