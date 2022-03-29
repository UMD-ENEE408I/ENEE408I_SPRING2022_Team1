import maze
import numpy as np
# def DFS(m, start, end):
#     for neigh in start.get_connections():
#         if neigh=end:




g = maze.Graph()

g.add_vertex((0,0))


g.add_edge((0,0), (5,0),1,1, 5)  
g.add_edge((5,0), (5,3),0,0, 3)
g.add_edge((5,0), (9,-5),1,1, 9)
g.add_edge((5,0), (5,-5),2,2, 5)
g.add_edge((5,-5), (3,-5),3,3, 2)
g.add_edge((5,-5), (5,-9),2,2, 4)
g.add_edge((5,-9), (12,0),1,0,5)
g.add_edge((5,3), (9,3),1,1,4)
g.add_edge((5,3), (12,0),0,2,12)
g.add_edge((9,3), (9,0),2,2,3)
g.add_edge((9,3), (10,3),1,1,1)
g.add_edge((9,0), (12,0),1,1,3)
g.add_edge((9,0), (9,-5),2,2,5)
g.add_edge((9,-5), (9,-6),2,2,1)



time=0
# get intersect



finished=False

new_maze=maze.Graph()
curr=g.get_vertex((0,0))
new_maze.add_vertex(curr.get_id()) 

while not finished:

    for i in range(0,3):
    # change initial path depending on algorithim

        if curr.direc_verts[i]:
            time+=curr.direc_weights[i]
            
            # cost and end direction need to bbe determined from mouse
            # get current orientation (end_direc)
            end_direc=np.where(curr.direc_verts==prev ) 
            # fix for multiple paths for same 2 nodes
            # wont be a problem outside of the simulation

            #end_direc=np.where(curr.direc_verts==prev and curr.direc_weights==cost) 


            new_maze.add_edge(prev.id,curr.id,direc,end_direc,cost)

            prev=curr

    
    
# def dfs(start, new_maze,time):
#     for i in range(0,3):
#     # change initial path depending on algorithim

#         if curr.direc_verts[i]:
#             time+=curr.direc_weights[i]
#             dfs_aux(curr.direc_verts[i],curr,i, curr.direc_weights[i],new_maze, time )

# def dfs_aux(curr,prev,int_direc,cost, new_maze,time):
#     # cost and end direction need to bbe determined from mouse
    

#     # get current orientation (end_direc)
#     end_direc=np.where(curr.direc_verts==prev ) 
#     # fix for multiple paths for same 2 nodes
#     # wont be a problem outside of the simulation

#     #end_direc=np.where(curr.direc_verts==prev and curr.direc_weights==cost) 


#     new_maze.add_edge(prev.id,curr.id,int_direc,end_direc,cost)

#     for i in range(0,3):
#     # change initial path depending on algorithim

#         if curr.direc_verts[i]:
#             time+=curr.direc_weights[i]
#             dfs_aux(curr.direc_verts[i],curr,i, curr.direc_weights[i],new_maze, time )
            

