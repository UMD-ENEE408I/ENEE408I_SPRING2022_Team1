from pickle import NONE
import maze
import numpy as np


total_cost=0




finished=False

m=maze.Graph()
start=m.add_vertex((0,0),0)
prev=start
prev_direc=0
# tell robot to move forward

while not finished:
    # if called run following
    # get info for the new vertex
    curr_pos=NONE
    curr_intersect=NONE
    curr_direc=NONE
    weight=0
    # round pos to what it will be on a grid
    if curr_pos not in m.vert_dict:
        m.add_vertex(curr_pos, curr_intersect)
    curr=m.get_vertex(curr_pos)
    m.add_edge(prev.pos, curr_pos,prev_direc, curr_direc, weight )

    total_cost+=weight
    # now to determine what direction to send the robot next

    if curr_intersect==1:
        # dead end
        # turn around and send back
        4
    elif curr_intersect==5:
        finished =True
    else:
        for i in range(0,3):
        # change initial path depending on algorithim

            if not curr.direc_verts[i]:     # goes down paths that arent visited yet
                
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
            

