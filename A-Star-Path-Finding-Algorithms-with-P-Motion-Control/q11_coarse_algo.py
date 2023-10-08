"""
Author: Ruotong Jia
Tasks: Offline A*
    Key steps: 
        1. Add start node to open list
        2. Enters a while loop. exits when Olist is empty
        3. In open list, there is only one node.
    --->3.5 update neighbors of the chessboard, according to the true chessboard
        4. Put the node in closed list, meaning this node has been explored
        5. Find neighbors, as long as the child index is within the matrix.
        6. For every child, see if the child is in the closed loop. If yes, then skip the rest of the loop and start on the next child.  
        7. Evaluate the f = g+h for each child. h = 1 for an empty cell, h = 1000 for occupied cells
        8. Select the child with the lowest f and put it in openlist. 
    
    
    Instruction: 
        1. color coding: 
          s -> red (start), p -> green (path), g -> blue (goal), o -> black (obstacle)

"""

import numpy as np
from utility import display_grid,to_matrix_coord,load_landmark_data
from utility import display_grid,to_matrix_coord,load_landmark_data, to_map_coord
from control import controller, motion_model

o = 3   #obstacle value in chessboard

class Node():
    def __eq__(self, an):
        return self.position == an.position

    def __init__(self, parent=None, position=None):
        self.position = position
        zero = 0
        self.parent = parent
        self.f = zero 
        self.h = zero 
        self.g = zero

def update_chessboard(chessboard,true_chessboard,expanded_node):
    """ updating a node's neighbors according to the true chessboard """
    num_row = len(true_chessboard)
    num_cln = len(true_chessboard[0])
    
    for new_position in [ (0, 1), (0, -1), (1, 0), (-1, 0), (-1, 1), (-1, -1),(1, 1), (1, -1) ]: 
        node_position = (expanded_node.position[0] + new_position[0], expanded_node.position[1] + new_position[1])
        if node_position[0] < 0 or (num_row-1) < node_position[0]  or (num_cln-1) < node_position[1] or 0 > node_position[1] :
           continue
        else:
            chessboard[node_position[0]][node_position[1]] = true_chessboard[node_position[0]][node_position[1]]
    
    return chessboard

def get_min_child(neighbors):
    min_child = neighbors[0]
    for child in neighbors:
        if child.f < min_child.f:
            min_child = child
    return min_child

def astar_online_control_coarse_map(true_chessboard, start, end, add_noise = True):

    # for controller, set up the real map params
    cell_size = 1
    heading_init = -1*np.pi/2
    x_range = [-2,5]
    y_range = [-6,6]
    start_map = to_map_coord(start,x_range,y_range, cell_size)
    turtle_path_start = np.append( start_map, heading_init) #[x,y,theta] in map coordinates

    turtle_path_x = np.array([turtle_path_start[0]])  #1 x m np array
    turtle_path_y = np.array([turtle_path_start[1]])  #1 x m np array
    turtle_path_theta = np.array([turtle_path_start[2]])  #1 x m np array


    s_node = Node(None, start)
    s_node.g = s_node.h = s_node.f = 0 
    e_node = Node(None, end)
    e_node.g = e_node.h = e_node.f = 0 

    Olist = []
    Clist = []
    Olist.append(s_node)

    num_row = len(true_chessboard)
    num_cln = len(true_chessboard[0])
    
    chessboard = np.ones((num_row,num_cln))*np.nan
    while len(Olist) > 0:

        expanded_node = Olist[0]
        c_index = 0

        
        chessboard = update_chessboard(chessboard,true_chessboard,expanded_node)
        Olist.pop(c_index)
        Clist.append(expanded_node)

        # Now let's return the closed list
        if expanded_node == e_node:
            turtle_path = ( turtle_path_x, turtle_path_y, turtle_path_theta )
            path = []
            a = expanded_node
            while a is not None:
                path.append(a.position)
                a = a.parent
            return path[::-1], turtle_path 


        neighbors = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            node_position = (expanded_node.position[0] + new_position[0], expanded_node.position[1] + new_position[1])

            if node_position[0] > (num_row-1) or node_position[0] < 0 or node_position[1] > (num_cln-1) or node_position[1] < 0:
                continue

            new_node = Node(expanded_node, node_position)

            if not new_node in Clist:
                # Append to valid child list
                neighbors.append(new_node)

        for child in neighbors:
            #Please notice that here unoccupied cell is np.nan
            if chessboard[child.position[0]][child.position[1]] == o:
                child.g = expanded_node.g+1000
            else:
                child.g = 1 + expanded_node.g

            hval =   ((child.position[1] - e_node.position[1]) ** 2) + ((child.position[0] - e_node.position[0]) ** 2) 
  
            child.h = hval
            child.f = hval + child.g
        # get the min child
        min_child = get_min_child(neighbors)
        

        # Issue control here. Also, update your current node.
        control_end = to_map_coord( min_child.position, x_range, y_range, cell_size )
        control_end = np.append( control_end, 0)
        x_vec,y_vec,theta_vec = control_interface( turtle_path_start, control_end, cell_size, add_noise)
        #updating turtle_path_start for next iteration
        turtle_path_start = np.array([x_vec[-1],y_vec[-1],theta_vec[-1]])
        turtle_path_x = np.append(turtle_path_x,x_vec)#1*m np array,
        turtle_path_y = np.append(turtle_path_y,y_vec)#1*m np array
        turtle_path_theta = np.append(turtle_path_theta,theta_vec)#1*m np array

        #check if controls end up in a wrong cell. If so, change min_child's position, in tuple
        # So we only update the location of the child, without changing the closed list.
        last_pos = turtle_path_start[:2]
        last_pos_mat = to_matrix_coord( last_pos, x_range, y_range, cell_size)
        if last_pos_mat != min_child.position:
            min_child.position = last_pos_mat


        Olist.append(min_child)

    turtle_path = ( turtle_path_x, turtle_path_y, turtle_path_theta )
    return [], turtle_path



def control_interface(start,end,cell_size,add_noise):
    """ 
        inputs: 
            1. start: 3x1 array (x,y,heading)
            2. end: 3 x 1 array (x,y,heading), where heading is not that important
            3. cell_size: float
        outputs:
            1. path from start to end within cell_size. 3 arrays each column is [x,y]
    """
    start = np.array(start)
    end = np.array(end)
    vel = np.array([0,0])
    delta_t = 0.1
    x_vec = [start[0]]
    y_vec = [start[1]]
    theta_vec = [start[2]]

    while np.linalg.norm(start[:-1]-end[:-1])>cell_size/10.0:
        vel = controller(start,end,vel,delta_t,add_noise)
        start = motion_model(start,vel,delta_t, add_noise)
        x_vec.append(start[0])
        y_vec.append(start[1])
        theta_vec.append(start[2])
    return x_vec, y_vec, theta_vec



#def main():
#
#broken
#
#    cell_size_real = 1  #1m*1m
#    map_x_range = np.array([-2,5])
#    map_y_range = np.array([-6,6])
#
#
#    path = astar_offline(chessboard, start, end)
#    #path = [(0, 0), (1, 0), (2, 1), (3, 2), (4, 3), (5, 4), (6, 5), (7, 6)]
#
#    for waypoint in path:
#        chessboard[waypoint[0]][waypoint[1]] = 2
#    print path
#
#
#main()
