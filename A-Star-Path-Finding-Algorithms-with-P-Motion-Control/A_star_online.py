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

def astar_online(true_chessboard, start, end):

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

        if expanded_node == e_node:
            path = []
            b = 0
            a = expanded_node
            while a is not None:
                path.append(a.position)
                a = a.parent
            return path[::-1]


        neighbors = []
        for new_position in [ (0, 1), (0, -1), (1, 0), (-1, 0), (-1, 1), (-1, -1),(1, 1), (1, -1) ]: 

            node_position = (expanded_node.position[0] + new_position[0], expanded_node.position[1] + new_position[1])

            if node_position[0] < 0 or (num_row-1) < node_position[0]  or (num_cln-1) < node_position[1] or 0 > node_position[1] :
                continue

            new_node = Node(expanded_node, node_position)

            if not new_node in Clist:
                neighbors.append(new_node)

        for child in neighbors:
            #Please notice that here unoccupied cell is np.nan
            if chessboard[child.position[0]][child.position[1]] == o:
                child.g = expanded_node.g+1000
            else:
                child.g = 1 + expanded_node.g

            child.h =  ((child.position[1] - e_node.position[1]) ** 2) + ((child.position[0] - e_node.position[0]) ** 2)
            child.f = child.h + child.g

        min_child = get_min_child(neighbors)
        Olist.append(min_child)
    return []


#def main():
#
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
