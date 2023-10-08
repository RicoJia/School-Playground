"""
Author: Ruotong Jia
Tasks: Online A* for fine grid
    Key steps: 
        1. Add start node to open list
        2. Enters a while loop. exits when Olist is empty
        3. In open list, find the node to be expanded, which has the lowest f
        4. Put the node in closed list, meaning this node has been explored
        5. Find neighbors, as long as the child index is within the matrix.
        6. For every child, see if the child is in the closed loop. If yes, then skip the rest of the loop and start on the next child.  
        7. Evaluate the f = g+h for each child. h = 1 for an empty cell, h = 1000 for occupied cells
        8. Abandon the child  if it's already in the open list and its true cost increases in the current iteraration. 
    
    Instruction: 
        1. color coding: 
          s -> red (start), p -> green (path), g -> blue (goal), o -> black (obstacle)

"""

import numpy as np
import heapq
from utility import display_grid,to_matrix_coord,load_landmark_data

o = 3   #obstacle value in matrix

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

def astar_heapq(chessboard, start, end):
    """
    Main difference from the regular implementation: heapque is used for faster sorting. 
    add (s_node.f,s_node) to Olist, enter while loop to see if list is empty -> heapify Olist ->access the first element to find expanded_node ->heappop the list-> ... -> heappush (child.f,child) when adding a child to the Olist. 
    """

    s_node = Node(None, start)
    s_node.g = s_node.h = s_node.f = 0 
    e_node = Node(None, end)
    e_node.g = e_node.h = e_node.f = 0 

    Olist = []
    Clist = []
    Olist.append((s_node.f,s_node))

    num_row = len(chessboard)
    num_cln = len(chessboard[0])
    
    while len(Olist) > 0:

        # update heapq
        heapq.heapify(Olist) 
        expanded_node = Olist[0][1]


        heapq.heappop(Olist)
        Clist.append(expanded_node)

        if expanded_node == e_node:
            path = []
            a = expanded_node
            current = a
            while current is not None:
                path.append(a.position)
                a = current.parent
                current = a
            return path[::-1] 

        neighbors = []
        for new_position in [ (0, 1), (0, -1), (1, 0), (-1, 0), (-1, 1), (-1, -1),(1, 1), (1, -1) ]: 

            node_position = (expanded_node.position[0] + new_position[0], expanded_node.position[1] + new_position[1])

            if node_position[0] < 0 or node_position[0] > (num_row-1) or node_position[1] > (num_cln-1) or node_position[1] < 0:
                continue

            new_node = Node(expanded_node, node_position)

            neighbors.append(new_node)

        for child in neighbors:

            if child in Clist:
                continue

            #Please notice that here unoccupied cell is np.nan
            if chessboard[child.position[0]][child.position[1]] == o:
                child.g = expanded_node.g+1000
            else:
                child.g = 1 + expanded_node.g
            
            co = 1.40
            hval =   co*(((child.position[1] - e_node.position[1]) ** 2) + ((child.position[0] - e_node.position[0]) ** 2))
  
            child.h = hval
            child.f = hval + child.g

            pass_flag = False
            for onode_tuple in Olist:
                if child == onode_tuple[1] and child.g>onode_tuple[1].g:
                    pass_flag = True
            if pass_flag == True:
                continue

            Olist.append((child.f,child))

    return []


def main():
    #broken 

#building a matrix for

    cell_size_real = 1  #1m*1m
    map_x_range = np.array([-2,5])
    map_y_range = np.array([-6,6])


    path = astar_heapq(chessboard, start, end)
    #path = [(0, 0), (1, 0), (2, 1), (3, 2), (4, 3), (5, 4), (6, 5), (7, 6)]

    for waypoint in path:
        chessboard[waypoint[0]][waypoint[1]] = 2
    print path


if __name__ == '__main__':
    main()
