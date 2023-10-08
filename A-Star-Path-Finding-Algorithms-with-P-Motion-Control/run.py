"""
Author: Ruotong Jia

Instruction:
    1. Simply run the run.py to see the result!!
Tasks: 
1. Online A*
    Key steps: 
        1. Add start node to open list
        2. Enters a while loop. exits when open_list is empty
        3. In open list, find the node to be expanded, which has the lowest f
        4. Put the node in closed list, meaning this node has been explored
        5. Find children, as long as the child index is within the matrix.
        6. For every child, see if the child is in the closed loop. If yes, then skip the rest of the loop and start on the next child.  
        7. Evaluate the f = g+h for each child. h = 1 for an empty cell, h = 1000 for occupied cells
        8. Abandon the child  if it's already in the open list and its true cost increases in the current iteraration. 
    
    Instruction: 
        1. color coding: 
          s -> red (start), p -> green (path), g -> blue (goal), o -> black (obstacle)

2. Offline A*:
    Key Steps: 
    Everthing is the same as above, except between step 3 and 4: 
    --->3.5 update neighbors of the maze, according to the true maze

3. Heapq A*
    Key steps: 
    Everthing is the same as above, except between step 3 and 4:
    Major difference: using heapque for faster sorting. 

4.Question 10 algorithms 
    Online planning and simultaneous control for fine grid with step 7 start and goals. 
"""

import numpy as np
from utility import display_grid,to_matrix_coord,load_landmark_data
from A_star_offline import astar
from A_star_online import astar_online
from A_star_heapq import astar_heapq
from q9_main import main_q9
from q10_main import main_q10_fine_grid
from q11_coarse_main import q11_coarse_main
from q11_fine_main import main_q11_fine_grid


def main_offline():
    #assigning status name to color: start -> red (start) o -> black (obstacle), p -> green (path), g -> blue (goal)
    p = 0
    s = 1
    g = 2
    o = 3
    #set up the real map parameters:
    x_range = [-2,5]    #cln 
    y_range = [-6,6]    #row
    cell_size = 1

    starts =[(0.5,-1.5),(4.5, 3.5),(-0.5, 5.5)]
    goals = [(0.5, 1.5),(4.5,-1.5),(1.5,-3.5)]
    #so start and goals are put in the form of [[start0,goal0],[start1,goal1]...].This is a 3d array. 
    start_goal_table = np.array([[starts[i],goals[i]] for i in range(len(starts))])
    #loading landmarks
    landmark_table = load_landmark_data()

    #set up the data matrix parameters
    row_num = (y_range[1]-y_range[0])/cell_size
    cln_num = (x_range[1]-x_range[0])/cell_size

    # fill in the map data for each path. A figure will show once the previous one is closed. 
    for path in start_goal_table:
        # make an empty data set
        data = np.ones((row_num, cln_num)) * np.nan
        # [start_coord,goal_coord], getting matrix coord of start and goal
        start_mat_coord = to_matrix_coord(path[0],x_range,y_range,cell_size)
        goal_mat_coord = to_matrix_coord(path[1],x_range,y_range,cell_size)

        for landmark in landmark_table:
            lm_mat_coord = to_matrix_coord( landmark, x_range,y_range,cell_size)
            data[lm_mat_coord[0], lm_mat_coord[1]]=o

        result = astar(data,start_mat_coord,goal_mat_coord)
        for waypoint in result:
            data[waypoint[0]][waypoint[1]] = p
        #displaying the final result, after calculating the result
        data[ start_mat_coord[0], start_mat_coord[1]] = s
        data[ goal_mat_coord[0], goal_mat_coord[1]] = g
        display_grid(data,x_range,y_range,cell_size,"World Map - A* Offline")

def main_online():
    #assigning status name to color: start -> red (start) o -> black (obstacle), p -> green (path), g -> blue (goal)
    p = 0
    s = 1
    g = 2
    o = 3
    #set up the real map parameters:
    x_range = [-2,5]    #cln 
    y_range = [-6,6]    #row
    cell_size = 1

    starts =[(0.5,-1.5),(4.5, 3.5),(-0.5, 5.5)]
    goals = [(0.5, 1.5),(4.5,-1.5),(1.5,-3.5)]
    #so start and goals are put in the form of [[start0,goal0],[start1,goal1]...].This is a 3d array. 
    start_goal_table = np.array([[starts[i],goals[i]] for i in range(len(starts))])
    #loading landmarks
    landmark_table = load_landmark_data()

    #set up the data matrix parameters
    row_num = (y_range[1]-y_range[0])/cell_size
    cln_num = (x_range[1]-x_range[0])/cell_size

    # fill in the map data for each path. A figure will show once the previous one is closed. 
    for path in start_goal_table:
        # make an empty data set
        data = np.ones((row_num, cln_num)) * np.nan
        # [start_coord,goal_coord], getting matrix coord of start and goal
        start_mat_coord = to_matrix_coord(path[0],x_range,y_range,cell_size)
        goal_mat_coord = to_matrix_coord(path[1],x_range,y_range,cell_size)

        for landmark in landmark_table:
            lm_mat_coord = to_matrix_coord( landmark, x_range,y_range,cell_size)
            data[lm_mat_coord[0], lm_mat_coord[1]]=o

        result = astar_online(data,start_mat_coord,goal_mat_coord)
        for waypoint in result:
            data[waypoint[0]][waypoint[1]] = p
        #displaying the final result, after calculating the result
        data[ start_mat_coord[0], start_mat_coord[1]] = s
        data[ goal_mat_coord[0], goal_mat_coord[1]] = g
        display_grid(data,x_range,y_range,cell_size,"World Map - A* Online")

def main_heapq():
    """ Works with finer grid, with inflated points for start,goal, and obstacles """
    #assigning status name to color: start -> red (start) o -> black (obstacle), p -> green (path), g -> blue (goal)
    p = 0
    s = 1
    g = 2
    o = 3
    #set up the real map parameters:
    x_range = [-2,5]    #cln 
    y_range = [-6,6]    #row
    cell_size = 0.1
    
    start_goal_table = [[[ 2.45,-3.55 ],[ 0.95,-1.55 ]],
            [[4.95,-0.05 ],[ 2.45, 0.25 ]],
            [[-0.55, 1.45],[ 1.95, 3.95 ]]]

    #loading landmarks
    landmark_table = load_landmark_data()

    #set up the data matrix parameters
    row_num = int((y_range[1]-y_range[0])/cell_size)
    cln_num = int((x_range[1]-x_range[0])/cell_size)

    # fill in the map data for each path. A figure will show once the previous one is closed. 
    for path in start_goal_table:
        # make an empty data set
        data = np.ones((row_num, cln_num)) * np.nan
        # [start_coord,goal_coord], getting matrix coord of start and goal
        start_mat_coord = to_matrix_coord(path[0],x_range,y_range,cell_size)
        goal_mat_coord = to_matrix_coord(path[1],x_range,y_range,cell_size)
        inflate_pt_size = 7
        # inflate landmarks
        for landmark in landmark_table:
            lm_mat_coord = to_matrix_coord( landmark, x_range,y_range,cell_size)
            inflated_pt_ls = inflate_point( lm_mat_coord, inflate_pt_size)
            for pt in inflated_pt_ls:
                if out_of_boudary(pt,row_num, cln_num) == False:
                    data[pt[0], pt[1]]=o
        
        result = astar_heapq(data,start_mat_coord,goal_mat_coord)
        for waypoint in result:
            data[waypoint[0]][waypoint[1]] = p
        #displaying the final result, after calculating the result
        #inflate start and goal 
        inflated_start = inflate_point_end( start_mat_coord, inflate_pt_size/2)
        inflated_goal = inflate_point_end( goal_mat_coord,inflate_pt_size/2)
        for pt in inflated_start:
            if out_of_boudary(pt,row_num, cln_num) == False: 
                data[pt[0],pt[1]] = s
        for pt in inflated_goal:
            if out_of_boudary(pt,row_num, cln_num) == False: 
                data[pt[0],pt[1]] = g
        display_grid(data,x_range,y_range,cell_size,"World Map - A* Fine-Grid - heuristic 3")

def inflate_point(center_pos,length):
    """ Returns a list of point indices that comprise an inflated point on a matrix
        NOTICE: Going out of boundary in the matrix is not being considered here 
        Params:    
            center_pos: center position of the inflated point
            length: odd integer length of one side of the inflated point in matrix. """
    ret_list = []
    for row in range(center_pos[0]-(length-1)/2,center_pos[0]+(length-1)/2+1):
        for cln in range(center_pos[1]-(length-1)/2,center_pos[1]+(length-1)/2+1):
            new_pt = [row,cln]
            ret_list.append(new_pt)

    return ret_list 

def inflate_point_end(center_pos,length = 3):
    """ Returns a list of point indices that comprise an inflated point on a matrix
        NOTICE: Going out of boundary in the matrix is not being considered here 
        Params:    
            center_pos: center position of the inflated point
            length: odd integer length of one side of the inflated point in matrix. """
    ret_list = []
    for row in range(center_pos[0]-(length-1)/2,center_pos[0]+(length-1)/2+1):
        for cln in range(center_pos[1]-(length-1)/2,center_pos[1]+(length-1)/2+1):
            new_pt = [row,cln]
            ret_list.append(new_pt)

    return ret_list 

def out_of_boudary(pt_index,row_num,col_num):
    """ Checks if a point is within the range of a matrix"""
    #[)
    if pt_index[0] < 0 or pt_index[0] >= row_num:
        return True
    if pt_index[1] < 0 or pt_index[1] >= col_num:
        return True
    return False

if __name__ == '__main__':
    main_offline()
    main_online()
    main_heapq()
    show_arrows = False
    add_noise = True
    main_q9 ( show_arrows, add_noise)
    main_q10_fine_grid( show_arrows, add_noise )    #in q10_main.py
    q11_coarse_main( show_arrows, add_noise )
    main_q11_fine_grid( show_arrows, add_noise)
