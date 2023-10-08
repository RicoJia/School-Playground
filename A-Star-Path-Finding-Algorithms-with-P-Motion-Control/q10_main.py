from control import controller,motion_model
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from utility import display_grid,to_matrix_coord,load_landmark_data,to_map_coord, display_grid_live
from q10_algo import astar_heapq_online_control


pause_time = 0.08
o = 3   #obstacle value in maze

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

def out_of_boudary(pt_index,row_num,col_num):
    """ Checks if a point is within the range of a matrix"""
    #[)
    if pt_index[0] < 0 or pt_index[0] >= row_num:
        return True
    if pt_index[1] < 0 or pt_index[1] >= col_num:
        return True
    return False


def main_q10_fine_grid( show_arrows = False, add_noise = False):
    """ Works with finer grid, with inflated points for start,goal, and obstacles """
    #configuration variables

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
    heading_init = -1*np.pi/2
    
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

        result,turtle_path = astar_heapq_online_control(data,start_mat_coord,goal_mat_coord, add_noise) #list

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
        
        display_grid_live(data, turtle_path, x_range,y_range,cell_size,"World Map -  Controller + A* Fine-Grid" , show_arrows)

        #display_grid(data, x_range,y_range,cell_size,"World Map - A* Fine-Grid")



if __name__ == '__main__':
    main_q10_fine_grid()
