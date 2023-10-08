from control import controller,motion_model
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from utility import display_grid,to_matrix_coord,load_landmark_data,to_map_coord, display_grid_live
from A_star_heapq import astar_heapq

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


def main_q9( show_arrows = False, add_noise = False):
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

        result = astar_heapq(data,start_mat_coord,goal_mat_coord) #list
        turtle_path_start = path[0] #start in map coord for each way point
        turtle_path_start.append( heading_init )
        turtle_path_x = np.array([turtle_path_start[0]])  #1 x m np array
        turtle_path_y = np.array([turtle_path_start[1]])  #1 x m np array
        turtle_path_theta = np.array([turtle_path_start[2]])  #1 x m np array

        for waypoint in result: 
            data[waypoint[0]][waypoint[1]] = p
            #each waypoint in data is transformed into real map coordinate, then we get a set of turtle's path
            control_end = to_map_coord(waypoint,x_range, y_range, cell_size)
            control_end = np.append( control_end,0)  #adding dummy 0 to fit into control
            x_vec,y_vec,theta_vec = control_interface(turtle_path_start, control_end,cell_size,add_noise)                
            turtle_path_start = np.array([x_vec[-1],y_vec[-1],theta_vec[-1]])
            turtle_path_x = np.append(turtle_path_x,x_vec)#1*m np array, 
            turtle_path_y = np.append(turtle_path_y,y_vec)#1*m np array
            turtle_path_theta = np.append(turtle_path_theta,theta_vec)#1*m np array
            
        turtle_path = ( turtle_path_x, turtle_path_y, turtle_path_theta)
        
        #displaying the final result, after calculating the result
        #inflate start and goal
        inflated_start = inflate_point_end( start_mat_coord, inflate_pt_size/2)
        inflated_goal = inflate_point_end ( goal_mat_coord,inflate_pt_size/2)
        for pt in inflated_start:
            if out_of_boudary(pt,row_num, cln_num) == False:
                data[pt[0],pt[1]] = s
        for pt in inflated_goal:
            if out_of_boudary(pt,row_num, cln_num) == False:
                data[pt[0],pt[1]] = g
        
        display_grid_live(data, turtle_path, x_range,y_range,cell_size,"World Map -  Controller + A* Fine-Grid" , show_arrows)

def control_interface(start,end,cell_size,add_noise):
    """ 
        inputs: 
            1. start: 3x1 array
            2. end: 3 x 1 array
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

    while np.linalg.norm(start[:-1]-end[:-1])>cell_size/3.0:
        vel = controller(start,end,vel,delta_t,add_noise)
        start = motion_model(start,vel,delta_t, add_noise)
        x_vec.append(start[0])
        y_vec.append(start[1])
        theta_vec.append(start[2])
    return x_vec, y_vec, theta_vec

if __name__ == '__main__':
    main_q9()
