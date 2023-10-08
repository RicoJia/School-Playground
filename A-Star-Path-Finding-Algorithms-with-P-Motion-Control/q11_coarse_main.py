

from control import controller,motion_model
import numpy as np
import math
import matplotlib.pyplot as plt
import time
from utility import display_grid,to_matrix_coord,load_landmark_data,to_map_coord, display_grid_live
from q11_coarse_algo import astar_online_control_coarse_map

def q11_coarse_main( show_arrows = False, add_noise = False):
    


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
    #so start and goals are put in the form of [[start0,goal0],[start1,goal1]...].This is a 3d arra
    start_goal_table = np.array([[starts[i],goals[i]] for i in range(len(starts))]) 
    heading_init = -1*np.pi/2

    
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

        #issuing control here TODO
        result, turtle_path = astar_online_control_coarse_map(data,start_mat_coord,goal_mat_coord, add_noise)

        for waypoint in result:
            data[waypoint[0]][waypoint[1]] = p
        #displaying the final result, after calculating the result
        data[ start_mat_coord[0], start_mat_coord[1]] = s
        data[ goal_mat_coord[0], goal_mat_coord[1]] = g
        #display_grid(data,x_range,y_range,cell_size,"World Map - Q11 Online Planning + Control")

        display_grid_live(data, turtle_path, x_range,y_range,cell_size,"World Map - Q11 Online Planning + Control",show_arrows)

if __name__ == '__main__':
    q11_coarse_main()
