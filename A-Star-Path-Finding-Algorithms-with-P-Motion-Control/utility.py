import matplotlib.pyplot as plt 
from matplotlib import colors
import numpy as np
import matplotlib

""" Provides utilities for: 
        1. Transforming a real map location to matrix (row, cln) indices
        2. Displaying the grid with the data matrix that marks all waypoints' status. The grid 
    transforms data matrix's indices to the real world locations. 
           Color Mapping: red for obstacles, green for path (including the starting point), blue for end goal
        3. loading the landmark positions from .dat into a 2d list.
        4. Displaying a grid with live updates on the car position
        5. Transforming a matrix location (row,cln) to matrix
    """

def display_grid_live(data,turtle_path, x_range,y_range,cell_size,plot_title, show_arrows = False):   #TODO
    """ Inputs:
            1. data: matrix of the planned path and obstacles
            2. turtle_path: tuple that consists of turtle_path_x, turtle_path_y, turtle_heading - Real Map Coordinates        
            3. x_range, y_range: real map axes' ranges
            4. cell_size: assume a cell is a square, this is the length of a side. 
        Outputs:
            1. plot that shows path waypoints in a matrix
            2. same plot that shows real life turtle's trajectory
    """
    # make a figure + axes
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    # make color map
    my_cmap = matplotlib.colors.ListedColormap(['g','r','b','k'])
    # set the 'bad' values (nan) to be white and transparent
    my_cmap.set_bad(color='w', alpha=0)
    # draw the grid
    if cell_size >= 1:
        for x in np.arange(y_range[0],y_range[1]+1,cell_size):
            ax.axhline(x, lw=2, color='k', zorder=5)
        for y in np.arange( x_range[0],x_range[1]+1,cell_size):
            ax.axvline(y, lw=2, color='k', zorder=5)
    # draw the boxes
    ax.imshow(data, interpolation='none', cmap=my_cmap, extent=[x_range[0], x_range[1], y_range[1], y_range[0]], zorder=0)
    # turn off the axis labels
    #ax.axis('off')
    ax.set_title(plot_title)
    plt.xlabel("x coordinate")
    plt.ylabel("y coordinate")
    
    #plot turtle path
    turtle_path_x = turtle_path[0]
    turtle_path_y = turtle_path[1]
    turtle_heading = turtle_path[2]
    plt.plot( turtle_path_x, turtle_path_y, color = 'black',linewidth = 1.4)
    
    #plot arrows at every other point
    arrow_counter = 10
    if show_arrows == True:
        for i in range(len(turtle_path_x)):
            if arrow_counter == 10:
                arr_start = np.array([turtle_path_x[i],turtle_path_y[i]])
                arr_theta = turtle_heading[i]
                arr_increment = 1*cell_size*np.array([np.cos(arr_theta),np.sin(arr_theta)])
                plt.arrow(arr_start[0],arr_start[1],arr_increment[0], arr_increment[1],head_width=0.02, head_length=0.01, fc='lightblue', ec='black') 
                arrow_counter = 1
            else:
                arrow_counter += 1

    #add pause
    plt.show()   

def to_map_coord(matrix_indices,x_range, y_range,cell_size):
    """
        Inputs:
            1.matrix indices
            x,y ranges
        Outputs:
            1. Map coordinate of the center of the matrix element. 
    """
    origin = [x_range[0],y_range[0]]
    matrix_indices_inv = matrix_indices[::-1]
    map_coord = np.array( matrix_indices_inv)*cell_size+np.array([0.5,0.5])*cell_size + np.array(origin)
    return map_coord

def display_grid(data,x_range,y_range,cell_size,plot_title):
    """ Inputs: 
            1. data - matrix that marks the status of all waypoints
            2. x_range, y_range: the axes ranges in the real world map
            3. cell_size: assume a cell is a square, this is the length of a side. 
            2. x_range, y_range: the axes ranges in the real world map
            3. cell_size: assume a cell is a square, this is the length of a side. 
        Outputs:
            1. a plot that shows waypoints, etc. 
            2. Color Mapping: red for obstacles, green for path (including the starting point), blue for end goal
    """
    # make a figure + axes
    fig, ax = plt.subplots(1, 1, tight_layout=True)
    # make color map
    my_cmap = matplotlib.colors.ListedColormap(['g','r','b','k'])
    # set the 'bad' values (nan) to be white and transparent
    my_cmap.set_bad(color='w', alpha=0)
    # draw the grid
    if cell_size >= 1:
        for x in np.arange(y_range[0],y_range[1]+1,cell_size):
            ax.axhline(x, lw=2, color='k', zorder=5)
        for y in np.arange( x_range[0],x_range[1]+1,cell_size):
            ax.axvline(y, lw=2, color='k', zorder=5)
    # draw the boxes
    ax.imshow(data, interpolation='none', cmap=my_cmap, extent=[x_range[0], x_range[1], y_range[1], y_range[0]], zorder=0)
    # turn off the axis labels
    #ax.axis('off')
    ax.set_title(plot_title)
    plt.xlabel("x coordinate")
    plt.ylabel("y coordinate")
    
    #add pause
    plt.show()

def to_matrix_coord(map_coord,x_range,y_range,cell_size):
    """
        Inputs: 
            1. map_coord: a point's coordinates on a map
        Outputs: 
            1. the matrix indices of the point
    """
    x = map_coord[0]
    y = map_coord[1]    
    x_matrix = int((x-x_range[0])/cell_size)    #cln number
    y_matrix = int((y-y_range[0])/cell_size)    #row number
    return (y_matrix,x_matrix)

def load_landmark_data():
    file_name = 'ds1_Landmark_Groundtruth.dat'
    with open(file_name):
        full_file = np.loadtxt(file_name)
        landmark_table = [[row[1],row[2]] for row in full_file]
    
    return np.array(landmark_table)

#TODO
def test():
    x_range = [-2,5]    #cln
    y_range = [-6,6]    #row
    cell_size = 1

    pt = [-2,-6]       #
    pt_mat = to_matrix_coord(pt, x_range, y_range, cell_size)
    pt_tr = to_map_coord(pt_mat, x_range, y_range, cell_size)

    print "Original Pos: "
    print pt
    print "matrix Pos:" 
    print pt_mat
    print "Transformed map position: "
    print pt_tr

if __name__ == "__main__":
    a = "keep pushing it"    
    #test()
    #test_arrow()


#def main():
#    #assigning status name to color: o -> red (obstacle), p -> green (path), g -> blue (goal)
#    p = 0
#    o = 1
#    g = 2
#    #set up the real map parameters:
#    x_range = [-2,5]    #cln 
#    y_range = [-6,6]    #row
#    cell_size = 1
#
#    starts =[[0.5,-1.5],[4.5, 3.5],[-0.5, 5.5]] 
#    goals = [[ 0.5, 1.5 ],[ 4.5,-1.5 ],[1.5,-3.5]]
#    #so start and goals are put in the form of [[start0,goal0],[start1,goal1]...].This is a 3d array. 
#    start_goal_table = np.array([[starts[i],goals[i]] for i in range(len(starts))])    
#    #loading landmarks
#    landmark_table = load_landmark_data()
#
#    #set up the data matrix parameters
#    row_num = (y_range[1]-y_range[0])/cell_size
#    cln_num = (x_range[1]-x_range[0])/cell_size
#        
#    # fill in the map data for each path. A figure will show once the previous one is closed. 
#    for path in start_goal_table:
#        # make an empty data set
#        data = np.ones((row_num, cln_num)) * np.nan
#        # [start_coord,goal_coord], getting matrix coord of start and goal
#        start_mat_coord = to_matrix_coord(path[0],x_range,y_range,cell_size)
#        data[ start_mat_coord[0], start_mat_coord[1]] = p
#        goal_mat_coord = to_matrix_coord(path[1],x_range,y_range,cell_size)
#        data[ goal_mat_coord[0], goal_mat_coord[1]] = g
#            lm_mat_coord = to_matrix_coord( landmark, x_range,y_range,cell_size)
#            data[lm_mat_coord[0], lm_mat_coord[1]]=o
#
#        display_grid(data,x_range,y_range,cell_size)
#
