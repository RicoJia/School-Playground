# 2D RRT, RRT-Connect Implementation 

This project is an exercise of the [Deep Blue Academy Class - Motion Planning for Mobile Robots](www.shenlanxueyuan.com). The project includes the following objectives:
1. An implementation of RRT\* using the ```ompl``` library
2. A custom implementation of RRT using a custom KD Tree / PCL KD tree.
3. A custom Implementation of RRT-connect using a custom kd tree / PCL tree

### RRT-Connect
The core idea of RRT-Connect is 
    1. Establish 2 KD-trees at the start and end points 
    2. While two trees not meet: 
        1. generate a point q_new, 
        2. Within step size, find q_new_1, q_new_2 towards q_new for each KD-tree
        3. From each KD-tree, find the nearest points to q_new_1, q_new_2: q_near_1, q_near_2
        4. Perform collision check: (q_near_1, q_new_1), (q_near_2, q_new_2). If collision free add q_new_1, q_new_2
        5. if q_new_1 and q_new_2 are the same point, the two trees meet 
    3. Backtrack path from the last q_new_1, q_new_2 towards start and end. Then we get the full path 

<p align="center">
<img src="https://user-images.githubusercontent.com/39393023/150183825-a4ee0acd-f628-4e71-b72e-e14aac4e58e7.png""" height="400" width="width"/>
</p>

## Build and Install
1. System Environment: 
    - Ubuntu 18.04
    - ROS Melodic 
    - Install OMPL and PCL
