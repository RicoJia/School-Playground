1. The entrance function is in pso/pso_test_obstacle.m. Your task is to complete the pso_select.m function inside the "To be finished by the student" part.

2. The input to the pso_select function are:
   - theta (1 x 1): current heading of the vehicle
   - omega (1 x 1): current angular speed of the vehicle's heading
   - v_ini (1 x 1): current forward speed of the vehicle
   - p0 (2 x 1): current position of the vehicle [x;y]
   - last_theta (1 x 1): previous heading target
   - last_v (1 x 1): previous speed target

3. The output to the pso_select function: ```global_best (1 x 3):``` is a vector consists of [theta_target, velocity_target, best_cost] 

4. Development notes: 
    1. Select the best PSO: 
        - each particle is a guess of v and w of the end point of a trajectory, after time tf. 
            - ```(v,w)```initial value is random 
            - **Note: a particle is always a guess of parameters.**
            - given current ```v,w, current_pos``` and guess ```v,w```, generate a traj with 40 points ```(x,y)``` 
            - get the cumulated cost of the trajectory
                1. In costmap, the global goal is already represented by the cell with 0 cost. The cost also encompasses the distance to the nearest obstacle, like ```log(dist_to_nearest_obs))```. 
                2. Therefore, the ```(v,w)``` with lowest cost must be collision free, and will lead to the goal
            - save the historical best cost and current global best velocity
            - update particle's ```(v,w)``` with velocity of historical best and current global best
    2. Move the robot using ```(v,w)``` for 0.1s



