# Motion Planning For Quadrotor Simulation 

## Introduction
This is the final project of the [Deep Blue Academy Class - Motion Planning for Mobile Robots](www.shenlanxueyuan.com). The project includes the following objectives: 
1. Path planning 
2. Trajectory generation
3. Trajectory replanning 
4. Trajectory Replanning for sensors' limited field of view

This package includes: 
- random complex: generate 3D random obstacles in point cloud 
- waypoint_generator: 
- odom visualization: visualize the quadrotor
- pcl_render_node: Simple model for camera/Lidar with a limited field of view. This module outputs point clouds of obstacles locally
- trajectory_generator_node: generate a **smooth, executable** trajectory
- traj_server: converts trajectory to control commands 
- so3_control: converts control commands to actual control variables. 
- quadrotor_simulator_so3: model of a quadrotor. 

Specifically, trajectory_generator_node is the main planning node with the following inputs and outputs: 
<p align="center">
<img src="https://user-images.githubusercontent.com/39393023/149847480-1ca9ffdb-4af1-49ff-b815-851e0f745191.png""" height="300" width="width"/>
</p>

## Installations & Dependencies
1. Dependencies
    ```bash
    sudo apt-get install cmake libopenblas-dev liblapack-dev libarpack-dev libarpack2-dev libsuperlu-dev
    ```
2. Install Armadillo
    ```
    xz -d armadillo-9.870.2.tar.xz
    tar -xvf armadillo-9.870.2.tar
    cd armadillo-9.870.2
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

## Notes
1. Workflow
    - Planning and execution strategy: 
        1. Plan a global path 
        2. In the locally visible range ```psi_p```, we plan a local trajectory
        3. However due to sensor noises, we have a relatively reliable "executable" range ```psi_e```, in which we actually execute the local plan
        4. Once we step outside ```psi_e```, we will replan. Also, we check if there are any collisions. In real time. If there is, we also replan
            <p align="center">
            <img src="https://user-images.githubusercontent.com/39393023/149848967-7c0d88d8-1d43-45c7-9f85-d3e9c4178dd0.png""" height="200" width="width"/>
            </p>
    - Flowchart
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/149856794-67b3449a-082b-45df-831d-6f10ce2cfcb7.png""" height="500" width="width"/>
        </p>

2. Demo 

3. Performance Comparison
    - ```A*``` vs ```simplified path```
    - ```trajectory reoptimization``` 
    - Other

4. Bonus: 
    - Simulation Model of the quadrotor
    - traj server: what inputs& outputs

5. Side Notes For Developers
    1. We use Ramer–Douglas–Peucker_algorithm (RDP) to reduce the size of A* waypoints. 
        - However, after size reduction, we may end up with segments hitting obstacles. 
        - For that, we can hypothetically shoot out rays from the start point to each waypoint and check for obstacles. 
        - The last fallback is to insert more waypoints and replan
    2. Replanning is triggered currently by time (replan_time) rather than position. Also, we currently don't check for almost hitting obstacles. 
        - This is okay because A* and minimum snap is fast enough for mostly static environments
    3. We assume regions outside of maps to be occupied by obstacles. 
    4. Due to the coupling in car dynamics (differential drive and Ackerman Steering), minimum snap must be modified. 

## TODO
1. 阅读代码：画出trajectory_generator_node运行流程图，重点是厘清
   1. 几个状态之间的切换过程；
   2. 各个主要功能之间的调用关系，不需要深入到各个功能的内部例如A*的流程。
2. path planning：推荐实现方案为A*，也可采用其他方案；
3. simplify the path：将整条path简化为少数几个关键waypoints，推荐方案为RDP算法；
4. trajectory optimization：推荐实现方案为minimum snap trajectory generation，也可采用其他方案；
5. safe checking: 验证生成的轨迹是否安全；
6. trajectory reoptimization：此环节只针对使用minimum snap trajectory generation的时候。由于该方法只对连续性进行优化，并不能保证优化后的轨迹不会撞上障碍物，所以需要对撞上障碍物的部分重优化。推荐方法详见文献：["Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments" part 3.5](https://dspace.mit.edu/bitstream/handle/1721.1/106840/Roy_Polynomial%20trajectory.pdf?sequence=1&isAllowed=y)。

伪代码（来源：[维基百科](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm)）：

```
function DouglasPeucker(PointList[], epsilon)
    // Find the point with the maximum distance
    dmax = 0
    index = 0
    end = length(PointList)
    for i = 2 to (end - 1) {
        d = perpendicularDistance(PointList[i], Line(PointList[1], PointList[end]))
        if (d > dmax) {
            index = i
            dmax = d
        }
    }

    ResultList[] = empty;

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        // Recursive call
        recResults1[] = DouglasPeucker(PointList[1...index], epsilon)
        recResults2[] = DouglasPeucker(PointList[index...end], epsilon)

        // Build the result list
        ResultList[] = {recResults1[1...length(recResults1) - 1], recResults2[1...length(recResults2)]}
    } else {
        ResultList[] = {PointList[1], PointList[end]}
    }
    // Return the result
    return ResultList[]
end
```

