# A\* and JPS (Jump Point Search)
## Common Framework of A\* and JPS 
1. Create an open List with start node, and empty closed list.
2. Loop until loop is empty 
    1. Pop the first node in the priority queue,
        - mark it as visited
        - and put it in closed list
    3. If the node is goal, quit
    4. For each neighbor node, 
        - If it's has been expanded, skip because it must come from an older parent node, hence its total cost is higher.
        - calculate ```h(x)```, update it h(x) with ```min(h(x), h(x)_0)```. Meanwhile, update parent node if necessary. 
        - If "unvisited": push on to priority queue. If it's already in open list, do nothing.
3. Back track from goal node following the "parent pointer"

## Comparison between different Heuristrics. 
1. **We generally want better smoothness"** in paths, even though they're already optimal. Note that the actual cost being used here is **Manhattan Distance**. In below illustrations, blue cells are explored cells; green cells on the ground plane mark the calculated path. The other colored cells represent a randomly generate map. 
    1. Manhattan Distance
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/140439330-d520804e-ac17-4351-9b6b-92e428925377.png" height="400" width="width"/>
        </p>
      
        - The explored area is quite uniform. The path is optimal, and for the most part, it prefers straight-lines. But it may get very close to obstacles

    2. Euclidean Distance
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/140440663-92af8eb0-b46a-45a9-b429-5528b6e519fe.png" height="400" width="width"/>
        </p>

        - The explored area is still pretty big, and the path might get dangerously close to obstacles at some point. In short, no significant difference from Manhattan Distance. 
    3. Diagonal Distance (AKA the "best distance")
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/140441407-c0a6e082-0fae-4a0e-953c-7e78f812341a.png" height="400" width="width"/>
        </p>

        - The explored area is smaller than for Euclidean distance or manhattan Distance. The paths are more "straight", which usually start going straight, then make a turn and keep going straight for a while. So it is better than the previous two.

2. Tie breaker: preference of straight line over zig-zags: looks pretty much the same as the previous ones, and it has a larger explored area than the that for diagonal distance

3. JPS vs ```A*```
    - First 2D A* is magnitudes faster than 3D (In some experiments, it's <10ms vs 100-1000ms)
    - JPS expands a few times fewer nodes than ```A*```, but it's actually **a few times slower!**
    - but JPS is more "straight", compared to ```A*```
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/140598529-1a1db42e-9895-4da2-944f-d102ee3fd24f.png" height="400" width="width"/>
        </p>


## Problems Encountered
1. Path was super long, not optimal at all! 
    1. Reduce the problem to 2D: Only work on 2D!
    2. Check the following items, one by one using **gdb** 
        - receive the right goal, start points
        - popping the right node (smallest cost)
        - adding the right 8 neighbors
        - calculating the right cost:
        - updating the right nodes (open & unexpanded)
        - adding the right nodes (unexpanded)

## Helpful References
1. [Heuristics Comparison](https://www.growingwiththeweb.com/2012/06/a-pathfinding-algorithm.html)
2. The **BEST tutorial** I found is actually [this, with pseudocode](https://gamedevelopment.tutsplus.com/tutorials/how-to-speed-up-a-pathfinding-with-the-jump-point-search-algorithm--gamedev-5818)
    - [JPS Online Demo](http://qiao.github.io/PathFinding.js/visual/)
