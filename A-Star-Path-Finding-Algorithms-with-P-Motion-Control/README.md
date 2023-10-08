# A Star Path Finding Algorithms with P Motion Control
Author: Ruotong Jia

### Instruction
The python version used for this is python2. libraries required for running the program has been placed in requirements.txt You can install all required libraries using 

```$ pip install -r requirements.txt```
    
To see all results, Simply run 
```$ python run.py```

### Tasks: 

##### Online A*

Key steps: 

    1. Add start node to open list
    2. Enters a while loop. exits when open_list is empty
    3. In open list, find the node to be expanded, which has the lowest f
    4. Put the node in closed list, meaning this node has been explored
    5. Find children, as long as the child index is within the matrix.
    6. For every child, see if the child is in the closed loop. If yes, then skip the rest of the loop and start on the next child.  
    7. Evaluate the f = g+h for each child. h = 1 for an empty cell, h = 1000 for occupied cells
    8. Abandon the child  if it's already in the open list and its true cost increases in the current iteraration. 

How to Read the Graphs: 

    1. color coding: s -> red (start), p -> green (path), g -> blue (goal), o -> black (obstacle)
  

##### Offline A*:
Key Steps: 

    Everthing is the same as above, except between step 3 and 4 there is one more step: 
    update neighbors of the maze, according to the true maze

##### Heapq A*
Key steps: 

    Everthing is the same as above, except between step 3 and 4:
    Major difference: using heapque for faster sorting. 
