#include "JPS_searcher.h"

using namespace std;
using namespace Eigen;

inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    // we are following the current ptr's "cameFrom" direction
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    // number of neighbors
    int num_neib  = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    // and evaluate the forced neighbor we had?
    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Vector3i neighborIdx;
        Vector3i expandDir;

        if( dev < num_neib ) {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        else {
            // get the index of a forced neighbor.
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            // the supposedly forced neighbor might be occupied. If so, check if the occupied cell has forced neighbor.
            if( isOccupied(nx, ny, nz) ) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }

        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;
        
        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}

bool JPSPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    // expDir is used as an index, too. 
    neiIdx = curIdx + expDir;

    if( !isFree(neiIdx) )
        return false;

    if( neiIdx == goalIdx )
        return true;

    if( hasForced(neiIdx, expDir) )
        return true;

    // Current node doesn't have forced neighbors.
    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];   //number of neighbors to look at for the given direction

    // number of neighbors - might be complicated to look at.
    for( int k = 0; k < num_neib - 1; ++k ){
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        // In 2D, we check vertical and horizontal. 
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }

    // Haven't found anything, keep searching in the same direction: if horizontal, keep horizontal. If diagonal, keep diagonal
    return jump(neiIdx, expDir, neiIdx);
}

inline bool JPSPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch(norm1){
        case 1:
            // 1-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        default:
            return false;
    }
}

inline bool JPSPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool JPSPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

// Flow chart: The biggest diff is the search of neighbors
void JPSPathFinder::JPSGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    //STEP 2: some else preparatory works which should be done before while loop
    double tentative_gScore;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){
        //step 3: Remove the node with lowest cost function from open set to closed set
        currentPtr = openSet.begin() -> second;
        currentPtr -> id = -1;
        openSet.erase(openSet.begin());

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[JPS]{sucess} Time in JPS is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
            return;
        }
        //get the successors
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets); //we have done it for you
        
        // STEP 4:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets.at(i); 
            if (neighborPtr -> id == -1){
                continue; 
            }
            double g_new = currentPtr -> gScore + edgeCostSets.at(i); 

            if (g_new < neighborPtr -> gScore){
              neighborPtr -> gScore = g_new; 
              neighborPtr -> cameFrom = currentPtr; 
              if (neighborPtr -> id == 0){
                // //TODO
                // std::cout<<__FUNCTION__<<"3"<<std::endl; 
                // This is a new node. Add to open set and flip its status to 1
                double total_cost = neighborPtr -> gScore + getHeu(neighborPtr, endPtr); 
                openSet.insert(std::make_pair(total_cost, neighborPtr)); 
                neighborPtr -> id = 1; 
              }
              else{
                  // Update the neighborPtr's "cameFrom" direction (normalized vector)
                  //THIS PART IS ABOUT JPS, you can ignore it when you do your Astar work
                  for(int i = 0; i < 3; i++){
                    // index is x,y,z in the node map
                      neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                      if( neighborPtr->dir(i) != 0)
                          neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                  }
                }
              }
            }
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in JPS path finding is %f", (time_2 - time_1).toSec() );
}
