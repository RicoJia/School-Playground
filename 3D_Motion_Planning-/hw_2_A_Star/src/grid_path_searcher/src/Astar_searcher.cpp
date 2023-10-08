#include "Astar_searcher.h"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    // lower bounds
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    // upper bounds
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    // data is a 1-d array
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    terminatePtr = nullptr;
    
    // GridNodeMap is a 3D data structure, with idx (i j k), and 3D position
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

// Loop though the map and find nodes that have been visited
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

// a coord is (index + 0.5) * res + bounds
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    */
    auto index = currentPtr -> index; 
    auto x = index(0); 
    auto y = index(1); 
    auto z = index(2); 
    for(int i = std::max(0, x - 1); i < std::min(GLX_SIZE, x + 2); ++i){
      for(int j = std::max(0, y - 1); j < std::min(GLY_SIZE, y + 2); ++j){      
          // //TODO-1 2D case
          // int k = z; 
        for(int k = std::max(0, z - 1); k < std::min(GLZ_SIZE, z + 2); ++k){      
          if(x != i || y != j || z != k){
            neighborPtrSets.emplace_back(GridNodeMap[i][j][k]); 
            // RICO: here we make Euclidean Distance the real cost, g(x) larger than any heuristics
            if (isOccupied(i, j, k)){
                edgeCostSets.emplace_back(inf); 
            }
            else{
                edgeCostSets.emplace_back(std::abs(x-i) + std::abs(y-j) + std::abs(z-k));

            }
          }
        }
      }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    */
    double h_x; 
    auto diff = node1->coord - node2->coord;
    auto diff_abs = (diff.array().abs());
    
    // Manhattan
    // h_x = diff_abs.sum();

    // H(x)1: Euclidean
    h_x = diff.norm();
  
    // // H(x)2: StraightLine (for 2D)
    // h_x = diff.norm() + 0.01 * abs(diff_abs(0) * diff_abs(1) - diff_abs(1)*diff_abs(0));
    //
    // Best Heuristics: h_x = |dx| + |dy| + |dx| + min(|dx|, |dy|, |dz|) * (2-sqrt(2)) (diagonal)
    // h_x = diff_abs.sum() + diff_abs.minCoeff()* (sqrt(2) - 2);
    //
    // // H(x)3: Dijkstra
    // h_x = 0; 
     

    return h_x;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //TODO-1 2D case
    // end_pt(2) = 0; 
    // start_pt(2) = 0; 

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
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    // STEP 2 :  some else preparatory works which should be done before while loop. TODO
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() ){
        // step 3: Remove the node with lowest cost function from open set to closed set. 
        currentPtr = openSet.begin() -> second; 
        currentPtr -> id = -1;
        openSet.erase(openSet.begin());

        // if the current node is the goal, return and back track
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //STEP 4: finish AstarPathFinder::AstarGetSucc yourself
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  

        //STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            * If has been expanded, skip, because its accumulative cost must be lower since it came from an older parent
            * Update the node's cost g = min(g_old, g_new)
            * If g_old > g_new, parent should become the current node
            * If a node has been 
            *   1. New: add to open list
            *   2. In openset: do nothing
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            */
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
              // Do nothing for node already in open set.
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    //STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    while(terminatePtr != nullptr){
      gridPath.emplace_back(terminatePtr); 
      terminatePtr = terminatePtr -> cameFrom; 
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}
