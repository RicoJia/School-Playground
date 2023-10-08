/**
* @brief: A ROS-Independent Implementation of RRT 
*/

#ifndef __RRT_2D_HPP__
#define __RRT_2D_HPP__
#include "rico_map_data_structures.hpp"
#include <functional>
#include <algorithm>
#include <climits>
#include <memory>
#include <vector>
#include <random>

#define UNINITIALIZED INT_MIN

namespace rico_rrt_plugin{

class Rrt2D
{
    public:
      using FreeSpacePointCheck = std::function<bool(const Eigen::Vector2d& )>; 
      using FreeSpacePathCheck = std::function<bool(const Eigen::Vector2d&, const Eigen::Vector2d&)>; 
      Rrt2D (const FreeSpacePointCheck& is_free_space_cell, const FreeSpacePathCheck& is_free_space_path) : is_free_space_cell_(is_free_space_cell), is_free_space_path_(is_free_space_path)
  {
      }

      ~Rrt2D () = default; 

      // lower_bound_corner_: (x_lower, y_lower)
      void set_bounds(const Eigen::Vector2d& lower_bound_corner, const Eigen::Vector2d& upper_bound_corner){
          lower_bound_corner_ = lower_bound_corner; 
          upper_bound_corner_ = upper_bound_corner; 
      }
    
      std::vector<Eigen::Vector2d> solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal); 

    private:
        FreeSpacePointCheck is_free_space_cell_ ;
        FreeSpacePathCheck is_free_space_path_ ;
        Eigen::Vector2d upper_bound_corner_ = {UNINITIALIZED, UNINITIALIZED}; 
        Eigen::Vector2d lower_bound_corner_ = {UNINITIALIZED, UNINITIALIZED}; 

        double step_size_ = 0.15; 
        double dist_tolerance_ = 0.25; 

        Util::NodePtr get_random_map_point(){
            static std::random_device dev; 
            static std::mt19937 rng(dev()); 
            static std::uniform_real_distribution<double> x_dist(lower_bound_corner_(0), upper_bound_corner_(0)); 
            static std::uniform_real_distribution<double> y_dist (lower_bound_corner_(1), upper_bound_corner_(1)); 
            return Util::NodePtr(new Util::Node{Vector2d(x_dist(rng), y_dist(rng)), nullptr}); 
        }

        bool close_to_goal(const Eigen::VectorXd& point_state, const Eigen::VectorXd& goal){
           return (goal-point_state).norm() < dist_tolerance_; 
        }
};

std::vector<Eigen::Vector2d> Rrt2D::solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal){
      Util::KdTree kd_tree; 
      std::vector<Eigen::Vector2d> path; 
      Util::Node* start_ptr = new Util::Node{start, nullptr};
      Util::Node* goal_ptr = new Util::Node{goal, nullptr}; 
      kd_tree.insert(Util::NodePtr(start_ptr));     

      while (true){
          // generate a random point 
          Util::NodePtr random_node; 
          do{
              random_node = get_random_map_point(); 
            }
          while(!is_free_space_cell_(random_node->state_));

          // find the nearest point in the tree, and insert it 
          Util::Node& nn = kd_tree.find_nearest_neighbor(*random_node); 
          auto direction_to_new = (random_node->state_ - nn.state_).normalized();
          auto new_node_state = nn.state_ + step_size_ * direction_to_new; 
          // if random_node is very close, then new_node_state might be farther and may not be free space cell.
          if (!is_free_space_cell_(new_node_state)) continue; 
          Util::Node* new_node = new Util::Node {new_node_state, &nn}; 
          if (!is_free_space_path_(new_node->state_, nn.state_)) continue; 
          kd_tree.insert(Util::NodePtr(new_node)); 

          // If close enough, try to connect to the goal, if successful, exit. 
          if (close_to_goal(new_node->state_, goal)) {
             if (is_free_space_path_(new_node->state_, goal)){
                //back_track - we keep going back until we hit the start ptr
                goal_ptr->parent_ = new_node; 
                kd_tree.insert(std::unique_ptr<Util::Node>(goal_ptr)); 
                for(auto ptr = goal_ptr; ptr != start_ptr; ptr = ptr -> parent_){
                    path.emplace_back(ptr->state_);
                }
                // NEED TO REVERSE PATH BEFORE BACKTRACKING!
                std::reverse(path.begin(), path.end()); 
                std::cout<<"waypoint number:"<<path.size()<<std::endl; 
                break; 
           } 
        }
      }   //end of while

      return path; 
}
  
}



#endif /* end of include guard: __RRT_2D_HPP__ */
