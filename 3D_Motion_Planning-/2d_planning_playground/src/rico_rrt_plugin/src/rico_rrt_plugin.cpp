#include <rico_rrt_plugin/rico_rrt_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <tuple>
#include <boost/thread/future.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

using std::cout; using std::endl; 

PLUGINLIB_EXPORT_CLASS(rico_rrt_plugin::RicoRrtPlugin, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace ros;

namespace rico_rrt_plugin {
    RicoRrtPlugin::RicoRrtPlugin(): 
      costmap_initialized_(false),
      costmap_2d_(nullptr), 
      rrt_2d_(nullptr)
    {
        auto is_free_space_cell = std::bind(&RicoRrtPlugin::is_free_space_cell, this, std::placeholders::_1); 
        auto is_free_space_path = std::bind(&RicoRrtPlugin::is_free_space_path, this, std::placeholders::_1, std::placeholders::_2);
        rrt_2d_ = std::make_unique<Rrt2D>(is_free_space_cell, is_free_space_path);
        ROS_INFO(__PRETTY_FUNCTION__); 
    }

    RicoRrtPlugin::~RicoRrtPlugin(){
        ROS_INFO(__PRETTY_FUNCTION__); 
    }

    void RicoRrtPlugin::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        //Fundamentals
        private_nh_ = ros::NodeHandle ("~"+name);
        costmap_2d_ = costmap_ros->getCostmap(); 
        // costmap2D has origin being the lower left corner
        Eigen::Vector2d lower_bound_corner{costmap_2d_ -> getOriginX(), costmap_2d_->getOriginY()};
        Eigen::Vector2d upper_bound_corner = lower_bound_corner + Eigen::Vector2d{costmap_2d_ -> getSizeInMetersX(), costmap_2d_ -> getSizeInMetersY()}; 
        rrt_2d_ -> set_bounds(lower_bound_corner, upper_bound_corner); 
    }

    bool RicoRrtPlugin::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
      ROS_INFO(__PRETTY_FUNCTION__); 

      // plan.push_back(start);
      double goal_x, goal_y, start_x, start_y; 
      start_x = start.pose.position.x; 
      start_y = start.pose.position.y;
      goal_x = goal.pose.position.x; 
      goal_y = goal.pose.position.y;

      if (!is_free_space_cell({goal_x, goal_y}))
        return false; 

      auto path = rrt_2d_ -> solve({start_x, start_y}, {goal_x, goal_y}); 
      // populate the plan msg
      geometry_msgs::PoseStamped msg; 
      msg.header.frame_id = "map"; 
      msg.header.stamp = ros::Time::now();
      msg.pose.position.z = 0; 
      msg.pose.orientation.w = 1.0;
      for (const auto& wp: path){
          msg.pose.position.x = wp(0); 
          msg.pose.position.y = wp(1); 
          plan.emplace_back(msg);
      }
      return true;
    }

    // Utils
    bool RicoRrtPlugin::is_free_space_cell(const Eigen::Vector2d& point){
        double wx  = point(0), wy = point(1); 
        unsigned int mx, my; 
        bool in_bounds = costmap_2d_ -> worldToMap(wx, wy, mx, my); 
        if (in_bounds){
          return is_free_space_map_cell(mx, my); 
        }
        else{
          return false; 
        }
    }

    bool RicoRrtPlugin::is_free_space_map_cell(unsigned int mx, unsigned int my){
        ROS_DEBUG("point value: %i", costmap_2d_ -> getCost(mx, my));
        return (costmap_2d_ -> getCost(mx, my) < CIRCUMSCRIBED_COST);
    }

    bool RicoRrtPlugin::is_free_space_path(const Eigen::Vector2d& start, const Eigen::Vector2d& end){
        short delta_mx = (end(0) > start(0))? 1 : -1; 
        short delta_my = (end(1) > start(1))? 1 : -1; 

        unsigned int start_mx, start_my, end_mx, end_my; 
        costmap_2d_ -> worldToMap(start(0), start(1), start_mx, start_my); 
        costmap_2d_ -> worldToMap(end(0), end(1), end_mx, end_my); 
        for (; start_mx != end_mx; start_mx+=delta_mx)
          for (; start_my != end_my; start_my+=delta_my){
              if (!is_free_space_map_cell(start_mx, start_my))
                return false;
          }
        return true; 
    }
}; 
