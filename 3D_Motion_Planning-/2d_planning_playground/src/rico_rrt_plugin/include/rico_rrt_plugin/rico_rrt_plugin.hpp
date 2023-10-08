#ifndef __RICO_RRT_PLUGIN_HPP__
#define __RICO_RRT_PLUGIN_HPP__
//TODO?
// #include <rico_rrt_plugin/RicoRrtPluginConfig.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/inflation_layer.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>
#include <nav_core/base_global_planner.h>
// #include <boost/thread.hpp>
//#include <boost/thread/future.hpp>
#include <functional>
#include <tuple>
#include <random>
#include "rrt_2d.hpp"

#define CIRCUMSCRIBED_COST 128    //from doc
namespace rico_rrt_plugin{
    
    unsigned int generate_random_int(unsigned int lower_bound, unsigned int upper_bound){
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist(lower_bound,upper_bound); // distribution in range [1, 6]
        return dist(rng);
    }

    class RicoRrtPlugin : public nav_core::BaseGlobalPlanner{
    public:
        RicoRrtPlugin();
        ~RicoRrtPlugin();

        /// \brief Initialization Function including the costmap and the name
        /// \param name The name of the planner
        /// \param costmap_ros a pointer to the ROS wrapper of the costmap to use.
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

        /// \brief Given a goal pose in the world, compute a plan
        /// \param start The start pose
        /// \param goal The goal pose
        /// \param plan The plan... filled by the planner
        /// \return True if a valid plan was found, false otherwise
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

    private: 
        std::unique_ptr<Rrt2D> rrt_2d_; 
        bool costmap_initialized_; 
        costmap_2d::Costmap2D* costmap_2d_;         
        ros::NodeHandle private_nh_;

        // util functions
        // checks if a point in world coordinates is in free space
        bool is_free_space_cell(const Eigen::Vector2d& point);
        bool is_free_space_map_cell(unsigned int mx, unsigned int my);
      
        // checks if a path in world coordinates is in free space
        bool is_free_space_path(const Eigen::Vector2d&, const Eigen::Vector2d&);
  };
}


#endif /* end of include guard: __RICO_RRT_PLUGIN_HPP__ */
