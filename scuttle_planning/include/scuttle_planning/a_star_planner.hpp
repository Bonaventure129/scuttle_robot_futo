#ifndef SCUTTLE_PLANNING__A_STAR_PLANNER_HPP_
#define SCUTTLE_PLANNING__A_STAR_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <cmath>
#include <functional> 

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace scuttle_planning
{

/**
 * @class AStarPlanner
 * @brief A global planner plugin implementing A* with Manhattan Heuristic.
 * Optimized for 4-connected grids.
 */
class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // ERROR FIX: Removed '&' from cancel_checker
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  struct Node {
    int index;
    float cost; // Represents F-Cost (G + H)
    bool operator>(const Node& other) const { return cost > other.cost; }
  };
};

} 
#endif
