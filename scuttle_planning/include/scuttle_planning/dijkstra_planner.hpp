#ifndef SCUTTLE_PLANNING__DIJKSTRA_PLANNER_HPP_
#define SCUTTLE_PLANNING__DIJKSTRA_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include <functional> // Required for std::function

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace scuttle_planning
{

/**
 * @class DijkstraPlanner
 * @brief A global planner plugin for Nav2 implementing Dijkstra's Algorithm.
 * * This class inherits from nav2_core::GlobalPlanner, allowing it to be loaded
 * dynamically by the Nav2 Planner Server. It uses a uniform cost search (Dijkstra)
 * to find the optimal path on a costmap.
 */
class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() = default;

  /**
   * @brief Configures the planner plugin.
   * @param parent Weak pointer to the lifecycle node managing this plugin.
   * @param name Name of the plugin.
   * @param tf TF buffer for transforms.
   * @param costmap_ros Wrapper for the 2D Costmap.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleans up resources.
   */
  void cleanup() override;

  /**
   * @brief Activates the plugin (transition to active state).
   */
  void activate() override;

  /**
   * @brief Deactivates the plugin.
   */
  void deactivate() override;

  /**
   * @brief Creates a path from start to goal.
   * @param start The starting pose of the robot.
   * @param goal The goal pose.
   * @param cancel_checker Function to check if the planning request has been cancelled.
   * @return nav_msgs::msg::Path The calculated path.
   */
  // ERROR FIX: Removed '&' from cancel_checker to match base class signature
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  // ROS Node handles and TF
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;

  // Costmap Pointers
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  /**
   * @brief Struct representing a node in the search graph.
   * Used in the priority queue to sort nodes by cost.
   */
  struct Node {
    int index;  // Flattened grid index (y * width + x)
    float cost; // Cumulative cost from start
    
    // Operator overload for Min-Priority Queue behavior (lowest cost pops first)
    bool operator>(const Node& other) const { return cost > other.cost; }
  };
};

}  // namespace scuttle_planning

#endif  // SCUTTLE_PLANNING__DIJKSTRA_PLANNER_HPP_