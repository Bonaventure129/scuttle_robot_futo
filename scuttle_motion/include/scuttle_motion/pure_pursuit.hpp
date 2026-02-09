#ifndef SCUTTLE_MOTION__PURE_PURSUIT_HPP_
#define SCUTTLE_MOTION__PURE_PURSUIT_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace scuttle_motion
{

class PurePursuit : public nav2_core::Controller
{
public:
  PurePursuit() = default;
  ~PurePursuit() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
      const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & path);

  geometry_msgs::msg::Point transformPointToRobotFrame(
      const geometry_msgs::msg::Point & point, const geometry_msgs::msg::PoseStamped & robot_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;

  // Parameters
  double lookahead_dist_;
  double max_v_;
  double max_w_;
  double desired_linear_vel_;
};

} // namespace scuttle_motion

#endif