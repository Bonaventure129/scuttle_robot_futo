#ifndef SCUTTLE_MOTION__PD_MOTION_PLANNER_HPP_
#define SCUTTLE_MOTION__PD_MOTION_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace scuttle_motion
{

class PDMotionPlanner : public nav2_core::Controller
{
public:
  PDMotionPlanner() = default;
  ~PDMotionPlanner() override = default;

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
  // Helpers
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
      const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & path);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  
  nav_msgs::msg::Path global_plan_;
  
  // Parameters
  double kp_;
  double kd_;
  double max_v_;
  double max_w_;
  double step_size_;
  double desired_linear_vel_; // speed limit applied

  double prev_error_;
};

} // namespace scuttle_motion

#endif