#include "scuttle_motion/pure_pursuit.hpp"
#include "nav2_util/node_utils.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"

// Use namespaces to simplify
using namespace std;
using nav2_util::declare_parameter_if_not_declared;

// Export as Nav2 Controller Plugin
PLUGINLIB_EXPORT_CLASS(scuttle_motion::PurePursuit, nav2_core::Controller)

namespace scuttle_motion
{

void PurePursuit::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  string name, shared_ptr<tf2_ros::Buffer> tf,
  shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) throw runtime_error("Unable to lock node!");

  // Load Parameters
  declare_parameter_if_not_declared(node, name + ".look_ahead_distance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name + ".max_angular_velocity", rclcpp::ParameterValue(1.0));

  node->get_parameter(name + ".look_ahead_distance", lookahead_dist_);
  node->get_parameter(name + ".max_linear_velocity", max_v_);
  node->get_parameter(name + ".max_angular_velocity", max_w_);

  desired_linear_vel_ = max_v_;
}

void PurePursuit::cleanup() {}
void PurePursuit::activate() {}
void PurePursuit::deactivate() {}

void PurePursuit::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void PurePursuit::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) desired_linear_vel_ = max_v_ * speed_limit / 100.0;
  else desired_linear_vel_ = speed_limit;
}

/**
 * @brief Calculates the velocity command using Pure Pursuit logic.
 * * Math:
 * 1. Find a lookahead point 'L' meters away on the path.
 * 2. Transform that point into the robot's local frame.
 * 3. Calculate curvature (gamma) required to hit that point in an arc.
 * gamma = (2 * y) / (L^2)
 * where 'y' is the lateral offset of the point in the robot frame.
 * 4. Angular Velocity = Linear Velocity * Curvature.
 */
geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = pose.header.stamp;

  if (global_plan_.poses.empty()) return cmd_vel;

  // 1. Find Lookahead Point (Global Frame)
  geometry_msgs::msg::PoseStamped target = getLookAheadPoint(pose, global_plan_);

  // 2. Transform target point to Robot Frame (Base Link)
  // In Robot Frame: X is forward, Y is left.
  geometry_msgs::msg::Point target_local = transformPointToRobotFrame(target.pose.position, pose);

  // 3. Calculate Curvature (gamma = 2y / L^2)
  double y = target_local.y;
  double L_sq = target_local.x * target_local.x + target_local.y * target_local.y; // Distance squared
  
  double gamma = 0.0;
  if (L_sq > 0.001) {
    gamma = (2.0 * y) / L_sq;
  }

  // 4. Calculate Velocities
  // w = v * gamma
  double w = desired_linear_vel_ * gamma;

  // Clamp angular velocity to safety limits
  w = max(min(w, max_w_), -max_w_);

  cmd_vel.twist.linear.x = desired_linear_vel_;
  cmd_vel.twist.angular.z = w;

  return cmd_vel;
}

/**
 * @brief Finds the goal point on the path at distance 'lookahead_dist_'.
 */
geometry_msgs::msg::PoseStamped PurePursuit::getLookAheadPoint(
    const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & path)
{
  size_t closest_idx = 0;
  double min_dist = numeric_limits<double>::max();

  // Find closest point index
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, path.poses[i]);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Walk forward to find lookahead point
  double current_dist = 0.0;
  for (size_t i = closest_idx; i < path.poses.size() - 1; ++i) {
    current_dist += nav2_util::geometry_utils::euclidean_distance(path.poses[i], path.poses[i+1]);
    if (current_dist >= lookahead_dist_) {
      return path.poses[i+1];
    }
  }
  return path.poses.back();
}

/**
 * @brief Transforms a point from Global Frame to Robot Local Frame.
 * Does standard 2D rotation matrix math manually.
 */
geometry_msgs::msg::Point PurePursuit::transformPointToRobotFrame(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // 1. Translate
  double dx = point.x - robot_pose.pose.position.x;
  double dy = point.y - robot_pose.pose.position.y;

  // 2. Rotate (Inverse of robot yaw)
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);

  geometry_msgs::msg::Point local_point;
  // Rotation Matrix: [ cos  sin]
  //                  [-sin  cos]
  local_point.x = dx * cos_yaw + dy * sin_yaw;
  local_point.y = -dx * sin_yaw + dy * cos_yaw;
  local_point.z = 0.0;

  return local_point;
}

} // namespace scuttle_motion