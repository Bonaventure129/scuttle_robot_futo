#include "scuttle_motion/pd_motion_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "angles/angles.h"

// Use namespaces to simplify code
using namespace std;
using nav2_util::declare_parameter_if_not_declared;

// Export this class as a Nav2 Controller Plugin
PLUGINLIB_EXPORT_CLASS(scuttle_motion::PDMotionPlanner, nav2_core::Controller)

namespace scuttle_motion
{

/**
 * @brief Configuration step called when the Controller Server starts.
 * Loads parameters like Kp, Kd, and limits.
 */
void PDMotionPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  string name, shared_ptr<tf2_ros::Buffer> tf,
  shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw runtime_error("Unable to lock node!");
  }

  // Declare parameters with default values
  declare_parameter_if_not_declared(node, name + ".kp", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name + ".kd", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name + ".max_linear_velocity", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(node, name + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".step_size", rclcpp::ParameterValue(0.2));

  // Get parameter values
  node->get_parameter(name + ".kp", kp_);
  node->get_parameter(name + ".kd", kd_);
  node->get_parameter(name + ".max_linear_velocity", max_v_);
  node->get_parameter(name + ".max_angular_velocity", max_w_);
  node->get_parameter(name + ".step_size", step_size_);

  desired_linear_vel_ = max_v_;
  prev_error_ = 0.0;
}

void PDMotionPlanner::cleanup() {}
void PDMotionPlanner::activate() {}
void PDMotionPlanner::deactivate() {}

/**
 * @brief Updates the global path plan that we need to follow.
 */
void PDMotionPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

/**
 * @brief Sets speed limit dynamically (e.g. if we enter a slow zone).
 */
void PDMotionPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    desired_linear_vel_ = max_v_ * speed_limit / 100.0;
  } else {
    desired_linear_vel_ = speed_limit;
  }
}

/**
 * @brief The main control loop. Calculates velocity commands (Twist).
 * * Logic:
 * 1. Find a point on the path 'step_size' meters ahead of the robot.
 * 2. Calculate the angle required to face that point.
 * 3. Use PD controller to turn towards that angle.
 * 4. If error is large, rotate in place. If small, drive forward.
 */
geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = pose.header.stamp;

  if (global_plan_.poses.empty()) {
    return cmd_vel;
  }

  // 1. Get Lookahead Point (Carrot)
  geometry_msgs::msg::PoseStamped target_pose = getLookAheadPoint(pose, global_plan_);

  // 2. Compute Angular Error
  // Current Robot Heading
  double current_yaw = tf2::getYaw(pose.pose.orientation);
  
  // Vector to Target
  double dx = target_pose.pose.position.x - pose.pose.position.x;
  double dy = target_pose.pose.position.y - pose.pose.position.y;
  double angle_to_target = atan2(dy, dx);

  // Shortest angular distance handles the -PI to PI wrap-around
  double error = angles::shortest_angular_distance(current_yaw, angle_to_target);

  // 3. Control Logic
  // Large Error (> 45 degrees or 0.785 rad): Turn in place
  if (abs(error) > 0.785) { 
    cmd_vel.twist.linear.x = 0.0;
    // Rotate at max speed in the direction of the error
    cmd_vel.twist.angular.z = (error > 0 ? 1.0 : -1.0) * max_w_;
  } 
  else {
    // Small Error: Drive and Turn simultaneously
    // PD Controller for angular velocity
    double p_term = kp_ * error;
    double d_term = kd_ * (error - prev_error_);
    double angular_vel = p_term + d_term;

    // Clamp Angular Velocity to max limits
    angular_vel = max(min(angular_vel, max_w_), -max_w_);

    cmd_vel.twist.linear.x = desired_linear_vel_;
    cmd_vel.twist.angular.z = angular_vel;
  }

  prev_error_ = error;
  return cmd_vel;
}

/**
 * @brief Helper to find a point on the path ahead of the robot.
 */
geometry_msgs::msg::PoseStamped PDMotionPlanner::getLookAheadPoint(
    const geometry_msgs::msg::PoseStamped & robot_pose, const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) return robot_pose;

  // Find index of the closest point on path to the robot
  size_t closest_idx = 0;
  double min_dist = numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, path.poses[i]);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  // Iterate forward from closest point until we exceed 'step_size'
  double current_dist = 0.0;
  for (size_t i = closest_idx; i < path.poses.size() - 1; ++i) {
    current_dist += nav2_util::geometry_utils::euclidean_distance(path.poses[i], path.poses[i+1]);
    if (current_dist >= step_size_) {
      return path.poses[i+1];
    }
  }

  // If we reach the end, return the final goal pose
  return path.poses.back();
}

} // namespace scuttle_motion