#include "scuttle_planning/a_star_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <limits>

PLUGINLIB_EXPORT_CLASS(scuttle_planning::AStarPlanner, nav2_core::GlobalPlanner)

namespace scuttle_planning
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
}

void AStarPlanner::cleanup() {}
void AStarPlanner::activate() {}
void AStarPlanner::deactivate() {}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker) // ERROR FIX: Removed '&'
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = rclcpp::Time();
  global_path.header.frame_id = costmap_ros_->getGlobalFrameID();

  if (cancel_checker()) return global_path;

  // 1. Convert to Grid Coordinates
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    return global_path;
  }

  int start_idx = costmap_->getIndex(mx_start, my_start);
  int goal_idx = costmap_->getIndex(mx_goal, my_goal);
  int size_x = costmap_->getSizeInCellsX();
  int size_y = costmap_->getSizeInCellsY();
  int total_cells = size_x * size_y;

  // 2. A* Initialization
  // g_cost stores the exact cost from start to node
  std::vector<float> g_cost(total_cells, std::numeric_limits<float>::infinity());
  std::vector<int> parent(total_cells, -1);
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  g_cost[start_idx] = 0.0;
  
  // Calculate Heuristic (Manhattan: |dx| + |dy|)
  float h = std::abs((int)mx_start - (int)mx_goal) + std::abs((int)my_start - (int)my_goal);
  
  // Push F-Cost (G + H)
  // Casting 0.0 to float ensures matching types
  pq.push({start_idx, 0.0f + h});

  const int dx[4] = {0, 0, 1, -1};
  const int dy[4] = {1, -1, 0, 0};

  int iterations = 0;
  while (!pq.empty()) {
    if (iterations++ % 1000 == 0 && cancel_checker()) return global_path;

    int u = pq.top().index;
    float f = pq.top().cost; 
    pq.pop();

    if (u == goal_idx) break;

    // Optimization: Re-calculate H to check if this is a stale node
    int ux_int = u % size_x;
    int uy_int = u / size_x;
    float current_h = std::abs(ux_int - (int)mx_goal) + std::abs(uy_int - (int)my_goal);

    // If popped F-cost is worse than current G + H, skip
    if (f > g_cost[u] + current_h) continue;

    unsigned int ux, uy;
    costmap_->indexToCells(u, ux, uy);

    // 3. Expand Neighbors
    for (int i = 0; i < 4; i++) {
      int nx = ux + dx[i];
      int ny = uy + dy[i];

      if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y) continue;

      unsigned char cost = costmap_->getCost(nx, ny);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) continue;

      // Tentative G Cost
      float weight = 1.0 + (cost / 255.0); 
      float tentative_g = g_cost[u] + weight;

      // Relaxation
      if (tentative_g < g_cost[costmap_->getIndex(nx, ny)]) {
        int v = costmap_->getIndex(nx, ny);
        g_cost[v] = tentative_g;
        parent[v] = u;
        
        // H for Neighbor
        float h_neighbor = std::abs(nx - (int)mx_goal) + std::abs(ny - (int)my_goal);
        
        pq.push({v, tentative_g + h_neighbor});
      }
    }
  }

  // 4. Path Reconstruction
  if (g_cost[goal_idx] == std::numeric_limits<float>::infinity()) return global_path;

  int curr = goal_idx;
  while (curr != -1) {
    unsigned int cx, cy;
    costmap_->indexToCells(curr, cx, cy);
    double wx, wy;
    costmap_->mapToWorld(cx, cy, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx; 
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;
    pose.header = global_path.header;
    global_path.poses.push_back(pose);
    curr = parent[curr];
  }
  std::reverse(global_path.poses.begin(), global_path.poses.end());
  return global_path;
}
}