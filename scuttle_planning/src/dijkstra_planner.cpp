#include "scuttle_planning/dijkstra_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <limits>

// Export this class as a plugin so Nav2 can load it
PLUGINLIB_EXPORT_CLASS(scuttle_planning::DijkstraPlanner, nav2_core::GlobalPlanner)

namespace scuttle_planning
{

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  // Get raw pointer to the underlying grid for fast access
  costmap_ = costmap_ros_->getCostmap();
}

void DijkstraPlanner::cleanup() {}
void DijkstraPlanner::activate() {}
void DijkstraPlanner::deactivate() {}

nav_msgs::msg::Path DijkstraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker) // ERROR FIX: Removed '&'
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = rclcpp::Time();
  global_path.header.frame_id = costmap_ros_->getGlobalFrameID();

  // 1. Check for cancellation immediately
  if (cancel_checker()) {
    return global_path;
  }

  // 2. Coordinate Conversion: World (meters) -> Grid (indices)
  unsigned int mx_start, my_start, mx_goal, my_goal;
  
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
      !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    // Start or Goal is outside the map boundaries
    return global_path;
  }

  // Flatten 2D coordinates to 1D index
  int start_idx = costmap_->getIndex(mx_start, my_start);
  int goal_idx = costmap_->getIndex(mx_goal, my_goal);
  int size_x = costmap_->getSizeInCellsX();
  int size_y = costmap_->getSizeInCellsY();
  int total_cells = size_x * size_y;

  // 3. Initialize Dijkstra Structures
  // 'dist' stores the shortest known distance to every cell
  std::vector<float> dist(total_cells, std::numeric_limits<float>::infinity());
  // 'parent' stores the index of the previous cell (for backtracking)
  std::vector<int> parent(total_cells, -1);
  // Priority Queue for Open List
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  // Initialize Start Node
  dist[start_idx] = 0.0;
  pq.push({start_idx, 0.0});

  // 4. Define Connectivity (4-way: Up, Down, Left, Right)
  const int dx[4] = {0, 0, 1, -1};
  const int dy[4] = {1, -1, 0, 0};

  // 5. Main Search Loop
  int iterations = 0;
  while (!pq.empty()) {
    // Periodic cancellation check (every 1000 iterations)
    if (iterations++ % 1000 == 0 && cancel_checker()) {
      return global_path;
    }

    int u = pq.top().index;
    float d = pq.top().cost;
    pq.pop();

    if (u == goal_idx) break; // Found Goal

    // Optimization: If a shorter path to 'u' was already found, skip this one
    if (d > dist[u]) continue;

    unsigned int ux, uy;
    costmap_->indexToCells(u, ux, uy);

    // Expand Neighbors
    for (int i = 0; i < 4; i++) {
      int nx = ux + dx[i];
      int ny = uy + dy[i];

      // Bounds Check
      if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y) continue;

      // Obstacle Check
      unsigned char cost = costmap_->getCost(nx, ny);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) continue;

      // Calculate new cost: Base move (1.0) + Obstacle penalty
      float weight = 1.0 + (cost / 255.0); 
      
      // Relaxation
      int v = costmap_->getIndex(nx, ny);
      if (dist[u] + weight < dist[v]) {
        dist[v] = dist[u] + weight;
        parent[v] = u;
        pq.push({v, dist[v]});
      }
    }
  }

  // 6. Reconstruct Path (Backtracking)
  if (dist[goal_idx] == std::numeric_limits<float>::infinity()) {
    return global_path; // No path found
  }

  int curr = goal_idx;
  while (curr != -1) {
    unsigned int cx, cy;
    costmap_->indexToCells(curr, cx, cy);
    double wx, wy;
    costmap_->mapToWorld(cx, cy, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header = global_path.header;
    global_path.poses.push_back(pose);

    curr = parent[curr];
  }

  // Reverse to get Start -> Goal
  std::reverse(global_path.poses.begin(), global_path.poses.end());
  return global_path;
}

} // namespace scuttle_planning