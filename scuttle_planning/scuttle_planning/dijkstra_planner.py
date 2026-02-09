#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from queue import PriorityQueue
import math

class GraphNode:
    def __init__(self, x, y, cost=0, prev=None):
        self.x = int(x)
        self.y = int(y)
        self.cost = cost
        self.prev = prev
    
    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other):
        return GraphNode(self.x + other[0], self.y + other[1])
    
    def __repr__(self):
        return f"Node({self.x}, {self.y})"

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for Map (Transient Local is required for maps/costmaps)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- UPDATE: Costmap Topic ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            "/costmap",  # <--- Updated as requested
            self.map_callback, 
            map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/dijkstra/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/dijkstra/visited_map", 1)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

        self.get_logger().info("Dijkstra Planner Initialized. Waiting for /costmap...")

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg
        self.visited_map_.header = map_msg.header
        self.visited_map_.info = map_msg.info
        self.visited_map_.data = [-1] * (map_msg.info.height * map_msg.info.width)
        self.get_logger().info(f"Costmap received: {map_msg.info.width}x{map_msg.info.height}")

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().warn("Cannot plan: No costmap received yet.")
            return

        # Reset visited map visualization
        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width)

        # Get current Robot Position from TF (base_link -> map/costmap frame)
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_link", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF Error: {e}")
            return

        start_pose = Pose()
        start_pose.position.x = t.transform.translation.x
        start_pose.position.y = t.transform.translation.y
        
        self.get_logger().info(f"Planning from ({start_pose.position.x:.2f}, {start_pose.position.y:.2f}) to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

        path = self.plan(start_pose, pose.pose)
        
        if path and len(path.poses) > 0:
            self.get_logger().info(f"Path found with {len(path.poses)} steps!")
            self.path_pub.publish(path)
            self.map_pub.publish(self.visited_map_)
        else:
            self.get_logger().warn("Failed to find a path.")

    def plan(self, start: Pose, goal: Pose):
        # 4-Connectivity neighbors (Up, Down, Left, Right)
        explore_directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        pending_nodes = PriorityQueue()
        visited_nodes = set()

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        # Verify start/goal are inside map
        if not self.pose_on_map(start_node) or not self.pose_on_map(goal_node):
            self.get_logger().warn("Start or Goal is outside the map bounds!")
            return None

        pending_nodes.put(start_node)
        visited_nodes.add(start_node) 

        final_node = None
        
        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            # Visualization: Mark as visited (50 = Gray)
            idx = self.pose_to_cell(active_node)
            self.visited_map_.data[idx] = 50 

            if active_node == goal_node:
                final_node = active_node
                break
            
            for dir_x, dir_y in explore_directions:
                new_node = active_node + (dir_x, dir_y)
                
                # Check bounds
                if not self.pose_on_map(new_node):
                    continue

                # Check if already visited
                if new_node in visited_nodes:
                    continue
                
                # --- OBSTACLE CHECK ---
                cell_idx = self.pose_to_cell(new_node)
                cell_val = self.map_.data[cell_idx]
                
                # Treat -1 (Unknown) and >= 50 (High Cost/Obstacle) as impassable
                if cell_val == -1 or cell_val >= 50:
                    continue 

                new_node.cost = active_node.cost + 1
                new_node.prev = active_node
                
                pending_nodes.put(new_node)
                visited_nodes.add(new_node)

        if final_node is None:
            return None

        # Reconstruct Path
        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        
        curr = final_node
        while curr is not None:
            p = self.grid_to_world(curr)
            
            ps = PoseStamped()
            ps.header = path.header
            ps.pose = p
            path.poses.append(ps)
            
            curr = curr.prev

        path.poses.reverse()
        return path

    def pose_on_map(self, node: GraphNode):
        return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()