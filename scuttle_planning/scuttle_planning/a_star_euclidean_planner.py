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
    def __init__(self, x, y, g_cost=0.0, h_cost=0.0, prev=None):
        self.x = int(x)
        self.y = int(y)
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.prev = prev
    
    @property
    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __repr__(self):
        return f"Node({self.x}, {self.y})"

class AStarEuclideanPlanner(Node):
    def __init__(self):
        super().__init__("a_star_euclidean_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for Costmap (Transient Local)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/a_star_euclidean/path", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/a_star_euclidean/visited_map", 1)

        self.map_ = None
        self.visited_map_ = OccupancyGrid()

        self.get_logger().info("A* Euclidean Planner (Python) Initialized using Costmap.")

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

        self.visited_map_.data = [-1] * (self.visited_map_.info.height * self.visited_map_.info.width)

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
        
        path = self.plan(start_pose, pose.pose)
        
        if path and len(path.poses) > 0:
            self.get_logger().info(f"Path found with {len(path.poses)} steps!")
            self.path_pub.publish(path)
            self.map_pub.publish(self.visited_map_)
        else:
            self.get_logger().warn("Failed to find a path.")

    def plan(self, start: Pose, goal: Pose):
        # 8-Connectivity (Allow Diagonal)
        moves = [
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),  # Cardinal
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414) # Diagonal
        ]

        pending_nodes = PriorityQueue()
        g_scores = {}

        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        if not self.pose_on_map(start_node) or not self.pose_on_map(goal_node):
            self.get_logger().warn("Start or Goal is outside map bounds!")
            return None

        start_node.g_cost = 0
        start_node.h_cost = self.euclidean_distance(start_node, goal_node)
        
        pending_nodes.put(start_node)
        g_scores[(start_node.x, start_node.y)] = 0

        final_node = None

        while not pending_nodes.empty() and rclpy.ok():
            active_node = pending_nodes.get()

            idx = self.pose_to_cell(active_node)
            self.visited_map_.data[idx] = 50 

            if active_node == goal_node:
                final_node = active_node
                break
            
            if active_node.g_cost > g_scores.get((active_node.x, active_node.y), float('inf')):
                continue

            for dx, dy, move_cost in moves:
                new_x = active_node.x + dx
                new_y = active_node.y + dy

                # 1. Bounds Check
                if not self.is_valid_cell(new_x, new_y):
                    continue
                
                # 2. Obstacle Check
                idx = new_y * self.map_.info.width + new_x
                cell_val = self.map_.data[idx]
                
                if cell_val == -1 or cell_val >= 50:
                    continue 

                new_g_cost = active_node.g_cost + move_cost
                
                if new_g_cost < g_scores.get((new_x, new_y), float('inf')):
                    g_scores[(new_x, new_y)] = new_g_cost
                    
                    new_node = GraphNode(new_x, new_y)
                    new_node.g_cost = new_g_cost
                    new_node.h_cost = self.euclidean_distance(new_node, goal_node)
                    new_node.prev = active_node
                    
                    pending_nodes.put(new_node)

        if final_node is None:
            return None

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

    def euclidean_distance(self, node_a, node_b):
        return math.sqrt((node_a.x - node_b.x)**2 + (node_a.y - node_b.y)**2)

    def is_valid_cell(self, x, y):
        return 0 <= x < self.map_.info.width and 0 <= y < self.map_.info.height

    def pose_on_map(self, node: GraphNode):
        return self.is_valid_cell(node.x, node.y)

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int(round((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution))
        grid_y = int(round((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution))
        return GraphNode(grid_x, grid_y)

    def grid_to_world(self, node: GraphNode) -> Pose:
        pose = Pose()
        pose.position.x = (node.x * self.map_.info.resolution) + self.map_.info.origin.position.x
        pose.position.y = (node.y * self.map_.info.resolution) + self.map_.info.origin.position.y
        pose.orientation.w = 1.0
        return pose

    def pose_to_cell(self, node: GraphNode):
        return node.y * self.map_.info.width + node.x

def main(args=None):
    rclpy.init(args=args)
    node = AStarEuclideanPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()