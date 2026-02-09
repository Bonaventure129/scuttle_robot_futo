#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, LookupException
import math
from tf_transformations import euler_from_quaternion

class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Parameters ---
        self.declare_parameter("path_topic", "/a_star/path") # FIX: Match your A* topic
        self.declare_parameter("lookahead_distance", 0.4) 
        self.declare_parameter("max_linear_velocity", 0.4)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("goal_tolerance", 0.15)

        self.path_topic = self.get_parameter("path_topic").value
        self.lookahead_dist = self.get_parameter("lookahead_distance").value
        self.max_v = self.get_parameter("max_linear_velocity").value
        self.max_w = self.get_parameter("max_angular_velocity").value
        self.tolerance = self.get_parameter("goal_tolerance").value

        # --- Subscribers & Publishers ---
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Visualization Marker (The "Carrot")
        self.marker_pub = self.create_publisher(Marker, "/pure_pursuit/marker", 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.global_path = None
        
        self.get_logger().info(f"Pure Pursuit listening on: {self.path_topic}")

    def path_callback(self, path: Path):
        self.global_path = path
        self.get_logger().info("Received new path!")

    def get_robot_pose(self):
        try:
            # Get Robot Pose in MAP frame
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Get Yaw
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            return x, y, yaw
        except LookupException:
            return None

    def get_target_point(self, rx, ry):
        """
        Finds the target point on the path.
        Logic: Find closest point -> Look forward by 'lookahead_dist' -> Pick target.
        """
        if not self.global_path: return None

        # 1. Find Closest Point Index
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(self.global_path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.hypot(px - rx, py - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # 2. Search forward from closest point for the lookahead point
        target_pose = self.global_path.poses[-1] # Default to end
        
        for i in range(closest_idx, len(self.global_path.poses)):
            pose = self.global_path.poses[i]
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist_from_robot = math.hypot(px - rx, py - ry)
            
            if dist_from_robot > self.lookahead_dist:
                target_pose = pose
                break
        
        return target_pose

    def control_loop(self):
        if not self.global_path: return

        # 1. Get Robot Pose
        pose = self.get_robot_pose()
        if pose is None: return
        rx, ry, ryaw = pose

        # 2. Check Distance to Goal
        last_pose = self.global_path.poses[-1]
        dist_to_goal = math.hypot(last_pose.pose.position.x - rx, last_pose.pose.position.y - ry)
        
        if dist_to_goal < self.tolerance:
            self.stop_robot()
            self.global_path = None
            self.get_logger().info("Goal Reached!")
            return

        # 3. Get Target (Carrot)
        target_msg = self.get_target_point(rx, ry)
        tx = target_msg.pose.position.x
        ty = target_msg.pose.position.y

        # Publish Marker for Debugging
        self.publish_marker(tx, ty)

        # 4. Calculate Errors in Robot Frame
        # Transform goal point to robot frame coordinates
        dx_map = tx - rx
        dy_map = ty - ry

        # Rotate into robot frame
        # x_robot = dx * cos(-yaw) - dy * sin(-yaw)
        # y_robot = dx * sin(-yaw) + dy * cos(-yaw)
        x_robot = dx_map * math.cos(-ryaw) - dy_map * math.sin(-ryaw)
        y_robot = dx_map * math.sin(-ryaw) + dy_map * math.cos(-ryaw)

        # 5. Pure Pursuit Math
        # Curvature = 2 * y / L^2
        lookahead_sq = x_robot**2 + y_robot**2
        curvature = (2.0 * y_robot) / lookahead_sq

        # 6. Command
        # Scale speed by curvature (slow down on turns)
        linear_vel = self.max_v / (1.0 + abs(curvature))
        linear_vel = max(0.1, min(linear_vel, self.max_v))
        
        angular_vel = linear_vel * curvature
        angular_vel = max(-self.max_w, min(angular_vel, self.max_w))

        cmd = Twist()
        cmd.linear.x = float(linear_vel)
        cmd.angular.z = float(angular_vel)
        self.cmd_pub.publish(cmd)

    def publish_marker(self, x, y):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = 0.2
        m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0
        self.marker_pub.publish(m)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()