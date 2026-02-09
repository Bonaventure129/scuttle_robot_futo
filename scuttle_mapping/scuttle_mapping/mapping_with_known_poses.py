#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion
import math
import numpy as np

# Log-Odds Constants
# A higher probability > 0.5 means occupied, < 0.5 means free
PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self, px=0, py=0):
        self.x = px
        self.y = py

def prob2logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def bresenham(start: Pose, end: Pose):
    """
    Standard Bresenham's Line Algorithm to find all grid cells 
    intersected by the laser ray.
    """
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    """
    Returns a list of (Pose, Probability) tuples.
    Cells along the ray are FREE. The endpoint is OCCUPIED.
    """
    occ_values = []
    line = bresenham(p_robot, p_beam)

    # All cells along the ray (excluding the last one) are likely free
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))

    # The final cell where the laser hit is likely occupied
    occ_values.append((line[-1], OCC_PROB))
    return occ_values

class MappingWithKnownPoses(Node):
    def __init__(self):
        super().__init__("mapping_with_known_poses")
        
        # Parameters for Map Size
        self.declare_parameter("width", 50.0)   # Meters
        self.declare_parameter("height", 50.0)  # Meters
        self.declare_parameter("resolution", 0.1) # Meters per pixel

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        # Initialize Occupancy Grid
        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        # Center the map so (0,0) is in the middle
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        
        # Initialize grid data to -1 (Unknown)
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)
        
        # Internal probability representation (using log odds)
        self.probability_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        # TF and Subs/Pubs
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scanCallback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        
        # Timer to publish the map periodically (every 2 seconds to save CPU)
        self.timer = self.create_timer(2.0, self.timerCallback)
        
        self.get_logger().info("Mapping Node Started. Waiting for Scan data...")

    def scanCallback(self, scan: LaserScan):
        try:
            # Look up the transform from map frame (odom) to sensor frame
            # We use Time() to get the latest available transform
            t = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, 
                scan.header.frame_id, 
                rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Could not transform scan to odom frame.")
            return

        # Robot Pose in Grid Coordinates
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        
        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().warn("Robot is outside the map boundaries!")
            return

        # Robot Yaw
        (roll, pitch, yaw) = euler_from_quaternion(
            [t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w])
        
        # Process every Nth ray to save CPU (Python is slow)
        step = 5 
        for i in range(0, len(scan.ranges), step):
            r = scan.ranges[i]
            
            # Ignore invalid ranges
            if math.isinf(r) or math.isnan(r) or r > scan.range_max or r < scan.range_min:
                continue

            # Polar to Cartesian conversion (Global Frame)
            # angle = sensor_angle + robot_yaw
            angle = scan.angle_min + (i * scan.angle_increment) + yaw
            
            px = r * math.cos(angle) + t.transform.translation.x
            py = r * math.sin(angle) + t.transform.translation.y

            # Beam Endpoint in Grid Coordinates
            beam_p = coordinatesToPose(px, py, self.map_.info)
            
            if not poseOnMap(beam_p, self.map_.info):
                continue

            # Raytrace
            poses = inverseSensorModel(robot_p, beam_p)

            # Update Log Odds
            for pose, value in poses:
                if poseOnMap(pose, self.map_.info):
                    cell = poseToCell(pose, self.map_.info)
                    self.probability_map_[cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB)

    def timerCallback(self):
        # Convert Log Odds back to 0-100 probability for visualization
        self.map_.data = [int(logodds2prob(value) * 100) for value in self.probability_map_]
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    node = MappingWithKnownPoses()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()