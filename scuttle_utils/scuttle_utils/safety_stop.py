#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from enum import Enum
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        
        # --- Parameters ---
        self.declare_parameter('danger_x_min', -0.097) 
        self.declare_parameter('danger_x_max',  0.3078) 
        self.declare_parameter('danger_y_width', 0.486) 

        self.declare_parameter('warning_x_min', -0.197)
        self.declare_parameter('warning_x_max',  0.4578)
        self.declare_parameter('warning_y_width', 0.586)

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('safety_stop_topic', 'safety_stop')
        self.declare_parameter('robot_frame', 'base_link')

        # Load Params
        self.danger_x_min = self.get_parameter('danger_x_min').value
        self.danger_x_max = self.get_parameter('danger_x_max').value
        self.danger_y_limit = self.get_parameter('danger_y_width').value / 2.0

        self.warning_x_min = self.get_parameter('warning_x_min').value
        self.warning_x_max = self.get_parameter('warning_x_max').value
        self.warning_y_limit = self.get_parameter('warning_y_width').value / 2.0

        self.scan_topic = self.get_parameter('scan_topic').value
        self.stop_topic = self.get_parameter('safety_stop_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.state = State.FREE
        self.prev_state = State.FREE

        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for Visualization
        marker_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Communication
        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, 'zones', marker_qos)

        self.get_logger().info("Safety Stop Node Started (Rectangle Mode)")

    def get_transform(self, source_frame, target_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = [t.transform.rotation.x, t.transform.rotation.y, 
                 t.transform.rotation.z, t.transform.rotation.w]
            _, _, yaw = euler_from_quaternion(q)
            return tx, ty, yaw
        except TransformException:
            return None

    def check_in_rect(self, x, y, x_min, x_max, y_limit):
        return (x >= x_min and x <= x_max) and (abs(y) <= y_limit)

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE
        
        transform = self.get_transform(msg.header.frame_id, self.robot_frame)
        if transform is None:
            return

        trans_x, trans_y, trans_yaw = transform
        cos_yaw = math.cos(trans_yaw)
        sin_yaw = math.sin(trans_yaw)

        angle = msg.angle_min
        
        for range_val in msg.ranges:
            if not math.isinf(range_val) and not math.isnan(range_val):
                lx = range_val * math.cos(angle)
                ly = range_val * math.sin(angle)

                bx = trans_x + (lx * cos_yaw - ly * sin_yaw)
                by = trans_y + (lx * sin_yaw + ly * cos_yaw)

                if self.check_in_rect(bx, by, self.danger_x_min, self.danger_x_max, self.danger_y_limit):
                    self.state = State.DANGER
                    break 
                elif self.check_in_rect(bx, by, self.warning_x_min, self.warning_x_max, self.warning_y_limit):
                    if self.state != State.DANGER:
                        self.state = State.WARNING
            
            angle += msg.angle_increment

        self.publish_state()
        self.publish_markers()

    def publish_state(self):
        msg = Bool()
        msg.data = (self.state == State.DANGER)
        self.safety_stop_pub.publish(msg)
        
        if self.state != self.prev_state:
            if self.state == State.DANGER:
                self.get_logger().error("DANGER! Obstacle in stop zone.")
            elif self.state == State.WARNING:
                self.get_logger().warn("Warning: Obstacle nearby.")
            else:
                self.get_logger().info("Zone Clear.")
            self.prev_state = self.state

    def publish_markers(self):
        zones = MarkerArray()
        
        def create_rect_marker(id, x_min, x_max, y_limit, r, g, b, a):
            m = Marker()
            m.header.frame_id = self.robot_frame
            
            # --- THE FIX IS HERE ---
            # Use 0 timestamp to use the latest available transform
            m.header.stamp.sec = 0
            m.header.stamp.nanosec = 0
            
            m.id = id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            
            length = x_max - x_min
            width = y_limit * 2.0
            m.scale.x = length
            m.scale.y = width
            m.scale.z = 0.01 
            
            m.pose.position.x = x_min + (length / 2.0)
            m.pose.position.y = 0.0
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a
            m.lifetime.sec = 0 # Markers persist until deleted or replaced
            return m

        w_alpha = 1.0 if self.state == State.WARNING else 0.2
        d_alpha = 1.0 if self.state == State.DANGER else 0.2

        zones.markers.append(create_rect_marker(
            0, self.warning_x_min, self.warning_x_max, self.warning_y_limit, 
            1.0, 1.0, 0.0, w_alpha))
            
        zones.markers.append(create_rect_marker(
            1, self.danger_x_min, self.danger_x_max, self.danger_y_limit, 
            1.0, 0.0, 0.0, d_alpha))
        
        self.zones_pub.publish(zones)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()