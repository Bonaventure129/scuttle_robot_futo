#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        self.sub = self.create_subscription(Point, '/object_center', self.control_callback, 10)
        
        # CHANGED: Publish to specific tracker topic for twist_mux
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel_tracker', 10)
        
        # --- PID Tuning ---
        self.turn_gain = 0.8
        self.target_size = 0.15

    def control_callback(self, msg):
        cmd = Twist()
        
        if msg.z > 0.5:
            # Steering
            cmd.angular.z = -1.0 * msg.x * self.turn_gain
            
            # Forward/Back
            if msg.y < self.target_size:
                cmd.linear.x = 0.15
            else:
                cmd.linear.x = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()