#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")
        
        # --- Relay 1: Unstamped -> Stamped ---
        # Useful if a teleop node publishes simple Twist, but the controller needs a Header/Timestamp
        self.controller_sub = self.create_subscription(
            Twist,
            "/scuttle_controller/cmd_vel_unstamped",
            self.controller_twist_callback,
            10
        )
        self.controller_pub = self.create_publisher(
            TwistStamped,
            "/scuttle_controller/cmd_vel",
            10
        )

        # --- Relay 2: Stamped -> Unstamped ---
        # Useful if a node publishes TwistStamped, but a tool requires simple Twist
        self.joy_sub = self.create_subscription(
            TwistStamped,
            "/input_joy/cmd_vel_stamped",
            self.joy_twist_callback,
            10
        )
        self.joy_pub = self.create_publisher(
            Twist,
            "/input_joy/cmd_vel",
            10
        )

    def controller_twist_callback(self, msg):
        """
        Receives a Twist message, adds a Header with the current time,
        and republishes it as TwistStamped.
        """
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link" # Good practice to set a frame_id
        twist_stamped.twist = msg
        self.controller_pub.publish(twist_stamped)

    def joy_twist_callback(self, msg):
        """
        Receives a TwistStamped message, strips the header,
        and republishes the inner Twist message.
        """
        twist = Twist()
        twist = msg.twist
        self.joy_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()