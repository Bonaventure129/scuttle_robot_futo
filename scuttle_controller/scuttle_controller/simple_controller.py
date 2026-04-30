#!/usr/bin/env python3
"""
ROS 2 Differential Drive Inverse Kinematics Controller for SCUTTLE Robot

This node, 'simple_controller', serves as a kinematic interface between a standard
ROS 2 geometry_msgs/Twist command and the low-level wheel velocity commands
required by a differential drive mobile robot like the SCUTTLE.

1. Inverse Kinematics (IK):
   - Subscribes to 'scuttle_controller/cmd_vel' (TwistStamped).
   - Calculates wheel angular velocities [Left, Right] using Inv(M).
   - Publishes to 'simple_velocity_controller/commands' (Float64MultiArray).

2. Forward Kinematics (FK) and Odometry:
   - Subscribes to 'joint_states' (JointState).
   - Calculates the resulting chassis velocity (linear.x, angular.z).
   - Accumulates pose (x, y, theta) using incremental odometry.
   - Publishes velocity feedback to 'scuttle_controller/fk_vel' (TwistStamped).
   - Publishes full pose and velocity data to 'scuttle_controller/odom' (Odometry).
   - **Publishes the 'odom' -> 'base_link' TF transform.**
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# Import TwistStamped and TransformStamped for TF message
from geometry_msgs.msg import TwistStamped, TransformStamped 
from sensor_msgs.msg import JointState
import numpy as np
import math as m
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler 
from tf2_ros import TransformBroadcaster # Import TransformBroadcaster

class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")
        
        # --- Kinematic Parameters ---
        self.declare_parameter("wheel_radius", 0.082 / 2)
        self.declare_parameter("wheel_separation", 0.402)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius_:.3f} m")
        self.get_logger().info(f"Using wheel separation (2L): {self.wheel_separation_:.3f} m")

        # --- State Variables for FK Estimation and Odometry ---
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Odometry state variables (Accumulated Pose)
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # --- ROS 2 Interfaces (Publishers) ---
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.fk_pub_ = self.create_publisher(TwistStamped, "scuttle_controller/fk_vel", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "scuttle_controller/odom", 10) 

        # --- ROS 2 Interfaces (Subscribers) ---
        self.vel_sub_ = self.create_subscription(TwistStamped, "scuttle_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        # --- Forward Kinematics Matrix (M) ---
        self.speed_conversion_ = np.array([[self.wheel_radius_ / 2, self.wheel_radius_ / 2],
                                           [-self.wheel_radius_ / self.wheel_separation_, self.wheel_radius_ / self.wheel_separation_]])

        # --- Odometry Message Initialization ---
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        
        # Initialize covariance matrices (typically sparse for differential drive)
        self.odom_msg_.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
        
        self.odom_msg_.twist.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.03]

        # --- TF Broadcaster Initialization ---
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"

        
        self.get_logger().info(f"The Forward Kinematics matrix (M) is:\n {self.speed_conversion_}")


    def velCallback(self, msg: TwistStamped):
        """
        Calculates required wheel angular velocities (IK) from a chassis Twist command.
        """
        
        # Input: [linear.x, angular.z]
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
                                
        # Inverse Kinematics: [phi_L_dot, phi_R_dot] = Inv(M) * [x_dot, theta_dot]
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        
        # Publish commands: [Left Wheel Speed, Right Wheel Speed]
        wheel_speed_msg.data = [wheel_speed[0, 0], wheel_speed[1, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg: JointState):
        """
        Performs Forward Kinematics (FK) and Odometry Integration, then publishes Odometry and TwistStamped.
        """
        
        # --- Robust Joint Index Lookup ---
        try:
            r_idx = msg.name.index('r_wheel_joint')
            l_idx = msg.name.index('l_wheel_joint')
        except ValueError:
            # Fallback if names are not present
            if len(msg.position) >= 2:
                r_idx = 0
                l_idx = 1
            else:
                # Log warning and exit if essential data is missing
                self.get_logger().warn("Wheel joints not found in joint_states message. Skipping FK/Odometry.")
                return

        # --- Calculate Incremental Change ---
        dp_left = msg.position[l_idx] - self.left_wheel_prev_pos_
        dp_right = msg.position[r_idx] - self.right_wheel_prev_pos_
        
        current_time = self.get_clock().now()
        dt_ros = current_time - self.prev_time_
        dt_sec = dt_ros.nanoseconds / 1e9

        # Update state variables for next iteration
        self.left_wheel_prev_pos_ = msg.position[l_idx]
        self.right_wheel_prev_pos_ = msg.position[r_idx]
        self.prev_time_ = current_time

        if dt_sec <= 0:
            return 
            
        # Angular velocity of the wheels (rad/s)
        fi_left = dp_left / dt_sec
        fi_right = dp_right / dt_sec

        # --- FORWARD KINEMATICS (Velocity Calculation) ---
        linear = ((self.wheel_radius_ * fi_left) + (self.wheel_radius_ * fi_right)) / 2
        angular = (-(self.wheel_radius_ * fi_left) + (self.wheel_radius_ * fi_right)) / self.wheel_separation_

        # --- ODOMETRY INTEGRATION (Pose Update) ---
        
        d_s = ((self.wheel_radius_ * dp_left) + (self.wheel_radius_ * dp_right)) / 2
        d_theta = (-(self.wheel_radius_ * dp_left) + (self.wheel_radius_ * dp_right)) / self.wheel_separation_

        # Update accumulated pose (x, y, theta)
        self.x_ += d_s * m.cos(self.theta_ + d_theta / 2)
        self.y_ += d_s * m.sin(self.theta_ + d_theta / 2)
        self.theta_ += d_theta
        
        # Normalize theta to [-pi, pi]
        self.theta_ = m.atan2(m.sin(self.theta_), m.cos(self.theta_))

        # --- Odometry and TF Publishing ---
        
        # 1. Calculate Quaternion from accumulated theta
        q = quaternion_from_euler(0, 0, self.theta_)

        # 2. Fill in Odometry message (nav_msgs/Odometry)
        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.angular.z = angular
        
        self.odom_pub_.publish(self.odom_msg_) # Publish the Odometry message

        # 3. Fill in TF message (geometry_msgs/TransformStamped)
        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        
        self.br_.sendTransform(self.transform_stamped_) # Broadcast the odom -> base_link TF

        # 4. Publish FK Result (Velocity Feedback - TwistStamped)
        fk_msg = TwistStamped()
        fk_msg.header.stamp = current_time.to_msg()
        fk_msg.header.frame_id = 'base_link'
        fk_msg.twist.linear.x = linear
        fk_msg.twist.angular.z = angular
        
        self.fk_pub_.publish(fk_msg) # Publish the velocity TwistStamped message

        # --- Logging ---
        # The logging lines are commented out to improve performance, but can be uncommented for debugging.
        # self.get_logger().info(f"linear: {linear:.4f} m/s | angular: {angular:.4f} rad/s")
        # self.get_logger().info(f"Pose: x={self.x_:.4f}, y={self.y_:.4f}, theta={self.theta_:.4f} rad")


def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()