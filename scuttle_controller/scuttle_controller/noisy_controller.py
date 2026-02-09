#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class NoisyController(Node):

    def __init__(self):
        # 1. Updated Node Name
        super().__init__("noisy_controller")
        
        # --- Kinematic Parameters (from template) ---
        self.declare_parameter("wheel_radius", 0.082 / 2)
        self.declare_parameter("wheel_separation", 0.402)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius_:.3f} m")
        self.get_logger().info(f"Using wheel separation (W): {self.wheel_separation_:.3f} m")

        # --- State Variables for FK Estimation and Odometry ---
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Odometry state variables (Accumulated Pose)
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # --- ROS 2 Interfaces (Publishers) ---
        # 2. Updated Odom Topic
        self.odom_pub_ = self.create_publisher(Odometry, "scuttle_controller/odom_noisy", 10) 

        # --- ROS 2 Interfaces (Subscribers) ---
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        # --- Forward Kinematics Matrix (M) ---
        # Kept for reference, though not explicitly used in the integration loop below
        self.speed_conversion_ = np.array([[self.wheel_radius_ / 2, self.wheel_radius_ / 2],
                                           [-self.wheel_radius_ / self.wheel_separation_, self.wheel_radius_ / self.wheel_separation_]])

        # --- Odometry Message Initialization ---
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link" # Set to base_link as per template
        
        # Initialize covariance matrices (from template)
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
        self.transform_stamped_.child_frame_id = "base_link_noisy" # Set to base_link as per template

        
        self.get_logger().info(f"The Forward Kinematics matrix (M) is:\n {self.speed_conversion_}")


    # Removed velCallback
    # ---

    def jointCallback(self, msg: JointState):
        """
        Performs Forward Kinematics (FK) and Odometry Integration, then publishes NOISY Odometry and TF.
        """
        
        # --- Robust Joint Index Lookup ---
        try:
            r_idx = msg.name.index('r_wheel_joint')
            l_idx = msg.name.index('l_wheel_joint')
        except ValueError:
            # Fallback if names are not present
            if len(msg.position) >= 2:
                # Assumed order: Left is index 0, Right is index 1
                l_idx = 0
                r_idx = 1
            else:
                self.get_logger().warn("Wheel joints not found in joint_states message. Skipping FK/Odometry.")
                return

        # 3. ADDED NOISE: Apply Gaussian noise to the wheel position readings
        wheel_encoder_left_pos = msg.position[l_idx] + np.random.normal(0, 0.005)
        wheel_encoder_right_pos = msg.position[r_idx] + np.random.normal(0, 0.005)

        # --- Calculate Incremental Change ---
        # Calculate delta position using the noisy encoder readings
        dp_left = wheel_encoder_left_pos - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right_pos - self.right_wheel_prev_pos_
        
        current_time = self.get_clock().now()
        dt_ros = current_time - self.prev_time_
        dt_sec = dt_ros.nanoseconds / 1e9

        # Update state variables for next iteration using the *noisy* position
        self.left_wheel_prev_pos_ = wheel_encoder_left_pos
        self.right_wheel_prev_pos_ = wheel_encoder_right_pos
        self.prev_time_ = current_time

        if dt_sec <= 0:
            return 
            
        # Angular velocity of the wheels (rad/s)
        fi_left = dp_left / dt_sec
        fi_right = dp_right / dt_sec

        # --- FORWARD KINEMATICS (Velocity Calculation - Using template's equations) ---
        linear = ((self.wheel_radius_ * fi_left) + (self.wheel_radius_ * fi_right)) / 2
        angular = (-(self.wheel_radius_ * fi_left) + (self.wheel_radius_ * fi_right)) / self.wheel_separation_

        # --- ODOMETRY INTEGRATION (Pose Update - Using template's equations and midpoint integration) ---
        d_s = ((self.wheel_radius_ * dp_left) + (self.wheel_radius_ * dp_right)) / 2
        d_theta = (-(self.wheel_radius_ * dp_left) + (self.wheel_radius_ * dp_right)) / self.wheel_separation_

        # Update accumulated pose (x, y, theta)
        self.x_ += d_s * math.cos(self.theta_ + d_theta / 2)
        self.y_ += d_s * math.sin(self.theta_ + d_theta / 2)
        self.theta_ += d_theta
        
        # Normalize theta to [-pi, pi]
        self.theta_ = math.atan2(math.sin(self.theta_), math.cos(self.theta_))

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
        
        self.odom_pub_.publish(self.odom_msg_) # Publish the NOISY Odometry message

        # 3. Fill in TF message (geometry_msgs/TransformStamped)
        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        
        self.br_.sendTransform(self.transform_stamped_) # Broadcast the odom -> base_link TF

        # Removed FK Velocity Publishing (TwistStamped)
        # ---


def main():
    rclpy.init()

    # 4. Updated class name in main
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()