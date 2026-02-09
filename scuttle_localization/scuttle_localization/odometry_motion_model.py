#!/usr/bin/env python3
import random
import time
from math import sin, cos, atan2, sqrt, fabs, pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion


def normalize(z):
    """Normalize angle to [-pi, pi]."""
    return atan2(sin(z), cos(z))


def angle_diff(a, b):
    """Compute the normalized difference between two angles."""
    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    return d1 if fabs(d1) < fabs(d2) else d2


class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__('odometry_motion_model')

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        self.first_odom = True

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('alpha1', 0.1),
                ('alpha2', 0.1),
                ('alpha3', 0.1),
                ('alpha4', 0.1),
                ('nr_samples', 300)
            ]
        )

        # Load parameters
        self.alpha1 = self.get_parameter('alpha1').value
        self.alpha2 = self.get_parameter('alpha2').value
        self.alpha3 = self.get_parameter('alpha3').value
        self.alpha4 = self.get_parameter('alpha4').value
        self.nr_samples = self.get_parameter('nr_samples').value

        # Create PoseArray for samples
        self.samples = PoseArray()
        self.samples.poses = [Pose() for _ in range(self.nr_samples)]

        # ROS interfaces
        self.odom_sub = self.create_subscription(
            Odometry,
            '/scuttle_controller/odom',
            self.odom_callback,
            10
        )

        self.samples_pub = self.create_publisher(
            PoseArray,
            '/odometry_motion_model/samples',
            10
        )

        self.get_logger().info("Odometry Motion Model Node Started")

    # ---------------------------------------------------------------------
    def odom_callback(self, odom_msg):
        """Process new odometry reading and sample poses."""

        # Extract yaw
        q = odom_msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # First odometry message → initialize
        if self.first_odom:
            self.first_odom = False
            self.samples.header.frame_id = odom_msg.header.frame_id

            self.last_x = odom_msg.pose.pose.position.x
            self.last_y = odom_msg.pose.pose.position.y
            self.last_theta = yaw
            return

        # Odometry increments
        dx = odom_msg.pose.pose.position.x - self.last_x
        dy = odom_msg.pose.pose.position.y - self.last_y
        dtheta = angle_diff(yaw, self.last_theta)

        # Motion model decomposition
        delta_rot1 = 0.0 if sqrt(dx*dx + dy*dy) < 0.01 else angle_diff(atan2(dy, dx), self.last_theta)
        delta_trans = sqrt(dx*dx + dy*dy)
        delta_rot2 = angle_diff(dtheta, delta_rot1)

        # Noise models
        rot1_var = self.alpha1 * abs(delta_rot1) + self.alpha2 * abs(delta_trans)
        trans_var = self.alpha3 * abs(delta_trans) + self.alpha4 * abs(delta_rot1 + delta_rot2)
        rot2_var = self.alpha1 * abs(delta_rot2) + self.alpha2 * abs(delta_trans)

        random.seed(int(time.time()))

        # Update samples
        for sample in self.samples.poses:
            # Noise added
            dr1 = delta_rot1 + random.gauss(0.0, rot1_var)
            dt  = delta_trans + random.gauss(0.0, trans_var)
            dr2 = delta_rot2 + random.gauss(0.0, rot2_var)

            # Extract orientation of sample
            sq = sample.orientation
            _, _, syaw = euler_from_quaternion([sq.x, sq.y, sq.z, sq.w])

            # Apply motion update
            sample.position.x += dt * cos(syaw + dr1)
            sample.position.y += dt * sin(syaw + dr1)

            new_q = quaternion_from_euler(0.0, 0.0, syaw + dr1 + dr2)
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = new_q

        # Save current odometry for next cycle
        self.last_x = odom_msg.pose.pose.position.x
        self.last_y = odom_msg.pose.pose.position.y
        self.last_theta = yaw

        # Publish
        self.samples_pub.publish(self.samples)

    # ---------------------------------------------------------------------


def main():
    rclpy.init()
    node = OdometryMotionModel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
