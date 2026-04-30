#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuRepublisher(Node):
    """
    Subscribes to raw /imu data, changes the frame_id to the URDF's 'imu_link', 
    and republishes it to a topic the EKF is listening on.
    """
    def __init__(self):
        super().__init__('imu_republisher_node')
        
        # 1. Create the Publisher to the EKF topic
        self.imu_pub_ = self.create_publisher(Imu, "imu_ekf", 10)
        
        # 2. Create the Subscriber to the raw IMU topic
        self.imu_sub_ = self.create_subscription(
            Imu, 
            "imu", 
            self.imu_callback, 
            10
        )
        
        # CRITICAL CORRECTION: Set the frame ID to the URDF's IMU link 
        # ('imu_link') so the Robot State Publisher provides the transform 
        # to 'base_link' (which the EKF then relates to 'base_link_ekf').
        self.target_frame_id_ = "imu_link_ekf"
        
        self.get_logger().info(f"IMU Republisher initialized. Publishing to /imu_ekf with frame_id: {self.target_frame_id_}")


    def imu_callback(self, imu_msg):
        """
        Receives the raw IMU message, modifies the header, and publishes it.
        """
        # Set the target frame_id required by the EKF
        imu_msg.header.frame_id = self.target_frame_id_
        
        # Publish the modified message
        self.imu_pub_.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    
    imu_republisher = ImuRepublisher()
    
    # Keeps the node alive and processing callbacks
    rclpy.spin(imu_republisher)
    
    # Clean up when the node is shut down
    imu_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()