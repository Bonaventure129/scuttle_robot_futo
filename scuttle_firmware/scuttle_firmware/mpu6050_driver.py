#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import smbus
import math

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68

class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")
        
        # I2C Interface
        self.bus_ = None
        self.is_connected_ = False
        self.init_i2c()

        # ROS 2 Interface
        # Use sensor data QoS for best effort delivery (low latency)
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "imu_link"  # Changed to match standard URDF frame
        
        # Covariance matrices (Estimated values)
        # Orientation covariance (unknown for now)
        self.imu_msg_.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        
        # Angular velocity covariance
        self.imu_msg_.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        # Linear acceleration covariance
        self.imu_msg_.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        self.frequency_ = 0.01  # 100 Hz
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        
        self.get_logger().info("MPU6050 Driver Started")

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1) # Use I2C bus 1
            # Wake up the sensor
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
            self.get_logger().info("MPU6050 Connected Successfully")
        except Exception as e:
            self.is_connected_ = False
            self.get_logger().error(f"Failed to connect to MPU6050: {str(e)}")

    def read_raw_data(self, addr):
        try:
            # Accelero and Gyro value are 16-bit
            high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
            low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)
            
            # Concatenate higher and lower value
            value = ((high << 8) | low)
                
            # To get signed value from mpu6050
            if(value > 32768):
                value = value - 65536
            return value
        except Exception:
            return 0

    def timerCallback(self):
        if not self.is_connected_:
            self.init_i2c()
            return

        try:
            # Read Accelerometer raw value
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)
            
            # Read Gyroscope raw value
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)
            
            # Constants for conversion (based on default sensitivity settings)
            # Accelerometer Sensitivity Range +/- 2g: 16384 LSB/g
            # Gyroscope Sensitivity Range +/- 250 deg/s: 131 LSB/(deg/s)
            
            # Convert to standard units
            # Acceleration: g -> m/s^2 (1g = 9.81 m/s^2)
            GRAVITY_MS2 = 9.80665
            self.imu_msg_.linear_acceleration.x = (acc_x / 16384.0) * GRAVITY_MS2
            self.imu_msg_.linear_acceleration.y = (acc_y / 16384.0) * GRAVITY_MS2
            self.imu_msg_.linear_acceleration.z = (acc_z / 16384.0) * GRAVITY_MS2
            
            # Angular Velocity: deg/s -> rad/s
            DEG_TO_RAD = math.pi / 180.0
            self.imu_msg_.angular_velocity.x = (gyro_x / 131.0) * DEG_TO_RAD
            self.imu_msg_.angular_velocity.y = (gyro_y / 131.0) * DEG_TO_RAD
            self.imu_msg_.angular_velocity.z = (gyro_z / 131.0) * DEG_TO_RAD

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
            
        except OSError:
            self.get_logger().warn("I2C Communication Error")
            self.is_connected_ = False

def main(args=None):
    rclpy.init(args=args)
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()