#!/usr/bin/env python3
"""
Diagnostic tool to check camera-IMU transform by comparing motion
Run this while moving the UAV to see if IMU and camera motions align
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import TwistStamped
import numpy as np
from collections import deque
import time

class TransformDiagnostic(Node):
    def __init__(self):
        super().__init__('transform_diagnostic')

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/telemetry/imu', self.imu_callback, 10)
        self.img_sub = self.create_subscription(Image, '/pegasus_1/rgb', self.img_callback, 10)

        # Storage
        self.imu_data = deque(maxlen=100)
        self.img_data = deque(maxlen=100)

        # Timing
        self.last_print = time.time()

        self.get_logger().info('Transform Diagnostic Tool Started')
        self.get_logger().info('Move the UAV and watch for IMU-Camera alignment issues')

    def imu_callback(self, msg):
        # Store IMU angular velocity
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        self.imu_data.append({
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'gyro': gyro,
            'accel': accel,
            'frame': msg.header.frame_id
        })

    def img_callback(self, msg):
        # Store image timestamp
        self.img_data.append({
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame': msg.header.frame_id
        })

        # Print diagnostic info every 2 seconds
        now = time.time()
        if now - self.last_print > 2.0 and len(self.imu_data) > 10 and len(self.img_data) > 5:
            self.print_diagnostic()
            self.last_print = now

    def print_diagnostic(self):
        # Get recent data
        recent_imu = list(self.imu_data)[-10:]
        recent_img = list(self.img_data)[-5:]

        # Calculate IMU motion magnitude
        gyro_mag = np.mean([np.linalg.norm(d['gyro']) for d in recent_imu])
        accel_mag = np.mean([np.linalg.norm(d['accel']) for d in recent_imu])

        # Calculate IMU directions
        gyro_mean = np.mean([d['gyro'] for d in recent_imu], axis=0)
        accel_mean = np.mean([d['accel'] for d in recent_imu], axis=0)

        self.get_logger().info('='*60)
        self.get_logger().info(f'IMU Frame: {recent_imu[0]["frame"]}')
        self.get_logger().info(f'Camera Frame: {recent_img[0]["frame"]}')
        self.get_logger().info('')
        self.get_logger().info(f'IMU Angular Velocity (rad/s):')
        self.get_logger().info(f'  X: {gyro_mean[0]:7.4f}  (roll rate)')
        self.get_logger().info(f'  Y: {gyro_mean[1]:7.4f}  (pitch rate)')
        self.get_logger().info(f'  Z: {gyro_mean[2]:7.4f}  (yaw rate)')
        self.get_logger().info(f'  Magnitude: {gyro_mag:7.4f}')
        self.get_logger().info('')
        self.get_logger().info(f'IMU Linear Acceleration (m/s²):')
        self.get_logger().info(f'  X: {accel_mean[0]:7.4f}')
        self.get_logger().info(f'  Y: {accel_mean[1]:7.4f}')
        self.get_logger().info(f'  Z: {accel_mean[2]:7.4f}  (should be ~9.81 when stationary)')
        self.get_logger().info(f'  Magnitude: {accel_mag:7.4f}')
        self.get_logger().info('')

        # Check for issues
        if abs(accel_mean[2]) < 5.0:
            self.get_logger().warn('⚠️  WARNING: Z acceleration very low! Check IMU orientation')
        if abs(accel_mean[2]) > 15.0:
            self.get_logger().warn('⚠️  WARNING: Z acceleration very high! Check IMU frame')

        if gyro_mag > 0.1:
            self.get_logger().info('📊 UAV is rotating - good for testing!')
            self.get_logger().info('   Watch if OpenVINS estimation follows the rotation correctly')
        else:
            self.get_logger().info('⏸️  UAV stationary - try rotating to test transform')

def main(args=None):
    rclpy.init(args=args)
    node = TransformDiagnostic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
