#!/usr/bin/env python3
"""
Timestamp fix node for IMU - converts wall time to sim time
This is a workaround for PX4/Pegasus not using use_sim_time
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuTimestampFixer(Node):
    def __init__(self):
        super().__init__('imu_timestamp_fixer')

        # Subscribe to original IMU with wall time
        self.subscription = self.create_subscription(
            Imu,
            '/telemetry/imu',
            self.imu_callback,
            10)

        # Publish fixed IMU with sim time
        self.publisher = self.create_publisher(Imu, '/telemetry/imu_fixed', 10)

        self.get_logger().info('IMU timestamp fixer started')
        self.get_logger().info('Subscribing to: /telemetry/imu')
        self.get_logger().info('Publishing to: /telemetry/imu_fixed')

    def imu_callback(self, msg):
        # Use current ROS time (which will be sim time if use_sim_time is true)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTimestampFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
