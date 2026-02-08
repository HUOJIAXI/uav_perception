#!/usr/bin/env python3
"""
Robust timestamp synchronizer for IMU
Calculates offset between wall time and sim time, then corrects IMU timestamps
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
import time

class TimestampSynchronizer(Node):
    def __init__(self):
        super().__init__('timestamp_synchronizer')

        self.sim_time = None
        self.wall_time = None
        self.offset_calculated = False
        self.time_offset = 0.0

        # Subscribe to clock to get sim time
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)

        # Subscribe to original IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/telemetry/imu',
            self.imu_callback,
            10)  # Use QoS depth 10 for RELIABLE

        # Publish synchronized IMU
        self.imu_pub = self.create_publisher(Imu, '/telemetry/imu_synced', 10)

        self.get_logger().info('Timestamp synchronizer started')
        self.get_logger().info('Waiting for /clock and /telemetry/imu...')

    def clock_callback(self, msg):
        """Track simulation time from /clock"""
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def imu_callback(self, msg):
        """Synchronize IMU timestamps"""
        if self.sim_time is None:
            self.get_logger().warn('No /clock received yet, skipping...', throttle_duration_sec=5.0)
            return

        # Get wall time from IMU message
        imu_wall_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Calculate offset on first message
        if not self.offset_calculated:
            self.time_offset = imu_wall_time - self.sim_time
            self.offset_calculated = True
            self.get_logger().info(f'Calculated time offset: {self.time_offset:.3f} seconds')
            self.get_logger().info(f'IMU wall time: {imu_wall_time:.3f}s, Sim time: {self.sim_time:.3f}s')

        # Apply offset to correct timestamp
        corrected_time = imu_wall_time - self.time_offset

        # Update message timestamp
        msg.header.stamp.sec = int(corrected_time)
        msg.header.stamp.nanosec = int((corrected_time - int(corrected_time)) * 1e9)

        # Publish corrected IMU
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimestampSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
