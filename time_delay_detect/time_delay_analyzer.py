#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu
from collections import deque
import time
import numpy as np
import json

class TimeDelayAnalyzer(Node):
    def __init__(self):
        super().__init__('time_delay_analyzer')

        # Storage for time measurements
        self.clock_times = deque(maxlen=1000)  # Store last 1000 samples
        self.camera_data = deque(maxlen=1000)
        self.imu_data = deque(maxlen=1000)

        # Latest clock time
        self.latest_clock_time = None

        # Subscribers
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/pegasus_1/rgb',
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/telemetry/imu',
            self.imu_callback,
            10
        )

        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(5.0, self.analyze_delays)

        self.get_logger().info('Time Delay Analyzer started. Collecting data...')

    def clock_callback(self, msg):
        """Record clock updates"""
        clock_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        wall_time = time.time()

        self.clock_times.append({
            'wall_time': wall_time,
            'clock_time': clock_time
        })

        self.latest_clock_time = clock_time

    def camera_callback(self, msg):
        """Record camera message timestamps"""
        wall_time = time.time()
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.camera_data.append({
            'wall_time': wall_time,
            'msg_time': msg_time,
            'clock_time_at_reception': self.latest_clock_time
        })

    def imu_callback(self, msg):
        """Record IMU message timestamps"""
        wall_time = time.time()
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.imu_data.append({
            'wall_time': wall_time,
            'msg_time': msg_time,
            'clock_time_at_reception': self.latest_clock_time
        })

    def analyze_delays(self):
        """Analyze the time delays"""
        if len(self.camera_data) < 10 or len(self.imu_data) < 10:
            self.get_logger().info('Collecting data... (need at least 10 samples)')
            return

        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('TIME DELAY ANALYSIS')
        self.get_logger().info('='*80)

        # Analyze camera delays
        camera_delays = []
        for data in self.camera_data:
            if data['clock_time_at_reception'] is not None:
                delay = data['clock_time_at_reception'] - data['msg_time']
                camera_delays.append(delay)

        # Analyze IMU delays
        imu_delays = []
        for data in self.imu_data:
            if data['clock_time_at_reception'] is not None:
                delay = data['clock_time_at_reception'] - data['msg_time']
                imu_delays.append(delay)

        # Calculate statistics
        if camera_delays:
            self.get_logger().info('\nCAMERA (/pegasus_1/rgb) Delays:')
            self.get_logger().info(f'  Samples: {len(camera_delays)}')
            self.get_logger().info(f'  Mean delay: {np.mean(camera_delays):.6f} s')
            self.get_logger().info(f'  Std deviation: {np.std(camera_delays):.6f} s')
            self.get_logger().info(f'  Min delay: {np.min(camera_delays):.6f} s')
            self.get_logger().info(f'  Max delay: {np.max(camera_delays):.6f} s')
            self.get_logger().info(f'  Range: {np.max(camera_delays) - np.min(camera_delays):.6f} s')

            # Check if delay is constant (std < 1ms indicates roughly constant)
            if np.std(camera_delays) < 0.001:
                self.get_logger().info(f'  ✓ Delay appears CONSTANT (std < 1ms)')
            else:
                self.get_logger().info(f'  ✗ Delay is VARIABLE (std >= 1ms)')

        if imu_delays:
            self.get_logger().info('\nIMU (/telemetry/imu) Delays:')
            self.get_logger().info(f'  Samples: {len(imu_delays)}')
            self.get_logger().info(f'  Mean delay: {np.mean(imu_delays):.6f} s')
            self.get_logger().info(f'  Std deviation: {np.std(imu_delays):.6f} s')
            self.get_logger().info(f'  Min delay: {np.min(imu_delays):.6f} s')
            self.get_logger().info(f'  Max delay: {np.max(imu_delays):.6f} s')
            self.get_logger().info(f'  Range: {np.max(imu_delays) - np.min(imu_delays):.6f} s')

            if np.std(imu_delays) < 0.001:
                self.get_logger().info(f'  ✓ Delay appears CONSTANT (std < 1ms)')
            else:
                self.get_logger().info(f'  ✗ Delay is VARIABLE (std >= 1ms)')

        # Compare camera vs IMU
        if camera_delays and imu_delays:
            self.get_logger().info('\nCOMPARISON:')
            delay_diff = abs(np.mean(camera_delays) - np.mean(imu_delays))
            self.get_logger().info(f'  Camera-IMU delay difference: {delay_diff:.6f} s ({delay_diff*1000:.3f} ms)')

        self.get_logger().info('='*80 + '\n')

    def save_data(self, filename='time_delay_data.json'):
        """Save collected data to file for further analysis"""
        data = {
            'camera_data': list(self.camera_data),
            'imu_data': list(self.imu_data),
            'clock_data': list(self.clock_times)
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'Data saved to {filename}')


def main(args=None):
    rclpy.init(args=args)

    analyzer = TimeDelayAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('\n\nShutting down...')
        analyzer.save_data()
        analyzer.get_logger().info('Final analysis:')
        analyzer.analyze_delays()
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
