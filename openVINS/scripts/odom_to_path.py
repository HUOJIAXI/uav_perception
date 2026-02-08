#!/usr/bin/env python3
"""
Converts /groundtruth/odometry to /groundtruth/path for RViz visualization
This allows RViz Path display to show the ground truth trajectory
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')

        # Parameters
        self.declare_parameter('input_topic', '/groundtruth/odometry')
        self.declare_parameter('output_topic', '/groundtruth/path')
        self.declare_parameter('max_path_length', 10000)  # Maximum poses to keep

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.max_length = self.get_parameter('max_path_length').value

        # Initialize path message
        self.path = Path()
        self.path.header.frame_id = "global"

        # Create subscriber and publisher
        self.sub = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(Path, output_topic, 10)

        self.get_logger().info(f'Converting {input_topic} -> {output_topic}')
        self.get_logger().info(f'Max path length: {self.max_length}')

    def odom_callback(self, msg):
        """Convert odometry message to path"""
        # Create PoseStamped from Odometry
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Add to path
        self.path.poses.append(pose)

        # Limit path length
        if len(self.path.poses) > self.max_length:
            self.path.poses.pop(0)

        # Update path header timestamp
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = msg.header.frame_id

        # Publish
        self.pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPath()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
