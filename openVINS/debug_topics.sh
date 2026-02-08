#!/bin/bash

# Debug ROS2 Topic Communication Issues
# Run this inside Docker to diagnose why topics aren't visible

echo "========================================"
echo "ROS2 Topic Communication Diagnostics"
echo "========================================"
echo ""

# Check ROS environment
echo "1. ROS2 Environment:"
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
echo "   ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-not set}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo ""

# List all topics
echo "2. Available ROS2 Topics:"
ros2 topic list
echo ""

# Check if OpenVINS node is running
echo "3. Active ROS2 Nodes:"
ros2 node list
echo ""

# Check daemon
echo "4. ROS2 Daemon Status:"
ros2 daemon status
echo ""

# Network configuration
echo "5. Network Configuration:"
echo "   Checking localhost..."
ping -c 1 127.0.0.1 > /dev/null 2>&1 && echo "   ✓ Localhost OK" || echo "   ✗ Localhost FAIL"
echo ""

# DDS discovery
echo "6. Testing Topic Echo (5 second timeout):"
echo "   Checking if bag topics are visible..."
timeout 5 ros2 topic echo /snappy_imu --once > /dev/null 2>&1 && echo "   ✓ Can see /snappy_imu" || echo "   ✗ Cannot see /snappy_imu"
echo ""

echo "========================================"
echo "Diagnostic Complete"
echo "========================================"
echo ""
echo "Common Issues & Fixes:"
echo ""
echo "Issue: Topics not visible across terminals"
echo "Fix: Set ROS_DOMAIN_ID consistently"
echo "  export ROS_DOMAIN_ID=0"
echo ""
echo "Issue: 'Cannot publish data' error"
echo "Fix: Restart ROS2 daemon"
echo "  ros2 daemon stop"
echo "  ros2 daemon start"
echo ""
echo "Issue: DDS discovery problems"
echo "Fix: Use localhost-only mode"
echo "  export ROS_LOCALHOST_ONLY=1"
echo ""
