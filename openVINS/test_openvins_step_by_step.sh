#!/bin/bash

# Step-by-step OpenVINS debugging script
# Run this inside Docker to diagnose why topics aren't appearing

echo "========================================"
echo "OpenVINS Step-by-Step Diagnosis"
echo "========================================"
echo ""

# Ensure we're in the right place
cd /catkin_ws
source install/setup.bash 2>/dev/null

# Step 1: Check ROS2 environment
echo "Step 1: Checking ROS2 Environment"
echo "-----------------------------------"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT SET}"
echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-NOT SET}"
echo ""

# Step 2: Restart ROS2 daemon
echo "Step 2: Restarting ROS2 Daemon"
echo "-----------------------------------"
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2
echo "✓ Daemon restarted"
echo ""

# Step 3: Check if bag file exists
echo "Step 3: Checking Dataset"
echo "-----------------------------------"
BAG_PATH="/datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3"
if [ -f "$BAG_PATH" ]; then
    echo "✓ Bag file found: $BAG_PATH"
else
    echo "✗ Bag file NOT found: $BAG_PATH"
    exit 1
fi
echo ""

# Step 4: Check bag topics
echo "Step 4: Checking Bag Topics"
echo "-----------------------------------"
echo "Topics in bag:"
ros2 bag info "$BAG_PATH" 2>&1 | grep -A 1 "Topic information"
echo ""

# Step 5: Test bag playback
echo "Step 5: Testing Bag Playback (5 seconds)"
echo "-----------------------------------"
echo "Playing bag for 5 seconds to verify topics..."
timeout 5 ros2 bag play "$BAG_PATH" -r 1.0 &
BAG_PID=$!
sleep 6

echo ""
echo "Topics visible from bag:"
ros2 topic list | grep -E "(snappy|groundtruth)"
echo ""

# Kill bag if still running
kill $BAG_PID 2>/dev/null
sleep 2

# Step 6: Check if OpenVINS executable exists
echo "Step 6: Checking OpenVINS Installation"
echo "-----------------------------------"
if [ -f "install/ov_msckf/lib/ov_msckf/run_subscribe_msckf" ]; then
    echo "✓ OpenVINS executable found"
else
    echo "✗ OpenVINS NOT installed. Run: colcon build"
    exit 1
fi
echo ""

# Step 7: Check config file
echo "Step 7: Checking Configuration"
echo "-----------------------------------"
CONFIG_PATH="src/open_vins/config/uzhfpv_indoor/estimator_config.yaml"
if [ -f "$CONFIG_PATH" ]; then
    echo "✓ Config file found: $CONFIG_PATH"
    echo ""
    echo "Camera topics in config:"
    grep -A 2 "topic:" "$CONFIG_PATH" | head -10
else
    echo "✗ Config file NOT found: $CONFIG_PATH"
    exit 1
fi
echo ""

# Step 8: Start bag in background
echo "Step 8: Starting Bag Player"
echo "-----------------------------------"
echo "Starting bag at 0.5x speed..."
ros2 bag play "$BAG_PATH" -r 0.5 --loop > /tmp/bag_output.log 2>&1 &
BAG_PID=$!
echo "Bag PID: $BAG_PID"
sleep 5

# Verify bag is publishing
echo ""
echo "Checking if bag topics are visible:"
ros2 topic list | grep snappy
echo ""

# Step 9: Check topic rates
echo "Step 9: Checking Topic Rates (5 seconds)"
echo "-----------------------------------"
echo "Camera topic rate:"
timeout 5 ros2 topic hz /snappy_cam/stereo_l 2>&1 | grep "average rate" || echo "No messages on /snappy_cam/stereo_l"
echo ""
echo "IMU topic rate:"
timeout 5 ros2 topic hz /snappy_imu 2>&1 | grep "average rate" || echo "No messages on /snappy_imu"
echo ""

# Step 10: Try running OpenVINS
echo "Step 10: Running OpenVINS (will run in foreground)"
echo "-----------------------------------"
echo "Starting OpenVINS..."
echo "Watch for errors below!"
echo "Press Ctrl+C to stop when you want to check topics"
echo ""
echo "Command: ros2 run ov_msckf run_subscribe_msckf $CONFIG_PATH"
echo ""
sleep 2

# Run OpenVINS
ros2 run ov_msckf run_subscribe_msckf "$CONFIG_PATH"

# Cleanup
echo ""
echo "Cleaning up..."
kill $BAG_PID 2>/dev/null
