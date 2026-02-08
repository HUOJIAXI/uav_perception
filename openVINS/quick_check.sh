#!/bin/bash

# Ultra-simple check script - run inside Docker
# This will tell you exactly what's wrong

echo "OpenVINS Quick Health Check"
echo "==========================="
echo ""

cd /catkin_ws 2>/dev/null || { echo "ERROR: Not in /catkin_ws"; exit 1; }
source install/setup.bash 2>/dev/null

# Check 1: Executable exists
echo -n "1. OpenVINS built? "
if [ -f "install/ov_msckf/lib/ov_msckf/run_subscribe_msckf" ]; then
    echo "✓ YES"
else
    echo "✗ NO - Run: colcon build"
    exit 1
fi

# Check 2: Bag exists
echo -n "2. Dataset exists? "
BAG="/datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3"
if [ -f "$BAG" ]; then
    echo "✓ YES"
else
    echo "✗ NO - Check dataset path"
    exit 1
fi

# Check 3: ROS environment
echo -n "3. ROS env set? "
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "⚠ ROS_DOMAIN_ID not set (might cause issues)"
else
    echo "✓ ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# Check 4: Play bag briefly
echo -n "4. Bag playable? "
timeout 3 ros2 bag play "$BAG" -r 1.0 > /dev/null 2>&1 &
PID=$!
sleep 4
kill $PID 2>/dev/null
ros2 topic list | grep -q snappy && echo "✓ YES" || echo "✗ NO - Bag topics not visible"

# Check 5: Topics visible
echo ""
echo "5. Available topics:"
ros2 topic list | grep -E "(snappy|ov_msckf|groundtruth)" || echo "   (none relevant)"

echo ""
echo "==========================="
echo ""
echo "Next step: Run OpenVINS manually"
echo ""
echo "Command:"
echo "  ros2 bag play $BAG -r 0.5 &"
echo "  sleep 5"
echo "  ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml"
echo ""
