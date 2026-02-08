#!/bin/bash

# Docker Mount Verification Script
# Run this INSIDE the Docker container to debug mounting issues

echo "======================================"
echo "Docker Mount Verification"
echo "======================================"
echo ""

echo "1. Current directory:"
pwd
echo ""

echo "2. Contents of /catkin_ws:"
ls -la /catkin_ws/ 2>/dev/null || echo "ERROR: /catkin_ws does not exist!"
echo ""

echo "3. Contents of /catkin_ws/src:"
ls -la /catkin_ws/src/ 2>/dev/null || echo "ERROR: /catkin_ws/src does not exist!"
echo ""

echo "4. Checking open_vins symlink:"
if [ -L "/catkin_ws/src/open_vins" ]; then
    echo "✓ Symlink exists"
    ls -la /catkin_ws/src/open_vins | grep "^l"
    echo ""
    echo "Target of symlink:"
    readlink /catkin_ws/src/open_vins
else
    echo "✗ Symlink does NOT exist at /catkin_ws/src/open_vins"
fi
echo ""

echo "5. Contents of /catkin_ws/src/open_vins (first 20 items):"
ls -la /catkin_ws/src/open_vins/ 2>/dev/null | head -20 || echo "ERROR: Cannot list directory!"
echo ""

echo "6. Looking for run_uzhfpv_indoor.sh:"
if [ -f "/catkin_ws/src/open_vins/run_uzhfpv_indoor.sh" ]; then
    echo "✓ FOUND at /catkin_ws/src/open_vins/run_uzhfpv_indoor.sh"
    ls -la /catkin_ws/src/open_vins/run_uzhfpv_indoor.sh
else
    echo "✗ NOT FOUND at /catkin_ws/src/open_vins/run_uzhfpv_indoor.sh"
fi
echo ""

echo "7. Searching for the script in other locations:"
find /catkin_ws -name "run_uzhfpv_indoor.sh" 2>/dev/null || echo "Not found anywhere in /catkin_ws"
echo ""

echo "8. Contents of /datasets:"
ls -la /datasets/ 2>/dev/null || echo "ERROR: /datasets does not exist!"
echo ""

echo "9. All shell scripts in open_vins directory:"
find /catkin_ws/src/open_vins -name "*.sh" -type f 2>/dev/null | head -10 || echo "No .sh files found"
echo ""

echo "======================================"
echo "Diagnostic complete!"
echo "======================================"
