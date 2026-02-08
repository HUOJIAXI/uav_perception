#!/bin/bash

# IMU Data Diagnostic Script
# Run this while Isaac Sim and OpenVINS are running

echo "=================================================="
echo "IMU Data Diagnostic Tool"
echo "=================================================="
echo ""

# Source ROS2
source install/setup.bash

echo "1. Checking available IMU topics..."
ros2 topic list | grep -i imu
echo ""

echo "2. Getting IMU topic info..."
ros2 topic info /telemetry/imu -v
echo ""

echo "3. Checking IMU publish rate..."
echo "   (Measuring for 5 seconds...)"
ros2 topic hz /telemetry/imu --window 100 &
HZ_PID=$!
sleep 5
kill $HZ_PID 2>/dev/null
echo ""

echo "4. Capturing 10 IMU samples..."
echo "   Checking for:"
echo "   - Accelerometer values (should be ~9.81 m/s^2 when stationary in Z)"
echo "   - Gyroscope values (should be ~0 rad/s when stationary)"
echo "   - Frame orientation"
echo "   - Timestamp consistency"
echo ""
timeout 10 ros2 topic echo /telemetry/imu --once > /tmp/imu_sample.txt 2>&1

if [ -f /tmp/imu_sample.txt ]; then
    cat /tmp/imu_sample.txt
    echo ""

    # Extract key values
    echo "=================================================="
    echo "ANALYSIS:"
    echo "=================================================="

    # Check accelerometer
    echo "Accelerometer (linear_acceleration):"
    grep -A 3 "linear_acceleration:" /tmp/imu_sample.txt | tail -3
    echo ""

    # Check gyroscope
    echo "Gyroscope (angular_velocity):"
    grep -A 3 "angular_velocity:" /tmp/imu_sample.txt | tail -3
    echo ""

    # Check orientation
    echo "Orientation (quaternion):"
    grep -A 4 "orientation:" /tmp/imu_sample.txt | tail -4
    echo ""

    # Check frame
    echo "Frame ID:"
    grep "frame_id:" /tmp/imu_sample.txt
    echo ""

    echo "=================================================="
    echo "COMMON ISSUES TO CHECK:"
    echo "=================================================="
    echo "1. Accelerometer Z-axis stationary value:"
    echo "   - Should be ~±9.81 m/s^2 (gravity)"
    echo "   - If ~0, gravity is missing or wrong frame"
    echo "   - If very large (>50), wrong units"
    echo ""
    echo "2. Gyroscope when stationary:"
    echo "   - Should be ~0 rad/s"
    echo "   - If large constant bias, IMU needs calibration"
    echo "   - If noisy (>0.1 rad/s), noise too high"
    echo ""
    echo "3. Frame orientation:"
    echo "   - Check if IMU frame matches camera frame"
    echo "   - Check T_imu_cam in kalibr_imucam_chain.yaml"
    echo ""
    echo "4. Units:"
    echo "   - Accelerometer: m/s^2 (NOT g's)"
    echo "   - Gyroscope: rad/s (NOT deg/s)"
    echo "   - If wrong, scale by 9.81 or π/180"
    echo ""

    # Check for obviously wrong values
    accel_z=$(grep -A 3 "linear_acceleration:" /tmp/imu_sample.txt | grep "z:" | awk '{print $2}')
    if [ ! -z "$accel_z" ]; then
        accel_z_abs=$(echo "$accel_z" | sed 's/-//')
        echo "5. Detected accelerometer Z = $accel_z m/s^2"

        # Check if close to 9.81
        if (( $(echo "$accel_z_abs > 8.0 && $accel_z_abs < 11.0" | bc -l) )); then
            echo "   ✅ GOOD: Close to 9.81 (gravity detected)"
        elif (( $(echo "$accel_z_abs < 1.0" | bc -l) )); then
            echo "   ❌ ERROR: Too small! Gravity missing or IMU in freefall frame"
        elif (( $(echo "$accel_z_abs > 50.0" | bc -l) )); then
            echo "   ❌ ERROR: Too large! Wrong units (maybe using g's instead of m/s^2)"
        else
            echo "   ⚠️  WARNING: Unusual value, check IMU frame and calibration"
        fi
    fi
    echo ""

else
    echo "❌ ERROR: Could not capture IMU data!"
    echo "   Make sure Isaac Sim is running and publishing to /telemetry/imu"
    echo ""
fi

echo "=================================================="
echo "6. Checking IMU configuration in OpenVINS..."
echo "=================================================="
echo ""
echo "IMU topic in config:"
grep -r "rostopic.*imu" src/open_vins_ros2_jazzy/config/pegasus_clean/ 2>/dev/null
echo ""
echo "IMU noise parameters:"
grep -A 4 "accelerometer_noise_density\|gyroscope_noise_density" src/open_vins_ros2_jazzy/config/pegasus_clean/kalibr_imu_chain.yaml
echo ""
echo "IMU update rate:"
grep "update_rate" src/open_vins_ros2_jazzy/config/pegasus_clean/kalibr_imu_chain.yaml
echo ""

echo "=================================================="
echo "To monitor IMU in real-time, run:"
echo "  ros2 topic echo /telemetry/imu"
echo ""
echo "To plot IMU data:"
echo "  rqt_plot /telemetry/imu/linear_acceleration/z"
echo "=================================================="
