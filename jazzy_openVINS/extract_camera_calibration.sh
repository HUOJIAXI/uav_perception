#!/bin/bash

# ========================================================================
# Camera Calibration Extraction Helper Script
# ========================================================================
# This script helps extract camera calibration from camera_info topic
# and displays it in OpenVINS-compatible format

set -e

echo "=========================================================================="
echo "OpenVINS Camera Calibration Extraction Helper"
echo "=========================================================================="
echo ""

# Check if topic is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <camera_info_topic>"
    echo ""
    echo "Example:"
    echo "  $0 /camera/camera_info"
    echo ""
    echo "Available camera_info topics:"
    ros2 topic list | grep camera_info || echo "  (none found)"
    echo ""
    exit 1
fi

TOPIC=$1

echo "Attempting to read from topic: $TOPIC"
echo "Waiting for message (this may take a few seconds)..."
echo ""

# Check if topic exists
if ! ros2 topic list | grep -q "^${TOPIC}$"; then
    echo "ERROR: Topic '${TOPIC}' not found!"
    echo ""
    echo "Available topics:"
    ros2 topic list | grep camera_info || echo "  (none found)"
    exit 1
fi

# Capture camera_info message
TEMP_FILE=$(mktemp)
timeout 10 ros2 topic echo "$TOPIC" --once > "$TEMP_FILE" 2>&1 || {
    echo "ERROR: Failed to read from topic (timeout after 10s)"
    echo "Make sure the topic is publishing."
    rm -f "$TEMP_FILE"
    exit 1
}

echo "✓ Successfully captured camera_info message"
echo ""
echo "=========================================================================="
echo "Extracted Calibration Parameters"
echo "=========================================================================="
echo ""

# Extract values
WIDTH=$(grep -A 1 "^width:" "$TEMP_FILE" | tail -1 | awk '{print $1}')
HEIGHT=$(grep -A 1 "^height:" "$TEMP_FILE" | tail -1 | awk '{print $1}')
DIST_MODEL=$(grep -A 1 "distortion_model:" "$TEMP_FILE" | tail -1 | tr -d "'\"" | xargs)

# Extract K matrix (3x3 in row-major order)
K_LINE=$(grep -A 1 "^k:" "$TEMP_FILE" | tail -1)
K_ARRAY=($K_LINE)
FX=${K_ARRAY[0]//[,\[\]]/}
FY=${K_ARRAY[4]//[,\[\]]/}
CX=${K_ARRAY[2]//[,\[\]]/}
CY=${K_ARRAY[5]//[,\[\]]/}

# Extract D (distortion coefficients)
D_LINE=$(grep -A 1 "^d:" "$TEMP_FILE" | tail -1)
D_ARRAY=($D_LINE)
K1=${D_ARRAY[0]//[,\[\]]/}
K2=${D_ARRAY[1]//[,\[\]]/}
P1=${D_ARRAY[2]//[,\[\]]/}
P2=${D_ARRAY[3]//[,\[\]]/}

# Determine OpenVINS distortion model
if [[ "$DIST_MODEL" == "plumb_bob" ]] || [[ "$DIST_MODEL" == "rational_polynomial" ]]; then
    OV_DIST_MODEL="radtan"
elif [[ "$DIST_MODEL" == *"fisheye"* ]] || [[ "$DIST_MODEL" == *"equidistant"* ]]; then
    OV_DIST_MODEL="equidistant"
else
    OV_DIST_MODEL="radtan"
fi

echo "Image Resolution:"
echo "  resolution: [$WIDTH, $HEIGHT]"
echo ""
echo "Camera Intrinsics:"
echo "  intrinsics: [$FX, $FY, $CX, $CY]"
echo ""
echo "Distortion Model:"
echo "  distortion_model: $OV_DIST_MODEL  (from ROS: $DIST_MODEL)"
echo ""
echo "Distortion Coefficients:"
echo "  distortion_coeffs: [$K1, $K2, $P1, $P2]"
echo ""

echo "=========================================================================="
echo "Copy-Paste for kalibr_imucam_chain.yaml"
echo "=========================================================================="
echo ""
cat << EOF
cam0:
  resolution: [$WIDTH, $HEIGHT]
  intrinsics: [$FX, $FY, $CX, $CY]
  distortion_model: $OV_DIST_MODEL
  distortion_coeffs: [$K1, $K2, $P1, $P2]
  rostopic: $TOPIC  # or your image topic
  camera_model: pinhole
  cam_overlaps: []
  timeshift_cam_imu: 0.0
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
EOF

echo ""
echo "=========================================================================="
echo "Next Steps"
echo "=========================================================================="
echo ""
echo "1. Copy the above configuration to your kalibr_imucam_chain.yaml file"
echo ""
echo "2. Update the rostopic field to your IMAGE topic (not camera_info)"
echo "   Example: rostopic: /camera/image_raw"
echo ""
echo "3. If you know the camera-IMU transformation, update T_cam_imu"
echo "   Otherwise, set calib_cam_extrinsics: true in estimator_config.yaml"
echo ""
echo "4. Configure your IMU topic in kalibr_imu_chain.yaml"
echo ""
echo "5. Launch OpenVINS:"
echo "   ros2 launch ov_msckf subscribe.launch.py \\"
echo "     config:=your_config_name \\"
echo "     max_cameras:=1 \\"
echo "     use_stereo:=false"
echo ""

# Cleanup
rm -f "$TEMP_FILE"
