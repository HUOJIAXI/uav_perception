# OpenVINS UAV Configuration Guide

A comprehensive guide for configuring OpenVINS with your custom UAV's monocular camera and IMU topics.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Detailed Configuration](#detailed-configuration)
- [Extracting Camera Calibration](#extracting-camera-calibration)
- [Launch Options](#launch-options)
- [Troubleshooting](#troubleshooting)
- [Advanced Topics](#advanced-topics)

---

## Overview

OpenVINS is a Visual-Inertial Navigation System that fuses camera and IMU data for state estimation. This guide shows you how to configure it for your UAV with:
- **Single monocular camera** (one image topic)
- **IMU topic**
- **Custom ROS2 topics**

### Important Notes
- OpenVINS **does NOT use camera_info topics** - all calibration comes from YAML files
- You only need to provide the **image topic** (not camera_info)
- Proper camera-IMU calibration is critical for good performance

---

## Prerequisites

### 1. Built OpenVINS ROS2 Package
```bash
cd ~/Desktop/uav_perception/jazzy_openVINS
colcon build
source install/setup.bash
```

### 2. Your UAV Topics Must Be Publishing
Check that your topics are available:
```bash
# Check IMU topic
ros2 topic echo /your/imu/topic --once

# Check camera topic
ros2 topic echo /your/camera/image/topic --once

# List all topics to verify
ros2 topic list
```

### 3. Camera-IMU Calibration
You need to know:
- Camera intrinsics (fx, fy, cx, cy)
- Camera distortion coefficients (k1, k2, p1, p2)
- Transformation between camera and IMU (rotation + translation)

If you don't have these, see [Extracting Camera Calibration](#extracting-camera-calibration).

---

## Quick Start

### Step 1: Create Your Configuration

```bash
cd ~/Desktop/uav_perception/jazzy_openVINS/src/open_vins_ros2_jazzy/config

# Copy an existing monocular config as template
cp -r rs_d435i_mono my_uav

cd my_uav
```

### Step 2: Edit Configuration Files

Edit the three YAML files (see [Detailed Configuration](#detailed-configuration) section):
- `estimator_config.yaml` - Main settings
- `kalibr_imu_chain.yaml` - IMU parameters and topic
- `kalibr_imucam_chain.yaml` - Camera parameters and topic

### Step 3: Launch OpenVINS

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false \
  verbosity:=INFO
```

Or with explicit topic names:
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  topic_imu:=/your/imu/topic \
  topic_camera0:=/your/camera/image \
  max_cameras:=1 \
  use_stereo:=false
```

---

## Detailed Configuration

### Configuration File Structure

Your config folder should contain:
```
my_uav/
├── estimator_config.yaml       # Main estimator settings
├── kalibr_imu_chain.yaml       # IMU calibration and topic
└── kalibr_imucam_chain.yaml    # Camera calibration and topic
```

### 1. estimator_config.yaml

Main configuration file for the estimator:

```yaml
%YAML:1.0

verbosity: "INFO"  # ALL, DEBUG, INFO, WARNING, ERROR, SILENT

# Camera configuration
use_stereo: false  # Must be false for monocular
max_cameras: 1     # Only one camera

# Calibration flags
calib_cam_extrinsics: true   # Optimize camera-IMU transform
calib_cam_intrinsics: true   # Optimize camera intrinsics
calib_cam_timeoffset: true   # Optimize time offset
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false

# State vector configuration
max_clones: 11       # Sliding window size
max_slam: 50         # SLAM features
max_msckf_in_update: 40

# Gravity magnitude (adjust for your location)
gravity_mag: 9.81

# Feature representation
feat_rep_msckf: "GLOBAL_3D"
feat_rep_slam: "ANCHORED_MSCKF_INVERSE_DEPTH"

# Initialization parameters
init_window_time: 2.0
init_imu_thresh: 1.5
init_max_disparity: 10.0
init_max_features: 50

# Dynamic initialization (recommended for UAVs)
init_dyn_use: true
init_dyn_mle_opt_calib: false
init_dyn_mle_max_iter: 50
init_dyn_num_pose: 6
init_dyn_min_deg: 10.0

# Feature tracking
use_klt: true
num_pts: 200              # Number of features to track
fast_threshold: 20        # FAST corner threshold
grid_x: 5                 # Grid for uniform feature distribution
grid_y: 5
min_px_dist: 10          # Minimum pixel distance between features
track_frequency: 21.0    # Tracking frequency (Hz)
downsample_cameras: false
num_opencv_threads: 4
histogram_method: "HISTOGRAM"  # NONE, HISTOGRAM, CLAHE

# Update parameters
up_msckf_sigma_px: 1
up_msckf_chi2_multipler: 1

# Reference to other config files
relative_config_imu: "kalibr_imu_chain.yaml"
relative_config_imucam: "kalibr_imucam_chain.yaml"
```

### 2. kalibr_imu_chain.yaml

IMU configuration and noise parameters:

```yaml
%YAML:1.0

imu0:
  # Transform from IMU to body frame (usually identity)
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]

  # IMU noise parameters (check your IMU datasheet)
  accelerometer_noise_density: 2.0000e-3   # m/s^2/sqrt(Hz)
  accelerometer_random_walk: 3.0000e-3     # m/s^3/sqrt(Hz)
  gyroscope_noise_density: 1.6968e-04      # rad/s/sqrt(Hz)
  gyroscope_random_walk: 1.9393e-05        # rad/s^2/sqrt(Hz)

  # YOUR IMU TOPIC HERE
  rostopic: /your/imu/topic

  time_offset: 0.0
  update_rate: 200.0  # IMU frequency in Hz

  # IMU model: "calibrated", "kalibr", or "rpng"
  model: "kalibr"

  # Intrinsic calibration matrices (usually identity if not calibrated)
  Tw:  # Gyroscope scale and misalignment
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  R_IMUtoGYRO:  # Gyroscope to IMU frame
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  Ta:  # Accelerometer scale and misalignment
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  R_IMUtoACC:  # Accelerometer to IMU frame
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  Tg:  # Gyroscope g-sensitivity
    - [0.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0]
    - [0.0, 0.0, 0.0]
```

### 3. kalibr_imucam_chain.yaml

Camera configuration and calibration:

```yaml
%YAML:1.0

cam0:
  # Transformation from camera to IMU frame
  # This is T_cam_imu or T_imu_cam depending on convention
  # Format: [R | t] where R is 3x3 rotation, t is translation
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.0]    # If you don't know, start with identity
    - [0.0, 1.0, 0.0, 0.0]    # and enable calib_cam_extrinsics: true
    - [0.0, 0.0, 1.0, 0.0]    # OpenVINS will optimize it
    - [0.0, 0.0, 0.0, 1.0]

  # Overlapping cameras (empty for monocular)
  cam_overlaps: []

  # Camera model (pinhole, fisheye, etc.)
  camera_model: pinhole

  # Distortion model and coefficients
  distortion_model: radtan  # or equidistant for fisheye
  distortion_coeffs: [k1, k2, p1, p2]  # Replace with your values
  # Example: [0.0, 0.0, 0.0, 0.0] for no distortion
  # Example: [-0.28, 0.07, 0.0, 0.0] for typical radtan

  # Camera intrinsics [fx, fy, cx, cy]
  intrinsics: [fx, fy, cx, cy]  # Replace with your values
  # fx, fy: focal lengths in pixels
  # cx, cy: principal point (usually image center)
  # Example: [458.654, 457.296, 367.215, 248.375]

  # Image resolution [width, height]
  resolution: [width, height]  # Replace with your values
  # Example: [752, 480]

  # YOUR CAMERA IMAGE TOPIC HERE
  rostopic: /your/camera/image/topic

  # Time offset between camera and IMU (will be optimized if enabled)
  timeshift_cam_imu: 0.0
```

---

## Extracting Camera Calibration

### Method 1: From camera_info Topic

If your camera publishes a camera_info topic:

```bash
# Echo the camera_info topic
ros2 topic echo /your/camera/camera_info --once
```

Extract these values from the output:

```yaml
# From camera_info message:
k: [fx, 0, cx, 0, fy, cy, 0, 0, 1]  # 3x3 matrix in row-major order
d: [k1, k2, p1, p2, k3]              # Distortion coefficients
width: 640
height: 480

# Convert to OpenVINS format:
intrinsics: [fx, fy, cx, cy]
distortion_coeffs: [k1, k2, p1, p2]  # Usually just first 4
resolution: [width, height]
```

### Method 2: Using ROS2 Camera Calibration

If you need to calibrate your camera:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.108 \
  image:=/your/camera/image \
  camera:=/your/camera
```

This will generate a calibration file with intrinsics and distortion parameters.

### Method 3: Using Kalibr

For full camera-IMU calibration (recommended):

```bash
# Install Kalibr (if not already installed)
# Follow: https://github.com/ethz-asl/kalibr

# Record a calibration dataset
ros2 bag record -o calib_data \
  /your/camera/image \
  /your/imu/topic

# Run Kalibr calibration
kalibr_calibrate_imu_camera \
  --bag calib_data.bag \
  --cam camchain.yaml \
  --imu imu.yaml \
  --target target.yaml
```

Kalibr will output both camera intrinsics and camera-IMU extrinsics.

---

## Launch Options

### Available Launch Arguments

```bash
ros2 launch ov_msckf subscribe.launch.py --show-args
```

Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config` | `euroc_mav` | Config folder name |
| `config_path` | (auto) | Full path to estimator_config.yaml |
| `topic_imu` | `/imu0` | IMU topic (overrides YAML) |
| `topic_camera0` | `/cam0/image_raw` | Camera topic (overrides YAML) |
| `max_cameras` | `2` | Number of cameras (set to 1) |
| `use_stereo` | `true` | Use stereo (set to false) |
| `verbosity` | `INFO` | Log level |
| `ov_enable` | `true` | Enable OpenVINS node |
| `rviz_enable` | `false` | Enable RViz visualization |
| `repub_enable` | `true` | Enable odometry republisher |

### Example Launch Commands

**Basic monocular launch:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false
```

**With custom topics:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  topic_imu:=/mavros/imu/data \
  topic_camera0:=/camera/image_raw \
  max_cameras:=1 \
  use_stereo:=false
```

**With RViz visualization:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false \
  rviz_enable:=true
```

**Debug mode:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false \
  verbosity:=DEBUG
```

**With custom config path:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/full/path/to/my_config/estimator_config.yaml \
  max_cameras:=1 \
  use_stereo:=false
```

---

## Troubleshooting

### Issue: "No IMU data received"

**Solution:**
1. Check topic is publishing: `ros2 topic hz /your/imu/topic`
2. Verify topic name in `kalibr_imu_chain.yaml` matches actual topic
3. Check IMU message type is `sensor_msgs/msg/Imu`
4. Ensure IMU frequency is high enough (>100 Hz recommended)

### Issue: "No image data received"

**Solution:**
1. Check topic: `ros2 topic hz /your/camera/image`
2. Verify topic name in `kalibr_imucam_chain.yaml`
3. Check message type is `sensor_msgs/msg/Image`
4. Verify image encoding is supported (mono8, bgr8, rgb8)

### Issue: "Initialization failed"

**Possible causes:**
1. **Insufficient motion** - Move the camera/UAV with rotation and translation
2. **Poor lighting** - Ensure scene has sufficient features
3. **Camera not calibrated** - Check intrinsics are correct
4. **High IMU noise** - Tune noise parameters in `kalibr_imu_chain.yaml`

**Solution:**
```yaml
# In estimator_config.yaml, try:
init_dyn_use: true           # Enable dynamic initialization
init_dyn_min_deg: 5.0        # Reduce minimum rotation (was 10.0)
init_max_features: 100       # Increase features (was 50)
init_window_time: 3.0        # Increase window (was 2.0)
```

### Issue: "Tracking lost"

**Solution:**
1. Increase number of tracked features:
```yaml
num_pts: 300  # Increase from 200
```

2. Adjust feature detection threshold:
```yaml
fast_threshold: 15  # Lower = more features (was 20)
```

3. Enable histogram equalization for better lighting:
```yaml
histogram_method: "CLAHE"  # Better than HISTOGRAM
```

### Issue: "Drift over time"

**Solution:**
1. Improve camera-IMU calibration (use Kalibr)
2. Enable online calibration:
```yaml
calib_cam_extrinsics: true
calib_cam_intrinsics: true
calib_cam_timeoffset: true
```
3. Add more SLAM features:
```yaml
max_slam: 100  # Increase from 50
```

### Issue: "Coordinate frame issues"

**Check:**
1. IMU coordinate frame convention (ENU, NED, FLU, FRD)
2. Camera coordinate frame (OpenCV: Z forward, Y down, X right)
3. Transformation between IMU and camera should account for these

---

## Advanced Topics

### Time Synchronization

For best results, camera and IMU should be hardware-synchronized. If not:

```yaml
# In kalibr_imucam_chain.yaml
timeshift_cam_imu: 0.0  # Will be optimized if calib_cam_timeoffset: true
```

### IMU Noise Parameters

Get accurate values from your IMU datasheet:

| Parameter | Typical Values | Units |
|-----------|---------------|-------|
| `accelerometer_noise_density` | 0.001 - 0.01 | m/s²/√Hz |
| `accelerometer_random_walk` | 0.001 - 0.01 | m/s³/√Hz |
| `gyroscope_noise_density` | 0.0001 - 0.001 | rad/s/√Hz |
| `gyroscope_random_walk` | 0.00001 - 0.0001 | rad/s²/√Hz |

### Coordinate Frames

OpenVINS conventions:
- **IMU frame**: Typically body frame (Forward-Right-Down or Forward-Left-Up)
- **Camera frame**: OpenCV convention (Z forward, Y down, X right)
- **Global frame**: ENU (East-North-Up) or gravity-aligned

### Multi-Camera Setup (Future)

If you later add more cameras:

```yaml
# In estimator_config.yaml
max_cameras: 2
use_stereo: true  # If cameras overlap

# In kalibr_imucam_chain.yaml
cam0:
  cam_overlaps: [1]  # Overlaps with cam1
  # ...

cam1:
  cam_overlaps: [0]  # Overlaps with cam0
  # ...
```

---

## Published Topics

OpenVINS publishes these topics (in namespace `ov_msckf`):

| Topic | Type | Description |
|-------|------|-------------|
| `/ov_msckf/poseimu` | `PoseWithCovarianceStamped` | IMU pose |
| `/ov_msckf/odomimu` | `Odometry` | IMU odometry |
| `/ov_msckf/pathimu` | `Path` | Trajectory path |
| `/ov_msckf/points_msckf` | `PointCloud` | MSCKF features |
| `/ov_msckf/points_slam` | `PointCloud` | SLAM landmarks |
| `/ov_msckf/trackhist` | `Image` | Feature tracks visualization |

---

## Performance Tuning

### For Real-time Performance

```yaml
# Reduce computational load
num_pts: 150                # Fewer features
max_msckf_in_update: 30    # Smaller updates
downsample_cameras: true   # Downsample images
num_opencv_threads: 4      # Use multi-threading
```

### For Better Accuracy

```yaml
# More features and longer window
num_pts: 300
max_clones: 15
max_slam: 100
max_msckf_in_update: 50
```

---

## References

- [OpenVINS Documentation](https://docs.openvins.com/)
- [OpenVINS GitHub](https://github.com/rpng/open_vins)
- [Kalibr Calibration](https://github.com/ethz-asl/kalibr)
- [ROS2 Camera Calibration](http://wiki.ros.org/camera_calibration)

---

## Example: Complete Configuration for DJI-style UAV

```bash
# Topics typically used by DJI UAVs
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_dji_uav \
  topic_imu:=/dji_osdk_ros/imu \
  topic_camera0:=/dji_osdk_ros/main_camera_images \
  max_cameras:=1 \
  use_stereo:=false \
  verbosity:=INFO
```

---

## Support

If you encounter issues:

1. Check OpenVINS logs: Look for ERROR or WARNING messages
2. Verify topics: `ros2 topic list` and `ros2 topic echo`
3. Test with example datasets first (EuroC, TUM-VI)
4. Review OpenVINS documentation: https://docs.openvins.com/

---

**Last Updated**: 2026-02-06
**OpenVINS Version**: ROS2 Jazzy
