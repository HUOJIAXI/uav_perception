# UAV Template Configuration

This is a **template configuration** for OpenVINS with a monocular camera and IMU.

## How to Use This Template

### Step 1: Copy This Template

```bash
cd ~/Desktop/uav_perception/jazzy_openVINS/src/open_vins_ros2_jazzy/config
cp -r uav_template my_uav_name
cd my_uav_name
```

### Step 2: Extract Your Camera Calibration

Use the helper script to extract calibration from your camera_info topic:

```bash
cd ~/Desktop/uav_perception/jazzy_openVINS
./extract_camera_calibration.sh /your/camera/camera_info
```

Copy the output into `kalibr_imucam_chain.yaml`.

### Step 3: Edit Configuration Files

#### `kalibr_imu_chain.yaml`
1. Set `rostopic:` to your IMU topic (e.g., `/mavros/imu/data`)
2. Update `update_rate:` to match your IMU frequency
3. Set noise parameters from your IMU datasheet (see comments in file)

#### `kalibr_imucam_chain.yaml`
1. Set `rostopic:` to your camera IMAGE topic (e.g., `/camera/image_raw`)
2. Update `intrinsics:`, `distortion_coeffs:`, and `resolution:` with your values
3. If you know camera-IMU transformation, update `T_cam_imu:`

#### `estimator_config.yaml`
1. Review and adjust parameters if needed
2. Most default values should work for typical UAV applications

### Step 4: Launch OpenVINS

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav_name \
  max_cameras:=1 \
  use_stereo:=false
```

## Important Notes

- **OpenVINS does NOT use camera_info topics** - all calibration must be in the YAML files
- Start with `calib_cam_extrinsics: true` if you don't know the camera-IMU transformation
- Ensure your IMU publishes at >100 Hz for good performance
- Move the UAV with rotation and translation during initialization

## Troubleshooting

See the main [UAV_SETUP_GUIDE.md](../../../../UAV_SETUP_GUIDE.md) for detailed troubleshooting.

Quick checks:
```bash
# Verify topics are publishing
ros2 topic hz /your/imu/topic
ros2 topic hz /your/camera/image

# Check message types
ros2 topic info /your/imu/topic
ros2 topic info /your/camera/image
```

## Files in This Template

- `estimator_config.yaml` - Main estimator configuration
- `kalibr_imu_chain.yaml` - IMU parameters and topic
- `kalibr_imucam_chain.yaml` - Camera parameters and topic
- `README.md` - This file
