# OpenVINS Integration with Isaac Sim

This document describes how to run OpenVINS with Isaac Sim simulation.

## Prerequisites

1. Isaac Sim must be running and publishing the following topics:
   - `/pegasus_1/rgb` - RGB camera images
   - `/pegasus_1/camera_info` - Camera calibration info
   - `/telemetry/imu` - IMU data

## Quick Start

### 1. Verify Isaac Sim Topics

Before launching OpenVINS, verify that Isaac Sim is publishing the required topics:

```bash
ros2 topic list | grep -E "(pegasus_1|telemetry)"
```

You should see:
- `/pegasus_1/rgb`
- `/pegasus_1/camera_info`
- `/telemetry/imu`

### 2. Check Topic Rates

Ensure topics are publishing at reasonable rates:

```bash
ros2 topic hz /pegasus_1/rgb
ros2 topic hz /telemetry/imu
```

Recommended rates:
- Camera: 20-30 Hz
- IMU: 100+ Hz

### 3. Launch OpenVINS

Use the provided launch script:

```bash
cd /home/huojiaxi/Desktop/uav_perception/jazzy_openVINS
./launch_isaac_sim.sh
```

Or manually:

```bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
    config:=isaac_sim \
    max_cameras:=1 \
    use_stereo:=false \
    verbosity:=INFO \
    rviz_enable:=true
```

## Configuration Details

The Isaac Sim configuration is located at:
- `src/open_vins_ros2_jazzy/config/isaac_sim/`

### Files:
1. **estimator_config.yaml** - Main OpenVINS parameters
   - Monocular camera setup
   - Feature tracking parameters optimized for simulation
   - Online extrinsic calibration enabled

2. **kalibr_imu_chain.yaml** - IMU configuration
   - Topic: `/telemetry/imu`
   - Noise parameters tuned for simulation
   - Update rate: 100 Hz

3. **kalibr_imucam_chain.yaml** - Camera-IMU calibration
   - Topic: `/pegasus_1/rgb`
   - Resolution: 1280x720
   - Intrinsics from Isaac Sim: fx=fy=3054.16, cx=640, cy=360
   - No distortion (simulation)
   - Initial extrinsics (will be calibrated online)

## Tuning

### If initialization fails:

1. **Reduce motion requirement:**
   Edit `estimator_config.yaml`:
   ```yaml
   init_dyn_min_deg: 5.0  # reduce from 10.0
   ```

2. **Adjust feature tracking:**
   ```yaml
   num_pts: 150           # reduce from 200
   fast_threshold: 15     # reduce from 20
   ```

### If tracking is unstable:

1. **Increase feature count:**
   ```yaml
   num_pts: 250
   ```

2. **Adjust noise parameters:**
   Edit `kalibr_imu_chain.yaml` to match your simulation's noise characteristics

### Camera-IMU extrinsics:

The initial extrinsics in `kalibr_imucam_chain.yaml` assume:
- Camera faces forward (along body x-axis)
- Camera is aligned with IMU frame

If your Isaac Sim setup is different, update the `T_imu_cam` transformation matrix.

## Output Topics

OpenVINS will publish:
- `/ov_msckf/odometry` - Visual-inertial odometry
- `/ov_msckf/path_imu` - Trajectory path
- `/ov_msckf/points_slam` - SLAM features
- `/ov_msckf/loop_pose` - Loop closure poses

## Troubleshooting

### No images received:
```bash
ros2 topic echo /pegasus_1/rgb --once
```

### IMU not updating:
```bash
ros2 topic echo /telemetry/imu
```

### Check OpenVINS status:
Look for initialization messages in the terminal. OpenVINS should print:
```
[INFO] - Waiting for IMU...
[INFO] - Waiting for camera...
[INFO] - INITIALIZED!
```

### Coordinate frame issues:

If the estimated trajectory looks wrong, check the coordinate frames:
```bash
ros2 run tf2_tools view_frames
```

Compare Isaac Sim's coordinate system with OpenVINS expectations.

## Advanced Usage

### Record a bag for offline processing:
```bash
ros2 bag record /pegasus_1/rgb /pegasus_1/camera_info /telemetry/imu
```

### Visualize in RViz:
RViz will automatically launch with the provided script. You can customize the RViz config at:
`install/ov_msckf/share/ov_msckf/launch/display_ros2.rviz`

### Disable online calibration:
If you have accurate extrinsics, edit `estimator_config.yaml`:
```yaml
calib_cam_extrinsics: false
```
