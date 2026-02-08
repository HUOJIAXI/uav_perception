# OpenVINS UAV Quick Start

Fast setup guide for monocular camera + IMU configuration.

## 1. Create Your Config (2 minutes)

```bash
cd ~/Desktop/uav_perception/jazzy_openVINS/src/open_vins_ros2_jazzy/config
cp -r rs_d435i_mono my_uav
cd my_uav
```

## 2. Edit 3 Files

### kalibr_imu_chain.yaml
```yaml
imu0:
  rostopic: /your/imu/topic  # CHANGE THIS
  update_rate: 200.0
```

### kalibr_imucam_chain.yaml
```yaml
cam0:
  intrinsics: [fx, fy, cx, cy]      # CHANGE THIS
  distortion_coeffs: [k1, k2, p1, p2]  # CHANGE THIS
  resolution: [width, height]       # CHANGE THIS
  rostopic: /your/camera/image      # CHANGE THIS
```

### estimator_config.yaml
```yaml
max_cameras: 1
use_stereo: false
```

## 3. Launch

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false
```

---

## Quick Commands

### Extract Camera Calibration
```bash
ros2 topic echo /your/camera/camera_info --once
```

### Check Topics
```bash
ros2 topic list | grep -E "imu|camera"
ros2 topic hz /your/imu/topic
ros2 topic hz /your/camera/image
```

### Launch with RViz
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false \
  rviz_enable:=true
```

### Debug Mode
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=my_uav \
  max_cameras:=1 \
  use_stereo:=false \
  verbosity:=DEBUG
```

---

## Typical UAV Topics

| UAV Type | IMU Topic | Camera Topic |
|----------|-----------|--------------|
| **PX4/MAVROS** | `/mavros/imu/data` | `/camera/image_raw` |
| **DJI** | `/dji_osdk_ros/imu` | `/dji_osdk_ros/main_camera_images` |
| **Custom** | `/uav/imu` | `/uav/camera/image` |

---

## Common Issues

**No IMU data?**
```bash
ros2 topic hz /your/imu/topic
# Check if publishing and >100 Hz
```

**No camera data?**
```bash
ros2 topic hz /your/camera/image
# Check if publishing and type is sensor_msgs/Image
```

**Initialization fails?**
- Move the UAV with rotation and translation
- Ensure good lighting and visible features
- Try `init_dyn_use: true` in estimator_config.yaml

**Need camera calibration?**
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.108 \
  image:=/your/camera/image
```

---

See `UAV_SETUP_GUIDE.md` for complete documentation.
