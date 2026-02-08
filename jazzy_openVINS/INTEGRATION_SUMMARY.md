# OpenVINS ↔ Isaac Sim Integration Summary

## ✅ What Was Done

### 1. **Analyzed Isaac Sim Topics**
Discovered the simulation publishes:
- `/pegasus_1/rgb` (sensor_msgs/Image) @ ~20 Hz
- `/pegasus_1/camera_info` (sensor_msgs/CameraInfo)
- `/telemetry/imu` (sensor_msgs/Imu) @ 100 Hz

### 2. **Created Custom Configuration**
Created `isaac_sim` config package at:
```
src/open_vins_ros2_jazzy/config/isaac_sim/
├── estimator_config.yaml       # Main OpenVINS parameters
├── kalibr_imu_chain.yaml       # IMU configuration
└── kalibr_imucam_chain.yaml    # Camera-IMU calibration
```

**Key Configuration Details:**
- **Monocular Setup** (1 camera)
- **Resolution:** 1280 x 720
- **Intrinsics:** fx=fy=3054.16, cx=640, cy=360
- **Distortion:** None (simulation)
- **IMU Topic:** `/telemetry/imu`
- **Camera Topic:** `/pegasus_1/rgb`
- **Online Extrinsic Calibration:** Enabled

### 3. **Built and Installed**
Rebuilt the package with:
```bash
colcon build --packages-select ov_msckf --symlink-install
```

Config installed to:
```
install/ov_msckf/share/ov_msckf/config/isaac_sim/
```

### 4. **Created Launch Infrastructure**
- `launch_isaac_sim.sh` - One-command launch script
- `ISAAC_SIM_SETUP.md` - Detailed setup and tuning guide
- `README_QUICKSTART.md` - Quick reference guide

## 🚀 How to Use

### Simple Launch:
```bash
./launch_isaac_sim.sh
```

### Manual Launch:
```bash
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py \
    config:=isaac_sim \
    max_cameras:=1 \
    use_stereo:=false \
    verbosity:=INFO \
    rviz_enable:=true
```

## 📊 Expected Behavior

### Initialization Sequence:
1. OpenVINS starts, waits for sensor data
2. Receives IMU and camera data
3. **Requires drone motion** (~10° rotation) to initialize
4. Prints: `[INFO] - INITIALIZED!`
5. Begins publishing odometry

### Output Topics:
- `/ov_msckf/odometry` - Main VIO output
- `/ov_msckf/path_imu` - Trajectory
- `/ov_msckf/points_slam` - SLAM landmarks
- `/ov_msckf/loop_pose` - Loop closures

## 🔧 Tuning Parameters

### If initialization fails:
Edit `config/isaac_sim/estimator_config.yaml`:
```yaml
init_dyn_min_deg: 5.0  # Reduce motion requirement
```

### If tracking quality is poor:
```yaml
num_pts: 250          # Increase feature count
fast_threshold: 15    # Reduce detection threshold
```

### Camera-IMU Extrinsics:
Edit `config/isaac_sim/kalibr_imucam_chain.yaml`:
```yaml
T_imu_cam:  # Transformation matrix [R | t]
```

Current assumption: Camera faces forward, aligned with IMU

## 📁 File Structure

```
jazzy_openVINS/
├── launch_isaac_sim.sh              ← Launch script
├── README_QUICKSTART.md             ← Quick start
├── ISAAC_SIM_SETUP.md               ← Detailed guide
├── INTEGRATION_SUMMARY.md           ← This file
└── src/open_vins_ros2_jazzy/config/isaac_sim/
    ├── estimator_config.yaml
    ├── kalibr_imu_chain.yaml
    └── kalibr_imucam_chain.yaml
```

## ✅ Pre-Flight Checklist

Before launching OpenVINS:

- [ ] Isaac Sim is running
- [ ] `/pegasus_1/rgb` is publishing (~20 Hz)
- [ ] `/telemetry/imu` is publishing (~100 Hz)
- [ ] `/pegasus_1/camera_info` is available
- [ ] Workspace is sourced: `source install/setup.bash`

Check with:
```bash
ros2 topic hz /pegasus_1/rgb
ros2 topic hz /telemetry/imu
```

## 🎯 Next Steps

1. **Test the integration:**
   ```bash
   ./launch_isaac_sim.sh
   ```

2. **Fly the drone** in Isaac Sim to trigger initialization

3. **Monitor the output:**
   ```bash
   ros2 topic echo /ov_msckf/odometry
   ```

4. **Fine-tune** if needed (see `ISAAC_SIM_SETUP.md`)

5. **Record data** for analysis:
   ```bash
   ros2 bag record -a
   ```

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| No initialization | Increase drone motion, reduce `init_dyn_min_deg` |
| Poor tracking | Increase `num_pts`, adjust `fast_threshold` |
| Wrong trajectory | Check camera-IMU extrinsics `T_imu_cam` |
| No camera data | Verify `/pegasus_1/rgb` topic |
| No IMU data | Verify `/telemetry/imu` topic |

## 📚 References

- OpenVINS Documentation: https://docs.openvins.com/
- Launch file: `src/open_vins_ros2_jazzy/ov_msckf/launch/subscribe.launch.py`
- Node executable: `install/ov_msckf/lib/ov_msckf/run_subscribe_msckf`

---

**Ready to fly!** 🚁
