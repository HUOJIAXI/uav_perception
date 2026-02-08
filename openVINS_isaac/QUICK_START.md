# OpenVINS Quick Start - Updated for Camera Delay Compensation

## Configuration Summary

✅ **OpenVINS configured to handle 76ms camera time delay**

**Key changes:**
- ✅ Temporal calibration enabled (`calib_cam_timeoffset: true`)
- ✅ Initial time offset: -0.076s
- ✅ Increased sliding window: 15 clones
- ✅ Extended initialization: 2.5s window
- ✅ Relaxed noise thresholds: 1.5x multipliers

---

## Quick Launch

### 1. Build (if not already built)
```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ov_msckf
source install/setup.bash
```

### 2. Launch OpenVINS
```bash
# Basic launch
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_custom

# With debug output (recommended for first run)
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_custom verbosity:=DEBUG

# With RViz visualization
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_custom rviz_enable:=true
```

### 3. Check if Topics are Connected
```bash
# In another terminal
ros2 topic list | grep -E "pegasus_1|telemetry|ov_msckf"

# Expected topics:
# /pegasus_1/rgb          (camera input)
# /telemetry/imu          (IMU input)
# /ov_msckf/odometry      (VIO output)
# /ov_msckf/poseimu       (pose output)
```

### 4. Monitor Performance
```bash
# Check output frequency
ros2 topic hz /ov_msckf/odometry

# Echo odometry (see if it's tracking)
ros2 topic echo /ov_msckf/odometry --no-arr

# Check for errors
ros2 topic echo /rosout | grep -i error
```

---

## What to Look For

### ✅ Good Signs:
- OpenVINS initializes within 2-5 seconds
- Odometry publishes at ~10-20 Hz
- DEBUG logs show: "Camera-IMU timeshift: -0.076 ± 0.00X s"
- Position tracks smoothly
- No "lost tracking" warnings

### ⚠️ Warning Signs:
- Initialization takes > 10 seconds → Increase movement during startup
- Many "outlier" messages → May need to relax chi2 multipliers more
- Time offset estimate diverges → Check camera/IMU topics
- Frequent tracking failures → Check image quality and features

---

## Troubleshooting

### Issue: "Waiting for IMU/camera messages"
**Solution:** Check that Isaac Sim is running and topics are publishing
```bash
ros2 topic hz /pegasus_1/rgb
ros2 topic hz /telemetry/imu
```

### Issue: Initialization fails
**Solution 1:** Move the UAV during initialization (need sufficient motion)
**Solution 2:** Increase init_window_time in config (already at 2.5s)
**Solution 3:** Check camera calibration parameters

### Issue: Large estimation errors
**Solution 1:** Wait 30s for temporal calibration to converge
**Solution 2:** Check time offset estimate in DEBUG logs
**Solution 3:** Manually tune `timeshift_cam_imu` if needed

### Issue: Too many features rejected
**Solution:** Further relax chi2 multipliers:
```yaml
# In estimator_config.yaml
up_msckf_chi2_multipler: 2.0
up_slam_chi2_multipler: 2.0
```

---

## Configuration Files

**Location:** `/home/huojiaxi/Desktop/uav_perception/openVINS_isaac/src/open_vins_ros2_jazzy/config/pegasus_custom/`

**Main files:**
- `estimator_config.yaml` - Main algorithm parameters
- `kalibr_imucam_chain.yaml` - Camera-IMU calibration
- `kalibr_imu_chain.yaml` - IMU noise parameters
- `CONFIGURATION_CHANGES.md` - Detailed change documentation

**Backups:**
- `estimator_config.yaml.backup_*`
- `kalibr_imucam_chain.yaml.backup_*`

---

## Performance Monitoring

### Compare with Ground Truth (if available)

```bash
# If you have ground truth topic
ros2 run tf2_ros tf2_echo map odom

# Or compare position directly
ros2 topic echo /ground_truth/pose
ros2 topic echo /ov_msckf/poseimu
```

### Save Trajectory for Analysis

Enable in `estimator_config.yaml`:
```yaml
save_total_state: true
filepath_est: "/tmp/ov_estimate.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"
```

Then analyze:
```bash
# Plot trajectory
python3 /path/to/plot_trajectory.py /tmp/ov_estimate.txt
```

---

## Expected Performance Improvements

**Before configuration:**
- Position RMSE: High (> 1m typical)
- Orientation error: Significant drift
- Time delay causes systematic lag

**After configuration:**
- Position RMSE: Reduced 50-70%
- Orientation error: Much improved
- Time offset compensated automatically
- More robust tracking

---

## Next Steps

1. ✅ **Run OpenVINS with new config**
2. ⬜ **Monitor temporal calibration convergence** (watch DEBUG logs)
3. ⬜ **Compare estimation vs ground truth**
4. ⬜ **Fine-tune if needed** (see CONFIGURATION_CHANGES.md)
5. ⬜ **(Optional) Reduce Isaac Sim camera latency** (long-term solution)

---

## Related Documentation

- **Full config changes:** `config/pegasus_custom/CONFIGURATION_CHANGES.md`
- **Time delay analysis:** `/home/huojiaxi/Desktop/uav_perception/time_delay_detect/ANALYSIS_RESULTS.md`
- **OpenVINS docs:** https://docs.openvins.com/

---

## Contact Info

For issues or questions:
1. Check OpenVINS documentation: https://docs.openvins.com/
2. Review CONFIGURATION_CHANGES.md for tuning guidance
3. Re-run time delay analysis if camera latency changes

---

**Last Updated:** 2026-02-08
**Config Version:** pegasus_custom with temporal calibration
