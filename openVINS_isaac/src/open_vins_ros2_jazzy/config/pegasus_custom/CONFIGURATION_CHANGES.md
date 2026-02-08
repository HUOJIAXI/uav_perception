# OpenVINS Configuration Changes for Isaac Sim Camera Delay

**Date:** 2026-02-08
**Purpose:** Reduce offset between ground truth and estimation by handling 76ms camera time delay
**Based on:** Time delay analysis results

---

## Problem Summary

Time delay analysis revealed:
- **Camera delay:** 75.86 ± 24.46 ms (highly variable, CV = 32.25%)
- **IMU delay:** -0.14 ± 1.95 ms (nearly synchronized)
- **Camera-IMU offset:** ~76 ms
- **Root cause:** Isaac Sim rendering pipeline latency with variable computational load

This delay causes misalignment between visual and inertial measurements, leading to estimation errors.

---

## Configuration Changes Applied

### 1. Temporal Calibration (PRIMARY FIX)

**File:** `estimator_config.yaml`

```yaml
# Line 12: ENABLED temporal calibration
calib_cam_timeoffset: true  # Was: false
```

**Effect:** OpenVINS will now estimate and optimize the time offset between camera and IMU online during operation. This handles the variable 76ms delay automatically.

---

### 2. Initial Time Offset Estimate

**File:** `kalibr_imucam_chain.yaml`

```yaml
# Line 18: Added initial time offset
cam0:
  ...
  timeshift_cam_imu: -0.076  # Camera lags IMU by 76ms
```

**Explanation:**
- Negative value because camera timestamps are 76ms behind true time
- OpenVINS definition: `t_imu = t_cam + timeshift_cam_imu`
- Initial estimate based on time delay analysis
- Will be refined online when `calib_cam_timeoffset: true`

---

### 3. Increased Sliding Window Size

**File:** `estimator_config.yaml`

```yaml
# Line 16: Increased from 11 to 15
max_clones: 15  # Was: 11
```

**Effect:** Maintains more historical poses in the sliding window, which:
- Improves temporal calibration accuracy
- Provides more constraints for time offset estimation
- Better handles variable delays

---

### 4. Extended Initialization Period

**File:** `estimator_config.yaml`

```yaml
# Line 40: Increased from 1.5s to 2.5s
init_window_time: 2.5  # Was: 1.5

# Line 43: Increased from 50 to 75
init_max_features: 75  # Was: 50
```

**Effect:**
- Collects more data before initialization (accounts for delayed camera frames)
- Tracks more features for more robust initialization
- Reduces initialization failures due to time synchronization issues

---

### 5. Delayed SLAM Feature Initialization

**File:** `estimator_config.yaml`

```yaml
# Line 20: Increased from 1s to 2s
dt_slam_delay: 2  # Was: 1
```

**Effect:** Waits longer before initializing SLAM features, allowing temporal calibration to converge first.

---

### 6. Relaxed Measurement Noise Thresholds

**File:** `estimator_config.yaml`

```yaml
# Lines 102-105: Increased from 1.0 to 1.5
up_msckf_sigma_px: 1.5      # Was: 1
up_msckf_chi2_multipler: 1.5  # Was: 1
up_slam_sigma_px: 1.5       # Was: 1
up_slam_chi2_multipler: 1.5   # Was: 1
```

**Effect:**
- More tolerant of measurement residuals caused by time delay variability
- Reduces outlier rejection of valid measurements
- Accounts for ~32% coefficient of variation in camera delay
- Prevents excessive feature rejection

---

## Expected Improvements

### Before Changes:
- Large position/orientation drift
- Poor tracking performance
- Estimation lags ground truth
- Frequent tracking failures

### After Changes:
- **Reduced position error:** Time offset compensation eliminates systematic lag
- **Improved orientation accuracy:** Better synchronization of angular rates
- **Online adaptation:** System continuously refines time offset estimate
- **More robust tracking:** Relaxed thresholds reduce false outlier rejection
- **Better initialization:** Extended window and more features improve startup

---

## How to Test

### 1. Rebuild OpenVINS (if needed)

```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
colcon build --packages-select ov_msckf
source install/setup.bash
```

### 2. Launch with New Configuration

```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_custom \
  verbosity:=DEBUG
```

### 3. Monitor Temporal Calibration

Watch the terminal output for time offset estimates:

```
[DEBUG] Camera-IMU timeshift: -0.0762 ± 0.0023 s
```

The estimate should:
- Start near -0.076s (our initial guess)
- Converge within 10-30 seconds
- Stabilize with low uncertainty (< 0.005s)

### 4. Compare Ground Truth vs Estimation

If you're logging data, check:

```bash
# Position RMSE should decrease significantly
# Orientation error should reduce
# Tracking should be more stable
```

---

## Further Tuning (If Needed)

### If initialization still fails:
```yaml
init_window_time: 3.0          # Increase further
init_max_features: 100         # Track even more features
init_dyn_min_deg: 3.0          # Reduce required motion
```

### If tracking is too conservative:
```yaml
up_msckf_chi2_multipler: 2.0   # Relax further
up_slam_chi2_multipler: 2.0
```

### If tracking is too aggressive (many outliers):
```yaml
up_msckf_sigma_px: 1.2         # Tighten slightly
up_msckf_chi2_multipler: 1.2
```

### If time offset doesn't converge:
```yaml
calib_cam_timeoffset: false    # Disable online calibration
# Then manually tune timeshift_cam_imu in kalibr_imucam_chain.yaml
# Try values from -0.070 to -0.082 based on your specific delay
```

---

## Advanced: Monitor Calibration Convergence

To see detailed calibration info, enable state saving:

```yaml
# In estimator_config.yaml
save_total_state: true
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
```

Then monitor:
```bash
tail -f /tmp/ov_estimate.txt | grep "timeoffset"
```

---

## Troubleshooting

### Issue: Estimation still drifts

**Possible causes:**
1. Time offset estimate not converging
   - Solution: Check DEBUG logs, may need manual tuning
2. Camera delay more variable than measured
   - Solution: Increase `up_msckf_sigma_px` to 2.0
3. Other calibration issues (extrinsics, intrinsics)
   - Solution: Verify camera calibration parameters

### Issue: Poor feature tracking

**Possible causes:**
1. Thresholds too relaxed, accepting outliers
   - Solution: Reduce chi2 multipliers to 1.3
2. Too few features tracked
   - Solution: Increase `num_pts` from 200 to 300
3. Image quality issues
   - Solution: Check if histogram equalization helps (`histogram_method: "CLAHE"`)

### Issue: Initialization takes too long

**Possible causes:**
1. Insufficient motion during init window
   - Solution: Move UAV more during first few seconds
2. Too few features
   - Solution: Already increased to 75, ensure good lighting/texture
3. Window time too long
   - Solution: Reduce back to 2.0s if motion is sufficient

---

## Backup Files

Original configurations backed up as:
- `estimator_config.yaml.backup_YYYYMMDD_HHMMSS`
- `kalibr_imucam_chain.yaml.backup_YYYYMMDD_HHMMSS`

To restore original:
```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac/src/open_vins_ros2_jazzy/config/pegasus_custom
cp estimator_config.yaml.backup_YYYYMMDD_HHMMSS estimator_config.yaml
cp kalibr_imucam_chain.yaml.backup_YYYYMMDD_HHMMSS kalibr_imucam_chain.yaml
```

---

## Related Files

- Time delay analysis: `/home/huojiaxi/Desktop/uav_perception/time_delay_detect/ANALYSIS_RESULTS.md`
- Delay data: `/home/huojiaxi/Desktop/uav_perception/time_delay_detect/time_delay_data.json`
- Visualization: `/home/huojiaxi/Desktop/uav_perception/time_delay_detect/time_delay_analysis_*.png`

---

## Summary of Changes

| Parameter | Old Value | New Value | Reason |
|-----------|-----------|-----------|--------|
| `calib_cam_timeoffset` | false | **true** | Enable online temporal calibration |
| `timeshift_cam_imu` | (none) | **-0.076** | Initial 76ms camera delay estimate |
| `max_clones` | 11 | **15** | More history for time calibration |
| `dt_slam_delay` | 1 | **2** | Allow time offset to converge first |
| `init_window_time` | 1.5 | **2.5** | More init data with delayed camera |
| `init_max_features` | 50 | **75** | More robust initialization |
| `up_msckf_sigma_px` | 1 | **1.5** | Account for delay variability |
| `up_msckf_chi2_multipler` | 1 | **1.5** | Less aggressive outlier rejection |
| `up_slam_sigma_px` | 1 | **1.5** | Account for delay variability |
| `up_slam_chi2_multipler` | 1 | **1.5** | Less aggressive outlier rejection |

---

## Next Steps

1. ✅ Configuration updated
2. ⬜ Rebuild and test OpenVINS
3. ⬜ Verify time offset convergence in logs
4. ⬜ Compare estimation vs ground truth
5. ⬜ Fine-tune parameters based on results
6. ⬜ (Optional) Re-run time delay analysis after Isaac Sim optimization

---

## Notes

- The 76ms delay is significant and will cause noticeable estimation errors if not addressed
- Online temporal calibration is the most robust solution for variable delays
- If you reduce the camera delay in Isaac Sim (recommended), re-run the time delay analysis and update `timeshift_cam_imu` accordingly
- Monitor system performance for first few runs to ensure convergence
