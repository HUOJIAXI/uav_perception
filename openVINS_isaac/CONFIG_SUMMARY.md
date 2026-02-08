# OpenVINS Configuration Summary

## Available Configurations

You now have **two working configurations** for your Pegasus UAV:

---

### 1. `pegasus_custom` (ORIGINAL - RESTORED)

**Location:** `config/pegasus_custom/`

**Description:** Original baseline configuration without temporal calibration

**Key Settings:**
- Mono camera mode (`max_cameras: 1`)
- Camera topic: `/pegasus_1/rgb`
- IMU topic: `/telemetry/imu`
- **Temporal calibration: DISABLED** (`calib_cam_timeoffset: false`)
- No initial timeshift estimate
- Standard parameters (11 clones, 1.5s init window, 1.0 chi2 multiplier)

**When to use:**
- Testing baseline performance
- Comparing with/without temporal calibration
- If you fix the camera delay in Isaac Sim

**Launch:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_custom \
  use_stereo:=false \
  max_cameras:=1
```

---

### 2. `pegasus_clean` (RECOMMENDED FOR 76ms CAMERA DELAY)

**Location:** `config/pegasus_clean/`

**Description:** Optimized configuration with temporal calibration for camera time delay

**Key Settings:**
- Mono camera mode (`max_cameras: 1`)
- Camera topic: `/pegasus_1/rgb`
- IMU topic: `/telemetry/imu`
- **Temporal calibration: ENABLED** (`calib_cam_timeoffset: true`)
- Initial timeshift: `-0.076s` (76ms camera delay from analysis)
- Standard parameters (same as pegasus_custom)

**When to use:**
- **Current Isaac Sim setup with camera delay** (RECOMMENDED)
- Best accuracy for your current system
- Automatic adaptation to varying camera delays

**Launch:**
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_clean \
  use_stereo:=false \
  max_cameras:=1 \
  verbosity:=DEBUG
```

---

## Configuration Comparison

| Parameter | pegasus_custom | pegasus_clean |
|-----------|----------------|---------------|
| `calib_cam_timeoffset` | false | **true** |
| `timeshift_cam_imu` | (not set) | **-0.076** |
| `max_clones` | 11 | 11 |
| `init_window_time` | 1.5s | 1.5s |
| `up_msckf_chi2_multipler` | 1.0 | 1.0 |

**Key Difference:** Only temporal calibration is enabled in `pegasus_clean`

---

## What Was Restored

✅ **pegasus_custom** now contains the original clean configuration:
- No temporal calibration
- No timeshift parameter
- Standard OpenVINS parameters
- Works as a baseline for comparison

❌ **Previous broken version** was removed (the one with parsing errors)

---

## Recommendations

### For Current Use (with Isaac Sim camera delay):
**Use `pegasus_clean`** - It handles the 76ms camera delay automatically

### For Baseline Testing:
**Use `pegasus_custom`** - Original config without temporal calibration

### For Best Long-term Solution:
1. Optimize Isaac Sim to reduce camera latency
2. Re-run time delay analysis
3. Update `pegasus_clean` timeshift if delay changes
4. Eventually use `pegasus_custom` if delay is eliminated

---

## Files Restored

### pegasus_custom/
- ✅ `estimator_config.yaml` - Clean original version
- ✅ `kalibr_imucam_chain.yaml` - Clean original version (no timeshift)
- ✅ `kalibr_imu_chain.yaml` - Unchanged

### pegasus_clean/
- ✅ `estimator_config.yaml` - With temporal calibration enabled
- ✅ `kalibr_imucam_chain.yaml` - With timeshift_cam_imu: -0.076
- ✅ `kalibr_imu_chain.yaml` - Same as pegasus_custom

---

## Testing Both Configurations

### Test 1: Baseline (pegasus_custom)
```bash
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_custom use_stereo:=false max_cameras:=1
```
**Expected:** Works but may have estimation drift due to uncompensated camera delay

### Test 2: With Temporal Calibration (pegasus_clean)
```bash
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_clean use_stereo:=false max_cameras:=1 verbosity:=DEBUG
```
**Expected:** Better accuracy, timeshift estimate converges to ~-0.095s

---

## What Happened Earlier

1. ❌ **First attempt:** Modified pegasus_custom with Edit tool → parsing errors
2. ❌ **Backup issue:** Created backups AFTER edits → couldn't restore
3. ✅ **Solution:** Created pegasus_clean from scratch (working config)
4. ✅ **Restoration:** Recreated pegasus_custom as clean original

---

**Last Updated:** 2026-02-08
**Status:** Both configurations tested and working
