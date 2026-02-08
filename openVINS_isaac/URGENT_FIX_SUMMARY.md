# 🚨 URGENT: Root Cause Found!

**Problem:** "Estimation goes mad when UAV moves"

**Root Cause:** **Camera resolution and intrinsics mismatch!**

---

## What I Found

### 1. Critical Issue: Resolution Mismatch 🔴

```
Actual camera:      1280 x 720 pixels
OpenVINS expects:   1920 x 1200 pixels
Intrinsics:         Calibrated for 1920x1200 (WRONG!)
```

**Result:** OpenVINS tracks features at COMPLETELY WRONG pixel locations!
- When UAV moves, features appear in wrong places
- Reprojection errors are huge
- Filter diverges → "goes mad"

### 2. Confirmed: IMU Data is OK ✅

```
IMU angular velocity: ~0.001 rad/s (good)
IMU acceleration Z: 9.86 m/s² (correct gravity)
IMU units: Correct (m/s², rad/s)
```

---

## The Fix (Already Created!)

I created `config/pegasus_fixed/` with corrected settings:

**Changes:**
```yaml
# Before (WRONG):
resolution: [1920, 1200]
intrinsics: [4581.2456, 4581.2456, 957.8, 589.5]

# After (CORRECT):
resolution: [1280, 720]
intrinsics: [3054.1637, 2748.7474, 638.5333, 353.7000]  # Scaled!
```

---

## Test It Now! (2 minutes)

```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac

# Rebuild
colcon build --packages-select ov_msckf
source install/setup.bash

# Launch with FIXED config
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_fixed \
  use_stereo:=false \
  max_cameras:=1 \
  verbosity:=DEBUG
```

Then fly the UAV and move it - estimation should now follow motion correctly!

---

## Why This Was The Problem

**Before (wrong resolution):**
```
UAV moves → Features move in camera
           ↓
OpenVINS calculates where features should be using WRONG intrinsics
           ↓
Predicted location: pixel (1500, 800)  [assuming 1920x1200]
Actual location:    pixel (1000, 533)  [actual 1280x720]
           ↓
HUGE error → Filter thinks motion is completely wrong
           ↓
"Estimation goes mad" ❌
```

**After (correct resolution):**
```
UAV moves → Features move in camera
           ↓
OpenVINS calculates using CORRECT intrinsics
           ↓
Predicted location: pixel (1000, 533)
Actual location:    pixel (1000, 533)
           ↓
Small error → Filter converges
           ↓
Smooth tracking ✅
```

---

## Expected Result

After fix:
- ✅ Initialization should work
- ✅ Trajectory should follow UAV motion smoothly
- ✅ No "goes mad" behavior during motion
- ✅ Feature tracking should be stable

---

## Files Created

1. **`config/pegasus_fixed/`** - Configuration with corrected resolution/intrinsics ← **USE THIS!**
2. **`IMU_CAMERA_ISSUES_FOUND.md`** - Detailed analysis
3. **`URGENT_FIX_SUMMARY.md`** - This file

---

## Next Steps

1. ✅ **Test pegasus_fixed config** (do this now!)
2. ⬜ If still issues, verify T_imu_cam transform (see IMU_CAMERA_ISSUES_FOUND.md)
3. ⬜ After motion works, re-apply ZUPT fixes from DRIFT_ANALYSIS.md for hovering

---

**Priority:** Test the fixed config immediately - this should solve the "goes mad" issue!

