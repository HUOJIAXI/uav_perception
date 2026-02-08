# Action Plan: Fix Unstable OpenVINS Estimation

**Current Status:** Estimation still unstable after resolution fix
**Root Cause:** Likely **wrong T_imu_cam transform**

---

## What I Fixed So Far

### ✅ Camera Intrinsics (CORRECTED)

Updated **`config/pegasus_fixed/`** with ACTUAL values from `/pegasus_1/camera_info`:

```yaml
resolution: [1280, 720]                      # Was: [1920, 1200]
intrinsics: [3054.1637, 3054.1637, 640.0, 360.0]  # Was: [4581..., 957.8, 589.5]
distortion_coeffs: [0.0, 0.0, 0.0, 0.0]      # Was: [0.14, -0.03, ...]
```

**This was critical** - wrong intrinsics cause massive feature tracking errors!

### ⚠️ Camera-IMU Transform (NEEDS TESTING)

Current setting: **Identity matrix** (assumes camera = IMU frame)
```yaml
T_imu_cam: [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
```

**Problem:** This is probably WRONG for Isaac Sim!

---

## Next Steps (Do These Now!)

### Step 1: Rebuild with Fixed Intrinsics (2 min)

```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
colcon build --packages-select ov_msckf
source install/setup.bash
```

---

### Step 2: Test Transform Options (5-10 min per test)

I created **3 test configurations** with different T_imu_cam transforms:

#### Test A: pegasus_test_1 (Identity - current assumption)
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_test_1 \
  verbosity:=DEBUG
```

**Try this first. If estimation explodes or drifts badly, try Test B.**

---

#### Test B: pegasus_test_2 (Front-facing optical frame)
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_test_2 \
  verbosity:=DEBUG
```

**This assumes camera follows ROS optical frame convention (X=right, Y=down, Z=forward).**

---

#### Test C: pegasus_test_3 (90° yaw rotation)
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_test_3 \
  verbosity:=DEBUG
```

**This assumes camera is rotated 90° around vertical axis.**

---

### Step 3: Simple Test Procedure

For each configuration:

1. **Start OpenVINS** with test config
2. **Keep UAV stationary** for initialization (2 seconds)
3. **Perform SLOW YAW ROTATION** (rotate around Z-axis only)
4. **Watch OpenVINS output**:
   ```bash
   # In another terminal:
   source install/setup.bash
   ros2 topic echo /ov_msckf/poseimu | grep -A 4 "orientation:"
   ```

5. **Check result:**
   - ✅ **GOOD:** Estimated rotation follows actual rotation smoothly
   - ❌ **BAD:** Estimation jumps, explodes, or wrong direction → Try next config

---

## How to Identify Correct Transform

### Good Signs (Correct Transform):
- ✅ Initialization succeeds quickly
- ✅ Yaw rotation tracked smoothly
- ✅ Translation tracked without drift
- ✅ No sudden jumps in estimation
- ✅ Few "chi-squared" warnings in logs

### Bad Signs (Wrong Transform):
- ❌ Estimation explodes immediately
- ❌ Rotation goes backwards or wrong axis
- ❌ Large drift during simple motion
- ❌ Many "invalid reprojection" errors
- ❌ Filter diverges and never recovers

---

## Step 4: Once You Find Stable Config

When one of the test configs works:

```bash
# Example: If test_2 works best
cp src/open_vins_ros2_jazzy/config/pegasus_test_2/kalibr_imucam_chain.yaml \
   src/open_vins_ros2_jazzy/config/pegasus_fixed/kalibr_imucam_chain.yaml

# Rebuild
colcon build --packages-select ov_msckf
source install/setup.bash

# Use pegasus_fixed as final config
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_fixed
```

---

## Alternative: Check Isaac Sim Directly

If none of the test configs work, you need to find the ACTUAL camera transform from Isaac Sim:

### Method 1: USD File Inspection
```bash
# Find your Isaac Sim scene file (.usd)
# Open with text editor or Isaac Sim
# Look for camera sensor definition
# Note the transform (position + rotation)
```

### Method 2: ROS TF Tree (if available)
```bash
source install/setup.bash
ros2 run tf2_tools view_frames

# Check generated frames.pdf for transform between base_link and camera
```

### Method 3: Isaac Sim GUI
1. Open Isaac Sim with your scene
2. Select the camera object in scene tree
3. Check Transform properties (position + rotation)
4. Convert to rotation matrix

---

## Diagnostic Tools Created

### 1. IMU Data Monitor
```bash
source install/setup.bash
python3 diagnose_transform.py
```
Shows real-time IMU motion - useful to verify IMU is correct.

### 2. Check Camera Info
```bash
source install/setup.bash
ros2 topic echo /pegasus_1/camera_info --once
```
Verify camera intrinsics match config.

---

## Common T_imu_cam Transforms

If you know your camera orientation, use these references:

### Camera aligned with IMU (unlikely but possible):
```yaml
T_imu_cam:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

### Front-facing camera (ROS optical frame):
```yaml
T_imu_cam:
  - [0.0, -1.0, 0.0, 0.0]
  - [0.0, 0.0, -1.0, 0.0]
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

### Down-facing camera (nadir):
```yaml
T_imu_cam:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, -1.0, 0.0, -0.1]  # -0.1 = 10cm below IMU
  - [0.0, 0.0, 0.0, 1.0]
```

---

## Expected Timeline

| Task | Time | Result |
|------|------|--------|
| Rebuild with fixed intrinsics | 2 min | Ready to test |
| Test config 1 | 5 min | Know if works |
| Test config 2 (if needed) | 5 min | Know if works |
| Test config 3 (if needed) | 5 min | Know if works |
| **Total** | **10-20 min** | **Find working transform** |

---

## Summary

**The core issues were:**
1. ✅ **FIXED:** Wrong resolution (1920x1200 vs 1280x720)
2. ✅ **FIXED:** Wrong intrinsics (calibrated for wrong resolution)
3. ✅ **FIXED:** Wrong distortion (had distortion, actual camera has none)
4. ⚠️ **TESTING:** Wrong T_imu_cam transform (identity probably incorrect)

**Once you find correct T_imu_cam, the estimation should stabilize!**

---

## Files Created

- ✅ `config/pegasus_fixed/` - Correct intrinsics, test transform here
- ✅ `config/pegasus_test_1/` - Test: Identity transform
- ✅ `config/pegasus_test_2/` - Test: Front-facing optical frame
- ✅ `config/pegasus_test_3/` - Test: 90° yaw rotation
- ✅ `diagnose_transform.py` - IMU diagnostic tool
- ✅ `ACTION_PLAN.md` - This guide
- ✅ `TEST_TRANSFORMS.md` - Detailed transform testing guide

---

**Start with Step 1 (rebuild) then Step 2 (test configs). Report which config works!**

