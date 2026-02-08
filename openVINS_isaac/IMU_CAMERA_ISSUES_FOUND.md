# CRITICAL: IMU and Camera Configuration Issues Found

**Date:** 2026-02-08
**Status:** 🔴 **CRITICAL ISSUES DETECTED**

---

## 🚨 Issues Discovered

### 1. **Camera Resolution Mismatch** 🔴 CRITICAL

**Actual camera output:**
```
Resolution: 1280 x 720 pixels
Frame ID: "camera"
Topic: /pegasus_1/rgb
```

**OpenVINS configuration:**
```yaml
# kalibr_imucam_chain.yaml
resolution: [1920, 1200]  # WRONG!
intrinsics: [4581.2456, 4581.2456, 957.8, 589.5]  # WRONG for 1280x720!
```

**Problem:**
- OpenVINS expects 1920x1200 but receives 1280x720
- **Intrinsics are calibrated for different resolution**
- Feature tracking will be completely wrong
- This explains "estimation goes mad" during motion!

**Impact:**
- Features tracked at wrong pixel locations
- Epipolar geometry incorrect
- Visual updates have huge errors
- Filter diverges when UAV moves

---

### 2. **Camera-IMU Frame Mismatch** 🔴 CRITICAL

**Actual frames:**
```
IMU frame:    "base_link"
Camera frame: "camera"
```

**OpenVINS configuration:**
```yaml
# kalibr_imucam_chain.yaml
T_imu_cam:  # Identity matrix (assumes same frame!)
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

**Problem:**
- IMU and camera are in **different coordinate frames**
- T_imu_cam is **identity** (assumes no rotation/translation)
- **No TF tree published** to provide correct transform
- When UAV rotates, IMU and camera measurements don't align

**Impact:**
- IMU propagation and visual updates use different reference frames
- Rotations cause massive estimation errors
- Filter diverges immediately when UAV moves

---

### 3. **IMU Data Check** ✅ MOSTLY OK

**Captured IMU sample:**
```
Angular velocity:
  x: 0.0012 rad/s
  y: -0.0019 rad/s
  z: 0.0004 rad/s

Linear acceleration:
  x: 0.62 m/s²
  y: -0.05 m/s²
  z: 9.86 m/s²
```

**Analysis:**
- ✅ Units correct (m/s², rad/s)
- ✅ Gravity detected (~9.81 m/s² in Z)
- ✅ Low gyro noise when stationary
- ⚠️ Small X-axis acceleration (0.62 m/s²) - UAV might be tilted or accelerating

**Note:** Isaac Sim also publishes `orientation` in IMU message (quaternion), but OpenVINS should ignore this and only use angular_velocity and linear_acceleration.

---

## 🔧 Required Fixes

### Fix 1: Correct Camera Resolution and Intrinsics 🔴 URGENT

**Option A: Update OpenVINS config to match actual camera**

Edit `config/pegasus_clean/kalibr_imucam_chain.yaml`:

```yaml
cam0:
  resolution: [1280, 720]  # CORRECTED to match actual camera

  # Intrinsics must be RECALIBRATED for 1280x720!
  # Current intrinsics are for 1920x1200
  # Need to scale:
  #   fx_new = fx_old * (width_new / width_old) = 4581.2456 * (1280/1920) = 3054.16
  #   fy_new = fy_old * (height_new / height_old) = 4581.2456 * (720/1200) = 2748.74
  #   cx_new = cx_old * (width_new / width_old) = 957.8 * (1280/1920) = 638.53
  #   cy_new = cy_old * (height_new / height_old) = 589.5 * (720/1200) = 353.70

  intrinsics: [3054.16, 2748.74, 638.53, 353.70]  # SCALED for 1280x720

  # Keep distortion coefficients (should scale similarly)
  distortion_coeffs: [0.14, -0.03, -0.0002, -0.00003]
```

**Option B: Change Isaac Sim camera to 1920x1200**
- Reconfigure Isaac Sim camera to output 1920x1200
- Keep current intrinsics
- May impact performance

**Recommendation:** Use Option A (update config) - faster to fix

---

### Fix 2: Correct Camera-IMU Transform 🔴 URGENT

**You need to determine the actual transform between base_link and camera frame.**

**Method 1: Check Isaac Sim USD file**
- Open Isaac Sim scene file
- Find camera and IMU sensor definitions
- Extract relative pose (position + rotation)

**Method 2: Measure from simulation**
If camera is mounted forward-facing on UAV:

Typical UAV configuration:
```
base_link (IMU):
  - X: forward
  - Y: left
  - Z: up

camera:
  - X: right (image horizontal)
  - Y: down (image vertical)
  - Z: forward (optical axis)
```

This would require a **90° rotation** between frames!

**Example transform (ADJUST based on your actual setup):**
```yaml
# If camera points forward, mounted on front of UAV:
T_imu_cam:
  # Rotation: IMU frame to camera optical frame (90° rotations)
  # Translation: camera offset from IMU center (in meters)
  - [0.0, 0.0, 1.0, 0.1]   # Camera X = IMU Z, offset 0.1m forward
  - [-1.0, 0.0, 0.0, 0.0]  # Camera Y = -IMU X
  - [0.0, -1.0, 0.0, 0.0]  # Camera Z = -IMU Y
  - [0.0, 0.0, 0.0, 1.0]
```

**⚠️ WARNING:** Do NOT use this example directly! You must:
1. Check your Isaac Sim camera orientation
2. Determine correct rotation matrix
3. Measure camera position offset from IMU

---

### Fix 3: Verify Frame IDs

Check that OpenVINS is actually receiving data:

```bash
# Check IMU topic
ros2 topic echo /telemetry/imu | grep frame_id
# Should show: frame_id: base_link

# Check camera topic
ros2 topic info /pegasus_1/rgb -v
# Should show publishers
```

---

## 📋 Step-by-Step Fix Procedure

### Step 1: Fix camera resolution (5 minutes)

```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac/src/open_vins_ros2_jazzy/config/pegasus_clean

# Backup
cp kalibr_imucam_chain.yaml kalibr_imucam_chain.yaml.backup

# Edit kalibr_imucam_chain.yaml:
# - Change resolution to [1280, 720]
# - Update intrinsics to scaled values (see Fix 1 above)
```

### Step 2: Determine camera-IMU transform (15-30 minutes)

**Option A: Check Isaac Sim**
1. Open Isaac Sim scene
2. Select camera object
3. Note position (x, y, z) relative to UAV center
4. Note rotation (roll, pitch, yaw) relative to UAV body

**Option B: Use calibration tool**
```bash
# If you have a checkerboard pattern in sim:
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag static_pattern.bag \
  --cam camchain.yaml \
  --imu imu.yaml \
  --target target.yaml
```

**Option C: Estimate from known mounting**
- If camera is front-facing, pointing forward
- Mounted ~10cm ahead of IMU center
- Use example transform above (with adjustments)

### Step 3: Update T_imu_cam

Edit `kalibr_imucam_chain.yaml` with correct transform matrix.

### Step 4: Rebuild and test

```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
colcon build --packages-select ov_msckf
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_clean \
  verbosity:=DEBUG
```

### Step 5: Verify fixes

During operation, check:
```bash
# Should see successful initialization
# Should see visual features tracked
# Should NOT see "WARNING: invalid reprojection" or similar
# Trajectory should follow UAV motion smoothly
```

---

## 🔍 Why This Causes "Estimation Goes Mad"

**Before fixes (current state):**
```
UAV moves → IMU measures rotation in base_link frame
           ↓
Camera sees features move → OpenVINS projects using WRONG intrinsics
           ↓
Feature locations completely wrong (wrong resolution)
           ↓
OpenVINS tries to match IMU rotation to camera features
           ↓
Frames don't align (identity T_imu_cam but frames are different)
           ↓
Huge reprojection errors → Filter diverges
           ↓
"Estimation goes mad"
```

**After fixes:**
```
UAV moves → IMU measures rotation in base_link frame
           ↓
Transform to camera frame using CORRECT T_imu_cam
           ↓
Camera sees features → OpenVINS projects using CORRECT intrinsics
           ↓
Features match expectations → Small reprojection errors
           ↓
Filter converges → Smooth tracking ✅
```

---

## 🎯 Priority

1. **CRITICAL (Fix immediately):**
   - ✅ Camera resolution and intrinsics (Fix 1)
   - ✅ Camera-IMU transform (Fix 2)

2. **High (Fix after critical):**
   - IMU noise parameters (from previous analysis)
   - ZUPT enable (from previous analysis)

3. **Medium:**
   - Initialization settings
   - Feature tracking parameters

**DO NOT attempt to fix ZUPT/inflation until you fix resolution and transform!**
The current setup will fail regardless of other parameters.

---

## 📊 Quick Verification Commands

```bash
# Check actual camera resolution
ros2 topic echo /pegasus_1/rgb --once | grep -E "height|width"

# Check actual IMU frame
ros2 topic echo /telemetry/imu --once | grep frame_id

# Check configured resolution
grep "resolution:" src/open_vins_ros2_jazzy/config/pegasus_clean/kalibr_imucam_chain.yaml

# Check configured intrinsics
grep "intrinsics:" src/open_vins_ros2_jazzy/config/pegasus_clean/kalibr_imucam_chain.yaml
```

---

## 📝 Summary

**Current state:**
- ❌ Resolution: expects 1920x1200, gets 1280x720
- ❌ Intrinsics: calibrated for wrong resolution
- ❌ T_imu_cam: identity matrix (assumes aligned frames)
- ✅ IMU data: units and values correct

**Required actions:**
1. Update resolution to [1280, 720]
2. Scale intrinsics accordingly
3. Determine and set correct T_imu_cam
4. Rebuild and test

**Expected result after fixes:**
- ✅ Features tracked at correct pixel locations
- ✅ IMU and camera frames properly aligned
- ✅ Filter converges during motion
- ✅ Trajectory follows UAV smoothly

---

**Next steps:** Fix resolution and intrinsics first (easiest), then determine correct camera-IMU transform.

