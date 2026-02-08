# Final Test Plan - Find Correct Transform

## What We Know So Far

### ✅ Fixed Issues:
1. **Camera resolution**: Corrected to 1280×720
2. **Camera intrinsics**: fx=fy=3054.16, cx=640, cy=360
3. **Distortion**: Set to zero (no distortion in Isaac Sim)

### ⚠️ Remaining Issue: T_imu_cam Transform

**Test Results:**
- **test_1 (identity)**: ❌ `not enough feats to compute disp: 0,XX` → Wrong transform, no parallax
- **test_2 (front-facing)**: ⚠️ Gets features BUT crashes with **Jacobian error** `1e+302` → Close but not quite right
- **test_3 (90° yaw)**: Not tested yet
- **test_4 (inverse)**: Created with disabled online calibration

---

## Quick Test Procedure

### Step 1: Rebuild
```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
colcon build --packages-select ov_msckf
source install/setup.bash
```

### Step 2: Test Each Config (5 min each)

#### Test 4: Inverse + No Calibration (Try this first!)
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_test_4 \
  use_stereo:=false \
  max_cameras:=1 \
  verbosity:=INFO
```
**What changed:** Disabled all online calibration, inverse rotation from test_2

**Move UAV** and watch for:
- ✅ GOOD: `[init]: initialization successful!`
- ❌ BAD: Jacobian error or `not enough feats`

---

#### Test 3: 90° Yaw Rotation
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_test_3 \
  use_stereo:=false \
  max_cameras:=1 \
  verbosity:=INFO
```

---

#### If All Fail: Try Simplified Config
Copy pegasus_test_4 settings to test_2 (keep transform, disable calibration):
```bash
cp src/open_vins_ros2_jazzy/config/pegasus_test_4/estimator_config.yaml \
   src/open_vins_ros2_jazzy/config/pegasus_test_2/
colcon build --packages-select ov_msckf
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py config:=pegasus_test_2 use_stereo:=false max_cameras:=1
```

---

## What to Watch For

### Good Signs (Correct Config):
```
[init]: initialization successful!
[TIME]: XXX hz, X.X ms behind
(No Jacobian errors)
(No segfaults)
```

### Bad Signs:
```
❌ not enough feats to compute disp: 0,XX  → Wrong T_imu_cam (identity)
❌ Manifold::PlusJacobian computation failed  → Wrong T_imu_cam (close but off)
❌ Segfault (exit code -11)  → Config parsing error
```

---

## If Test_4 Works

Copy it to pegasus_fixed:
```bash
cp src/open_vins_ros2_jazzy/config/pegasus_test_4/*.yaml \
   src/open_vins_ros2_jazzy/config/pegasus_fixed/
```

Then test with motion and check stability!

---

## If Nothing Works

You need to get the **ACTUAL** camera transform from Isaac Sim:

### Method: Check Isaac Sim Scene

1. Open Isaac Sim
2. Find camera object in scene tree
3. Note its **Transform** relative to UAV base
4. **Key question:** What direction is the camera's **optical axis** (Z-axis) pointing?
   - Forward (+X body)?
   - Down (-Z body)?
   - Other direction?

### Common UAV Camera Setups:

**Front-facing (FPV-style):**
```
Camera optical axis: Forward (body +X)
Camera up: body -Z
Camera right: body -Y
```

**Down-facing (mapping-style):**
```
Camera optical axis: Down (body -Z)
Camera up: Forward (body +X)
Camera right: body +Y
```

Tell me which one and I can generate the correct T_imu_cam!

---

## Config Transforms Summary

| Config | T_imu_cam | Calibration | Status |
|--------|-----------|-------------|--------|
| test_1 | Identity | On | ❌ No disparity |
| test_2 | Front-facing optical | On | ⚠️ Jacobian error |
| test_3 | 90° yaw | On | ⬜ Not tested |
| test_4 | Inverse of test_2 | **OFF** | ⬜ **Try this!** |

---

## Alternative: Visualize Features

To see if features are being tracked:
```bash
# Terminal 1: Launch OpenVINS
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_test_4

# Terminal 2: Check feature tracking
ros2 topic echo /ov_msckf/trackedfeats | grep "size"

# Should see: size > 50 if features tracked correctly
```

---

## Next Steps

1. ✅ Test pegasus_test_4 (inverse + no calibration)
2. ⬜ If that fails, test pegasus_test_3 (90° yaw)
3. ⬜ If still failing, check Isaac Sim camera orientation directly
4. ⬜ Report which transform (if any) allows initialization

**The key is finding a transform where:**
- Features have disparity (not zero)
- Jacobian doesn't explode
- Initialization succeeds

Let me know the results!

