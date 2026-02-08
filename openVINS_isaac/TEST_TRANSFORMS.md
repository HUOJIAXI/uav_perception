# Test Different Camera-IMU Transforms

**Problem:** Estimation still unstable after fixing resolution/intrinsics
**Next Step:** Test different T_imu_cam transforms

---

## Updated Configuration

I've updated **`config/pegasus_fixed/`** with ACTUAL camera intrinsics:

```yaml
resolution: [1280, 720]  # Matches actual camera
intrinsics: [3054.1637, 3054.1637, 640.0, 360.0]  # From camera_info
distortion_coeffs: [0.0, 0.0, 0.0, 0.0]  # No distortion in Isaac Sim
```

---

## Possible Issue: Wrong T_imu_cam Transform

The identity matrix assumes IMU and camera are **perfectly aligned**. This is unlikely in Isaac Sim.

Common UAV camera orientations:

### Test 1: Identity (Current) - Aligned Frames
```yaml
# config/pegasus_test_1/
T_imu_cam:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```
**Assumes:** Camera and IMU have same orientation

---

### Test 2: Front-Facing Camera (Optical Frame Convention)
```yaml
# config/pegasus_test_2/
T_imu_cam:
  # Standard optical frame: X=right, Y=down, Z=forward
  # If UAV body: X=forward, Y=left, Z=up
  # Then: cam_X = -body_Y, cam_Y = -body_Z, cam_Z = body_X
  - [0.0, -1.0, 0.0, 0.0]
  - [0.0, 0.0, -1.0, 0.0]
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```
**Assumes:** Front-facing camera, standard ROS optical frame

---

### Test 3: Camera Rotated 90° Around Z
```yaml
# config/pegasus_test_3/
T_imu_cam:
  - [0.0, -1.0, 0.0, 0.0]
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```
**Assumes:** Camera rotated 90° yaw from IMU

---

## How to Test Each Configuration

### Method 1: Quick Visual Test

1. **Launch OpenVINS with test config:**
   ```bash
   cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
   colcon build --packages-select ov_msckf
   source install/setup.bash

   # Test each config
   ros2 launch ov_msckf subscribe.launch.py config:=pegasus_test_1 verbosity:=DEBUG
   ```

2. **Perform simple rotation test:**
   - Keep UAV stationary (no translation)
   - Rotate UAV slowly in YAW (around Z-axis)
   - Watch OpenVINS estimated pose

3. **Check result:**
   - ✅ **GOOD:** Estimated rotation follows actual rotation smoothly
   - ❌ **BAD:** Estimation jumps, diverges, or rotates in wrong direction

4. **Try next config** if current one fails

---

### Method 2: Use Diagnostic Tool

Run diagnostic tool to monitor IMU data:
```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
source install/setup.bash
python3 diagnose_transform.py
```

**What to watch:**
- IMU angular velocity direction when you rotate UAV
- Acceleration direction (Z should be ~9.81)
- Compare with OpenVINS estimated motion

---

## How to Update T_imu_cam

If you find which test config works, or need to set custom transform:

### Edit the config file:
```bash
nano src/open_vins_ros2_jazzy/config/pegasus_fixed/kalibr_imucam_chain.yaml
```

### Find T_imu_cam section and update matrix

### Rebuild:
```bash
colcon build --packages-select ov_msckf
source install/setup.bash
```

---

## Understanding T_imu_cam

**T_imu_cam is a 4x4 transformation matrix:**
```
[R R R tx]
[R R R ty]
[R R R tz]
[0 0 0  1]
```

- **R (3x3)**: Rotation from IMU frame to camera frame
- **t (3x1)**: Translation (camera position relative to IMU)

**Example:**
```yaml
T_imu_cam:
  - [0.0, -1.0, 0.0, 0.1]   # Camera X = -IMU_Y, offset 0.1m in IMU_X
  - [0.0, 0.0, -1.0, 0.0]   # Camera Y = -IMU_Z
  - [1.0, 0.0, 0.0, 0.0]    # Camera Z = IMU_X
  - [0.0, 0.0, 0.0, 1.0]
```

---

## Quick Test Procedure

```bash
# Terminal 1: Run diagnostic
source install/setup.bash
python3 diagnose_transform.py

# Terminal 2: Launch OpenVINS with test config
source install/setup.bash
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_test_1 verbosity:=DEBUG

# Terminal 3: Monitor OpenVINS pose
source install/setup.bash
ros2 topic echo /ov_msckf/poseimu
```

**Test sequence:**
1. Rotate UAV in yaw slowly
2. Watch if OpenVINS pose rotates correctly
3. If wrong, try next test config
4. Repeat until you find stable configuration

---

## Common Symptoms

| Symptom | Likely Cause | Try |
|---------|--------------|-----|
| Estimation explodes immediately | Wrong T_imu_cam rotation | Test configs 2 or 3 |
| Rotation direction backwards | Coordinate frame flipped | Negate rotation matrix |
| Estimation drifts during translation | Wrong intrinsics OR scale issue | Check intrinsics again |
| Features not tracked | Wrong resolution/intrinsics | Already fixed ✅ |
| Initialization fails | Multiple issues | Check IMU motion during init |

---

## Next Steps

1. ✅ **Test pegasus_test_1** (identity transform - current)
2. ⬜ If unstable, **test pegasus_test_2** (front-facing optical frame)
3. ⬜ If unstable, **test pegasus_test_3** (90° rotation)
4. ⬜ If all fail, check Isaac Sim camera definition directly
5. ⬜ Once stable config found, copy to pegasus_fixed

---

## Advanced: Check Isaac Sim Directly

If none work, you need to check Isaac Sim camera definition:

1. Open Isaac Sim
2. Select camera sensor in scene tree
3. Check "Transform" properties (position + rotation)
4. Note rotation relative to UAV body
5. Convert to T_imu_cam matrix

---

**Created:** 2026-02-08
**Status:** Ready for testing

