# Pegasus Fixed Configuration

**Purpose:** Corrected configuration with proper camera resolution and intrinsics

**Status:** ⚠️ **RESOLUTION FIXED** - T_imu_cam still needs verification

---

## Critical Fixes Applied

### 1. Camera Resolution ✅ FIXED
```yaml
resolution: [1280, 720]  # Was: [1920, 1200]
```

### 2. Camera Intrinsics ✅ FIXED (scaled)
```yaml
# Old (for 1920x1200):
intrinsics: [4581.2456, 4581.2456, 957.8, 589.5]

# New (for 1280x720):
intrinsics: [3054.1637, 2748.7474, 638.5333, 353.7000]
```

---

## How to Test

```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
colcon build --packages-select ov_msckf
source install/setup.bash

ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_fixed \
  verbosity:=DEBUG
```

---

**Created:** 2026-02-08
