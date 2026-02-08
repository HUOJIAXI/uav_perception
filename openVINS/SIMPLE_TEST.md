# Simple Test - Find Why /ov_msckf/pathimu Doesn't Appear

## Quick Diagnosis

Run this **inside Docker** to find the problem:

```bash
cd /catkin_ws
source install/setup.bash

# Run the automated diagnostic
./src/open_vins/test_openvins_step_by_step.sh
```

Watch the output for any **errors** or **warnings**.

---

## Manual Step-by-Step Test

If you want to manually debug:

### Terminal 1: Start Docker with Correct Settings

```bash
# On host
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

./run_docker.sh
```

### Terminal 1 (Inside Docker): Check Everything

```bash
cd /catkin_ws
source install/setup.bash

# 1. Restart daemon
ros2 daemon stop
ros2 daemon start

# 2. Verify workspace is built
ls install/ov_msckf/lib/ov_msckf/run_subscribe_msckf
# Should show the executable

# 3. Play bag in background
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5 &

# 4. Wait 5 seconds
sleep 5

# 5. Verify bag topics are visible
ros2 topic list | grep snappy
# Should show:
#   /snappy_cam/stereo_l
#   /snappy_cam/stereo_r
#   /snappy_imu

# 6. Check if data is flowing
ros2 topic hz /snappy_imu
# Should show ~100 Hz (bag is 0.5x speed)
# Press Ctrl+C after a few seconds

# 7. Run OpenVINS with verbose output
ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml
```

### What to Look For in OpenVINS Output:

#### ✅ GOOD - System is initializing:
```
[INFO] [run_subscribe_msckf]: subscribing to /snappy_cam/stereo_l
[INFO] [run_subscribe_msckf]: subscribing to /snappy_cam/stereo_r
[INFO] [run_subscribe_msckf]: subscribing to /snappy_imu
[INFO] [run_subscribe_msckf]: Received first IMU message
[INFO] [run_subscribe_msckf]: Received first camera message
[INFO] [run_subscribe_msckf]: Trying to initialize...
[INFO] [run_subscribe_msckf]: System initialized!
```

#### ❌ BAD - Not receiving messages:
```
[WARN] [run_subscribe_msckf]: No messages received
[ERROR] [run_subscribe_msckf]: Failed to subscribe to /snappy_cam/stereo_l
```

#### ❌ BAD - Initialization failing:
```
[WARN] [run_subscribe_msckf]: Waiting for sufficient motion...
[WARN] [run_subscribe_msckf]: Not enough features tracked
[ERROR] [run_subscribe_msckf]: Initialization failed
```

---

## Common Problems and Solutions

### Problem 1: Workspace Not Built

**Symptom:** `run_subscribe_msckf` not found

**Fix:**
```bash
cd /catkin_ws
colcon build --packages-select ov_core ov_init ov_msckf ov_eval
source install/setup.bash
```

### Problem 2: Topics Not Visible

**Symptom:** `ros2 topic list` doesn't show `/snappy_*` topics

**Fix:**
```bash
# Check if bag is really playing
ps aux | grep "ros2 bag"

# Kill and restart bag
killall -9 ros2
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5 &
sleep 5
ros2 topic list
```

### Problem 3: QoS Mismatch

**Symptom:** Topics visible but OpenVINS says "No messages received"

**Check:**
```bash
# See QoS settings of bag topics
ros2 topic info /snappy_cam/stereo_l -v
```

**Potential Fix:** OpenVINS might be using incompatible QoS. Let me check the source code...

### Problem 4: Initialization Not Happening

**Symptom:** OpenVINS runs but says "waiting for initialization"

**Reasons:**
- Not enough motion in the beginning of the bag
- Camera images too dark/bright
- Feature tracking failing

**Fix:** Skip ahead in the bag:
```bash
# Play from 10 seconds into the bag
ros2 bag play <bag> -r 0.5 --start-offset 10.0
```

Or enable initialization with less motion:
Edit `config/uzhfpv_indoor/estimator_config.yaml`:
```yaml
init_imu_thresh: 0.10  # Reduce from 0.30
init_max_disparity: 1.0  # Reduce from 2.0
```

### Problem 5: Still No Topics After Initialization

**Symptom:** OpenVINS says "initialized" but `/ov_msckf/pathimu` still doesn't appear

**Debug:**
```bash
# List all ov_msckf topics
ros2 topic list | grep ov_msckf

# If empty, check node list
ros2 node list

# Check if node is in a namespace
ros2 node list | grep msckf
```

**Possible Issue:** Node might be in wrong namespace

**Fix:** Check if topics are published with namespace:
```bash
ros2 topic list | grep -i msckf
# Might show: /ov_msckf/run_subscribe_msckf/pathimu (extra namespace)
```

---

## Most Likely Issue: Build or DDS Problem

### Quick Nuclear Option (Rebuilds Everything):

```bash
cd /catkin_ws

# 1. Clean everything
rm -rf build install log

# 2. Rebuild from scratch
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Source
source install/setup.bash

# 4. Restart daemon
ros2 daemon stop
sleep 2
ros2 daemon start
sleep 2

# 5. Try again
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5 &
sleep 5
ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml
```

---

## Report Back

After running the diagnostic script, tell me:

1. **Does the bag play successfully?** (`ros2 topic list` shows `/snappy_*` topics)
2. **Does OpenVINS start?** (No errors when launching)
3. **What does OpenVINS print?** (Copy the first 20 lines)
4. **Any errors or warnings?**

This will help me pinpoint the exact issue!
