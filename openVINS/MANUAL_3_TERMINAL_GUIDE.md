# Manual 3-Terminal OpenVINS Setup for UAV Dataset

## Overview

This guide shows how to run OpenVINS with the UZH-FPV UAV dataset using **3 separate terminals** for maximum control and visibility.

---

## Prerequisites

1. **Docker image built:**
   ```bash
   ./build_docker.sh
   ```

2. **Dataset available at:**
   ```
   ~/datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/
   ```

3. **Workspace built** (first time only - see "One-Time Setup" below)

---

## Quick Start: 3-Terminal Setup

**CRITICAL:** Only use `./run_docker.sh` for the FIRST terminal. For terminals 2 and 3, use `./attach_docker.sh` to attach to the same container. This ensures all processes share the same ROS2 environment!

### Terminal 1: Start Docker & Bag Player

```bash
# On host - START new container
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
./run_docker.sh

# Inside Docker
cd /catkin_ws
source install/setup.bash

# Play the dataset
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5 --loop
```

**Playback speed options:**
- `-r 0.5` = 50% speed (recommended for first run)
- `-r 1.0` = real-time
- `-r 0.1` = 10% speed (for debugging)
- `--loop` = repeat continuously

---

### Terminal 2: Attach to Docker & Run OpenVINS

**Wait ~5 seconds** after starting the bag player, then:

```bash
# On host (new terminal)
# IMPORTANT: Use attach_docker.sh to attach to the running container!
./attach_docker.sh

# Inside Docker
cd /catkin_ws
source install/setup.bash

# Run OpenVINS
ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml
```

**Watch for:**
- `subscribing to /snappy_cam/stereo_l`
- `subscribing to /snappy_imu`
- `Received first IMU message`
- `System initialized!`

---

### Terminal 3: Attach to Docker & Run RViz

**Wait ~10 seconds** after starting OpenVINS, then:

```bash
# On host (new terminal)
# IMPORTANT: Use attach_docker.sh to attach to the running container!
./attach_docker.sh

# Inside Docker
cd /catkin_ws
source install/setup.bash

# Launch RViz
ros2 run rviz2 rviz2 -d src/open_vins/ov_msckf/launch/display_ros2.rviz
```

**What you should see:**
- 🟢 **Green path** = OpenVINS VIO estimate (`/ov_msckf/pathimu`)
- 🔵 **Cyan path** = Ground truth (`/groundtruth/odometry`)
- 🔴 **Orange/Red points** = MSCKF feature points
- 🟡 **Yellow points** = SLAM features
- 📷 **TF frames** = Camera and IMU frames

---

## One-Time Setup

If this is your first time, build the workspace:

```bash
./run_docker.sh

# Inside Docker
cd /catkin_ws

# Build OpenVINS
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

**Note:** The build artifacts are saved on your host machine at:
`~/Desktop/uav_perception/catkin_ws_ov/`

You only need to build once (unless you modify the code).

---

## Troubleshooting

### Issue: Topics not visible between terminals

**Most Common Cause:** Using `./run_docker.sh` for all terminals instead of `./attach_docker.sh`!

**Fix:**
1. Close all terminals
2. Terminal 1: `./run_docker.sh` (starts container)
3. Terminal 2: `./attach_docker.sh` (attaches to same container)
4. Terminal 3: `./attach_docker.sh` (attaches to same container)

**Why this matters:** Each `./run_docker.sh` creates a NEW container. Using `./attach_docker.sh` attaches to the EXISTING container, ensuring all processes share the same ROS2 environment.

**Secondary causes:**
- ROS_DOMAIN_ID or ROS_LOCALHOST_ONLY not set (but this is handled by run_docker.sh now)

---

### Issue: OpenVINS shows "No messages received"

**Diagnostic:**
```bash
# In Terminal 2 (OpenVINS), press Ctrl+C to stop it
# Then check if bag topics are visible:
ros2 topic list | grep snappy

# Check topic rates:
ros2 topic hz /snappy_imu
ros2 topic hz /snappy_cam/stereo_l
```

**If topics not visible:**
```bash
# Restart ROS2 daemon in each terminal:
ros2 daemon stop
ros2 daemon start
```

---

### Issue: "/ov_msckf/pathimu" doesn't appear

**Diagnostic (in Terminal 3):**
```bash
# Stop RViz (Ctrl+C)
# Check what topics exist:
ros2 topic list | grep ov_msckf

# Check if OpenVINS node is running:
ros2 node list | grep msckf
```

**If no ov_msckf topics:**
- OpenVINS may not have initialized
- Check Terminal 2 for errors
- Look for "System initialized!" message

**Common causes:**
1. Not enough motion in dataset → Skip ahead: `ros2 bag play <bag> --start-offset 10.0`
2. Bag playing too fast → Slow down: `-r 0.3`
3. Feature tracking failing → Check OpenVINS terminal for errors

---

### Issue: System doesn't initialize

**Check OpenVINS output** (Terminal 2) for:
- "Waiting for sufficient motion"
- "Not enough features tracked"
- "Disparity too low"

**Fix:** Reduce initialization thresholds in config:

Edit `config/uzhfpv_indoor/estimator_config.yaml`:
```yaml
init_imu_thresh: 0.10      # Reduce from 0.30
init_max_disparity: 1.0    # Reduce from 2.0
init_max_features: 30      # Reduce from 50
```

---

### Issue: RViz shows nothing

**Check:**
1. Is OpenVINS publishing? `ros2 topic hz /ov_msckf/pathimu`
2. Is RViz subscribed? Check RViz "Displays" panel
3. Is Fixed Frame correct? Should be "global"

**Try:**
```bash
# Reload RViz config
File → Open Config → src/open_vins/ov_msckf/launch/display_ros2.rviz
```

---

## Helpful Commands

### Check all OpenVINS topics:
```bash
ros2 topic list | grep ov_msckf
```

### Monitor topic rates:
```bash
ros2 topic hz /ov_msckf/pathimu
ros2 topic hz /ov_msckf/poseimu
```

### Echo a topic (see data):
```bash
ros2 topic echo /ov_msckf/poseimu --once
```

### Check bag info:
```bash
ros2 bag info /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3
```

### Quick health check:
```bash
./src/open_vins/quick_check.sh
```

---

## Advanced: Verify Ground Truth Visualization

The dataset includes ground truth. To see it in RViz:

1. **Check ground truth topics exist:**
   ```bash
   ros2 topic list | grep groundtruth
   # Should show: /groundtruth/pose and /groundtruth/odometry
   ```

2. **In RViz:** The cyan path should appear automatically (configured in display_ros2.rviz)

3. **If not visible:** Add manually:
   - Click "Add" button in RViz
   - Choose "Odometry" display type
   - Set topic to `/groundtruth/odometry`
   - Set color to cyan

---

## Dataset Information

**UZH-FPV Indoor Forward 5:**
- **Duration:** ~150 seconds
- **Camera:** Stereo 640x480 @ 31 Hz
- **IMU:** 200 Hz
- **Ground Truth:** Yes (from motion capture)
- **Environment:** Indoor flight

**Topics:**
- `/snappy_cam/stereo_l` - Left camera
- `/snappy_cam/stereo_r` - Right camera
- `/snappy_imu` - IMU data
- `/groundtruth/pose` - GT poses
- `/groundtruth/odometry` - GT odometry

---

## Stopping Everything

1. **Terminal 3 (RViz):** Ctrl+C or close window
2. **Terminal 2 (OpenVINS):** Ctrl+C
3. **Terminal 1 (Bag):** Ctrl+C

**Or:** Just close all terminal windows (Docker containers will auto-cleanup)

---

## Next Steps

After successfully running:
1. Try different playback speeds (`-r 0.5`, `-r 1.0`, `-r 2.0`)
2. Skip to interesting parts (`--start-offset 20.0`)
3. Try other UZH-FPV sequences
4. Tune parameters in `config/uzhfpv_indoor/estimator_config.yaml`
5. Save trajectories for evaluation

---

## Tips

- **Always start in order:** Bag → OpenVINS → RViz
- **Wait between starts:** Give each component 5-10 seconds
- **Watch Terminal 2:** Most errors appear in the OpenVINS terminal
- **Use `--loop`:** Bag will repeat automatically for testing
- **Slow playback:** Use `-r 0.5` for better accuracy on first run
- **Check topics first:** Use `ros2 topic list` to verify everything
