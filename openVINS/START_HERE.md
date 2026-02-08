# OpenVINS UAV Dataset - Quick Start

## 📋 What You Have

- **Dataset:** UZH-FPV Indoor UAV (ROS2 format)
- **Location:** `~/datasets/uzhfpv_indoor/`
- **Ground Truth:** ✅ Included in bag file
- **Docker:** ✅ Configured with ROS2

---

## 🚀 Quick Start (3 Terminals)

**⚠️ CRITICAL:** Use `run_docker.sh` ONLY for Terminal 1, then `attach_docker.sh` for Terminals 2 & 3!
- `./run_docker.sh` = Creates a NEW container
- `./attach_docker.sh` = Attaches to EXISTING container

**This is why your topics weren't visible!** Using `run_docker.sh` three times creates three separate containers that can't communicate!

### Terminal 1: Start Docker & Bag Player
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
./run_docker.sh

# Inside Docker:
cd /catkin_ws && source install/setup.bash
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5 --loop
```

### Terminal 2: Attach to Docker & Run OpenVINS (wait 5 sec)
```bash
# IMPORTANT: Use attach_docker.sh, NOT run_docker.sh!
./attach_docker.sh

# Inside Docker:
cd /catkin_ws && source install/setup.bash
ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml
```

### Terminal 3: Attach to Docker & Run RViz (wait 10 sec)
```bash
# IMPORTANT: Use attach_docker.sh, NOT run_docker.sh!
./attach_docker.sh

# Inside Docker:
cd /catkin_ws && source install/setup.bash
ros2 run rviz2 rviz2 -d src/open_vins/ov_msckf/launch/display_ros2.rviz
```

---

## ❌ If `/ov_msckf/pathimu` Still Doesn't Appear

### Step 1: Quick Check
```bash
./run_docker.sh
cd /catkin_ws && source install/setup.bash
./src/open_vins/quick_check.sh
```

### Step 2: Manual Debug
In Terminal 2 (while OpenVINS is running), check for these messages:
- ✅ `subscribing to /snappy_cam/stereo_l`
- ✅ `subscribing to /snappy_imu`
- ✅ `Received first IMU message`
- ✅ `System initialized!`

If you see ❌ errors or warnings, copy them and report back.

### Step 3: Verify Topics in Another Terminal
```bash
# Attach to running container
docker ps  # Get container ID
docker exec -it <container_id> bash

cd /catkin_ws && source install/setup.bash

# Check OpenVINS topics
ros2 topic list | grep ov_msckf

# Should show:
#   /ov_msckf/pathimu
#   /ov_msckf/poseimu
#   /ov_msckf/points_msckf
```

---

## 📚 Documentation

- **Full Guide:** `MANUAL_3_TERMINAL_GUIDE.md` - Complete step-by-step instructions
- **Troubleshooting:** `FIX_TOPICS.md` - If topics don't appear
- **Diagnostic:** `SIMPLE_TEST.md` - Detailed debugging steps

---

## 🛠️ Useful Scripts

- **`./run_docker.sh`** - Start Docker container
- **`./quick_check.sh`** - Health check (run inside Docker)
- **`./debug_topics.sh`** - Topic diagnostics (run inside Docker)
- **`./test_openvins_step_by_step.sh`** - Comprehensive test (run inside Docker)

---

## 💡 Tips

1. **Always set environment variables** in each terminal:
   ```bash
   export ROS_DOMAIN_ID=0
   export ROS_LOCALHOST_ONLY=1
   ```

2. **Start in order:** Bag → OpenVINS → RViz (with 5-10 sec delays)

3. **Watch Terminal 2** for OpenVINS status and errors

4. **Slow playback first:** Use `-r 0.5` for initial testing

5. **Check topics between steps:**
   ```bash
   ros2 topic list | grep snappy  # After starting bag
   ros2 topic list | grep ov_msckf  # After starting OpenVINS
   ```

---

## 🎯 What You Should See in RViz

- 🟢 **Green path** = OpenVINS estimate
- 🔵 **Cyan path** = Ground truth (from bag)
- 🔴 **Orange/Red points** = Tracked features
- 📷 **TF frames** = Camera/IMU visualization

---

## ❓ Still Having Issues?

Run the diagnostic and share the output:
```bash
./run_docker.sh
cd /catkin_ws && source install/setup.bash
./src/open_vins/test_openvins_step_by_step.sh
```

Copy the output and report:
1. What errors appear?
2. Does OpenVINS say "System initialized!"?
3. What does `ros2 topic list | grep ov_msckf` show?
