# ✅ CORRECT 3-Terminal Workflow

## The Key Point

**ONE Container, THREE Terminals**

```
Terminal 1: ./run_docker.sh    (creates container)
Terminal 2: ./attach_docker.sh (attaches to same container)
Terminal 3: ./attach_docker.sh (attaches to same container)
```

## Why This Matters

### ❌ WRONG (creates 3 separate containers)
```
Terminal 1: ./run_docker.sh  → Container A
Terminal 2: ./run_docker.sh  → Container B  ❌ Can't see Container A's topics!
Terminal 3: ./run_docker.sh  → Container C  ❌ Can't see Container A's topics!
```
**Result:** Topics not visible, OpenVINS doesn't work

### ✅ CORRECT (all in same container)
```
Terminal 1: ./run_docker.sh    → Container A
Terminal 2: ./attach_docker.sh → Container A (attached)
Terminal 3: ./attach_docker.sh → Container A (attached)
```
**Result:** All processes share the same ROS2 environment, topics visible everywhere!

---

## Step-by-Step Commands

### Terminal 1: Start Container & Bag
```bash
# On host
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
./run_docker.sh

# Inside Docker (Terminal 1)
cd /catkin_ws
source install/setup.bash
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5 --loop
```

### Terminal 2: Attach & Run OpenVINS
```bash
# On host - NEW terminal window
./attach_docker.sh

# Inside Docker (Terminal 2) - same container!
cd /catkin_ws
source install/setup.bash
ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml
```

### Terminal 3: Attach & Run RViz
```bash
# On host - NEW terminal window
./attach_docker.sh

# Inside Docker (Terminal 3) - same container!
cd /catkin_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d src/open_vins/ov_msckf/launch/display_ros2.rviz
```

---

## How attach_docker.sh Works

```bash
# attach_docker.sh automatically:
# 1. Finds the running ov_ros2_22_04 container
# 2. Attaches your terminal to it
# 3. Opens a new bash shell inside the SAME container
```

If it says "No running container found", you forgot to run `./run_docker.sh` first!

---

## Verify You're in the Same Container

In each terminal (after attaching), check:

```bash
# All three terminals should show the SAME container ID
hostname

# All three terminals should see the same processes
ps aux | grep ros2
```

---

## Common Mistakes

### Mistake 1: Using run_docker.sh for all terminals
```bash
# DON'T DO THIS:
Terminal 1: ./run_docker.sh
Terminal 2: ./run_docker.sh  ❌ Wrong!
Terminal 3: ./run_docker.sh  ❌ Wrong!
```

### Mistake 2: Forgetting to start the first container
```bash
# DON'T DO THIS:
Terminal 1: ./attach_docker.sh  ❌ No container to attach to!
```

### Mistake 3: Starting multiple containers and trying to fix with env vars
```bash
# This won't help if you have multiple containers:
export ROS_DOMAIN_ID=0  # Doesn't fix the problem!
```

---

## If You Made a Mistake

### Stop all containers and start fresh:
```bash
# On host - stop all Docker containers
docker stop $(docker ps -q)

# Verify all stopped
docker ps

# Now start correctly:
# Terminal 1: ./run_docker.sh
# Terminal 2: ./attach_docker.sh
# Terminal 3: ./attach_docker.sh
```

---

## Quick Test

After starting all three terminals correctly:

**In Terminal 2 or 3:**
```bash
# Should see the bag topics from Terminal 1
ros2 topic list | grep snappy

# Expected output:
#   /snappy_cam/stereo_l
#   /snappy_cam/stereo_r
#   /snappy_imu
```

If you see them, you're in the same container! ✅

If you don't see them, you're in different containers! ❌

---

## Pro Tip

Add this to your workflow checklist:

1. ✅ Terminal 1: `./run_docker.sh`
2. ✅ Wait for command prompt inside Docker
3. ✅ Terminal 2: `./attach_docker.sh` (should say "Found running container")
4. ✅ Terminal 3: `./attach_docker.sh` (should say "Found running container")

If attach_docker.sh says "No running container found" → You skipped step 1!
