# Fix: OpenVINS Topics Not Visible

## Problem
- OpenVINS runs but doesn't publish topics
- Error: "cannot publish data" or "Error in destruction of rcl publisher"
- `ros2 topic list` doesn't show `/ov_msckf/*` topics

## Root Cause
ROS2 DDS (Data Distribution Service) discovery/communication issues between Docker containers or terminal sessions.

## Quick Fix (Try This First!)

### Step 1: Stop ALL Docker Containers
```bash
# On host machine
docker stop $(docker ps -q)
```

### Step 2: Restart with Fixed Configuration
```bash
# On host machine
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

# Start fresh
./run_docker.sh
```

### Step 3: Inside Docker - Restart ROS2 Daemon
```bash
# Inside Docker
ros2 daemon stop
ros2 daemon start
```

### Step 4: Verify Topics Work
```bash
# Inside Docker
cd /catkin_ws
source install/setup.bash

# Test 1: List topics
ros2 topic list

# Test 2: Check if bag topics appear (run this in one terminal)
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.1 &

# Give it 5 seconds
sleep 5

# Test 3: Should see bag topics now
ros2 topic list | grep snappy

# If you see topics, kill the test bag
killall ros2
```

## Complete Solution: Run Everything in One Container

The issue is often caused by running multiple Docker containers. Here's a better approach:

### Option A: Use tmux (Single Container, Multiple Windows)

```bash
# On host
./run_docker_tmux.sh
```

This script (create it below) runs everything in one container with split windows.

### Option B: Manual Single-Container Setup

```bash
# Terminal 1: Start Docker
./run_docker.sh
cd /catkin_ws
source install/setup.bash

# In this same terminal, run everything with & and tmux:
# Install tmux if needed
apt-get update && apt-get install -y tmux

# Start tmux
tmux

# Create 3 panes
# Ctrl+B then " (split horizontal)
# Ctrl+B then % (split vertical)
# Ctrl+B then arrow keys to navigate

# Pane 1 (top): Bag player
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5

# Pane 2 (bottom-left): OpenVINS
# Switch pane: Ctrl+B then arrow key
ros2 run ov_msckf run_subscribe_msckf src/open_vins/config/uzhfpv_indoor/estimator_config.yaml

# Pane 3 (bottom-right): RViz
# Switch pane: Ctrl+B then arrow key
ros2 run rviz2 rviz2 -d src/open_vins/ov_msckf/launch/display_ros2.rviz
```

## Environment Variables Explained

### ROS_DOMAIN_ID
- **What:** Isolates ROS2 communication
- **Default:** 0
- **Issue:** If different terminals have different IDs, they can't communicate
- **Fix:** Set to same value everywhere: `export ROS_DOMAIN_ID=0`

### ROS_LOCALHOST_ONLY
- **What:** Restricts DDS to localhost only
- **Default:** 0 (off)
- **Issue:** DDS tries to discover nodes over network, causing delays/failures in Docker
- **Fix:** Set to 1: `export ROS_LOCALHOST_ONLY=1`

### RMW_IMPLEMENTATION
- **What:** Which DDS implementation to use
- **Default:** Usually `rmw_cyclonedds_cpp` or `rmw_fastrtps_cpp`
- **Issue:** Inconsistent implementations can cause problems
- **Fix:** Set explicitly: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

## Permanent Fix: Update Docker Run Script

The `run_docker.sh` has been updated with:
```bash
export ROS_LOCALHOST_ONLY=1
--ipc=host
```

This should fix the issue for future runs.

## Verify Everything Works

After applying the fix, run this diagnostic:

```bash
# Inside Docker
./src/open_vins/debug_topics.sh
```

You should see:
- ✓ Localhost OK
- ✓ ROS_DOMAIN_ID set to 0
- ✓ ROS_LOCALHOST_ONLY set to 1
- ✓ Topics visible

## Alternative: Use ROS2 Launch File

Instead of multiple terminals, use a launch file (more reliable):

```bash
# Inside Docker
ros2 launch ov_msckf subscribe.launch.py \
    config:=uzhfpv_indoor \
    rviz_enable:=true
```

Then in another terminal/pane, play the bag:
```bash
ros2 bag play /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3 -r 0.5
```

## Still Not Working?

### Check 1: Is OpenVINS actually running?
```bash
ros2 node list
# Should show: /ov_msckf/run_subscribe_msckf
```

### Check 2: Are there any error messages?
Look at the OpenVINS terminal output for:
- "Failed to subscribe to..."
- "No messages received..."
- "Initialization failed..."

### Check 3: Rebuild OpenVINS
```bash
cd /catkin_ws
rm -rf build install log
colcon build --event-handlers console_cohesion+
source install/setup.bash
```

### Check 4: Check bag file
```bash
ros2 bag info /datasets/uzhfpv_indoor/indoor_forward_5_snapdragon_with_gt_ros2/indoor_forward_5_snapdragon_with_gt_ros2.db3
```

Make sure topics match what OpenVINS expects:
- `/snappy_cam/stereo_l`
- `/snappy_cam/stereo_r`
- `/snappy_imu`

### Check 5: Docker networking
If using multiple containers:
```bash
# All containers must use the same network
docker network create ros2_net

# Then run with:
docker run --network=ros2_net ...
```

But using `--net=host` (current setup) is simpler.

## Best Practice: Single Container Workflow

To avoid ALL these issues, always run everything in ONE container:

```bash
# Start one container
./run_docker.sh

# Inside, use tmux/screen for multiple "terminals"
# OR run bag player in background:
ros2 bag play <bag> -r 0.5 &

# Run OpenVINS
ros2 run ov_msckf run_subscribe_msckf <config> &

# Run RViz
ros2 run rviz2 rviz2 -d <config>
```

This guarantees all processes share the same ROS2 environment.
