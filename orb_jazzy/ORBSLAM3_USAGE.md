# ORB-SLAM3 Monocular-Inertial for Isaac Sim UAV

## Quick Start

### Launch Everything (ORB-SLAM3 + RViz2)

```bash
source install/setup.bash
ros2 launch orbslam3 isaac_sim_mono_inertial.launch.py
```

This will start:
- ✅ ORB-SLAM3 monocular-inertial node
- ✅ RViz2 with pre-configured visualization

---

## Launch Options

### With RViz2 (Default)
```bash
ros2 launch orbslam3 isaac_sim_mono_inertial.launch.py
```

### Without RViz2
```bash
ros2 launch orbslam3 isaac_sim_mono_inertial.launch.py use_rviz:=false
```

### Custom Topics
```bash
ros2 launch orbslam3 isaac_sim_mono_inertial.launch.py \
  camera_topic:=/your/camera/topic \
  imu_topic:=/your/imu/topic
```

### All Options Combined
```bash
ros2 launch orbslam3 isaac_sim_mono_inertial.launch.py \
  camera_topic:=/pegasus_1/rgb \
  imu_topic:=/telemetry/imu \
  use_rviz:=true
```

---

## Manual Control

### Run ORB-SLAM3 Only
```bash
source install/setup.bash
ros2 run orbslam3 mono-inertial \
  install/orbslam3/share/orbslam3/vocabulary/ORBvoc.txt \
  install/orbslam3/share/orbslam3/config/monocular-inertial/IsaacSim_UAV.yaml
```

### Run RViz2 Separately
```bash
source install/setup.bash
rviz2 -d install/orbslam3/share/orbslam3/rviz/orbslam3.rviz
```

---

## Topics

### Subscribed Topics
- **Camera:** `/pegasus_1/rgb` (sensor_msgs/Image)
- **IMU:** `/telemetry/imu` (sensor_msgs/Imu)

### Published Topics
- **Camera Pose:** `/orbslam3/camera_pose` (geometry_msgs/PoseStamped)
- **Trajectory:** `/orbslam3/trajectory` (nav_msgs/Path)
- **Map Points:** `/orbslam3/map_points` (sensor_msgs/PointCloud2)
- **TF:** `/tf` (camera transform)

---

## Configuration Files

### Available Configs

Located in `install/orbslam3/share/orbslam3/config/monocular-inertial/`:

- **IsaacSim_UAV.yaml** - Balanced (1000 features, recommended)

Located in `install/orbslam3/share/orbslam3/config/monocular/`:

- **IsaacSim_UAV.yaml** - Monocular-only balanced (1200 features)
- **IsaacSim_UAV_Fast.yaml** - Fast mode (1000 features)
- **IsaacSim_UAV_VeryFast.yaml** - Maximum speed (800 features, 640x360)

---

## Monitoring

### Check Topic Rates
```bash
ros2 topic hz /orbslam3/camera_pose
ros2 topic hz /orbslam3/trajectory
ros2 topic hz /pegasus_1/rgb
ros2 topic hz /telemetry/imu
```

### Check Node Status
```bash
ros2 node list | grep SLAM
ros2 node info /ORB_SLAM3_Monocular_Inertial
```

---

## Output

### Trajectory File
When you stop ORB-SLAM3 (Ctrl+C), trajectory is saved to:
```
KeyFrameTrajectory.txt
```

Format: TUM RGB-D dataset format
```
timestamp tx ty tz qx qy qz qw
```

---

## Troubleshooting

### No Images Received
```bash
# Check camera is publishing
ros2 topic hz /pegasus_1/rgb

# Check if node is subscribed
ros2 topic info /pegasus_1/rgb --verbose
```

### No IMU Data
```bash
# Check IMU is publishing
ros2 topic hz /telemetry/imu

# Echo IMU data
ros2 topic echo /telemetry/imu --once
```

### Tracking Lost
- Move UAV slowly initially
- Point at textured areas (avoid blank walls)
- Ensure good lighting
- Check IMU data is arriving

### Poor Performance
- Use faster config: `IsaacSim_UAV_Fast.yaml`
- Reduce features in config file
- Close RViz2 if not needed

---

## Tips for Best Results

1. **Initialization:**
   - Start with UAV stationary
   - Move slowly with rotation initially
   - Point at feature-rich areas

2. **During Flight:**
   - Maintain moderate speeds
   - Avoid sudden rotations
   - Keep textured areas in view

3. **Performance:**
   - Monocular-Inertial is much more robust than monocular-only
   - IMU helps with fast rotations and scale estimation
   - Expect ~15-25 Hz processing rate

---

## System Requirements

- ROS2 Jazzy
- Isaac Sim running with UAV simulation
- Topics publishing:
  - `/pegasus_1/rgb` (~20-25 Hz)
  - `/telemetry/imu` (~40-50 Hz)
