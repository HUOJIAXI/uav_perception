# Quick Start Guide - OpenVINS with Isaac Sim

## Ready to Run!

Your OpenVINS is now configured for Isaac Sim simulation.

### Step 1: Verify Isaac Sim is Running

Check that these topics exist:
```bash
ros2 topic list | grep -E "(pegasus_1|telemetry)"
```

Expected output:
```
/pegasus_1/rgb
/pegasus_1/camera_info
/telemetry/imu
```

### Step 2: Launch OpenVINS

Simply run:
```bash
cd /home/huojiaxi/Desktop/uav_perception/jazzy_openVINS
./launch_isaac_sim.sh
```

### Step 3: Move the Drone

For OpenVINS to initialize, the drone needs to move:
- **Minimum rotation:** ~10 degrees
- **Recommended:** Fly the drone around, rotate, translate

Watch the terminal for:
```
[INFO] - INITIALIZED!
```

### Step 4: Visualize

RViz will open automatically showing:
- Camera feed
- Feature tracks
- Estimated trajectory
- SLAM features

### View the Odometry Output

In another terminal:
```bash
source install/setup.bash
ros2 topic echo /ov_msckf/odometry
```

## Configuration Summary

- **Camera:** Monocular, 1280x720, no distortion
- **IMU Rate:** 100 Hz expected
- **Camera Rate:** ~20 Hz (current)
- **Feature Points:** 200
- **Tracking:** KLT-based
- **Calibration:** Online extrinsic calibration enabled

## Need Help?

See `ISAAC_SIM_SETUP.md` for detailed documentation and troubleshooting.

## Common Commands

**Check OpenVINS topics:**
```bash
ros2 topic list | grep ov_msckf
```

**Monitor odometry:**
```bash
ros2 topic hz /ov_msckf/odometry
```

**Record data for later:**
```bash
ros2 bag record /pegasus_1/rgb /pegasus_1/camera_info /telemetry/imu /ov_msckf/odometry
```
