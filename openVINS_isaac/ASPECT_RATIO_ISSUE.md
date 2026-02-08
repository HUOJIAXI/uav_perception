# Camera Aspect Ratio Mismatch Issue

## Problem Found

**Isaac Sim camera config (launch_with_camera.py line 162-164):**
```python
camera = Camera(
    prim_path=camera_prim_path,
    resolution=(640, 480),  # 4:3 aspect ratio
    frequency=30
)
```

**ROS2 published (actual):**
```
width: 1280
height: 720  # 16:9 aspect ratio!
```

## This is Critical!

The ROS2 Bridge is:
1. Upscaling 2x (640→1280, 480→720)
2. **Changing aspect ratio** (4:3 → 16:9)

This breaks the intrinsics completely!

## Solutions

### Option 1: Change Isaac Sim to match ROS (RECOMMENDED)

Edit `/home/huojiaxi/Desktop/uav_sim/launch_with_camera.py` line 162:

```python
camera = Camera(
    prim_path=camera_prim_path,
    resolution=(1280, 720),  # Match ROS output!
    frequency=30
)
```

Then the intrinsics will be correct.

### Option 2: Force ROS2 Bridge to Not Change Aspect Ratio

Need to configure the ROS2 Bridge to output 640×480 or scale correctly to 1280×960 (keeping 4:3).

### Option 3: Compute Correct Intrinsics for Cropped/Scaled Image

If 640×480 is being cropped/scaled to 1280×720, we need to:
1. Determine if it's cropping (cutting off top/bottom)
2. Determine if it's distorting (stretching)
3. Calculate correct intrinsics for the distortion

## Testing

After changing Isaac Sim camera resolution to 1280×720:

1. Restart Isaac Sim
2. Verify camera_info matches:
   ```bash
   ros2 topic echo /pegasus_1/camera_info --once
   ```
3. Rebuild OpenVINS (no config changes needed)
4. Test again

This should fix the fundamental problem!
