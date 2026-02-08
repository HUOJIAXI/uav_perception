# Time Delay Analysis Results

**Date:** 2026-02-08
**Analysis Duration:** 30 seconds
**Samples Collected:** 1000 camera + 1000 IMU measurements

---

## Executive Summary

The camera topic (`/pegasus_1/rgb`) has a **significant and variable time delay** of approximately **76 milliseconds** compared to the `/clock` topic, while the IMU topic (`/telemetry/imu`) is **nearly perfectly synchronized** with the clock.

---

## Detailed Results

### Camera (`/pegasus_1/rgb`)
| Metric | Value |
|--------|-------|
| **Mean Delay** | **75.86 ms** |
| **Median Delay** | 64.00 ms |
| **Std Deviation** | 24.46 ms |
| **Min Delay** | 32.00 ms |
| **Max Delay** | 160.00 ms |
| **Range** | 128.00 ms |
| **Coefficient of Variation** | **32.25%** |
| **Status** | ✗ **HIGHLY VARIABLE** |

### IMU (`/telemetry/imu`)
| Metric | Value |
|--------|-------|
| **Mean Delay** | -0.14 ms |
| **Median Delay** | 0.00 ms |
| **Std Deviation** | 1.95 ms |
| **Min Delay** | -32.00 ms |
| **Max Delay** | 0.00 ms |
| **Range** | 32.00 ms |
| **Coefficient of Variation** | 1357.15% (near-zero mean) |
| **Status** | ✓ **Nearly Synchronized** |

### Camera vs IMU Comparison
- **Mean Delay Difference:** 76.00 ms
- **Std Deviation Difference:** 22.51 ms
- **Conclusion:** Camera lags behind IMU by approximately 76ms with high variability

---

## Interpretation

### 1. **Camera Delay is NOT Constant**
- The standard deviation of 24.46 ms is significant (32.25% coefficient of variation)
- Delay ranges from 32ms to 160ms, a span of 128ms
- This indicates the delay varies substantially over time

### 2. **IMU is Well Synchronized**
- Mean delay of -0.14ms indicates near-perfect synchronization
- Small variations (1.95ms std) are negligible for most applications

### 3. **Root Cause**
The camera delay pattern suggests:
- **Isaac Sim rendering pipeline latency:** Camera frames take time to render
- **Variable computational load:** Rendering time varies based on scene complexity
- **Buffering/queuing:** Frames may be buffered before publishing
- **Not a fixed transport delay:** The variability rules out simple transmission delays

---

## Recommendations for OpenVINS Integration

### Option 1: Temporal Calibration (Recommended)
Since the delay is variable, use temporal calibration to estimate time offset:

```yaml
# In your OpenVINS config
temporal_calibration:
  enabled: true
  max_camera_imu_offset: 0.200  # 200ms max offset
  estimate_time_offset: true
```

**Pros:**
- Handles variable delays automatically
- OpenVINS will estimate and compensate for the offset
- Most robust solution

**Cons:**
- Requires additional state estimation
- May need more calibration data initially

### Option 2: Fixed Time Offset (Not Recommended)
Apply a fixed -76ms offset to camera timestamps:

```python
# Adjust camera timestamps
camera_msg.header.stamp = original_stamp - rclpy.duration.Duration(seconds=0.076)
```

**Pros:**
- Simple to implement
- No configuration changes needed

**Cons:**
- **Does not handle variability** (±24ms error remains)
- May cause inconsistent results
- Not recommended given the high CV (32%)

### Option 3: Approximate Time Synchronizer
Use ROS2's approximate time synchronizer with larger tolerance:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

camera_sub = Subscriber('/pegasus_1/rgb', Image)
imu_sub = Subscriber('/telemetry/imu', Imu)

# Allow 150ms slop to account for camera delay variability
sync = ApproximateTimeSynchronizer(
    [camera_sub, imu_sub],
    queue_size=10,
    slop=0.150  # 150ms tolerance
)
```

**Pros:**
- Handles variable delays
- Simple ROS2-based solution

**Cons:**
- May match incorrect message pairs if delay exceeds slop
- Not ideal for high-frequency VIO

### Option 4: Reduce Isaac Sim Camera Latency (Best Long-term Solution)
Investigate Isaac Sim settings to reduce rendering latency:

1. **Reduce camera resolution** if acceptable for your application
2. **Increase camera frame rate** to reduce buffering
3. **Check Isaac Sim rendering settings** (e.g., sync mode, physics time step)
4. **Disable unnecessary visual effects** (shadows, reflections, etc.)
5. **Use GPU-accelerated rendering** if not already enabled

---

## Next Steps

1. **Enable temporal calibration in OpenVINS** (Option 1)
2. **Monitor VIO performance** with current delay characteristics
3. **Investigate Isaac Sim camera settings** to reduce latency
4. **Consider camera downsampling** if high resolution isn't needed
5. **Re-run this analysis** after any configuration changes

---

## Files Generated

- `time_delay_data.json` - Raw data (1000 samples each)
- `time_delay_analysis_20260208_102958.png` - Visualization plots
- `ANALYSIS_RESULTS.md` - This summary report

---

## Visualization Summary

The plots show:
1. **Top plot:** Camera delay oscillates between 30-160ms, IMU stays near zero
2. **Middle plot:** Histogram shows camera delay distribution centered around 64-75ms
3. **Bottom plot:** Mean comparison clearly shows the 76ms gap

---

## Technical Notes

- Analysis performed using ROS2 Jazzy
- Clock source: `/clock` topic from Isaac Sim
- Camera topic: `/pegasus_1/rgb` (sensor_msgs/Image)
- IMU topic: `/telemetry/imu` (sensor_msgs/Imu)
- Time delay = (clock_at_reception - message_header_timestamp)
