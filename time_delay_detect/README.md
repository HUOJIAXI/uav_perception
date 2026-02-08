# Time Delay Detection for OpenVINS

This directory contains scripts to analyze time delays between camera topics, IMU topics, and the /clock topic in ROS2.

## Problem
When using Isaac Sim camera helper with OpenVINS for UAV localization:
- Camera topics (`/pegasus_1/rgb`, `/pegasus_1/depth`) show time delays relative to `/clock`
- IMU topic (`/telemetry/imu`) is synchronized with `/clock`
- Need to determine if the delay is constant or variable

## Scripts

### 1. `time_delay_analyzer.py`
Real-time analyzer that subscribes to topics and calculates time delays.

**Features:**
- Subscribes to `/clock`, `/pegasus_1/rgb`, and `/telemetry/imu`
- Calculates delay between message timestamp and clock time
- Provides periodic statistics (every 5 seconds)
- Determines if delay is constant (std < 1ms) or variable
- Saves data to JSON file on exit for further analysis

**Usage:**
```bash
# Run the analyzer (make sure your ROS2 simulation is running)
python3 time_delay_analyzer.py

# Or use ROS2 run
ros2 run <your_package> time_delay_analyzer.py

# Press Ctrl+C to stop and save data
```

**Output:**
- Real-time console output with statistics
- Saves `time_delay_data.json` with all collected measurements

### 2. `plot_time_delays.py`
Visualization tool for analyzing saved delay data.

**Features:**
- Loads data from JSON file
- Creates 3 plots:
  1. Time delay over time (shows trends and variations)
  2. Histogram of delays (shows distribution)
  3. Mean delay comparison (Camera vs IMU)
- Prints detailed statistics including coefficient of variation
- Saves plot as PNG file

**Usage:**
```bash
# After running time_delay_analyzer.py and collecting data
python3 plot_time_delays.py

# Or specify a custom data file
python3 plot_time_delays.py my_data.json
```

**Output:**
- PNG file with timestamp: `time_delay_analysis_YYYYMMDD_HHMMSS.png`
- Console output with detailed statistics

## Workflow

1. **Start your simulation** (Isaac Sim with ROS2 topics publishing)

2. **Run the analyzer:**
   ```bash
   python3 time_delay_analyzer.py
   ```

3. **Let it collect data** for 30-60 seconds to get good statistics

4. **Stop with Ctrl+C** - this saves the data automatically

5. **Visualize the results:**
   ```bash
   python3 plot_time_delays.py
   ```

6. **Interpret the results:**
   - **Constant delay**: Standard deviation < 1ms, CV < 5%
     - Can be compensated with a fixed time offset
   - **Variable delay**: Standard deviation >= 1ms, CV >= 5%
     - Requires time synchronization or temporal calibration
     - May need to use approximate time sync in OpenVINS

## Understanding the Output

### Key Metrics:
- **Mean delay**: Average time difference between message timestamp and clock
- **Std deviation**: How much the delay varies
- **Coefficient of Variation (CV)**: Normalized variability measure (Std/Mean × 100%)
  - CV < 5%: Delay is quite constant
  - CV 5-15%: Moderate variation
  - CV > 15%: High variation

### For OpenVINS Integration:
- If camera delay is **constant**: Apply fixed time offset to camera timestamps
- If camera delay is **variable**: Consider using approximate time synchronizer or temporal calibration
- Compare camera vs IMU delays to understand synchronization issues

## Dependencies
- ROS2 (tested with Humble/Foxy)
- Python packages: rclpy, numpy, matplotlib
- Install with: `pip install numpy matplotlib`

## Troubleshooting

**No data collected:**
- Verify topics are publishing: `ros2 topic list`
- Check topic rates: `ros2 topic hz /pegasus_1/rgb`
- Ensure simulation is running

**Permission errors:**
- Make scripts executable: `chmod +x *.py`

**Import errors:**
- Install dependencies: `pip install numpy matplotlib rclpy`
