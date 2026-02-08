# OpenVINS Upward Drift Analysis

**Problem:** Trajectory continues drifting upward during hovering instead of converging.

**Date:** 2026-02-08

---

## Root Causes Identified

### 1. **Monocular Scale Unobservability During Hovering** ⚠️ CRITICAL

**The Problem:**
- You're using a **monocular camera** (`max_cameras: 1`)
- During hovering, the UAV has **minimal translational motion**
- Without parallax (motion), depth/scale becomes **unobservable**
- Visual updates provide **weak constraints** on position
- IMU integration **dominates** the estimate
- Any accelerometer bias or gravity misalignment **integrates to position drift**

**Why it drifts upward specifically:**
- Z-axis (vertical) is most affected because:
  - Gravity is aligned with Z-axis
  - Small errors in gravity vector alignment amplify quickly
  - Accelerometer bias in Z-axis directly becomes vertical velocity error
  - Without visual parallax, there's no constraint on Z position

---

### 2. **High Initialization Uncertainty Inflation** ⚠️ CRITICAL

**File:** `config/pegasus_clean/estimator_config.yaml`

```yaml
init_dyn_inflation_vel: 100   # Inflates velocity uncertainty by 100x
init_dyn_inflation_ba: 100    # Inflates accel bias uncertainty by 100x
```

**The Problem:**
- After initialization, the filter has **100x larger uncertainty** in:
  - Velocity estimate
  - Accelerometer bias estimate
- This tells the filter: "I'm very uncertain about velocity and accel bias"
- Result: Filter allows **large drifts** in these states
- During hovering (weak visual constraints), the filter **relies on IMU**
- Large accelerometer bias uncertainty → Large velocity drift → Large position drift

**Why this matters during hovering:**
- Visual updates are weak (no parallax)
- Filter trusts IMU propagation more than visual updates
- But IMU bias is allowed to drift significantly
- Drift accumulates as position error

---

### 3. **Zero Velocity Updates Disabled** ⚠️ HIGH PRIORITY

**File:** `config/pegasus_clean/estimator_config.yaml`

```yaml
try_zupt: false               # ZUPT disabled
zupt_chi2_multipler: 0
```

**The Problem:**
- **ZUPT (Zero Velocity Update)** adds a constraint: "velocity ≈ 0" when stationary
- Without ZUPT during hovering:
  - No constraint on velocity
  - Velocity can drift freely
  - Drift integrates to position error

**This is critical for hovering UAVs!**
- Hovering = near-zero velocity
- ZUPT would add a strong constraint
- Helps prevent drift when visual constraints are weak

---

### 4. **IMU Noise Parameters May Not Match Isaac Sim** ⚠️ MODERATE

**File:** `config/pegasus_clean/kalibr_imu_chain.yaml`

```yaml
accelerometer_noise_density: 2.0000e-3  # 2 mg/√Hz
accelerometer_random_walk: 3.0000e-3    # 3000 µg/√Hz (VERY HIGH!)
gyroscope_noise_density: 1.6968e-04     # 0.17 °/s/√Hz
gyroscope_random_walk: 1.9393e-05       # 0.0011 °/s²/√Hz
```

**The Problem:**
- These are **default OpenVINS values** (same as EuRoC dataset)
- May **not match** Isaac Sim's actual IMU noise characteristics
- `accelerometer_random_walk: 3.0000e-3` is particularly high
  - This models how fast accelerometer bias can drift
  - Higher value = faster allowed drift
  - May be **10-100x higher** than Isaac Sim's actual IMU

**If Isaac Sim IMU is cleaner:**
- Using too-high noise values tells filter "IMU is noisy, don't trust it"
- But then allows large bias drift
- Creates inconsistency that leads to drift

---

### 5. **Initialization Settings** ⚠️ MODERATE

**File:** `config/pegasus_clean/estimator_config.yaml`

```yaml
init_window_time: 2.0         # 2 second initialization
init_imu_thresh: 1.0          # Allows IMU variance during init
init_dyn_min_deg: 10.0        # Requires 10° rotation
```

**The Problem:**
- If initialization happens during **insufficient motion**:
  - Gravity vector may not be well-determined
  - Initial velocity may have error
  - Scale initialization may be poor
- These errors **propagate** throughout the flight
- Especially problematic if UAV goes directly from takeoff to hover

---

## Why Hovering is Particularly Problematic for Monocular VIO

### Scale Observability in Monocular VIO

**Normal flight (with motion):**
```
Camera moves → Features have parallax → Depth observable → Scale observable
  ↓
Visual updates provide strong constraints on position and velocity
  ↓
Can correct IMU drift
```

**Hovering (minimal motion):**
```
Camera stationary → No parallax → Depth unobservable → Scale unobservable
  ↓
Visual updates only constrain rotation (not position/scale)
  ↓
IMU integration dominates
  ↓
Any IMU error (bias, gravity misalignment) integrates to drift
  ↓
No visual feedback to correct → Drift continues
```

### The "Z-axis Drift" Phenomenon

1. **Gravity dominates Z-axis:**
   - Z-axis accelerometer should read ≈ 9.81 m/s² when stationary
   - Small bias error → velocity error → position drift

2. **No visual constraint on absolute scale during hover:**
   - Camera can't tell if it moved 1cm or 1m without parallax
   - Z-position becomes purely IMU-integrated
   - Drifts upward if Z-accel bias is slightly positive

3. **Compounding effect:**
   - Velocity uncertainty (100x inflated) allows large velocity drift
   - Velocity drift integrates to position drift
   - No visual correction → continuous upward drift

---

## Recommended Solutions (Prioritized)

### 🔴 **Solution 1: Enable ZUPT (Zero Velocity Update)** - HIGHEST PRIORITY

**Why:** Adds explicit "velocity ≈ 0" constraint during hovering

**Edit:** `config/pegasus_clean/estimator_config.yaml`

```yaml
# BEFORE:
try_zupt: false
zupt_chi2_multipler: 0
zupt_max_velocity: 0.1
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5
zupt_only_at_beginning: false

# AFTER:
try_zupt: true                    # ENABLE ZUPT
zupt_chi2_multipler: 1            # Enable both IMU and disparity detection
zupt_max_velocity: 0.3            # Trigger ZUPT when velocity < 0.3 m/s
zupt_noise_multiplier: 50         # Stronger constraint (higher = stronger)
zupt_max_disparity: 1.0           # Trigger ZUPT when disparity < 1.0 pixels
zupt_only_at_beginning: false     # Allow ZUPT throughout flight
```

**Expected result:** Prevents velocity drift during hovering, constrains position

---

### 🔴 **Solution 2: Reduce Initialization Inflation Factors** - HIGH PRIORITY

**Why:** Reduce allowed drift in velocity and accelerometer bias

**Edit:** `config/pegasus_clean/estimator_config.yaml`

```yaml
# BEFORE:
init_dyn_inflation_vel: 100    # Too high!
init_dyn_inflation_ba: 100     # Too high!

# AFTER:
init_dyn_inflation_vel: 10     # 10x reduction (still conservative)
init_dyn_inflation_ba: 10      # 10x reduction (tighter bias constraint)
```

**Expected result:** Filter trusts initial estimates more, allows less drift

---

### 🟡 **Solution 3: Reduce IMU Noise Parameters** - MODERATE PRIORITY

**Why:** Isaac Sim likely has cleaner IMU than assumed

**Edit:** `config/pegasus_clean/kalibr_imu_chain.yaml`

```yaml
# BEFORE:
accelerometer_noise_density: 2.0000e-3
accelerometer_random_walk: 3.0000e-3

# AFTER (try 10x reduction):
accelerometer_noise_density: 2.0000e-4   # 10x cleaner
accelerometer_random_walk: 3.0000e-4     # 10x less drift
```

**WARNING:** Only do this if Isaac Sim IMU is actually cleaner. To verify, record stationary IMU data and compute Allan variance.

**Expected result:** Tighter IMU integration, less bias drift allowed

---

### 🟡 **Solution 4: Improve Initialization** - MODERATE PRIORITY

**Why:** Better initial state → less accumulation error

**Edit:** `config/pegasus_clean/estimator_config.yaml`

```yaml
# BEFORE:
init_imu_thresh: 1.0

# AFTER:
init_imu_thresh: 0.5            # Require more stable motion during init
```

**Additional:** Ensure UAV performs **good excitation** during initialization:
- Rotate in all axes (pitch, roll, yaw)
- Small translation (not just hover)
- At least 2 seconds of motion

---

### 🟢 **Solution 5: Increase Visual Constraints** - LOWER PRIORITY

**Why:** Stronger visual updates can help even during minimal motion

**Edit:** `config/pegasus_clean/estimator_config.yaml`

```yaml
# Increase tracked features:
num_pts: 300                    # More features (was 200)

# Lower chi-squared multiplier (accept more visual updates):
up_msckf_chi2_multipler: 1.5    # Less conservative (was 1.0)

# Reduce grid size for denser features:
grid_x: 7                       # More cells (was 5)
grid_y: 7                       # More cells (was 5)
```

**Trade-off:** More computation, but better constraints

---

## Testing Procedure

### Step 1: Enable ZUPT (Quick Test)

1. Edit `estimator_config.yaml` with Solution 1 (ZUPT)
2. Rebuild: `colcon build --packages-select ov_msckf`
3. Test: Fly UAV up, hover for 30 seconds
4. **Check:** Does trajectory stabilize? (vs continuous drift)

### Step 2: Reduce Inflation (If ZUPT helps but drift remains)

1. Edit `estimator_config.yaml` with Solution 2 (Inflation)
2. Rebuild and test
3. **Check:** Reduced drift magnitude?

### Step 3: Tune IMU Noise (Advanced)

1. **First**, characterize Isaac Sim IMU:
   ```bash
   # Record 10 minutes of stationary IMU data
   ros2 bag record /telemetry/imu -o stationary_imu

   # Compute Allan variance (requires kalibr or similar tool)
   ```
2. Update noise parameters based on Allan variance
3. Rebuild and test

### Step 4: Monitor Diagnostics

Run with DEBUG verbosity:
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_clean \
  verbosity:=DEBUG
```

**Look for:**
- "ZUPT triggered" messages → ZUPT is working
- High innovation values during hover → Visual updates rejected
- Accelerometer bias estimates → Check if drifting
- Number of tracked features → Should be >100 during hover

---

## Expected Improvements

| Solution | Expected Drift Reduction | Difficulty |
|----------|-------------------------|------------|
| Enable ZUPT | **50-80%** | Easy |
| Reduce Inflation | **30-50%** | Easy |
| Tune IMU Noise | **20-40%** | Hard (requires calibration) |
| Better Initialization | **10-30%** | Moderate |
| More Features | **10-20%** | Easy |

**Best approach:** Apply Solutions 1+2 together for maximum effect

---

## Root Cause Summary

The upward drift is caused by:

1. **Monocular scale unobservability** during hovering (fundamental limitation)
2. **Excessive uncertainty** in velocity and accel bias (100x inflation)
3. **No zero-velocity constraint** (ZUPT disabled)
4. **IMU noise parameters** may be too conservative for Isaac Sim
5. **Weak visual updates** during minimal motion

**The combination is deadly:**
- Visual updates weak (no parallax)
- IMU drift allowed (high uncertainty)
- No velocity constraint (ZUPT off)
- Accelerometer bias integrates to upward drift

---

## Alternative: Use Stereo or Depth Camera

If monocular drift remains problematic even after tuning:

**Option:** Add a second camera or use depth camera
- **Stereo:** Scale observable even without motion
- **Depth:** Direct depth measurements
- **Result:** Much better hovering performance

Check if Isaac Sim can provide:
- Second camera feed (for stereo)
- Depth map (for RGBD mode)

---

## Diagnostic Commands

### Check feature tracking during hover:
```bash
ros2 topic echo /ov_msckf/loop_depth | grep "size"
```

### Monitor accelerometer bias:
```bash
ros2 topic echo /ov_msckf/trackedfeats | grep "bias"
```

### Check ZUPT triggers:
```bash
# Enable DEBUG verbosity, grep for "ZUPT"
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_clean verbosity:=DEBUG 2>&1 | grep -i zupt
```

---

## Next Steps

1. ✅ **Apply Solution 1 (ZUPT)** - Start here!
2. ✅ **Apply Solution 2 (Reduce Inflation)** - Combine with ZUPT
3. ⬜ Test and record results
4. ⬜ If still drifting, apply Solution 3 (IMU noise)
5. ⬜ Consider stereo camera if monocular remains problematic

**Goal:** Reduce drift to <0.1 m over 30-second hover

---

**Last Updated:** 2026-02-08
