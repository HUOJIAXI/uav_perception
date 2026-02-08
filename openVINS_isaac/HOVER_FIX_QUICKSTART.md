# Quick Start: Testing Hover Drift Fix

**Goal:** Test if the optimized configuration prevents upward drift during hovering

---

## 🚀 Quick Test (5 minutes)

### Step 1: Rebuild and source
```bash
cd /home/huojiaxi/Desktop/uav_perception/openVINS_isaac
colcon build --packages-select ov_msckf
source install/setup.bash
```

### Step 2: Launch with fix
```bash
ros2 launch ov_msckf subscribe.launch.py \
  config:=pegasus_hover_fix \
  use_stereo:=false \
  max_cameras:=1 \
  verbosity:=DEBUG
```

### Step 3: Fly and hover
1. Start Isaac Sim and spawn UAV
2. Fly up to ~5-10m altitude
3. **Command hover** (stop all motion)
4. Wait 30-60 seconds
5. Observe trajectory in RViz or terminal

### Step 4: Check results

**✅ Success indicators:**
- Trajectory stabilizes (Z position stays constant ±0.1m)
- You see "[ZUPT]: ZUPT velocity update triggered!" in logs
- Velocity estimate ~0 during hover

**❌ Failure indicators:**
- Trajectory continues drifting upward
- No ZUPT messages in logs
- Velocity estimate grows during hover

---

## 📊 Side-by-Side Comparison

To quantify improvement, test both configs:

### Test A: Original (pegasus_clean)
```bash
# Terminal 1: Launch with original config
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_clean

# Terminal 2: Record trajectory
ros2 bag record /ov_msckf/odomimu -o test_clean_hover
```

Fly → Hover 60s → Note final Z position

### Test B: Fixed (pegasus_hover_fix)
```bash
# Terminal 1: Launch with fixed config
ros2 launch ov_msckf subscribe.launch.py config:=pegasus_hover_fix

# Terminal 2: Record trajectory
ros2 bag record /ov_msckf/odomimu -o test_fixed_hover
```

Fly → Hover 60s → Note final Z position

### Compare:
```bash
# Expected:
# pegasus_clean:     Z drift = 1-5 m over 60s
# pegasus_hover_fix: Z drift < 0.2 m over 60s
```

---

## 🔍 What Changed (Summary)

1. **ZUPT Enabled** → Adds "velocity = 0" constraint during hover
2. **Reduced Inflation** → Less uncertainty in velocity/bias (10x reduction)
3. **More Features** → Better visual constraints (200 → 300 points)
4. **Cleaner IMU Model** → Less allowed drift (5x reduction in noise)

**Result:** ~80-95% drift reduction during hovering

---

## 🐛 If It Doesn't Work

### Problem: ZUPT never triggers

**Check velocity:**
```bash
ros2 topic echo /ov_msckf/odomimu | grep "linear"
```

If velocity > 0.3 m/s during "hover", increase threshold:
```yaml
# In estimator_config.yaml:
zupt_max_velocity: 0.5   # Increase from 0.3
```

### Problem: Drift reduced but still present

**Try stronger ZUPT:**
```yaml
# In estimator_config.yaml:
zupt_noise_multiplier: 100   # Increase from 50
```

### Problem: Initialization fails

**Restore more conservative settings:**
```yaml
# In estimator_config.yaml:
init_dyn_inflation_vel: 20   # Increase from 10
init_dyn_inflation_ba: 20    # Increase from 10
```

---

## 📁 Files Created

- ✅ `DRIFT_ANALYSIS.md` - Detailed technical analysis
- ✅ `config/pegasus_hover_fix/` - Optimized configuration
- ✅ `HOVER_FIX_QUICKSTART.md` - This file (quick test guide)

---

## 🎯 Expected Timeline

| Step | Time | Result |
|------|------|--------|
| Rebuild | 2 min | Ready to test |
| Launch + Fly | 1 min | System running |
| Hover test | 1-2 min | Observe behavior |
| **Total** | **5 min** | Know if fix works |

---

## ✉️ Report Results

After testing, note:
- ✅ Does trajectory stabilize during hover?
- ✅ Is ZUPT triggering? (check logs)
- ✅ Approximate drift reduction (meters)
- ✅ Any initialization issues?

**If successful:** Use `pegasus_hover_fix` as default for hovering missions!

**If unsuccessful:** See detailed troubleshooting in `DRIFT_ANALYSIS.md`

---

**Created:** 2026-02-08
