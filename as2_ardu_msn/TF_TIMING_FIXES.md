# TF Timing Fixes - ROS 2 Compatible Solution

## Problem
The mission was experiencing TF "extrapolation into the future" warnings caused by:
1. Inconsistent time bases across nodes (`use_sim_time` mismatch)
2. Short TF timeout in Aerostack2 platform (default 0.05s insufficient for 6-10ms lag)
3. Missing TF readiness check before flight operations

## Solution Applied

### 1. Unified Time Base (`basic_aerostack2.launch.py`)
- Changed `use_sim_time` default from `false` to `true` for consistency
- Updated platform node to use `use_sim_time` parameter instead of hardcoded `False`
- Updated mission node DroneInterface to use `use_sim_time=True`

### 2. Increased TF Timeouts (`basic_aerostack2.launch.py`)
- Increased `tf_timeout_threshold` from 0.1s to 0.15s to handle observed TF lag
- Increased `tf_timeout` from 0.15s to 0.2s for TF lookups
- These changes accommodate the 6-10ms timing delays seen in logs

### 3. ROS 2-Compatible TF Readiness Check (`basic_survey_mission.py`)
- Added `wait_for_tf_ready()` method using proper TF2 buffer/listener
- Checks for `earth -> drone0/base_link` transform availability before arming
- Uses bounded timeout (15s) with consecutive success requirement (3 checks)
- Replaces problematic ROS 1 busy-wait patterns with ROS 2 best practices
- Added as "Phase 0" before arming/takeoff operations

## Key Differences from Forum Hack

**Forum approach (problematic):**
- Tight busy-wait loop with `canTransform()`
- Creates duplicate TF listeners in hot paths
- Blocks ROS 2 executors
- No timeout bounds

**Our ROS 2 approach (proper):**
- Uses shared node's TF buffer/listener
- Bounded timeout with reasonable delays
- Allows executor to process other callbacks
- Requires consecutive successes for stability
- Uses `Time()` (zero time) for latest available transform

## Testing

Build and test with:
```bash
cd /home/dhandavishal/aerostack2_ws
colcon build --packages-select as2_ardu_msn
source install/setup.bash
ros2 launch as2_ardu_msn basic_aerostack2.launch.py
```

## Expected Results

- Elimination of "extrapolation into the future" warnings
- Stable TF lookups during mission execution
- Proper startup sequencing with TF readiness verification
- No executor blocking or duplicate TF listeners
- Consistent timing across all nodes

## Technical Notes

- The TF readiness check runs before arming, catching timing issues early
- Using `Time()` requests latest available transform, avoiding future-time requests
- Consecutive success requirement ensures TF stability, not just momentary availability
- Timeout parameters accommodate real-world timing variations in SITL/hardware setups
