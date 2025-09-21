# Takeoff Failure Troubleshooting Guide

## Problem Analysis

Your mission shows the drone:
1. ✅ **Arms successfully** (FCU: "Arming motors")
2. ✅ **Sets GUIDED mode** correctly
3. ✅ **Takeoff behavior starts** (`TakeoffBehavior: START`, `TAKING_OFF` state)
4. ❌ **Never gains altitude** (feedback shows `actual_takeoff_height=0.0` continuously)
5. ❌ **FCU disarms after ~10s** (ArduPilot failsafe triggered)

## Root Causes

### 1. ArduPilot EKF Not Ready
ArduPilot requires a good position estimate before allowing takeoff. Common issues:
- EKF not converged
- GPS lock insufficient (if GPS enabled)
- Barometer not calibrated
- Compass interference

### 2. Throttle/Motor Configuration
- Motor output range incorrect
- Throttle failsafe engaged
- PWM values not reaching motors

### 3. Aerostack2 State Estimation Issues
- `raw_odometry` plugin not receiving proper MAVROS data
- Transform chain broken (`earth` -> `odom` -> `base_link`)
- Position estimates unreliable

### 4. ArduPilot Parameter Issues
- `ARMING_CHECK` preventing full operation
- `THR_MIN`/`MOT_THR_MIN` too low
- EKF type misconfigured

## Diagnostic Steps

### Step 1: Run ArduPilot Diagnostics

```bash
cd ~/aerostack2_ws
source install/setup.bash

# In one terminal, start the system
ros2 launch as2_ardu_msn basic_aerostack2.launch.py

# In another terminal, run diagnostics
ros2 run as2_ardu_msn ardupilot_diagnostics.py
```

Look for:
- **GPS Status**: Should be "FIX" or better
- **Extended State**: `landed_state` should be "ON_GROUND"
- **EK3_ENABLE**: Should be 1
- **ARMING_CHECK**: Check if too restrictive

### Step 2: Check MAVROS Topics

```bash
# Check if odometry is being published
ros2 topic hz /drone0/mavros/local_position/odom

# Check local position data
ros2 topic echo /drone0/mavros/local_position/pose --once

# Check extended state
ros2 topic echo /drone0/mavros/extended_state --once
```

### Step 3: Manual Motor Test

```bash
# Check if motors can spin (SITL)
rostopic pub /drone0/mavros/actuator_control \
  mavros_msgs/ActuatorControl \
  "header: {stamp: now}
   group_motor: 0
   controls: [0.2, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0]"
```

## Fixes Applied

### 1. Enhanced Mission Script

Added diagnostics and fallback strategies:
- Pre-takeoff condition checks
- Multi-attempt takeoff with monitoring
- Real-time altitude tracking during takeoff
- Timeout handling (30s per attempt)

### 2. State Estimator Configuration

Updated to use `raw_odometry` plugin:
- Simpler than `ground_truth` for ArduPilot SITL
- Direct MAVROS odometry consumption
- Proper frame naming without namespaces

### 3. ArduPilot Diagnostics Tool

Created comprehensive monitoring:
- Real-time parameter checking
- State and position monitoring
- EKF and GPS status tracking

## Quick Fixes to Try

### Fix 1: Reduce ArduPilot Safety Checks

```bash
# Connect to SITL console (if accessible) and set:
param set ARMING_CHECK 1      # Reduce checks (was likely 1022)
param set EK3_GPS_CHECK 0     # Disable GPS check for indoor flight
param set GPS_TYPE 0          # Disable GPS entirely for SITL
```

### Fix 2: Force Position Mode

Add to mission script before takeoff:
```python
# Set position mode explicitly
req = SetMode.Request()
req.custom_mode = 'POSHOLD'  # or 'LOITER'
future = self.set_mode_client.call_async(req)
time.sleep(2)
```

### Fix 3: Manual Throttle Test

```python
# Add direct throttle command before takeoff
from mavros_msgs.msg import OverrideRCIn
rc_override = OverrideRCIn()
rc_override.channels = [1500, 1500, 1200, 1500, 0, 0, 0, 0]  # Low throttle
```

## Next Steps

1. **Build and test** the updated package:
   ```bash
   cd ~/aerostack2_ws
   colcon build --packages-select as2_ardu_msn
   source install/setup.bash
   ```

2. **Run with diagnostics**:
   ```bash
   # Terminal 1: Main system
   ros2 launch as2_ardu_msn basic_aerostack2.launch.py
   
   # Terminal 2: ArduPilot diagnostics
   ros2 run as2_ardu_msn ardupilot_diagnostics.py
   
   # Terminal 3: TF diagnostics (optional)
   ros2 run as2_ardu_msn tf_diagnostics.py
   ```

3. **Monitor the enhanced takeoff** - it will now show:
   - Pre-takeoff condition checks
   - Real-time altitude progress
   - Multiple attempts if needed
   - Clear failure reasons

## Expected Improvements

With the fixes:
- **Better diagnostics**: Clear visibility into why takeoff fails
- **Retry logic**: Up to 3 takeoff attempts with 5s delays
- **Altitude monitoring**: Real-time feedback on climbing progress
- **ArduPilot status**: Continuous monitoring of EKF, GPS, parameters
- **Simpler state estimation**: Less complex transform chain

The enhanced mission will provide much clearer error messages to help identify the specific issue preventing takeoff.
