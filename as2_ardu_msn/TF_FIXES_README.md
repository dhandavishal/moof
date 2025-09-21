# TF Extrapolation Fixes for AeroStack2 MAVROS Integration

## Issues Addressed

1. **Double RCL shutdown error** in mission script
2. **MAVROS timesync RTT spikes** causing TF timing issues  
3. **TF extrapolation into the future** warnings
4. **Platform node TF lookup timeouts**

## Changes Made

### 1. Mission Script Fix (`as2_ardu_msn/basic_survey_mission.py`)
- Added `if rclpy.ok():` guard before `rclpy.shutdown()`
- Prevents "rcl_shutdown already called" error during Ctrl+C

### 2. MAVROS Timing Configuration (`config/mavros_timing_fix.yaml`)
- Reduced timesync rate from 10Hz to 1Hz
- Increased connection timeouts
- Disabled unnecessary plugins that can cause timing issues

### 3. Launch File Updates (`launch/basic_aerostack2.launch.py`)
- Added MAVROS timing fix config to launch arguments
- Added TF timeout parameters to platform node

### 4. State Estimator Configuration (`config/state_estimator.yaml`)
- Added TF timeout and tolerance parameters
- Increased publish rates for smoother TF

### 5. Diagnostic Tool (`scripts/tf_diagnostic.py`)
- New script to monitor TF health and timing
- Helps verify fixes are working

## Testing the Fixes

### 1. Build the updated package:
```bash
cd ~/aerostack2_ws
colcon build --packages-select as2_ardu_msn
source install/setup.bash
```

### 2. Test the system:
```bash
# Terminal 1: Launch the system
ros2 launch as2_ardu_msn basic_aerostack2.launch.py

# Terminal 2: Monitor TF health
cd ~/aerostack2_ws/src/as2_ardu_msn/scripts
python3 tf_diagnostic.py

# Terminal 3: Check TF topics
ros2 topic list | grep tf
ros2 topic hz /tf
```

### 3. Expected improvements:
- Reduced "extrapolation into the future" warnings
- Lower MAVROS timesync RTT values
- Smoother mission execution
- Clean shutdown without RCL errors

## Manual Verification Commands

```bash
# Check system load (should be lower after fixes)
uptime

# Verify TF frames exist
ros2 run tf2_tools view_frames

# Test specific frame transforms
ros2 run tf2_ros tf2_echo earth base_link

# Monitor MAVROS connection health
ros2 topic echo /drone0/mavros/state --once
```

## Troubleshooting

If issues persist:

1. **High CPU usage**: Close unnecessary applications, especially multiple VS Code instances
2. **Network latency**: Ensure ArduPilot SITL and MAVROS are on same host
3. **Time sync**: Verify system NTP is working: `timedatectl`
4. **Missing frames**: Check state estimator is running: `ros2 node list | grep state_estimator`

## Further Optimizations

For extreme cases, you can:
- Set `timesync_rate: 0.0` to disable MAVROS timesync completely
- Increase platform delay in launch file from 10s to 15s
- Use `use_sim_time: true` if running in pure simulation

## Files Modified

- `as2_ardu_msn/basic_survey_mission.py` - Shutdown guard
- `config/mavros_timing_fix.yaml` - New MAVROS config
- `config/state_estimator.yaml` - TF timing parameters  
- `launch/basic_aerostack2.launch.py` - Config integration
- `scripts/tf_diagnostic.py` - New diagnostic tool
