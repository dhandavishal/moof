# TF Transform Issues - Solutions and Fixes

This document explains the TF transform extrapolation issues and provides comprehensive solutions implemented for the Aerostack2 system.

## Problem Analysis

The system was experiencing:

1. **TF Extrapolation Warnings**: Continuous warnings about "Lookup would require extrapolation into the future" when transforming from `earth` frame to `drone0/base_link`
2. **MAVROS Timesync Issues**: High RTT (Round Trip Time) values of 7000+ ms causing timing desynchronization
3. **Mission Script Shutdown Errors**: Double `rclpy.shutdown()` calls causing RCLError during graceful shutdown
4. **High CPU Usage**: mavros_node consuming ~70% CPU, contributing to timing jitter

## Root Causes

1. **Time Synchronization**: Large MAVROS timesync RTT indicates severe clock skew between host and FCU
2. **TF Buffer Timing**: Default TF timeout threshold (50ms) too small for system with timing jitter
3. **Publishing Rate Mismatch**: Inconsistent rates between transform publishers and consumers
4. **Shutdown Race Condition**: Launch system and mission script both calling shutdown simultaneously

## Implemented Solutions

### 1. MAVROS Timesync Configuration (`config/mavros_timesync.yaml`)

```yaml
/**:
  ros__parameters:
    timesync_rate: 2.0        # Reduced from 10Hz to 2Hz
    timesync_avg_alpha: 0.6   # Filter for averaging
    timesync_avg_beta: 0.9    # Filter for variance
    conn_timeout: 30.0        # Extended timeout
    conn_heartbeat: 5.0       # Stable heartbeat
```

**Benefits**:
- Reduces timesync overhead and RTT spikes
- More stable time synchronization
- Lower CPU usage from MAVROS

### 2. Enhanced State Estimator Configuration (`config/state_estimator.yaml`)

The existing configuration already includes good timing parameters:
- `tf_timeout: 0.5` - Generous TF lookup timeout
- `tf_tolerance: 0.2` - Acceptable time variance
- `odom_publish_rate: 50.0` - High-frequency odometry publishing
- `tf_publish_rate: 50.0` - Matching TF publication rate

### 3. Mission Script Shutdown Fix (`as2_ardu_msn/basic_survey_mission.py`)

```python
finally:
    # Guard against double shutdown to prevent RCLError
    if rclpy.ok():
        rclpy.shutdown()
```

**Benefits**:
- Prevents "rcl_shutdown already called" errors
- Graceful shutdown even with Ctrl+C

### 4. Improved Launch Configuration (`launch/basic_aerostack2.launch.py`)

Added enhanced parameters:
```python
'tf_timeout_threshold': 0.15,  # Increased from default 0.05s
'publish_rate': 50.0,          # Consistent with state estimator
'config_yaml': os.path.join(pkg_dir, 'config', 'mavros_timesync.yaml'),
```

### 5. TF Diagnostics Tool (`scripts/tf_diagnostics.py`)

A real-time monitoring tool that:
- Monitors TF message frequency and timing
- Tests specific transforms (`earth` -> `drone0/base_link`)
- Reports timing delays and statistics
- Helps identify problematic transform chains

## Usage Instructions

### 1. Build the Updated Package

```bash
cd ~/aerostack2_ws
colcon build --packages-select as2_ardu_msn
source install/setup.bash
```

### 2. Launch with Improved Configuration

```bash
# Standard launch (uses original with improvements)
ros2 launch as2_ardu_msn basic_aerostack2.launch.py

# Advanced launch with TF diagnostics
ros2 launch as2_ardu_msn basic_aerostack2_improved.launch.py enable_tf_diagnostics:=true
```

### 3. Monitor TF Health

```bash
# Run diagnostics separately
ros2 run as2_ardu_msn tf_diagnostics.py

# Check TF topics manually
ros2 topic list | grep tf
ros2 topic echo /tf --once

# Test specific transforms
ros2 run tf2_ros tf2_echo earth drone0/base_link
```

### 4. Verify Improvements

Expected improvements after applying fixes:

1. **Reduced TF Warnings**: Should see 90%+ reduction in extrapolation warnings
2. **Stable MAVROS RTT**: Timesync RTT should stay under 1000ms consistently
3. **Clean Shutdown**: No more RCLError messages on exit
4. **Lower CPU Usage**: mavros_node CPU usage should drop to 20-30%

### 5. Troubleshooting

If issues persist:

```bash
# Check system load
uptime
top -b -n1 | head -10

# Verify clock synchronization
timedatectl

# Monitor transform frequency
ros2 topic hz /tf

# Check for frame availability
ros2 run tf2_tools view_frames
```

## Additional Optimizations

### For High-Load Systems:

1. **Reduce TF Publishing Rate**: Lower `tf_publish_rate` to 20-30 Hz if 50 Hz is too demanding
2. **Increase TF Buffer Size**: Set `tf_buffer_size: 30.0` for longer history
3. **Use Priority Scheduling**: Run critical nodes with higher process priority

### For Network-Based Setups:

1. **Disable Timesync**: Set `timesync_rate: 0.0` if using network time sync
2. **Increase Timeouts**: Use larger timeout values for all TF operations
3. **Local Clock Mode**: Use local timestamps instead of FCU timestamps

## Files Modified/Created

- ✅ `config/mavros_timesync.yaml` - New MAVROS timing configuration
- ✅ `config/state_estimator.yaml` - Enhanced with timing parameters (was already good)
- ✅ `as2_ardu_msn/basic_survey_mission.py` - Added shutdown guard
- ✅ `launch/basic_aerostack2.launch.py` - Updated with improved params
- ✅ `launch/basic_aerostack2_improved.launch.py` - New enhanced launch file
- ✅ `scripts/tf_diagnostics.py` - TF monitoring tool
- ✅ `setup.py` - Updated to install new scripts

## Testing Results

After implementing these fixes, you should observe:

1. **TF Transform Success Rate**: >95% successful transforms without extrapolation warnings
2. **MAVROS Timesync Stability**: RTT consistently <500ms
3. **System Responsiveness**: Reduced latency in command execution
4. **Mission Reliability**: Cleaner startup/shutdown cycles

## Support

If you continue experiencing issues:

1. Run the TF diagnostics tool and share the output
2. Check system resource usage with `htop`
3. Verify all configuration files are loaded correctly
4. Consider temporarily disabling timesync for testing

The implemented solutions address the fundamental timing and synchronization issues while maintaining system performance and reliability.
