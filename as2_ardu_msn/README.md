# AS2 ArduPilot Survey Mission

A clean, minimal Aerostack2 package for ArduPilot survey missions.

## Contents

### Core Files
- `package.xml` - ROS package definition
- `setup.py` - Python package setup
- `setup.cfg` - Python setup configuration

### Mission
- `as2_ardu_msn/basic_survey_mission.py` - Main survey mission script

### Launch
- `launch/basic_aerostack2.launch.py` - Complete system launch file

### Configuration
- `config/mission_config.yaml` - Mission parameters (survey area, altitude, etc.)
- `config/mavros_timing_fix.yaml` - MAVROS configuration with RTT fixes
- `config/platform_params.yaml` - Platform parameters
- `config/control_modes.yaml` - Available control modes
- `config/state_estimator.yaml` - State estimator configuration
- `config/motion_controller.yaml` - Motion controller configuration (fixed frames)
- `config/pid_speed_controller.yaml` - PID controller parameters
- `config/motion_behaviors.yaml` - Behavior configuration (takeoff, navigation, etc.)

## Usage

```bash
# Launch the complete system
ros2 launch as2_ardu_msn basic_aerostack2.launch.py

# Or with custom mission config
ros2 launch as2_ardu_msn basic_aerostack2.launch.py mission_config:=/path/to/custom_config.yaml
```

## Key Fixes Applied

1. **Motion Controller**: Fixed frame alignment (`drone0/odom`, `drone0/base_link`) and disabled bypass
2. **ArduPilot Compatibility**: Uses GUIDED mode instead of OFFBOARD
3. **High RTT Tolerance**: Improved timing for communication delays
4. **Clean Structure**: Removed all unused files and dependencies

This package now contains only the essential files needed for a functioning ArduPilot survey mission with Aerostack2.
