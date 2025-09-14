# AS2 ArduPilot Mission Package

A clean, modular Aerostack2 package for conducting survey missions with ArduPilot SITL using a plug-and-play architecture.

## Project Structure

```
as2_ardu_msn/
├── basic_survey_mission.py          # Main survey mission script
├── config/                          # Configuration files
│   ├── config.yaml                  # Main AS2 configuration
│   ├── control_modes.yaml           # AS2 control modes
│   ├── mission_config.yaml          # Mission parameters
│   └── pid_speed_controller.yaml    # PID controller settings
├── launch/
│   └── basic_aerostack2.launch.py   # Complete AS2 launch file
├── scripts/
│   └── start_ardupilot_sitl.sh      # ArduPilot SITL startup script
├── package.xml                      # ROS2 package definition
├── setup.py                         # Python package setup
├── setup.cfg                        # Python package configuration
└── README_basic_mission.md          # This file
```

## Features

- **Standardized Mission Framework**: Modular design for easy mission customization
- **Plug-and-Play Configuration**: YAML-based parameter configuration
- **Complete AS2 Integration**: Includes all necessary AS2 behaviors and controllers
- **ArduPilot SITL Support**: Optimized for ArduPilot Software-in-the-Loop simulation
- **Lawnmower Survey Pattern**: Efficient area coverage for mapping/surveying
- **Safety Features**: Emergency landing, parameter validation, error handling

## Quick Start

### 1. Start ArduPilot SITL
```bash
# Terminal 1: Start ArduPilot SITL
cd ~/aerostack2_ws/src/as2_ardu_msn
./scripts/start_ardupilot_sitl.sh
```

### 2. Launch AS2 System
```bash
# Terminal 2: Launch complete AS2 system
cd ~/aerostack2_ws
source install/setup.bash
ros2 launch as2_ardu_msn basic_aerostack2.launch.py
```

### 3. Run Survey Mission
```bash
# Terminal 3: Execute survey mission
cd ~/aerostack2_ws
source install/setup.bash
python3 src/as2_ardu_msn/basic_survey_mission.py
```

## Mission Configuration

Edit `config/mission_config.yaml` to customize mission parameters:

```yaml
survey_parameters:
  # Survey area (meters from home position)
  area_width: 50.0      # Width of survey area
  area_length: 50.0     # Length of survey area
  altitude: 10.0        # Flight altitude
  
  # Pattern parameters
  line_spacing: 5.0     # Distance between survey lines
  
  # Flight speeds (m/s)
  survey_speed: 2.0     # Speed during survey lines
  transition_speed: 4.0 # Speed between waypoints
  
  # Mission options
  auto_start: false     # Require manual start confirmation
  rtl_on_completion: true # Return to launch after survey
```

## System Architecture

This package demonstrates a complete AS2 system integration:

- **AS2 Platform (Mavlink)**: Interfaces with ArduPilot via MAVROS
- **State Estimator**: Provides pose estimation using raw odometry
- **Motion Controller**: PID speed controller for smooth flight
- **Motion Behaviors**: Takeoff, landing, go_to, and follow_path behaviors
- **Mission Logic**: High-level mission coordination and execution

## Troubleshooting

### MAVROS Connection Issues
- Ensure ArduPilot SITL is running before launching AS2
- Check that MAVProxy outputs match MAVROS connection settings
- Verify UDP ports: ArduPilot SITL → MAVProxy → MAVROS

### Behavior Issues  
- Confirm all AS2 behavior nodes are running: `ros2 node list`
- Check behavior services: `ros2 service list | grep behavior`
- Monitor behavior status via logs or RQT

### Mission Execution Issues
- Verify drone is armed and in offboard mode before mission start
- Check mission configuration parameters are within safe limits
- Ensure proper coordinate frame alignment (NED vs ENU)

## Development Notes

- Uses AS2 Python API for clean mission scripting
- Modular configuration allows easy parameter tuning
- Error handling includes emergency landing procedures
- Compatible with AS2 behavior tree integration for complex missions

## Mission Phases

1. **Pre-flight**: Wait for systems, arm drone
2. **Takeoff**: Climb to survey altitude
3. **Survey**: Execute lawnmower pattern
4. **Return**: Go back to launch position
5. **Landing**: Automated landing and disarm

## Troubleshooting

- If behaviors are not available, check that all nodes are running
- If connection fails, verify MAVROS namespace configuration
- If mission fails, check ArduPilot SITL console for errors

## Safety

- Always run in simulation first
- Keep emergency stop ready (Ctrl+C)
- Monitor ArduPilot console for warnings
- Check GPS fix before flight (in real hardware)
