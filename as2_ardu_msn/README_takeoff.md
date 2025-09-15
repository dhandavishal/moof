# Takeoff Mission - Aerostack2

A simple takeoff mission that uses Aerostack2 to take off to a specified altitude and land.

## Features

- **Configurable Altitude**: Default 100m (configurable in config file)
- **Speed Control**: Configurable takeoff and landing speeds
- **Hold Pattern**: Holds position at altitude for specified time
- **Optional Waypoint**: Can navigate to a waypoint before landing
- **Safety Features**: Emergency landing and timeout protection

## Quick Start

1. **Start your ArduPilot SITL** (make sure it's running on default ports)

2. **Launch the takeoff mission**:
   ```bash
   cd /home/dhandavishal/aerostack2_ws
   source install/setup.bash
   ros2 launch as2_ardu_msn takeoff_aerostack2.launch.py
   ```

3. **Wait for system to initialize** (you'll see the mission ready message)

4. **Press Enter to start the mission**

## Configuration

Edit `/config/takeoff_config.yaml` to customize:

```yaml
takeoff_parameters:
  altitude: 100.0          # Target altitude in meters
  takeoff_speed: 3.0       # Takeoff speed in m/s
  landing_speed: 2.0       # Landing speed in m/s
  hold_time: 10           # Time to hold at altitude (seconds)
  
  # Optional waypoint navigation
  waypoint:
    enabled: false        # Set to true to enable waypoint
    x: 50.0              # X coordinate in meters
    y: 0.0               # Y coordinate in meters
    hold_time: 5         # Time to hold at waypoint
```

## Mission Sequence

1. **Phase 1**: Arm and takeoff to specified altitude
2. **Phase 2**: Hold position at altitude
3. **Phase 3** (optional): Navigate to waypoint if enabled
4. **Phase 4**: Land and return to manual mode

## Safety

- **Emergency Landing**: Automatic emergency landing on any error
- **Timeout Protection**: Mission has built-in timeout limits
- **Manual Override**: Can abort mission with Ctrl+C

## Troubleshooting

- Ensure SITL is running before launching mission
- Check MAVROS connection logs for any issues
- Verify all Aerostack2 components are properly loaded
- Make sure drone is in a valid state for takeoff

## Files

- `takeoff_mission.py` - Main mission script
- `takeoff_config.yaml` - Configuration file
- `takeoff_aerostack2.launch.py` - Launch file
