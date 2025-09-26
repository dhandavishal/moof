# AS2 ArduPilot Direct MAVRO## Usage

```bash
# Terminal 1: Start ArduPilot SITL
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --map --console

# Terminal 2: Launch mission (minimal)
cd /home/dhandavishal/aerostack2_ws
source install/setup.bash
ros2 launch as2_ardu_msn basic_aerostack2.launch.py

# Or with state estimator enabled
ros2 launch as2_ardu_msn basic_aerostack2.launch.py use_state_estimator:=true

# Or with custom mission config
ros2 launch as2_ardu_msn basic_aerostack2.launch.py mission_config:=/path/to/custom_config.yaml
```

## Launch Arguments

- `drone_namespace` (default: 'drone0') - Drone namespace
- `use_sim_time` (default: 'true') - Use simulation time
- `use_state_estimator` (default: 'false') - Enable Aerostack2 state estimator
- `mission_config` (default: config/mission_config.yaml) - Mission configuration filehis package provides a direct MAVROS control solution for ArduPilot survey missions, bypassing the Aerostack2 platform abstraction layer that had compatibility issues with ArduPilot's state machine.

## Architecture

The solution uses **direct MAVROS commands** instead of Aerostack2 platform abstraction:

- **Launch**: Simplified launch file with only MAVROS, state estimator (optional), and mission node
- **Control**: Direct MAVROS services (`/mavros/cmd/arming`, `/mavros/cmd/takeoff`, `/mavros/cmd/land`)
- **Navigation**: Direct setpoint publishing to `/mavros/setpoint_position/local`
- **State**: Direct MAVROS state monitoring (`/mavros/state`, `/mavros/local_position/pose`)

## Files

- `launch/basic_aerostack2.launch.py` - Simplified launch file (MAVROS + mission only)
- `as2_ardu_msn/basic_survey_mission.py` - Direct MAVROS mission implementation
- `config/mission_config.yaml` - Mission parameters (survey area, altitude, flight speed)
- `config/mavros_timing_fix.yaml` - MAVROS timing optimizations for high RTT
- `config/state_estimator.yaml` - Optional TF/odometry (disabled by default)

## Key Improvements

1. **No Platform Abstraction**: Bypasses `as2_platform_mavlink` which had ArduPilot compatibility issues
2. **Direct Control**: Uses MAVROS services and topics directly for reliable communication
3. **Simplified Architecture**: Fewer components = fewer failure points
4. **ArduPilot Optimized**: Designed specifically for ArduPilot GUIDED mode behavior

## Usage

```bash
# Terminal 1: Start ArduPilot SITL
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --map --console

# Terminal 2: Launch mission
cd /home/dhandavishal/aerostack2_ws
source install/setup.bash
ros2 launch as2_ardu_msn basic_aerostack2.launch.py
```

## Mission Flow

1. **MAVROS Readiness**: Wait for MAVROS connection
2. **Mode Setting**: Set GUIDED mode via MAVROS
3. **Arming**: Arm via MAVROS service
4. **Takeoff**: Direct MAVROS takeoff command
5. **Waypoints**: Navigate using position setpoints
6. **Landing**: Direct MAVROS land command

## Benefits vs Previous Aerostack2 Approach

- ✅ **Direct Control**: No FSM state machine conflicts
- ✅ **ArduPilot Compatible**: Uses GUIDED mode properly
- ✅ **Reduced Latency**: Fewer abstraction layers
- ✅ **Better Debugging**: Direct MAVROS topic/service visibility
- ✅ **Simplified**: Fewer components to troubleshoot
