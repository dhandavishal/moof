# Task Execution Engine (TEE)

## Overview

The Task Execution Engine (TEE) is the mission orchestration layer for the multi-drone system. It receives high-level mission commands, validates them, generates primitive sequences, and coordinates execution through the Flight Abstraction Layer (FAL).

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  Squadron Manager                           │
│              (Multi-drone Coordination)                     │
└─────────────────────┬───────────────────────────────────────┘
                      │ Mission Commands
                      ▼
┌─────────────────────────────────────────────────────────────┐
│               Task Execution Engine (TEE)                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │State Machine │  │  Task Queue  │  │  Validators  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  Executors   │  │   Monitors   │  │   Progress   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────┬───────────────────────────────────────┘
                      │ Primitive Commands
                      ▼
┌─────────────────────────────────────────────────────────────┐
│         Flight Abstraction Layer (FAL)                      │
│              (Primitive Execution)                          │
└─────────────────────┬───────────────────────────────────────┘
                      │ MAVROS Commands
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                    PX4 Autopilot                            │
└─────────────────────────────────────────────────────────────┘
```

## Components

### Core Components

1. **State Machine** (`core/state_machine.py`)
   - Manages mission lifecycle states
   - States: IDLE, VALIDATING, EXECUTING, PAUSED, COMPLETED, ABORTED, EMERGENCY
   - Validates state transitions
   - Executes entry/exit callbacks
   - Tracks transition history

2. **Task Queue** (`core/task_queue.py`)
   - Priority-based task queue (heapq)
   - Dependency resolution with NetworkX DAG
   - Circular dependency detection
   - Emergency task injection
   - Task status tracking

3. **Task Validator** (`core/task_validator.py`)
   - Syntax validation (task structure, parameters)
   - Battery validation (energy estimation, RTL reserve)
   - GPS validation (satellite count, HDOP, fix type)
   - Connection validation

### Executors

1. **Waypoint Executor** (`executors/waypoint_executor.py`)
   - Generates waypoint navigation primitives
   - Configurable velocity and acceptance radius

2. **Survey Executor** (`executors/survey_executor.py`)
   - Lawn mower pattern generation
   - GSD-based path planning
   - Forward/side overlap configuration

3. **Search Executor** (`executors/search_executor.py`)
   - Expanding square search pattern
   - Spiral search pattern
   - Coverage-based path planning

### Monitors

1. **Battery Monitor** (`monitors/battery_monitor.py`)
   - Voltage and percentage tracking
   - Capacity estimation
   - State: good/warning/critical

2. **GPS Monitor** (`monitors/gps_monitor.py`)
   - Satellite count tracking
   - HDOP monitoring
   - Fix type validation

3. **Health Monitor** (`monitors/health_monitor.py`)
   - Aggregates all monitor data
   - Overall health state calculation
   - Emergency callback triggering

4. **Progress Monitor** (`monitors/progress_monitor.py`)
   - Tracks mission completion percentage
   - ETA estimation
   - Primitive-level progress

### Integration

- **Primitive Command Handler** (`primitive_command_handler.py`)
  - TEE → FAL bridge
  - Primitive command publishing
  - Status feedback handling

## Message Flow

```
1. Squadron Manager sends MissionCommand
   ↓
2. TEE receives via /squadron/mission_command
   ↓
3. Task enqueued in priority queue
   ↓
4. State machine transitions to VALIDATING
   ↓
5. TaskValidator checks feasibility
   ↓
6. Executor generates primitive sequence
   ↓
7. State machine transitions to EXECUTING
   ↓
8. Primitives sent to FAL via PrimitiveCommandHandler
   ↓
9. FAL executes primitives
   ↓
10. Status feedback via /fal/primitive_status
    ↓
11. Progress tracked and published
    ↓
12. State machine transitions to COMPLETED
```

## Configuration

Configuration file: `config/tee_config.yaml`

### Drone Specifications
- Mass, speed limits, climb rate
- Battery capacity and voltage
- Hover power consumption

### Camera Specifications
- Sensor dimensions
- Focal length
- Resolution

### Safety Thresholds
- Minimum battery percentage (25%)
- Critical battery percentage (20%)
- Minimum GPS satellites (8)
- Maximum HDOP (2.0)
- RTL altitude (50m)

### Task Parameters
- Survey: altitude, overlap, photo speed
- Search: speed, altitude
- Waypoint: acceptance radius, max distance

## Usage

### Launch TEE Node

```bash
# Launch TEE alone
ros2 launch task_execution tee.launch.py

# Launch TEE with FAL
ros2 launch task_execution tee_with_fal.launch.py

# Launch with custom config
ros2 launch task_execution tee.launch.py config_file:=/path/to/config.yaml
```

### Send Test Missions

```bash
# Run test mission sender
ros2 run task_execution test_mission_execution

# Monitor TEE status
ros2 run task_execution monitor_tee_status
```

### Send Mission via Command Line

```bash
# Waypoint mission
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{
    \"mission_id\": \"waypoint_001\",
    \"task_type\": \"waypoint\",
    \"priority\": 50,
    \"timeout\": 300.0,
    \"parameters\": {
      \"waypoints\": [
        {\"position\": [0, 0, 50]},
        {\"position\": [100, 0, 50]}
      ],
      \"velocity\": 10.0
    }
  }"'

# Survey mission
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{
    \"mission_id\": \"survey_001\",
    \"task_type\": \"survey\",
    \"priority\": 50,
    \"timeout\": 600.0,
    \"parameters\": {
      \"area\": [[0,0], [100,0], [100,100], [0,100]],
      \"altitude\": 50.0,
      \"overlap_forward\": 0.75,
      \"overlap_side\": 0.65
    }
  }"'
```

### Control Commands

```bash
# Pause current mission
ros2 topic pub /squadron/pause std_msgs/msg/Empty

# Resume paused mission
ros2 topic pub /squadron/resume std_msgs/msg/Empty

# Abort all missions
ros2 topic pub /squadron/abort std_msgs/msg/Empty
```

## Topics

### Subscribed Topics

- `/squadron/mission_command` (std_msgs/String) - Mission commands
- `/squadron/pause` (std_msgs/Empty) - Pause execution
- `/squadron/resume` (std_msgs/Empty) - Resume execution
- `/squadron/abort` (std_msgs/Empty) - Abort all missions
- `/fal/primitive_status` (multi_drone_msgs/PrimitiveStatus) - Primitive feedback

### Published Topics

- `/tee/mission_status` (std_msgs/String) - Current mission status
- `/tee/primitive_command` (multi_drone_msgs/PrimitiveCommand) - Primitives to FAL

### Monitor Topics

- `/mavros/battery` (sensor_msgs/BatteryState) - Battery data
- `/mavros/global_position/global` (sensor_msgs/NavSatFix) - GPS data

## State Machine Transitions

```
IDLE ──────────────────> VALIDATING
                            │
                            ├──> EXECUTING ──> PAUSED
                            │         │
                            │         ├──> COMPLETED ──> IDLE
                            │         │
                            │         └──> ABORTED ──> IDLE
                            │
                            └──> ABORTED ──> IDLE

Any state can transition to EMERGENCY
EMERGENCY ──> IDLE or ABORTED
```

## Task Queue Priority Levels

- **EMERGENCY** (0): RTL, emergency landing
- **CRITICAL** (10): Safety-critical tasks
- **HIGH** (20): High-priority missions
- **NORMAL** (50): Standard missions
- **LOW** (100): Background tasks

## Validation Checks

### Syntax Validation
- Task type must be known (waypoint, survey, search, rtl)
- Required parameters must be present
- Parameter values must be valid

### Battery Validation
- Current percentage > critical threshold (20%)
- Sufficient energy for mission + RTL + 20% margin
- Energy estimation based on task type and duration

### GPS Validation
- 3D fix acquired
- Satellite count ≥ minimum (8)
- HDOP ≤ maximum (2.0)

### Connection Validation
- MAVROS connected to autopilot

## Emergency Handling

Emergency procedures triggered on:
- Critical battery level (< 20%)
- GPS loss (satellites < 8)
- Connection loss
- Primitive execution failure
- Health monitor critical state

Emergency response:
1. Inject emergency RTL task at priority 0
2. Pause current task
3. Transition to EMERGENCY state
4. Execute RTL to home position
5. Return to IDLE or ABORTED after completion

## Building

```bash
# Build package
colcon build --packages-select task_execution

# Source workspace
source install/setup.bash
```

## Dependencies

- **ROS2 Packages**: rclpy, std_msgs, sensor_msgs, geometry_msgs
- **Custom Messages**: multi_drone_msgs (PrimitiveCommand, PrimitiveStatus)
- **Python Libraries**: 
  - networkx (dependency resolution)
  - shapely (polygon operations)
  - numpy (calculations)
  - yaml (configuration)

## Testing

```bash
# Run all tests
colcon test --packages-select task_execution

# Run specific test
pytest src/task_execution/test/test_state_machine.py

# Check test results
colcon test-result --verbose
```

## Development Status

✅ **Completed Components**:
- Core infrastructure (state machine, task queue, validator)
- All executors (waypoint, survey, search)
- All monitors (battery, GPS, health, progress)
- FAL integration (primitive command handler)
- Main TEE node
- Launch files
- Test scripts

## Future Enhancements

- [ ] Real-time re-planning based on environment changes
- [ ] Multi-drone coordination primitives
- [ ] Advanced search patterns (grid, random)
- [ ] Obstacle avoidance integration
- [ ] Mission persistence and recovery
- [ ] Web-based mission monitoring dashboard
- [ ] Mission replay and simulation mode

## Troubleshooting

### TEE node fails to start
- Check that all dependencies are installed
- Verify FAL is running if using integrated launch
- Check configuration file exists and is valid YAML

### Missions rejected during validation
- Check battery level (must be > 25%)
- Verify GPS has 3D fix with 8+ satellites
- Ensure MAVROS is connected
- Review validation error messages in logs

### Primitives not executing
- Verify FAL node is running
- Check `/fal/primitive_status` topic for feedback
- Ensure primitive commands are being published to `/tee/primitive_command`

### Emergency RTL triggered unexpectedly
- Check health monitor logs for trigger reason
- Verify battery and GPS thresholds in config
- Review monitor status topics

## License

Apache 2.0

## Maintainer

dhandavishal <dhandavishal@yahoo.co.in>
