# Squadron Manager

Multi-drone coordination, task allocation, and formation control for the MOOFS system.

## Overview

The Squadron Manager is responsible for:
- **Drone Registry**: Track all drones, their status, capabilities, and health
- **Task Allocation**: Intelligently assign tasks to available drones
- **Formation Control**: Coordinate multi-drone formations (line, wedge, grid, circle, column)
- **Health Monitoring**: Monitor drone health and trigger corrective actions
- **Mission Coordination**: Receive high-level missions and distribute to individual drones

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Squadron Manager                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │    Drone     │  │     Task     │  │  Formation   │  │
│  │   Registry   │  │  Allocator   │  │  Controller  │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└────────────┬────────────────────────────────────────────┘
             │
             ├─→ Monitors: /drone_X/mavros/state
             ├─→ Monitors: /drone_X/mavros/local_position/pose
             ├─→ Monitors: /drone_X/mavros/battery
             ├─→ Monitors: /drone_X/tee/mission_status
             │
             ├─→ Subscribes: /squadron/mission_command
             ├─→ Subscribes: /squadron/formation_command
             │
             ├─→ Publishes: /squadron/status
             └─→ Publishes: /drone_X/tee/mission_command (to each TEE)
```

## Components

### 1. Drone Registry (`drone_registry.py`)

Tracks all drones in the squadron:

```python
class DroneInfo:
    - drone_id: Unique identifier
    - state: AVAILABLE, BUSY, CHARGING, MAINTENANCE, ERROR, OFFLINE
    - capabilities: Speed, altitude, sensors, payload
    - position: Current x, y, z
    - battery: Voltage, percentage, remaining time
    - gps: Fix type, satellites
    - current_task_id: Assigned task (if any)
```

**Methods**:
- `register_drone()`: Add drone to squadron
- `get_available_drones()`: Get drones ready for tasks
- `assign_task()`: Assign task to drone
- `update_drone_state()`: Update drone status
- `get_squadron_status()`: Overall squadron health

### 2. Task Allocator (`task_allocator.py`)

Allocates tasks to drones using different strategies:

**Strategies**:
- **GREEDY**: First available drone (fast, simple)
- **NEAREST**: Closest drone to task location (minimize travel)
- **LOAD_BALANCED**: Balance workload across drones (battery-aware)
- **CAPABILITY_BASED**: Match capabilities to requirements (optimal)

```python
requirements = TaskRequirements(
    min_battery=30.0,
    requires_camera=True,
    target_location=(10.0, 10.0, 50.0),
    estimated_duration=10.0  # minutes
)

drone_id = task_allocator.allocate_task(
    task_id="mission_123",
    available_drones=available_drones,
    requirements=requirements
)
```

### 3. Formation Controller (`formation_controller.py`)

Manages multi-drone formations:

**Formation Types**:
- **LINE**: Drones in a line
- **WEDGE**: V-formation
- **GRID**: Square grid pattern
- **CIRCLE**: Circular formation
- **COLUMN**: Vertical column

```python
params = FormationParameters(
    formation_type=FormationType.WEDGE,
    spacing=10.0,  # meters
    altitude=50.0,  # meters
    center=(0.0, 0.0, 50.0)
)

formation_controller.create_formation(drones, params)
```

### 4. Squadron Manager Node (`squadron_manager_node.py`)

Main ROS2 node that coordinates everything.

## Usage

### Launch Squadron Manager

```bash
# Default: 3 drones, nearest strategy
ros2 launch squadron_manager squadron_manager.launch.py

# Custom configuration
ros2 launch squadron_manager squadron_manager.launch.py \
    num_drones:=5 \
    allocation_strategy:=load_balanced \
    enable_formations:=true
```

### Send Mission Command

```bash
# Single-drone waypoint mission
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"patrol_001\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 120.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 20.0, \"y\": 20.0, \"z\": 50.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.0,
      \"acceptance_radius\": 1.0
    }
  }'}"

# Multi-drone formation mission
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"formation_patrol\",
    \"task_type\": \"waypoint\",
    \"multi_drone\": true,
    \"formation_type\": \"wedge\",
    \"spacing\": 15.0,
    \"altitude\": 50.0,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 50.0, \"y\": 0.0, \"z\": 50.0}
      ]
    }
  }'}"
```

### Formation Commands

```bash
# Create line formation
ros2 topic pub --once /squadron/formation_command std_msgs/msg/String \
  "{data: '{
    \"type\": \"create\",
    \"formation_type\": \"line\",
    \"spacing\": 10.0,
    \"altitude\": 50.0,
    \"center\": [0.0, 0.0, 50.0]
  }'}"

# Update formation center (follow moving target)
ros2 topic pub --once /squadron/formation_command std_msgs/msg/String \
  "{data: '{
    \"type\": \"update_center\",
    \"center\": [20.0, 10.0, 50.0]
  }'}"

# Break formation
ros2 topic pub --once /squadron/formation_command std_msgs/msg/String \
  "{data: '{\"type\": \"break\"}'}"
```

### Monitor Squadron Status

```bash
# Watch squadron status
ros2 topic echo /squadron/status

# Watch individual drone mission status
ros2 topic echo /drone_0/tee/mission_status
ros2 topic echo /drone_1/tee/mission_status
```

## Configuration

Edit `config/squadron_config.yaml`:

```yaml
squadron_manager:
  ros__parameters:
    num_drones: 3
    allocation_strategy: 'nearest'  # greedy, nearest, load_balanced, capability_based
    enable_formations: true
    
    # Health monitoring
    heartbeat_timeout: 5.0
    min_battery_threshold: 20.0
    
    # Formation defaults
    default_spacing: 10.0
    default_altitude: 50.0
    formation_tolerance: 2.0
```

## Integration with TEE

The Squadron Manager works with the Task Execution Engine (TEE):

1. **Squadron receives high-level mission** via `/squadron/mission_command`
2. **Squadron allocates task** to best available drone
3. **Squadron sends mission** to drone's TEE via `/drone_X/tee/mission_command`
4. **TEE validates and executes** primitives
5. **TEE reports progress** via `/drone_X/tee/mission_status`
6. **Squadron monitors completion** and reallocates if needed

## Task Allocation Examples

### Example 1: Nearest Drone

```python
# Mission at (50, 50, 50)
# Drone 0 at (10, 10, 50) - distance = 56.6m
# Drone 1 at (40, 40, 50) - distance = 14.1m ← SELECTED
# Drone 2 at (0, 0, 50) - distance = 70.7m
```

### Example 2: Capability-Based

```python
# Mission requires camera + LIDAR
# Drone 0: camera + LIDAR ← SELECTED
# Drone 1: camera only
# Drone 2: camera only
```

### Example 3: Load-Balanced

```python
# Drone 0: 95% battery, idle ← SELECTED
# Drone 1: 60% battery, busy
# Drone 2: 40% battery, idle
```

## Formation Examples

### Line Formation (3 drones, 10m spacing)
```
Drone_0 ─────── Drone_1 ─────── Drone_2
   (0,0,50)      (10,0,50)       (20,0,50)
```

### Wedge Formation (3 drones, 10m spacing)
```
           Drone_0 (Leader)
              (0,0,50)
             /          \
    Drone_1              Drone_2
   (-7,-5,50)           (-7,5,50)
```

### Grid Formation (4 drones, 10m spacing)
```
Drone_0 ─── Drone_1
   │           │
Drone_2 ─── Drone_3
```

## Testing

```bash
# Build
colcon build --packages-select squadron_manager --symlink-install

# Test imports
python3 -c "from squadron_manager.drone_registry import DroneRegistry"
python3 -c "from squadron_manager.task_allocator import TaskAllocator"
python3 -c "from squadron_manager.formation_controller import FormationController"

# Launch with test system
ros2 launch squadron_manager squadron_manager.launch.py num_drones:=3
```

## Future Enhancements

- [ ] Hungarian algorithm for optimal task allocation
- [ ] Dynamic task reallocation on failure
- [ ] Collision avoidance during formation flight
- [ ] Energy-optimal formation selection
- [ ] Real-time formation adaptation
- [ ] Multi-waypoint formation paths
- [ ] Emergency protocols (drone failure, low battery)
- [ ] Geofencing and no-fly zones
- [ ] Swarm behaviors (flocking, schooling)

## Dependencies

- `rclpy`: ROS2 Python client
- `multi_drone_msgs`: Custom drone messages
- `geometry_msgs`: Position/pose messages
- `sensor_msgs`: Battery, GPS messages
- `mavros_msgs`: MAVROS state messages

## License

Apache 2.0
