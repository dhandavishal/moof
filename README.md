# MOOFS - Mission Oriented Operating Framework for Software Defined Drones

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Compatible-orange.svg)](https://ardupilot.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Documentation](https://img.shields.io/badge/docs-comprehensive-brightgreen.svg)](../docs/)

**MOOFS** is a comprehensive, modular framework for autonomous drone mission execution built on ROS2. It provides a complete software stack for mission planning, validation, execution, and monitoring of software-defined drones.

> **📚 NEW: [Comprehensive Documentation Available](../docs/)** - 1000+ pages covering architecture, implementation, ROS2 concepts, and operational workflows!

## 🚁 Overview

MOOFS implements a hierarchical architecture that separates high-level mission planning from low-level flight control, enabling:

- **Mission-oriented operations**: Define missions as high-level tasks (waypoint navigation, area surveys, search patterns)
- **Automated flight sequences**: ARM → TAKEOFF → MISSION → LAND with full lifecycle management
- **Real-time validation**: Pre-flight checks for battery, GPS, safety constraints
- **Health monitoring**: Continuous monitoring of drone health with emergency procedures
- **Multi-drone coordination**: Squadron Manager for fleet operations with intelligent task allocation
- **Formation flying**: Coordinated formation patterns (wedge, line, V, diamond, circle)
- **Synchronized operations**: Sync barriers for coordinated takeoff, altitude, and formation assembly

## 📖 Documentation

**Complete documentation is now available** in the [`docs/`](../docs/) directory:

- **[Documentation Hub](../docs/README.md)** - Start here
- **[Quick Start Guide](../docs/QUICKSTART_SINGLE_DRONE.md)** - Get running in 10 minutes
- **[Project Overview](../docs/01_PROJECT_OVERVIEW.md)** - Goals, features, and technology
- **[System Architecture](../docs/02_SYSTEM_ARCHITECTURE.md)** - Complete architectural deep dive
- **[ROS2 Fundamentals](../docs/03_ROS2_FUNDAMENTALS.md)** - ROS2 concepts in MOOFS
- **[Task Execution Engine](../docs/04_TASK_EXECUTION_ENGINE.md)** - Mission orchestration
- **[Mission Execution Flow](../docs/11_MISSION_EXECUTION_FLOW.md)** - Complete walkthrough

See **[Table of Contents](../docs/00_TABLE_OF_CONTENTS.md)** for all 25+ documentation files.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Squadron Manager                             │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────────┐          │
│  │   Drone     │  │    Task      │  │   Formation    │          │
│  │  Registry   │  │  Allocator   │  │  Controller    │          │
│  └─────────────┘  └──────────────┘  └────────────────┘          │
│  ┌─────────────────────────────────────────────────────┐        │
│  │  Sync Barriers: Takeoff | Altitude | Formation      │        │
│  └─────────────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────────────┘
                                 │
         ┌───────────────────────┼───────────────────────┐
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  TEE (drone_0)  │    │  TEE (drone_1)  │    │  TEE (drone_2)  │
│  ┌───────────┐  │    │  ┌───────────┐  │    │  ┌───────────┐  │
│  │ Namespace │  │    │  │ Namespace │  │    │  │ Namespace │  │
│  │  Aware    │  │    │  │  Aware    │  │    │  │  Aware    │  │
│  └───────────┘  │    └───────────┘  │    │  └───────────┘  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  FAL (drone_0)  │    │  FAL (drone_1)  │    │  FAL (drone_2)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ MAVROS (drone_0)│    │ MAVROS (drone_1)│    │ MAVROS (drone_2)│
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
    SITL:14550              SITL:14560              SITL:14570
```

## 📦 Components

### 1. **Squadron Manager** ✨ NEW
Central coordination for multi-drone operations:
- **Drone Registry**: Tracks all drone states, positions, battery levels, and availability
- **Task Allocator**: Intelligent task distribution with multiple strategies:
  - `nearest`: Assign to closest available drone
  - `greedy`: Assign to first available drone
  - `load_balanced`: Distribute tasks evenly
  - `capability_based`: Match drone capabilities to task requirements
- **Formation Controller**: Coordinated formation flying:
  - Line, Wedge, V, Diamond, Circle formations
  - Configurable spacing and altitude
- **Sync Barriers**: Coordinated multi-drone operations:
  - Takeoff barriers (all drones take off together)
  - Altitude barriers (synchronize at formation altitude)
  - Formation barriers (establish formation before proceeding)
- **Mission Distribution**: Breaks multi-drone missions into individual drone tasks

### 2. **Task Execution Engine (TEE)**
High-level mission orchestration and management:
- **Mission Queue**: Priority-based task scheduling
- **Task Validators**: Pre-flight safety checks (battery, GPS, geofence)
- **Executors**: Mission-specific logic (waypoint, survey, search)
- **Monitors**: Real-time health monitoring (battery, GPS, connectivity)
- **State Machine**: Mission lifecycle management (IDLE → VALIDATING → EXECUTING → COMPLETED)
- **Dynamic Namespace Resolution**: Automatically detects drone namespace from ROS2 node namespace

### 3. **Flight Abstraction Layer (FAL)**
Low-level flight primitive execution:
- **ROS2 Action Servers**: Takeoff, Land, GoToWaypoint, ExecutePrimitive
- **Primitives**: Atomic flight operations (Arm, Takeoff, Goto, Land, Loiter, RTL)
- **State Management**: Tracks drone state and primitive execution
- **MAVROS Integration**: Direct communication with flight controller
- **Dynamic Namespace Resolution**: Auto-configures for multi-drone deployment
- **Robust Timeout Handling**: Graceful handling of MAVROS service timeouts with state verification

### 4. **Multi-Drone Messages**
Custom ROS2 message and action definitions:
- Mission commands and status
- Primitive commands and feedback
- Health reports
- Drone telemetry

## 🚀 Getting Started

### Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **ArduPilot SITL** or compatible flight controller
- **MAVROS** (ROS2 Humble version)
- **Python 3.10+**

### Installation

1. **Clone the repository:**
```bash
cd ~
mkdir -p multi_drone_ws/src
cd multi_drone_ws/src
git clone https://github.com/dhandavishal/moof.git .
```

2. **Install dependencies:**
```bash
cd ~/multi_drone_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the workspace:**
```bash
cd ~/multi_drone_ws
colcon build --symlink-install
source install/setup.bash
```

### Running MOOFS

#### 1. Start ArduPilot SITL (Terminal 1)
```bash
cd ~/ardupilot/Tools/autotest
./sim_vehicle.py -v ArduCopter -f quad --console --map
```

#### 2. Launch MOOFS System (Terminal 2)
```bash
cd ~/multi_drone_ws
source install/setup.bash

# Single drone
ros2 launch task_execution complete_system.launch.py drone_id:=0

# Multiple drones (3 drones)
ros2 launch task_execution multi_drone_system.launch.py num_drones:=3
```

#### 3. Launch Squadron Manager (Terminal 3) - For Multi-Drone Operations
```bash
cd ~/multi_drone_ws
source install/setup.bash
ros2 launch squadron_manager squadron_manager.launch.py num_drones:=3
```

This launches:
- MAVROS (connects to ArduPilot) per drone
- Flight Abstraction Layer (FAL) per drone
- Task Execution Engine (TEE) per drone
- Squadron Manager (centralized coordinator)

#### 4. Send a Mission (Terminal 4)
```bash
cd ~/multi_drone_ws
source install/setup.bash

# Single drone waypoint mission
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{\"mission_id\": \"mission_1\", 
            \"task_type\": \"waypoint\", 
            \"priority\": 100, 
            \"timeout\": 120.0, 
            \"multi_drone\": false,
            \"parameters\": {
              \"waypoints\": [
                {\"x\": 10.0, \"y\": 10.0, \"z\": 15.0, \"yaw\": 0.0}
              ], 
              \"velocity\": 2.0, 
              \"acceptance_radius\": 1.0
            }
          }'}"

# Multi-drone formation mission
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{\"mission_id\": \"formation_patrol\", 
            \"task_type\": \"waypoint\", 
            \"multi_drone\": true,
            \"formation_type\": \"wedge\",
            \"spacing\": 15.0, 
            \"altitude\": 50.0,
            \"parameters\": {
              \"waypoints\": [
                {\"x\": 50.0, \"y\": 0.0, \"z\": 50.0},
                {\"x\": 50.0, \"y\": 50.0, \"z\": 50.0}
              ], 
              \"velocity\": 3.0
            }
          }'}"
```

#### 5. Monitor Mission Status
```bash
# Watch squadron status
ros2 topic echo /squadron/status

# Watch individual drone TEE status
ros2 topic echo /drone_0/tee/mission_status
ros2 topic echo /drone_1/tee/mission_status

# Monitor drone state
ros2 topic echo /drone_0/mavros/state
```

## 📋 Mission Examples

### Waypoint Mission
```json
{
  "mission_id": "waypoint_mission",
  "task_type": "waypoint",
  "priority": 100,
  "timeout": 180.0,
  "parameters": {
    "waypoints": [
      {"x": 10.0, "y": 0.0, "z": 15.0, "yaw": 0.0},
      {"x": 10.0, "y": 10.0, "z": 15.0, "yaw": 1.57},
      {"x": 0.0, "y": 10.0, "z": 15.0, "yaw": 3.14}
    ],
    "velocity": 3.0,
    "acceptance_radius": 1.0
  }
}
```

### Survey Mission
```json
{
  "mission_id": "survey_mission",
  "task_type": "survey",
  "priority": 80,
  "timeout": 300.0,
  "parameters": {
    "area": [
      {"x": 0.0, "y": 0.0},
      {"x": 50.0, "y": 0.0},
      {"x": 50.0, "y": 50.0},
      {"x": 0.0, "y": 50.0}
    ],
    "altitude": 40.0,
    "overlap_forward": 0.75,
    "overlap_side": 0.65
  }
}
```

### Search Pattern Mission
```json
{
  "mission_id": "search_mission",
  "task_type": "search",
  "priority": 90,
  "timeout": 240.0,
  "parameters": {
    "center": {"x": 25.0, "y": 25.0},
    "radius": 30.0,
    "pattern": "spiral",
    "altitude": 25.0
  }
}
```

### Multi-Drone Formation Mission ✨ NEW
```json
{
  "mission_id": "formation_patrol",
  "task_type": "waypoint",
  "multi_drone": true,
  "formation_type": "wedge",
  "spacing": 15.0,
  "altitude": 50.0,
  "parameters": {
    "waypoints": [
      {"x": 50.0, "y": 0.0, "z": 50.0},
      {"x": 50.0, "y": 50.0, "z": 50.0},
      {"x": 0.0, "y": 50.0, "z": 50.0}
    ],
    "velocity": 3.0
  }
}
```

**Supported Formation Types:**
- `line` - Drones in a straight line
- `wedge` - V-shaped formation with leader at front
- `v` - Classic V formation
- `diamond` - Diamond pattern
- `circle` - Circular arrangement around center

## 🔧 Configuration

### Multi-Drone Launch Options ✨ NEW
```bash
# Basic launch with 3 drones (UDP connections)
ros2 launch task_execution multi_drone_system.launch.py num_drones:=3

# With TCP connection (alternative)
ros2 launch task_execution multi_drone_system.launch.py num_drones:=3 connection_protocol:=tcp

# With performance monitoring
ros2 launch task_execution multi_drone_system.launch.py num_drones:=3 enable_monitor:=true

# Full options (up to 10 drones)
ros2 launch task_execution multi_drone_system.launch.py \
  num_drones:=5 \
  log_level:=warn \
  fcu_host:=host.docker.internal \
  sitl_port:=14570
```

### Squadron Manager Configuration
```bash
ros2 launch squadron_manager squadron_manager.launch.py \
  num_drones:=3 \
  allocation_strategy:=nearest
```

**Allocation Strategies:**
- `nearest` - Assign task to closest available drone
- `greedy` - Assign to first available drone
- `load_balanced` - Distribute tasks evenly across drones
- `capability_based` - Match drone capabilities to task requirements

### Safety Parameters
Edit `task_execution/config/` or modify default config in `tee_node.py`:

```python
'safety': {
    'min_battery_percentage': 0.25,      # Minimum 25% battery
    'critical_battery_percentage': 0.20, # Emergency at 20%
    'min_gps_satellites': 8,             # Require 8 satellites
    'max_gps_hdop': 2.0,                 # Max HDOP of 2.0
    'rtl_altitude': 50.0                 # RTL altitude in meters
}
```

### Drone Parameters
```python
'drone': {
    'mass': 2.5,              # kg
    'max_speed': 15.0,        # m/s
    'cruise_speed': 10.0,     # m/s
    'hover_power': 250        # watts
}
```

## 🧪 Testing

### Unit Tests
```bash
cd ~/multi_drone_ws
colcon test --packages-select flight_abstraction task_execution
colcon test-result --verbose
```

### Integration Tests
```bash
# Run system diagnostics
ros2 run task_execution test_integration

# Check topic communication
ros2 topic list
ros2 topic hz /drone_0/fal/status
ros2 topic hz /tee/mission_status
```

## 📊 System Monitoring

### Health Monitoring
- **Battery Monitor**: Voltage, current, percentage, temperature
- **GPS Monitor**: Fix quality, satellite count, HDOP
- **Health Monitor**: Overall system health aggregation

### Progress Tracking
- Mission completion percentage
- Primitives executed / total
- Waypoints reached
- Area covered (for surveys)
- Estimated time remaining

### State Machine States
- **IDLE**: Waiting for missions
- **VALIDATING**: Checking mission safety
- **EXECUTING**: Running primitives
- **PAUSED**: Holding position
- **COMPLETED**: Mission successful
- **ABORTED**: Mission cancelled
- **EMERGENCY**: Emergency procedures active

## 🛠️ Development

### Adding a New Primitive

1. Create primitive class in `flight_abstraction/primitives/`:
```python
from .base_primitive import BasePrimitive, PrimitiveState

class MyPrimitive(BasePrimitive):
    def execute(self, **kwargs) -> bool:
        # Implementation
        pass
    
    def update(self) -> bool:
        # Update logic
        pass
```

2. Register in `fal_node.py`:
```python
self.primitives['my_primitive'] = MyPrimitive(self, ...)
```

### Adding a New Task Executor

1. Create executor class in `task_execution/executors/`:
```python
from .base_executor import BaseExecutor

class MyExecutor(BaseExecutor):
    def execute(self, parameters: dict) -> List[PrimitiveCommand]:
        # Generate primitive sequence
        pass
```

2. Register in `tee_node.py`:
```python
self.executors['my_task'] = MyExecutor(self.config)
```

## 🔍 Troubleshooting

### MAVROS Connection Issues
```bash
# Check MAVROS nodes
ros2 node list | grep mavros

# Check MAVROS connection
ros2 topic echo /drone_0/mavros/state

# Verify ArduPilot SITL is running
ps aux | grep sim_vehicle
```

### GPS Validation Failures
```bash
# Check GPS status
ros2 topic echo /drone_0/mavros/global_position/global

# Check raw GPS data
ros2 topic echo /drone_0/mavros/gpsstatus/gps1/raw

# For SITL testing, GPS validation is relaxed automatically
```

### Action Server Not Available
```bash
# List available action servers
ros2 action list

# Check FAL node status
ros2 node info /drone_0/fal_node

# Verify action server
ros2 action info /drone_0/takeoff
```

### Multi-Drone Service Issues ✨ NEW
```bash
# Verify all drone arm services are available
ros2 service list | grep arm
# Should show: /drone_0/arm_disarm, /drone_1/arm_disarm, etc.

# Check all FAL nodes are running
ros2 node list | grep fal

# Verify Squadron Manager connectivity
ros2 topic echo /squadron/status

# Check individual drone TEE status
ros2 topic echo /drone_0/tee/mission_status
ros2 topic echo /drone_1/tee/mission_status
```

### Formation Mission Issues
```bash
# Monitor sync barrier status (in Squadron Manager logs)
# Look for: "Created barrier 'mission_takeoff' for N drones"

# Check all drones are registered
ros2 topic echo /squadron/status
# Should show all drones with state=AVAILABLE
```

## 📚 Documentation

- [Architecture Guide](docs/ARCHITECTURE.md) - Detailed system architecture
- [API Reference](docs/API.md) - ROS2 topics, services, actions
- [Development Guide](docs/DEVELOPMENT.md) - Contributing guidelines
- [Integration Tests](docs/INTEGRATION_TEST_RESULTS.md) - Test results
- [Deployment Guide](docs/DEPLOYMENT.md) - Production deployment

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS2** - Robot Operating System 2
- **ArduPilot** - Open source autopilot
- **MAVROS** - MAVLink to ROS gateway
- **PX4** - Open source flight control platform


## 🗺️ Roadmap

- [x] Core FAL implementation
- [x] TEE mission orchestration
- [x] Waypoint missions
- [x] Health monitoring
- [x] Survey mission executor
- [x] Search pattern executor
- [x] Multi-drone coordination
- [x] Squadron Manager with task allocation
- [x] Formation flying (wedge, line, V, diamond, circle)
- [x] Sync barriers for coordinated operations
- [x] Dynamic namespace resolution for multi-drone
- [x] MAVROS timeout handling and state verification
- [ ] Web-based mission planner GUI
- [ ] Obstacle avoidance integration
- [ ] Computer vision integration
- [ ] Real-time telemetry dashboard
- [ ] Swarm intelligence algorithms

---


