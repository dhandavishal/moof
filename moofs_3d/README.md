# MOOFS-3D: Multi-Drone SITL & Control System

## Overview
MOOFS-3D (Multi-Drone Operations Framework for Simulation) is a comprehensive ROS2 package for simulating and controlling multiple drones using ArduPilot SITL and MAVROS.

## Phase 1: Multi-Drone Infrastructure Setup ✅

### Current Implementation Status

#### ✅ Phase 1.1: Multi-SITL Environment Setup
- Multi-drone SITL launcher script
- MAVROS multi-instance launch files
- Configuration management
- Validation test scripts

#### 🔄 Phase 1.2: Project Structure (Planned)
- Custom message definitions
- Flight Abstraction Layer (FAL)
- Task Execution Engine (TEE)
- Squadron Manager
- Mission Planner
- Web GUI

## Prerequisites

### System Requirements
- Ubuntu 20.04 or 22.04
- ROS2 (Humble or newer)
- Python 3.8+
- ArduPilot SITL

### Dependencies Installation

```bash
# Install ROS2 (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install MAVROS
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
sudo apt install ros-$ROS_DISTRO-mavros-msgs

# Download GeographicLib datasets for MAVROS
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Install screen for multi-SITL management
sudo apt install screen

# Install ArduPilot SITL
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Install Python dependencies
pip3 install pymavlink MAVProxy
```

## Installation

```bash
# Create workspace (if not exists)
mkdir -p ~/multi_drone_ws/src
cd ~/multi_drone_ws/src

# Clone or copy moofs_3d package
# (assuming package is already in src/)

# Build the workspace
cd ~/multi_drone_ws
colcon build --packages-select moofs_3d

# Source the workspace
source install/setup.bash
```

## Usage

### Step 1: Launch Multi-SITL Instances

```bash
# Make scripts executable
chmod +x ~/multi_drone_ws/src/moofs_3d/scripts/*.sh

# Set ArduPilot directory (if not in default location)
export ARDUPILOT_DIR=$HOME/ardupilot

# Launch 3 SITL instances
cd ~/multi_drone_ws/src/moofs_3d
./scripts/launch_multi_sitl.sh 3
```

**Expected Output:**
```
========================================
Multi-Drone SITL Launcher
========================================
Number of drones: 3
...
Drone 0:
  SYSID: 1
  MAVProxy port: 14550
  ...
All SITL instances launched!
```

**Verify SITL Instances:**
```bash
# List screen sessions
screen -ls
# Should show: drone_0_sitl, drone_1_sitl, drone_2_sitl

# Attach to a drone's console
screen -r drone_0_sitl

# Check SYSID in MAVProxy
param show SYSID_THISMAV

# Detach from screen: Ctrl+A then D
```

### Step 2: Launch MAVROS Multi-Instance

```bash
# In a new terminal, source workspace
cd ~/multi_drone_ws
source install/setup.bash

# Launch MAVROS for 3 drones
ros2 launch moofs_3d multi_mavros.launch.py num_drones:=3
```

**Verify MAVROS:**
```bash
# Check ROS2 nodes
ros2 node list
# Should show: /drone_0/mavros, /drone_1/mavros, /drone_2/mavros

# Check topics
ros2 topic list | grep drone_

# Echo state of drone 0
ros2 topic echo /drone_0/mavros/state
```

### Step 3: Run Validation Tests

```bash
# Validate SITL setup (Checkpoint 1.1)
cd ~/multi_drone_ws/src/moofs_3d
./scripts/validate_checkpoint_1_1.sh

# Validate MAVROS setup (Checkpoint 1.2)
./scripts/validate_checkpoint_1_2.sh
```

## Checkpoint Validation

### ✅ Checkpoint 1.1: SITL Validation

**Tests:**
1. ✓ All 3 SITL instances running
2. ✓ Unique SYSID per drone
3. ✓ Different UDP ports active
4. ✓ Can connect QGroundControl separately

**Manual Validation:**
```bash
# Check running instances
screen -ls

# Check ports
netstat -tuln | grep 145

# Check SYSID in MAVProxy
screen -r drone_0_sitl
# In MAVProxy: param show SYSID_THISMAV
```

### ✅ Checkpoint 1.2: MAVROS Multi-Instance

**Tests:**
1. ✓ All namespaces exist (/drone_0, /drone_1, /drone_2)
2. ✓ Topics available for each drone
3. ✓ Services available for each drone
4. ✓ Connection state readable

**Manual Validation:**
```bash
# Check namespaces
ros2 node list

# Verify independent control
# Arm only drone_0
ros2 service call /drone_0/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Check if drone_1 is still disarmed
ros2 topic echo /drone_1/mavros/state --once | grep armed
```

## Configuration

### Multi-Drone Config: `config/multi_drone_config.yaml`

```yaml
# Modify drone count, capabilities, and safety parameters
drones:
  drone_0:
    name: "Alpha"
    sysid: 1
    start_position: [0, 0, 0]
```

### ArduPilot Parameters: `config/ardupilot_defaults.parm`

Default parameters applied to each SITL instance.

## Port Mapping

| Drone ID | SYSID | MAVProxy Port | SITL Out | QGC Connection |
|----------|-------|---------------|----------|----------------|
| drone_0  | 1     | 14550         | 14555    | UDP:14550      |
| drone_1  | 2     | 14560         | 14565    | UDP:14560      |
| drone_2  | 3     | 14570         | 14575    | UDP:14570      |
| drone_3  | 4     | 14580         | 14585    | UDP:14580      |
| drone_4  | 5     | 14590         | 14595    | UDP:14590      |

## Troubleshooting

### SITL won't start
```bash
# Check ArduPilot directory
ls $ARDUPILOT_DIR

# Check for existing processes
pkill -f sim_vehicle.py

# Check screen sessions
screen -ls
```

### MAVROS won't connect
```bash
# Check if SITL is running
screen -ls

# Check ports
netstat -tuln | grep 145

# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Check MAVROS logs
ros2 launch moofs_3d multi_mavros.launch.py num_drones:=1
```

### Port already in use
```bash
# Kill existing SITL instances
./scripts/kill_multi_sitl.sh

# Find process using port
lsof -i :14550
# Or
sudo netstat -tulpn | grep 14550
```

## Stopping the System

```bash
# Stop SITL instances
./scripts/kill_multi_sitl.sh

# Stop MAVROS (Ctrl+C in launch terminal)

# Or kill all related processes
pkill -f sim_vehicle
pkill -f mavros
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  QGroundControl                     │
│              (UDP: 14550, 14560, 14570)            │
└─────────────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────┐
│            ArduPilot SITL Instances                 │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐            │
│  │ Drone 0 │  │ Drone 1 │  │ Drone 2 │            │
│  │ SYSID:1 │  │ SYSID:2 │  │ SYSID:3 │            │
│  └─────────┘  └─────────┘  └─────────┘            │
└─────────────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────┐
│              MAVROS Multi-Instance                  │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐│
│  │ /drone_0/   │  │ /drone_1/   │  │ /drone_2/    ││
│  │   mavros    │  │   mavros    │  │   mavros     ││
│  └─────────────┘  └─────────────┘  └──────────────┘│
└─────────────────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────┐
│              ROS2 Topics & Services                 │
│  - State, Position, Battery, Commands              │
│  - Independent control per drone                   │
└─────────────────────────────────────────────────────┘
```

## Next Steps

### Phase 2: Flight Abstraction Layer (Coming Soon)
- Define FAL interface
- Implement flight primitives (arm, takeoff, goto, land)
- Create action definitions
- Single and multi-drone FAL testing

### Phase 3: Task Execution Engine
- State machine implementation
- Task queue system
- Contingency handling

### Phase 4+: Squadron management, Mission planning, GUI

## Contributing

This is part of a phased implementation plan. Each phase builds upon the previous with clear validation checkpoints.

## License

Apache-2.0

## Contact

Maintainer: dhandavishal@yahoo.co.in

## References

- [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [ROS2 Documentation](https://docs.ros.org/)
