# MOOFS-3D Quick Start Guide

## ✅ Phase 1.1 Complete: Multi-SITL Infrastructure

Congratulations! You now have a complete multi-drone SITL infrastructure setup.

## What's Been Implemented

### 📁 Project Structure
```
moofs_3d/
├── config/
│   ├── multi_drone_config.yaml      # Drone configurations
│   └── ardupilot_defaults.parm      # ArduPilot default parameters
├── launch/
│   ├── multi_mavros.launch.py       # MAVROS multi-instance launcher
│   └── multi_drone_system.launch.py # Complete system launcher
├── scripts/
│   ├── launch_multi_sitl.sh         # Multi-SITL launcher
│   ├── kill_multi_sitl.sh           # Stop all SITL instances
│   ├── quick_start.sh               # Interactive menu system
│   ├── validate_checkpoint_1_1.sh   # SITL validation
│   └── validate_checkpoint_1_2.sh   # MAVROS validation
├── moofs_3d/
│   └── multi_drone_monitor.py       # Status monitoring node
└── README.md                         # Full documentation
```

## 🚀 Quick Start (3 Steps)

### Step 1: Build the Package
```bash
cd ~/multi_drone_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select moofs_3d --symlink-install
source install/setup.bash
```

### Step 2: Launch SITL Instances
```bash
# Terminal 1: Launch 3 SITL instances
cd ~/multi_drone_ws/src/moofs_3d
./scripts/launch_multi_sitl.sh 3
```

Wait for all instances to initialize (~30 seconds).

### Step 3: Launch MAVROS & Monitor
```bash
# Terminal 2: Launch MAVROS and monitoring
cd ~/multi_drone_ws
source install/setup.bash
ros2 launch moofs_3d multi_drone_system.launch.py num_drones:=3
```

You should see the multi-drone status monitor displaying:
- Connection status for each drone
- Armed/disarmed state
- Flight mode
- Position (X, Y, Z)
- Battery voltage and percentage

## 📊 Validation

### Run Automated Tests
```bash
# Test 1: Validate SITL setup
./scripts/validate_checkpoint_1_1.sh

# Test 2: Validate MAVROS connections
./scripts/validate_checkpoint_1_2.sh
```

### Manual Verification

**Check ROS2 Nodes:**
```bash
ros2 node list
# Expected output:
# /drone_0/mavros
# /drone_1/mavros
# /drone_2/mavros
# /multi_drone_monitor
```

**Check Topics:**
```bash
ros2 topic list | grep drone_
```

**Monitor Individual Drone:**
```bash
# Watch drone_0 state
ros2 topic echo /drone_0/mavros/state

# Watch drone_0 position
ros2 topic echo /drone_0/mavros/local_position/pose
```

**Test Independent Control:**
```bash
# Arm drone_0 only
ros2 service call /drone_0/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Check if drone_1 is still disarmed
ros2 topic echo /drone_1/mavros/state --once | grep armed
# Should show: armed: false
```

## 🎮 Using QGroundControl

Connect QGroundControl to individual drones:

1. Open QGroundControl
2. Application Settings → Comm Links
3. Add new UDP connection:
   - **Drone 0:** Port 14550
   - **Drone 1:** Port 14560
   - **Drone 2:** Port 14570

You can switch between drones by changing the port.

## 🛠️ Useful Commands

### SITL Management
```bash
# List running SITL sessions
screen -ls

# Attach to drone 0 console
screen -r drone_0_sitl

# Detach from screen (inside screen)
Ctrl+A, then D

# Stop all SITL instances
./scripts/kill_multi_sitl.sh
```

### ROS2 Debugging
```bash
# Check node info
ros2 node info /drone_0/mavros

# List services for drone 0
ros2 service list | grep drone_0

# Echo specific topic
ros2 topic echo /drone_0/mavros/battery

# Check topic frequency
ros2 topic hz /drone_0/mavros/state
```

## 🔧 Troubleshooting

### SITL won't start
```bash
# Check ArduPilot directory
echo $ARDUPILOT_DIR
# If empty, set it:
export ARDUPILOT_DIR=$HOME/ardupilot

# Kill any existing processes
./scripts/kill_multi_sitl.sh
pkill -f sim_vehicle
```

### MAVROS connection failed
```bash
# Check if SITL is running
screen -ls

# Check if ports are listening
netstat -tuln | grep 145

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

### Port already in use
```bash
# Find what's using the port
sudo lsof -i :14550

# Kill specific process
kill <PID>
```

## 📋 Checkpoint Status

### ✅ Checkpoint 1.1: SITL Validation
- [x] All 3 SITL instances running
- [x] Unique SYSID per drone (1, 2, 3)
- [x] Different UDP ports active (14550, 14560, 14570)
- [x] Can connect QGroundControl separately

### ✅ Checkpoint 1.2: MAVROS Multi-Instance  
- [x] All namespaces exist (/drone_0, /drone_1, /drone_2)
- [x] Topics available for each drone
- [x] Services available for each drone
- [x] Independent control verified

## 🎯 Next Steps

### Phase 2: Flight Abstraction Layer (Coming Next)
You'll implement:
1. **FAL Node** - Abstract interface for flight primitives
2. **Flight Primitives** - Arm, Takeoff, GoTo, Land
3. **Action Servers** - ROS2 actions for each primitive
4. **Single Drone Testing** - Validate each primitive
5. **Multi-Drone FAL** - Parallel operation of multiple drones

### What You Need to Do
1. Create `flight_abstraction` package
2. Define action messages in `multi_drone_msgs`
3. Implement primitive classes
4. Create FAL node with action servers
5. Write test scripts

## 📖 Additional Resources

- **Full README:** `~/multi_drone_ws/src/moofs_3d/README.md`
- **Configuration:** `~/multi_drone_ws/src/moofs_3d/config/`
- **ArduPilot Docs:** https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- **MAVROS Docs:** http://wiki.ros.org/mavros
- **ROS2 Docs:** https://docs.ros.org/

## 💡 Tips

1. **Use the interactive menu:**
   ```bash
   ./scripts/quick_start.sh
   ```

2. **Keep terminals organized:**
   - Terminal 1: SITL
   - Terminal 2: MAVROS + Monitor
   - Terminal 3: Command testing

3. **Save your terminal layout:**
   Consider using `tmux` or `terminator` for multiple panes

4. **Monitor system resources:**
   Running multiple SITL instances is CPU-intensive. Check with `htop`.

## 🎉 Congratulations!

You've successfully completed Phase 1.1 of the MOOFS-3D multi-drone system!

Your system can now:
- ✅ Launch multiple ArduPilot SITL instances
- ✅ Run multiple MAVROS nodes independently
- ✅ Monitor all drones in real-time
- ✅ Control drones independently via ROS2

Ready to move to Phase 2? Check the main implementation plan!
