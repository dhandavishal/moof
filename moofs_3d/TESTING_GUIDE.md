# Flight Abstraction Layer Testing Guide

This guide provides instructions for testing the Flight Abstraction Layer (FAL) implementation.

---

## Prerequisites

1. **ArduPilot SITL** installed and configured
2. **MAVROS** installed (should be available via ROS2 Humble)
3. **Workspace built**: `cd ~/multi_drone_ws && colcon build --symlink-install`
4. **Workspace sourced**: `source ~/multi_drone_ws/install/setup.bash`

---

## Quick Start: Single Drone Test

### Terminal Setup (4 terminals required)

#### Terminal 1: Launch SITL
```bash
cd ~/multi_drone_ws
source install/setup.bash
./src/moofs_3d/scripts/launch_multi_sitl.sh 1
```

Wait for SITL to fully initialize (you'll see "APM: EKF2 IMU0 is using GPS" message).

#### Terminal 2: Launch MAVROS + FAL
```bash
cd ~/multi_drone_ws
source install/setup.bash
ros2 launch moofs_3d single_drone_test.launch.py drone_id:=0
```

Wait for MAVROS to connect (you'll see "CON: Got HEARTBEAT" messages).

#### Terminal 3: Run Automated Test
```bash
cd ~/multi_drone_ws
source install/setup.bash
ros2 run flight_abstraction test_single_drone --ros-args -p drone_id:=0
```

The test will execute a complete flight sequence automatically.

#### Terminal 4: Monitor (Optional)
```bash
cd ~/multi_drone_ws
source install/setup.bash

# Monitor drone state
ros2 topic echo /drone_0/mavros/state

# Monitor local position
ros2 topic echo /drone_0/mavros/local_position/pose

# Monitor action feedback
ros2 action list
```

---

## Test Sequence Explained

The automated test (`test_single_drone.py`) executes the following sequence:

1. **Arm** (via service)
   - Service: `/drone_0/arm_disarm`
   - Expected: Armed state confirmed via MAVROS

2. **Takeoff** to 10m (via action)
   - Action: `/drone_0/takeoff`
   - Target altitude: 10.0m
   - Feedback: Current altitude and progress percentage
   - Expected: Altitude reaches 10.0m ± 0.5m

3. **Navigate Square Pattern** (4 waypoints via action)
   - Waypoint 1: (10, 0, 10)
   - Waypoint 2: (10, 10, 10)
   - Waypoint 3: (0, 10, 10)
   - Waypoint 4: (0, 0, 10) - Return to origin
   - Action: `/drone_0/goto_waypoint`
   - Feedback: Distance remaining, ETA, current position
   - Expected: Reach each waypoint within 1.0m acceptance radius

4. **Land** (via action)
   - Action: `/drone_0/land`
   - Feedback: Current altitude and progress
   - Expected: Altitude reaches <0.3m, extended state confirms landing

5. **Disarm** (via service)
   - Service: `/drone_0/arm_disarm`
   - Expected: Disarmed state confirmed

---

## Manual Testing with CLI

### 1. Arm the Drone
```bash
ros2 service call /drone_0/arm_disarm multi_drone_msgs/srv/ArmDisarm "{arm: true, force: false}"
```

Expected output:
```
success: true
message: "Armed successfully"
```

### 2. Takeoff to 10m
```bash
ros2 action send_goal /drone_0/takeoff multi_drone_msgs/action/Takeoff \
  "{target_altitude: 10.0, climb_rate: 1.0}" --feedback
```

Watch for feedback messages showing altitude increase.

### 3. Navigate to Waypoint
```bash
ros2 action send_goal /drone_0/goto_waypoint multi_drone_msgs/action/GoToWaypoint \
  "{target_position: {x: 10.0, y: 0.0, z: 10.0}, target_heading: 0.0, max_speed: 2.0, acceptance_radius: 1.0}" --feedback
```

Watch for feedback showing distance decreasing.

### 4. Land
```bash
ros2 action send_goal /drone_0/land multi_drone_msgs/action/Land \
  "{descent_rate: 0.5}" --feedback
```

Watch for altitude decreasing to ground level.

### 5. Disarm
```bash
ros2 service call /drone_0/arm_disarm multi_drone_msgs/srv/ArmDisarm "{arm: false, force: false}"
```

---

## Monitoring and Debugging

### Check Action Servers
```bash
ros2 action list
```

Should show:
- `/drone_0/takeoff`
- `/drone_0/land`
- `/drone_0/goto_waypoint`
- `/drone_0/execute_primitive`

### Check Service Servers
```bash
ros2 service list | grep drone_0
```

Should show:
- `/drone_0/arm_disarm`
- Plus all MAVROS services

### Monitor MAVROS State
```bash
# State (armed, mode, etc.)
ros2 topic echo /drone_0/mavros/state

# Local position
ros2 topic echo /drone_0/mavros/local_position/pose

# Extended state (landed state)
ros2 topic echo /drone_0/mavros/extended_state

# Battery
ros2 topic echo /drone_0/mavros/battery
```

### Check FAL Node
```bash
# See if FAL node is running
ros2 node list | grep fal

# View node info
ros2 node info /fal_node
```

### View Logs
```bash
# FAL node logs
ros2 run rqt_console rqt_console

# Filter by node: /fal_node
# Filter by severity: Info, Warning, Error
```

---

## Troubleshooting

### Problem: SITL won't start
**Solution**: 
```bash
# Kill any existing SITL processes
pkill -9 -f arducopter
screen -wipe

# Check ArduPilot path
ls ~/ardupilot/ardupilot/Tools/autotest/sim_vehicle.py
```

### Problem: MAVROS won't connect
**Symptoms**: No "CON: Got HEARTBEAT" message

**Solutions**:
1. Check SITL is running: `screen -ls` should show `drone_0_sitl`
2. Check SITL output: `screen -r drone_0_sitl`
3. Verify ports: SITL should output to UDP 14555
4. Check MAVROS FCU URL in launch file

### Problem: Action servers not available
**Symptoms**: `ros2 action list` doesn't show FAL actions

**Solutions**:
1. Check FAL node is running: `ros2 node list | grep fal`
2. Check FAL node logs: `ros2 node info /fal_node`
3. Rebuild and source workspace:
   ```bash
   cd ~/multi_drone_ws
   colcon build --packages-select flight_abstraction
   source install/setup.bash
   ```

### Problem: Arm command fails
**Symptoms**: Service returns `success: false`

**Solutions**:
1. Check MAVROS connection: `ros2 topic echo /drone_0/mavros/state`
2. Verify `state.connected` is True
3. Check SITL mode: Should be in GUIDED or STABILIZE
4. Check GPS fix: `ros2 topic echo /drone_0/mavros/global_position/global`
5. Try with force: `{arm: true, force: true}`

### Problem: Takeoff fails immediately
**Symptoms**: Action returns failure before any altitude gain

**Possible causes**:
1. **Not armed**: Must arm before takeoff
2. **Wrong mode**: FAL sets mode to GUIDED automatically, but check
3. **GPS not ready**: Wait for GPS fix (3D fix, satellites_visible >= 6)
4. **Invalid altitude**: Must be 0 < altitude <= 100m

**Debug**:
```bash
# Check arming state
ros2 topic echo /drone_0/mavros/state --field armed

# Check mode
ros2 topic echo /drone_0/mavros/state --field mode

# Check GPS
ros2 topic echo /drone_0/mavros/global_position/global
```

### Problem: Waypoint navigation overshoots/oscillates
**Symptoms**: Drone oscillates around waypoint or overshoots significantly

**Solutions**:
1. Reduce `max_speed` parameter (try 1.0 m/s)
2. Increase `acceptance_radius` (try 2.0m for testing)
3. Check ArduPilot parameters (WPNAV_SPEED, WPNAV_ACCEL)
4. Ensure sufficient altitude (>5m recommended)

### Problem: Landing never completes
**Symptoms**: Land action times out without confirming landing

**Solutions**:
1. Check altitude: `ros2 topic echo /drone_0/mavros/local_position/pose --field pose.position.z`
2. Check extended state: `ros2 topic echo /drone_0/mavros/extended_state --field landed_state`
   - Should eventually be 1 (ON_GROUND)
3. Check for obstacles in SITL (shouldn't be any, but check)
4. Try manual mode switch: `ros2 service call /drone_0/mavros/set_mode ...`

---

## Multi-Drone Testing

### Launch 3 Drones

#### Terminal 1: SITL (3 instances)
```bash
./src/moofs_3d/scripts/launch_multi_sitl.sh 3
```

#### Terminal 2: MAVROS (3 instances)
```bash
ros2 launch moofs_3d multi_mavros.launch.py num_drones:=3
```

#### Terminal 3-5: FAL Nodes (one per drone)
```bash
# Terminal 3
ros2 launch flight_abstraction fal.launch.py drone_namespace:=/drone_0

# Terminal 4
ros2 launch flight_abstraction fal.launch.py drone_namespace:=/drone_1

# Terminal 5
ros2 launch flight_abstraction fal.launch.py drone_namespace:=/drone_2
```

#### Terminal 6: Test Script (multiple drones)
```bash
# Test drone 0
ros2 run flight_abstraction test_single_drone --ros-args -p drone_id:=0

# In parallel terminal, test drone 1
ros2 run flight_abstraction test_single_drone --ros-args -p drone_id:=1

# In parallel terminal, test drone 2
ros2 run flight_abstraction test_single_drone --ros-args -p drone_id:=2
```

---

## Performance Benchmarks

Expected performance metrics:

| Operation | Expected Time | Notes |
|-----------|--------------|-------|
| Arm | 0.5-2s | Includes MAVROS confirmation |
| Takeoff to 10m | 10-15s | Depends on climb rate |
| Navigate 10m | 5-10s | At 2 m/s max speed |
| Landing | 15-25s | Depends on altitude and descent rate |
| Full test sequence | 60-90s | Complete 4-waypoint square + takeoff/land |

---

## Next Steps

After successful single-drone testing:

1. **Multi-Drone Coordination Test**: Test 3 drones simultaneously
2. **Add Telemetry Publishing**: Implement DroneStatus publisher in FAL
3. **Unit Tests**: Create pytest tests for primitives
4. **Phase 3**: Implement Task Execution Engine (TEE)

---

## Success Criteria

Your FAL implementation passes if:

- ✅ All 15 validation tests pass (Checkpoint 2.1)
- ✅ Single drone completes full test sequence
- ✅ Arm/disarm works reliably
- ✅ Takeoff reaches target altitude within tolerance
- ✅ Navigation reaches waypoints within acceptance radius
- ✅ Landing confirms touchdown properly
- ✅ No crashes or exceptions during execution
- ✅ Action feedback is continuous and accurate

---

## Support

For issues:
1. Check this guide's troubleshooting section
2. Review `/moofs_3d/PHASE2_SUMMARY.md` for architecture details
3. Check ArduPilot SITL logs: `screen -r drone_0_sitl`
4. Check MAVROS logs in ROS2 output
5. Check FAL logs: `ros2 run rqt_console rqt_console`
