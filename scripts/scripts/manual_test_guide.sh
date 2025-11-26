#!/bin/bash

# Manual Test Checklist for FAL-TEE Integration

# Run these tests step-by-step to verify system functionality## 🚀 SINGLE DRONE TESTING - START HERE



echo "════════════════════════════════════════════════════════════════"Since you already have **ArduPilot SITL running**, follow these steps:

echo "  FAL-TEE Integration Manual Test Guide"

echo "════════════════════════════════════════════════════════════════"### **Terminal 1: Launch Complete System** (NEW TERMINAL)

echo ""

```bash

# Colors for outputcd ~/multi_drone_ws

GREEN='\033[0;32m'source install/setup.bash

YELLOW='\033[1;33m'ros2 launch task_execution test_system.launch.py

BLUE='\033[0;34m'```

RED='\033[0;31m'

NC='\033[0m' # No ColorThis launches:

- **MAVROS** (connects to your ArduPilot SITL on ports 14550/14555)

echo -e "${BLUE}PREREQUISITE: System must be running!${NC}"- **FAL** (after 3 seconds)

echo "If not already running:"- **TEE** (after 6 seconds)

echo "  Terminal 1: cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f quad --console --map"

echo "  Terminal 2: cd ~/multi_drone_ws && source install/setup.bash && ros2 launch task_execution test_system.launch.py"**Wait 10 seconds** for everything to initialize, then proceed...

echo ""

echo "Press ENTER when system is running..."---

read

### **Terminal 2: Verify System** (NEW TERMINAL)

echo ""

echo "════════════════════════════════════════════════════════════════"```bash

echo -e "${BLUE}TEST 1: Verify Topic Communication Setup${NC}"cd ~/multi_drone_ws

echo "════════════════════════════════════════════════════════════════"source install/setup.bash

echo ""

# 1. Check MAVROS connection (should show "connected: true")

echo -e "${YELLOW}1.1: Check primitive_command topic (TEE → FAL)${NC}"ros2 topic echo /drone_0/mavros/state --once

ros2 topic info /tee/primitive_command -v

echo ""# 2. Check GPS (should show status: 2)

echo "EXPECTED: Publisher count: 1 (TEE), Subscription count: 1 (FAL)"ros2 topic echo /drone_0/mavros/global_position/global --once | grep -E "status|latitude"

echo "Press ENTER to continue..."

read# 3. Check battery (should show voltage)

ros2 topic echo /drone_0/mavros/battery --once | grep voltage

echo -e "${YELLOW}1.2: Check primitive_status topic (FAL → TEE)${NC}"

ros2 topic info /fal/primitive_status -v# 4. Check TEE status (should show "state": "idle")

echo ""ros2 topic echo /tee/mission_status --once

echo "EXPECTED: Publisher count: 1 (FAL), Subscription count: 1 (TEE)"

echo "Press ENTER to continue..."# 5. List all active nodes

readros2 node list

```

echo ""

echo "════════════════════════════════════════════════════════════════"**Expected nodes:**

echo -e "${BLUE}TEST 2: Verify MAVROS Connection${NC}"- `/drone_0/mavros`

echo "════════════════════════════════════════════════════════════════"- `/drone_0/fal_node`

echo ""- `/task_execution_engine`



echo -e "${YELLOW}2.1: Check MAVROS state${NC}"---

ros2 topic echo /drone_0/mavros/state --once

echo ""### **Terminal 3: Send Test Mission** (NEW TERMINAL)

echo "EXPECTED: connected: true"

echo "Press ENTER to continue..."```bash

readcd ~/multi_drone_ws

source install/setup.bash

echo -e "${YELLOW}2.2: Check current GPS position${NC}"

ros2 topic echo /drone_0/mavros/global_position/global --once# Send automated test missions (waypoint, survey, search)

echo ""ros2 run task_execution test_mission_execution

echo "EXPECTED: Valid lat/lon/alt values"```

echo "Press ENTER to continue..."

read---



echo ""### **Terminal 4: Monitor Execution** (NEW TERMINAL)

echo "════════════════════════════════════════════════════════════════"

echo -e "${BLUE}TEST 3: Health Monitoring${NC}"```bash

echo "════════════════════════════════════════════════════════════════"cd ~/multi_drone_ws

echo ""source install/setup.bash



echo -e "${YELLOW}3.1: Check battery health${NC}"# Watch real-time status

ros2 topic echo /drone_0/tee/health/battery --onceros2 run task_execution monitor_tee_status

echo ""```

echo "EXPECTED: is_healthy: true, valid voltage/percentage"

echo "Press ENTER to continue..."---

read

## ✅ What to Look For

echo -e "${YELLOW}3.2: Check GPS health${NC}"

ros2 topic echo /drone_0/tee/health/gps --once### **In Terminal 1 (Launch)** - You should see:

echo ""```

echo "EXPECTED: is_healthy: true, valid fix data"[mavros]: FCU: ArduPilot

echo "Press ENTER to continue..."[mavros]: connected: true

read[fal_node]: Flight Abstraction Layer initialized

[task_execution_engine]: Task Execution Engine initialized

echo -e "${YELLOW}3.3: Check overall health${NC}"```

ros2 topic echo /drone_0/tee/health/overall --once

echo ""### **In Terminal 4 (Monitor)** - You should see state changes:

echo "EXPECTED: is_healthy: true or appropriate status"```

echo "Press ENTER to continue..."State: IDLE → VALIDATING → EXECUTING → COMPLETED

readProgress: 0% → 50% → 100%

Health: GOOD

echo ""```

echo "════════════════════════════════════════════════════════════════"

echo -e "${BLUE}TEST 4: Topic-Based Primitive Commands${NC}"---

echo "════════════════════════════════════════════════════════════════"

echo ""## 🐛 Quick Troubleshooting



echo -e "${YELLOW}4.1: Test ARM command via topic${NC}"**If MAVROS doesn't connect:**

echo "In another terminal, run:"```bash

echo -e "${GREEN}ros2 topic pub --once /tee/primitive_command multi_drone_msgs/msg/PrimitiveCommand \"{primitive_id: 'test_arm_1', primitive_type: 'arm', parameters: [1.0], target_position: {x: 0.0, y: 0.0, z: 0.0}, metadata: ''}\"${NC}"# Check if ArduPilot SITL is still running

echo ""screen -ls | grep drone_0_sitl

echo "Monitor status with:"

echo -e "${GREEN}ros2 topic echo /fal/primitive_status${NC}"# Reattach to SITL to see logs

echo ""screen -r drone_0_sitl

echo "EXPECTED: Should see 'executing' then 'success' status"```

echo "ArduPilot console should show: 'ARMING MOTORS'"

echo "Press ENTER when done observing..."**If TEE doesn't start:**

read```bash

# Check for errors

echo -e "${YELLOW}4.2: Monitor MAVROS armed state${NC}"ros2 node info /task_execution_engine

ros2 topic echo /drone_0/mavros/state --once | grep armed```

echo ""

echo "EXPECTED: armed: true"**If mission doesn't execute:**

echo "Press ENTER to continue..."```bash

read# Check state machine

ros2 topic echo /tee/mission_status

echo -e "${YELLOW}4.3: Test DISARM command via topic${NC}"

echo "In another terminal, run:"# Check primitives being sent

echo -e "${GREEN}ros2 topic pub --once /tee/primitive_command multi_drone_msgs/msg/PrimitiveCommand \"{primitive_id: 'test_disarm_1', primitive_type: 'arm', parameters: [0.0], target_position: {x: 0.0, y: 0.0, z: 0.0}, metadata: ''}\"${NC}"ros2 topic echo /tee/primitive_command

echo ""```

echo "EXPECTED: Should see disarm in status, MAVROS armed: false"

echo "Press ENTER when done observing..."---

read

## 🎯 Expected Flow

echo ""

echo "════════════════════════════════════════════════════════════════"1. **Mission sent** → TEE receives it

echo -e "${BLUE}TEST 5: Action Server Interface (Alternative)${NC}"2. **State: VALIDATING** → Checks battery, GPS, syntax

echo "════════════════════════════════════════════════════════════════"3. **State: EXECUTING** → Generates primitives (Arm → Takeoff → Goto → Land)

echo ""4. **Primitives sent to FAL** → FAL converts to MAVROS commands

5. **MAVROS sends to ArduPilot** → Drone executes

echo -e "${YELLOW}5.1: List available actions${NC}"6. **State: COMPLETED** → Mission done!

ros2 action list

echo ""---

echo "EXPECTED: /drone_0/takeoff, /land, /goto_waypoint, /execute_primitive"

echo "Press ENTER to continue..."**Ready? Start with Terminal 1 above!** 🚀

read

Made changes.
echo -e "${YELLOW}5.2: Check takeoff action info${NC}"
ros2 action info /drone_0/takeoff
echo ""
echo "EXPECTED: Action servers: 1, FAL node providing"
echo "Press ENTER to continue..."
read

echo ""
echo "════════════════════════════════════════════════════════════════"
echo -e "${BLUE}TEST 6: Service Interface${NC}"
echo "════════════════════════════════════════════════════════════════"
echo ""

echo -e "${YELLOW}6.1: Test arm/disarm service${NC}"
echo "ARM command:"
echo -e "${GREEN}ros2 service call /drone_0/arm_disarm multi_drone_msgs/srv/ArmDisarm \"{arm: true, force: false}\"${NC}"
echo ""
echo "After testing, DISARM:"
echo -e "${GREEN}ros2 service call /drone_0/arm_disarm multi_drone_msgs/srv/ArmDisarm \"{arm: false, force: false}\"${NC}"
echo ""
echo "EXPECTED: Response success: true"
echo "Press ENTER when done..."
read

echo ""
echo "════════════════════════════════════════════════════════════════"
echo -e "${BLUE}TEST 7: Data Flow Validation${NC}"
echo "════════════════════════════════════════════════════════════════"
echo ""

echo -e "${YELLOW}7.1: Check message rates${NC}"
echo "Health monitoring rate:"
timeout 3s ros2 topic hz /drone_0/tee/health/overall
echo ""
echo "MAVROS state rate:"
timeout 3s ros2 topic hz /drone_0/mavros/state
echo ""
echo "Press ENTER to continue..."
read

echo ""
echo "════════════════════════════════════════════════════════════════"
echo -e "${GREEN}TESTS COMPLETE!${NC}"
echo "════════════════════════════════════════════════════════════════"
echo ""

echo "For automated diagnostics, run:"
echo -e "${GREEN}./scripts/system_diagnostics.sh${NC}"
echo ""
