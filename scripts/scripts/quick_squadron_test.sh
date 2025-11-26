#!/bin/bash
# Quick Squadron Manager Test - Single Drone
# This script helps you test squadron manager with 1 drone

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo ""
echo "=========================================="
echo "  Squadron Manager - Single Drone Test"
echo "=========================================="
echo ""

# Check if sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 environment not sourced${NC}"
    echo "Please run: source ~/multi_drone_ws/install/setup.bash"
    exit 1
fi

echo -e "${CYAN}This script will guide you through testing Squadron Manager with 1 drone.${NC}"
echo ""
echo "You'll need 5 terminals open. This is Terminal 1."
echo ""
echo -e "${YELLOW}Setup Overview:${NC}"
echo "  Terminal 1: ArduPilot SITL"
echo "  Terminal 2: ROS2 Single-Drone System (MAVROS + FAL + TEE)"
echo "  Terminal 3: Squadron Manager"
echo "  Terminal 4: System Verification"
echo "  Terminal 5: Mission Testing"
echo ""

# Function to show terminal command
show_terminal() {
    local term_num=$1
    local title=$2
    local cmd=$3
    
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}Terminal ${term_num}: ${title}${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}${cmd}${NC}"
    echo ""
}

# Terminal 1: SITL
show_terminal "1" "Start ArduPilot SITL" \
"cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map"

echo -e "${YELLOW}⏸  Wait for: 'APM: EKF2 IMU0 is using GPS' (10-15 seconds)${NC}"
echo ""

read -p "Press Enter when SITL is ready and you're in Terminal 2..."
echo ""

# Terminal 2: Single-Drone System
show_terminal "2" "Launch Single-Drone System" \
"cd ~/multi_drone_ws
source install/setup.bash
ros2 launch task_execution complete_system.launch.py"

echo -e "${YELLOW}⏸  Wait for: All nodes to start (about 10 seconds)${NC}"
echo ""

read -p "Press Enter when system is running and you're in Terminal 3..."
echo ""

# Terminal 3: Squadron Manager
show_terminal "3" "Launch Squadron Manager" \
"cd ~/multi_drone_ws
source install/setup.bash
ros2 launch squadron_manager squadron_manager.launch.py num_drones:=1"

echo -e "${YELLOW}⏸  Wait for: 'Squadron Manager initialized with 1 drones'${NC}"
echo ""

read -p "Press Enter when Squadron Manager is running and you're in Terminal 4..."
echo ""

# Terminal 4: Verify System
show_terminal "4" "Verify System is Running" \
"cd ~/multi_drone_ws
source install/setup.bash

# Check nodes
echo 'Checking nodes...'
ros2 node list

echo ''
echo 'Expected nodes:'
echo '  /drone_0/fal_node'
echo '  /drone_0/mavros'
echo '  /task_execution_engine'
echo '  /squadron_manager'
echo ''

# Check squadron status
echo 'Checking squadron status...'
timeout 2 ros2 topic echo /squadron/status --once"

echo ""
read -p "Press Enter when verification is complete and you're in Terminal 5..."
echo ""

# Terminal 5: Send Test Mission
MISSION_ID="squadron_test_$(date +%s)"

show_terminal "5" "Send Test Mission via Squadron Manager" \
"cd ~/multi_drone_ws
source install/setup.bash

# Send mission to Squadron Manager (NOT directly to drone!)
MISSION_ID=\"${MISSION_ID}\"

ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \\
  \"{data: '{
    \\\"mission_id\\\": \\\"'\${MISSION_ID}'\\\",
    \\\"task_type\\\": \\\"waypoint\\\",
    \\\"priority\\\": 100,
    \\\"timeout\\\": 120.0,
    \\\"multi_drone\\\": false,
    \\\"parameters\\\": {
      \\\"waypoints\\\": [
        {\\\"x\\\": 10.0, \\\"y\\\": 10.0, \\\"z\\\": 15.0, \\\"yaw\\\": 0.0}
      ],
      \\\"velocity\\\": 2.0,
      \\\"acceptance_radius\\\": 1.0
    }
  }'}\"

echo \"Mission \${MISSION_ID} sent to Squadron Manager!\"
echo \"\"
echo \"Watch Terminal 3 (Squadron Manager) for allocation\"
echo \"Watch Terminal 2 (TEE) for execution\""

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}What to Watch For:${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Terminal 3 (Squadron Manager):${NC}"
echo "  • 'Received mission: ${MISSION_ID}'"
echo "  • 'Selected drone_0 for mission'"
echo "  • 'Sent mission ${MISSION_ID} to drone_0'"
echo ""
echo -e "${YELLOW}Terminal 2 (TEE):${NC}"
echo "  • 'Received mission command: ${MISSION_ID}'"
echo "  • 'State transition: idle -> validating'"
echo "  • 'State transition: validating -> executing'"
echo "  • 'Executing primitive 1/4: ARM'"
echo "  • 'Executing primitive 2/4: TAKEOFF'"
echo "  • 'Executing primitive 3/4: GOTO'"
echo "  • 'Executing primitive 4/4: LAND'"
echo "  • 'State transition: executing -> completed'"
echo ""
echo -e "${YELLOW}Terminal 1 (SITL):${NC}"
echo "  • Watch the drone fly in the map window"
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${CYAN}Optional Monitoring (open new terminal):${NC}"
echo ""
echo "Monitor TEE status:"
echo -e "${GREEN}ros2 topic echo /drone_0/tee/mission_status${NC}"
echo ""
echo "Monitor Squadron status:"
echo -e "${GREEN}ros2 topic echo /squadron/status${NC}"
echo ""
echo "Monitor drone position:"
echo -e "${GREEN}ros2 topic echo /drone_0/mavros/local_position/pose${NC}"
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${CYAN}🎯 Key Difference:${NC}"
echo ""
echo -e "${RED}OLD: Mission sent directly to drone${NC}"
echo -e "     ros2 topic pub /drone_0/tee/mission_command ..."
echo ""
echo -e "${GREEN}NEW: Mission sent to Squadron Manager${NC}"
echo -e "     ros2 topic pub /squadron/mission_command ..."
echo -e "     └─> Squadron allocates to best drone"
echo -e "         └─> Forwards to /drone_0/tee/mission_command"
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}For more tests, use the interactive script:${NC}"
echo -e "${GREEN}./scripts/test_squadron_missions.sh${NC}"
echo ""
echo -e "${YELLOW}Full documentation:${NC}"
echo -e "${GREEN}docs/SQUADRON_SINGLE_DRONE_TEST.md${NC}"
echo ""
echo -e "${BLUE}Happy Testing! 🚀${NC}"
echo ""
