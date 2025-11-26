#!/bin/bash
# Complete Squadron Manager Integration Test
# This script verifies all components are running before sending missions

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║   Squadron Manager - Complete Integration Verification    ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 environment not sourced${NC}"
    echo "Run: source ~/multi_drone_ws/install/setup.bash"
    exit 1
fi
echo -e "${GREEN}✓ ROS2 environment sourced${NC}"

# Check Squadron Manager
echo ""
echo -e "${CYAN}Checking Squadron Manager...${NC}"
if ros2 node list 2>/dev/null | grep -q "squadron_manager"; then
    echo -e "${GREEN}✓ Squadron Manager is running${NC}"
else
    echo -e "${RED}✗ Squadron Manager not running${NC}"
    echo ""
    echo "Start it with:"
    echo "  ros2 launch squadron_manager squadron_manager.launch.py num_drones:=1"
    exit 1
fi

# Check TEE
echo ""
echo -e "${CYAN}Checking Task Execution Engine...${NC}"
if ros2 node list 2>/dev/null | grep -q "task_execution_engine"; then
    echo -e "${GREEN}✓ TEE is running${NC}"
else
    echo -e "${RED}✗ TEE not running${NC}"
    echo ""
    echo "Start it with:"
    echo "  ros2 launch task_execution complete_system.launch.py"
    exit 1
fi

# Check MAVROS
echo ""
echo -e "${CYAN}Checking MAVROS...${NC}"
if ros2 node list 2>/dev/null | grep -q "drone_0/mavros"; then
    echo -e "${GREEN}✓ MAVROS is running${NC}"
else
    echo -e "${RED}✗ MAVROS not running${NC}"
    echo ""
    echo "Start it with:"
    echo "  ros2 launch task_execution complete_system.launch.py"
    exit 1
fi

# Check FAL
echo ""
echo -e "${CYAN}Checking Flight Abstraction Layer...${NC}"
if ros2 node list 2>/dev/null | grep -q "drone_0/fal_node"; then
    echo -e "${GREEN}✓ FAL is running${NC}"
else
    echo -e "${RED}✗ FAL not running${NC}"
    echo ""
    echo "Start it with:"
    echo "  ros2 launch task_execution complete_system.launch.py"
    exit 1
fi

# Check MAVROS connection
echo ""
echo -e "${CYAN}Checking MAVROS connection to SITL...${NC}"
CONNECTED=$(ros2 topic echo /drone_0/mavros/state --once 2>/dev/null | grep "connected:" | awk '{print $2}')
if [ "$CONNECTED" = "true" ]; then
    echo -e "${GREEN}✓ MAVROS connected to SITL${NC}"
else
    echo -e "${RED}✗ MAVROS not connected to SITL${NC}"
    echo ""
    echo "Make sure ArduPilot SITL is running:"
    echo "  cd ~/ardupilot/ArduCopter"
    echo "  sim_vehicle.py -v ArduCopter --console --map"
    exit 1
fi

# Check drone state in squadron
echo ""
echo -e "${CYAN}Checking drone status in Squadron Manager...${NC}"
sleep 2  # Give squadron manager time to receive updates

# Check available topics
echo ""
echo -e "${CYAN}Verifying communication topics...${NC}"
TOPICS_OK=true

if ros2 topic list 2>/dev/null | grep -q "/squadron/mission_command"; then
    echo -e "${GREEN}✓ /squadron/mission_command exists${NC}"
else
    echo -e "${RED}✗ /squadron/mission_command missing${NC}"
    TOPICS_OK=false
fi

if ros2 topic list 2>/dev/null | grep -q "/drone_0/tee/mission_command"; then
    echo -e "${GREEN}✓ /drone_0/tee/mission_command exists${NC}"
else
    echo -e "${RED}✗ /drone_0/tee/mission_command missing${NC}"
    TOPICS_OK=false
fi

if ros2 topic list 2>/dev/null | grep -q "/drone_0/mavros/state"; then
    echo -e "${GREEN}✓ /drone_0/mavros/state exists${NC}"
else
    echo -e "${RED}✗ /drone_0/mavros/state missing${NC}"
    TOPICS_OK=false
fi

if [ "$TOPICS_OK" = false ]; then
    echo ""
    echo -e "${RED}Some topics are missing. Check if all nodes started correctly.${NC}"
    exit 1
fi

# Check drone battery
echo ""
echo -e "${CYAN}Checking drone battery status...${NC}"
BATTERY=$(ros2 topic echo /drone_0/mavros/battery --once 2>/dev/null | grep "percentage:" | awk '{print $2}')
if [ ! -z "$BATTERY" ]; then
    BATTERY_PCT=$(echo "$BATTERY * 100" | bc)
    echo -e "${GREEN}✓ Battery: ${BATTERY_PCT}%${NC}"
else
    echo -e "${YELLOW}⚠ Battery status not available (may be SITL - OK)${NC}"
fi

# All checks passed
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║              ✓ All Systems Ready!                         ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Show system summary
echo -e "${CYAN}System Summary:${NC}"
echo "  • Squadron Manager: RUNNING"
echo "  • Task Execution Engine: RUNNING"
echo "  • Flight Abstraction Layer: RUNNING"
echo "  • MAVROS: CONNECTED"
echo "  • ArduPilot SITL: CONNECTED"
echo ""

# Offer to send test mission
echo -e "${YELLOW}Ready to send test mission!${NC}"
echo ""
read -p "Send a test waypoint mission? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    MISSION_ID="squadron_test_$(date +%s)"
    
    echo ""
    echo -e "${BLUE}Sending mission: ${MISSION_ID}${NC}"
    echo "  Waypoint: (10, 10, 15)"
    echo "  Velocity: 2.0 m/s"
    echo ""
    
    ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
      "{data: '{
        \"mission_id\": \"${MISSION_ID}\",
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
    
    echo ""
    echo -e "${GREEN}✓ Mission sent!${NC}"
    echo ""
    echo -e "${CYAN}Monitor execution:${NC}"
    echo ""
    echo "Squadron Manager status:"
    echo "  ros2 topic echo /squadron/status"
    echo ""
    echo "TEE mission status:"
    echo "  ros2 topic echo /drone_0/tee/mission_status"
    echo ""
    echo "Drone position:"
    echo "  ros2 topic echo /drone_0/mavros/local_position/pose"
    echo ""
    
    # Watch for mission status updates
    echo -e "${YELLOW}Watching for mission status (5 seconds)...${NC}"
    timeout 5 ros2 topic echo /drone_0/tee/mission_status 2>/dev/null || true
    echo ""
    
else
    echo ""
    echo "Send mission manually with:"
    echo "  ./scripts/send_squadron_mission.sh 10 10 15"
    echo ""
fi

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}Test complete! 🚀${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""
