#!/bin/bash
# Test Squadron Manager functionality

echo "=========================================="
echo "Squadron Manager Test Script"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if squadron manager is running
echo -e "${YELLOW}Checking if Squadron Manager is running...${NC}"
if ros2 node list | grep -q "squadron_manager"; then
    echo -e "${GREEN}✓ Squadron Manager is running${NC}"
else
    echo -e "${RED}✗ Squadron Manager not found${NC}"
    echo "Please launch it first:"
    echo "  ros2 launch squadron_manager squadron_manager.launch.py"
    exit 1
fi

echo ""
echo "=========================================="
echo "Test 1: Check Squadron Status"
echo "=========================================="
ros2 topic echo --once /squadron/status

echo ""
echo "=========================================="
echo "Test 2: Send Single-Drone Mission"
echo "=========================================="
MISSION_ID="test_single_$(date +%s)"
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 120.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [{\"x\": 10.0, \"y\": 10.0, \"z\": 15.0, \"yaw\": 0.0}],
      \"velocity\": 2.0,
      \"acceptance_radius\": 1.0
    }
  }'}"

echo -e "${GREEN}✓ Sent single-drone mission: ${MISSION_ID}${NC}"
sleep 2

echo ""
echo "=========================================="
echo "Test 3: Create Line Formation"
echo "=========================================="
ros2 topic pub --once /squadron/formation_command std_msgs/msg/String \
  "{data: '{
    \"type\": \"create\",
    \"formation_type\": \"line\",
    \"spacing\": 10.0,
    \"altitude\": 50.0,
    \"center\": [0.0, 0.0, 50.0]
  }'}"

echo -e "${GREEN}✓ Sent line formation command${NC}"
sleep 2

echo ""
echo "=========================================="
echo "Test 4: Create Wedge Formation"
echo "=========================================="
ros2 topic pub --once /squadron/formation_command std_msgs/msg/String \
  "{data: '{
    \"type\": \"create\",
    \"formation_type\": \"wedge\",
    \"spacing\": 12.0,
    \"altitude\": 50.0,
    \"center\": [0.0, 0.0, 50.0]
  }'}"

echo -e "${GREEN}✓ Sent wedge formation command${NC}"
sleep 2

echo ""
echo "=========================================="
echo "Test 5: Send Multi-Drone Formation Mission"
echo "=========================================="
FORMATION_MISSION_ID="test_formation_$(date +%s)"
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${FORMATION_MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 180.0,
    \"multi_drone\": true,
    \"formation_type\": \"wedge\",
    \"spacing\": 15.0,
    \"altitude\": 50.0,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 50.0, \"y\": 0.0, \"z\": 50.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.0
    }
  }'}"

echo -e "${GREEN}✓ Sent multi-drone mission: ${FORMATION_MISSION_ID}${NC}"
sleep 2

echo ""
echo "=========================================="
echo "Test 6: Break Formation"
echo "=========================================="
ros2 topic pub --once /squadron/formation_command std_msgs/msg/String \
  "{data: '{\"type\": \"break\"}'}"

echo -e "${GREEN}✓ Sent break formation command${NC}"

echo ""
echo "=========================================="
echo "All Tests Complete!"
echo "=========================================="
echo ""
echo "Monitor squadron status with:"
echo "  ros2 topic echo /squadron/status"
echo ""
echo "Monitor individual drone missions with:"
echo "  ros2 topic echo /drone_0/tee/mission_status"
echo "  ros2 topic echo /drone_1/tee/mission_status"
echo ""
