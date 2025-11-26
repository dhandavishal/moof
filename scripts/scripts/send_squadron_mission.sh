#!/bin/bash
# Simple test mission sender for Squadron Manager

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "=========================================="
echo "Squadron Manager - Send Test Mission"
echo "=========================================="
echo ""

# Check if squadron manager is running
if ! ros2 node list 2>/dev/null | grep -q "squadron_manager"; then
    echo -e "${RED}✗ Squadron Manager not running${NC}"
    echo ""
    echo "Please start it first:"
    echo "  ros2 launch squadron_manager squadron_manager.launch.py num_drones:=1"
    echo ""
    exit 1
fi

echo -e "${GREEN}✓ Squadron Manager detected${NC}"
echo ""

# Generate mission ID
MISSION_ID="squadron_test_$(date +%s)"

# Mission parameters
X=${1:-10.0}
Y=${2:-10.0}
Z=${3:-15.0}

echo "Sending waypoint mission:"
echo "  Mission ID: ${MISSION_ID}"
echo "  Waypoint: (${X}, ${Y}, ${Z})"
echo "  Velocity: 2.0 m/s"
echo ""

# Send mission
ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 120.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": ${X}, \"y\": ${Y}, \"z\": ${Z}, \"yaw\": 0.0}
      ],
      \"velocity\": 2.0,
      \"acceptance_radius\": 1.0
    }
  }'}"

echo -e "${GREEN}✓ Mission sent!${NC}"
echo ""
echo "Mission ID: ${MISSION_ID}"
echo ""
echo -e "${YELLOW}Watch for:${NC}"
echo "  • Squadron Manager logs: allocation decision"
echo "  • TEE logs: mission execution"
echo ""
echo "Monitor mission status:"
echo "  ros2 topic echo /drone_0/tee/mission_status"
echo ""
echo "Monitor squadron status:"
echo "  ros2 topic echo /squadron/status"
echo ""

# Usage instructions
echo "Usage: $0 [x] [y] [z]"
echo "Example: $0 20 20 30  (waypoint at x=20, y=20, z=30)"
echo ""
