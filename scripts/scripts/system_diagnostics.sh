#!/bin/bash
# System Diagnostics Script for FAL + TEE + MAVROS Integration
# Usage: ./system_diagnostics.sh

echo "════════════════════════════════════════════════════════════════"
echo "  Multi-Drone System Diagnostics"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 1. NODE STATUS
echo -e "${BLUE}1. NODE STATUS${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Active Nodes:"
ros2 node list
echo ""

# 2. NODE INFO - FAL
echo -e "${BLUE}2. FAL NODE DETAILS${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Publishers from FAL node:"
ros2 node info /drone_0/fal_node | grep -A 20 "Publishers:"
echo ""
echo "Subscribers from FAL node:"
ros2 node info /drone_0/fal_node | grep -A 20 "Subscribers:"
echo ""
echo "Services from FAL node:"
ros2 node info /drone_0/fal_node | grep -A 10 "Service Servers:"
echo ""
echo "Action Servers from FAL node:"
ros2 node info /drone_0/fal_node | grep -A 10 "Action Servers:"
echo ""

# 3. NODE INFO - TEE
echo -e "${BLUE}3. TEE NODE DETAILS${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Publishers from TEE node:"
ros2 node info /task_execution_engine | grep -A 20 "Publishers:"
echo ""
echo "Subscribers from TEE node:"
ros2 node info /task_execution_engine | grep -A 20 "Subscribers:"
echo ""
echo "Services from TEE node:"
ros2 node info /task_execution_engine | grep -A 10 "Service Servers:"
echo ""
echo "Action Clients from TEE node:"
ros2 node info /task_execution_engine | grep -A 10 "Action Clients:"
echo ""

# 4. CRITICAL TOPICS INFO
echo -e "${BLUE}4. CRITICAL TOPICS ANALYSIS${NC}"
echo "───────────────────────────────────────────────────────────────"

echo -e "${YELLOW}a) Primitive Command Topic (TEE → FAL):${NC}"
ros2 topic info /tee/primitive_command
echo ""

echo -e "${YELLOW}b) Primitive Status Topic (FAL → TEE):${NC}"
ros2 topic info /fal/primitive_status
echo ""

echo -e "${YELLOW}c) Mission Status Topic (TEE → Squadron):${NC}"
ros2 topic info /tee/mission_status
echo ""

echo -e "${YELLOW}d) Battery Health Topic (TEE internal):${NC}"
ros2 topic info /drone_0/tee/health/battery
echo ""

echo -e "${YELLOW}e) GPS Health Topic (TEE internal):${NC}"
ros2 topic info /drone_0/tee/health/gps
echo ""

echo -e "${YELLOW}f) Overall Health Topic (TEE internal):${NC}"
ros2 topic info /drone_0/tee/health/overall
echo ""

echo -e "${YELLOW}g) MAVROS State (MAVROS → FAL):${NC}"
ros2 topic info /drone_0/mavros/state
echo ""

echo -e "${YELLOW}h) MAVROS Battery (MAVROS → TEE):${NC}"
ros2 topic info /drone_0/mavros/battery
echo ""

# 5. TOPIC PUBLICATION RATES
echo -e "${BLUE}5. TOPIC PUBLICATION RATES${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Checking topic rates (5 second sample)..."
echo ""

echo -e "${YELLOW}MAVROS State rate:${NC}"
timeout 5s ros2 topic hz /drone_0/mavros/state 2>&1 | grep "average rate" || echo "No messages"
echo ""

echo -e "${YELLOW}MAVROS Battery rate:${NC}"
timeout 5s ros2 topic hz /drone_0/mavros/battery 2>&1 | grep "average rate" || echo "No messages"
echo ""

echo -e "${YELLOW}MAVROS IMU rate:${NC}"
timeout 5s ros2 topic hz /drone_0/mavros/imu/data 2>&1 | grep "average rate" || echo "No messages"
echo ""

echo -e "${YELLOW}Battery Health rate:${NC}"
timeout 5s ros2 topic hz /drone_0/tee/health/battery 2>&1 | grep "average rate" || echo "No messages"
echo ""

echo -e "${YELLOW}Overall Health rate:${NC}"
timeout 5s ros2 topic hz /drone_0/tee/health/overall 2>&1 | grep "average rate" || echo "No messages"
echo ""

# 6. SERVICE AVAILABILITY
echo -e "${BLUE}6. SERVICE AVAILABILITY CHECK${NC}"
echo "───────────────────────────────────────────────────────────────"

echo -e "${YELLOW}Checking FAL Arm/Disarm service:${NC}"
ros2 service type /drone_0/arm_disarm
echo ""

echo -e "${YELLOW}Checking MAVROS Arming service:${NC}"
ros2 service type /drone_0/mavros/cmd/arming
echo ""

echo -e "${YELLOW}Checking MAVROS Set Mode service:${NC}"
ros2 service type /drone_0/mavros/set_mode
echo ""

# 7. ACTION SERVER STATUS
echo -e "${BLUE}7. ACTION SERVER STATUS${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Available action servers:"
ros2 action list
echo ""

echo -e "${YELLOW}Takeoff action info:${NC}"
ros2 action info /drone_0/takeoff
echo ""

echo -e "${YELLOW}Land action info:${NC}"
ros2 action info /drone_0/land
echo ""

echo -e "${YELLOW}GoToWaypoint action info:${NC}"
ros2 action info /drone_0/goto_waypoint
echo ""

# 8. PARAMETER CHECK
echo -e "${BLUE}8. PARAMETER VERIFICATION${NC}"
echo "───────────────────────────────────────────────────────────────"
echo -e "${YELLOW}FAL Node Parameters:${NC}"
ros2 param list /drone_0/fal_node
echo ""

echo -e "${YELLOW}TEE Node Parameters:${NC}"
ros2 param list /task_execution_engine
echo ""

# 9. TOPIC ECHO SAMPLES
echo -e "${BLUE}9. LIVE DATA SAMPLES${NC}"
echo "───────────────────────────────────────────────────────────────"

echo -e "${YELLOW}MAVROS State (1 sample):${NC}"
timeout 2s ros2 topic echo /drone_0/mavros/state --once 2>&1 || echo "No data received"
echo ""

echo -e "${YELLOW}Battery Health (1 sample):${NC}"
timeout 2s ros2 topic echo /drone_0/tee/health/battery --once 2>&1 || echo "No data received"
echo ""

echo -e "${YELLOW}Overall Health (1 sample):${NC}"
timeout 2s ros2 topic echo /drone_0/tee/health/overall --once 2>&1 || echo "No data received"
echo ""

# 10. INTERFACE DEFINITIONS
echo -e "${BLUE}10. MESSAGE/SERVICE INTERFACE DEFINITIONS${NC}"
echo "───────────────────────────────────────────────────────────────"

echo -e "${YELLOW}PrimitiveCommand message structure:${NC}"
ros2 interface show multi_drone_msgs/msg/PrimitiveCommand
echo ""

echo -e "${YELLOW}PrimitiveStatus message structure:${NC}"
ros2 interface show multi_drone_msgs/msg/PrimitiveStatus
echo ""

echo -e "${YELLOW}ArmDisarm service structure:${NC}"
ros2 interface show multi_drone_msgs/srv/ArmDisarm
echo ""

# 11. COMPUTATION GRAPH
echo -e "${BLUE}11. SYSTEM CONNECTIVITY${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Checking node connections..."
echo ""

echo -e "${YELLOW}TEE → FAL connections:${NC}"
echo "TEE publishes to /tee/primitive_command"
ros2 topic info /tee/primitive_command | grep "Subscription count"
echo ""
echo "FAL subscribes from /tee/primitive_command"
ros2 node info /drone_0/fal_node | grep "/tee/primitive_command"
echo ""

echo -e "${YELLOW}FAL → TEE connections:${NC}"
echo "FAL publishes to /fal/primitive_status"
ros2 topic info /fal/primitive_status | grep "Publisher count"
echo ""
echo "TEE subscribes from /fal/primitive_status"
ros2 node info /task_execution_engine | grep "/fal/primitive_status"
echo ""

# 12. DIAGNOSTICS TOPIC
echo -e "${BLUE}12. ROS DIAGNOSTICS${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Checking /diagnostics topic for system health:"
timeout 2s ros2 topic echo /diagnostics --once 2>&1 | head -20 || echo "No diagnostics data"
echo ""

# 13. TF TRANSFORMS
echo -e "${BLUE}13. TF TRANSFORM TREE${NC}"
echo "───────────────────────────────────────────────────────────────"
echo "Available TF frames:"
timeout 2s ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -10 || echo "No TF data available yet"
echo ""

# 14. SUMMARY
echo -e "${BLUE}14. SYSTEM HEALTH SUMMARY${NC}"
echo "───────────────────────────────────────────────────────────────"

# Count active nodes
NODE_COUNT=$(ros2 node list | wc -l)
echo -e "Active Nodes: ${GREEN}${NODE_COUNT}${NC}"

# Check critical nodes
if ros2 node list | grep -q "/drone_0/fal_node"; then
    echo -e "FAL Node: ${GREEN}✓ Running${NC}"
else
    echo -e "FAL Node: ${RED}✗ Not Found${NC}"
fi

if ros2 node list | grep -q "/task_execution_engine"; then
    echo -e "TEE Node: ${GREEN}✓ Running${NC}"
else
    echo -e "TEE Node: ${RED}✗ Not Found${NC}"
fi

if ros2 node list | grep -q "mavros"; then
    echo -e "MAVROS Node: ${GREEN}✓ Running${NC}"
else
    echo -e "MAVROS Node: ${RED}✗ Not Found${NC}"
fi

# Check critical topics exist
TOPIC_COUNT=$(ros2 topic list | wc -l)
echo -e "Active Topics: ${GREEN}${TOPIC_COUNT}${NC}"

# Check critical services
SERVICE_COUNT=$(ros2 service list | wc -l)
echo -e "Active Services: ${GREEN}${SERVICE_COUNT}${NC}"

# Check critical actions
ACTION_COUNT=$(ros2 action list | wc -l)
echo -e "Active Actions: ${GREEN}${ACTION_COUNT}${NC}"

echo ""
echo "════════════════════════════════════════════════════════════════"
echo -e "  ${GREEN}Diagnostics Complete!${NC}"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "For more detailed analysis, use:"
echo "  - rqt_graph (visual node/topic graph)"
echo "  - rqt_console (live log viewer)"
echo "  - rqt_top (resource usage monitor)"
echo "  - ros2 bag record <topics> (record data for analysis)"
echo ""
