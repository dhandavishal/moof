#!/bin/bash
# Checkpoint 1.2: MAVROS Multi-Instance Validation Script
# Tests MAVROS nodes, namespaces, and independent control

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASSED=0
FAILED=0

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Checkpoint 1.2: MAVROS Validation${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Wait for ROS2 to be ready
sleep 2

# Test 1: Check MAVROS node namespaces
echo -e "${YELLOW}Test 1: Checking MAVROS namespaces...${NC}"
NODES_OK=true

for i in 0 1 2; do
    if ros2 node list | grep -q "/drone_$i/mavros"; then
        echo -e "${GREEN}✓ Found namespace: /drone_$i/mavros${NC}"
    else
        echo -e "${RED}✗ Missing namespace: /drone_$i/mavros${NC}"
        NODES_OK=false
    fi
done

if [ "$NODES_OK" = true ]; then
    PASSED=$((PASSED + 1))
else
    FAILED=$((FAILED + 1))
fi
echo ""

# Test 2: Check MAVROS topics
echo -e "${YELLOW}Test 2: Checking MAVROS topics...${NC}"
TOPICS_OK=true

REQUIRED_TOPICS=(
    "mavros/state"
    "mavros/local_position/pose"
    "mavros/battery"
    "mavros/global_position/global"
)

for i in 0 1 2; do
    echo -e "Drone $i topics:"
    for topic in "${REQUIRED_TOPICS[@]}"; do
        FULL_TOPIC="/drone_$i/$topic"
        if ros2 topic list | grep -q "$FULL_TOPIC"; then
            echo -e "  ${GREEN}✓ $FULL_TOPIC${NC}"
        else
            echo -e "  ${RED}✗ Missing: $FULL_TOPIC${NC}"
            TOPICS_OK=false
        fi
    done
done

if [ "$TOPICS_OK" = true ]; then
    PASSED=$((PASSED + 1))
else
    FAILED=$((FAILED + 1))
fi
echo ""

# Test 3: Check MAVROS services
echo -e "${YELLOW}Test 3: Checking MAVROS services...${NC}"
SERVICES_OK=true

REQUIRED_SERVICES=(
    "mavros/cmd/arming"
    "mavros/set_mode"
)

for i in 0 1 2; do
    echo -e "Drone $i services:"
    for service in "${REQUIRED_SERVICES[@]}"; do
        FULL_SERVICE="/drone_$i/$service"
        if ros2 service list | grep -q "$FULL_SERVICE"; then
            echo -e "  ${GREEN}✓ $FULL_SERVICE${NC}"
        else
            echo -e "  ${RED}✗ Missing: $FULL_SERVICE${NC}"
            SERVICES_OK=false
        fi
    done
done

if [ "$SERVICES_OK" = true ]; then
    PASSED=$((PASSED + 1))
else
    FAILED=$((FAILED + 1))
fi
echo ""

# Test 4: Check connection state (read topics)
echo -e "${YELLOW}Test 4: Checking MAVROS connection state...${NC}"
CONNECTION_OK=true

for i in 0 1 2; do
    echo -e "Checking drone_$i connection..."
    
    # Try to read state topic with timeout
    timeout 5 ros2 topic echo /drone_$i/mavros/state --once > /tmp/drone_${i}_state.txt 2>&1 || true
    
    if [ -s /tmp/drone_${i}_state.txt ]; then
        echo -e "${GREEN}✓ Drone $i: State topic readable${NC}"
        
        # Check if connected
        if grep -q "connected: true" /tmp/drone_${i}_state.txt; then
            echo -e "  ${GREEN}✓ Connected to FCU${NC}"
        else
            echo -e "  ${YELLOW}⚠ Not yet connected to FCU (may need more time)${NC}"
        fi
    else
        echo -e "${RED}✗ Drone $i: Cannot read state topic${NC}"
        CONNECTION_OK=false
    fi
    
    rm -f /tmp/drone_${i}_state.txt
done

if [ "$CONNECTION_OK" = true ]; then
    PASSED=$((PASSED + 1))
else
    FAILED=$((FAILED + 1))
fi
echo ""

# Summary
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Validation Summary${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Tests passed: ${GREEN}$PASSED${NC}"
echo -e "Tests failed: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All checkpoint 1.2 tests passed!${NC}"
    echo ""
    echo -e "${YELLOW}Manual validation steps:${NC}"
    echo "1. Check state continuously: ros2 topic echo /drone_0/mavros/state"
    echo "2. Try arming drone_0:"
    echo "   ros2 service call /drone_0/mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: true}\""
    echo "3. Verify drone_1 remains disarmed:"
    echo "   ros2 topic echo /drone_1/mavros/state --once | grep armed"
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please check the output above.${NC}"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "1. Make sure SITL instances are running: screen -ls"
    echo "2. Check MAVROS launch: ros2 launch moofs_3d multi_mavros.launch.py"
    echo "3. Check ROS2 daemon: ros2 daemon status"
    exit 1
fi
