#!/bin/bash
# Test Squadron Manager with multi-drone missions

echo "=========================================="
echo "Squadron Manager Multi-Drone Test"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if squadron manager is running
echo -e "${YELLOW}Checking if Squadron Manager is running...${NC}"
if ros2 node list | grep -q "squadron_manager"; then
    echo -e "${GREEN}✓ Squadron Manager is running${NC}"
else
    echo -e "${RED}✗ Squadron Manager not found${NC}"
    echo "Please launch the squadron system first:"
    echo "  ros2 launch task_execution multi_drone_squadron.launch.py num_drones:=3"
    exit 1
fi

# Check available drones
echo ""
echo -e "${YELLOW}Checking available drones...${NC}"
DRONE_COUNT=0
for i in {0..9}; do
    if ros2 node list | grep -q "drone_${i}"; then
        echo -e "${GREEN}✓ Drone ${i} found${NC}"
        DRONE_COUNT=$((DRONE_COUNT + 1))
    fi
done

if [ $DRONE_COUNT -eq 0 ]; then
    echo -e "${RED}No drones found!${NC}"
    exit 1
fi

echo -e "${GREEN}Found ${DRONE_COUNT} drone(s)${NC}"
echo ""

# Function to send mission
send_mission() {
    local mission_type=$1
    local mission_json=$2
    
    echo -e "${BLUE}Sending ${mission_type} mission...${NC}"
    ros2 topic pub --once /squadron/mission_command std_msgs/msg/String "{data: '${mission_json}'}"
    echo -e "${GREEN}✓ Mission sent${NC}"
    echo ""
}

# Menu
while true; do
    echo "=========================================="
    echo "Test Options:"
    echo "=========================================="
    echo "1. Check squadron status"
    echo "2. Send single waypoint mission (auto-allocated)"
    echo "3. Send multiple missions (one per drone)"
    echo "4. Create line formation"
    echo "5. Create wedge formation"
    echo "6. Create circle formation"
    echo "7. Send formation mission"
    echo "8. Monitor all drone statuses"
    echo "9. Emergency stop all drones"
    echo "0. Exit"
    echo ""
    read -p "Enter option: " option
    
    case $option in
        1)
            echo ""
            echo "=========================================="
            echo "Squadron Status"
            echo "=========================================="
            timeout 2 ros2 topic echo /squadron/status --once 2>/dev/null || echo "No status available yet"
            echo ""
            ;;
            
        2)
            echo ""
            MISSION_ID="single_$(date +%s)"
            MISSION="{
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
            }"
            send_mission "single waypoint" "$MISSION"
            echo "Mission ID: ${MISSION_ID}"
            echo "Monitor with: ros2 topic echo /drone_0/tee/mission_status"
            sleep 2
            ;;
            
        3)
            echo ""
            echo -e "${BLUE}Sending ${DRONE_COUNT} missions (one per drone)...${NC}"
            for ((i=0; i<DRONE_COUNT; i++)); do
                MISSION_ID="multi_${i}_$(date +%s)"
                X=$((10 + i * 20))
                Y=$((10 + i * 20))
                MISSION="{
                    \"mission_id\": \"${MISSION_ID}\",
                    \"task_type\": \"waypoint\",
                    \"priority\": 100,
                    \"timeout\": 120.0,
                    \"multi_drone\": false,
                    \"parameters\": {
                        \"waypoints\": [{\"x\": ${X}.0, \"y\": ${Y}.0, \"z\": 15.0, \"yaw\": 0.0}],
                        \"velocity\": 2.0,
                        \"acceptance_radius\": 1.0
                    }
                }"
                send_mission "mission for drone ${i}" "$MISSION"
                echo "Mission ID: ${MISSION_ID} -> Waypoint (${X}, ${Y}, 15)"
                sleep 1
            done
            echo -e "${GREEN}All missions sent!${NC}"
            echo ""
            ;;
            
        4)
            echo ""
            FORMATION="{
                \"type\": \"create\",
                \"formation_type\": \"line\",
                \"spacing\": 10.0,
                \"altitude\": 50.0,
                \"center\": [0.0, 0.0, 50.0]
            }"
            echo -e "${BLUE}Creating line formation...${NC}"
            ros2 topic pub --once /squadron/formation_command std_msgs/msg/String "{data: '${FORMATION}'}"
            echo -e "${GREEN}✓ Line formation command sent${NC}"
            echo ""
            ;;
            
        5)
            echo ""
            FORMATION="{
                \"type\": \"create\",
                \"formation_type\": \"wedge\",
                \"spacing\": 12.0,
                \"altitude\": 50.0,
                \"center\": [0.0, 0.0, 50.0]
            }"
            echo -e "${BLUE}Creating wedge formation...${NC}"
            ros2 topic pub --once /squadron/formation_command std_msgs/msg/String "{data: '${FORMATION}'}"
            echo -e "${GREEN}✓ Wedge formation command sent${NC}"
            echo ""
            ;;
            
        6)
            echo ""
            FORMATION="{
                \"type\": \"create\",
                \"formation_type\": \"circle\",
                \"spacing\": 15.0,
                \"altitude\": 50.0,
                \"center\": [0.0, 0.0, 50.0]
            }"
            echo -e "${BLUE}Creating circle formation...${NC}"
            ros2 topic pub --once /squadron/formation_command std_msgs/msg/String "{data: '${FORMATION}'}"
            echo -e "${GREEN}✓ Circle formation command sent${NC}"
            echo ""
            ;;
            
        7)
            echo ""
            MISSION_ID="formation_mission_$(date +%s)"
            MISSION="{
                \"mission_id\": \"${MISSION_ID}\",
                \"task_type\": \"waypoint\",
                \"priority\": 100,
                \"timeout\": 180.0,
                \"multi_drone\": true,
                \"formation_type\": \"wedge\",
                \"spacing\": 15.0,
                \"altitude\": 50.0,
                \"parameters\": {
                    \"waypoints\": [{\"x\": 50.0, \"y\": 50.0, \"z\": 50.0, \"yaw\": 0.0}],
                    \"velocity\": 2.0
                }
            }"
            send_mission "formation mission" "$MISSION"
            echo "Mission ID: ${MISSION_ID}"
            echo ""
            ;;
            
        8)
            echo ""
            echo "=========================================="
            echo "All Drone Statuses"
            echo "=========================================="
            for ((i=0; i<DRONE_COUNT; i++)); do
                echo -e "${YELLOW}Drone ${i}:${NC}"
                timeout 1 ros2 topic echo /drone_${i}/tee/mission_status --once 2>/dev/null || echo "No status"
                echo ""
            done
            ;;
            
        9)
            echo ""
            echo -e "${RED}EMERGENCY STOP - Sending RTL to all drones${NC}"
            read -p "Are you sure? (yes/no): " confirm
            if [ "$confirm" = "yes" ]; then
                for ((i=0; i<DRONE_COUNT; i++)); do
                    echo "Stopping drone ${i}..."
                    # Send emergency mission (land in place)
                    EMERGENCY_ID="emergency_${i}_$(date +%s)"
                    MISSION="{
                        \"mission_id\": \"${EMERGENCY_ID}\",
                        \"task_type\": \"emergency_land\",
                        \"priority\": 999,
                        \"timeout\": 30.0,
                        \"multi_drone\": false,
                        \"parameters\": {}
                    }"
                    ros2 topic pub --once /squadron/mission_command std_msgs/msg/String "{data: '${MISSION}'}"
                done
                echo -e "${GREEN}Emergency stop commands sent${NC}"
            fi
            echo ""
            ;;
            
        0)
            echo "Exiting..."
            exit 0
            ;;
            
        *)
            echo -e "${RED}Invalid option${NC}"
            echo ""
            ;;
    esac
done
