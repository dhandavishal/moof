#!/bin/bash
# Launch multiple ArduPilot SITL instances for squadron testing

echo "=========================================="
echo "Multi-Drone SITL Launcher"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Number of drones (default: 3)
NUM_DRONES=${1:-3}

# Check if ArduPilot is installed
if [ ! -d "$HOME/ardupilot" ]; then
    echo -e "${RED}Error: ArduPilot not found at $HOME/ardupilot${NC}"
    echo "Please clone ArduPilot first:"
    echo "  git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot"
    exit 1
fi

echo -e "${YELLOW}Starting ${NUM_DRONES} ArduPilot SITL instances...${NC}"
echo ""

# Array to store PIDs
PIDS=()

# Launch each SITL instance
for ((i=0; i<NUM_DRONES; i++)); do
    # Calculate ports
    # Drone 0: 14550/14555 (out/in), SITL: 5760
    # Drone 1: 14560/14565 (out/in), SITL: 5770
    # Drone 2: 14570/14575 (out/in), SITL: 5780
    
    SITL_PORT=$((5760 + i * 10))
    OUT_PORT=$((14550 + i * 10))
    
    # Different home location for each drone (offset by 50m)
    LAT=$(echo "-35.363261 + $i * 0.0005" | bc)
    LON="149.165230"
    ALT="584"
    HDG="0"
    
    HOME_LOC="${LAT},${LON},${ALT},${HDG}"
    
    # Create instance directory
    INSTANCE_DIR="/tmp/ardupilot_sitl_${i}"
    mkdir -p ${INSTANCE_DIR}
    
    echo -e "${GREEN}Drone ${i}:${NC}"
    echo "  - SITL Port: ${SITL_PORT}"
    echo "  - MAVLink Out: ${OUT_PORT}"
    echo "  - Home: ${HOME_LOC}"
    echo "  - Directory: ${INSTANCE_DIR}"
    
    # Launch SITL
    cd ${INSTANCE_DIR}
    
    sim_vehicle.py \
        -v ArduCopter \
        --instance ${i} \
        --out 127.0.0.1:${OUT_PORT} \
        -L CMAC \
        --home=${HOME_LOC} \
        --console \
        --map \
        > ${INSTANCE_DIR}/sitl.log 2>&1 &
    
    PID=$!
    PIDS+=($PID)
    
    echo "  - PID: ${PID}"
    echo ""
    
    # Small delay between launches
    sleep 2
done

echo -e "${GREEN}All SITL instances started!${NC}"
echo ""
echo "PIDs: ${PIDS[@]}"
echo ""
echo "To stop all instances, run:"
echo "  kill ${PIDS[@]}"
echo ""
echo "Or save PIDs to file:"
echo "${PIDS[@]}" > /tmp/ardupilot_sitl_pids.txt
echo "PIDs saved to /tmp/ardupilot_sitl_pids.txt"
echo ""
echo -e "${YELLOW}Waiting for SITL instances to initialize (30s)...${NC}"
sleep 30

echo -e "${GREEN}✓ SITL instances ready!${NC}"
echo ""
echo "Now launch the ROS2 squadron system:"
echo "  ros2 launch task_execution multi_drone_squadron.launch.py num_drones:=${NUM_DRONES}"
