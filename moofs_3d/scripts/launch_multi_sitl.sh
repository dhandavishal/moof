#!/bin/bash
# Multi-Drone SITL Launcher for ArduPilot
# Usage: ./launch_multi_sitl.sh <num_drones> [vehicle_type]
# Example: ./launch_multi_sitl.sh 3 copter

set -e

# Configuration
NUM_DRONES=${1:-3}
VEHICLE_TYPE=${2:-copter}
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
BASE_SYSID=1
BASE_PORT_MAVPROXY=14550
BASE_PORT_SITL_OUT=14555
BASE_PORT_SITL_IN=14560
SPACING=10  # meters between drones

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Multi-Drone SITL Launcher${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Number of drones: ${YELLOW}${NUM_DRONES}${NC}"
echo -e "Vehicle type: ${YELLOW}${VEHICLE_TYPE}${NC}"
echo -e "ArduPilot directory: ${YELLOW}${ARDUPILOT_DIR}${NC}"
echo ""

# Check if ArduPilot directory exists
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo -e "${RED}Error: ArduPilot directory not found at $ARDUPILOT_DIR${NC}"
    echo "Please set ARDUPILOT_DIR environment variable or install ArduPilot"
    exit 1
fi

# Check if sim_vehicle.py exists
if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
    echo -e "${RED}Error: sim_vehicle.py not found${NC}"
    exit 1
fi

# Check if screen is installed
if ! command -v screen &> /dev/null; then
    echo -e "${RED}Error: 'screen' command not found. Please install it:${NC}"
    echo "sudo apt-get install screen"
    exit 1
fi

# Function to kill all SITL instances
cleanup() {
    echo -e "\n${YELLOW}Cleaning up SITL instances...${NC}"
    for i in $(seq 0 $((NUM_DRONES - 1))); do
        screen -S drone_${i}_sitl -X quit 2>/dev/null || true
    done
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Set up trap to cleanup on script exit
trap cleanup EXIT INT TERM

# Kill any existing SITL instances
cleanup

echo -e "${GREEN}Launching $NUM_DRONES SITL instances...${NC}"
echo ""

# Launch each SITL instance in a screen session
for i in $(seq 0 $((NUM_DRONES - 1))); do
    DRONE_ID=$i
    SYSID=$((BASE_SYSID + DRONE_ID))
    INSTANCE=$DRONE_ID
    
    # Calculate ports
    MAVPROXY_PORT=$((BASE_PORT_MAVPROXY + DRONE_ID * 10))
    SITL_OUT_PORT=$((BASE_PORT_SITL_OUT + DRONE_ID * 10))
    SITL_IN_PORT=$((BASE_PORT_SITL_IN + DRONE_ID * 10))
    
    # Calculate starting position (offset by SPACING meters in X direction)
    START_LOC_X=$(echo "$DRONE_ID * $SPACING" | bc)
    START_LOC="0,0,0,0"  # Will be set via parameters
    
    # Screen session name
    SESSION_NAME="drone_${DRONE_ID}_sitl"
    
    echo -e "${GREEN}Drone $DRONE_ID:${NC}"
    echo "  SYSID: $SYSID"
    echo "  MAVProxy port: $MAVPROXY_PORT"
    echo "  SITL out port: $SITL_OUT_PORT"
    echo "  SITL in port: $SITL_IN_PORT"
    echo "  Instance: $INSTANCE"
    echo "  Screen session: $SESSION_NAME"
    
    # Create instance directory
    INSTANCE_DIR="$HOME/sitl_instances/drone_${DRONE_ID}"
    mkdir -p "$INSTANCE_DIR"
    
    # Build sim_vehicle command
    SIM_CMD="cd $ARDUPILOT_DIR/Tools/autotest && "
    SIM_CMD+="python3 sim_vehicle.py "
    SIM_CMD+="-v ArduCopter "
    SIM_CMD+="-I $INSTANCE "
    SIM_CMD+="--out=127.0.0.1:$MAVPROXY_PORT "
    SIM_CMD+="--out=127.0.0.1:$SITL_OUT_PORT "
    SIM_CMD+="--model + "
    SIM_CMD+="--speedup 1 "
    SIM_CMD+="--defaults $ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm "
    SIM_CMD+="--add-param-file=$INSTANCE_DIR/params.parm "
    
    # Create custom parameter file for this drone
    cat > "$INSTANCE_DIR/params.parm" <<EOF
SYSID_THISMAV $SYSID
SERIAL1_PROTOCOL 2
SERIAL1_BAUD 921600
EOF
    
    # Launch in screen session
    screen -dmS "$SESSION_NAME" bash -c "$SIM_CMD; exec bash"
    
    echo -e "  ${GREEN}✓ Launched${NC}"
    echo ""
    
    # Small delay between launches
    sleep 2
done

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All SITL instances launched!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Management Commands:${NC}"
echo "  List sessions:    screen -ls"
echo "  Attach to drone N: screen -r drone_N_sitl"
echo "  Detach from screen: Ctrl+A then D"
echo "  Kill all:         ./scripts/kill_multi_sitl.sh"
echo ""
echo -e "${YELLOW}Port Mapping:${NC}"
for i in $(seq 0 $((NUM_DRONES - 1))); do
    MAVPROXY_PORT=$((BASE_PORT_MAVPROXY + i * 10))
    SITL_OUT_PORT=$((BASE_PORT_SITL_OUT + i * 10))
    echo "  Drone $i: MAVProxy=$MAVPROXY_PORT, SITL_OUT=$SITL_OUT_PORT"
done
echo ""
echo -e "${YELLOW}Connect QGroundControl to individual drones:${NC}"
echo "  Use UDP ports: 14550, 14560, 14570, ..."
echo ""
echo -e "${GREEN}Press Ctrl+C to stop all SITL instances${NC}"

# Keep script running
while true; do
    sleep 1
    
    # Check if all screen sessions are still running
    RUNNING=0
    for i in $(seq 0 $((NUM_DRONES - 1))); do
        if screen -list | grep -q "drone_${i}_sitl"; then
            RUNNING=$((RUNNING + 1))
        fi
    done
    
    if [ $RUNNING -eq 0 ]; then
        echo -e "\n${RED}All SITL instances have stopped${NC}"
        break
    fi
done
