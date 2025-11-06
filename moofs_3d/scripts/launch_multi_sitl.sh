#!/bin/bash
# Multi-Drone SITL Launcher for ArduPilot
# Usage: ./launch_multi_sitl.sh <num_drones> [vehicle_type]
# Example: ./launch_multi_sitl.sh 3 copter

set -e

# --- Configuration ---
NUM_DRONES=${1:-3}
VEHICLE_TYPE=${2:-copter}

# Path to your ArduPilot project and venv activate script
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot/ardupilot}"
VENV_PATH="${VENV_PATH:-$HOME/ardupilot/bin/activate}"

# --- Internal Variables ---
BASE_SYSID=1
BASE_PORT_MAVPROXY=14550
BASE_PORT_SITL_OUT=14555
BASE_PORT_SITL_IN=14560
SPACING=10  # meters between drones
AUTOTEST_DIR="$ARDUPILOT_DIR/Tools/autotest"

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
echo -e "Venv Path: ${YELLOW}${VENV_PATH}${NC}"
echo ""

# --- Sanity Checks ---

# Check if ArduPilot directory exists
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo -e "${RED}Error: ArduPilot directory not found at $ARDUPILOT_DIR${NC}"
    echo "Please set ARDUPILOT_DIR environment variable correctly."
    exit 1
fi

# Check if venv activate script exists
if [ ! -f "$VENV_PATH" ]; then
    echo -e "${RED}Error: Virtual environment activate script not found at $VENV_PATH${NC}"
    echo "Please set VENV_PATH environment variable correctly."
    exit 1
fi

# Check if sim_vehicle.py exists
if [ ! -f "$AUTOTEST_DIR/sim_vehicle.py" ]; then
    echo -e "${RED}Error: sim_vehicle.py not found in $AUTOTEST_DIR${NC}"
    exit 1
fi

# Check if screen is installed
if ! command -v screen &> /dev/null; then
    echo -e "${RED}Error: 'screen' command not found. Please install it:${NC}"
    echo "sudo apt-get install screen"
    exit 1
fi

# Array to store screen session names
declare -a SCREEN_SESSIONS=()

# Function to kill all SITL instances
cleanup() {
    echo -e "\n${YELLOW}Cleaning up SITL instances...${NC}"
    
    # Kill all screen sessions we created
    for session in "${SCREEN_SESSIONS[@]}"; do
        echo -e "${YELLOW}  Terminating screen session: $session${NC}"
        screen -S "$session" -X quit 2>/dev/null || true
    done
    
    # Kill sim_vehicle.py processes
    echo -e "${YELLOW}  Killing sim_vehicle.py processes...${NC}"
    pkill -9 -f sim_vehicle.py 2>/dev/null || true
    
    # Kill actual SITL binaries (arducopter, arduplane, etc.)
    echo -e "${YELLOW}  Killing SITL binaries...${NC}"
    pkill -9 -f "ardu${VEHICLE_TYPE}" 2>/dev/null || true
    pkill -9 -f "ardupilot" 2>/dev/null || true
    
    # Kill MAVProxy processes
    echo -e "${YELLOW}  Killing MAVProxy processes...${NC}"
    pkill -9 -f "mavproxy" 2>/dev/null || true
    
    # Additional cleanup for any orphaned processes
    pkill -9 -f "JSBSim" 2>/dev/null || true
    
    # Wait a moment for processes to terminate
    sleep 1
    
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Set up trap to cleanup on script exit
trap cleanup EXIT INT TERM SIGTERM SIGINT

# Kill any existing SITL instances before starting
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
    
    # Screen session name
    SESSION_NAME="drone_${DRONE_ID}_sitl"
    
    echo -e "${GREEN}Drone $DRONE_ID:${NC}"
    echo "  SYSID: $SYSID"
    echo "  MAVProxy port: $MAVPROXY_PORT"
    echo "  SITL out port: $SITL_OUT_PORT"
    echo "  Instance: $INSTANCE"
    echo "  Screen session: $SESSION_NAME"
    
    # Create instance directory for logs and parameters
    INSTANCE_DIR="$HOME/sitl_instances/drone_${DRONE_ID}"
    mkdir -p "$INSTANCE_DIR"
    
    # Create custom parameter file for this drone
    PARAM_FILE="$INSTANCE_DIR/params.parm"
    cat > "$PARAM_FILE" <<EOF
SYSID_THISMAV $SYSID
# Add any other drone-specific parameters here
EOF
    
    # Build the sim_vehicle command to include venv activation
    SIM_CMD="source $VENV_PATH && "
    SIM_CMD+="cd $AUTOTEST_DIR && "
    SIM_CMD+="./sim_vehicle.py "
    SIM_CMD+="-v $VEHICLE_TYPE "
    SIM_CMD+="-I $INSTANCE "
    #SIM_CMD+="--out=127.0.0.1:$MAVPROXY_PORT "
    SIM_CMD+="--out=127.0.0.1:$SITL_OUT_PORT "
    SIM_CMD+="--model + "
    SIM_CMD+="--speedup 1 "
    SIM_CMD+="--console "
    SIM_CMD+="--add-param-file=$PARAM_FILE "
    
    # Launch in a detached screen session
    # Note: We don't use 'exec bash' so the screen terminates when sim_vehicle exits
    screen -dmS "$SESSION_NAME" bash -c "$SIM_CMD"
    
    # Add session to our tracking array
    SCREEN_SESSIONS+=("$SESSION_NAME")
    
    echo -e "  ${GREEN}✓ Launched${NC}"
    echo ""
    
    # Small delay between launches to prevent race conditions
    sleep 2
done

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}All SITL instances launched!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Management Commands:${NC}"
echo "  List sessions:    screen -ls"
echo "  Attach to drone N: screen -r drone_N_sitl (e.g., screen -r drone_0_sitl)"
echo "  Detach from screen: Ctrl+A then D"
echo ""
echo -e "${YELLOW}Connect QGroundControl or Mission Planner:${NC}"
for i in $(seq 0 $((NUM_DRONES - 1))); do
    MAVPROXY_PORT=$((BASE_PORT_MAVPROXY + i * 10))
    echo "  Drone $i: Connect via UDP to 127.0.0.1:$MAVPROXY_PORT"
done
echo ""
echo -e "${GREEN}Press Ctrl+C to stop all SITL instances.${NC}"
echo ""
echo -e "${YELLOW}Monitoring SITL processes...${NC}"
echo -e "${YELLOW}(Script will automatically clean up all processes on exit)${NC}"

# Keep script running to hold the trap active
# Use a sleep loop instead of wait since screen sessions aren't child processes
while true; do
    sleep 1
done