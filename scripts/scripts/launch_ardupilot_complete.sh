#!/bin/bash
# Complete ArduPilot SITL + FAL + TEE Launch Script
# This script integrates with your existing launch_multi_sitl.sh

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}ArduPilot SITL + MAVROS + FAL + TEE Complete Setup${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""

# Configuration
NUM_DRONES=${1:-1}
WORKSPACE_DIR="/home/dhandavishal/multi_drone_ws"
ARDUPILOT_SITL_SCRIPT="$WORKSPACE_DIR/src/moofs_3d/scripts/launch_multi_sitl.sh"

# Check if workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}Error: Workspace not found at $WORKSPACE_DIR${NC}"
    exit 1
fi

# Check if ArduPilot SITL script exists
if [ ! -f "$ARDUPILOT_SITL_SCRIPT" ]; then
    echo -e "${RED}Error: ArduPilot SITL script not found at $ARDUPILOT_SITL_SCRIPT${NC}"
    exit 1
fi

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}Error: tmux not installed. Install with: sudo apt install tmux${NC}"
    exit 1
fi

# Check if workspace is built
if [ ! -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${YELLOW}Warning: Workspace not built${NC}"
    read -p "Build workspace now? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd "$WORKSPACE_DIR"
        colcon build --symlink-install
        if [ $? -ne 0 ]; then
            echo -e "${RED}Build failed!${NC}"
            exit 1
        fi
    else
        exit 1
    fi
fi

echo -e "${GREEN}Configuration:${NC}"
echo -e "  Number of drones: ${YELLOW}$NUM_DRONES${NC}"
echo -e "  Workspace: ${YELLOW}$WORKSPACE_DIR${NC}"
echo ""

# Function to cleanup
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    
    # Kill tmux sessions
    for i in $(seq 0 $((NUM_DRONES - 1))); do
        tmux kill-session -t "ardupilot_drone_${i}" 2>/dev/null || true
    done
    
    # Kill ArduPilot SITL instances
    pkill -f sim_vehicle.py || true
    
    echo -e "${GREEN}Cleanup complete${NC}"
}

trap cleanup EXIT INT TERM

echo -e "${BLUE}Step 1/4: Launching ArduPilot SITL ($NUM_DRONES drones)...${NC}"
echo ""

# Launch ArduPilot SITL in background
"$ARDUPILOT_SITL_SCRIPT" "$NUM_DRONES" copter &
SITL_PID=$!

# Wait for SITL to initialize
echo -e "${YELLOW}Waiting for ArduPilot SITL to initialize (15 seconds)...${NC}"
sleep 15

# Check if SITL is running
if ! ps -p $SITL_PID > /dev/null; then
    echo -e "${RED}Error: ArduPilot SITL failed to start${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ArduPilot SITL running${NC}"
echo ""

# Launch components for each drone
for DRONE_ID in $(seq 0 $((NUM_DRONES - 1))); do
    SESSION_NAME="ardupilot_drone_${DRONE_ID}"
    
    # Calculate ports
    SYSID=$((1 + DRONE_ID))
    MAVPROXY_PORT=$((14550 + DRONE_ID * 10))
    SITL_OUT_PORT=$((14555 + DRONE_ID * 10))
    NAMESPACE="drone_${DRONE_ID}"
    
    echo -e "${BLUE}Launching components for Drone $DRONE_ID (SYSID $SYSID)...${NC}"
    
    # Create tmux session
    tmux new-session -d -s "$SESSION_NAME"
    tmux rename-window -t "$SESSION_NAME:0" "MAVROS"
    
    # Window 0: MAVROS
    echo -e "  ${YELLOW}MAVROS${NC} on ports $MAVPROXY_PORT/$SITL_OUT_PORT"
    MAVROS_CMD="cd $WORKSPACE_DIR && source install/setup.bash && "
    MAVROS_CMD+="ros2 run mavros mavros_node --ros-args "
    MAVROS_CMD+="-r __ns:=/$NAMESPACE "
    MAVROS_CMD+="-p fcu_url:=udp://:${MAVPROXY_PORT}@127.0.0.1:${SITL_OUT_PORT} "
    MAVROS_CMD+="-p tgt_system:=${SYSID} "
    MAVROS_CMD+="-p tgt_component:=1 "
    MAVROS_CMD+="-p fcu_protocol:=v2.0"
    
    tmux send-keys -t "$SESSION_NAME:0" "$MAVROS_CMD" C-m
    
    # Wait for MAVROS to initialize
    sleep 3
    
    # Window 1: FAL
    tmux new-window -t "$SESSION_NAME:1" -n "FAL"
    echo -e "  ${YELLOW}FAL${NC}"
    FAL_CMD="cd $WORKSPACE_DIR && source install/setup.bash && "
    FAL_CMD+="ros2 launch flight_abstraction fal.launch.py "
    FAL_CMD+="drone_namespace:=/$NAMESPACE"
    
    tmux send-keys -t "$SESSION_NAME:1" "$FAL_CMD" C-m
    
    # Wait for FAL to initialize
    sleep 2
    
    # Window 2: TEE
    tmux new-window -t "$SESSION_NAME:2" -n "TEE"
    echo -e "  ${YELLOW}TEE${NC}"
    TEE_CMD="cd $WORKSPACE_DIR && source install/setup.bash && "
    TEE_CMD+="ros2 launch task_execution tee.launch.py"
    
    tmux send-keys -t "$SESSION_NAME:2" "$TEE_CMD" C-m
    
    # Wait for TEE to initialize
    sleep 2
    
    # Window 3: Monitor
    tmux new-window -t "$SESSION_NAME:3" -n "Monitor"
    echo -e "  ${YELLOW}Monitor${NC}"
    MONITOR_CMD="cd $WORKSPACE_DIR && source install/setup.bash && sleep 3 && "
    MONITOR_CMD+="ros2 run task_execution monitor_tee_status"
    
    tmux send-keys -t "$SESSION_NAME:3" "$MONITOR_CMD" C-m
    
    # Window 4: Control Terminal
    tmux new-window -t "$SESSION_NAME:4" -n "Control"
    echo -e "  ${YELLOW}Control Terminal${NC}"
    CONTROL_CMD="cd $WORKSPACE_DIR && source install/setup.bash && "
    CONTROL_CMD+="echo -e '${GREEN}Drone $DRONE_ID Control Terminal${NC}' && "
    CONTROL_CMD+="echo -e '${YELLOW}Send test mission: ros2 run task_execution test_mission_execution${NC}' && "
    CONTROL_CMD+="bash"
    
    tmux send-keys -t "$SESSION_NAME:4" "$CONTROL_CMD" C-m
    
    echo -e "  ${GREEN}✓ Drone $DRONE_ID components launched${NC}"
    echo ""
done

echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}All components launched successfully!${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo -e "${YELLOW}Tmux Sessions:${NC}"
for DRONE_ID in $(seq 0 $((NUM_DRONES - 1))); do
    echo -e "  Drone $DRONE_ID: ${BLUE}tmux attach -t ardupilot_drone_${DRONE_ID}${NC}"
done
echo ""
echo -e "${YELLOW}Tmux Window Navigation:${NC}"
echo -e "  Switch windows: ${BLUE}Ctrl+B then number (0-4)${NC}"
echo -e "    0: MAVROS"
echo -e "    1: FAL"
echo -e "    2: TEE"
echo -e "    3: Monitor"
echo -e "    4: Control Terminal"
echo -e "  Detach: ${BLUE}Ctrl+B then D${NC}"
echo ""
echo -e "${YELLOW}Testing:${NC}"
echo -e "  1. Attach to control terminal: ${BLUE}tmux attach -t ardupilot_drone_0${NC}"
echo -e "  2. Switch to window 4: ${BLUE}Ctrl+B then 4${NC}"
echo -e "  3. Send test mission: ${BLUE}ros2 run task_execution test_mission_execution${NC}"
echo ""
echo -e "${YELLOW}Monitoring:${NC}"
echo -e "  MAVROS state: ${BLUE}ros2 topic echo /drone_0/mavros/state${NC}"
echo -e "  TEE status: ${BLUE}ros2 topic echo /tee/mission_status${NC}"
echo -e "  GPS: ${BLUE}ros2 topic echo /drone_0/mavros/global_position/global${NC}"
echo ""
echo -e "${YELLOW}Cleanup:${NC}"
echo -e "  Kill all: ${BLUE}tmux kill-session -t ardupilot_drone_0${NC} (repeat for each drone)"
echo -e "  Or: ${BLUE}Ctrl+C this script${NC}"
echo ""

# Offer to attach to first drone
if [ "$NUM_DRONES" -eq 1 ]; then
    read -p "Attach to drone_0 tmux session now? (Y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        tmux attach -t "ardupilot_drone_0"
    fi
fi

# Keep script running to maintain trap
echo -e "${GREEN}Press Ctrl+C to stop all components${NC}"
wait $SITL_PID
