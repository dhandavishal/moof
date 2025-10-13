#!/bin/bash

# Launch script for multiple ArduPilot SITL instances
# This script starts multiple ArduCopter SITL instances for swarm simulation

# Configuration
NUM_DRONES=2
ARDUPILOT_PATH=~/ardupilot  # Adjust this to your ArduPilot installation path
FRAME_TYPE="+"  # Quadcopter frame type

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting ArduPilot SITL Swarm with $NUM_DRONES drones${NC}"

# Kill any existing SITL instances
echo -e "${YELLOW}Cleaning up any existing SITL instances...${NC}"
pkill -f arducopter
pkill -f sim_vehicle.py
sleep 2

# Function to launch a single SITL instance
launch_sitl_instance() {
    local instance=$1
    local sysid=$((instance + 1))
    
    # Port configuration for each instance
    local sim_port=$((5760 + instance * 10))
    local mavlink_port=$((14550 + instance * 10))
    local mavlink2_port=$((14555 + instance * 10))
    
    # Home locations (slightly offset to avoid collision)
    local home_lat=-35.363261
    local home_lon=149.165230
    local home_alt=584
    local home_heading=0
    
    # Offset each drone by 5 meters in the East direction
    local offset=$((instance * 5))
    
    echo -e "${GREEN}Launching Drone $instance (SYSID=$sysid)${NC}"
    echo "  SIM Port: $sim_port"
    echo "  MAVLink Port: $mavlink_port"
    echo "  MAVLink2 Port: $mavlink2_port"
    echo "  Home: $home_lat,$home_lon,$home_alt,$home_heading"
    
    # Create instance directory
    mkdir -p ~/ardupilot_sitl/drone_$instance
    
    # Launch SITL instance in a new terminal
    gnome-terminal --tab --title="Drone$instance SITL" -- bash -c "
        cd $ARDUPILOT_PATH/ArduCopter && \
        ../Tools/autotest/sim_vehicle.py \
            --vehicle=ArduCopter \
            --frame=$FRAME_TYPE \
            --instance=$instance \
            --sysid=$sysid \
            --home=$home_lat,$home_lon,$home_alt,$home_heading \
            --sim-port-in=$sim_port \
            --sim-port-out=$((sim_port + 1)) \
            --mavlink-port=$mavlink_port \
            --mavlink-port2=$mavlink2_port \
            --no-mavproxy \
            --wipe-eeprom \
            --console
        exec bash
    " &
    
    sleep 5  # Wait between launches
}

# Launch all SITL instances
for ((i=0; i<$NUM_DRONES; i++)); do
    launch_sitl_instance $i
done

echo -e "${GREEN}All $NUM_DRONES SITL instances launched!${NC}"
echo -e "${YELLOW}Waiting for SITL initialization...${NC}"
sleep 10

# Display connection information
echo -e "${GREEN}=== Connection Information ===${NC}"
for ((i=0; i<$NUM_DRONES; i++)); do
    echo -e "Drone$i: udp://:$((14555 + i * 10))@127.0.0.1:$((14550 + i * 10))"
done

echo -e "${GREEN}=== Launch ROS2 Swarm Mission ===${NC}"
echo "Now you can launch the ROS2 multi-drone mission with:"
echo -e "${YELLOW}ros2 launch as2_ardu_msn multi_drone_aerostack2.launch.py${NC}"

# Keep script running
echo -e "${GREEN}Press Ctrl+C to stop all SITL instances${NC}"

# Trap Ctrl+C and cleanup
trap cleanup EXIT

cleanup() {
    echo -e "${RED}Stopping all SITL instances...${NC}"
    pkill -f arducopter
    pkill -f sim_vehicle.py
    exit 0
}

# Keep the script running
while true; do
    sleep 1
done