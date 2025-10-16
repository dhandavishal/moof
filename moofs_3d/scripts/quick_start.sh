#!/bin/bash
# Quick Start Script for MOOFS-3D Multi-Drone System
# This script helps set up and launch the entire system

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   MOOFS-3D Multi-Drone System Quick Start     ║${NC}"
echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo ""

# Function to check prerequisites
check_prerequisites() {
    echo -e "${YELLOW}Checking prerequisites...${NC}"
    
    # Check ROS2
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}✗ ROS2 not sourced. Please source ROS2 setup.bash first${NC}"
        exit 1
    else
        echo -e "${GREEN}✓ ROS2 ($ROS_DISTRO) detected${NC}"
    fi
    
    # Check MAVROS
    if ros2 pkg list | grep -q mavros; then
        echo -e "${GREEN}✓ MAVROS installed${NC}"
    else
        echo -e "${RED}✗ MAVROS not found${NC}"
        echo "Install with: sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras"
        exit 1
    fi
    
    # Check screen
    if command -v screen &> /dev/null; then
        echo -e "${GREEN}✓ screen installed${NC}"
    else
        echo -e "${RED}✗ screen not found${NC}"
        echo "Install with: sudo apt install screen"
        exit 1
    fi
    
    # Check ArduPilot
    if [ -z "$ARDUPILOT_DIR" ]; then
        if [ -d "$HOME/ardupilot" ]; then
            export ARDUPILOT_DIR="$HOME/ardupilot"
            echo -e "${GREEN}✓ ArduPilot found at $ARDUPILOT_DIR${NC}"
        else
            echo -e "${RED}✗ ArduPilot not found${NC}"
            echo "Please set ARDUPILOT_DIR or install ArduPilot to ~/ardupilot"
            exit 1
        fi
    else
        echo -e "${GREEN}✓ ArduPilot directory: $ARDUPILOT_DIR${NC}"
    fi
    
    echo ""
}

# Function to show menu
show_menu() {
    echo -e "${BLUE}Select an option:${NC}"
    echo "1) Launch Multi-SITL (3 drones)"
    echo "2) Launch Multi-SITL (custom number)"
    echo "3) Launch MAVROS only"
    echo "4) Launch Full System (SITL + MAVROS)"
    echo "5) Run Checkpoint 1.1 Validation"
    echo "6) Run Checkpoint 1.2 Validation"
    echo "7) Stop All"
    echo "8) Exit"
    echo ""
    read -p "Enter choice [1-8]: " choice
}

# Function to launch SITL
launch_sitl() {
    NUM_DRONES=$1
    echo -e "${YELLOW}Launching $NUM_DRONES SITL instances...${NC}"
    ./scripts/launch_multi_sitl.sh $NUM_DRONES &
    SITL_PID=$!
    echo -e "${GREEN}SITL launched (PID: $SITL_PID)${NC}"
    sleep 5
}

# Function to launch MAVROS
launch_mavros() {
    NUM_DRONES=$1
    echo -e "${YELLOW}Launching MAVROS for $NUM_DRONES drones...${NC}"
    ros2 launch moofs_3d multi_mavros.launch.py num_drones:=$NUM_DRONES
}

# Main script
check_prerequisites

while true; do
    show_menu
    
    case $choice in
        1)
            echo ""
            launch_sitl 3
            echo ""
            echo -e "${GREEN}SITL running. Press Enter to continue...${NC}"
            read
            ;;
        2)
            echo ""
            read -p "Enter number of drones: " num
            launch_sitl $num
            echo ""
            echo -e "${GREEN}SITL running. Press Enter to continue...${NC}"
            read
            ;;
        3)
            echo ""
            read -p "Enter number of drones: " num
            launch_mavros $num
            ;;
        4)
            echo ""
            read -p "Enter number of drones [3]: " num
            num=${num:-3}
            launch_sitl $num
            echo ""
            echo -e "${YELLOW}Waiting for SITL to initialize...${NC}"
            sleep 10
            echo ""
            launch_mavros $num
            ;;
        5)
            echo ""
            ./scripts/validate_checkpoint_1_1.sh
            echo ""
            echo -e "${GREEN}Press Enter to continue...${NC}"
            read
            ;;
        6)
            echo ""
            ./scripts/validate_checkpoint_1_2.sh
            echo ""
            echo -e "${GREEN}Press Enter to continue...${NC}"
            read
            ;;
        7)
            echo ""
            echo -e "${YELLOW}Stopping all instances...${NC}"
            ./scripts/kill_multi_sitl.sh
            pkill -f mavros || true
            echo -e "${GREEN}All stopped${NC}"
            echo ""
            ;;
        8)
            echo ""
            echo -e "${GREEN}Goodbye!${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice${NC}"
            ;;
    esac
done
