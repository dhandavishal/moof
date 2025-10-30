#!/bin/bash
# Emergency cleanup script to kill all SITL instances
# Use this if the launch script didn't clean up properly

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Emergency SITL Cleanup${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Kill all drone SITL screen sessions
echo -e "${YELLOW}Killing screen sessions...${NC}"
screen -ls | grep drone.*sitl | cut -d. -f1 | awk '{print $1}' | xargs -I {} screen -S {} -X quit 2>/dev/null || true

# Kill sim_vehicle.py processes
echo -e "${YELLOW}Killing sim_vehicle.py processes...${NC}"
pkill -9 -f sim_vehicle.py 2>/dev/null || true

# Kill actual SITL binaries
echo -e "${YELLOW}Killing SITL binaries...${NC}"
pkill -9 -f "arducopter" 2>/dev/null || true
pkill -9 -f "arduplane" 2>/dev/null || true
pkill -9 -f "ardurover" 2>/dev/null || true
pkill -9 -f "ardusub" 2>/dev/null || true
pkill -9 -f "ardupilot" 2>/dev/null || true

# Kill MAVProxy processes
echo -e "${YELLOW}Killing MAVProxy processes...${NC}"
pkill -9 -f "mavproxy" 2>/dev/null || true

# Kill JSBSim processes
echo -e "${YELLOW}Killing JSBSim processes...${NC}"
pkill -9 -f "JSBSim" 2>/dev/null || true

# Wait for processes to terminate
sleep 1

# Show any remaining related processes
echo ""
echo -e "${YELLOW}Checking for remaining SITL processes...${NC}"
remaining=$(ps aux | grep -E "(sim_vehicle|ardu|mavproxy|JSBSim)" | grep -v grep | grep -v kill_all_sitl)
if [ -z "$remaining" ]; then
    echo -e "${GREEN}✓ All SITL processes cleaned up successfully!${NC}"
else
    echo -e "${RED}Warning: Some processes may still be running:${NC}"
    echo "$remaining"
    echo ""
    echo -e "${YELLOW}You may need to kill these manually with:${NC}"
    echo "  ps aux | grep -E \"(sim_vehicle|ardu|mavproxy)\" | grep -v grep"
    echo "  kill -9 <PID>"
fi

echo ""
echo -e "${GREEN}Cleanup complete!${NC}"
