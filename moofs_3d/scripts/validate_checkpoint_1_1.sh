#!/bin/bash
# Checkpoint 1.1: SITL Validation Script
# Tests multi-SITL instance launch and validation

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASSED=0
FAILED=0

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Checkpoint 1.1: SITL Validation${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Test 1: Check if all screen sessions are running
echo -e "${YELLOW}Test 1: Checking SITL screen sessions...${NC}"
NUM_EXPECTED=3
NUM_RUNNING=$(screen -ls | grep -c "drone_.*_sitl" || true)

if [ "$NUM_RUNNING" -eq "$NUM_EXPECTED" ]; then
    echo -e "${GREEN}✓ PASS: All $NUM_EXPECTED SITL instances running${NC}"
    PASSED=$((PASSED + 1))
else
    echo -e "${RED}✗ FAIL: Expected $NUM_EXPECTED instances, found $NUM_RUNNING${NC}"
    FAILED=$((FAILED + 1))
fi
echo ""

# Test 2: Check unique UDP ports
echo -e "${YELLOW}Test 2: Checking UDP ports...${NC}"
BASE_PORT=14550
PORTS_OK=true

for i in 0 1 2; do
    PORT=$((BASE_PORT + i * 10))
    if netstat -tuln 2>/dev/null | grep -q ":$PORT " || ss -tuln 2>/dev/null | grep -q ":$PORT "; then
        echo -e "${GREEN}✓ Port $PORT is active (drone_$i)${NC}"
    else
        echo -e "${RED}✗ Port $PORT is NOT active (drone_$i)${NC}"
        PORTS_OK=false
    fi
done

if [ "$PORTS_OK" = true ]; then
    PASSED=$((PASSED + 1))
else
    FAILED=$((FAILED + 1))
fi
echo ""

# Test 3: Check SITL instance directories
echo -e "${YELLOW}Test 3: Checking SITL instance directories...${NC}"
DIRS_OK=true

for i in 0 1 2; do
    DIR="$HOME/sitl_instances/drone_$i"
    if [ -d "$DIR" ] && [ -f "$DIR/params.parm" ]; then
        echo -e "${GREEN}✓ Instance directory exists: $DIR${NC}"
        
        # Check SYSID in params
        EXPECTED_SYSID=$((i + 1))
        if grep -q "SYSID_THISMAV $EXPECTED_SYSID" "$DIR/params.parm"; then
            echo -e "${GREEN}  ✓ Correct SYSID: $EXPECTED_SYSID${NC}"
        else
            echo -e "${RED}  ✗ Incorrect SYSID in params.parm${NC}"
            DIRS_OK=false
        fi
    else
        echo -e "${RED}✗ Instance directory missing: $DIR${NC}"
        DIRS_OK=false
    fi
done

if [ "$DIRS_OK" = true ]; then
    PASSED=$((PASSED + 1))
else
    FAILED=$((FAILED + 1))
fi
echo ""

# Test 4: Process check
echo -e "${YELLOW}Test 4: Checking ArduPilot processes...${NC}"
NUM_PROCESSES=$(pgrep -f "sim_vehicle.py" | wc -l)

if [ "$NUM_PROCESSES" -ge "$NUM_EXPECTED" ]; then
    echo -e "${GREEN}✓ PASS: Found $NUM_PROCESSES ArduPilot processes${NC}"
    PASSED=$((PASSED + 1))
else
    echo -e "${RED}✗ FAIL: Expected at least $NUM_EXPECTED processes, found $NUM_PROCESSES${NC}"
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
    echo -e "${GREEN}✓ All checkpoint 1.1 tests passed!${NC}"
    echo ""
    echo -e "${YELLOW}Manual validation steps:${NC}"
    echo "1. Connect QGroundControl to UDP port 14550 (drone_0)"
    echo "2. Check SYSID shows as 1"
    echo "3. Connect to other drones on ports 14560, 14570"
    echo "4. Attach to screen: screen -r drone_0_sitl"
    echo "5. Run: param show SYSID_THISMAV"
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please check the output above.${NC}"
    exit 1
fi
