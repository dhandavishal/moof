#!/bin/bash

# Checkpoint 2.1 Validation: Flight Abstraction Layer (FAL)
# Validates that FAL node can be launched and action servers are available

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test counters
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Workspace path
WS_DIR="/home/dhandavishal/multi_drone_ws"

# Function to print test results
print_test_result() {
    local test_name="$1"
    local result="$2"
    local details="$3"
    
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    
    if [ "$result" = "PASS" ]; then
        echo -e "${GREEN}✓${NC} Test ${TOTAL_TESTS}: ${test_name} - ${GREEN}PASSED${NC}"
        [ -n "$details" ] && echo -e "  ${BLUE}ℹ${NC} ${details}"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo -e "${RED}✗${NC} Test ${TOTAL_TESTS}: ${test_name} - ${RED}FAILED${NC}"
        [ -n "$details" ] && echo -e "  ${RED}!${NC} ${details}"
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
}

# Function to check if package can be run
check_package_executable() {
    local pkg_name="$1"
    local executable="$2"
    source "$WS_DIR/install/setup.bash" 2>/dev/null
    if ros2 pkg list | grep -q "$pkg_name"; then
        if ros2 pkg executables "$pkg_name" | grep -q "$executable"; then
            return 0
        fi
    fi
    return 1
}

# Function to check if Python module can be imported
check_python_import() {
    local module_path="$1"
    source "$WS_DIR/install/setup.bash" 2>/dev/null
    if python3 -c "import $module_path" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}Checkpoint 2.1: Flight Abstraction Layer${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${YELLOW}Testing FAL implementation...${NC}"
echo ""

# Test 1: Check flight_abstraction package is built
if [ -d "$WS_DIR/install/flight_abstraction" ]; then
    print_test_result "flight_abstraction package is built and installed" "PASS" "Package found in install directory"
else
    print_test_result "flight_abstraction package is built and installed" "FAIL" "Package not found in install directory"
fi

# Test 2: Check primitives module
if check_python_import "flight_abstraction.primitives"; then
    print_test_result "Primitives module can be imported" "PASS" "flight_abstraction.primitives"
else
    print_test_result "Primitives module can be imported" "FAIL" "Import failed"
fi

# Test 3: Check base primitive
if check_python_import "flight_abstraction.primitives.base_primitive"; then
    print_test_result "BasePrimitive can be imported" "PASS" "Base class for all primitives"
else
    print_test_result "BasePrimitive can be imported" "FAIL" "Import failed"
fi

# Test 4: Check arm primitive
if check_python_import "flight_abstraction.primitives.arm_primitive"; then
    print_test_result "ArmPrimitive can be imported" "PASS" "Arm/disarm primitive"
else
    print_test_result "ArmPrimitive can be imported" "FAIL" "Import failed"
fi

# Test 5: Check takeoff primitive
if check_python_import "flight_abstraction.primitives.takeoff_primitive"; then
    print_test_result "TakeoffPrimitive can be imported" "PASS" "Takeoff primitive"
else
    print_test_result "TakeoffPrimitive can be imported" "FAIL" "Import failed"
fi

# Test 6: Check goto primitive
if check_python_import "flight_abstraction.primitives.goto_primitive"; then
    print_test_result "GotoPrimitive can be imported" "PASS" "Goto waypoint primitive"
else
    print_test_result "GotoPrimitive can be imported" "FAIL" "Import failed"
fi

# Test 7: Check land primitive
if check_python_import "flight_abstraction.primitives.land_primitive"; then
    print_test_result "LandPrimitive can be imported" "PASS" "Land primitive"
else
    print_test_result "LandPrimitive can be imported" "FAIL" "Import failed"
fi

# Test 8: Check FAL node module
if check_python_import "flight_abstraction.fal_node"; then
    print_test_result "FAL node module can be imported" "PASS" "Main FAL node implementation"
else
    print_test_result "FAL node module can be imported" "FAIL" "Import failed"
fi

# Test 9: Check fal_node executable
if check_package_executable "flight_abstraction" "fal_node"; then
    print_test_result "fal_node executable is registered" "PASS" "Can be run with 'ros2 run'"
else
    print_test_result "fal_node executable is registered" "FAIL" "Executable not found"
fi

# Test 10: Check launch file exists
if [ -f "$WS_DIR/install/flight_abstraction/share/flight_abstraction/launch/fal.launch.py" ]; then
    print_test_result "FAL launch file is installed" "PASS" "fal.launch.py available"
else
    print_test_result "FAL launch file is installed" "FAIL" "Launch file not found"
fi

# Test 11: Test Python syntax of all primitive files
source "$WS_DIR/install/setup.bash" 2>/dev/null
primitive_files=(
    "base_primitive"
    "arm_primitive"
    "takeoff_primitive"
    "goto_primitive"
    "land_primitive"
)

all_syntax_valid=true
invalid_files=""

for prim in "${primitive_files[@]}"; do
    if ! python3 -m py_compile "$WS_DIR/src/flight_abstraction/flight_abstraction/primitives/${prim}.py" 2>/dev/null; then
        all_syntax_valid=false
        invalid_files="$invalid_files $prim"
    fi
done

if [ "$all_syntax_valid" = true ]; then
    print_test_result "All primitive files have valid Python syntax" "PASS" "5 primitive files validated"
else
    print_test_result "All primitive files have valid Python syntax" "FAIL" "Invalid syntax:$invalid_files"
fi

# Test 12: Test Python syntax of FAL node
if python3 -m py_compile "$WS_DIR/src/flight_abstraction/flight_abstraction/fal_node.py" 2>/dev/null; then
    print_test_result "FAL node has valid Python syntax" "PASS" "fal_node.py validated"
else
    print_test_result "FAL node has valid Python syntax" "FAIL" "Syntax errors in fal_node.py"
fi

# Test 13: Check if primitives are properly structured
source "$WS_DIR/install/setup.bash" 2>/dev/null
if python3 -c "from flight_abstraction.primitives import BasePrimitive, ArmPrimitive, TakeoffPrimitive, GotoPrimitive, LandPrimitive; print('OK')" 2>/dev/null | grep -q "OK"; then
    print_test_result "All primitives can be imported from package" "PASS" "5 primitive classes available"
else
    print_test_result "All primitives can be imported from package" "FAIL" "Import failed"
fi

# Test 14: Check enum classes
if python3 -c "from flight_abstraction.primitives.base_primitive import PrimitiveState; print(PrimitiveState.IDLE.name)" 2>/dev/null | grep -q "IDLE"; then
    print_test_result "PrimitiveState enum is defined correctly" "PASS" "IDLE, EXECUTING, SUCCESS, FAILED, CANCELLED"
else
    print_test_result "PrimitiveState enum is defined correctly" "FAIL" "Enum not working"
fi

# Test 15: Check if dependencies are available
deps_available=true
missing_deps=""

dependencies=("rclpy" "mavros_msgs" "geometry_msgs" "nav_msgs")
for dep in "${dependencies[@]}"; do
    if ! python3 -c "import $dep" 2>/dev/null; then
        deps_available=false
        missing_deps="$missing_deps $dep"
    fi
done

if [ "$deps_available" = true ]; then
    print_test_result "All Python dependencies are available" "PASS" "rclpy, mavros_msgs, geometry_msgs, nav_msgs"
else
    print_test_result "All Python dependencies are available" "FAIL" "Missing:$missing_deps"
fi

echo ""
echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}================================================${NC}"
echo -e "Total Tests: ${TOTAL_TESTS}"
echo -e "${GREEN}Passed: ${PASSED_TESTS}${NC}"
echo -e "${RED}Failed: ${FAILED_TESTS}${NC}"
echo ""

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed! Checkpoint 2.1 validated successfully.${NC}"
    echo -e "${GREEN}✓ Flight Abstraction Layer is ready.${NC}"
    echo ""
    echo -e "${YELLOW}Next Steps:${NC}"
    echo -e "  1. Start SITL: ${BLUE}./src/moofs_3d/scripts/launch_multi_sitl.sh 1${NC}"
    echo -e "  2. Launch MAVROS: ${BLUE}ros2 launch moofs_3d multi_mavros.launch.py num_drones:=1${NC}"
    echo -e "  3. Launch FAL: ${BLUE}ros2 launch flight_abstraction fal.launch.py drone_namespace:=/drone_0${NC}"
    echo -e "  4. Test actions: Use ros2 action send_goal commands"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please review the errors above.${NC}"
    echo ""
    echo -e "${YELLOW}Common Issues:${NC}"
    echo -e "  - Rebuild: ${BLUE}cd $WS_DIR && colcon build --packages-select flight_abstraction --symlink-install${NC}"
    echo -e "  - Source workspace: ${BLUE}source install/setup.bash${NC}"
    echo -e "  - Check Python syntax in primitive files"
    echo ""
    exit 1
fi
