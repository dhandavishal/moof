#!/bin/bash

# Checkpoint 1.3 Validation: Project Structure Setup
# Validates that all required ROS2 packages are created and built successfully

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

# Function to check if package exists in source
check_package_source() {
    local pkg_name="$1"
    if [ -d "$WS_DIR/src/$pkg_name" ]; then
        return 0
    else
        return 1
    fi
}

# Function to check if package is built
check_package_built() {
    local pkg_name="$1"
    if [ -d "$WS_DIR/install/$pkg_name" ]; then
        return 0
    else
        return 1
    fi
}

# Function to check if Python package can be imported
check_python_import() {
    local pkg_name="$1"
    source "$WS_DIR/install/setup.bash" 2>/dev/null
    if python3 -c "import $pkg_name" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

# Function to check message/service/action interfaces
check_interfaces() {
    local interface_count
    source "$WS_DIR/install/setup.bash" 2>/dev/null
    interface_count=$(ros2 interface list 2>/dev/null | grep -c "multi_drone_msgs/" || echo "0")
    if [ "$interface_count" -ge 8 ]; then
        echo "$interface_count"
        return 0
    else
        echo "$interface_count"
        return 1
    fi
}

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}Checkpoint 1.3: Project Structure Setup${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo -e "${YELLOW}Testing ROS2 workspace and package structure...${NC}"
echo ""

# Test 1: Check moofs_3d package
if check_package_source "moofs_3d" && check_package_built "moofs_3d"; then
    print_test_result "moofs_3d package exists and is built" "PASS" "Infrastructure package for multi-drone SITL"
else
    print_test_result "moofs_3d package exists and is built" "FAIL" "Package missing or not built"
fi

# Test 2: Check multi_drone_msgs package
if check_package_source "multi_drone_msgs" && check_package_built "multi_drone_msgs"; then
    print_test_result "multi_drone_msgs package exists and is built" "PASS" "Custom messages, services, and actions"
else
    print_test_result "multi_drone_msgs package exists and is built" "FAIL" "Package missing or not built"
fi

# Test 3: Check flight_abstraction package
if check_package_source "flight_abstraction" && check_package_built "flight_abstraction"; then
    if check_python_import "flight_abstraction"; then
        print_test_result "flight_abstraction package exists, built, and importable" "PASS" "Flight Abstraction Layer package"
    else
        print_test_result "flight_abstraction package exists, built, and importable" "FAIL" "Package built but not importable"
    fi
else
    print_test_result "flight_abstraction package exists, built, and importable" "FAIL" "Package missing or not built"
fi

# Test 4: Check task_execution package
if check_package_source "task_execution" && check_package_built "task_execution"; then
    if check_python_import "task_execution"; then
        print_test_result "task_execution package exists, built, and importable" "PASS" "Task execution and state machine"
    else
        print_test_result "task_execution package exists, built, and importable" "FAIL" "Package built but not importable"
    fi
else
    print_test_result "task_execution package exists, built, and importable" "FAIL" "Package missing or not built"
fi

# Test 5: Check squadron_manager package
if check_package_source "squadron_manager" && check_package_built "squadron_manager"; then
    if check_python_import "squadron_manager"; then
        print_test_result "squadron_manager package exists, built, and importable" "PASS" "Squadron coordination and management"
    else
        print_test_result "squadron_manager package exists, built, and importable" "FAIL" "Package built but not importable"
    fi
else
    print_test_result "squadron_manager package exists, built, and importable" "FAIL" "Package missing or not built"
fi

# Test 6: Check mission_planner package
if check_package_source "mission_planner" && check_package_built "mission_planner"; then
    if check_python_import "mission_planner"; then
        print_test_result "mission_planner package exists, built, and importable" "PASS" "Mission planning and FDL parsing"
    else
        print_test_result "mission_planner package exists, built, and importable" "FAIL" "Package built but not importable"
    fi
else
    print_test_result "mission_planner package exists, built, and importable" "FAIL" "Package missing or not built"
fi

# Test 7: Check multi_drone_msgs interfaces
interface_result=$(check_interfaces)
interface_count=$?
if [ $interface_count -eq 0 ]; then
    print_test_result "multi_drone_msgs interfaces are generated" "PASS" "Found $interface_result interfaces (expected >= 8)"
else
    print_test_result "multi_drone_msgs interfaces are generated" "FAIL" "Found only $interface_result interfaces (expected >= 8)"
fi

# Test 8: Check specific message types
source "$WS_DIR/install/setup.bash" 2>/dev/null
msg_types=("DroneStatus" "Waypoint" "Mission")
all_msgs_found=true
missing_msgs=""

for msg_type in "${msg_types[@]}"; do
    if ! ros2 interface show multi_drone_msgs/msg/$msg_type &>/dev/null; then
        all_msgs_found=false
        missing_msgs="$missing_msgs $msg_type"
    fi
done

if [ "$all_msgs_found" = true ]; then
    print_test_result "Required message types are available" "PASS" "DroneStatus, Waypoint, Mission"
else
    print_test_result "Required message types are available" "FAIL" "Missing:$missing_msgs"
fi

# Test 9: Check service types
if ros2 interface show multi_drone_msgs/srv/ArmDisarm &>/dev/null; then
    print_test_result "ArmDisarm service is available" "PASS" "Service for arming/disarming drones"
else
    print_test_result "ArmDisarm service is available" "FAIL" "Service not found"
fi

# Test 10: Check action types
action_types=("Takeoff" "Land" "GoToWaypoint" "ExecutePrimitive")
all_actions_found=true
missing_actions=""

for action_type in "${action_types[@]}"; do
    if ! ros2 interface show multi_drone_msgs/action/$action_type &>/dev/null; then
        all_actions_found=false
        missing_actions="$missing_actions $action_type"
    fi
done

if [ "$all_actions_found" = true ]; then
    print_test_result "Required action types are available" "PASS" "Takeoff, Land, GoToWaypoint, ExecutePrimitive"
else
    print_test_result "Required action types are available" "FAIL" "Missing:$missing_actions"
fi

# Test 11: Check package.xml files exist
all_package_xml_exist=true
packages=("moofs_3d" "multi_drone_msgs" "flight_abstraction" "task_execution" "squadron_manager" "mission_planner")
missing_xml=""

for pkg in "${packages[@]}"; do
    if [ ! -f "$WS_DIR/src/$pkg/package.xml" ]; then
        all_package_xml_exist=false
        missing_xml="$missing_xml $pkg"
    fi
done

if [ "$all_package_xml_exist" = true ]; then
    print_test_result "All packages have package.xml files" "PASS" "6 packages validated"
else
    print_test_result "All packages have package.xml files" "FAIL" "Missing package.xml:$missing_xml"
fi

# Test 12: Check Python setup.py files for ament_python packages
py_packages=("moofs_3d" "flight_abstraction" "task_execution" "squadron_manager" "mission_planner")
all_setup_py_exist=true
missing_setup=""

for pkg in "${py_packages[@]}"; do
    if [ ! -f "$WS_DIR/src/$pkg/setup.py" ]; then
        all_setup_py_exist=false
        missing_setup="$missing_setup $pkg"
    fi
done

if [ "$all_setup_py_exist" = true ]; then
    print_test_result "All Python packages have setup.py files" "PASS" "5 Python packages validated"
else
    print_test_result "All Python packages have setup.py files" "FAIL" "Missing setup.py:$missing_setup"
fi

# Test 13: Check CMakeLists.txt for multi_drone_msgs
if [ -f "$WS_DIR/src/multi_drone_msgs/CMakeLists.txt" ]; then
    if grep -q "rosidl_generate_interfaces" "$WS_DIR/src/multi_drone_msgs/CMakeLists.txt"; then
        print_test_result "multi_drone_msgs CMakeLists.txt has interface generation" "PASS" "rosidl_generate_interfaces configured"
    else
        print_test_result "multi_drone_msgs CMakeLists.txt has interface generation" "FAIL" "rosidl_generate_interfaces not found"
    fi
else
    print_test_result "multi_drone_msgs CMakeLists.txt has interface generation" "FAIL" "CMakeLists.txt not found"
fi

# Test 14: Check workspace build directory
if [ -d "$WS_DIR/build" ] && [ "$(ls -A $WS_DIR/build)" ]; then
    build_count=$(ls -1 "$WS_DIR/build" | wc -l)
    print_test_result "Workspace build directory exists and populated" "PASS" "Found $build_count package builds"
else
    print_test_result "Workspace build directory exists and populated" "FAIL" "Build directory empty or missing"
fi

# Test 15: Check workspace install directory
if [ -d "$WS_DIR/install" ] && [ "$(ls -A $WS_DIR/install)" ]; then
    install_count=$(ls -1d "$WS_DIR/install"/*/ 2>/dev/null | wc -l)
    print_test_result "Workspace install directory exists and populated" "PASS" "Found $install_count installed packages"
else
    print_test_result "Workspace install directory exists and populated" "FAIL" "Install directory empty or missing"
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
    echo -e "${GREEN}✓ All tests passed! Checkpoint 1.3 validated successfully.${NC}"
    echo -e "${GREEN}✓ Project structure setup is complete.${NC}"
    echo ""
    echo -e "${YELLOW}Next Steps:${NC}"
    echo -e "  1. Source the workspace: ${BLUE}source install/setup.bash${NC}"
    echo -e "  2. Start implementing Phase 2: Flight Abstraction Layer"
    echo -e "  3. Create primitive classes in flight_abstraction package"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please review the errors above.${NC}"
    echo ""
    echo -e "${YELLOW}Common Issues:${NC}"
    echo -e "  - Run: ${BLUE}cd $WS_DIR && colcon build --symlink-install${NC}"
    echo -e "  - Source workspace: ${BLUE}source install/setup.bash${NC}"
    echo -e "  - Check package dependencies in package.xml files"
    echo ""
    exit 1
fi
