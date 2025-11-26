#!/bin/bash

# Squadron Manager Debug Launcher
# This script launches Squadron Manager with comprehensive debug logging

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "╔════════════════════════════════════════════════════════════════════════╗"
echo "║          Squadron Manager - DEBUG Mode Launcher                       ║"
echo "╚════════════════════════════════════════════════════════════════════════╝"
echo ""

# Source workspace
echo "🔧 Sourcing workspace..."
cd "$WS_DIR"
source install/setup.bash

echo ""
echo "📋 Pre-flight Checks:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if MAVROS is running
if ros2 node list | grep -q "mavros"; then
    echo "✅ MAVROS node detected"
else
    echo "❌ MAVROS node NOT found"
    echo "   Please start complete_system.launch.py first!"
    exit 1
fi

# Check MAVROS state topic
if ros2 topic list | grep -q "/drone_0/mavros/state"; then
    echo "✅ MAVROS state topic exists"
    
    # Try to get state
    echo "   Checking MAVROS connection status..."
    if timeout 2 ros2 topic echo /drone_0/mavros/state --once 2>/dev/null | grep -q "connected: true"; then
        echo "✅ MAVROS connected to ArduPilot"
    else
        echo "⚠️  MAVROS not yet connected (this is OK, will connect soon)"
    fi
else
    echo "❌ MAVROS state topic NOT found"
    exit 1
fi

echo ""
echo "🚀 Launching Squadron Manager with DEBUG logging..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📊 What to look for in the logs:"
echo "   1. [DEBUG] State callback for drone_0: connected=..."
echo "   2. [INFO] Drone drone_0 updated: connected=False->True"
echo "   3. [INFO] Drone drone_0 updated: state=UNKNOWN->AVAILABLE"
echo ""
echo "⏳ Wait 10 seconds after launch for state updates to arrive"
echo "   Then send a mission: ./scripts/send_squadron_mission.sh 10 10 15"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch with DEBUG logging
ros2 launch squadron_manager squadron_manager.launch.py \
    num_drones:=1 \
    --ros-args --log-level squadron_manager:=DEBUG
