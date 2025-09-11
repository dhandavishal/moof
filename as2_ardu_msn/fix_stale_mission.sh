#!/bin/bash

# Quick fix for "Mission is stale" issue in ArduPilot SITL
echo "🔧 Fixing 'Mission is stale' issue..."

# Check if MAVROS is running
if ! pgrep -f "mavros" > /dev/null; then
    echo "❌ MAVROS not running. Please start your launch file first."
    exit 1
fi

echo "✓ MAVROS is running"

# Run the mission cleaner
echo "🧹 Clearing stale mission from ArduPilot..."
cd /home/dhandavishal/aerostack2_ws/src/as2_ardu_msn
python3 scripts/clear_mission.py

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Mission cleanup complete!"
    echo "   The 'Mission is stale' issue should now be resolved."
    echo "   You can now run your survey mission."
else
    echo ""
    echo "❌ Mission cleanup failed!"
    echo "   Please check that:"
    echo "   - MAVROS is connected to ArduPilot SITL"
    echo "   - ArduPilot SITL is running"
    echo "   - Network connection is working"
fi
