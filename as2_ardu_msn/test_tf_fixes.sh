#!/bin/bash

# TF Issues Verification Script
# This script tests the implemented fixes

echo "========================================="
echo "TF Transform Issues - Verification Test"
echo "========================================="

# Source the workspace
cd ~/aerostack2_ws
source install/setup.bash

echo "1. Checking package build status..."
if [ -f "install/as2_ardu_msn/share/as2_ardu_msn/config/mavros_timesync.yaml" ]; then
    echo "✅ MAVROS timesync config installed"
else
    echo "❌ MAVROS timesync config missing"
fi

if [ -f "install/as2_ardu_msn/share/as2_ardu_msn/scripts/tf_diagnostics.py" ]; then
    echo "✅ TF diagnostics script installed"
else
    echo "❌ TF diagnostics script missing"
fi

if [ -f "install/as2_ardu_msn/share/as2_ardu_msn/launch/basic_aerostack2_improved.launch.py" ]; then
    echo "✅ Improved launch file installed"
else
    echo "❌ Improved launch file missing"
fi

echo ""
echo "2. Testing launch file syntax..."
ros2 launch as2_ardu_msn basic_aerostack2_improved.launch.py --check-args 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ Launch file syntax valid"
else
    echo "❌ Launch file has syntax errors"
fi

echo ""
echo "3. Testing TF diagnostics tool..."
timeout 3 ros2 run as2_ardu_msn tf_diagnostics.py > /dev/null 2>&1 &
DIAG_PID=$!
sleep 2
if ps -p $DIAG_PID > /dev/null; then
    echo "✅ TF diagnostics tool starts successfully"
    kill $DIAG_PID 2>/dev/null
else
    echo "❌ TF diagnostics tool failed to start"
fi

echo ""
echo "4. Configuration file validation..."
echo "MAVROS timesync config:"
head -10 install/as2_ardu_msn/share/as2_ardu_msn/config/mavros_timesync.yaml

echo ""
echo "========================================="
echo "Verification Complete!"
echo ""
echo "To test the full system:"
echo "1. ros2 launch as2_ardu_msn basic_aerostack2_improved.launch.py enable_tf_diagnostics:=true"
echo "2. Monitor the output for reduced TF warnings"
echo "3. Check MAVROS timesync RTT values are < 1000ms"
echo "========================================="
