#!/bin/bash

echo "🧹 Cleaning up ROS2 processes..."

# Kill any existing ROS2 processes
pkill -f ros2
pkill -f mavros
pkill -f launch
sleep 2

echo "🔄 Restarting ROS2 daemon..."
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 2

echo "✅ Ready to launch!"
echo ""
echo "Now run:"
echo "1. Start ArduPilot SITL: sim_vehicle.py -v ArduCopter --map --console"
echo "2. Start Aerostack2: ros2 launch as2_ardu_msn basic_aerostack2.launch.py"
echo "3. Run mission: python3 basic_survey_mission.py --config config/basic_mission_config.yaml"
