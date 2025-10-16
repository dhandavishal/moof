#!/bin/bash
# Kill all multi-drone SITL instances

echo "Killing all SITL instances..."

# Kill all screen sessions matching drone_*_sitl
for session in $(screen -ls | grep "drone_.*_sitl" | awk '{print $1}'); do
    echo "Killing session: $session"
    screen -S "$session" -X quit
done

# Kill any remaining ArduPilot processes
pkill -f "sim_vehicle.py" || true
pkill -f "arducopter" || true

echo "All SITL instances stopped"
