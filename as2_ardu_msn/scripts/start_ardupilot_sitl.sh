#!/bin/bash

# Start ArduPilot SITL with MAVROS-compatible configuration
# This script starts ArduPilot SITL with the correct MAVProxy configuration for MAVROS

# Check if ArduPilot is installed
if [ ! -d "$HOME/ardupilot/ardupilot" ]; then
    echo "Error: ArduPilot not found in $HOME/ardupilot"
    echo "Please install ArduPilot first"
    exit 1
fi

# Kill any existing ArduPilot processes
echo "Stopping any existing ArduPilot processes..."
pkill -f arducopter
pkill -f mavproxy
sleep 2

# Change to ArduPilot directory
cd $HOME/ardupilot/ardupilot

# Start ArduPilot SITL with MAVProxy
echo "Starting ArduPilot SITL..."
echo "MAVROS will connect to udp://:14555@127.0.0.1:14550"

# Start SITL with the correct configuration for MAVROS
sim_vehicle.py -v ArduCopter \
    --map \
    --console \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14555 \
    -I0

echo "ArduPilot SITL stopped"
