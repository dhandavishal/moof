#!/bin/bash
# Test mission script for MOOFS
# Sends mission to Squadron Manager (if running) or directly to TEE

MISSION_ID="test_$(date +%s)"

# Check if squadron manager is running
if ros2 node list 2>/dev/null | grep -q "squadron_manager"; then
    echo "Squadron Manager detected - sending mission via Squadron Manager"
    TOPIC="/squadron/mission_command"
    MONITOR_TOPIC="/squadron/status"
else
    echo "No Squadron Manager - sending mission directly to drone TEE"
    TOPIC="/drone_0/tee/mission_command"
    MONITOR_TOPIC="/drone_0/tee/mission_status"
fi

ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 300.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 20.0, \"y\": 0.0, \"z\": 30.0, \"yaw\": 0.0},
        {\"x\": 20.0, \"y\": 20.0, \"z\": 30.0, \"yaw\": 1.57},
        {\"x\": 0.0, \"y\": 20.0, \"z\": 40.0, \"yaw\": 3.14},
        {\"x\": -20.0, \"y\": 20.0, \"z\": 40.0, \"yaw\": -1.57},
        {\"x\": -20.0, \"y\": 0.0, \"z\": 30.0, \"yaw\": 0.0},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 50.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.0,
      \"acceptance_radius\": 2.0
    }
  }'}"

echo ""
echo "=========================================="
echo "Mission ${MISSION_ID} sent!"
echo "=========================================="
echo "Complex mission profile:"
echo "  - 6 waypoints in a box pattern"
echo "  - Altitude range: 30m - 50m"
echo "  - Velocity: 3.0 m/s"
echo "  - Total distance: ~120m"
echo "  - Expected duration: ~2-3 minutes"
echo ""
echo "Flight pattern:"
echo "  1. (20, 0, 30m)   - East"
echo "  2. (20, 20, 30m)  - Northeast"
echo "  3. (0, 20, 40m)   - North (climb)"
echo "  4. (-20, 20, 40m) - Northwest"
echo "  5. (-20, 0, 30m)  - West (descend)"
echo "  6. (0, 0, 50m)    - Home (climb high)"
echo ""
echo "Monitor status: ros2 topic echo ${MONITOR_TOPIC}"
echo "=========================================="
