#!/bin/bash
# Complex stress test mission for MOOFS
# Tests: Multiple waypoints, altitude changes, heading changes, longer duration

MISSION_ID="complex_$(date +%s)"

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

echo "=========================================="
echo "COMPLEX STRESS TEST MISSION"
echo "=========================================="

ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 600.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 30.0, \"y\": 0.0, \"z\": 25.0, \"yaw\": 0.0},
        {\"x\": 30.0, \"y\": 30.0, \"z\": 25.0, \"yaw\": 1.57},
        {\"x\": 0.0, \"y\": 30.0, \"z\": 35.0, \"yaw\": 3.14},
        {\"x\": -30.0, \"y\": 30.0, \"z\": 35.0, \"yaw\": 3.14},
        {\"x\": -30.0, \"y\": 0.0, \"z\": 45.0, \"yaw\": -1.57},
        {\"x\": -30.0, \"y\": -30.0, \"z\": 45.0, \"yaw\": -1.57},
        {\"x\": 0.0, \"y\": -30.0, \"z\": 55.0, \"yaw\": 0.0},
        {\"x\": 30.0, \"y\": -30.0, \"z\": 55.0, \"yaw\": 0.0},
        {\"x\": 30.0, \"y\": 0.0, \"z\": 40.0, \"yaw\": 1.57},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 60.0, \"yaw\": 0.0}
      ],
      \"velocity\": 4.0,
      \"acceptance_radius\": 2.5
    }
  }'}"

echo ""
echo "=========================================="
echo "Mission ${MISSION_ID} sent!"
echo "=========================================="
echo "Complex stress test profile:"
echo "  - 10 waypoints in large box spiral pattern"
echo "  - Altitude range: 25m - 60m"
echo "  - Velocity: 4.0 m/s"
echo "  - Total distance: ~360m"
echo "  - Expected duration: ~4-5 minutes"
echo ""
echo "Flight pattern (Large Spiral):"
echo "   1. (30, 0, 25m)      - East start"
echo "   2. (30, 30, 25m)     - Northeast corner"
echo "   3. (0, 30, 35m)      - North (climb 10m)"
echo "   4. (-30, 30, 35m)    - Northwest corner"
echo "   5. (-30, 0, 45m)     - West (climb 10m)"
echo "   6. (-30, -30, 45m)   - Southwest corner"
echo "   7. (0, -30, 55m)     - South (climb 10m)"
echo "   8. (30, -30, 55m)    - Southeast corner"
echo "   9. (30, 0, 40m)      - East (descend 15m)"
echo "  10. (0, 0, 60m)       - Home (climb to max)"
echo ""
echo "Tests:"
echo "  ✓ Long distance waypoints (30m segments)"
echo "  ✓ Multiple altitude changes (10m increments)"
echo "  ✓ All heading directions (0, 90, 180, 270 deg)"
echo "  ✓ Higher velocity (4.0 m/s)"
echo "  ✓ Extended mission duration (4-5 min)"
echo ""
echo "Monitor status: ros2 topic echo ${MONITOR_TOPIC}"
echo "Monitor position: ros2 topic echo /drone_0/mavros/local_position/pose"
echo "=========================================="
