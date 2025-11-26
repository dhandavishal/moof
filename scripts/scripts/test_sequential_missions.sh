#!/bin/bash
# Sequential mission test - sends multiple missions back-to-back
# Tests the 5-second cooldown period and mission queuing

echo "=========================================="
echo "SEQUENTIAL MISSION TEST"
echo "=========================================="
echo "This test sends 3 missions in rapid succession"
echo "to verify cooldown and queuing work correctly."
echo ""

# Check if squadron manager is running
if ros2 node list 2>/dev/null | grep -q "squadron_manager"; then
    echo "Squadron Manager detected ✓"
    TOPIC="/squadron/mission_command"
    MONITOR_TOPIC="/squadron/status"
else
    echo "Direct TEE mode (no Squadron Manager)"
    TOPIC="/drone_0/tee/mission_command"
    MONITOR_TOPIC="/drone_0/tee/mission_status"
fi

echo ""
echo "Sending Mission 1: Small square pattern..."
ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"seq_1_$(date +%s)\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 180.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 15.0, \"y\": 0.0, \"z\": 25.0, \"yaw\": 0.0},
        {\"x\": 15.0, \"y\": 15.0, \"z\": 25.0, \"yaw\": 1.57},
        {\"x\": 0.0, \"y\": 15.0, \"z\": 25.0, \"yaw\": 3.14},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 35.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.0,
      \"acceptance_radius\": 2.0
    }
  }'}" > /dev/null 2>&1

echo "  ✓ Mission 1 sent (15m square, 4 waypoints, ~90s)"
sleep 1

echo ""
echo "Sending Mission 2: Triangle pattern..."
ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"seq_2_$(date +%s)\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 180.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 20.0, \"y\": 0.0, \"z\": 30.0, \"yaw\": 0.0},
        {\"x\": 10.0, \"y\": 17.3, \"z\": 40.0, \"yaw\": 2.09},
        {\"x\": -10.0, \"y\": 17.3, \"z\": 40.0, \"yaw\": -2.09},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 50.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.5,
      \"acceptance_radius\": 2.0
    }
  }'}" > /dev/null 2>&1

echo "  ✓ Mission 2 sent (Triangle, 4 waypoints, ~100s)"
sleep 1

echo ""
echo "Sending Mission 3: Diamond pattern..."
ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"seq_3_$(date +%s)\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 180.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 25.0, \"y\": 0.0, \"z\": 35.0, \"yaw\": 0.0},
        {\"x\": 0.0, \"y\": 25.0, \"z\": 45.0, \"yaw\": 1.57},
        {\"x\": -25.0, \"y\": 0.0, \"z\": 35.0, \"yaw\": 3.14},
        {\"x\": 0.0, \"y\": -25.0, \"z\": 45.0, \"yaw\": -1.57},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 60.0, \"yaw\": 0.0}
      ],
      \"velocity\": 4.0,
      \"acceptance_radius\": 2.5
    }
  }'}" > /dev/null 2>&1

echo "  ✓ Mission 3 sent (Diamond, 5 waypoints, ~120s)"

echo ""
echo "=========================================="
echo "All 3 missions sent!"
echo "=========================================="
echo ""
echo "Expected behavior:"
echo "  1. Mission 1 executes immediately (~90s)"
echo "  2. ⏱️  5-second cooldown after completion"
echo "  3. Mission 2 executes (~100s)"
echo "  4. ⏱️  5-second cooldown after completion"
echo "  5. Mission 3 executes (~120s)"
echo ""
echo "Total expected time: ~5-6 minutes"
echo ""
echo "Watch for cooldown messages:"
echo "  'Mission complete - starting 5.0s cooldown before next mission'"
echo "  'Mission cooldown: waiting X.Xs before starting next mission'"
echo "  'Cooldown complete (5.Xs elapsed), starting next mission'"
echo ""
echo "Monitor: ros2 topic echo ${MONITOR_TOPIC}"
echo "=========================================="
