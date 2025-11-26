#!/bin/bash
# Test validation race condition fix - single drone

echo "=========================================="
echo "VALIDATION FIX TEST (Single Drone)"
echo "=========================================="

echo "Sending test waypoint mission..."
MISSION_ID="validation_test_$(date +%s)"

ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 300.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 10.0, \"y\": 0.0, \"z\": 20.0, \"yaw\": 0.0},
        {\"x\": 10.0, \"y\": 10.0, \"z\": 20.0, \"yaw\": 1.57}
      ],
      \"velocity\": 3.0,
      \"acceptance_radius\": 2.0
    }
  }'}"

echo ""
echo "✓ Mission sent: ${MISSION_ID}"
echo ""
echo "Watch for:"
echo "  - Task should NOT abort with 'No tasks available'"
echo "  - Task should validate successfully"
echo "  - Task should transition to EXECUTING state"
echo ""
echo "Monitor with: ros2 topic echo /drone_0/tee/mission_status"
