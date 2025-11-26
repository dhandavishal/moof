#!/bin/bash
# Test mission script for MOOFS

MISSION_ID="test_$(date +%s)"

ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 120.0,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 10.0, \"y\": 10.0, \"z\": 15.0, \"yaw\": 0.0}
      ],
      \"velocity\": 2.0,
      \"acceptance_radius\": 1.0
    }
  }'}"

echo ""
echo "Mission ${MISSION_ID} sent!"
echo "Monitor status with: ros2 topic echo /tee/mission_status"
