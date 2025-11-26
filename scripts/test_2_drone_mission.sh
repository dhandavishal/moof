#!/bin/bash
# Test 2-drone parallel mission execution

echo "=========================================="
echo "2-DRONE PARALLEL MISSION TEST"
echo "=========================================="

# Check if squadron manager is running
if ! ros2 node list 2>/dev/null | grep -q "squadron_manager"; then
    echo "ERROR: Squadron Manager not running!"
    echo "Start it with: ros2 launch squadron_manager squadron_manager.launch.py num_drones:=2"
    exit 1
fi

echo "Sending mission to Drone 0 (via Squadron Manager)..."
MISSION_ID_0="drone0_$(date +%s)"

ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID_0}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 300.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": 20.0, \"y\": 0.0, \"z\": 30.0, \"yaw\": 0.0},
        {\"x\": 20.0, \"y\": 20.0, \"z\": 30.0, \"yaw\": 1.57},
        {\"x\": 0.0, \"y\": 20.0, \"z\": 40.0, \"yaw\": 3.14},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 50.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.0,
      \"acceptance_radius\": 2.0
    }
  }'}" > /dev/null 2>&1

echo "  ✓ Mission ${MISSION_ID_0} sent (will go to drone_0)"
sleep 2

echo ""
echo "Sending mission to Drone 1 (via Squadron Manager)..."
MISSION_ID_1="drone1_$(date +%s)"

ros2 topic pub --once /squadron/mission_command std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID_1}\",
    \"task_type\": \"waypoint\",
    \"priority\": 100,
    \"timeout\": 300.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"waypoints\": [
        {\"x\": -20.0, \"y\": 0.0, \"z\": 30.0, \"yaw\": 3.14},
        {\"x\": -20.0, \"y\": -20.0, \"z\": 30.0, \"yaw\": -1.57},
        {\"x\": 0.0, \"y\": -20.0, \"z\": 40.0, \"yaw\": 0.0},
        {\"x\": 0.0, \"y\": 0.0, \"z\": 50.0, \"yaw\": 0.0}
      ],
      \"velocity\": 3.0,
      \"acceptance_radius\": 2.0
    }
  }'}" > /dev/null 2>&1

echo "  ✓ Mission ${MISSION_ID_1} sent (will go to drone_1)"

echo ""
echo "=========================================="
echo "BOTH MISSIONS SENT!"
echo "=========================================="
echo ""
echo "Flight plan:"
echo "  Drone 0: Flies EAST  (20, 0) → (20, 20) → (0, 20) → HOME"
echo "  Drone 1: Flies WEST (-20, 0) → (-20, -20) → (0, -20) → HOME"
echo ""
echo "Both drones will execute SIMULTANEOUSLY!"
echo ""
echo "Monitor Squadron: ros2 topic echo /squadron/status"
echo "Monitor Drone 0:  ros2 topic echo /drone_0/tee/mission_status"
echo "Monitor Drone 1:  ros2 topic echo /drone_1/tee/mission_status"
echo "=========================================="
