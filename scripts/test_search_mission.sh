#!/bin/bash
# Search mission test - Expanding square pattern
# Perfect for search & rescue operations

MISSION_ID="search_$(date +%s)"

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
echo "SEARCH MISSION - Expanding Square"
echo "=========================================="

ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"search\",
    \"priority\": 100,
    \"timeout\": 600.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"pattern\": \"expanding_square\",
      \"center\": {\"x\": 0.0, \"y\": 0.0},
      \"altitude\": 40.0,
      \"radius\": 80.0,
      \"spacing\": 25.0,
      \"velocity\": 3.5,
      \"sensor_fov\": 60.0
    }
  }'}"

echo ""
echo "=========================================="
echo "Mission ${MISSION_ID} sent!"
echo "=========================================="
echo "Search mission profile:"
echo "  - Pattern: Expanding Square"
echo "  - Center: (0, 0)"
echo "  - Altitude: 40m AGL"
echo "  - Maximum radius: 80m"
echo "  - Track spacing: 25m"
echo "  - Velocity: 3.5 m/s"
echo "  - Sensor FOV: 60°"
echo "  - Expected duration: ~5-6 minutes"
echo ""
echo "Search pattern:"
echo "  📍 Start at center"
echo "  ⬜ Fly expanding squares"
echo "  🔍 Complete area coverage"
echo "  📡 Optimized for sensor overlap"
echo ""
echo "Flight sequence:"
echo "  1. Square 1: 25m × 25m"
echo "  2. Square 2: 50m × 50m"
echo "  3. Square 3: 75m × 75m"
echo "  4. Return to center"
echo ""
echo "Use cases:"
echo "  ✓ Search & rescue operations"
echo "  ✓ Lost object detection"
echo "  ✓ Area surveillance"
echo "  ✓ Wildlife tracking"
echo ""
echo "Monitor status: ros2 topic echo ${MONITOR_TOPIC}"
echo "=========================================="
