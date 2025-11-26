#!/bin/bash
# Search mission test - Spiral pattern
# Efficient outward spiral search

MISSION_ID="spiral_$(date +%s)"

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
echo "SEARCH MISSION - Spiral Pattern"
echo "=========================================="

ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"search\",
    \"priority\": 100,
    \"timeout\": 600.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"pattern\": \"spiral\",
      \"center\": {\"x\": 0.0, \"y\": 0.0},
      \"altitude\": 45.0,
      \"radius\": 70.0,
      \"spacing\": 20.0,
      \"velocity\": 4.0,
      \"sensor_fov\": 70.0
    }
  }'}"

echo ""
echo "=========================================="
echo "Mission ${MISSION_ID} sent!"
echo "=========================================="
echo "Spiral search mission profile:"
echo "  - Pattern: Outward Spiral"
echo "  - Center: (0, 0)"
echo "  - Altitude: 45m AGL"
echo "  - Maximum radius: 70m"
echo "  - Spiral spacing: 20m"
echo "  - Velocity: 4.0 m/s"
echo "  - Sensor FOV: 70°"
echo "  - Expected duration: ~4-5 minutes"
echo ""
echo "Search pattern:"
echo "  📍 Start at center"
echo "  🌀 Fly outward spiral"
echo "  🔄 Smooth continuous path"
echo "  📡 Continuous sensor coverage"
echo ""
echo "Advantages:"
echo "  ✓ Smooth flight path (no sharp turns)"
echo "  ✓ Efficient for circular areas"
echo "  ✓ Good for thermal imaging"
echo "  ✓ Natural search progression"
echo ""
echo "Use cases:"
echo "  ✓ Person/vehicle detection"
echo "  ✓ Debris field analysis"
echo "  ✓ Contamination mapping"
echo "  ✓ Emergency response"
echo ""
echo "Monitor status: ros2 topic echo ${MONITOR_TOPIC}"
echo "=========================================="
