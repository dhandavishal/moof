#!/bin/bash
# Survey mission test - Area mapping with lawn-mower pattern
# Perfect for photogrammetry and orthophoto generation

MISSION_ID="survey_$(date +%s)"

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
echo "SURVEY MISSION - Aerial Mapping"
echo "=========================================="

ros2 topic pub --once ${TOPIC} std_msgs/msg/String \
  "{data: '{
    \"mission_id\": \"${MISSION_ID}\",
    \"task_type\": \"survey\",
    \"priority\": 100,
    \"timeout\": 600.0,
    \"multi_drone\": false,
    \"parameters\": {
      \"area\": [
        {\"x\": 0.0, \"y\": 0.0},
        {\"x\": 80.0, \"y\": 0.0},
        {\"x\": 80.0, \"y\": 60.0},
        {\"x\": 0.0, \"y\": 60.0}
      ],
      \"altitude\": 50.0,
      \"orientation\": 0.0,
      \"overlap\": 0.75,
      \"sidelap\": 0.65,
      \"velocity\": 5.0,
      \"camera\": {
        \"sensor_width\": 23.5,
        \"sensor_height\": 15.6,
        \"focal_length\": 16.0,
        \"image_width\": 6000,
        \"image_height\": 4000
      }
    }
  }'}"

echo ""
echo "=========================================="
echo "Mission ${MISSION_ID} sent!"
echo "=========================================="
echo "Survey mission profile:"
echo "  - Area: 80m × 60m (4800 m²)"
echo "  - Altitude: 50m AGL"
echo "  - Pattern: Lawn-mower (horizontal)"
echo "  - Forward overlap: 75%"
echo "  - Side overlap: 65%"
echo "  - Velocity: 5.0 m/s"
echo "  - Expected duration: ~8-10 minutes"
echo ""
echo "Coverage pattern:"
echo "  📐 Rectangular survey area"
echo "  ↔️  Parallel flight lines"
echo "  🔄 Alternating direction (boustrophedon)"
echo "  📸 Optimized for photogrammetry"
echo ""
echo "Use cases:"
echo "  ✓ Orthophoto generation"
echo "  ✓ 3D reconstruction"
echo "  ✓ Agricultural surveys"
echo "  ✓ Infrastructure inspection"
echo ""
echo "Monitor status: ros2 topic echo ${MONITOR_TOPIC}"
echo "=========================================="
