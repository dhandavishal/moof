# TEE Testing and Usage Guide

## Quick Start

### 1. Verify Installation

```bash
source install/setup.bash
python3 src/task_execution/scripts/verify_installation.py
```

Expected output: "SUCCESS: All components verified!"

### 2. Launch TEE

```bash
# Option A: TEE alone (for testing)
ros2 launch task_execution tee.launch.py

# Option B: TEE with FAL (full system)
ros2 launch task_execution tee_with_fal.launch.py
```

### 3. Monitor Status

In a new terminal:

```bash
source install/setup.bash
ros2 run task_execution monitor_tee_status
```

### 4. Send Test Missions

In a new terminal:

```bash
source install/setup.bash
ros2 run task_execution test_mission_execution
```

## Testing Scenarios

### Scenario 1: Simple Waypoint Mission

```bash
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{
    \"mission_id\": \"wp_test_001\",
    \"task_type\": \"waypoint\",
    \"priority\": 50,
    \"timeout\": 300.0,
    \"parameters\": {
      \"waypoints\": [
        {\"position\": [0.0, 0.0, 50.0]},
        {\"position\": [100.0, 0.0, 50.0]},
        {\"position\": [100.0, 100.0, 50.0]},
        {\"position\": [0.0, 0.0, 50.0]}
      ],
      \"velocity\": 10.0
    }
  }"' --once
```

### Scenario 2: Survey Mission

```bash
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{
    \"mission_id\": \"survey_test_001\",
    \"task_type\": \"survey\",
    \"priority\": 50,
    \"timeout\": 600.0,
    \"parameters\": {
      \"area\": [
        [0.0, 0.0],
        [200.0, 0.0],
        [200.0, 200.0],
        [0.0, 200.0]
      ],
      \"altitude\": 50.0,
      \"overlap_forward\": 0.75,
      \"overlap_side\": 0.65,
      \"gsd\": 0.02
    }
  }"' --once
```

### Scenario 3: Search Mission

```bash
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{
    \"mission_id\": \"search_test_001\",
    \"task_type\": \"search\",
    \"priority\": 50,
    \"timeout\": 600.0,
    \"parameters\": {
      \"center\": [150.0, 150.0],
      \"radius\": 50.0,
      \"pattern\": \"expanding_square\",
      \"altitude\": 30.0,
      \"search_speed\": 5.0
    }
  }"' --once
```

### Scenario 4: Priority Queue Test

Send multiple missions with different priorities:

```bash
# Low priority mission
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{\"mission_id\": \"low_priority\", \"task_type\": \"waypoint\", \"priority\": 100, \"timeout\": 300.0, \"parameters\": {\"waypoints\": [{\"position\": [50.0, 0.0, 50.0]}], \"velocity\": 5.0}}"' --once

# High priority mission (should execute first)
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{\"mission_id\": \"high_priority\", \"task_type\": \"waypoint\", \"priority\": 20, \"timeout\": 300.0, \"parameters\": {\"waypoints\": [{\"position\": [100.0, 0.0, 50.0]}], \"velocity\": 10.0}}"' --once
```

### Scenario 5: Pause/Resume

```bash
# Start a long mission
ros2 run task_execution test_mission_execution

# Pause execution
ros2 topic pub /squadron/pause std_msgs/msg/Empty --once

# Resume execution
ros2 topic pub /squadron/resume std_msgs/msg/Empty --once
```

### Scenario 6: Abort

```bash
# Abort all missions
ros2 topic pub /squadron/abort std_msgs/msg/Empty --once
```

## Debugging

### View All Active Topics

```bash
ros2 topic list
```

Expected TEE topics:
- `/squadron/mission_command`
- `/squadron/pause`
- `/squadron/resume`
- `/squadron/abort`
- `/tee/mission_status`
- `/tee/primitive_command`
- `/fal/primitive_status`

### Monitor Mission Status

```bash
ros2 topic echo /tee/mission_status
```

### Monitor Primitive Commands

```bash
ros2 topic echo /tee/primitive_command
```

### Monitor Primitive Status (from FAL)

```bash
ros2 topic echo /fal/primitive_status
```

### View TEE Logs

```bash
ros2 run task_execution tee_node --ros-args --log-level debug
```

### Check Node Info

```bash
ros2 node info /task_execution_engine
```

## Validation Testing

### Test 1: Invalid Mission (Missing Parameters)

This should be rejected during validation:

```bash
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{\"mission_id\": \"invalid_001\", \"task_type\": \"survey\", \"priority\": 50, \"timeout\": 300.0, \"parameters\": {\"altitude\": 50.0}}"' --once
```

Expected: Validation failure (missing `area` parameter)

### Test 2: Unknown Task Type

```bash
ros2 topic pub /squadron/mission_command std_msgs/msg/String \
  'data: "{\"mission_id\": \"unknown_001\", \"task_type\": \"unknown_type\", \"priority\": 50, \"timeout\": 300.0, \"parameters\": {}}"' --once
```

Expected: No executor found for task type

### Test 3: Emergency Scenario

Simulate low battery:

```bash
# Publish low battery state
ros2 topic pub /mavros/battery sensor_msgs/msg/BatteryState \
  '{voltage: 12.0, percentage: 0.15, current: -5.0}' --once
```

Expected: Emergency RTL triggered

## Performance Metrics

### Monitor Execution Rate

```bash
ros2 topic hz /tee/mission_status
```

Expected: ~2 Hz (configurable in config)

### Check Primitive Generation Time

View logs for "Generated X primitives" messages.

### Monitor State Transitions

View logs for "State transition:" messages.

## Integration with FAL

### Full System Test

1. Launch complete system:

```bash
ros2 launch task_execution tee_with_fal.launch.py
```

2. Verify FAL is receiving primitives:

```bash
ros2 topic echo /tee/primitive_command
```

3. Verify FAL is sending status:

```bash
ros2 topic echo /fal/primitive_status
```

4. Send test mission and observe complete flow:

```bash
ros2 run task_execution test_mission_execution
```

## Expected Execution Flow

```
1. Mission received → TEE state: IDLE → VALIDATING
2. Validation passed → State: VALIDATING → EXECUTING
3. Primitives generated (e.g., 10 primitives for survey)
4. Primitive 1/10 sent to FAL
5. FAL status: started → progress: 0.5 → completed
6. Primitive 2/10 sent to FAL
7. ... (repeat for all primitives)
8. All primitives complete → State: EXECUTING → COMPLETED
9. State: COMPLETED → IDLE (if no more tasks)
```

## Common Issues and Solutions

### Issue 1: TEE node fails to start

**Symptoms**: Import errors or missing dependencies

**Solution**:
```bash
# Check installation
python3 src/task_execution/scripts/verify_installation.py

# Install missing dependencies
pip install networkx shapely numpy pyyaml
```

### Issue 2: Missions not executing

**Symptoms**: Missions enqueued but not starting

**Solutions**:
- Check state machine state (should transition to VALIDATING)
- Check validation logs for failures
- Verify battery and GPS monitors have data
- Check `/mavros/battery` and `/mavros/global_position/global` topics

### Issue 3: Primitives not sent to FAL

**Symptoms**: No messages on `/tee/primitive_command`

**Solutions**:
- Verify executors are generating primitives (check logs)
- Check state is EXECUTING
- Verify no primitive failures

### Issue 4: Status not publishing

**Symptoms**: No messages on `/tee/mission_status`

**Solution**:
- Check TEE node is running
- Verify execution timer is active (20Hz)
- Check for node crashes in logs

## Advanced Configuration

### Modify Safety Thresholds

Edit `config/tee_config.yaml`:

```yaml
safety:
  min_battery_percentage: 0.30  # Increase to 30%
  min_gps_satellites: 10  # Require more satellites
  max_gps_hdop: 1.5  # Stricter HDOP
```

### Adjust Execution Rate

Edit `tee_node.py` line ~95:

```python
self.execution_timer = self.create_timer(
    0.05,  # Change to desired rate (0.05 = 20Hz)
    self._execution_loop,
    callback_group=self.callback_group
)
```

### Change Log Level

```bash
ros2 launch task_execution tee.launch.py log_level:=debug
```

## Verification Checklist

Before considering TEE complete, verify:

- [ ] TEE node starts without errors
- [ ] All components import successfully
- [ ] State machine transitions work correctly
- [ ] Task queue orders by priority
- [ ] Validation rejects invalid missions
- [ ] Executors generate correct primitives
- [ ] Primitives published to correct topic
- [ ] Status messages published regularly
- [ ] Pause/Resume works
- [ ] Abort clears queue
- [ ] Emergency RTL triggers correctly
- [ ] Multiple missions queue and execute sequentially
- [ ] Health monitoring detects critical states

## Next Steps

After successful TEE testing:

1. **Integration Testing**: Test with actual FAL execution
2. **Hardware Testing**: Test with real drone hardware
3. **Squadron Manager Integration**: Connect to multi-drone coordinator
4. **Performance Tuning**: Optimize execution rates and parameters
5. **Edge Case Testing**: Test failure scenarios, timeouts, etc.

## Support

For issues or questions:
- Check logs: `ros2 run task_execution tee_node --ros-args --log-level debug`
- Review README.md for architecture details
- Verify installation: `python3 scripts/verify_installation.py`

## Maintainer

dhandavishal <dhandavishal@yahoo.co.in>
