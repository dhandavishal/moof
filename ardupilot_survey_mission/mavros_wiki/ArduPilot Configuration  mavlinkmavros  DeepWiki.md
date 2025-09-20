Relevant source files

-   [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml)
-   [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml)
-   [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml)
-   [mavros/launch/px4\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml)

This document provides detailed information about configuring MAVROS for use with ArduPilot-based flight controllers. For PX4-specific configuration, please see [PX4 Configuration](https://deepwiki.com/mavlink/mavros/4.1-px4-configuration).

## Overview

ArduPilot requires specific MAVROS configuration settings that differ from those used with PX4. These settings ensure proper communication, data exchange, and functionality when using ArduPilot firmware variants (ArduCopter, ArduPlane, ArduRover, etc.) with MAVROS.

```
ArduPilot Configuration StructureKey Configuration SectionsSystem Pluginssys_status, sys_timeapm_config.yamlCore MAVROS Pluginsglobal_position, local_position, etc.apm_pluginlists.yamlDenied Pluginsactuator_control, ftp, etc.
```

Sources: [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml) [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml)

## Configuration Files

ArduPilot configuration in MAVROS is primarily managed through two YAML files:

1.  **apm\_config.yaml** - Contains configuration parameters for all enabled plugins
2.  **apm\_pluginlists.yaml** - Specifies which plugins should be disabled (denylisted)

### Plugin Denylist

ArduPilot has specific plugins that are disabled by default due to compatibility issues or because equivalent functionality is provided through different mechanisms.

```
Plugin Denylist for ArduPilotCommon Denied Pluginsactuator_controlftphilaltitudedebug_valueimage_pubpx4flowvibrationvision_speed_estimatewheel_odometry
```

Sources: [mavros/launch/apm\_pluginlists.yaml1-16](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml#L1-L16)

## Key ArduPilot-Specific Settings

ArduPilot configuration differs from PX4 in several important areas. Below are the most significant differences:

### System Configuration

| Parameter | ArduPilot Setting | Description |
| --- | --- | --- |
| `heartbeat_mav_type` | `"ONBOARD_CONTROLLER"` | MAVLink component type sent in heartbeat messages |
| `conn_timeout` | `10.0` | Heartbeat timeout in seconds |
| `timesync_mode` | `MAVLINK` | Time synchronization mode |

The heartbeat configuration is particularly important as it identifies MAVROS as an onboard controller to ArduPilot.

Sources: [mavros/launch/apm\_config.yaml9-26](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L9-L26)

### Coordinate Frames and Transformations

ArduPilot uses specific frame definitions that must be properly configured:

```
ArduPilot Frame Configurationglobal_positionframe_id: mapchild_frame_id: base_linklocal_positionframe_id: mapchild_frame_id: base_linkimuframe_id: base_linkvision_poseframe_id: mapchild_frame_id: vision_estimateodometryparent_id: mapchild_id: mapMAVROS Core
```

Note that ArduPilot's odometry configuration uses `map` as both parent and child frames, unlike PX4 which uses `map` and `base_link`.

Sources: [mavros/launch/apm\_config.yaml49-80](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L49-L80) [mavros/launch/apm\_config.yaml232-236](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L232-L236)

### Setpoint Configuration

Setpoint plugins control how MAVROS sends target commands to ArduPilot:

| Plugin | Key Parameter | Value | Description |
| --- | --- | --- | --- |
| `setpoint_position` | `mav_frame` | `LOCAL_NED` | Reference frame for position setpoints |
| `setpoint_velocity` | `mav_frame` | `LOCAL_NED` | Reference frame for velocity setpoints |
| `setpoint_attitude` | `use_quaternion` | `false` | Whether to use quaternion representation |
| `setpoint_raw` | `thrust_scaling` | `1.0` | Scaling factor for thrust commands |

These settings ensure that commands sent to ArduPilot are properly formatted and interpreted.

Sources: [mavros/launch/apm\_config.yaml87-127](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L87-L127)

### External Positioning Systems

ArduPilot has specific configurations for external positioning systems:

#### GNSS Emulation (fake\_gps)

```
ArduPilot Fake GPS Configurationuse_mocap: trueuse_vision: falseuse_hil_gps: truemocap_transform: falsemocap_withcovariance: falsegeo_origin.lat/lon/altfix_type: 3gps_id: 4Input SourcesCoordinate TransformsOutput to ArduPilotMotion CaptureVision SystemHIL GPSCoordinate ConversionArduPilot FCU
```

ArduPilot's fake GPS setup includes additional parameters not present in PX4:

-   `use_hil_gps: true` - Uses HIL\_GPS message type
-   `gps_id: 4` - Specific GPS ID for ArduPilot
-   `horiz_accuracy: 0.5` - Horizontal accuracy in meters
-   `vert_accuracy: 0.5` - Vertical accuracy in meters

Sources: [mavros/launch/apm\_config.yaml171-197](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L171-L197)

#### Vision Pose Estimation

ArduPilot can receive vision-based pose information through the `vision_pose` plugin:

| Parameter | Value | Description |
| --- | --- | --- |
| `tf.frame_id` | `map` | Reference frame |
| `tf.child_frame_id` | `vision_estimate` | Vision system frame |
| `tf.rate_limit` | `10.0` | Maximum update rate (Hz) |

Sources: [mavros/launch/apm\_config.yaml246-252](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L246-L252)

### Sensor Configuration

#### Distance Sensors

ArduPilot supports rangefinder sensors through the `distance_sensor` plugin:

```
rangefinder_pub:
  id: 0
  frame_id: "lidar"
  field_of_view: 0.0
  send_tf: false
  sensor_position: {x:  0.0, y:  0.0, z:  -0.1}
rangefinder_sub:
  subscriber: true
  id: 1
  orientation: PITCH_270
```

For ArduPilot, the `PITCH_270` orientation is especially important as it's one of the few orientations supported by ArduPilot (as of version 3.4+).

Sources: [mavros/launch/apm\_config.yaml151-165](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L151-L165)

## Major Differences from PX4 Configuration

The table below highlights the most significant configuration differences between ArduPilot and PX4:

| Feature | ArduPilot | PX4 | Notes |
| --- | --- | --- | --- |
| Heartbeat MAV Type | `ONBOARD_CONTROLLER` | Not specified | Identifies MAVROS component type to autopilot |
| Odometry Child Frame | `map` | `base_link` | Affects coordinate frame transformations |
| fake\_gps | Uses `HIL_GPS` message | Uses GPS\_INPUT message | Different message types for external position |
| Denied Plugins | More extensive list | Fewer denied plugins | ArduPilot denies more plugins by default |
| Wheel Odometry TF | `tf.send: true` | `tf.send: false` | ArduPilot publishes TF data for wheel odometry |

Sources: [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml) [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml) [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml) [mavros/launch/px4\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml)

## Common Configuration Parameters

Below are some commonly used parameters that may need adjustment for your specific ArduPilot setup:

### Connection Settings

Connection timeout and heartbeat rate can be adjusted based on your link quality:

```
/**/sys:
  ros__parameters:
    heartbeat_rate: 1.0   # send heartbeat rate in Hertz
    heartbeat_mav_type: "ONBOARD_CONTROLLER"
    conn_timeout: 10.0    # heartbeat timeout in seconds
```

### Battery Monitoring

Battery monitoring thresholds can be set to match your specific battery:

```
/**/sys:
  ros__parameters:
    min_voltage: [10.0]   # diagnostics min voltage
```

For multiple batteries, you can specify a vector: `[16.2, 16.0]`

### Setpoint Frame Configuration

If you need to change the coordinate frame used for setpoints:

```
/**/setpoint_position:
  ros__parameters:
    mav_frame: LOCAL_NED  # Options include LOCAL_NED, LOCAL_OFFSET_NED, etc.
```

Sources: [mavros/launch/apm\_config.yaml9-17](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L9-L17) [mavros/launch/apm\_config.yaml107-127](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L107-L127)

## Conclusion

Configuring MAVROS for ArduPilot requires attention to specific settings and plugin configurations that differ from PX4. The primary configuration files (`apm_config.yaml` and `apm_pluginlists.yaml`) provide a solid starting point, but you may need to adjust certain parameters based on your specific hardware setup and use case.

For more information about the general MAVROS architecture, see [MAVROS Overview](https://deepwiki.com/mavlink/mavros/1-mavros-overview) and [Architecture and Communication Flow](https://deepwiki.com/mavlink/mavros/2-architecture-and-communication-flow).