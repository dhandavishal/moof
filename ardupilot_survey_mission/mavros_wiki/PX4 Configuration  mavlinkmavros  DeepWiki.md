Relevant source files

-   [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml)
-   [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml)
-   [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml)
-   [mavros/launch/px4\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml)

This document outlines the specific configuration details for using MAVROS with the PX4 autopilot. For ArduPilot-specific configuration, please refer to [ArduPilot Configuration](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration).

## 1\. Introduction

MAVROS requires specific configuration to work optimally with PX4 autopilots. This configuration includes plugin selection, coordinate frame settings, and communication parameters. The primary configuration files for PX4 are `px4_config.yaml` and `px4_pluginlists.yaml` which define the behavior of MAVROS when communicating with PX4 flight controllers.

```
HardwareMAVROS SystemMAVROS Configuration Filespx4_config.yamlpx4_pluginlists.yamlmavros_nodePlugin LoaderActive PluginsPX4 Flight Controller
```

Sources: [mavros/launch/px4\_config.yaml1-5](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L1-L5) [mavros/launch/px4\_pluginlists.yaml1-15](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml#L1-L15)

## 2\. Configuration Files Overview

### 2.1 Plugin List Configuration

The `px4_pluginlists.yaml` file specifies which plugins should not be loaded when connecting to a PX4 flight controller:

```
Denied PluginsPlugin Selection for PX4Remove DeniedAll Available PluginsPlugin DenylistActive Plugins for PX4image_pubvibrationdistance_sensorrangefinderwheel_odometry
```

The PX4 denylist is more limited compared to ArduPilot, allowing more plugins to be used by default with PX4.

Sources: [mavros/launch/px4\_pluginlists.yaml1-15](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml#L1-L15) [mavros/launch/apm\_pluginlists.yaml1-19](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml#L1-L19)

### 2.2 Main Configuration File

The `px4_config.yaml` contains the configuration for all plugins that can be used with PX4. It follows a hierarchical structure where each plugin has its own section with specific parameters.

```
px4_config.yaml Structure/**/**/sys/**/time/**/setpoint_*//global_position//local_position/**/missionUnsupported markdown: listUnsupported markdown: listUnsupported markdown: listUnsupported markdown: listUnsupported markdown: list
```

Sources: [mavros/launch/px4\_config.yaml1-10](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L1-L10) [mavros/launch/px4\_config.yaml134-139](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L134-L139)

## 3\. PX4-Specific Settings

### 3.1 USB Quirk Setting

PX4 has a specific USB connection quirk parameter that may be needed for some hardware:

```
startup_px4_usb_quirk: false
```

This parameter should be set to `true` if you experience connection issues with PX4 over USB. When enabled, it adds a small delay during startup to accommodate timing issues in some PX4 USB implementations.

Sources: [mavros/launch/px4\_config.yaml5](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L5-L5)

### 3.2 System Status Configuration

```
min_voltage: [10.0]   # Use vector for multiple batteries
disable_diag: false   # Enable diagnostics
heartbeat_rate: 1.0   # Heartbeat rate in Hz
conn_timeout: 10.0    # Heartbeat timeout in seconds
```

These settings control how the system monitors the health of the PX4 autopilot. The `min_voltage` parameter can be expanded to monitor multiple batteries, for example: `[16.2, 16.0]`.

Sources: [mavros/launch/px4\_config.yaml10-16](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L10-L16)

### 3.3 Time Synchronization

```
time_ref_source: "fcu"
timesync_mode: MAVLINK
timesync_avg_alpha: 0.6
timesync_rate: 10.0
system_time_rate: 1.0
```

These settings manage time synchronization between the companion computer and PX4. The `timesync_mode` set to `MAVLINK` uses the MAVLink protocol for time synchronization, which is the recommended mode for PX4.

Sources: [mavros/launch/px4\_config.yaml19-25](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L19-L25)

## 4\. Plugin Configurations

### 4.1 Position and Orientation Plugins

The position and orientation plugins handle coordinate frames and transformations:

| Plugin | Important Parameters | Description |
| --- | --- | --- |
| Global Position | `frame_id: "map"`, `use_relative_alt: true` | Configures global position coordinates |
| Local Position | `frame_id: "map"`, `tf.send: false` | Configures local position coordinates |
| IMU | `linear_acceleration_stdev: 0.0003` | Sets IMU data parameters |

```
Position Data FlowGPS DataLocal Position DataIMU DataNav Satellite Fix/mavros/global_position/globalPose with Covariance/mavros/local_position/poseIMU Data/mavros/imu/dataPX4 FCUGlobal Position PluginLocal Position PluginIMU PluginROS Topics
```

Sources: [mavros/launch/px4\_config.yaml49-59](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L49-L59) [mavros/launch/px4\_config.yaml62-70](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L62-L70) [mavros/launch/px4\_config.yaml72-79](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L72-L79)

### 4.2 Setpoint Plugins

Setpoint plugins are critical for controlling PX4 vehicles:

| Plugin | Parameters | Description |
| --- | --- | --- |
| Setpoint Attitude | `reverse_thrust: false`, `use_quaternion: false` | Control vehicle attitude |
| Setpoint Position | `mav_frame: LOCAL_NED` | Control vehicle position |
| Setpoint Raw | `thrust_scaling: 1.0` | Direct actuator control |
| Setpoint Velocity | `mav_frame: LOCAL_NED` | Control vehicle velocity |

PX4 expects normalized thrust values between 0 and 1, as noted in the configuration.

Sources: [mavros/launch/px4\_config.yaml92-100](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L92-L100) [mavros/launch/px4\_config.yaml102-107](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L102-L107) [mavros/launch/px4\_config.yaml108-116](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L108-L116) [mavros/launch/px4\_config.yaml125-128](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L125-L128)

### 4.3 Mission and Waypoint Plugin

The mission plugin handles waypoint operations with PX4:

```
pull_after_gcs: true        # update mission if GCS updates
use_mission_item_int: true  # use MISSION_ITEM_INT message
```

Setting `use_mission_item_int` to `true` is recommended for PX4 as it provides higher precision for mission waypoints.

Sources: [mavros/launch/px4\_config.yaml134-139](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L134-L139)

### 5.1 Distance Sensor Configuration

The distance sensor configuration in PX4 supports multiple sensors:

```
hrlv_ez4_pub:
  id: 0
  frame_id: "hrlv_ez4_sonar"
  orientation: PITCH_270 # downward-facing
  field_of_view: 0.0
  send_tf: true
  sensor_position: {x: 0.0, y: 0.0, z: -0.1}
lidarlite_pub:
  id: 1
  frame_id: "lidarlite_laser"
  orientation: PITCH_270
  field_of_view: 0.0
  send_tf: true
  sensor_position: {x: 0.0, y: 0.0, z: -0.1}
```

The configuration allows multiple sensors with different IDs and orientations. For PX4, the orientation values follow the MAVLink coordinate system.

Sources: [mavros/launch/px4\_config.yaml149-183](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L149-L183)

### 5.2 Vision Pose Estimation

For external vision systems (like motion capture):

```
tf.listen: false
tf.frame_id: "odom"
tf.child_frame_id: "vision_estimate"
tf.rate_limit: 10.0
```

These settings configure how vision-based position estimation is integrated with PX4.

Sources: [mavros/launch/px4\_config.yaml258-264](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L258-L264)

### 5.3 Fake GPS Configuration

The fake GPS plugin allows position data from external sources to be sent to PX4 as GPS data:

```
use_mocap: true
mocap_transform: true
use_vision: false
geo_origin.lat: 47.3667
geo_origin.lon: 8.5500
geo_origin.alt: 408.0
```

This is particularly useful for indoor navigation or testing where GPS signal isn't available.

Sources: [mavros/launch/px4\_config.yaml190-209](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L190-L209)

## 6\. Frame Transformations and TF Settings

PX4 uses different coordinate frames than ROS. MAVROS handles these transformations:

```
MAVROS Frame IDsCoordinate SystemsMAVROSTransformationPX4 (NED)North-East-DownROS (ENU)East-North-Upmapearthbase_linklocal_positionglobal_position
```

Each plugin can be configured to publish TF transforms between these frames. By default, many TF publishers are disabled (`tf.send: false`) to avoid duplicate transforms.

| Plugin | Parameter | Default | Purpose |
| --- | --- | --- | --- |
| Global Position | `tf.send` | `false` | TF between earth and base\_link |
| Local Position | `tf.send` | `false` | TF between map and base\_link |
| Vision Pose | `tf.send` | N/A | TF for external vision system |

Sources: [mavros/launch/px4\_config.yaml49-59](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L49-L59) [mavros/launch/px4\_config.yaml72-79](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L72-L79) [mavros/launch/px4\_config.yaml258-264](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L258-L264)

## 7\. Plugin Selection for PX4

PX4 uses a shorter denylist compared to ArduPilot, meaning more plugins are enabled by default. The following plugins are disabled for PX4:

-   `image_pub`
-   `vibration`
-   `distance_sensor`
-   `rangefinder`
-   `wheel_odometry`

If you need to use any of these plugins, you can edit the `px4_pluginlists.yaml` file and remove them from the denylist.

Sources: [mavros/launch/px4\_pluginlists.yaml1-15](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml#L1-L15)

## 8\. Usage and Launch

To use these configurations with a PX4 autopilot, you would typically include the configuration files in your launch file. MAVROS provides convenient launch files that already include these configurations.

Example launch command:

```
ros2 launch mavros px4.launch.py fcu_url:="<connection_url>"
```

Where `<connection_url>` specifies the connection method to your PX4 device, such as:

-   Serial: `/dev/ttyACM0:57600`
-   UDP: `udp://:14540@127.0.0.1:14557`
-   TCP: `tcp://127.0.0.1:5760`

## 9\. Conclusion

This document covers the essential configuration settings for using MAVROS with PX4 autopilots. The default configuration in `px4_config.yaml` provides a good starting point for most use cases, but you may need to adjust specific parameters based on your hardware setup, sensor configuration, and application requirements.

For comparing PX4 configuration with ArduPilot configuration, see [ArduPilot Configuration](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration).