Relevant source files

-   [CONTRIBUTING.md](https://github.com/mavlink/mavros/blob/44fa2f90/CONTRIBUTING.md)
-   [README.md](https://github.com/mavlink/mavros/blob/44fa2f90/README.md)
-   [libmavconn/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md)
-   [mavros/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md)
-   [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml)
-   [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml)
-   [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml)
-   [mavros/launch/px4\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml)
-   [mavros\_extras/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/README.md)

This page explains how to configure MAVROS for different autopilot types and use cases. It covers connection configuration, plugin management, and parameter setup for optimizing MAVROS functionality. For autopilot-specific details, see [PX4 Configuration](https://deepwiki.com/mavlink/mavros/4.1-px4-configuration) and [ArduPilot Configuration](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration).

## Configuration Architecture

MAVROS configuration consists of several components that work together to establish communication between ROS and MAVLink-enabled flight controllers:

```
Runtime ConfigurationConfiguration FilesConfiguration ComponentsConnection ConfigurationPlugin ConfigurationParameter SettingsCoordinate Frame Configurationpx4_config.yamlapm_config.yamlpx4_pluginlists.yamlapm_pluginlists.yamlConnection URLCommand Line ArgumentsLaunch Files
```

Sources: [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml) [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml) [mavros/launch/px4\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml) [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml)

## Connection Configuration

MAVROS uses a URL-based system to specify the connection to the flight controller unit (FCU) and ground control station (GCS).

### Connection URL Format

Connection URLs follow a schema-based format that specifies the transport type and associated parameters:

```
Transport ImplementationsConnection URL ComponentsspecifiesidentifiesspecifiescustomizesSchema(transport type)Host/Device PathPort NumberOptions(e.g., system ID)MAVConnInterfaceENDPOINTNETWORK_ENDPOINTCONNECTION_BEHAVIORMAVConnSerialMAVConnUDPMAVConnTCPClientMAVConnTCPServer
```

Sources: [mavros/README.md33-53](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L33-L53) [libmavconn/README.md9-26](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L9-L26)

### Supported URL Schemas

| Schema | Format | Description |
| --- | --- | --- |
| Serial | `/path/to/serial/device[:baudrate]` | Basic serial connection |
| Serial | `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]` | Serial with system ID specification |
| Serial HW Flow Control | `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]` | Serial with hardware flow control |
| UDP | `udp://[bind_host][:port]@[remote_host[:port]][/?ids=sysid,compid]` | UDP connection |
| UDP Broadcast | `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]` | UDP broadcast until GCS discovery |
| UDP Permanent Broadcast | `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]` | UDP permanent broadcast |
| TCP Client | `tcp://[server_host][:port][/?ids=sysid,compid]` | TCP client connection |
| TCP Server | `tcp-l://[bind_host][:port][/?ids=sysid,compid]` | TCP server connection |

Sources: [mavros/README.md36-45](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L36-L45) [libmavconn/README.md16-24](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L16-L24)

### URL Parameters

-   `bind_host`: Default is `0.0.0.0` (IPv4 ANY)
-   UDP default ports: 14555 @ 14550
-   TCP default port: 5760
-   `ids=sysid,compid`: Override system and component IDs

Sources: [mavros/README.md47-53](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L47-L53)

## Plugin Configuration

MAVROS uses a plugin system to organize and modularize functionality. Plugins can be enabled or disabled based on the specific requirements of your application.

### Plugin Lists Configuration

Plugin configuration is managed through the `plugin_denylist` and `plugin_allowlist` parameters in the YAML configuration files:

```
Plugin CategoriesPlugin SelectionPlugin Configuration Filespx4_pluginlists.yamlapm_pluginlists.yamlplugin_denylist(Plugins to exclude)plugin_allowlist(Only load these plugins)Core Plugins
```

Sources: [mavros/launch/px4\_pluginlists.yaml1-14](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml#L1-L14) [mavros/launch/apm\_pluginlists.yaml1-19](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml#L1-L19)

### PX4 vs ArduPilot Plugin Configuration

PX4 and ArduPilot use different plugin configurations due to differences in their functionality:

| Feature | PX4 | ArduPilot |
| --- | --- | --- |
| Disabled plugins | image\_pub, vibration, distance\_sensor, rangefinder, wheel\_odometry | actuator\_control, ftp, hil, altitude, debug\_value, image\_pub, px4flow, vibration, vision\_speed\_estimate, wheel\_odometry |
| Plugin support | Supports more plugins by default | Disables more plugins due to compatibility issues |

Sources: [mavros/launch/px4\_pluginlists.yaml3-11](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml#L3-L11) [mavros/launch/apm\_pluginlists.yaml3-15](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml#L3-L15)

## Parameter Configuration

MAVROS parameters configure various aspects of the system, from plugin-specific settings to system-wide behaviors.

### Parameter Structure

Parameters are organized hierarchically in YAML files, with sections for system plugins and individual MAVROS plugins:

```
Parameter CategoriesParameter Configuration Filespx4_config.yamlapm_config.yamlSystem Plugin ParametersMAVROS Plugin Parameterssyssys_status parameterstimesys_time parameterscmdcommand parameterslocal_positionlocal position parametersglobal_positionglobal position parametersdistance_sensordistance sensor parametersfake_gpsfake GPS parameters
```

Sources: [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml) [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml)

### Common System Parameters

| Parameter | Description | Default Value |
| --- | --- | --- |
| `startup_px4_usb_quirk` | USB quirk for PX4 | `false` |
| `sys/min_voltage` | Diagnostics minimum voltage | `[10.0]` |
| `sys/disable_diag` | Disable system status diagnostics | `false` |
| `sys/heartbeat_rate` | Send heartbeat rate in Hz | `1.0` |
| `sys/conn_timeout` | Heartbeat timeout in seconds | `10.0` |
| `time/time_ref_source` | Time reference source | `"fcu"` |
| `time/timesync_mode` | Timesync mode | `MAVLINK` |
| `time/timesync_rate` | TIMESYNC rate in Hz | `10.0` |

Sources: [mavros/launch/px4\_config.yaml5-25](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L5-L25) [mavros/launch/apm\_config.yaml5-26](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L5-L26)

### Plugin-Specific Parameters

Many plugins have their own parameters to configure behavior. Some key plugin parameters include:

#### Global Position Plugin Parameters

```
global_position:
  frame_id: "map"               # origin frame
  child_frame_id: "base_link"   # body-fixed frame
  rot_covariance: 99999.0       # covariance for attitude
  gps_uere: 1.0                 # User Equivalent Range Error (UERE) of GPS sensor (m)
  use_relative_alt: true        # use relative altitude for local coordinates
  tf.send: false                # send TF?
  tf.frame_id: "map"            # TF frame_id
  tf.global_frame_id: "earth"   # TF earth frame_id
  tf.child_frame_id: "base_link"  # TF child_frame_id
```

Sources: [mavros/launch/px4\_config.yaml49-60](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L49-L60) [mavros/launch/apm\_config.yaml50-60](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L50-L60)

#### IMU Plugin Parameters

```
imu:
  frame_id: "base_link"
  linear_acceleration_stdev: 0.0003
  angular_velocity_stdev: 0.0003490659  # 0.02 degrees
  orientation_stdev: 1.0
  magnetic_stdev: 0.0
```

Sources: [mavros/launch/px4\_config.yaml62-70](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L62-L70) [mavros/launch/apm\_config.yaml63-70](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L63-L70)

### Configuration Differences Between PX4 and ArduPilot

While many parameters are shared between PX4 and ArduPilot configurations, there are some important differences:

| Parameter | PX4 | ArduPilot |
| --- | --- | --- |
| `sys/heartbeat_mav_type` | Not specified | `"ONBOARD_CONTROLLER"` |
| `fake_gps/use_hil_gps` | Not present | `true` |
| `fake_gps/gps_id` | Not present | `4` |
| `fake_gps/satellites_visible` | `5` | `6` |
| `fake_gps/fix_type` | `3` | `3` |
| `odometry/fcu.odom_child_id_des` | `"base_link"` | `"map"` |

Sources: [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml) [mavros/launch/apm\_config.yaml16-17](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L16-L17) [mavros/launch/apm\_config.yaml179-191](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L179-L191) [mavros/launch/px4\_config.yaml247-248](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L247-L248) [mavros/launch/apm\_config.yaml235-236](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L235-L236)

## Coordinate Frame Configuration

MAVROS translates between different coordinate frame conventions used by MAVLink and ROS. Understanding and properly configuring these frame transformations is essential.

### Coordinate Frame Transformation

```
ROS FramesMAVLink (FCU) Framestransform_frame_ned_enutransform_frame_aircraft_baselinkNEDNorth-East-DownAircraftForward-Right-DownENUEast-North-Upbase_linkForward-Left-Up
```

Sources: [mavros/README.md56-67](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L56-L67)

### Frame Configuration Parameters

Several plugins have parameters to configure frame transforms:

| Plugin | Parameter | Description |
| --- | --- | --- |
| `global_position` | `tf.send` | Enable/disable TF publishing |
| `global_position` | `tf.frame_id` | Origin frame ID (typically "map") |
| `global_position` | `tf.child_frame_id` | Body-fixed frame ID (typically "base\_link") |
| `local_position` | `frame_id` | Origin frame for local position |
| `local_position` | `tf.send` | Enable/disable TF publishing |
| `setpoint_position` | `mav_frame` | MAVLink frame type (typically LOCAL\_NED) |
| `setpoint_velocity` | `mav_frame` | MAVLink frame type for velocity commands |

Sources: [mavros/launch/px4\_config.yaml49-79](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L49-L79) [mavros/launch/px4\_config.yaml108-128](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L108-L128)

## Usage Examples

### Basic Configuration with Launch Files

To start MAVROS with a specific configuration:

```
# For PX4 autopilot
ros2 launch mavros px4.launch

# For ArduPilot
ros2 launch mavros apm.launch
```

### Custom Connection Configuration

To specify custom connection URLs:

```
# Connect to PX4 over USB with custom baudrate
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600

# Connect to ArduPilot over network
ros2 launch mavros apm.launch fcu_url:=udp://@192.168.1.10:14550
```

### Configuration with YAML Files

To use custom YAML configuration files:

```
ros2 run mavros mavros_node --ros-args --params-file custom_config.yaml
```

Sources: [mavros/README.md125-126](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L125-L126)

## Common Configuration Issues and Solutions

| Issue | Cause | Solution |
| --- | --- | --- |
| No connection to FCU | Incorrect URL or permissions | Check device permissions and URL format |
| Missing geographic data | GeographicLib datasets not installed | Run `install_geographiclib_datasets.sh` script |
| Plugin not loading | Plugin in denylist | Remove from plugin\_denylist in configuration |
| Frame transformation issues | Incorrect frame\_id settings | Check frame\_id parameters in config files |
| Parameter changes not taking effect | ROS parameter namespace issues | Verify parameter namespace and formatting |

Sources: [mavros/README.md154-157](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L154-L157) [README.md14-20](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L14-L20)

This page provides a comprehensive overview of MAVROS configuration. For autopilot-specific details, see [PX4 Configuration](https://deepwiki.com/mavlink/mavros/4.1-px4-configuration) and [ArduPilot Configuration](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration), and for information on command-line tools for interacting with MAVROS, see [Command Line Tools](https://deepwiki.com/mavlink/mavros/5-command-line-tools).