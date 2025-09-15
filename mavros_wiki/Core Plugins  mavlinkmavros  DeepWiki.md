Relevant source files

-   [libmavconn/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/CHANGELOG.rst)
-   [libmavconn/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/package.xml)
-   [mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CHANGELOG.rst)
-   [mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml)
-   [mavros/src/plugins/command.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp)
-   [mavros/src/plugins/dummy.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp)
-   [mavros/src/plugins/ftp.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/ftp.cpp)
-   [mavros/src/plugins/param.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp)
-   [mavros/src/plugins/rc\_io.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/rc_io.cpp)
-   [mavros/src/plugins/sys\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp)
-   [mavros/src/plugins/sys\_time.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_time.cpp)
-   [mavros/src/plugins/waypoint.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp)
-   [mavros\_extras/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CHANGELOG.rst)
-   [mavros\_extras/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/package.xml)
-   [mavros\_msgs/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CHANGELOG.rst)
-   [mavros\_msgs/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/package.xml)
-   [test\_mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/CHANGELOG.rst)
-   [test\_mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/package.xml)

Core plugins in MAVROS provide essential functionality for communicating with and controlling MAVLink-compatible flight controllers. They form the foundation of the MAVROS system, offering standard capabilities required for most unmanned vehicle operations. This page documents the purpose, architecture, and functionality of the core plugins included in the main MAVROS package. For information about additional plugins provided by the mavros\_extras package, see [Extra Plugins](https://deepwiki.com/mavlink/mavros/3.2-extra-plugins).

## Plugin System Architecture

The MAVROS plugin system allows for modular implementation of different functionalities. Each plugin inherits from a common base class and registers itself to handle specific MAVLink messages and provide ROS services/topics.

Sources: [mavros/src/plugins/sys\_status.cpp482-1027](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L482-L1027) [mavros/src/plugins/command.cpp67-262](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp#L67-L262) [mavros/src/plugins/param.cpp75-1088](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp#L75-L1088) [mavros/src/plugins/waypoint.cpp33-380](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp#L33-L380) [mavros/src/plugins/rc\_io.cpp38-278](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/rc_io.cpp#L38-L278) [mavros/src/plugins/sys\_time.cpp145-500](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_time.cpp#L145-L500) [mavros/src/plugins/ftp.cpp133-792](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/ftp.cpp#L133-L792) [mavros/src/plugins/dummy.cpp35-128](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp#L35-L128)

## Plugin Registration and Loading

All plugins are registered using the `MAVROS_PLUGIN_REGISTER` macro, which adds them to a plugin registry. The MAVROS node loads these plugins at startup. Below is a diagram showing the plugin loading process:

```
UAS (Plugin Container)Plugin ImplementationPlugin RegistryMAVROS NodeUAS (Plugin Container)Plugin ImplementationPlugin RegistryMAVROS Nodeloop[For each plugin]Load available pluginsReturn plugin listCreate plugin instanceInitialize with UAS pointerRegister message subscriptionsCreate publishers/subscribers/servicesStart message routing
```

Sources: [mavros/src/plugins/dummy.cpp133-134](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp#L133-L134) [mavros/package.xml77-94](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L77-L94)

## Core Plugins Overview

The following table provides a summary of the core plugins included in the MAVROS package:

| Plugin Name | Purpose | Key Services/Topics |
| --- | --- | --- |
| sys\_status | System status monitoring | ~/state, ~/battery, ~/statustext |
| command | Command execution | ~/command, ~/arming, ~/takeoff, ~/land |
| param | Parameter management | ~/param/get, ~/param/set, ~/param/pull |
| waypoint | Mission/waypoint handling | ~/waypoints, ~/push, ~/pull |
| rc\_io | RC channel input/output | ~/in, ~/out, ~/override |
| sys\_time | Time synchronization | ~/time\_reference |
| ftp | File transfer protocol | ~/file/\*, ~/checksum |
| local\_position | Local position information | ~/pose, ~/velocity, ~/accel |
| global\_position | Global position information | ~/global, ~/rel\_alt, ~/compass\_hdg |
| imu | IMU data handling | ~/data, ~/mag, ~/temperature |
| setpoint\_\* | Various setpoint plugins | ~/setpoint/\*, ~/cmd\_vel |

## Detailed Plugin Descriptions

### System Status Plugin

The System Status plugin (`sys_status`) is responsible for monitoring the status of the flight controller, including heartbeat, system diagnostics, battery status, and hardware health.

```
TopicsHEARTBEATSYS_STATUSSTATUSTEXTBATTERY_STATUSMEMINFOHWSTATUSEXTENDED_SYS_STATEEVENTFlight ControllerSystemStatusPluginBatteryStatusDiagHeartbeatStatusSystemStatusDiagMemInfoHwStatus~/state~/battery~/statustext~/extended_state~/system_status~/event
```

Key functionality:

-   Monitors connection to the FCU
-   Tracks heartbeat messages for system status
-   Publishes detailed system status information
-   Monitors battery levels and health
-   Provides diagnostic information
-   Reports system errors and warnings
-   Handles firmware version detection

Configuration parameters:

-   `conn_timeout`: Connection timeout in seconds
-   `sys/timesync_mode`: Time synchronization mode (MAVLINK, ONBOARD, or NONE)
-   `sys/min_voltage`: Minimum voltage(s) for battery monitoring

Sources: [mavros/src/plugins/sys\_status.cpp40-157](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L40-L157) [mavros/src/plugins/sys\_status.cpp163-270](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L163-L270) [mavros/src/plugins/sys\_status.cpp277-359](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L277-L359) [mavros/src/plugins/sys\_status.cpp482-1027](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L482-L1027)

### Command Plugin

The Command plugin (`command`) provides an interface to send MAVLink commands to the flight controller.

```
ServicesCOMMAND_LONGCOMMAND_INTCOMMAND_ACK~/command~/command_int~/arming~/set_home~/takeoff~/land~/vtol_transitionCommandPluginFlight Controller
```

Key functionality:

-   Sends MAVLink commands using COMMAND\_LONG and COMMAND\_INT messages
-   Provides services for common commands (arming, takeoff, land, etc.)
-   Handles command acknowledgments
-   Supports command timeouts and retries

Configuration parameters:

-   `command_ack_timeout`: Command acknowledgment timeout in seconds
-   `use_comp_id_system_control`: Whether to use component ID for system control

Sources: [mavros/src/plugins/command.cpp47-152](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp#L47-L152) [mavros/src/plugins/command.cpp187-205](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp#L187-L205)

### Parameter Plugin

The Parameter plugin (`param`) provides interfaces to get, set, and update parameters on the flight controller.

```
TopicsServicesPARAM_REQUEST_LISTPARAM_REQUEST_READPARAM_SETPARAM_VALUE~/param/get~/param/set~/param/pull~/param/push~/param/value~/param/eventParamPluginFlight ControllerROS Parameters
```

Key functionality:

-   Fetches, stores, and updates parameters on the FCU
-   Provides parameter services compatible with ROS parameter system
-   Supports parameter caching for improved performance
-   Handles different MAVLink parameter types
-   Supports parameter namespaces and grouping

Configuration parameters:

-   `param/use_extended`: Whether to use extended parameter protocol
-   `param/quiet_mode`: Whether to reduce parameter-related log messages
-   `param/exclude_from_params`: Parameters to exclude from ROS parameter list

Sources: [mavros/src/plugins/param.cpp57-172](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp#L57-L172) [mavros/src/plugins/param.cpp584-1088](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp#L584-L1088)

### Waypoint Plugin

The Waypoint plugin (`waypoint`) manages mission waypoints, providing interfaces to upload, download, and manipulate mission plans.

```
TopicsServicesMISSION_REQUEST_LISTMISSION_COUNTMISSION_ITEM(_INT)MISSION_CLEAR_ALLMISSION_SET_CURRENTMISSION_ITEM(_INT)MISSION_REQUESTMISSION_ACKMISSION_CURRENTMISSION_ITEM_REACHED~/pull~/push~/clear~/set_current~/waypoints~/reachedWaypointPluginFlight Controller
```

Key functionality:

-   Downloads and uploads mission waypoints
-   Supports mission state tracking
-   Handles mission item types and frames
-   Provides mission flow control
-   Supports partial mission updates (APM-specific)

Configuration parameters:

-   `pull_after_gcs`: Whether to pull waypoints after GCS changes them
-   `use_mission_item_int`: Whether to use MISSION\_ITEM\_INT instead of MISSION\_ITEM
-   `enable_partial_push`: Whether to enable partial waypoint updates

Sources: [mavros/src/plugins/waypoint.cpp33-304](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp#L33-L304)

### RC I/O Plugin

The RC I/O plugin (`rc_io`) handles radio control input and output channels, allowing for monitoring and overriding RC channels.

```
TopicsRC_CHANNELS_RAWRC_CHANNELSSERVO_OUTPUT_RAWRC_CHANNELS_OVERRIDE~/in~/out~/overrideRCIOPluginFlight Controller
```

Key functionality:

-   Publishes RC input channel values
-   Publishes servo output values
-   Handles RC channel overrides
-   Supports MAVLink v2 extensions (up to 18 channels)

Sources: [mavros/src/plugins/rc\_io.cpp38-278](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/rc_io.cpp#L38-L278)

### System Time Plugin

The System Time plugin (`sys_time`) handles time synchronization between the FCU and ROS.

```
TopicsSYSTEM_TIMETIMESYNC(optional)~/time_reference~/timesync_status/clockSystemTimePluginFlight Controller
```

Key functionality:

-   Synchronizes system time between ROS and FCU
-   Estimates time offset between systems
-   Provides time reference topics
-   Supports different timesync modes (MAVLINK, ONBOARD, NONE)
-   Can publish to ROS `/clock` topic (useful for simulation)

Configuration parameters:

-   `time_ref_source`: Source for time reference
-   `timesync_mode`: Time synchronization mode
-   `system_time_rate`: Rate for system time messages
-   `timesync_rate`: Rate for timesync messages
-   `publish_sim_time`: Whether to publish to `/clock` topic

Sources: [mavros/src/plugins/sys\_time.cpp40-138](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_time.cpp#L40-L138) [mavros/src/plugins/sys\_time.cpp145-500](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_time.cpp#L145-L500)

### FTP Plugin

The FTP plugin (`ftp`) implements the MAVLink File Transfer Protocol, allowing file operations on the flight controller.

```
ServicesFILE_TRANSFER_PROTOCOL~/file/list~/file/open~/file/close~/file/read~/file/write~/file/mkdir~/file/rmdir~/file/remove~/file/rename~/file/truncate~/file/checksumFTPPluginFlight Controller
```

Key functionality:

-   Provides a complete set of file operations
-   Handles file transfers in blocks
-   Supports directory operations
-   Manages file checksums
-   Interfaces with MAVLink FTP protocol

Sources: [mavros/src/plugins/ftp.cpp60-792](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/ftp.cpp#L60-L792)

## Developing Custom Plugins

MAVROS can be extended with custom plugins by inheriting from the `Plugin` base class. The `dummy.cpp` file provides a template for creating new plugins:

```
Custom Plugin ImplementationConstructor (setup services/topics)get_subscriptions() (define message handlers)Message handlersPlugin Base ClassMAVROS_PLUGIN_REGISTER macro
```

Key steps to create a custom plugin:

1.  Create a class inheriting from `Plugin`
2.  Implement a constructor that sets up services and topics
3.  Implement `get_subscriptions()` to define message handlers
4.  Implement message handler functions
5.  Register the plugin using `MAVROS_PLUGIN_REGISTER` macro

Sources: [mavros/src/plugins/dummy.cpp35-128](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp#L35-L128) [mavros/src/plugins/dummy.cpp133-134](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp#L133-L134)

## Common Plugin Patterns

All core plugins follow similar patterns for integration with the MAVROS system:

1.  **Message Subscription**: Using `make_handler()` to subscribe to specific MAVLink message types
2.  **Service Handlers**: Creating ROS services with appropriate callbacks
3.  **Parameter Monitoring**: Using `node_declare_and_watch_parameter()` for configuration
4.  **Diagnostic Functions**: Implementing diagnostic tasks for system monitoring
5.  **Timing Management**: Using ROS timers for periodic operations

These patterns ensure consistent behavior across plugins and make the system more maintainable.

## Plugin Loading and Configuration

MAVROS loads plugins based on the configuration in `mavros_plugins.xml`. This file defines which plugins are available to the system. Plugins can be enabled or disabled using ROS parameters.

Configuration parameters:

-   `plugin_allowlist`: List of allowed plugins (if specified, only these will be loaded)
-   `plugin_denylist`: List of denied plugins (these will not be loaded)

Sources: [mavros/package.xml77-94](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L77-L94)