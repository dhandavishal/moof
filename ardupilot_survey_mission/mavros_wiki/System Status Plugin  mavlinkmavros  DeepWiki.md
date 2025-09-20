Relevant source files

-   [mavros/src/lib/enum\_to\_string.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/enum_to_string.cpp)
-   [mavros/src/plugins/command.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp)
-   [mavros/src/plugins/dummy.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp)
-   [mavros/src/plugins/ftp.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/ftp.cpp)
-   [mavros/src/plugins/param.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp)
-   [mavros/src/plugins/rc\_io.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/rc_io.cpp)
-   [mavros/src/plugins/sys\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp)
-   [mavros/src/plugins/sys\_time.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_time.cpp)
-   [mavros/src/plugins/waypoint.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp)
-   [mavros\_extras/src/plugins/companion\_process\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/companion_process_status.cpp)
-   [mavros\_msgs/msg/CompanionProcessStatus.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/CompanionProcessStatus.msg)

The System Status Plugin is a core MAVROS plugin that handles vital communication between ROS and the flight controller unit (FCU), providing essential status information about the vehicle. It manages heartbeat messages, system health diagnostics, battery information, status text messages, and maintains the connection state.

For information about time synchronization, see [System Time Plugin](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration#3.1.4), and for parameter management, see [Parameter Plugin](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration#3.1.5).

## Overview and Purpose

The System Status Plugin performs several critical functions:

1.  Maintains connection status through heartbeat monitoring
2.  Publishes system status, including armed state and flight mode
3.  Provides battery monitoring and diagnostics
4.  Processes diagnostic information about the FCU
5.  Handles status text messages from the flight controller
6.  Reports memory and hardware status for ArduPilot-based vehicles
7.  Stores and provides vehicle information

This plugin is required by all MAVROS applications as it establishes the basic connection status and monitoring functionality.

Sources: [mavros/src/plugins/sys\_status.cpp476-477](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L476-L477) [mavros/src/plugins/sys\_status.cpp9-16](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L9-L16)

## Architecture

The System Status Plugin consists of several components working together to provide a comprehensive view of the vehicle's status.

```
SystemStatusPlugin-HeartbeatStatus hb_diag-SystemStatusDiag sys_diag-BatteryStatusDiag[] batt_diag-MemInfo mem_diag-HwStatus hwst_diag+handle_heartbeat()+handle_sys_status()+handle_statustext()+handle_extended_sys_state()+handle_battery_status()+handle_meminfo()+handle_hwstatus()+handle_autopilot_version()HeartbeatStatus+clear()+tick()+run()SystemStatusDiag+set()+run()BatteryStatusDiag+set()+setcell_v()+run()MemInfo+set()+run()HwStatus+set()+run()
```

Sources: [mavros/src/plugins/sys\_status.cpp65-157](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L65-L157) [mavros/src/plugins/sys\_status.cpp163-270](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L163-L270) [mavros/src/plugins/sys\_status.cpp276-359](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L276-L359) [mavros/src/plugins/sys\_status.cpp365-415](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L365-L415) [mavros/src/plugins/sys\_status.cpp421-473](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L421-L473) [mavros/src/plugins/sys\_status.cpp482-496](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L482-L496)

## Message Flow

The following diagram illustrates how messages flow from the FCU through the System Status Plugin to ROS topics:

```
ROS TopicsSystemStatusPluginFlight Controller UnitHEARTBEAT MessageSYS_STATUS MessageSTATUSTEXT MessageEXTENDED_SYS_STATE MessageBATTERY_STATUS MessageMEMINFO Message (APM)HWSTATUS Message (APM)handle_heartbeat()handle_sys_status()handle_statustext()handle_extended_sys_state()handle_battery_status()handle_meminfo()handle_hwstatus()HeartbeatStatusSystemStatusDiagBatteryStatusDiagMemInfoHwStatusstatesys_statusstatustext/recvextended_statebatteryDiagnostic Updater
```

Sources: [mavros/src/plugins/sys\_status.cpp893-946](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L893-L946) [mavros/src/plugins/sys\_status.cpp949-960](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L949-L960) [mavros/src/plugins/sys\_status.cpp962-1021](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L962-L1021) [mavros/src/plugins/sys\_status.cpp1023-1037](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L1023-L1037)

## Core Components

### SystemStatusPlugin

The main plugin class that initializes all necessary components, subscribes to MAVLink messages, publishes to ROS topics, and provides services.

Key functionality:

-   Maintains vehicle connection status
-   Processes heartbeat messages to determine armed state
-   Publishes system status information
-   Handles status text messages
-   Manages various diagnostic components

Sources: [mavros/src/plugins/sys\_status.cpp482-626](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L482-L626)

### HeartbeatStatus

Monitors heartbeat message frequency and provides diagnostic information about connection reliability.

Key functionality:

-   Tracks heartbeat frequency
-   Monitors vehicle state (ACTIVE, STANDBY, etc.)
-   Provides diagnostic alerts if heartbeat frequency is too high or too low

Sources: [mavros/src/plugins/sys\_status.cpp65-157](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L65-L157)

### SystemStatusDiag

Processes system status information and provides diagnostics about sensor health.

Key functionality:

-   Monitors onboard sensor health
-   Tracks CPU load
-   Reports communication drop rate and errors

Sources: [mavros/src/plugins/sys\_status.cpp163-270](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L163-L270)

### BatteryStatusDiag

Monitors battery state and provides diagnostics about battery health.

Key functionality:

-   Tracks battery voltage, current, and remaining capacity
-   Monitors individual cell voltages
-   Provides alerts for low voltage conditions

Sources: [mavros/src/plugins/sys\_status.cpp276-359](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L276-L359)

### MemInfo

ArduPilot-specific diagnostic component that monitors memory usage.

Key functionality:

-   Tracks free memory
-   Monitors heap usage
-   Provides alerts for low memory conditions

Sources: [mavros/src/plugins/sys\_status.cpp365-415](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L365-L415)

### HwStatus

ArduPilot-specific diagnostic component that monitors hardware status.

Key functionality:

-   Monitors core voltage
-   Tracks I2C errors
-   Provides alerts for hardware issues

Sources: [mavros/src/plugins/sys\_status.cpp421-473](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L421-L473)

## Published Topics

The System Status Plugin publishes to the following ROS topics:

| Topic | Message Type | Description |
| --- | --- | --- |
| `state` | `mavros_msgs/State` | Connection status, armed state, guided mode, and system status |
| `extended_state` | `mavros_msgs/ExtendedState` | Extended state information including VTOL state and landing state |
| `sys_status` | `mavros_msgs/SysStatus` | System status information, including sensor health and battery status |
| `battery` | `sensor_msgs/BatteryState` | Battery state including voltage, current, and remaining percentage |
| `statustext/recv` | `mavros_msgs/StatusText` | Status text messages from the FCU |
| `status_event` | `mavros_msgs/StatusEvent` | Event messages from the FCU (PX4 specific) |

Sources: [mavros/src/plugins/sys\_status.cpp568-587](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L568-L587)

## Services

The System Status Plugin provides the following ROS services:

| Service | Type | Description |
| --- | --- | --- |
| `set_mode` | `mavros_msgs/SetMode` | Set the FCU flight mode |
| `set_stream_rate` | `mavros_msgs/StreamRate` | Set MAVLink stream rates |
| `set_message_interval` | `mavros_msgs/MessageInterval` | Set the interval for specific MAVLink messages |
| `vehicle_info_get` | `mavros_msgs/VehicleInfoGet` | Get information about the vehicle |

Sources: [mavros/src/plugins/sys\_status.cpp597-615](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L597-L615)

## Diagnostic Information

The System Status Plugin provides extensive diagnostic information through the ROS diagnostic system. This includes:

1.  **Heartbeat Status**: Monitors connection health by tracking heartbeat frequency
2.  **System Status**: Reports sensor health and system load
3.  **Battery Status**: Tracks battery health and alerts on low voltage
4.  **Memory Status** (APM): Monitors available memory
5.  **Hardware Status** (APM): Tracks core voltage and I2C errors

Diagnostics are published to the standard ROS `/diagnostics` topic and can be viewed using tools like `rqt_runtime_monitor`.

Sources: [mavros/src/plugins/sys\_status.cpp526-539](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L526-L539)

## Status Text Processing

The plugin processes status text messages from the flight controller and:

1.  Publishes them to the `statustext/recv` topic
2.  Logs them to the ROS log system with appropriate severity levels

```
STATUSTEXT MessageSeverity MappingPublishSeverity Mappingmaps tomaps tomaps tomaps tomaps tomaps tomaps toMAV_SEVERITY_EMERGENCYRCLCPP_ERRORMAV_SEVERITY_ALERTMAV_SEVERITY_CRITICALMAV_SEVERITY_WARNINGRCLCPP_WARNMAV_SEVERITY_NOTICEMAV_SEVERITY_INFORCLCPP_INFOMAV_SEVERITY_DEBUGRCLCPP_DEBUGFlight Controllerhandle_statustext()ROS Loggerstatustext/recv Topic
```

Sources: [mavros/src/plugins/sys\_status.cpp716-754](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L716-L754) [mavros/src/plugins/sys\_status.cpp1023-1037](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L1023-L1037)

## Battery Monitoring

The plugin provides comprehensive battery monitoring capabilities:

1.  Processes both `SYS_STATUS` and `BATTERY_STATUS` messages
2.  Publishes standard ROS `sensor_msgs/BatteryState` messages
3.  Provides diagnostic information about battery health
4.  Reports per-cell voltage information when available

Battery information is published to the `battery` topic and includes:

-   Voltage
-   Current draw
-   Remaining capacity percentage
-   Individual cell voltages (when available)

Sources: [mavros/src/plugins/sys\_status.cpp963-1021](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L963-L1021)

## Configuration Parameters

The System Status Plugin can be configured using the following ROS parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `conn_timeout` | 10.0 | Connection timeout in seconds |
| `min_voltage` | 10.0 | Minimum battery voltage for diagnostics |
| `disable_diag` | false | Disable diagnostic reporting |
| `heartbeat_mav_type` | "ONBOARD\_CONTROLLER" | MAVLink type to use for heartbeats |
| `heartbeat_rate` | 1.0 | Rate at which to send heartbeats (Hz) |

Sources: [mavros/src/plugins/sys\_status.cpp504-566](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L504-L566)

## Vehicle Information Tracking

The plugin maintains a database of information about vehicles in the network:

1.  Tracks vehicles by system ID and component ID
2.  Stores information from heartbeat and autopilot version messages
3.  Provides a service (`vehicle_info_get`) to retrieve this information

The stored information includes:

-   Vehicle type
-   Autopilot type
-   System status
-   Software versions
-   Capabilities

Sources: [mavros/src/plugins/sys\_status.cpp679-709](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L679-L709) [mavros/src/plugins/sys\_status.cpp890-918](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L890-L918)

The System Status Plugin is closely related to other MAVROS components:

1.  **Command Plugin**: Used for changing vehicle modes and commanding the vehicle
2.  **Parameter Plugin**: Used for configuring the vehicle
3.  **System Time Plugin**: Handles time synchronization
4.  **Companion Process Status Plugin**: For reporting status of companion computer processes

These components work together to provide a comprehensive interface to the vehicle.