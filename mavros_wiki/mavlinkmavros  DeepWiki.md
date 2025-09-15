## MAVROS Overview

Relevant source files

-   [CONTRIBUTING.md](https://github.com/mavlink/mavros/blob/44fa2f90/CONTRIBUTING.md)
-   [README.md](https://github.com/mavlink/mavros/blob/44fa2f90/README.md)
-   [dependencies.rosinstall](https://github.com/mavlink/mavros/blob/44fa2f90/dependencies.rosinstall)
-   [libmavconn/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/CHANGELOG.rst)
-   [libmavconn/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md)
-   [libmavconn/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/package.xml)
-   [mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CHANGELOG.rst)
-   [mavros/CMakeLists.txt](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt)
-   [mavros/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md)
-   [mavros/mavros\_plugins.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros_plugins.xml)
-   [mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml)
-   [mavros/src/plugins/home\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/home_position.cpp)
-   [mavros\_extras/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CHANGELOG.rst)
-   [mavros\_extras/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/README.md)
-   [mavros\_extras/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/package.xml)
-   [mavros\_msgs/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CHANGELOG.rst)
-   [mavros\_msgs/msg/ExtendedState.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/ExtendedState.msg)
-   [mavros\_msgs/msg/HomePosition.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/HomePosition.msg)
-   [mavros\_msgs/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/package.xml)
-   [test\_mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/CHANGELOG.rst)
-   [test\_mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/package.xml)

MAVROS is a MAVLink extendable communication node for ROS (Robot Operating System). It serves as a bridge between ROS and MAVLink-enabled autopilots such as PX4 and ArduPilot. This overview provides a technical introduction to the MAVROS architecture, its key components, and how they interact to enable communication between ROS applications and flight controllers.

For more detailed information about installation and configuration, see [Installation and Setup](https://deepwiki.com/mavlink/mavros/1.2-installation-and-setup).

## System Architecture

MAVROS consists of several key components that work together to provide seamless communication between ROS and MAVLink-enabled devices.

### High-Level Architecture

```
MAVROS EcosystemExternal SystemsPluginsCore ComponentsMAVLinkMAVLinkROS Messages/ServicesFlight Controller UnitGround Control StationROS ApplicationslibmavconnMAVLink Connection Librarymavros_nodeRouter NodeUAS NodePlugin ContainerCore Pluginssys_status, command, param, etc.mavros_msgsMessage Definitions
```

Sources: [mavros/CMakeLists.txt99-118](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L99-L118) [README.md23-43](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L23-L43) [mavros/README.md84-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L84-L99)

### Key Components

1.  **libmavconn**: A standalone MAVLink communication library that handles connection to MAVLink-enabled devices through various transport protocols (serial, UDP, TCP).
    
2.  **Router Node**: Manages connections between endpoints (FCU, GCS, UAS) and routes MAVLink messages between them.
    
3.  **UAS (Unmanned Aircraft System) Node**: Acts as a plugin container, managing all protocol plugins and their interactions.
    
4.  **Plugins**: Modular components that translate between MAVLink messages and ROS topics/services, providing specific functionalities.
    
5.  **mavros\_msgs**: Defines ROS messages and services used by MAVROS.
    
6.  **mavros\_node**: A pre-configured composite node container that integrates Router and UAS components.
    

Sources: [libmavconn/README.md1-27](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L1-L27) [mavros/README.md84-98](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L84-L98) [mavros/CMakeLists.txt100-117](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L100-L117)

## Communication Flow

The communication flow in MAVROS follows a well-defined path, enabling bidirectional exchange between flight controllers and ROS applications.

```
"ROS Application""Plugin""UAS Node""Router Node""Flight Controller""ROS Application""Plugin""UAS Node""Router Node""Flight Controller"MAVLink Message FlowCommand FlowMAVLink MessageMAVLink MessageHandle specific messagePublish ROS message/topicCall ROS serviceCreate MAVLink command messageSend MAVLink messageSend MAVLink messageCommand ACKCommand ACKProcess acknowledgementService response
```

Sources: [mavros/src/plugins/home\_position.cpp108-143](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/home_position.cpp#L108-L143)

### MAVLink Message Flow

1.  The Flight Controller (FCU) sends a MAVLink message.
2.  The Router receives the message and forwards it to the UAS.
3.  The UAS determines which plugin should handle the message based on its type.
4.  The appropriate plugin processes the message and publishes corresponding ROS topics.

### Command Flow

1.  A ROS application calls a service provided by a MAVROS plugin.
2.  The plugin creates a MAVLink command message.
3.  The UAS sends the message to the Router.
4.  The Router sends the message to the FCU.
5.  The FCU sends an acknowledgment back through the same path.
6.  The plugin processes the acknowledgment and returns a service response.

## Plugin System

MAVROS uses a plugin-based architecture to provide modular functionality. Each plugin handles specific MAVLink message types and exposes corresponding ROS topics and services.

```
Extra PluginsCore PluginsPlugin Base SystemPlugin Base ClassTF2ListenerMixinSetpointMixinSystemStatusPluginCommandPluginParameterPluginLocalPositionPluginGlobalPositionPluginIMUPluginSetpointPositionPluginSetpointAttitudePluginWaypointPluginVisionPoseEstimatePluginDistanceSensorPluginMocapPoseEstimatePluginFakeGPSPluginObstacleDistancePlugin
```

Sources: [mavros/mavros\_plugins.xml1-142](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros_plugins.xml#L1-L142) [mavros/CMakeLists.txt137-167](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L137-L167)

### Core Plugins

MAVROS includes a set of core plugins that provide essential functionality:

| Plugin Name | Description |
| --- | --- |
| sys\_status | System status information, battery status, state monitoring |
| command | Interface for sending commands to the flight controller |
| param | Parameter manipulation (get/set flight controller parameters) |
| waypoint | Mission planning and management |
| global\_position | Global position handling (GPS) |
| local\_position | Local position in ENU frame |
| imu | IMU data handling and conversion |
| setpoint\_position | Position setpoint control |
| setpoint\_velocity | Velocity setpoint control |
| setpoint\_attitude | Attitude setpoint control |
| home\_position | Home position handling |

Sources: [mavros/mavros\_plugins.xml15-139](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros_plugins.xml#L15-L139)

Additional functionality is provided through the mavros\_extras package, which includes plugins for:

-   Vision pose estimation
-   Distance sensors
-   Motion capture
-   Fake GPS
-   Obstacle avoidance
-   Camera control
-   Terrain height estimation
-   And many more

Sources: [mavros\_extras/README.md1-163](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/README.md#L1-L163)

## Connection System

MAVROS supports various connection types to communicate with flight controllers and ground control stations.

```
Router SystemTransport ImplementationsMAVConnInterfaceURL SystemMAVLink Connection URLMAVConnInterfaceMAVConnSerialMAVConnUDPMAVConnTCPClientMAVConnTCPServerRouter NodeMAVConnEndpointROSEndpoint
```

Sources: [libmavconn/README.md9-26](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L9-L26) [mavros/README.md31-55](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L31-L55)

### Connection URL Formats

Connections are defined by URLs with specific formats:

| Type | URL Format |
| --- | --- |
| Serial | `/path/to/serial/device[:baudrate]` |
| Serial | `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]` |
| Serial with hardware flow control | `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]` |
| UDP | `udp://[bind_host][:port]@[remote_host[:port]][/?ids=sysid,compid]` |
| UDP broadcast | `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]` |
| UDP permanent broadcast | `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]` |
| TCP client | `tcp://[server_host][:port][/?ids=sysid,compid]` |
| TCP server | `tcp-l://[bind_host][:port][/?ids=sysid,compid]` |

Notes:

-   `bind_host` defaults to `0.0.0.0` (any interface)
-   UDP default ports: 14555 @ 14550
-   TCP default port: 5760
-   The `ids` parameter overrides system\_id and component\_id parameters

Sources: [mavros/README.md31-55](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L31-L55) [libmavconn/README.md9-26](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L9-L26)

## Coordinate Frame Transformations

MAVROS translates between different coordinate frames used by MAVLink and ROS.

```
Frame Transformation Functionstransform_frame_ned_enutransform_frame_enu_ecefGeographicLib Conversionstransform_frame_aircraft_baselinkCoordinate FramesNED_ENUENU_ECEFECEF_LLAAIRCRAFT_BASELINKNEDNorth-East-DownENUEast-North-UpECEFEarth-Centered Earth-FixedLLALatitude-Longitude-AltitudeAircraft FrameForward-Right-DownBaselink FrameForward-Left-Up
```

Sources: [mavros/README.md56-82](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L56-L82) [mavros/CMakeLists.txt104-107](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L104-L107)

### Key Transformations

MAVROS applies specific transformations to convert between coordinate frames:

1.  **Airframe Transformations**: 180° rotation about ROLL (X) axis to convert between aircraft frame and ROS base\_link frame.
    
2.  **Local Position Transformations**: 180° rotation about ROLL (X) and 90° rotation about YAW (Z) axes to convert between NED (North-East-Down) and ENU (East-North-Up) frames.
    
3.  **Geographic Conversions**:
    
    -   Conversion between geodetic coordinates (latitude, longitude, altitude) and ECEF (Earth-Centered, Earth-Fixed)
    -   Conversion between AMSL (Above Mean Sea Level) and WGS-84 ellipsoid height

These transformations use GeographicLib, which requires specific datasets to be installed. The package includes a script (`install_geographiclib_datasets.sh`) to install these datasets.

Sources: [mavros/README.md56-82](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L56-L82) [mavros/src/plugins/home\_position.cpp117-136](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/home_position.cpp#L117-L136)

## Composite Node Structure

MAVROS uses ROS2 composition to create a flexible and modular system.

### mavros::router::Router

The Router node manages connections to FCUs, GCSs, and UAS nodes. It allows adding and removing endpoints dynamically without restarting the node.

### mavros::uas::UAS

The UAS node is a plugin container that manages all protocol plugins. Each plugin operates as a subnode to the UAS node.

### mavros\_node

This is a pre-configured composite node container that integrates the Router and UAS components, providing similar parameters as the ROS1 mavros\_node. It configures the components to work together by setting up the necessary connections.

Sources: [mavros/README.md84-109](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L84-L109) [mavros/CMakeLists.txt198-199](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L198-L199)

## Summary

MAVROS provides a comprehensive bridge between ROS and MAVLink-enabled autopilots. Its modular plugin-based architecture allows for extensibility and customization. The system handles coordinate frame transformations, parameter management, mission planning, and various other functionalities required for controlling and monitoring unmanned aircraft systems.

The core components (libmavconn, Router, UAS) work together to establish communication between ROS applications and flight controllers, while the plugin system provides specific functionalities exposed as ROS topics and services.

Sources: [mavros/package.xml1-20](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L1-L20) [README.md1-20](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L1-L20)