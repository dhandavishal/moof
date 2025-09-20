Relevant source files

-   [CONTRIBUTING.md](https://github.com/mavlink/mavros/blob/44fa2f90/CONTRIBUTING.md)
-   [README.md](https://github.com/mavlink/mavros/blob/44fa2f90/README.md)
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
-   [mavros\_extras/CMakeLists.txt](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt)
-   [mavros\_extras/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/README.md)
-   [mavros\_extras/mavros\_plugins.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml)
-   [mavros\_extras/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/package.xml)
-   [mavros\_extras/src/plugins/onboard\_computer\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/onboard_computer_status.cpp)
-   [mavros\_msgs/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CHANGELOG.rst)
-   [mavros\_msgs/CMakeLists.txt](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CMakeLists.txt)
-   [mavros\_msgs/msg/ExtendedState.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/ExtendedState.msg)
-   [mavros\_msgs/msg/HomePosition.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/HomePosition.msg)
-   [mavros\_msgs/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/package.xml)
-   [test\_mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/CHANGELOG.rst)
-   [test\_mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/package.xml)

This page describes the organization of packages within the MAVROS system and their relationships. It provides a high-level overview of how the codebase is structured, helping developers understand the various components and their purposes.

## Overview of MAVROS Packages

MAVROS is divided into several interconnected packages, each with a specific purpose within the overall system architecture.

```
MAVROS EcosystemmavrosCore functionalitylibmavconnMAVLink connection librarymavros_msgsMessage definitionstest_mavrosTesting tools
```

Sources: [README.md23-47](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L23-L47) [mavros/package.xml2-94](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L2-L94) [libmavconn/package.xml2-41](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/package.xml#L2-L41) [mavros\_extras/package.xml2-93](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/package.xml#L2-L93) [mavros\_msgs/package.xml2-46](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/package.xml#L2-L46)

## Package Descriptions

### mavros

The main package containing core functionality including the main node and standard plugins. It serves as the bridge between ROS and MAVLink-based autopilots like PX4 and ArduPilot.

Key components:

-   Core plugins for essential functionality
-   MAVLink-to-ROS message conversion
-   Coordinate frame transformations
-   Parameter and waypoint handling
-   Main node implementation

### libmavconn

A standalone MAVLink connection library that handles communication with MAVLink-enabled devices via different transport methods.

Key features:

-   Connection handling via Serial, UDP, and TCP
-   URL-based connection configuration
-   MAVLink message framing and parsing
-   Thread-safe communication

### mavros\_msgs

Contains all message and service definitions used by MAVROS, enabling communication between different ROS nodes and MAVLink systems.

Includes:

-   ROS2 messages matching MAVLink messages
-   Service definitions for commands and queries
-   Custom data types for MAVROS operations

Contains additional plugins that extend the core functionality of MAVROS with specialized features.

Features:

-   Vision-related plugins
-   Sensor interface plugins
-   Advanced control plugins
-   Debug utilities

### test\_mavros

Contains test tools and utilities for testing MAVROS with various autopilots.

## Package Dependencies

The following table summarizes the key dependencies of each package:

| Package | Primary Dependencies |
| --- | --- |
| libmavconn | asio, mavlink, console-bridge-dev |
| mavros\_msgs | rcl\_interfaces, geographic\_msgs, geometry\_msgs, sensor\_msgs |
| mavros | libmavconn, mavros\_msgs, rclcpp, rclcpp\_components, tf2\_ros, diagnostic\_updater |
| mavros\_extras | mavros, libmavconn, mavros\_msgs, urdf, yaml-cpp |
| test\_mavros | mavros, mavros\_extras |

Sources: [mavros/package.xml20-76](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L20-L76) [libmavconn/package.xml27-35](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/package.xml#L27-L35) [mavros\_msgs/package.xml22-38](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/package.xml#L22-L38) [mavros\_extras/package.xml22-73](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/package.xml#L22-L73)

## Build System

MAVROS uses a standard ROS2 build system with CMake. The build settings are defined in each package's `CMakeLists.txt` file.

Key build configuration:

-   C++17/C++20 standard
-   Inclusion of GeographicLib for coordinate transformations
-   Plugin definitions for discovery
-   Component registration for composable nodes

Sources: [mavros/CMakeLists.txt1-20](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L1-L20) [mavros\_extras/CMakeLists.txt1-20](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L1-L20)

## Core Components Structure

The following diagram illustrates the main architectural components and their relationships:

```
Connection SystemExternal ComponentsMAVROS Core ComponentsMAVLink messagesLoads and managesMAVLinkMAVLinkMAVLink messagesROS messages/servicesROS messages/servicesmavros::router::RouterMessage routing between endpointsmavros::uas::UASPlugin container and state managementPlugin SystemExtensible functionalityFlight Controller(PX4, ArduPilot, etc)Ground Control StationROS2 Nodes/ApplicationslibmavconnConnection interfacesMAVConn Endpoints(Serial, UDP, TCP)
```

Sources: [mavros/CMakeLists.txt100-117](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L100-L117) [mavros/CMakeLists.txt135](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L135-L135) [mavros/README.md89-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L89-L99)

## Plugin System

MAVROS is built on a flexible plugin architecture where functionality is modularized into plugins. These plugins are loaded by the UAS (Unmanned Aircraft System) node.

### Plugin Organization

```
Extra Plugins (mavros_extras)Core Plugins (mavros)Plugin BasePlugin Base Classmavros::plugin::PluginSystemStatusPluginCommandPluginParamPluginLocalPositionPluginGlobalPositionPluginSetpointPositionPluginSetpointAttitudePluginetc.WaypointPluginVisionPoseEstimatePluginDistanceSensorPluginFakeGPSPluginObstacleDistancePluginCameraPluginGimbalControlPlugin
```

Sources: [mavros/mavros\_plugins.xml7-140](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros_plugins.xml#L7-L140) [mavros\_extras/mavros\_plugins.xml7-140](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L7-L140) [mavros/CMakeLists.txt137-196](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L137-L196) [mavros\_extras/CMakeLists.txt91-157](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L91-L157)

### Plugin Registration

Plugins are registered using the pluginlib library in the respective CMakeLists.txt files:

1.  For core plugins: `pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)`
2.  For extra plugins: `pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)`

Each plugin is defined in the XML file with its class name, type, and description.

Sources: [mavros/CMakeLists.txt196](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L196-L196) [mavros\_extras/CMakeLists.txt157](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L157-L157)

## Composable Node Architecture

MAVROS uses ROS2 composable nodes for better performance and flexibility.

### Main Components

```
ConnectionsROS2 Component ContainerLoadsLoadsConnects toMAVLinkMAVLinkROS2mavros_nodeContainer nodemavros::router::RouterComponentmavros::uas::UASComponentFlight ControllerGround Control StationROS2 Applications
```

Sources: [mavros/CMakeLists.txt198-199](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CMakeLists.txt#L198-L199) [mavros/README.md84-106](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L84-L106)

MAVROS registers two main components:

-   `mavros::router::Router`: Handles MAVLink message routing
-   `mavros::uas::UAS`: Acts as a plugin container

The `mavros_node` is a pre-configured component container that loads these components.

## Message and Service System

The `mavros_msgs` package defines all the messages and services used within MAVROS.

Key message categories:

-   Command messages for controlling the vehicle
-   Status messages for vehicle information
-   Parameter messages for configuration
-   Navigation and waypoint messages
-   Sensor data messages

Sources: [mavros\_msgs/CMakeLists.txt30-107](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CMakeLists.txt#L30-L107) [mavros\_msgs/CMakeLists.txt111-161](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CMakeLists.txt#L111-L161)

## Summary

The MAVROS package structure follows a modular design with several packages that handle different aspects of MAVLink-ROS communication. The core functionality is in the `mavros` package, with the communication library in `libmavconn`, message definitions in `mavros_msgs`, and additional functionality in `mavros_extras`. The system uses a plugin architecture for extensibility and ROS2 composable nodes for flexibility and performance.