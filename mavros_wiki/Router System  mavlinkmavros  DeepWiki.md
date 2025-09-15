Relevant source files

-   [mavros/include/mavros/mavros\_router.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp)
-   [mavros/include/mavros/mavros\_uas.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp)
-   [mavros/include/mavros/plugin.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp)
-   [mavros/include/mavros/plugin\_filter.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp)
-   [mavros/src/lib/mavros\_router.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp)
-   [mavros/src/lib/mavros\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp)
-   [mavros/src/lib/plugin.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp)
-   [mavros/src/lib/uas\_ap.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_ap.cpp)
-   [mavros/src/mavros\_node.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp)
-   [mavros/test/test\_router.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/test/test_router.cpp)
-   [mavros/test/test\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/test/test_uas.cpp)
-   [mavros\_msgs/srv/EndpointAdd.srv](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/srv/EndpointAdd.srv)
-   [mavros\_msgs/srv/EndpointDel.srv](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/srv/EndpointDel.srv)

The Router System in MAVROS is responsible for managing MAVLink message routing between different endpoints. It acts as a central communication hub, directing messages between Flight Controller Units (FCUs), Ground Control Stations (GCS), and MAVROS UAS nodes. This document explains the architecture, functionality, and configuration of the Router System.

For information about connection URLs and formats, see [Connection System](https://deepwiki.com/mavlink/mavros/2.3-connection-system). For information on the UAS node that interacts with the router, see [UAS Node](https://deepwiki.com/mavlink/mavros/2.1-uas-node).

## Architecture Overview

The Router System consists of a central `Router` class that manages multiple endpoints. Each endpoint represents a communication channel with a specific device type (FCU, GCS, or UAS node).

```
managesimplementsimplements1manyRouter+endpoints: Map+route_message()+add_endpoint()+del_endpoint()+periodic_reconnect_endpoints()+periodic_clear_stale_remote_addrs()«abstract»Endpoint+id: uint32_t+link_type: Type+url: string+remote_addrs: set+stale_addrs: set+is_open()+open()+close()+send_message()+recv_message()MAVConnEndpoint+link: MAVConnInterface+is_open()+open()+close()+send_message()+diag_run()ROSEndpoint+source: Publisher+sink: Subscription+is_open()+open()+close()+send_message()+ros_recv_message()+diag_run()
```

Sources: [mavros/include/mavros/mavros\_router.hpp69-300](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L69-L300) [mavros/src/lib/mavros\_router.cpp29-548](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L29-L548)

## Endpoint Types

The Router supports three types of endpoints, each serving a specific role in the MAVLink communication system:

| Endpoint Type | Implementation | Description | Connection Method |
| --- | --- | --- | --- |
| FCU (Flight Controller Unit) | MAVConnEndpoint | Connects to autopilot firmware | MAVLink via libmavconn (Serial, UDP, TCP) |
| GCS (Ground Control Station) | MAVConnEndpoint | Connects to control software | MAVLink via libmavconn (UDP, TCP) |
| UAS (Unmanned Aircraft System) | ROSEndpoint | Connects MAVROS UAS nodes | ROS topics (mavlink\_source, mavlink\_sink) |

```
External SystemsRouter SystemEndpoint TypesRouter NodeFCU EndpointsGCS EndpointsUAS EndpointsFlight Controller UnitsGround Control StationsUAS Nodes (Plugin Containers)
```

Sources: [mavros/include/mavros/mavros\_router.hpp74-79](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L74-L79) [mavros/include/mavros/mavros\_router.hpp238-300](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L238-L300)

## Message Routing

The core functionality of the Router is routing MAVLink messages between endpoints. The routing is based on the source endpoint type, target address in the message, and the registered addresses of destination endpoints.

### Addressing System

The Router uses a 32-bit address (addr\_t) composed of:

-   High byte (8 bits): System ID
-   Low byte (8 bits): Component ID

This allows targeting specific systems and components in the MAVLink network.

### Routing Rules

```
YesNoYesNoYesNoRetry < 2Retry >= 2Message Received from Source EndpointExtract Target Address from MessageIs Source = Destination?Are Source and Destination of Same Type?Find Endpoints with Matching Target AddressTargets Found?Send Message to Target EndpointsSet Target to BroadcastRetry with Broadcast AddressDrop Message
```

The router implements the following routing rules:

1.  **FCU broadcast**: Messages sent to all GCS and UAS endpoints, but not to other FCUs
2.  **FCU targeted**: Messages sent to GCS/UAS endpoints with matching address
3.  **GCS broadcast**: Messages sent to all FCU and UAS endpoints
4.  **GCS targeted**: Messages sent to FCU/UAS endpoints with matching address
5.  **UAS broadcast**: Messages sent to all FCU and GCS endpoints
6.  **UAS targeted**: Messages sent to FCU/GCS endpoints with matching address

The Router also implements an automatic retry mechanism. If a message's target is not found, it retries by treating the message as a broadcast message.

Sources: [mavros/include/mavros/mavros\_router.hpp116-133](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L116-L133) [mavros/src/lib/mavros\_router.cpp36-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L36-L99)

## Message Flow Example

```
UAS NodeROSEndpointRouterMAVConnEndpointFlight ControllerUAS NodeROSEndpointRouterMAVConnEndpointFlight ControllerCommand Flow (GCS to FCU)Telemetry Flow (FCU to UAS)mavlink_sink (SET_MODE)ros_recv_message()recv_message()route_message()send_message()MAVLink MessageMAVLink Message (HEARTBEAT)recv_message()recv_message()route_message()send_message()mavlink_source
```

Sources: [mavros/src/lib/mavros\_router.cpp36-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L36-L99) [mavros/src/lib/mavros\_router.cpp337-543](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L337-L543)

## Address Tracking

Each endpoint maintains two sets of addresses:

-   `remote_addrs`: Addresses that have been seen on this endpoint
-   `stale_addrs`: Temporary storage for tracking stale addresses

The Router periodically checks for stale addresses and removes them from the `remote_addrs` set. This ensures that the router only routes messages to active endpoints.

```
Address Tracking ProcessEndpoint.recv_message()Add Source Address to remote_addrsRemove Address from stale_addrsRouter.periodic_clear_stale_remote_addrs()Remove stale_addrs from remote_addrsCopy remote_addrs to stale_addrs
```

Sources: [mavros/src/lib/mavros\_router.cpp337-361](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L337-L361) [mavros/src/lib/mavros\_router.cpp292-316](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L292-L316)

## Configuration and Management

The Router can be configured through ROS parameters and services.

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `fcu_urls` | string\[\] | List of URLs for FCU endpoints |
| `gcs_urls` | string\[\] | List of URLs for GCS endpoints |
| `uas_urls` | string\[\] | List of URLs for UAS endpoints |

### Services

| Service | Description |
| --- | --- |
| `~/add_endpoint` | Adds a new endpoint to the router |
| `~/del_endpoint` | Removes an endpoint from the router |

### Automatic Management

The Router also provides functionality for automatic management of endpoints:

-   `periodic_reconnect_endpoints()`: Periodically attempts to reconnect closed endpoints
-   `periodic_clear_stale_remote_addrs()`: Periodically cleans up stale remote addresses

Sources: [mavros/src/lib/mavros\_router.cpp101-150](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L101-L150) [mavros/src/lib/mavros\_router.cpp152-185](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L152-L185) [mavros/src/lib/mavros\_router.cpp187-265](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L187-L265)

## Integration with MAVROS

The Router is integrated into the MAVROS system as follows:

```
ExternalMAVROS SystemUAS NodeRouter NodeRouterFCU EndpointsGCS EndpointsUAS EndpointsUASPluginsFlight ControllersGround Stations
```

The MAVROS Node initializes both the Router and UAS components:

1.  Creates a Router Node with configured FCU and GCS URLs
2.  Sets up a UAS endpoint in the Router for communication with the UAS Node
3.  Creates a UAS Node and connects it to the specified UAS endpoint

Sources: [mavros/src/mavros\_node.cpp26-108](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp#L26-L108)

## Diagnostics and Monitoring

The Router provides diagnostic information for monitoring its status.

### Statistics

The Router maintains several statistics counters:

-   `stat_msg_routed`: Number of messages received for routing
-   `stat_msg_sent`: Number of messages successfully sent to destinations
-   `stat_msg_dropped`: Number of messages dropped (no destination found)

Each endpoint also provides diagnostics about its connection status, including:

-   Remote addresses detected
-   For MAVConnEndpoint: received packets, dropped packets, buffer overruns, etc.
-   For ROSEndpoint: basic connection status

This information is published through the ROS diagnostic system and can be viewed using diagnostic tools like `rqt_runtime_monitor`.

Sources: [mavros/src/lib/mavros\_router.cpp318-335](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L318-L335) [mavros/src/lib/mavros\_router.cpp420-457](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L420-L457) [mavros/src/lib/mavros\_router.cpp533-548](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L533-L548)

## Summary

The Router System provides a flexible and powerful message routing mechanism for MAVROS. It routes MAVLink messages between different types of endpoints (FCU, GCS, UAS) based on well-defined rules. The Router can be configured through ROS parameters and services, and it provides diagnostic information for monitoring its status.