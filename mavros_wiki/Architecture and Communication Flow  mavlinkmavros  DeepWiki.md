Relevant source files

-   [libmavconn/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/CHANGELOG.rst)
-   [libmavconn/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/package.xml)
-   [mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/CHANGELOG.rst)
-   [mavros/include/mavros/mavros\_router.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp)
-   [mavros/include/mavros/mavros\_uas.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp)
-   [mavros/include/mavros/plugin.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp)
-   [mavros/include/mavros/plugin\_filter.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp)
-   [mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml)
-   [mavros/src/lib/mavros\_router.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp)
-   [mavros/src/lib/mavros\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp)
-   [mavros/src/lib/plugin.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp)
-   [mavros/src/lib/uas\_ap.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_ap.cpp)
-   [mavros/src/mavros\_node.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp)
-   [mavros/test/test\_router.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/test/test_router.cpp)
-   [mavros/test/test\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/test/test_uas.cpp)
-   [mavros\_extras/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CHANGELOG.rst)
-   [mavros\_extras/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/package.xml)
-   [mavros\_msgs/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CHANGELOG.rst)
-   [mavros\_msgs/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/package.xml)
-   [mavros\_msgs/srv/EndpointAdd.srv](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/srv/EndpointAdd.srv)
-   [mavros\_msgs/srv/EndpointDel.srv](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/srv/EndpointDel.srv)
-   [test\_mavros/CHANGELOG.rst](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/CHANGELOG.rst)
-   [test\_mavros/package.xml](https://github.com/mavlink/mavros/blob/44fa2f90/test_mavros/package.xml)

This page describes the high-level architecture of MAVROS and how communication flows between its components. MAVROS serves as a communication bridge between ROS2 (Robot Operating System) and MAVLink-based flight controllers, enabling seamless integration of drone autopilots with ROS applications. For information about specific plugins and their functionality, see [Plugin System](https://deepwiki.com/mavlink/mavros/3-plugin-system).

## System Overview

MAVROS has a modular architecture that consists of three primary components: the Router node, the UAS (Unmanned Aircraft System) node, and the Plugin system. The Router handles communication with external MAVLink devices, the UAS node manages plugins and provides common functionality, and the Plugins implement specific features by converting between MAVLink and ROS messages.

```
MAVROS EcosystemExternal SystemsPluginsCore ComponentsMAVLink MessagesMAVLink MessagesROS Messages/ServicesFlight Controller UnitGround Control StationROS ApplicationslibmavconnMAVLink Connection LibraryRouter Nodemavros::router::RouterUAS Nodemavros::uas::UASmavros_nodeContainerCore Pluginssys_status, command, param, etc.mavros_msgsMessage Definitions
```

Sources:

-   [mavros/src/mavros\_node.cpp26-108](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp#L26-L108)
-   [mavros/include/mavros/mavros\_router.hpp134-187](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L134-L187)
-   [mavros/include/mavros/mavros\_uas.hpp233-268](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L233-L268)
-   [mavros/package.xml5-8](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L5-L8)

## Core Components

### Router Node

The Router (`mavros::router::Router`) manages connections to MAVLink devices and routes messages between them based on system and component IDs. It's responsible for:

1.  Creating and managing endpoints to communicate with FCUs, GCSs, and UAS nodes
2.  Routing messages between endpoints according to specific traffic rules
3.  Keeping track of known remote devices (through their MAVLink addresses)
4.  Handling connection/disconnection and reconnection attempts

The Router creates three types of endpoints (defined in `Endpoint::Type`):

-   FCU (Flight Controller Unit) - connects to autopilots
-   GCS (Ground Control Station) - connects to control software
-   UAS (Unmanned Aircraft System) - connects to MAVROS UAS node

Each endpoint can connect to multiple remote systems, which is particularly useful for mesh networks or swarms.

```
Endpoint PropertiesEndpoint TypesRouter ComponentsRouter Classmavros::router::Routerendpoints: Mapadd_endpoint()del_endpoint()route_message()EndpointBase ClassMAVConnEndpointFor FCU/GCSROSEndpointFor UASid: uint32_tlink_type: Typeurl: stringremote_addrs: set
```

Sources:

-   [mavros/include/mavros/mavros\_router.hpp134-227](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L134-L227)
-   [mavros/src/lib/mavros\_router.cpp36-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L36-L99) (route\_message implementation)
-   [mavros/src/lib/mavros\_router.cpp101-150](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L101-L150) (add\_endpoint implementation)
-   [mavros/src/lib/mavros\_router.cpp152-185](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L152-L185) (del\_endpoint implementation)

### UAS Node

The UAS (Unmanned Aircraft System) Node (`mavros::uas::UAS`) serves as the main interface between MAVLink and ROS. It's responsible for:

1.  Loading and managing plugins
2.  Distributing MAVLink messages to appropriate plugins
3.  Maintaining system state information (e.g., connection status, autopilot type)
4.  Providing coordinate frame transformations
5.  Managing time synchronization

The UAS class maintains information about the target system and component IDs, which are used to identify the specific autopilot the node communicates with. It also provides a data store (`mavros::uas::Data`) for sharing information between plugins.

```
Plugin SystemUAS MethodsUAS Class ComponentsUAS Classmavros::uas::UASdata: DataIMU, GPS infotf2_buffer, tf2_broadcasterCoordinate transformsloaded_plugins: Maptarget_system: uint8_ttarget_component: uint8_tconnected: booladd_plugin()is_my_target()update_connection_status()update_heartbeat()send_message()plugin_factory_loaderplugin_subscriptionsplugin_msg_cb()
```

Sources:

-   [mavros/include/mavros/mavros\_uas.hpp233-353](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L233-L353) (UAS class definition)
-   [mavros/src/lib/mavros\_uas.cpp34-65](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L34-L65) (UAS constructor)
-   [mavros/src/lib/uas\_ap.cpp27-32](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_ap.cpp#L27-L32) (update\_heartbeat implementation)
-   [mavros/include/mavros/mavros\_uas.hpp78-215](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L78-L215) (Data class definition)

### Plugin System

The Plugin system extends MAVROS functionality through a modular architecture. Plugins are derived from the `mavros::plugin::Plugin` base class and implement specific features by:

1.  Subscribing to specific MAVLink messages
2.  Converting between MAVLink and ROS message formats
3.  Publishing ROS topics and providing ROS services
4.  Sending MAVLink commands to the autopilot

Each plugin must implement the `get_subscriptions()` method, which returns a list of message handlers for the MAVLink messages the plugin is interested in. Plugins can use different message filters (defined in `mavros::plugin::filter`) to process only specific messages.

```
Plugin ImplementationMessage FiltersPlugin Base SystemPlugin Base Classmavros::plugin::Pluginmake_handler()Creates message subscriptionsget_subscriptions()Abstract methodconnection_cb()Called on connection changesAnyOkAny valid messageSystemAndOkMessages for target systemComponentAndOkMessages for target componentSystemStatusPluginCommandPluginParameterPluginLocalPositionPluginGlobalPositionPlugin
```

Sources:

-   [mavros/include/mavros/plugin.hpp62-116](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L62-L116) (Plugin base class)
-   [mavros/src/lib/plugin.cpp21-66](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp#L21-L66) (Plugin implementation)
-   [mavros/include/mavros/plugin\_filter.hpp40-76](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp#L40-L76) (Message filters)

## Communication Flow

The overall communication flow in MAVROS passes through multiple components, with the Router handling external communication and the UAS node distributing messages to plugins.

### Message Routing

```
"ROS Application""Plugin""UAS Node""Router Node""Flight Controller""ROS Application""Plugin""UAS Node""Router Node""Flight Controller"MAVLink Message Flow (FCU to ROS)ROS Command Flow (ROS to FCU)MAVLink Messagemavlink_message_tProcess message via handlerPublish ROS message/topicCall ROS serviceCreate MAVLink commandSend MAVLink messageMAVLink CommandCommand ACKMAVLink ACK messageProcess acknowledgementService response
```

Sources:

-   [mavros/src/lib/mavros\_router.cpp36-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L36-L99) (route\_message implementation)
-   [mavros/include/mavros/plugin.hpp143-177](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L143-L177) (message handler implementation)

### Routing Decision Process

The Router Node uses a set of rules to determine which endpoints should receive each message. The routing logic in `route_message()` examines:

1.  The source endpoint (to avoid echoing messages back)
2.  The link type (to maintain traffic separation between endpoint types)
3.  The target system and component IDs in the message

The main routing rules are:

1.  FCU broadcasts → GCS, UAS (not to other FCUs)
2.  FCU targeted messages → GCS/UAS with matching address
3.  GCS broadcasts → FCU, UAS
4.  UAS broadcasts → FCU, GCS
5.  Messages are never echoed back to their source endpoint
6.  Messages are not routed between endpoints of the same type

Sources:

-   [mavros/src/lib/mavros\_router.cpp36-99](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L36-L99) (route\_message implementation)
-   [mavros/include/mavros/mavros\_router.hpp120-133](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L120-L133) (Router class comment with rules)

## Connection System

MAVROS uses a URL-based connection system provided by the `libmavconn` library to establish links with MAVLink devices.

### Endpoint Types and URLs

```
Router EndpointsTransport ImplementationsConnection InterfaceURL Systemroute_message()MAVLink Connection URLMAVConnInterfaceopen_url()MAVConnSerialserial:///dev/ttyACM0:57600MAVConnSerial+HW Flow Controlserial-hwfc:///dev/ttyAMA0:57600MAVConnUDPudp://192.168.1.1:14550@14555MAVConnUDP Broadcastudp-b://192.168.255.255:14550@:14555MAVConnTCPClienttcp://localhost:5760MAVConnTCPServertcp-l://0.0.0.0:5760MAVConnEndpointFor FCU/GCS connectionsROSEndpointFor UAS connections
```

Source:

-   [libmavconn/package.xml6-12](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/package.xml#L6-L12) (libmavconn description)
-   [mavros/include/mavros/mavros\_router.hpp69-114](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp#L69-L114) (Endpoint base class)
-   [mavros/src/lib/mavros\_router.cpp368-409](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_router.cpp#L368-L409) (MAVConnEndpoint implementation)

### System Startup and Initialization

When MAVROS starts up, the following initialization sequence occurs:

1.  The `mavros_node` container creates:
    
    -   A `Router` node
    -   A `UAS` node
2.  The Router is configured with:
    
    -   FCU URL (connection to flight controller)
    -   GCS URL (connection to ground control, if any)
    -   UAS URL (internal connection to the UAS node)
3.  The UAS node:
    
    -   Establishes connection with the Router through ROSEndpoint
    -   Loads all available plugins
    -   Sets up TF2 transformations
    -   Configures parameters for targeting specific system/component IDs

```
Runtime OperationInitialization Processmain() in mavros_node.cppCreate Router nodeCreate UAS nodeConfigure Router with FCU/GCS/UAS URLsConfigure UAS with target IDsUAS loads pluginsUAS connects to RouterRouter routes MAVLink messagesUAS processes messagesPlugins provide ROS interfaces
```

Sources:

-   [mavros/src/mavros\_node.cpp26-108](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp#L26-L108) (main function implementation)
-   [mavros/src/lib/mavros\_uas.cpp86-160](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L86-L160) (UAS initialization)

## Plugin Message Processing

### Plugin Subscription Mechanism

MAVROS uses a subscription system to route MAVLink messages to the appropriate plugin handlers:

1.  Each plugin implements the `get_subscriptions()` method, returning a list of message handlers
2.  Handlers are registered with the UAS node during plugin initialization
3.  When a message arrives, the UAS node:
    -   Looks up handlers for the message ID
    -   Calls each handler, possibly with automatic message decoding
    -   Handlers can use filters to only process certain messages

```
Message FilteringPlugin Handler CreationUAS Message ProcessingUAS receive_message()Find handlers for message IDCall plugin handlersPlugin::get_subscriptions()make_handler()HandlerInfo tupleRaw Message HandlerDecoding HandlerMessage Filter(AnyOk, SystemAndOk, etc.)
```

Sources:

-   [mavros/include/mavros/plugin.hpp128-177](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L128-L177) (make\_handler implementations)
-   [mavros/include/mavros/plugin\_filter.hpp40-76](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp#L40-L76) (Message filters)

## Code-to-Concept Relationship Map

The following diagram maps major MAVROS concepts to their corresponding code entities, helping bridge between the conceptual understanding and the actual implementation:

```
Key Methods/FunctionsImplementation ClassesMAVROS Architecture ConceptsMessage RouterUAS/Plugin ContainerPlugin SystemConnection EndpointsCoordinate Transformationsmavros::router::Routermavros_router.hppmavros::uas::UASmavros_uas.hppmavros::plugin::Pluginplugin.hppmavros::router::Endpointmavros_router.hppmavros::uas::frame_tfframe_tf.hppRouter::route_message()mavros_router.cppUAS::add_plugin()mavros_uas.cppPlugin::get_subscriptions()plugin.hppPlugin::make_handler()plugin.hpptransform_frame_ned_enu()frame_tf.hpp
```

Sources:

-   [mavros/include/mavros/mavros\_router.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp)
-   [mavros/include/mavros/mavros\_uas.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp)
-   [mavros/include/mavros/plugin.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp)
-   [mavros/include/mavros/frame\_tf.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/frame_tf.hpp)

## Summary

MAVROS architecture consists of three primary components:

1.  **Router Node** - Manages connections to MAVLink devices and routes messages between them
2.  **UAS Node** - Loads plugins, distributes messages, and provides common functionality
3.  **Plugin System** - Implements specific features by converting between MAVLink and ROS messages

The communication flow follows these steps:

1.  External MAVLink messages enter through the Router's endpoints
2.  The Router forwards messages to the UAS node
3.  The UAS node dispatches messages to appropriate plugins
4.  Plugins convert MAVLink messages to ROS topics/services
5.  For outgoing messages, the process is reversed

This architecture provides a flexible, extensible framework for integrating MAVLink-based autopilots with ROS applications.

Sources:

-   [mavros/package.xml5-8](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/package.xml#L5-L8)
-   [mavros/include/mavros/mavros\_router.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_router.hpp)
-   [mavros/include/mavros/mavros\_uas.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp)
-   [mavros/include/mavros/plugin.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp)