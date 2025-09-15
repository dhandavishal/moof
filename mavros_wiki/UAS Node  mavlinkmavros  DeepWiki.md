Relevant source files

-   [.editorconfig](https://github.com/mavlink/mavros/blob/44fa2f90/.editorconfig)
-   [mavros/include/mavros/mavros\_uas.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp)
-   [mavros/include/mavros/plugin.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp)
-   [mavros/include/mavros/plugin\_filter.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp)
-   [mavros/src/lib/mavros\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp)
-   [mavros/src/lib/plugin.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp)
-   [mavros/src/lib/uas\_ap.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_ap.cpp)
-   [mavros/src/lib/uas\_data.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_data.cpp)
-   [mavros/src/mavros\_node.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp)
-   [mavros/test/test\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/test/test_uas.cpp)

The UAS (Unmanned Aircraft System) Node is a core component in the MAVROS architecture that serves as a central hub for managing communication between ROS and flight controllers. This node handles critical functions including plugin management, message routing, vehicle data storage, and coordinate frame transformations. For information about the Router system that connects to this node, see [Router System](https://deepwiki.com/mavlink/mavros/2.2-router-system).

## UAS Node Architecture

The UAS Node consists of two main classes: the `UAS` class that implements the node functionality and the `Data` class that stores shared vehicle information.

```
containsloads1manyUAS+Data data+diagnostic_updater+tf2_buffer+tf2_listener+tf2_broadcaster+tf2_static_broadcaster+is_connected() : : bool+get_type() : : MAV_TYPE+get_autopilot() : : MAV_AUTOPILOT+get_armed() : : bool+get_capabilities() : : uint64_t+send_message(msg, src_compid)+update_connection_status(bool)+add_plugin(string)+plugin_route(msg, framing)Data+update_attitude_imu_enu(imu)+update_attitude_imu_ned(imu)+update_gps_fix_epts(fix, eph, epv, fix_type, satellites)+get_attitude_imu_enu() : : imu+get_attitude_imu_ned() : : imu+get_gps_fix() : : fix+geoid_to_ellipsoid_height(lla)+ellipsoid_to_geoid_height(lla)PluginBase+get_subscriptions() : : Subscriptions+make_handler() : : HandlerInfo+enable_connection_cb()+enable_capabilities_cb()
```

Sources: [mavros/include/mavros/mavros\_uas.hpp233-643](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L233-L643) [mavros/src/lib/mavros\_uas.cpp34-202](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L34-L202)

## Initialization and Operation

The UAS Node is initialized in the MAVROS startup sequence as follows:

```
"Plugins""Router Node""UAS Node""mavros_node.cpp""Plugins""Router Node""UAS Node""mavros_node.cpp"Constructor setupAfter startup delayDuring operationCreate UAS NodeSetup diagnostic updaterInitialize TF2 componentsCreate startup_delay_timerProcess ROS parametersCreate UAS executor threadLoad pluginsConnect to RouterPublish static transformsMAVLink messagesRoute messages to handlersSend MAVLink messagesForward to FCU
```

Sources: [mavros/src/mavros\_node.cpp82-103](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp#L82-L103) [mavros/src/lib/mavros\_uas.cpp34-202](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L34-L202)

## The Data Class

The `Data` class within the UAS Node serves as a central storage for vehicle information that can be accessed by multiple plugins. It provides thread-safe methods for updating and retrieving:

1.  IMU data in both ENU and NED coordinate frames
2.  GPS fix data with accuracy metrics
3.  Utility functions for height transformations (geoid/ellipsoid conversions)

This data store is implemented with concurrency support using shared mutexes to ensure thread safety.

| Data Type | Storage Variables | Update Methods | Access Methods |
| --- | --- | --- | --- |
| IMU (ENU) | `imu_enu_data` | `update_attitude_imu_enu()` | `get_attitude_imu_enu()`, `get_attitude_orientation_enu()` |
| IMU (NED) | `imu_ned_data` | `update_attitude_imu_ned()` | `get_attitude_imu_ned()`, `get_attitude_orientation_ned()` |
| GPS | `gps_fix`, `gps_eph`, `gps_epv`, etc. | `update_gps_fix_epts()` | `get_gps_fix()`, `get_gps_epts()` |

Sources: [mavros/include/mavros/mavros\_uas.hpp78-215](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L78-L215) [mavros/src/lib/uas\_data.cpp29-158](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_data.cpp#L29-L158)

## Plugin System Integration

The UAS Node dynamically loads and manages plugins that extend MAVROS functionality. Each plugin provides handlers for specific MAVLink messages.

```
Plugin SystemUAS Nodecreate_plugin_instance()create_plugin_instance()create_plugin_instance()get_subscriptions()get_subscriptions()get_subscriptions()plugin_route()handler callbackhandler callbackhandler callbackStartup SequencePlugin Factory LoaderSubscription MapMessage RouterPlugin Base ClassPlugin 1Plugin 2Plugin 3
```

The plugin loading and registration process involves:

1.  Using `pluginlib` to discover available plugins
2.  Checking if each plugin is allowed based on allowlist/denylist configuration
3.  Creating plugin instances and obtaining their message subscriptions
4.  Building a subscription map that associates message IDs with handler callbacks
5.  Adding plugin nodes to the executor if they're separate from the UAS node

Sources: [mavros/src/lib/mavros\_uas.cpp262-345](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L262-L345) [mavros/include/mavros/plugin.hpp62-277](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L62-L277)

## Message Routing

The UAS Node acts as a message router between the MAVLink communication channel and the plugins. When it receives a MAVLink message from the Router Node, it:

1.  Identifies the message ID
2.  Looks up handlers registered for that message ID in its subscription map
3.  Calls each handler with the message and framing information

```
"Plugin Handler""plugin_subscriptions map""UAS Node""Router Node""Plugin Handler""plugin_subscriptions map""UAS Node""Router Node"Convert to mavlink_message_tloop[For each handler]"mavlink_source" topic (MAVLink message)recv_message()plugin_route()Find handlers for message IDVector of HandlerInfoCall handler callback
```

The core routing functionality is implemented in the `plugin_route` method, which is just a few lines of code that efficiently dispatches messages to registered handlers.

Sources: [mavros/src/lib/mavros\_uas.cpp204-214](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L204-L214) [mavros/src/lib/mavros\_uas.cpp385-395](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L385-L395)

## Connection and Capabilities Management

The UAS Node manages two important aspects of the flight controller interaction:

1.  **Connection Status**: Tracks whether the system is connected to the flight controller based on the reception of HEARTBEAT messages
2.  **Capabilities**: Tracks the capabilities of the flight controller, which indicate its supported features

Both systems use observer patterns that allow plugins to register callbacks for status changes:

```
PluginsUAS Nodeupdate_connection_status()update_capabilities()add_connection_change_handler()add_connection_change_handler()add_capabilities_change_handler()On changeOn changeNotifyNotifyNotifyHEARTBEAT HandlerAUTOPILOT_VERSION Handlerconnected (bool)fcu_capabilities (uint64_t)connection_cb_veccapabilities_cb_vecPlugin 1Plugin 2
```

Sources: [mavros/src/lib/uas\_ap.cpp27-102](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_ap.cpp#L27-L102) [mavros/include/mavros/mavros\_uas.hpp267-476](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L267-L476)

## Transform (TF2) Integration

The UAS Node integrates with ROS's TF2 library to handle coordinate frame transformations. It maintains:

1.  A TF2 Buffer for storing transformation data
2.  Transform listeners and broadcasters
3.  Helper methods for creating and publishing transforms

The node also publishes static transforms between different coordinate systems used in MAVLink and ROS, such as:

-   Converting between NED (North-East-Down) and ENU (East-North-Up) frames
-   Establishing relationships between base\_link, odom, and map frames

```
Standard TF FramesUAS Nodetf2_buffertf2_listenertf2_broadcastertf2_static_broadcasteradd_static_transform()publish_static_transform()base_linkbase_link_frdodomodom_nedmapmap_ned
```

Sources: [mavros/include/mavros/mavros\_uas.hpp353-380](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L353-L380) [mavros/src/lib/mavros\_uas.cpp160-184](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L160-L184)

## Time Synchronization

The UAS Node provides time synchronization capabilities between the flight controller and ROS:

1.  Maintains a time offset between FCU time and ROS time
2.  Provides methods to synchronize timestamps in messages
3.  Supports different synchronization modes (NONE, MAVLINK, ONBOARD)

```
// Example of time synchronization usage
rclcpp::Time UAS::synchronise_stamp(uint32_t time_boot_ms)
{
    // Convert time_boot_ms to a ROS timestamp using the stored time offset
    // ...
}
```

Sources: [mavros/include/mavros/mavros\_uas.hpp384-430](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L384-L430)

## Configuration Parameters

The UAS Node can be configured through ROS parameters:

| Parameter | Description | Default |
| --- | --- | --- |
| `uas_url` | URL for connecting to UAS endpoints | `/uas1` |
| `fcu_protocol` | MAVLink protocol version (v1.0 or v2.0) | `v2.0` |
| `system_id` | Source system ID | `1` |
| `component_id` | Source component ID | `191` |
| `target_system_id` | Target system ID | `1` |
| `target_component_id` | Target component ID | `1` |
| `plugin_allowlist` | List of allowed plugins | `[]` |
| `plugin_denylist` | List of denied plugins | `[]` |
| `base_link_frame_id` | TF frame ID for base link | `base_link` |
| `odom_frame_id` | TF frame ID for odometry | `odom` |
| `map_frame_id` | TF frame ID for map | `map` |

Sources: [mavros/src/lib/mavros\_uas.cpp75-104](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L75-L104) [mavros/src/mavros\_node.cpp41-55](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp#L41-L55)

## Integration with Other Components

The UAS Node is connected to the broader MAVROS system as follows:

```
External SystemsMAVROS EcosystemROS Environmentmavlink_source/sink topicsplugin_route()ParametersTopicsRouter NodeUAS NodePluginsROS TopicsROS ServicesTF TreeROS ParametersFlight ControllerGround Control StationROS Applications
```

Sources: [mavros/src/mavros\_node.cpp26-108](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp#L26-L108) [mavros/include/mavros/mavros\_uas.hpp233-643](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp#L233-L643)

## Conclusion

The UAS Node is the central component of the MAVROS system, responsible for:

1.  Managing plugins that extend MAVROS functionality
2.  Routing MAVLink messages between the flight controller and plugins
3.  Storing and providing access to vehicle data
4.  Handling coordinate frame transformations
5.  Managing connection status and capabilities

It serves as the bridge between the low-level MAVLink communication handled by the Router Node and the high-level ROS functionality provided by the plugins.