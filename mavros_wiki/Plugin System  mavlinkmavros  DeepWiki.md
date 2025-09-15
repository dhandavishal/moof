Relevant source files

-   [mavros/include/mavros/mavros\_uas.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/mavros_uas.hpp)
-   [mavros/include/mavros/plugin.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp)
-   [mavros/include/mavros/plugin\_filter.hpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp)
-   [mavros/src/lib/mavros\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp)
-   [mavros/src/lib/plugin.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp)
-   [mavros/src/lib/uas\_ap.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/uas_ap.cpp)
-   [mavros/src/mavros\_node.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/mavros_node.cpp)
-   [mavros/src/plugins/command.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp)
-   [mavros/src/plugins/dummy.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp)
-   [mavros/src/plugins/ftp.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/ftp.cpp)
-   [mavros/src/plugins/param.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp)
-   [mavros/src/plugins/rc\_io.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/rc_io.cpp)
-   [mavros/src/plugins/sys\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp)
-   [mavros/src/plugins/sys\_time.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_time.cpp)
-   [mavros/src/plugins/waypoint.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp)
-   [mavros/test/test\_uas.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/test/test_uas.cpp)

The MAVROS Plugin System provides an extensible architecture for adding new functionality to interact with MAVLink-enabled flight controllers. This system allows developers to create modular components that handle specific MAVLink messages and provide corresponding ROS interfaces (topics, services, parameters). For information about specific plugins like System Status, Command, or Parameter handling, see their respective pages ([Core Plugins](https://deepwiki.com/mavlink/mavros/3.1-core-plugins), [Extra Plugins](https://deepwiki.com/mavlink/mavros/3.2-extra-plugins)).

## Plugin Architecture Overview

The plugin system is built around a base `Plugin` class from which all plugins inherit. Each plugin registers for specific MAVLink messages, processes them, and exposes appropriate ROS interfaces. The UAS (Unmanned Aircraft System) node is responsible for loading plugins, routing messages to them, and managing their lifecycle.

```
Plugin Loading ProcessPlugin Base Systemroute messagesPlugin Base ClassTF2ListenerMixinSetpointMixinUAS NodePlugin FactoryPlugin InstanceMessage Subscriptions
```

Sources: [mavros/include/mavros/plugin.hpp62-329](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L62-L329) [mavros/src/lib/mavros\_uas.cpp262-325](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L262-L325)

## Plugin Base Class

All MAVROS plugins inherit from the `Plugin` base class, which provides common functionality:

```
Plugin#UASPtr uas#Node::SharedPtr node+Plugin(UASPtr uas_)+Plugin(UASPtr uas_, string subnode)+Subscriptions get_subscriptions() : virtual#make_handler()#connection_cb()#capabilities_cb()#node_on_set_parameters_cb()#enable_node_watch_parameters()SystemStatusPlugin+SystemStatusPlugin(UASPtr uas_)+Subscriptions get_subscriptions()-handle_heartbeat()-handle_sys_status()-handle_statustext()CommandPlugin+CommandPlugin(UASPtr uas_)+Subscriptions get_subscriptions()-handle_command_ack()-command_long_cb()-arming_cb()
```

Key components of the `Plugin` class:

| Component | Description |
| --- | --- |
| `UASPtr uas` | Connection to the UAS node that provides access to the MAVLink system |
| `Node::SharedPtr node` | ROS node pointer for creating publishers, subscribers, services |
| `get_subscriptions()` | Pure virtual method that returns list of message subscriptions |
| `make_handler()` | Helper method to create message handlers with type safety |
| `connection_cb()` | Callback for connection status changes |
| `capabilities_cb()` | Callback for autopilot capabilities changes |
| `node_on_set_parameters_cb()` | Handles ROS parameter changes |

Sources: [mavros/include/mavros/plugin.hpp62-121](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L62-L121) [mavros/include/mavros/plugin.hpp177-205](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L177-L205)

## Message Subscription System

The core of the plugin system is the message subscription mechanism that routes MAVLink messages to the appropriate handler functions in each plugin.

Plugins subscribe to messages by implementing the `get_subscriptions()` method:

```
Subscriptions get_subscriptions() override
{
  return {
    make_handler(&SystemStatusPlugin::handle_heartbeat),
    make_handler(&SystemStatusPlugin::handle_sys_status),
    make_handler(&SystemStatusPlugin::handle_statustext),
  };
}
```

The `make_handler()` method creates a handler for a specific message type using template metaprogramming to ensure type safety.

When a MAVLink message is received, the UAS node routes it to all plugins that have registered for that message ID.

Sources: [mavros/include/mavros/plugin.hpp128-176](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L128-L176) [mavros/src/plugins/sys\_status.cpp629-642](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L629-L642) [mavros/src/lib/mavros\_uas.cpp204-214](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L204-L214)

## Message Filtering System

MAVROS provides a filtering system that controls which messages are processed by each handler:

```
«interface»Filter+operator()(UASPtr, mavlink_message_t*, Framing) : boolAnyOk+operator()(UASPtr, mavlink_message_t*, Framing) : boolSystemAndOk+operator()(UASPtr, mavlink_message_t*, Framing) : boolComponentAndOk+operator()(UASPtr, mavlink_message_t*, Framing) : bool
```

Filter types:

-   **AnyOk**: Processes all messages with valid framing
-   **SystemAndOk**: Only processes messages from the target system ID
-   **ComponentAndOk**: Only processes messages from the target system and component ID

Example usage in a handler:

```
void handle_heartbeat(
  const mavlink::mavlink_message_t* msg,
  mavlink::minimal::msg::HEARTBEAT& hb,
  plugin::filter::SystemAndOk filter)
{
  // Only called for messages from the target system
}
```

Sources: [mavros/include/mavros/plugin\_filter.hpp40-76](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin_filter.hpp#L40-L76) [mavros/src/plugins/sys\_status.cpp894-947](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L894-L947)

## Plugin Parameter System

Plugins can expose ROS parameters using a simplified parameter declaration and handling system:

```
ROSUASPluginROSUASPluginenable_node_watch_parameters()node_declare_and_watch_parameter()declare_parameter()parameter_change_callback()Update internal state
```

Example parameter declaration:

```
node_declare_and_watch_parameter(
  "heartbeat_rate", 1.0, [&](const rclcpp::Parameter & p) {
    auto rate_d = p.as_double();
    
    if (rate_d == 0) {
      if (heartbeat_timer) {
        heartbeat_timer->cancel();
      }
    } else {
      heartbeat_timer = 
        node->create_wall_timer(
          rate.period(), std::bind(&SystemStatusPlugin::heartbeat_cb, this));
    }
  });
```

Sources: [mavros/include/mavros/plugin.hpp220-254](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L220-L254) [mavros/src/lib/plugin.cpp37-66](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp#L37-L66) [mavros/src/plugins/sys\_status.cpp548-566](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L548-L566)

## Plugin Registration and Loading

Plugins are registered using the `MAVROS_PLUGIN_REGISTER` macro, which creates a factory for the plugin:

```
// At the end of the plugin file
#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::SystemStatusPlugin)
```

The UAS node dynamically loads and instantiates plugins:

1.  Discover plugin classes using pluginlib
2.  Filter plugins based on allowlist/denylist
3.  Create instances of allowed plugins
4.  Get message subscriptions from each plugin
5.  Add subscriptions to a routing table
6.  Route incoming messages to plugins based on the table

Sources: [mavros/src/plugins/sys\_status.cpp557-558](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L557-L558) [mavros/src/lib/mavros\_uas.cpp155-324](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/mavros_uas.cpp#L155-L324)

## Core Plugins

MAVROS includes several core plugins that provide essential functionality:

| Plugin | Description | Main Message Types |
| --- | --- | --- |
| `SystemStatusPlugin` | System status, heartbeat, battery info | `HEARTBEAT`, `SYS_STATUS`, `STATUSTEXT` |
| `CommandPlugin` | MAVLink command handling | `COMMAND_LONG`, `COMMAND_ACK` |
| `ParameterPlugin` | Vehicle parameter access | `PARAM_VALUE`, `PARAM_SET` |
| `WaypointPlugin` | Mission handling | `MISSION_ITEM`, `MISSION_CURRENT` |
| `FTPPlugin` | File transfer protocol | `FILE_TRANSFER_PROTOCOL` |

Sources: [mavros/src/plugins/sys\_status.cpp482-809](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/sys_status.cpp#L482-L809) [mavros/src/plugins/command.cpp67-551](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/command.cpp#L67-L551) [mavros/src/plugins/param.cpp417-544](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/param.cpp#L417-L544) [mavros/src/plugins/waypoint.cpp33-324](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp#L33-L324)

## Creating Custom Plugins

To create a custom plugin:

1.  Create a class that inherits from `plugin::Plugin`
2.  Implement the required `get_subscriptions()` method
3.  Create message handlers for your subscriptions
4.  Register your plugin with `MAVROS_PLUGIN_REGISTER`

Example of a minimal plugin:

```
class MyPlugin : public plugin::Plugin
{
public:
  explicit MyPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "my_plugin") 
  {
    // Initialize plugin, create publishers, services, etc.
    status_pub = node->create_publisher<std_msgs::msg::String>("~/status", 10);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&MyPlugin::handle_heartbeat)
    };
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;

  void handle_heartbeat(
    const mavlink::mavlink_message_t* msg,
    mavlink::minimal::msg::HEARTBEAT& hb,
    plugin::filter::SystemAndOk filter)
  {
    // Process the heartbeat message
    auto status = std_msgs::msg::String();
    status.data = "Heartbeat received";
    status_pub->publish(status);
  }
};

// Register the plugin
MAVROS_PLUGIN_REGISTER(MyPlugin)
```

See the `DummyPlugin` implementation for a complete example of a minimal plugin.

Sources: [mavros/src/plugins/dummy.cpp35-134](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/dummy.cpp#L35-L134)

## Advanced Plugin Features

### Connection Status Callbacks

Plugins can register for connection status changes:

```
void enable_connection_cb();

void connection_cb(bool connected) override
{
  // Handle connection status change
  if (connected) {
    // Start operations that require connection
  } else {
    // Stop operations, clear state
  }
}
```

### Capabilities Callbacks

Plugins can receive notifications when FCU capabilities change:

```
void enable_capabilities_cb();

void capabilities_cb(uas::MAV_CAP capabilities) override
{
  // Check for specific capabilities
  if (uas->has_capability(uas::MAV_CAP::MISSION_INT)) {
    // Use MISSION_ITEM_INT messages
  } else {
    // Fall back to MISSION_ITEM messages
  }
}
```

Sources: [mavros/src/plugins/waypoint.cpp117-129](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/waypoint.cpp#L117-L129) [mavros/src/lib/plugin.cpp21-34](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/plugin.cpp#L21-L34)

## Plugin Mixins

Some plugins use mixin classes to extend functionality:

-   **TF2ListenerMixin**: Provides TF2 transform operations for plugins that need coordinate frame transformations
-   **SetpointMixin**: Provides common functionality for setpoint-related plugins

These mixins are used by applying multiple inheritance:

```
class VisionPoseEstimatePlugin : 
  public plugin::Plugin,
  private plugin::TF2ListenerMixin<VisionPoseEstimatePlugin>
{
  // Implementation
};
```

Sources: [mavros/include/mavros/plugin.hpp62-65](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/include/mavros/plugin.hpp#L62-L65)