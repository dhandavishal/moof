Relevant source files

-   [CONTRIBUTING.md](https://github.com/mavlink/mavros/blob/44fa2f90/CONTRIBUTING.md)
-   [README.md](https://github.com/mavlink/mavros/blob/44fa2f90/README.md)
-   [libmavconn/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md)
-   [libmavconn/src/interface.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/interface.cpp)
-   [libmavconn/src/serial.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/serial.cpp)
-   [libmavconn/src/tcp.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/tcp.cpp)
-   [libmavconn/src/udp.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/udp.cpp)
-   [libmavconn/test/test\_mavconn.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/test/test_mavconn.cpp)
-   [mavros/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md)
-   [mavros\_extras/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/README.md)

The MAVROS Connection System provides a unified interface for establishing communication links between ROS applications and MAVLink-capable devices like Flight Controller Units (FCUs) and Ground Control Stations (GCS). It abstracts the underlying transport protocols (Serial, UDP, TCP) and provides a consistent URL-based configuration mechanism.

For information about how messages are routed between endpoints, see [Router System](https://deepwiki.com/mavlink/mavros/2.2-router-system).

## URL-Based Connection Configuration

MAVROS uses a URL-based system to define connections. The URL format specifies the transport protocol, connection parameters, and optionally system and component IDs.

### URL Format

```
protocol://[host/device][:port/baudrate][@remote][:port][/?ids=sysid,compid]
```

### Supported URL Schemas

| Schema | Description | Format |
| --- | --- | --- |
| Serial | Direct device path | `/path/to/serial/device[:baudrate]` |
| Serial | URL format | `serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]` |
| Serial with HW flow control | URL format | `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]` |
| UDP | Standard UDP | `udp://[bind_host][:port]@[remote_host[:port]][/?ids=sysid,compid]` |
| UDP Broadcast | Discovery mode | `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]` |
| UDP Permanent Broadcast | Always broadcast | `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]` |
| TCP Client | Connect to server | `tcp://[server_host][:port][/?ids=sysid,compid]` |
| TCP Server | Accept connections | `tcp-l://[bind_host][:port][/?ids=sysid,compid]` |

**Notes:**

-   System and component IDs from URL override values given by `system_id` & `component_id` parameters
-   Default bind host is `0.0.0.0` (all interfaces)
-   UDP default ports: 14555 (bind) @ 14550 (remote)
-   TCP default port: 5760

Sources: [mavros/README.md32-54](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L32-L54)

## Connection Interface Architecture

The Connection System is implemented in the `libmavconn` library, which provides a unified interface for all connection types.

```
«abstract»MAVConnInterface+open_url(url, system_id, component_id) : MAVConnInterface::Ptr+send_message(message)+close()+connect(message_cb, close_cb)#get_status()#iostat_tx_add()#iostat_rx_add()MAVConnSerial-serial_dev-io_service-rx_buf-tx_q+send_message()-do_read()-do_write()MAVConnUDP-socket-bind_ep-remote_ep-permanent_broadcast-rx_buf-tx_q+send_message()-do_recvfrom()-do_sendto()MAVConnTCPClient-socket-server_ep-rx_buf-tx_q+send_message()-do_read()-do_write()MAVConnTCPServer-acceptor-bind_ep-client_list+send_message()-do_accept()
```

Sources: [libmavconn/src/interface.cpp33-456](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/interface.cpp#L33-L456) [libmavconn/src/serial.cpp30-285](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/serial.cpp#L30-L285) [libmavconn/src/udp.cpp25-361](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/udp.cpp#L25-L361) [libmavconn/src/tcp.cpp33-531](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/tcp.cpp#L33-L531)

## Connection Process Flow

The following diagram illustrates how connections are established and messages are processed in the MAVROS Connection System:

```
FCU/GCSSpecific TransportMAVConnInterfaceApplicationFCU/GCSSpecific TransportMAVConnInterfaceApplicationloop[Message Handling]alt[Connection Closed]open_url(url)parse_url()create instanceopen connectionconnect(message_cb, close_cb)start io_threadstart async read/writereceive dataparse_buffer()message_received_cb(msg)send_message(msg)send dataconnection lostport_closed_cb()
```

Sources: [libmavconn/src/interface.cpp437-456](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/interface.cpp#L437-L456) [libmavconn/src/serial.cpp118-134](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/serial.cpp#L118-L134) [libmavconn/src/udp.cpp142-164](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/udp.cpp#L142-L164) [libmavconn/src/tcp.cpp147-169](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/tcp.cpp#L147-L169)

## URL Parsing and Connection Creation

The URL parsing system is a key component of the Connection System, allowing flexible configuration of connections.

```
serial://serial-hwfc://udp://udp-b:// or udp-pb://tcp://tcp-l://no protocolopen_url(url)Parse URLExtract protocol, host, path, queryCheck protocolParse serial parameterspath, baudrate, idsParse serial parameterswith hardware flow controlParse UDP parametersbind_host, bind_port, remote_host, remote_port, idsParse UDP broadcast parametersParse TCP client parametersserver_host, server_port, idsParse TCP server parametersbind_host, bind_port, idsAssume file pathDefault to serialCreate MAVConnSerialCreate MAVConnSerial with hwflow=trueCreate MAVConnUDPCreate MAVConnUDP with broadcastCreate MAVConnTCPClientCreate MAVConnTCPServerConnect callbacksStart I/O operations
```

Sources: [libmavconn/src/interface.cpp208-436](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/interface.cpp#L208-L436)

## Transport Implementation Details

### Serial Connection

The serial connection implementation (`MAVConnSerial`) uses ASIO to communicate with serial devices. It supports:

-   Configurable baud rates
-   Hardware flow control (optional)
-   Low-latency mode on Linux systems

Key features:

-   Uses asynchronous I/O for non-blocking operation
-   Handles serial port configuration (baud rate, parity, stop bits)
-   Enables Linux-specific optimizations when available

Sources: [libmavconn/src/serial.cpp40-111](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/serial.cpp#L40-L111) [libmavconn/src/serial.cpp223-284](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/serial.cpp#L223-L284)

### UDP Connection

The UDP connection implementation (`MAVConnUDP`) provides:

-   Client/server communication
-   Broadcasting capabilities
-   Dynamic remote endpoint discovery

Key features:

-   Supports binding to a specific host/port
-   Can connect to a known remote host/port
-   Broadcast mode for discovery
-   Permanent broadcast mode
-   Large buffer sizes for optimal performance

Sources: [libmavconn/src/udp.cpp73-130](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/udp.cpp#L73-L130) [libmavconn/src/udp.cpp278-354](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/udp.cpp#L278-L354)

### TCP Connections

There are two TCP implementations:

1.  `MAVConnTCPClient` - connects to a TCP server
    
    -   Maintains a persistent connection to a server
    -   Reconnection capabilities
    -   Message queuing
2.  `MAVConnTCPServer` - accepts incoming TCP connections
    
    -   Accepts multiple client connections
    -   Broadcasts messages to all connected clients
    -   Provides connection management

Sources: [libmavconn/src/tcp.cpp83-338](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/tcp.cpp#L83-L338) [libmavconn/src/tcp.cpp341-530](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/tcp.cpp#L341-L530)

## Transport Message Processing

All transport implementations follow a similar pattern for message processing:

1.  Async reading from the connection
2.  Buffer parsing to extract MAVLink messages
3.  Callback invocation for received messages
4.  Queuing and async sending of outgoing messages

The `MAVConnInterface` base class provides common functionality for:

-   MAVLink message parsing
-   Protocol version detection
-   Statistics tracking
-   Error handling

Sources: [libmavconn/src/interface.cpp100-125](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/interface.cpp#L100-L125) [libmavconn/src/interface.cpp159-184](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/interface.cpp#L159-L184)

## Usage Examples

### Connecting to a Flight Controller via Serial

```
// Create a serial connection to a Pixhawk at 921600 baud
auto serial_conn = MAVConnInterface::open_url(
    "serial:///dev/ttyACM0:921600",
    1,  // system_id
    1,  // component_id
    [](const mavlink_message_t* msg, Framing framing) {
        // Handle received message
    },
    []() {
        // Handle connection closed
    }
);
```

### Setting up a UDP Link to a Ground Control Station

```
// Create a UDP connection to QGroundControl
auto udp_conn = MAVConnInterface::open_url(
    "udp://:14555@localhost:14550",
    1,  // system_id
    1,  // component_id
    message_handler_callback,
    connection_closed_callback
);
```

### Creating a TCP Server for Multiple Clients

```
// Create a TCP server that accepts connections on port 5760
auto tcp_server = MAVConnInterface::open_url(
    "tcp-l://0.0.0.0:5760",
    1,  // system_id
    1,  // component_id
    message_handler_callback,
    connection_closed_callback
);
```

## Integration with MAVROS

The Connection System is integrated with the broader MAVROS system through the Router node, which uses the libmavconn library to establish connections to FCUs and GCSs.

```
External DevicesConnection SystemMAVROS SystemRouter NodeUAS NodePlugin ContainerPluginssys_status, command, etc.MAVConnInterfaceMAVConnSerialMAVConnUDPMAVConnTCPClientMAVConnTCPServerFlight Controller UnitGround Control Station
```

Sources: [mavros/README.md89-97](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md#L89-L97)

## Error Handling

The Connection System includes robust error handling for common communication issues:

-   Connection failures (device not found, permission denied)
-   Network errors (host unreachable, connection refused)
-   Protocol errors (invalid data, buffer overruns)
-   Timeout handling for all connection types

When errors occur, the system:

1.  Logs detailed error information
2.  Attempts recovery when appropriate (e.g., for temporary network issues)
3.  Closes the connection cleanly if recovery isn't possible
4.  Notifies the application through the closed port callback

Sources: [libmavconn/src/serial.cpp234-237](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/serial.cpp#L234-L237) [libmavconn/src/udp.cpp324-333](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/udp.cpp#L324-L333) [libmavconn/src/tcp.cpp282-286](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/src/tcp.cpp#L282-L286)