Relevant source files

-   [CONTRIBUTING.md](https://github.com/mavlink/mavros/blob/44fa2f90/CONTRIBUTING.md)
-   [README.md](https://github.com/mavlink/mavros/blob/44fa2f90/README.md)
-   [dependencies.rosinstall](https://github.com/mavlink/mavros/blob/44fa2f90/dependencies.rosinstall)
-   [libmavconn/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md)
-   [mavros/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/README.md)
-   [mavros/launch/apm\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml)
-   [mavros/launch/apm\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml)
-   [mavros/launch/px4\_config.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml)
-   [mavros/launch/px4\_pluginlists.yaml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml)
-   [mavros\_extras/README.md](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/README.md)

This page provides a comprehensive guide for installing MAVROS, setting up required dependencies, and performing initial configuration. MAVROS serves as a bridge between ROS2 and MAVLink-based flight controllers, enabling communication with autopilot systems like PX4 Pro and ArduPilot.

For information about the overall MAVROS architecture, see [MAVROS Overview](https://deepwiki.com/mavlink/mavros/1-mavros-overview). For package structure details, see [Package Structure](https://deepwiki.com/mavlink/mavros/1.1-package-structure).

## System Requirements

Before installing MAVROS, ensure your system meets these requirements:

-   Linux operating system (MAVROS is not supported on Windows or macOS)
-   ROS2 Humble or newer (Humble, Iron, Jazzy, or Rolling)
-   Internet connection for downloading packages and dependencies
-   C++14 compatible compiler

Sources: [README.md14-21](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L14-L21)

## Installation Methods

MAVROS can be installed using pre-built binary packages or built from source. The binary installation is simpler and recommended for most users, while building from source provides access to the latest features.

### Installation Workflow

```
Binary(Recommended)Source(Latest features)Start InstallationChooseInstallation MethodInstall with apt:sudo apt install ros--mavrosCreate workspace:mkdir -p ~/ros2_ws/srccd ~/ros2_wsInstall GeographicLib datasets:ros2 run mavros install_geographiclib_datasets.shGet source code usingrosinstall_generator and vcsInstall dependencies:rosdep install --from-paths src --ignore-src -yInstall GeographicLib datasets:./src/mavros/mavros/scripts/install_geographiclib_datasets.shBuild workspace:colcon buildSource workspace:source install/setup.bashInstallation Complete
```

Sources: [README.md129-217](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L129-L217)

### Binary Installation

ROS2 repositories provide binary packages for common architectures:

```
sudo apt install ros-<distro>-mavros
```

Replace `<distro>` with your ROS2 distribution name (e.g., `humble`, `iron`).

After installation, you must install the GeographicLib datasets:

```
ros2 run mavros install_geographiclib_datasets.sh
```

Sources: [README.md162-176](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L162-L176)

### Source Installation

For the latest features or if binary packages aren't available for your platform:

```
# Install required tools
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon

# Create workspace (if needed)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Get source code
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos

# Import source
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos

# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Install GeographicLib datasets
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

Sources: [README.md179-217](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L179-L217) [dependencies.rosinstall1-4](https://github.com/mavlink/mavros/blob/44fa2f90/dependencies.rosinstall#L1-L4)

## Required Dependencies

### GeographicLib

GeographicLib is a critical dependency used for coordinate transformations. MAVROS requires not just the library itself but also specific datasets, particularly the geoid dataset.

> **Important**: The geoid dataset is mandatory. MAVROS nodes will shut down if this dataset is not available.

Install the datasets using the provided script:

```
ros2 run mavros install_geographiclib_datasets.sh
```

Or directly with GeographicLib tools:

```
# For most modern distributions:
geographiclib-get-geoids egm96-5
```

Sources: [README.md133-159](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L133-L159)

### Other Dependencies

Most other dependencies are managed automatically by `rosdep` when following the installation procedures. These include:

-   Asio library for network communication
-   console-bridge for logging
-   MAVLink message definitions

Sources: [libmavconn/README.md29-37](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L29-L37)

## Connection Configuration

### Connection URL System

MAVROS uses URL strings to define connections to Flight Control Units (FCUs) and Ground Control Stations (GCS). The URL format specifies the transport type, addressing information, and optional parameters.

```
MAVROS Connection Systemserial://serial-hwfc://udp://udp-b://udp-pb://tcp://tcp-l://MAVConnInterface::open_url()Parse URL TypeMAVConnSerialMAVConnSerialwith hardware flow controlMAVConnUDPMAVConnUDPBroadcast ModeMAVConnTCPClientMAVConnTCPServerConnection toFlight ControllerConnection toGround Control Station
```

Sources: [README.md32-54](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L32-L54) [libmavconn/README.md9-26](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L9-L26)

### Supported URL Formats

| Connection Type | URL Format | Description |
| --- | --- | --- |
| Serial | `/path/to/device[:baudrate]` | Simple serial connection |
| Serial | `serial:///path/to/device[:baudrate][?ids=sysid,compid]` | Serial with optional system/component ID |
| Serial with flow control | `serial-hwfc:///path/to/device[:baudrate][?ids=sysid,compid]` | Serial with hardware flow control |
| UDP | `udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]` | UDP connection with specific endpoints |
| UDP broadcast (discovery) | `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]` | UDP broadcast until GCS discovery |
| UDP broadcast (permanent) | `udp-pb://[bind_host][:port]@[:port][/?ids=sysid,compid]` | Permanent UDP broadcast |
| TCP client | `tcp://[server_host][:port][/?ids=sysid,compid]` | Connect to TCP server |
| TCP server | `tcp-l://[bind_host][:port][/?ids=sysid,compid]` | Listen for TCP connections |

**Notes:**

-   Default bind host is `0.0.0.0` (all interfaces)
-   Default UDP ports: 14555 @ 14550
-   Default TCP port: 5760
-   System/component IDs in URL override those specified by parameters
-   For UDP, the remote address is updated with each incoming packet

Sources: [README.md32-54](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L32-L54) [libmavconn/README.md12-26](https://github.com/mavlink/mavros/blob/44fa2f90/libmavconn/README.md#L12-L26)

## Running MAVROS

MAVROS can be started using the `mavros_node` executable, which is a composite container that loads the Router and UAS nodes:

```
ros2 run mavros mavros_node --ros-args --params-file params.yaml
```

Sources: [README.md103-110](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L103-L110)

### Configuration Files

MAVROS provides default configuration files for different autopilots:

-   `px4_config.yaml` - Configuration for PX4 autopilot
-   `px4_pluginlists.yaml` - Plugin denylist/allowlist for PX4
-   `apm_config.yaml` - Configuration for ArduPilot
-   `apm_pluginlists.yaml` - Plugin denylist/allowlist for ArduPilot

```
Configuration StructureloadsloadsConfiguration Filespx4_config.yamlpx4_pluginlists.yamlapm_config.yamlapm_pluginlists.yamlSystem Plugins Configuration- sys_status- sys_timeCore Plugins Configuration- imu- command- setpoint_position- etc.Plugin Denylist(Disabled Plugins)mavros_nodeRouter NodeUAS Node(Plugin Container)
```

Sources: [mavros/launch/px4\_config.yaml1-297](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L1-L297) [mavros/launch/apm\_config.yaml1-285](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_config.yaml#L1-L285) [mavros/launch/px4\_pluginlists.yaml1-15](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_pluginlists.yaml#L1-L15) [mavros/launch/apm\_pluginlists.yaml1-19](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/apm_pluginlists.yaml#L1-L19)

### Plugin Configuration

The configuration files contain parameters for various plugins. For example, to configure the global position plugin in PX4:

```
/**/global_position:
  ros__parameters:
    frame_id: "map"               # origin frame
    child_frame_id: "base_link"   # body-fixed frame
    rot_covariance: 99999.0       # covariance for attitude
    use_relative_alt: true        # use relative altitude for local coordinates
    tf.send: false                # send TF?
```

Sources: [mavros/launch/px4\_config.yaml49-60](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/launch/px4_config.yaml#L49-L60)

## Troubleshooting

### Common Issues

#### Missing GeographicLib Datasets

**Symptom**: MAVROS node shuts down immediately with messages about missing GeographicLib datasets.

**Solution**: Install the required datasets:

```
ros2 run mavros install_geographiclib_datasets.sh
```

Sources: [README.md153-158](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L153-L158)

#### Serial Port Connection Issues

**Symptom**: Error "serial0: receive: End of file"

**Solution**: This issue was fixed in MAVROS v0.23.2. It was identified as a Boost.ASIO error and should be resolved in releases with Boost 1.66 or newer.

Sources: [README.md235-238](https://github.com/mavlink/mavros/blob/44fa2f90/README.md#L235-L238)

#### Permissions Issues with Serial Ports

**Symptom**: Unable to open serial device due to permission denied errors.

**Solution**: Add your user to the `dialout` group:

```
sudo usermod -a -G dialout $USER
```

Then log out and log back in for the changes to take effect.