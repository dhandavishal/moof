Relevant source files

-   [as2\_core/as2\_core/declare\_launch\_arguments\_from\_config\_file.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/declare_launch_arguments_from_config_file.py)
-   [as2\_core/as2\_core/launch\_configuration\_from\_config\_file.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/launch_configuration_from_config_file.py)
-   [as2\_core/as2\_core/launch\_param\_utils.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/launch_param_utils.py)
-   [as2\_core/as2\_core/launch\_plugin\_utils.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/launch_plugin_utils.py)
-   [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp)
-   [as2\_core/include/as2\_core/names/topics.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/names/topics.hpp)
-   [as2\_core/include/as2\_core/sensor.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp)
-   [as2\_core/include/as2\_core/utils/frame\_utils.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/utils/frame_utils.hpp)
-   [as2\_core/src/aerial\_platform.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp)
-   [as2\_core/src/node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/node.cpp)
-   [as2\_core/src/platform\_state\_machine.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/platform_state_machine.cpp)
-   [as2\_core/src/rate.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/rate.cpp)
-   [as2\_core/src/sensor.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/sensor.cpp)
-   [as2\_core/src/utils/control\_mode\_utils.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/utils/control_mode_utils.cpp)
-   [as2\_core/src/utils/frame\_utils.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/utils/frame_utils.cpp)
-   [as2\_core/tests/mocks/aerial\_platform/mock\_aerial\_platform.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/mocks/aerial_platform/mock_aerial_platform.cpp)
-   [as2\_core/tests/mocks/aerial\_platform/mock\_aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/mocks/aerial_platform/mock_aerial_platform.hpp)
-   [as2\_core/tests/platform\_state\_machine\_test.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/platform_state_machine_test.cpp)
-   [as2\_core/tests/sensor\_test.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/sensor_test.cpp)
-   [as2\_msgs/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/CMakeLists.txt)
-   [as2\_msgs/msg/PolygonList.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/PolygonList.msg)
-   [as2\_utilities/as2\_geozones/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/CMakeLists.txt)
-   [as2\_utilities/as2\_geozones/config/geozones.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/config/geozones.yaml)
-   [as2\_utilities/as2\_geozones/include/as2\_geozones/as2\_geozones.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/include/as2_geozones/as2_geozones.hpp)
-   [as2\_utilities/as2\_geozones/launch/as2\_geozones\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/launch/as2_geozones_launch.py)
-   [as2\_utilities/as2\_geozones/src/as2\_geozones.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/src/as2_geozones.cpp)
-   [as2\_utilities/as2\_geozones/src/as2\_geozones\_node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/src/as2_geozones_node.cpp)
-   [as2\_utilities/as2\_geozones/tests/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/tests/CMakeLists.txt)

The AS2 Core and Messages system forms the foundation of the Aerostack2 framework. This document covers the core functionality and message definitions that enable all other components in the system to communicate and operate effectively. For information about specific implementations such as state estimation or motion control, see their respective pages ([State Estimation](https://deepwiki.com/aerostack2/aerostack2/2.2-state-estimation), [Motion Control](https://deepwiki.com/aerostack2/aerostack2/2.3-motion-control)).

## 1\. Overview and Purpose

The AS2 Core provides fundamental building blocks for all Aerostack2 components, including:

-   Base class functionality for nodes and platforms
-   Sensor abstractions and utilities
-   Platform state machine management
-   Standardized naming conventions

The AS2 Messages package defines the communication interfaces between components, ensuring a standardized way to exchange information throughout the system.

```
AS2 MessagesAS2 Coreas2::Nodeas2::AerialPlatformas2::PlatformStateMachineas2::sensors::SensorUtilitiesControlModePlatformInfoThrustAlertEventTrajectorySetpointsService Definitions
```

Sources:

-   [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp)
-   [as2\_core/include/as2\_core/sensor.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp)
-   [as2\_msgs/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/CMakeLists.txt)

## 2\. AS2 Core Architecture

The AS2 Core provides the base classes and utilities that all Aerostack2 components build upon. It establishes a consistent framework for node creation, platform management, and sensor interaction.

### 2.1 Core Components

```
Node+generate_global_name()+generate_local_name()AerialPlatform-platform_info_msg_-state_machine_+bool setArmingState()+bool setOffboardControl()+bool setPlatformControlMode()+bool takeoff()+bool land()#virtual bool ownSendCommand()#virtual bool ownSetArmingState()#virtual bool ownSetOffboardControl()#virtual bool ownSetPlatformControlMode()#virtual bool ownTakeoff()#virtual bool ownLand()#virtual void ownKillSwitch()#virtual void ownStopPlatform()PlatformStateMachine-state_-transitions_+bool processEvent()+bool setState()+getState()Sensor<T>+updateData()+setData()+publish()
```

Sources:

-   [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp)
-   [as2\_core/src/aerial\_platform.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp)
-   [as2\_core/src/node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/node.cpp)
-   [as2\_core/include/as2\_core/sensor.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp)

### 2.2 Node Base Class

The `as2::Node` class serves as the base for all Aerostack2 nodes, providing common functionality for topic and service naming.

Key methods:

-   `generate_global_name(const std::string &name)`: Generates a global topic/service name
-   `generate_local_name(const std::string &name)`: Generates a node-specific topic/service name

Sources:

-   [as2\_core/src/node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/node.cpp)

### 2.3 Platform State Machine

The platform state machine manages the state transitions of aerial platforms, ensuring that operations occur in the correct sequence.

```
ARMTAKE_OFFTOOK_OFFLANDLANDEDDISARMEMERGENCYEMERGENCYEMERGENCYEMERGENCYEMERGENCYDISARMDISARMEDLANDEDTAKING_OFFFLYINGLANDINGEMERGENCY
```

Sources:

-   [as2\_core/src/platform\_state\_machine.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/platform_state_machine.cpp)
-   [as2\_core/tests/platform\_state\_machine\_test.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/platform_state_machine_test.cpp)

## 3\. Aerial Platform Base Class

The `as2::AerialPlatform` class is the foundation for all vehicle implementations in Aerostack2. It provides:

1.  Standard interfaces for platform control
2.  State machine management
3.  Service interfaces for vehicle operations
4.  Message handling for commands and status

Platform-specific implementations must override several virtual methods to provide platform-specific behavior:

```
- ownSendCommand()
- ownSetArmingState()
- ownSetOffboardControl()
- ownSetPlatformControlMode()
- ownTakeoff()
- ownLand()
- ownKillSwitch()
- ownStopPlatform()
```

Sources:

-   [as2\_core/include/as2\_core/aerial\_platform.hpp118-183](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L118-L183)
-   [as2\_core/src/aerial\_platform.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp)

### 3.1 Platform Initialization Flow

```
as2::PlatformStateMachineas2::AerialPlatformUseras2::PlatformStateMachineas2::AerialPlatformUsersetArmingState(true)ownSetArmingState(true)handleStateMachineEvent(ARM)setOffboardControl(true)ownSetOffboardControl(true)setPlatformControlMode(mode)ownSetPlatformControlMode(mode)takeoff()ownTakeoff()handleStateMachineEvent(TOOK_OFF)
```

Sources:

-   [as2\_core/include/as2\_core/aerial\_platform.hpp193-227](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L193-L227)
-   [as2\_core/src/aerial\_platform.cpp197-271](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L197-L271)

## 4\. Sensor Framework

The sensor framework provides abstractions for different sensor types and handles data publishing and transformation.

```
SensorData<T>-msg_data_-sensor_publisher_+setData()+publish()+updateAndPublish()GenericSensor#pub_freq_#timer_+dataUpdated()+virtual publishData()TFStatic+setStaticTransform()TFDynamic+setDynamicTransform()Sensor<T>+updateData()Camera-camera_info_-image_data_+updateData()+setCameraInfo()GroundTruth+updateData()Gimbal+updateData()+setGimbalBaseTransform()
```

The sensor framework includes specialized implementations for different sensor types:

-   Camera
-   IMU
-   GPS
-   Odometry
-   Lidar
-   Battery
-   Barometer
-   Compass
-   Range Finder
-   Ground Truth
-   Gimbal

Sources:

-   [as2\_core/include/as2\_core/sensor.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp)
-   [as2\_core/src/sensor.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/sensor.cpp)
-   [as2\_core/tests/sensor\_test.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/sensor_test.cpp)

## 5\. AS2 Messages

The AS2 Messages package defines the communication interfaces between all Aerostack2 components. These messages ensure standardized data exchange throughout the system.

### 5.1 Core Message Types

| Message Type | Description | Key Fields |
| --- | --- | --- |
| `PlatformInfo` | Platform status information | `armed`, `offboard`, `current_control_mode`, `status` |
| `PlatformStatus` | Current state of the platform | `state` (DISARMED, LANDED, TAKING\_OFF, etc.) |
| `ControlMode` | Mode of control for the platform | `control_mode`, `yaw_mode`, `reference_frame` |
| `TrajectorySetpoints` | Trajectory setpoints for motion | `positions`, `velocities`, `accelerations` |
| `AlertEvent` | Messages for alert events | `alert`, `description` |
| `Thrust` | Custom message for thrust commands | `thrust` |

Sources:

-   [as2\_msgs/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/CMakeLists.txt)

### 5.2 Communication Patterns

```
State EstimatorBehaviorsPlatformControlleractuator_command/poseactuator_command/twistactuator_command/thrustactuator_command/trajectorymotion_reference/posemotion_reference/twistmotion_reference/trajectoryself_localization/poseself_localization/twistself_localization/odomself_localization/poseself_localization/twistself_localization/odomplatform/infoplatform/infoas2_motion_controlleras2::AerialPlatformas2_behaviorsas2_state_estimator
```

Sources:

-   [as2\_core/include/as2\_core/names/topics.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/names/topics.hpp)
-   [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp)

## 6\. Utility Functions

The AS2 Core provides various utility functions to support the operation of the framework.

### 6.1 Frame Utilities

The frame utilities provide functions for transforming between different coordinate frames and handling quaternion/euler conversions.

Key functions:

-   `transform()`: Apply a quaternion rotation to a vector
-   `transformInverse()`: Apply an inverse quaternion rotation to a vector
-   `quaternionToEuler()`: Convert quaternion to euler angles
-   `eulerToQuaternion()`: Convert euler angles to quaternion
-   `wrapAngle0To2Pi()`: Wrap angle to \[0, 2\*pi\]
-   `wrapAnglePiToPi()`: Wrap angle to \[-pi, pi\]
-   `angleMinError()`: Compute the minimum angle between two angles

Sources:

-   [as2\_core/include/as2\_core/utils/frame\_utils.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/utils/frame_utils.hpp)
-   [as2\_core/src/utils/frame\_utils.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/utils/frame_utils.cpp)

### 6.2 Launch Configuration Utilities

The AS2 Core provides utilities for handling launch configurations:

-   `LaunchConfigurationFromConfigFile`: Override launch configuration from a config file with arguments
-   `DeclareLaunchArgumentsFromConfigFile`: Declare launch arguments from elements in a config file

Sources:

-   [as2\_core/as2\_core/launch\_configuration\_from\_config\_file.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/launch_configuration_from_config_file.py)
-   [as2\_core/as2\_core/declare\_launch\_arguments\_from\_config\_file.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/declare_launch_arguments_from_config_file.py)
-   [as2\_core/as2\_core/launch\_param\_utils.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/as2_core/launch_param_utils.py)

## 7\. Integration with Geozones

The AS2 Core integrates with the Geozones utility to provide geofencing capabilities.

```
Alert SystemNavigationAS2 CoreGeozonesGeozonesas2::Nodeas2::sensors::Sensorself_localization/posesensor_measurements/gpsalert_event
```

The Geozones system:

-   Monitors vehicle position relative to defined geographic boundaries
-   Generates alert events when boundaries are crossed
-   Supports both cartesian and GPS coordinates
-   Provides visualization in RViz

Sources:

-   [as2\_utilities/as2\_geozones/include/as2\_geozones/as2\_geozones.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/include/as2_geozones/as2_geozones.hpp)
-   [as2\_utilities/as2\_geozones/src/as2\_geozones.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/src/as2_geozones.cpp)
-   [as2\_utilities/as2\_geozones/launch/as2\_geozones\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_utilities/as2_geozones/launch/as2_geozones_launch.py)

## 8\. Conclusion

The AS2 Core and Messages system forms the foundation of the Aerostack2 framework. It provides:

1.  Base classes for creating platform-specific implementations
2.  A comprehensive sensor framework for handling different types of sensors
3.  Standard message definitions for communication between components
4.  Utility functions for coordinate transformations and launch configurations

Platform-specific implementations can be built by extending the `as2::AerialPlatform` class and implementing the required virtual methods.