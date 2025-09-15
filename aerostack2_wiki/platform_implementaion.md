Relevant source files

-   [as2\_aerial\_platforms/as2\_platform\_gazebo/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/CMakeLists.txt)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/package.xml)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo\_node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo_node.cpp)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/tests/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/tests/CMakeLists.txt)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/tests/as2\_platform\_gazebo\_gtest.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/tests/as2_platform_gazebo_gtest.cpp)
-   [as2\_aerial\_platforms/as2\_platform\_multirotor\_simulator/src/as2\_platform\_multirotor\_simulator\_node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_multirotor_simulator/src/as2_platform_multirotor_simulator_node.cpp)
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

This page documents the Platform Implementations in Aerostack2, which provide the hardware abstraction layer that enables the framework to interface with different drone types, both in simulation and with real hardware. The Platform Implementations serve as the bridge between Aerostack2's high-level functions and the specific hardware or simulation environment being used.

For information about the Aerial Platform Base class that these implementations extend, see [Aerial Platform Base](https://deepwiki.com/aerostack2/aerostack2/6.1-aerial-platform-base). For information about the Gazebo simulation environment specifically, see [Gazebo Platform](https://deepwiki.com/aerostack2/aerostack2/6.2-gazebo-platform).

## Overview of Platform Architecture

The Platform Implementations in Aerostack2 follow an object-oriented design, with a common base class (`AerialPlatform`) that defines the interface and shared functionality, and specific derived classes that implement this interface for different hardware or simulation environments.

```
AerialPlatform+initialize()+resetPlatform()+configureSensors()+ownSendCommand()+ownSetArmingState(bool)+ownSetOffboardControl(bool)+ownSetPlatformControlMode(ControlMode)+ownKillSwitch()+ownStopPlatform()+ownTakeoff()+ownLand()GazeboPlatform+configureSensors()+ownSendCommand()+ownSetArmingState(bool)+ownSetOffboardControl(bool)+ownSetPlatformControlMode(ControlMode)+ownKillSwitch()+ownStopPlatform()+ownTakeoff()+ownLand()MultirotorSimulatorPlatform+configureSensors()+ownSendCommand()+ownSetArmingState(bool)+ownSetOffboardControl(bool)+ownSetPlatformControlMode(ControlMode)+ownKillSwitch()+ownStopPlatform()+ownTakeoff()+ownLand()RealHardwarePlatform+configureSensors()+ownSendCommand()+ownSetArmingState(bool)+ownSetOffboardControl(bool)+ownSetPlatformControlMode(ControlMode)+ownKillSwitch()+ownStopPlatform()+ownTakeoff()+ownLand()
```

Sources:

-   [as2\_core/include/as2\_core/aerial\_platform.hpp76-409](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L76-L409)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp37-110](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L37-L110)

## Platform Base and State Management

The `AerialPlatform` base class handles common functionality including:

1.  **State Machine Management**: Managing the platform's state (disarmed, armed, taking off, flying, etc.)
2.  **Command Subscription**: Subscribing to command topics for trajectory, pose, twist, and thrust
3.  **Service Interface**: Providing services for arming, offboard control, takeoff, landing, etc.
4.  **Control Mode Management**: Handling different control modes (position, velocity, attitude, etc.)

```
ARMDISARMTAKE_OFFTOOK_OFFLANDLANDEDEMERGENCYEMERGENCYEMERGENCYEMERGENCYEMERGENCYDISARMEDLANDEDTAKING_OFFFLYINGLANDINGEmergency
```

Sources:

-   [as2\_core/src/platform\_state\_machine.cpp103-167](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/platform_state_machine.cpp#L103-L167)
-   [as2\_core/tests/platform\_state\_machine\_test.cpp45-86](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/tests/platform_state_machine_test.cpp#L45-L86)

## Platform Communication Flow

The platform implementations serve as the interface between the Aerostack2 framework and the actual hardware or simulation environment. They translate high-level commands into specific hardware commands and publish platform state and sensor data.

```
External EnvironmentPlatform AbstractionAerostack2 Coreactuator_command/poseactuator_command/twistactuator_command/thrustactuator_command/trajectoryStateEventsControl ModePlatform StatusVirtual Method CallsHardware-specific CommandsState UpdatesSensor Datasensor_measurements/*platform/infoplatform/infoMotion ControllersBehaviorsState MachineAerialPlatform Base ClassPlatform Implementation(GazeboPlatform, etc.)Hardware/SimulatorSensors
```

Sources:

-   [as2\_core/src/aerial\_platform.cpp68-162](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L68-L162)
-   [as2\_core/include/as2\_core/names/topics.hpp38-114](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/names/topics.hpp#L38-L114)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp92-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L92-L124)

## Available Platform Implementations

Aerostack2 includes several platform implementations for different environments:

### 1\. Gazebo Platform

The Gazebo Platform implementation (`GazeboPlatform`) interfaces with the Gazebo simulator, allowing Aerostack2 to control simulated drones in the Gazebo environment.

Key features:

-   Translates Aerostack2 commands into Gazebo-specific messages
-   Handles control modes for position, velocity, attitude, etc.
-   Supports simulated takeoff and landing
-   Provides bridging between ROS2 and Gazebo

Sources:

-   [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp61-106](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L61-L106)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp41-364](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L41-L364)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py29-148](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L29-L148)

### 2\. Multirotor Simulator Platform

A simpler built-in simulator for testing and development purposes, not requiring a full Gazebo installation.

Sources:

-   [as2\_aerial\_platforms/as2\_platform\_multirotor\_simulator/src/as2\_platform\_multirotor\_simulator\_node.cpp37-50](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_multirotor_simulator/src/as2_platform_multirotor_simulator_node.cpp#L37-L50)

## Platform Implementation Details

### Command Handling Flow

The platform implementations receive commands through ROS topics and services, process them, and then send appropriate commands to the hardware or simulator.

```
"Hardware/Simulator""Platform Implementation""AerialPlatform""Client (Behavior/Controller)""Hardware/Simulator""Platform Implementation""AerialPlatform""Client (Behavior/Controller)"Set Control ModeownSetPlatformControlMode()Hardware-specific Control ModeArm CommandownSetArmingState(true)Hardware-specific Arm CommandTakeoff CommandownTakeoff()Hardware-specific TakeoffPosition/Velocity/Trajectory CommandUpdate Command MessageownSendCommand()Hardware-specific CommandLand CommandownLand()Hardware-specific LandDisarm CommandownSetArmingState(false)Hardware-specific Disarm
```

Sources:

-   [as2\_core/src/aerial\_platform.cpp192-271](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L192-L271)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp92-164](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L92-L164)

### Control Mode Management

Platforms support various control modes that define how commands are interpreted:

| Control Mode | Description | Frame Reference Options |
| --- | --- | --- |
| HOVER | Maintain position | N/A |
| ACRO | Direct control of angular rates and thrust | N/A |
| ATTITUDE | Control roll, pitch, yaw angles | N/A |
| SPEED | Control linear velocity | BODY\_FLU\_FRAME, LOCAL\_ENU\_FRAME |
| SPEED\_IN\_A\_PLANE | Control speed in a 2D plane | BODY\_FLU\_FRAME, LOCAL\_ENU\_FRAME |
| POSITION | Control position coordinates | BODY\_FLU\_FRAME, LOCAL\_ENU\_FRAME, GLOBAL\_LAT\_LONG\_ASML |
| TRAJECTORY | Follow trajectory setpoints | BODY\_FLU\_FRAME, LOCAL\_ENU\_FRAME, GLOBAL\_LAT\_LONG\_ASML |

Sources:

-   [as2\_core/src/utils/control\_mode\_utils.cpp48-142](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/utils/control_mode_utils.cpp#L48-L142)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp137-144](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L137-L144)

## Sensor Integration

Platform implementations can integrate with various sensors using the Aerostack2 sensor framework, which provides standardized interfaces for different sensor types.

```
GenericSensor+dataUpdated()+publishData()SensorData<T>+setData(T)+publish()+updateAndPublish(T)Sensor<T>+updateData(T)+publishData()TFStatic+setStaticTransform()TFDynamic+setDynamicTransform()Camera+updateData(Image)+setCameraInfo()+setCameraLinkTransform()GroundTruth+updateData(PoseStamped)+updateData(TwistStamped)Gimbal+updateData(PoseStamped)+setGimbalBaseTransform()
```

Sources:

-   [as2\_core/include/as2\_core/sensor.hpp75-795](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp#L75-L795)
-   [as2\_core/src/sensor.cpp40-520](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/sensor.cpp#L40-L520)

## Extending with Custom Platform Implementations

To create a new platform implementation for a specific hardware or simulator:

1.  Create a new class that inherits from `as2::AerialPlatform`
2.  Implement all the required virtual methods:
    -   `configureSensors()`
    -   `ownSendCommand()`
    -   `ownSetArmingState(bool)`
    -   `ownSetOffboardControl(bool)`
    -   `ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &)`
    -   `ownKillSwitch()`
    -   `ownStopPlatform()`
    -   `ownTakeoff()` (optional)
    -   `ownLand()` (optional)
3.  Create the necessary publishers and subscribers to communicate with your hardware
4.  Create a launch file to configure and start your platform

Example using GazeboPlatform as a reference:

```
Create Platform Classclass MyPlatform : public as2::AerialPlatformImplement Required Methodsonfigure_sensors(), own_send_command(), etc.Create Hardware Interfacee.g., publishers/subscribers for your hardwareImplement State Managemente.g., arming, takeoff, landingCreate Launch Fileto configure and start platformCreate CMakeLists.txt and package.xmlto build and install your platform
```

Sources:

-   [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp61-106](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L61-L106)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp41-364](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L41-L364)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/CMakeLists.txt1-95](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/CMakeLists.txt#L1-L95)
-   [as2\_aerial\_platforms/as2\_platform\_gazebo/package.xml1-28](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/package.xml#L1-L28)

## Platform Services and Topics

Each platform implementation exposes a set of standard services and topics:

### Services

| Service Name | Type | Description |
| --- | --- | --- |
| `set_platform_control_mode` | `as2_msgs::srv::SetControlMode` | Set the control mode of the platform |
| `set_arming_state` | `std_srvs::srv::SetBool` | Arm or disarm the platform |
| `set_offboard_mode` | `std_srvs::srv::SetBool` | Enable or disable offboard control |
| `takeoff` | `std_srvs::srv::SetBool` | Execute takeoff |
| `land` | `std_srvs::srv::SetBool` | Execute landing |
| `list_control_modes` | `as2_msgs::srv::ListControlModes` | List available control modes |

### Topics

| Topic Name | Type | Direction | Description |
| --- | --- | --- | --- |
| `platform/info` | `as2_msgs::msg::PlatformInfo` | Published | Platform status information |
| `actuator_command/trajectory` | `as2_msgs::msg::TrajectorySetpoints` | Subscribed | Trajectory command |
| `actuator_command/pose` | `geometry_msgs::msg::PoseStamped` | Subscribed | Position command |
| `actuator_command/twist` | `geometry_msgs::msg::TwistStamped` | Subscribed | Velocity command |
| `actuator_command/thrust` | `as2_msgs::msg::Thrust` | Subscribed | Thrust command |
| `alert_event` | `as2_msgs::msg::AlertEvent` | Subscribed | Alert events (e.g., emergency) |

Sources:

-   [as2\_core/src/aerial\_platform.cpp68-156](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L68-L156)
-   [as2\_core/include/as2\_core/names/topics.hpp38-114](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/names/topics.hpp#L38-L114)

## Conclusion

The Platform Implementations in Aerostack2 provide a flexible and extensible way to interface with different types of drones and simulators. By implementing a common interface defined in the `AerialPlatform` base class, these implementations enable Aerostack2 to work with various hardware and simulation environments while maintaining a consistent API for higher-level components like controllers and behaviors.

The current implementations include the Gazebo Platform for interfacing with the Gazebo simulator and the Multirotor Simulator Platform for simpler simulation needs. Additional platform implementations can be created by extending the `AerialPlatform` base class and implementing the required methods for specific hardware.