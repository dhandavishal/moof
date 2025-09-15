Relevant source files

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

The Aerial Platform Base in Aerostack2 provides a core abstraction layer for different aerial platforms (drones). It serves as a base class that defines common functionality and interfaces that all specific platform implementations must implement. This page describes the architecture, state machine, and key components of the platform abstraction layer, which allows Aerostack2 to control various physical or simulated drones through a unified interface.

For information about specific platform implementations like Gazebo Platform, see [Gazebo Platform](https://deepwiki.com/aerostack2/aerostack2/6.2-gazebo-platform).

## Architecture Overview

```
AerialPlatform#command_trajectory_msg_#command_pose_msg_#command_twist_msg_#command_thrust_msg_#platform_info_msg_+initialize()+configureSensors()+ownSendCommand()+ownSetArmingState(bool state)+ownSetOffboardControl(bool offboard)+ownSetPlatformControlMode(ControlMode)+ownTakeoff()+ownLand()+ownKillSwitch()+ownStopPlatform()+setArmingState(bool state)+setOffboardControl(bool offboard)+setPlatformControlMode(ControlMode)+takeoff()+land()+alertEvent(AlertEvent)+sendCommand()Node+generate_global_name(string)+generate_local_name(string)PlatformStateMachine+setState()+getState()+processEvent()ConcreteAerialPlatform+configureSensors()+ownSendCommand()+ownSetArmingState(bool state)+ownSetOffboardControl(bool offboard)+ownSetPlatformControlMode(ControlMode)+ownTakeoff()+ownLand()+ownKillSwitch()+ownStopPlatform()
```

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp77-146](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L77-L146) [as2\_core/src/aerial\_platform.cpp44-168](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L44-L168)

The `AerialPlatform` class inherits from `as2::Node` and includes a `PlatformStateMachine` to handle state transitions. It defines both concrete methods that all platforms share and abstract methods (marked with \*) that must be implemented by derived classes.

### Platform Abstraction Design

The Aerial Platform Base uses an abstract class pattern to define common functionality while allowing for platform-specific implementations:

```
Platform StatePlatform InterfacePlatform AbstractionAerialPlatform (Abstract Base Class)PX4GazeboPlatformSimulatorPlatformPX4PlatformCrazyfliePlatformCommand SubscribersPlatform Info PublisherPlatform ServicesPlatform State MachineControl Modes
```

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp78-318](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L78-L318) [as2\_core/src/aerial\_platform.cpp164-173](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L164-L173)

## Platform State Machine

The state machine manages transitions between platform states to ensure safe operation:

```
ARMDISARMTAKE_OFFTOOK_OFFLANDLANDEDEMERGENCYEMERGENCYEMERGENCYEMERGENCYEMERGENCYDISARMEDLANDEDTAKING_OFFFLYINGLANDINGEMERGENCY
```

Sources: [as2\_core/src/platform\_state\_machine.cpp103-167](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/platform_state_machine.cpp#L103-L167)

The platform state machine handles the following states:

-   **DISARMED**: The platform is disarmed and motors are inactive
-   **LANDED**: The platform is armed but on the ground
-   **TAKING\_OFF**: The platform is in the process of taking off
-   **FLYING**: The platform is in flight and can receive movement commands
-   **LANDING**: The platform is in the process of landing
-   **EMERGENCY**: The platform has entered an emergency state

State transitions are triggered by specific events like ARM, DISARM, TAKE\_OFF, etc., which can be initiated through service calls or programmatically.

## Command Interface

The Aerial Platform Base handles four types of command interfaces:

```
Command TypesCommandsownSendCommand()Motion ControllerAerial Platform BaseTrajectorySetpointsPoseStampedTwistStampedThrustPlatform Hardware/Simulation
```

Sources: [as2\_core/src/aerial\_platform.cpp67-101](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L67-L101) [as2\_core/include/as2\_core/aerial\_platform.hpp93-97](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L93-L97)

The Aerial Platform subscribes to the following command topics:

-   **Trajectory Commands**: Setpoints for trajectory tracking
-   **Pose Commands**: Position and orientation setpoints
-   **Twist Commands**: Linear and angular velocity setpoints
-   **Thrust Commands**: Direct thrust control commands

These commands are processed and passed to the concrete platform implementation through the `ownSendCommand()` method.

## Control Modes

The platform supports various control modes that define how the platform interprets and executes commands:

| Control Mode | Description |
| --- | --- |
| UNSET | No control mode set |
| HOVER | Maintains position in place |
| ACRO | Direct control of angular rates |
| ATTITUDE | Controls roll, pitch, and yaw angles |
| SPEED | Controls linear velocity |
| SPEED\_IN\_A\_PLANE | Controls velocity in a 2D plane |
| POSITION | Controls position setpoints |
| TRAJECTORY | Follows trajectory setpoints |

Each control mode can be combined with different yaw modes (YAW\_ANGLE, YAW\_SPEED, NONE) and reference frames (BODY\_FLU\_FRAME, LOCAL\_ENU\_FRAME, GLOBAL\_LAT\_LONG\_ASML).

Sources: [as2\_core/src/utils/control\_mode\_utils.cpp48-216](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/utils/control_mode_utils.cpp#L48-L216)

## Platform Services

The Aerial Platform Base provides several services for controlling the platform:

```
Platform ServicesClientBehavior/MissionUser Commandset_platform_control_modeset_arming_stateset_offboard_modetakeofflandlist_control_modesAerial Platform
```

Sources: [as2\_core/src/aerial\_platform.cpp103-149](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L103-L149) [as2\_core/include/as2\_core/aerial\_platform.hpp339-407](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L339-L407)

These services enable:

-   Setting the platform control mode
-   Arming and disarming the platform
-   Enabling offboard control
-   Executing takeoff and landing operations
-   Listing available control modes

## Sensor Integration

The Aerial Platform Base provides a framework for integrating various sensors:

```
Sensor TypesAerial PlatformconfigureSensors()ImuGPSCameraOdometryLidarBatteryBarometerCompassRangeFinderGimbalGroundTruth
```

Sources: [as2\_core/include/as2\_core/sensor.hpp73-795](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp#L73-L795) [as2\_core/src/sensor.cpp46-517](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/sensor.cpp#L46-L517)

Each sensor type inherits from base classes that provide common functionality:

-   **TFStatic**: For publishing static transforms
-   **TFDynamic**: For publishing dynamic transforms
-   **SensorData**: For publishing sensor data on topics
-   **GenericSensor**: For periodic publishing of sensor data

Concrete platform implementations must override the `configureSensors()` method to set up the specific sensors available on that platform.

## Emergency Handling

The platform provides mechanisms for handling emergency situations:

```
Emergency HandlersExternal Emergency EventsKILL_SWITCHEMERGENCY_HOVEREmergency StopEmergency HoverAlertEventownKillSwitch()ownStopPlatform()Platform Motors
```

Sources: [as2\_core/src/aerial\_platform.cpp273-295](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L273-L295)

Emergency events can be triggered through the global `alert_event` topic and are handled by the platform's emergency methods:

-   `ownKillSwitch()`: Immediately stops motors (potentially causing the platform to fall)
-   `ownStopPlatform()`: Attempts to hover in place as best as possible

## Implementing a Custom Platform

To implement a custom aerial platform, create a class that inherits from `AerialPlatform` and implement all the required virtual methods:

| Required Method | Purpose |
| --- | --- |
| `configureSensors()` | Configure all sensors on the platform |
| `ownSendCommand()` | Send actuator commands to the platform |
| `ownSetArmingState()` | Handle platform-specific arming/disarming |
| `ownSetOffboardControl()` | Enable/disable offboard control mode |
| `ownSetPlatformControlMode()` | Set the control mode on the platform |
| `ownTakeoff()` | Implement platform-specific takeoff |
| `ownLand()` | Implement platform-specific landing |
| `ownKillSwitch()` | Handle emergency motor cutoff |
| `ownStopPlatform()` | Handle emergency hover |

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp118-184](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L118-L184)

## Platform Information

The platform regularly publishes information about its state:

```
PlatformInfo Contentsplatform_infoAerial PlatformPlatformInfo Messagearmed (bool)offboard (bool)connected (bool)current_control_modestatus (State)
```

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp328-333](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L328-L333) [as2\_core/src/aerial\_platform.cpp155-157](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L155-L157)

This information allows other components in the system to monitor the platform's state and make decisions accordingly.

The Aerial Platform Base is part of a larger architecture in Aerostack2:

-   It receives commands from higher-level components like Motion Controllers and Behaviors
-   It abstracts platform-specific details from the rest of the system
-   It provides standardized interfaces for sensors and actuators
-   It ensures safety through state machine enforcement and emergency handling

This abstraction allows Aerostack2 to support different hardware platforms and simulators through a consistent interface.