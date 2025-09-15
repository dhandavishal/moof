Relevant source files

-   [aerostack2/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/aerostack2/package.xml)
-   [as2\_behavior\_tree/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behavior_tree/package.xml)
-   [as2\_behaviors/as2\_behavior/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behavior/package.xml)
-   [as2\_behaviors/as2\_behaviors\_motion/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/package.xml)
-   [as2\_behaviors/as2\_behaviors\_perception/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/package.xml)
-   [as2\_behaviors/as2\_behaviors\_platform/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_platform/package.xml)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/package.xml)
-   [as2\_cli/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_cli/package.xml)
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
-   [as2\_motion\_reference\_handlers/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_reference_handlers/package.xml)
-   [as2\_msgs/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/package.xml)
-   [as2\_python\_api/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/package.xml)
-   [as2\_python\_api/setup.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/setup.py)

This document describes the high-level architecture of Aerostack2 (AS2), a comprehensive framework for developing autonomous aerial robot systems based on ROS 2. The architecture documentation presented here covers the major components and their interactions, providing a foundation for understanding how the system is organized and how data flows between components.

For specific information about getting started with Aerostack2, see [Getting Started](https://deepwiki.com/aerostack2/aerostack2/1.2-getting-started).

## System Overview

Aerostack2 is designed as a modular framework that provides a complete stack for controlling aerial robots. It abstracts platform-specific details while providing a rich set of behaviors and control capabilities.

```
User InterfacesMotion ControlBehavior SystemPlatform AbstractionCore SystemAS2 CoreAS2 MessagesAerial PlatformPlatform State MachineSensor FrameworkAerial Platform BaseGazebo PlatformReal PlatformsBehavior ServerBehavior TypesMotion BehaviorsPerception BehaviorsPlatform BehaviorsMotion Reference HandlersMotion ControllerTrajectory GenerationPython APIMission InterpreterCommand Line Interface
```

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp) [aerostack2/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/aerostack2/package.xml) [as2\_python\_api/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/package.xml) [as2\_behavior\_tree/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behavior_tree/package.xml) [as2\_behaviors/as2\_behavior/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behavior/package.xml)

## Core Components

The architecture is organized around several core components that provide the foundational functionality of the framework:

### Aerial Platform

The `AerialPlatform` class serves as the base abstraction for all aerial platforms (drones). It handles:

-   Platform state management through a state machine
-   Command processing and execution
-   Interface with hardware-specific implementations
-   Sensor management

```
inheritscontainsAerialPlatform-state_machine: PlatformStateMachine+initialize()+configureSensors()+ownSendCommand()+ownSetArmingState(bool)+ownSetOffboardControl(bool)+ownSetPlatformControlMode(ControlMode)+ownTakeoff()+ownLand()+ownKillSwitch()+ownStopPlatform()-resetPlatform()-sendCommand()-publishPlatformInfo()Node+generate_global_name(string)+generate_local_name(string)PlatformStateMachine+processEvent(Event)+setState(state)+getState()-defineTransitions()
```

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp) [as2\_core/src/aerial\_platform.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp) [as2\_core/src/platform\_state\_machine.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/platform_state_machine.cpp)

### Platform State Machine

The platform operates through a state machine that manages transitions between different operating states:

```
ARMDISARMTAKE_OFFTOOK_OFFLANDLANDEDEMERGENCYEMERGENCYEMERGENCYEMERGENCYEMERGENCYDISARMEDLANDEDTAKING_OFFFLYINGLANDINGEMERGENCY
```

Sources: [as2\_core/src/platform\_state\_machine.cpp103-167](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/platform_state_machine.cpp#L103-L167)

### Sensor Framework

Aerostack2 provides a comprehensive sensor framework that allows for easy integration of various sensor types:

```
inheritsinheritsinheritsinheritsinheritsinheritsinheritsinheritsinheritsinheritsTFStatic+setStaticTransform()TFDynamic+setDynamicTransform()GenericSensor+dataUpdated()+publishData()SensorData<T>+setData(T)+publish()+updateAndPublish(T)Sensor<T>+updateData(T)Camera+updateData(image)+setCameraInfo()+setEncoding()GroundTruth+updateData(pose)+updateData(twist)Gimbal+setGimbalBaseTransform()+updateData(pose)
```

Specialized sensor types include:

-   Odometry
-   IMU
-   GPS
-   Lidar
-   Camera
-   Gimbal
-   Ground Truth

Sources: [as2\_core/include/as2\_core/sensor.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/sensor.hpp) [as2\_core/src/sensor.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/sensor.cpp)

## Control Flow and Communication

Aerostack2 follows the ROS 2 communication paradigm with topics, services, and actions. The control flow is organized hierarchically:

```
Feedback LoopPlatform LevelMotion LevelBehavior LevelUser LevelUser Code/InterfacePython APIMissionBehavior ServerBehaviorsTrajectory GenerationMotion Reference HandlersMotion ControllerAerial PlatformPlatform Hardware/SimulationSensorsState Estimator
```

The communication between components follows a specific path:

1.  **User Commands**: Initiated through the Python API, CLI, or direct ROS 2 interfaces
2.  **Behavior Execution**: Processed through the behavior system
3.  **Motion References**: Generated either directly or through trajectory generators
4.  **Control Commands**: Generated by the motion controller
5.  **Platform Commands**: Sent to the platform-specific implementation

Sources: [as2\_core/src/aerial\_platform.cpp44-162](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp#L44-L162) [as2\_python\_api/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/package.xml)

## Topic Structure

Aerostack2 organizes ROS 2 topics in a hierarchical structure for clarity and organization:

| Topic Category | Base Name | Example Topics | Description |
| --- | --- | --- | --- |
| Sensor Measurements | `sensor_measurements/` | `sensor_measurements/imu`, `sensor_measurements/odom` | Raw sensor data |
| Self Localization | `self_localization/` | `self_localization/pose`, `self_localization/twist` | Estimated state |
| Motion Reference | `motion_reference/` | `motion_reference/pose`, `motion_reference/trajectory` | Desired robot state |
| Actuator Command | `actuator_command/` | `actuator_command/thrust`, `actuator_command/pose` | Commands to platform |
| Platform | `platform/` | `platform/info` | Platform status information |
| Ground Truth | `ground_truth/` | `ground_truth/pose`, `ground_truth/twist` | Simulation ground truth |

Sources: [as2\_core/include/as2\_core/names/topics.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/names/topics.hpp)

## Behavior System

The behavior system is a core abstraction that enables complex autonomous operation through composable behaviors:

```
implementsimplementsimplementsimplementsusesBehaviorServer+activate()+deactivate()+modify()+pause()+resume()MotionBehavior+takeoff()+land()+goTo()+followPath()PerceptionBehavior+pointGimbal()+detectAruco()PlatformBehavior+arm()+disarm()+setOffboard()TrajectoryGenerator+generatePolynomialTrajectory()
```

Behaviors are exposed as ROS 2 Action servers that can be called from client applications or sequenced in mission plans.

Sources: [as2\_behaviors/as2\_behavior/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behavior/package.xml) [as2\_behaviors/as2\_behaviors\_motion/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/package.xml) [as2\_behaviors/as2\_behaviors\_perception/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/package.xml) [as2\_behaviors/as2\_behaviors\_trajectory\_generation/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/package.xml)

## Platform Abstraction

One of the key features of Aerostack2 is its platform abstraction layer, which allows the same high-level code to run on different drone platforms:

```
Platform-Specific HardwarePlatform ImplementationsAbstract LayerextendsextendsextendsextendsAerialPlatformPX4 PlatformGazebo PlatformMultirotor SimulatorCustom PlatformPX4 HardwareGazebo PhysicsSimple PhysicsCustom Hardware
```

Platform implementations must override key methods from the `AerialPlatform` base class:

-   `ownSendCommand()`: Send the command to the specific platform
-   `ownSetArmingState()`: Handle platform-specific arming
-   `ownSetOffboardControl()`: Enable/disable offboard control
-   `ownSetPlatformControlMode()`: Set control mode
-   `ownTakeoff()`: Handle platform takeoff
-   `ownLand()`: Handle platform landing
-   `ownKillSwitch()`: Emergency stop
-   `ownStopPlatform()`: Safety hover

Sources: [as2\_core/include/as2\_core/aerial\_platform.hpp119-183](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp#L119-L183)

## User Interfaces

Aerostack2 provides several interfaces for interacting with the platform:

### Python API

The Python API provides a high-level, programmer-friendly interface to the entire framework:

```
ROS 2 SystemPython APIusescallssubscribes tosequencesaction requestsDroneInterfaceMissionInterpreterBehavior Action ClientsBehavior Action ServersPlatform ServicesState/Sensor Topics
```

### Command Line Interface

The CLI provides a quick way to execute common commands for testing and debugging:

```
Platform ServicesCLI Commandscallscallscallscallsas2 armas2 takeoffas2 landas2 gotoset_arming_statetakeofflandset_control_mode
```

Sources: [as2\_python\_api/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/package.xml) [as2\_cli/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_cli/package.xml)

## Extensibility Model

Aerostack2 is designed to be extensible in several ways:

1.  **New Platforms**: By extending the `AerialPlatform` class
2.  **New Sensors**: Using the sensor framework
3.  **New Behaviors**: By implementing the `BehaviorServer` interface
4.  **New Controllers**: By creating custom controllers
5.  **New User Interfaces**: By using the ROS 2 interfaces

This extensible architecture allows Aerostack2 to be adapted to a wide range of aerial robotics applications, from research to industrial use cases.

## Summary

Aerostack2 is built on a modular, layered architecture that abstracts the complexities of aerial robot control while providing powerful capabilities for autonomous behavior. The key architectural patterns include:

1.  **Layered Abstraction**: From hardware to behaviors to missions
2.  **Component-Based Design**: Modular components with clear interfaces
3.  **State Machine Control**: For robust platform state management
4.  **Behavior-Based Programming**: For complex autonomous operations
5.  **Platform Independence**: For hardware flexibility

This architecture enables developers to create sophisticated autonomous aerial robotics applications while maintaining flexibility and extensibility.

Sources: [aerostack2/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/aerostack2/package.xml) [as2\_core/include/as2\_core/aerial\_platform.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/include/as2_core/aerial_platform.hpp) [as2\_core/src/aerial\_platform.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_core/src/aerial_platform.cpp)