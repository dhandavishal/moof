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
-   [as2\_msgs/action/PointGimbal.action](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/action/PointGimbal.action)
-   [as2\_msgs/msg/GimbalControl.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/GimbalControl.msg)

This page documents the integration between Aerostack2 and the Gazebo simulator, focusing on the `as2_platform_gazebo` package which provides the interface for controlling drones in the Gazebo simulation environment. For information about drone and sensor models used in simulation, see [Drone and Sensor Models](https://deepwiki.com/aerostack2/aerostack2/4.1-drone-and-sensor-models).

## Overview

The Gazebo integration in Aerostack2 enables simulation-based development and testing of drone applications before deployment to real hardware. It implements the `AerialPlatform` interface to provide a consistent API regardless of whether code is running in simulation or on real hardware.

### Architecture

The following diagram illustrates the high-level architecture of the Gazebo integration in Aerostack2:

```
Gazebo SimulatorBridges Systemas2_platform_gazeboAerostack2 Frameworkas2_core (AerialPlatform)Behavior SystemMotion ControllerGazeboPlatform classas2_platform_gazebo_nodeROS<->Gazebo Bridgesdrone_bridges.pyGazebo Physics EngineDrone Model (SDF)Simulation World
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp) [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py)

## Platform Implementation

The Gazebo platform is implemented in the `GazeboPlatform` class, which inherits from the `as2::AerialPlatform` base class. This ensures that the platform-specific implementation follows the same interface as other platform implementations.

### Class Structure

```
AerialPlatform+sendCommand()+setArmingState()+setOffboardControl()+setPlatformControlMode()+killSwitch()+stopPlatform()+takeoff()+land()GazeboPlatform-twist_pub_-arm_pub_-acro_pub_-twist_state_sub_-reset_srv_-control_in_-yaw_rate_limit_-enable_takeoff_-enable_land_+GazeboPlatform(options)+ownSendCommand()+ownSetArmingState(state)+ownSetOffboardControl(offboard)+ownSetPlatformControlMode(control_in)+ownKillSwitch()+ownStopPlatform()+ownTakeoff()+ownLand()-resetCommandTwistMsg()-state_callback(twist_msg)-reset_callback(request, response)
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp60-106](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L60-L106)

### Control Mode Implementation

The Gazebo platform supports different control modes, which are handled in the `ownSendCommand()` method. The following diagram shows how different control modes are processed:

```
ACROHOVEROthersownSendCommand()Control Mode?Create ACRO messagePublish to acro_pub_Create zero twist messagePublish to twist_pub_Limit yaw rate if neededPublish twist message to twist_pub_Return true
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp92-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L92-L124)

## Communication with Gazebo

The Gazebo platform communicates with the Gazebo simulator through a set of ROS topics that are handled by bridges. The platform node publishes commands to these topics, which are then translated to Gazebo actions by the bridges.

### Message Flow

```
"Gazebo Simulator""ROS-Gazebo Bridge""GazeboPlatform""Behavior or Controller""Gazebo Simulator""ROS-Gazebo Bridge""GazeboPlatform""Behavior or Controller"alt[ACRO Control Mode][HOVER Control Mode][Other Control Modes]Control CommandPublish to acro_topicPublish zero twist to cmd_vel_topicPublish twist to cmd_vel_topicConvert ROS message to Gazebo actionApply physicsState updateState information (via topics)State feedback
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp65-73](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L65-L73) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp92-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L92-L124)

## Configuration

The Gazebo platform is configured through several parameters that can be set in the launch file or via configuration files.

### Required Parameters

| Parameter | Description | Default |
| --- | --- | --- |
| `cmd_vel_topic` | Topic for sending velocity commands | `/gz/{namespace}/cmd_vel` |
| `arm_topic` | Topic for sending arm/disarm commands | `/gz/{namespace}/arm` |
| `acro_topic` | Topic for sending acrobatic commands | `/gz/{namespace}/acro` |
| `control_modes_file` | File defining allowed control modes | `config/control_modes.yaml` |
| `enable_takeoff_platform` | Enable takeoff handling in platform | `false` |
| `enable_land_platform` | Enable landing handling in platform | `false` |

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp43-64](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L43-L64) [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py65-95](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L65-L95)

## Launch Files

The package provides a launch file for starting the Gazebo platform node with the appropriate configuration.

### platform\_gazebo\_launch.py

This launch file sets up the Gazebo platform node and optionally creates the necessary bridges between ROS and Gazebo.

```
Launch ActionsLaunch Configurationplatform_config_file.yamlcontrol_modes.yamlas2_platform_gazebo_nodedrone_bridges.py
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py102-147](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L102-L147)

## Special Features

### Takeoff and Landing

The Gazebo platform can optionally handle takeoff and landing internally, which is useful for debugging purposes. When enabled, the platform uses TF transformations to track the drone's height and controls its vertical movement until the desired height is reached or it lands.

```
Takeoff ProcessfalsetrueNoYesownTakeoff()enable_takeoff_?Return falseInitialize TF handlerInitialize callbacksWait for state informationSend upward velocity commandsReached desiredheight?Send hover commandClean up resourcesReturn true
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp166-247](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L166-L247)

Similar logic applies to the landing process, with the difference that it sends downward velocity commands and checks when the drone has reached the ground or stopped moving.

### Platform Reset

The platform provides a service to reset the platform state, which disarms the drone and resets the platform to its initial state.

```
/platform/state_machine/_reset
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp74-77](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L74-L77) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp355-362](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L355-L362)

## Integration with Aerostack2

The Gazebo platform is fully integrated with the rest of the Aerostack2 framework, allowing it to be used seamlessly with the behavior system, motion controllers, and other components.

```
SimulationPlatform LayerCore ComponentsHigh-Level ComponentsUser InterfacesPython APICommand Line InterfaceBehavior SystemMission InterpreterMotion ControllerState EstimatorAerialPlatform BaseGazeboPlatformROS-Gazebo BridgesGazebo Simulator
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp60-63](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L60-L63) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp41-42](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L41-L42)

## Usage Example

To use the Gazebo platform, you need to:

1.  Launch the Gazebo simulator with your desired world and drone models
2.  Start the Gazebo platform node using the provided launch file
3.  Interact with the platform using the Aerostack2 API

This can be done with the following command:

```
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone0
```

For more complex setups involving multiple drones or custom worlds, you would typically create a custom launch file that includes both the Gazebo simulator and the platform nodes.