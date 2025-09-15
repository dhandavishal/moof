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

The Gazebo Platform is a component within Aerostack2 that provides an interface between the Aerostack2 framework and the Gazebo simulator. It implements the abstract `AerialPlatform` base class to allow aerial robots to be simulated in Gazebo while maintaining compatibility with the Aerostack2 control architecture. For information about the base Aerial Platform abstraction, see [Aerial Platform Base](https://deepwiki.com/aerostack2/aerostack2/6.1-aerial-platform-base).

## Overview and Architecture

The Gazebo Platform serves as a bridge connecting Aerostack2's control systems to simulated drones in Gazebo. It translates high-level commands from behaviors and controllers into the format required by Gazebo, and converts simulator state information back into Aerostack2's standardized formats.

### Architecture Diagram

```
Simulation EnvironmentGazebo PlatformAerostack2 FrameworkState FeedbackState Topics(/tf, /twist)Behavior SystemMotion Controllersas2::AerialPlatform Base ClassGazeboPlatform NodeCommand Publishers- twist_pub_- acro_pub_- arm_pub_Service Providers- reset_srv_ROS-Gazebo BridgesGazebo SimulatorDrone Model
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp37-107](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L37-L107) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp37-164](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L37-L164)

### Class Structure

```
«Abstract»AerialPlatform+sendCommand()+setArmingState()+setOffboardControl()+setPlatformControlMode()+killSwitch()+stopPlatform()+takeoff()+land()GazeboPlatform-twist_pub_-arm_pub_-acro_pub_-twist_state_sub_-reset_srv_-control_in_-yaw_rate_limit_-enable_takeoff_-enable_land_-tf_handler_+GazeboPlatform()+configureSensors()+ownSendCommand()+ownSetArmingState()+ownSetOffboardControl()+ownSetPlatformControlMode()+ownKillSwitch()+ownStopPlatform()+ownTakeoff()+ownLand()-resetCommandTwistMsg()-state_callback()-reset_callback()
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/include/as2\_platform\_gazebo/as2\_platform\_gazebo.hpp37-107](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/include/as2_platform_gazebo/as2_platform_gazebo.hpp#L37-L107)

## Command Flow to Gazebo

The Gazebo Platform provides several control modes for commanding simulated drones:

1.  **Velocity Control**: Sends velocity commands through `twist_pub_` publisher
2.  **Acrobatic Control**: Sends angular rates and thrust through `acro_pub_` publisher
3.  **Hover Mode**: Sends zero velocities to maintain position

The platform handles translating different control modes to appropriate Gazebo commands:

```
Gazebo InterfaceGazeboPlatformControl Inputcontrol_mode == ACROcontrol_mode == HOVEROther modes/gz/{namespace}/cmd_vel/gz/{namespace}/acroPosition ModeVelocity ModeAcro ModeHover ModeownSendCommand()twist_pub_(geometry_msgs::msg::Twist)acro_pub_(as2_msgs::msg::Acro)Gazebo Simulator
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp92-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L92-L124)

## Core Features

### 1\. Control Mode Translation

The platform handles the translation between Aerostack2's control modes and Gazebo's command interfaces. This happens in the `ownSendCommand()` and `ownSetPlatformControlMode()` methods:

-   For ACRO mode, angular rates and thrust are sent
-   For HOVER mode, zero velocities are sent
-   For other modes, twist commands (linear and angular velocities) are sent

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp92-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L92-L124) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp137-144](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L137-L144)

### 2\. Arming and Safety Controls

The platform provides methods for arming/disarming the drone and emergency stopping:

-   `ownSetArmingState()`: Arms or disarms the simulated drone by publishing to the arm topic
-   `ownKillSwitch()`: Emergency stop function that disarms the drone
-   `ownStopPlatform()`: Stops the drone by sending zero velocities

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp126-133](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L126-L133) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp146-164](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L146-L164)

### 3\. Optional Platform Takeoff and Landing

The platform includes optional implementations for takeoff and landing operations:

-   `ownTakeoff()`: When enabled via the `enable_takeoff_platform` parameter, provides a direct implementation of the takeoff operation
-   `ownLand()`: When enabled via the `enable_land_platform` parameter, provides a direct implementation of the landing operation

These are primarily for debugging and testing purposes, as typically these operations would be handled by behavior nodes in Aerostack2.

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp166-247](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L166-L247) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp249-338](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L249-L338)

## Configuration and Usage

### Parameters

The Gazebo Platform accepts the following parameters:

| Parameter | Type | Description | Default |
| --- | --- | --- | --- |
| `cmd_vel_topic` | string | Topic for velocity commands | `/gz/{namespace}/cmd_vel` |
| `acro_topic` | string | Topic for acrobatic commands | `/gz/{namespace}/acro` |
| `arm_topic` | string | Topic for arming commands | `/gz/{namespace}/arm` |
| `enable_takeoff_platform` | bool | Enable platform-level takeoff | `false` |
| `enable_land_platform` | bool | Enable platform-level landing | `false` |
| `control_modes_file` | string | Path to control modes configuration | `config/control_modes.yaml` |

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp41-55](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L41-L55) [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py66-93](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L66-L93)

### Launch File Usage

The platform provides a launch file for easy deployment:

```
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone1
```

The launch file sets up:

1.  The Gazebo platform node with appropriate parameters
2.  ROS-Gazebo bridges through the `drone_bridges.py` launch file from `as2_gazebo_assets` package

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py102-147](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L102-L147)

## Integration with ROS-Gazebo Bridges

The Gazebo Platform relies on bridges between ROS 2 and Gazebo to function. These bridges are typically launched via the `drone_bridges.py` launch file from the `as2_gazebo_assets` package, which is included in the platform launch file.

```
"Gazebo Simulator""ROS-Gazebo Bridges""GazeboPlatform""Aerostack2 Framework""Gazebo Simulator""ROS-Gazebo Bridges""GazeboPlatform""Aerostack2 Framework"Communication SetupCommand FlowState FeedbackCreate Publishers(/gz/{namespace}/cmd_vel, /gz/{namespace}/arm, etc.)Convert ROS messages to Gazebo formatsendCommand(twist_msg)Publish to cmd_vel topicApply forces/velocities to modelModel state updatesPublish to ROS topics (/tf, /twist)State information available
```

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py136-142](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L136-L142)

## Implementation Details

### Node Initialization

The Gazebo Platform node is initialized in `as2_platform_gazebo_node.cpp`:

```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gazebo_platform::GazeboPlatform>();
  as2::spinLoop(node);
  rclcpp::shutdown();
  return 0;
}
```

The platform node inherits from `as2::AerialPlatform` which provides the base functionality for all platform implementations in Aerostack2.

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo\_node.cpp37-46](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo_node.cpp#L37-L46)

### Platform Reset Service

The platform provides a service to reset its state machine:

-   Service: `platform/state_machine/_reset`
-   Implementation: Disarms the drone and resets the platform state

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp75-77](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L75-L77) [as2\_aerial\_platforms/as2\_platform\_gazebo/src/as2\_platform\_gazebo.cpp355-362](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/src/as2_platform_gazebo.cpp#L355-L362)

## Usage Example

To use the Gazebo Platform in a simulation:

1.  Launch the Gazebo simulator with appropriate drone models
2.  Launch the Gazebo Platform node with correct namespace
3.  Launch other Aerostack2 components (state estimator, controller, etc.)
4.  Control the drone using behaviors or direct command interfaces

The Gazebo Platform handles the communication with the simulator, allowing the rest of the Aerostack2 framework to operate as if it were controlling a real drone.

Sources: [as2\_aerial\_platforms/as2\_platform\_gazebo/launch/platform\_gazebo\_launch.py1-147](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_aerial_platforms/as2_platform_gazebo/launch/platform_gazebo_launch.py#L1-L147)