## Overview

Relevant source files

-   [.github/ISSUE\_TEMPLATE/BUG-REPORT.yml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/BUG-REPORT.yml)
-   [.github/ISSUE\_TEMPLATE/FEATURE-REQUEST.yml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/ISSUE_TEMPLATE/FEATURE-REQUEST.yml)
-   [.github/PULL\_REQUEST\_TEMPLATE.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE.md)
-   [.github/PULL\_REQUEST\_TEMPLATE/PULL\_REQUEST\_TEMPLATE.yml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/PULL_REQUEST_TEMPLATE/PULL_REQUEST_TEMPLATE.yml)
-   [.github/workflows/build-humble.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml)
-   [.github/workflows/codecov\_test.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml)
-   [README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md)
-   [aerostack2/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/aerostack2/package.xml)
-   [as2\_behavior\_tree/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behavior_tree/package.xml)
-   [as2\_behaviors/as2\_behavior/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behavior/package.xml)
-   [as2\_behaviors/as2\_behaviors\_motion/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/package.xml)
-   [as2\_behaviors/as2\_behaviors\_perception/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/package.xml)
-   [as2\_behaviors/as2\_behaviors\_platform/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_platform/package.xml)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/README.md)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/package.xml)
-   [as2\_cli/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_cli/package.xml)
-   [as2\_motion\_controller/.gitignore](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_controller/.gitignore)
-   [as2\_motion\_controller/README.md](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_controller/README.md)
-   [as2\_motion\_reference\_handlers/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_reference_handlers/package.xml)
-   [as2\_msgs/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/package.xml)
-   [as2\_python\_api/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/package.xml)
-   [as2\_python\_api/setup.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/setup.py)
-   [codecov.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/codecov.yaml)

Aerostack2 (AS2) is a comprehensive framework built on ROS 2 for developing autonomous aerial robot systems. It provides developers with a modular, platform-independent architecture designed to simplify the creation of complex drone applications, including multi-robot systems.

## Purpose and Scope

Aerostack2 aims to provide a complete framework for aerial robotics that handles everything from low-level control to high-level autonomous behaviors. This overview introduces the framework's architecture and key components. For installation instructions and basic usage, see [Getting Started](https://deepwiki.com/aerostack2/aerostack2/1.2-getting-started).

## System Architecture

The following diagram illustrates the high-level architecture of Aerostack2:

```
SimulationUser InterfacesPlatform AbstractionBehaviors SystemCore SystemAS2 CoreAS2 MessagesState EstimatorMotion ControllerMotion Reference HandlersBehavior ServerMotion BehaviorsTrajectory GenerationPerception BehaviorsPlatform BehaviorsBehavior TreeAerial Platform BaseGazebo PlatformMultirotor SimulatorReal PlatformsPython APIMission InterpreterKeyboard TeleoperationCommand Line InterfaceGazebo AssetsIgnition AssetsDrone/Payload Models
```

Sources: [README.md5-21](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L5-L21) [aerostack2/package.xml1-45](https://github.com/aerostack2/aerostack2/blob/10200cd9/aerostack2/package.xml#L1-L45)

The Aerostack2 framework is organized into five main subsystems, each providing distinct functionality while maintaining clear interfaces with other components:

### Core System

The core system provides fundamental functionality for the framework:

-   **AS2 Core (`as2_core`)**: Base classes, utilities, and common functionality used throughout the framework
-   **AS2 Messages (`as2_msgs`)**: Custom message types for inter-component communication
-   **State Estimator (`as2_state_estimator`)**: Provides aerial platform state information (position, velocity, orientation)
-   **Motion Controller (`as2_motion_controller`)**: Implements control algorithms for different flight modes
-   **Motion Reference Handlers (`as2_motion_reference_handlers`)**: Manages motion references and interfaces with controllers

Sources: [as2\_msgs/package.xml1-36](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/package.xml#L1-L36) [as2\_motion\_reference\_handlers/package.xml1-36](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_motion_reference_handlers/package.xml#L1-L36)

### Behaviors System

The behaviors system implements higher-level drone capabilities through a standardized behavior interface:

```
BehaviorServer+on_activate()+on_modify()+on_deactivate()+on_pause()+on_resume()+on_run()TakeoffBehaviorLandBehaviorGoToBehaviorFollowPathBehaviorPointGimbalBehaviorDetectMarkersBehaviorTrajectoryGenerator[as2\_behaviors\_trajectory\_generation/package.xml1-37](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors_trajectory_generation/package.xml#L1-L37) [as2\_behaviors\_platform/package.xml1-28](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors_platform/package.xml#L1-L28) [as2\_behaviors\_perception/package.xml1-34](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors_perception/package.xml#L1-L34) [as2\_behavior\_tree/package.xml1-33](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behavior_tree/package.xml#L1-L33)

### Platform Abstraction

The platform abstraction layer enables hardware independence:

-   **Aerial Platform Base**: Abstract interface for different drone platforms
-   **Gazebo Platform (`as2_platform_gazebo`)**: Implementation for the Gazebo simulator
-   **Multirotor Simulator (`as2_platform_multirotor_simulator`)**: Simple simulator for testing without Gazebo
-   **Real Platforms**: Implementations for various physical drone platforms (e.g., PX4, Crazyflie, Tello, DJI)

Sources: [.github/workflows/build-humble.yaml77-108](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/build-humble.yaml#L77-L108)

### User Interfaces

User interfaces provide ways to interact with and control drones:

-   **Python API (`as2_python_api`)**: Programmatic interface for controlling drones and defining missions
-   **Mission Interpreter**: Translates high-level mission specifications into behaviors
-   **Keyboard Teleoperation (`as2_keyboard_teleoperation`)**: Manual control interface using keyboard
-   **Command Line Interface (`as2_cli`)**: Terminal-based drone control

Sources: [as2\_python\_api/package.xml1-32](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/package.xml#L1-L32) [as2\_python\_api/setup.py1-27](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/setup.py#L1-L27) [as2\_cli/package.xml1-18](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_cli/package.xml#L1-L18)

### Simulation

Simulation components enable testing without physical hardware:

-   **Gazebo Assets (`as2_gazebo_assets`)**: Models and plugins for Gazebo simulation
-   **Ignition Assets**: Assets for Ignition Gazebo simulation
-   **Drone/Payload Models**: Specifications for various drone and payload configurations

## Data Flow

This diagram illustrates how data flows through the system during drone operations:

```
EnvironmentCore FrameworkUser InteractionUser/ClientMission InterpreterBehavior ActionsState EstimatorMotion ControllerAerial PlatformSensorsPhysical World
```

User commands flow through the mission interpreter and behaviors to the motion controller, while sensor data is processed by the state estimator to inform motion control decisions.

## Key Features

Aerostack2 provides the following key features:

| Feature | Description |
| --- | --- |
| **Modularity** | Components can be changed or interchanged without affecting the rest of the system |
| **Platform Independence** | Abstract interfaces enable code to run on different drone platforms without modification |
| **Project Orientation** | Allows installing and using only necessary packages for specific applications |
| **Swarming Capabilities** | Designed to support multi-robot operations and coordination |
| **ROS 2 Native** | Built from the ground up on ROS 2, taking advantage of its improved communication and security features |
| **Behavior System** | Provides high-level autonomous behaviors through a flexible behavior framework |
| **Simulation Integration** | Seamless transition between simulation and real hardware |

Sources: [README.md16-22](https://github.com/aerostack2/aerostack2/blob/10200cd9/README.md#L16-L22)

## Package Organization

Aerostack2 is organized as a meta-package containing multiple ROS 2 packages, each responsible for specific functionality:

```
ToolsUser InterfacesPlatformsBehaviorsCoreaerostack2as2_coreas2_msgsas2_motion_reference_handlersas2_motion_controlleras2_state_estimatoras2_behavioras2_behaviors_motionas2_behaviors_trajectory_generationas2_behaviors_perceptionas2_behaviors_platformas2_behaviors_path_planningas2_behavior_treeas2_platform_gazeboas2_platform_multirotor_simulatoras2_python_apias2_clias2_keyboard_teleoperationas2_gazebo_assetsas2_visualizationas2_rviz_pluginsas2_external_object_to_tfas2_map_serveras2_geozones
```

Sources: [aerostack2/package.xml12-39](https://github.com/aerostack2/aerostack2/blob/10200cd9/aerostack2/package.xml#L12-L39) [.github/workflows/codecov\_test.yaml27-54](https://github.com/aerostack2/aerostack2/blob/10200cd9/.github/workflows/codecov_test.yaml#L27-L54)

## For More Information

For more detailed information about specific subsystems, refer to the following pages:

-   Architecture details: [Architecture](https://deepwiki.com/aerostack2/aerostack2/1.1-architecture)
-   Installation and basic usage: [Getting Started](https://deepwiki.com/aerostack2/aerostack2/1.2-getting-started)
-   Core components: [Core System](https://deepwiki.com/aerostack2/aerostack2/2-core-system)
-   Behavior framework: [Behavior System](https://deepwiki.com/aerostack2/aerostack2/3-behavior-system)
-   Simulation capabilities: [Simulation](https://deepwiki.com/aerostack2/aerostack2/4-simulation)
-   User interfaces: [User Interfaces](https://deepwiki.com/aerostack2/aerostack2/5-user-interfaces)
-   Platform implementations: [Platform Implementations](https://deepwiki.com/aerostack2/aerostack2/6-platform-implementations)