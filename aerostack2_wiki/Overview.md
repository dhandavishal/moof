# aerostack2/aerostack2 | DeepWiki
Overview
--------

Relevant source files

*   .github/ISSUE\_TEMPLATE/BUG-REPORT.yml
*   .github/ISSUE\_TEMPLATE/FEATURE-REQUEST.yml
*   .github/PULL\_REQUEST\_TEMPLATE.md
*   .github/PULL\_REQUEST\_TEMPLATE/PULL\_REQUEST\_TEMPLATE.yml
*   .github/workflows/build-humble.yaml
*   .github/workflows/codecov\_test.yaml
*   README.md
*   aerostack2/package.xml
*   as2\_behavior\_tree/package.xml
*   as2\_behaviors/as2\_behavior/package.xml
*   as2\_behaviors/as2\_behaviors\_motion/package.xml
*   as2\_behaviors/as2\_behaviors\_perception/package.xml
*   as2\_behaviors/as2\_behaviors\_platform/package.xml
*   as2\_behaviors/as2\_behaviors\_trajectory\_generation/README.md
*   as2\_behaviors/as2\_behaviors\_trajectory\_generation/package.xml
*   as2\_cli/package.xml
*   as2\_motion\_controller/.gitignore
*   as2\_motion\_controller/README.md
*   as2\_motion\_reference\_handlers/package.xml
*   as2\_msgs/package.xml
*   as2\_python\_api/package.xml
*   as2\_python\_api/setup.py
*   codecov.yaml

Aerostack2 (AS2) is a comprehensive framework built on ROS 2 for developing autonomous aerial robot systems. It provides developers with a modular, platform-independent architecture designed to simplify the creation of complex drone applications, including multi-robot systems.

Purpose and Scope
-----------------

Aerostack2 aims to provide a complete framework for aerial robotics that handles everything from low-level control to high-level autonomous behaviors. This overview introduces the framework's architecture and key components. For installation instructions and basic usage, see Getting Started.

System Architecture
-------------------

The following diagram illustrates the high-level architecture of Aerostack2:

```

```


Sources: README.md5-21 aerostack2/package.xml1-45

The Aerostack2 framework is organized into five main subsystems, each providing distinct functionality while maintaining clear interfaces with other components:

### Core System

The core system provides fundamental functionality for the framework:

*   **AS2 Core (`as2_core`)**: Base classes, utilities, and common functionality used throughout the framework
*   **AS2 Messages (`as2_msgs`)**: Custom message types for inter-component communication
*   **State Estimator (`as2_state_estimator`)**: Provides aerial platform state information (position, velocity, orientation)
*   **Motion Controller (`as2_motion_controller`)**: Implements control algorithms for different flight modes
*   **Motion Reference Handlers (`as2_motion_reference_handlers`)**: Manages motion references and interfaces with controllers

Sources: as2\_msgs/package.xml1-36 as2\_motion\_reference\_handlers/package.xml1-36

### Behaviors System

The behaviors system implements higher-level drone capabilities through a standardized behavior interface:

```

```


*   **Behavior Base (`as2_behavior`)**: Defines the behavior server architecture and common interface
*   **Motion Behaviors (`as2_behaviors_motion`)**: Implements basic movements like takeoff, landing, and path following
*   **Trajectory Generation (`as2_behaviors_trajectory_generation`)**: Generates smooth trajectories for advanced motion
*   **Perception Behaviors (`as2_behaviors_perception`)**: Handles sensor-based behaviors like marker detection
*   **Platform Behaviors (`as2_behaviors_platform`)**: Manages platform-specific operations
*   **Behavior Tree (`as2_behavior_tree`)**: Enables composition of complex behaviors from simple ones

Sources: as2\_behavior/package.xml1-29 as2\_behaviors\_motion/package.xml1-31 as2\_behaviors\_trajectory\_generation/package.xml1-37 as2\_behaviors\_platform/package.xml1-28 as2\_behaviors\_perception/package.xml1-34 as2\_behavior\_tree/package.xml1-33

### Platform Abstraction

The platform abstraction layer enables hardware independence:

*   **Aerial Platform Base**: Abstract interface for different drone platforms
*   **Gazebo Platform (`as2_platform_gazebo`)**: Implementation for the Gazebo simulator
*   **Multirotor Simulator (`as2_platform_multirotor_simulator`)**: Simple simulator for testing without Gazebo
*   **Real Platforms**: Implementations for various physical drone platforms (e.g., PX4, Crazyflie, Tello, DJI)

Sources: .github/workflows/build-humble.yaml77-108

### User Interfaces

User interfaces provide ways to interact with and control drones:

*   **Python API (`as2_python_api`)**: Programmatic interface for controlling drones and defining missions
*   **Mission Interpreter**: Translates high-level mission specifications into behaviors
*   **Keyboard Teleoperation (`as2_keyboard_teleoperation`)**: Manual control interface using keyboard
*   **Command Line Interface (`as2_cli`)**: Terminal-based drone control

Sources: as2\_python\_api/package.xml1-32 as2\_python\_api/setup.py1-27 as2\_cli/package.xml1-18

### Simulation

Simulation components enable testing without physical hardware:

*   **Gazebo Assets (`as2_gazebo_assets`)**: Models and plugins for Gazebo simulation
*   **Ignition Assets**: Assets for Ignition Gazebo simulation
*   **Drone/Payload Models**: Specifications for various drone and payload configurations

Data Flow
---------

This diagram illustrates how data flows through the system during drone operations:

```

```


User commands flow through the mission interpreter and behaviors to the motion controller, while sensor data is processed by the state estimator to inform motion control decisions.

Key Features
------------

Aerostack2 provides the following key features:



* Feature: Modularity
  * Description: Components can be changed or interchanged without affecting the rest of the system
* Feature: Platform Independence
  * Description: Abstract interfaces enable code to run on different drone platforms without modification
* Feature: Project Orientation
  * Description: Allows installing and using only necessary packages for specific applications
* Feature: Swarming Capabilities
  * Description: Designed to support multi-robot operations and coordination
* Feature: ROS 2 Native
  * Description: Built from the ground up on ROS 2, taking advantage of its improved communication and security features
* Feature: Behavior System
  * Description: Provides high-level autonomous behaviors through a flexible behavior framework
* Feature: Simulation Integration
  * Description: Seamless transition between simulation and real hardware


Sources: README.md16-22

Package Organization
--------------------

Aerostack2 is organized as a meta-package containing multiple ROS 2 packages, each responsible for specific functionality:

```

```


Sources: aerostack2/package.xml12-39 .github/workflows/codecov\_test.yaml27-54

For More Information
--------------------

For more detailed information about specific subsystems, refer to the following pages:

*   Architecture details: Architecture
*   Installation and basic usage: Getting Started
*   Core components: Core System
*   Behavior framework: Behavior System
*   Simulation capabilities: Simulation
*   User interfaces: User Interfaces
*   Platform implementations: Platform Implementations