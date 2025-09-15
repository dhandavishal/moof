Relevant source files

-   [as2\_msgs/msg/MissionUpdate.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/MissionUpdate.msg)
-   [as2\_python\_api/as2\_python\_api/behavior\_actions/behavior\_handler.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/behavior_actions/behavior_handler.py)
-   [as2\_python\_api/as2\_python\_api/drone\_interface.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface.py)
-   [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py)
-   [as2\_python\_api/as2\_python\_api/drone\_interface\_gps.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_gps.py)
-   [as2\_python\_api/as2\_python\_api/drone\_interface\_teleop.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_teleop.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_stack.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_stack.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py)
-   [as2\_python\_api/as2\_python\_api/modules/dummy\_module.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/dummy_module.py)
-   [as2\_python\_api/as2\_python\_api/modules/module\_base.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/module_base.py)
-   [as2\_python\_api/as2\_python\_api/test/behaviors/behavior\_client.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/behaviors/behavior_client.py)
-   [as2\_python\_api/as2\_python\_api/test/modules/modules\_as\_properties.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/modules/modules_as_properties.py)
-   [as2\_python\_api/as2\_python\_api/test/profiling.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/profiling.py)
-   [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py)
-   [as2\_python\_api/as2\_python\_api/test/test\_mission\_stack.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_mission_stack.py)
-   [as2\_python\_api/as2\_python\_api/test/try\_multiple\_drone.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/try_multiple_drone.py)
-   [as2\_python\_api/as2\_python\_api/tools/utils.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/tools/utils.py)

The Python API provides a high-level interface for controlling drones and creating autonomous missions using Aerostack2. It abstracts the underlying ROS 2 communication, allowing developers to programmatically control drones without directly interacting with ROS 2 topics, services, and actions.

For information about defining missions, see [Mission Interpreter](https://deepwiki.com/aerostack2/aerostack2/5.2-mission-interpreter).

## Architecture Overview

The Python API consists of several key components that work together to provide drone control capabilities:

```
loadsusescontrolsDroneInterfaceBase+position+orientation+speed+info+arm()+disarm()+offboard()+manual()+load_module()DroneInterface+takeoff+go_to+follow_path+landDroneInterfaceGPS+gps+go_to+follow_pathDroneInterfaceTeleop+motion_ref_handlerModuleBase+alias+deps+get_plan_item()BehaviorHandler+feedback+status+start()+pause()+resume()+stop()MissionInterpreter+load_mission()+start_mission()+pause_mission()+resume_mission()+stop_mission()+next_item()
```

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py) [as2\_python\_api/as2\_python\_api/drone\_interface.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface.py) [as2\_python\_api/as2\_python\_api/drone\_interface\_gps.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_gps.py) [as2\_python\_api/as2\_python\_api/drone\_interface\_teleop.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_teleop.py) [as2\_python\_api/as2\_python\_api/modules/module\_base.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/module_base.py) [as2\_python\_api/as2\_python\_api/behavior\_actions/behavior\_handler.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/behavior_actions/behavior_handler.py) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py)

## Core Components

### DroneInterface

The `DroneInterface` class provides direct control over drones through a Python interface. It handles the underlying ROS 2 communication and exposes methods and properties for drone control.

```
DroneInterfacearm()disarm()offboard()position, orientationspeedinfotakeoffgo_tofollow_pathlandROS2 NodeService ClientsTopic Subscribers/PublishersBehavior ModulesArm ServiceDisarm ServiceOffboard ServicePose SubscriberTwist SubscriberPlatform Info SubscriberTakeoff ModuleGo To ModuleFollow Path ModuleLand Module
```

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py58-288](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py#L58-L288) [as2\_python\_api/as2\_python\_api/drone\_interface.py42-66](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface.py#L42-L66)

#### Basic Usage

To create and use a `DroneInterface`:

```
import rclpy
from as2_python_api.drone_interface import DroneInterface

# Initialize ROS 2
rclpy.init()

# Create the drone interface
drone = DroneInterface(drone_id='drone0', verbose=True, use_sim_time=False)

# Get drone information
print(f"Position: {drone.position}")
print(f"Orientation: {drone.orientation}")
print(f"Speed: {drone.speed}")
print(f"Info: {drone.info}")

# Perform basic operations
drone.arm()
drone.offboard()

# Use behavior modules
drone.takeoff(height=2.0, speed=1.0)
drone.go_to(x=5.0, y=5.0, z=2.0, speed=2.0)
drone.land(speed=1.0)

# Clean up
drone.shutdown()
rclpy.shutdown()
```

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py146-183](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py#L146-L183) [as2\_python\_api/as2\_python\_api/test/try\_multiple\_drone.py44-52](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/try_multiple_drone.py#L44-L52)

#### Available Properties

The `DroneInterface` provides access to drone state information through properties:

| Property | Type | Description |
| --- | --- | --- |
| `drone_id` | str | The namespace of the drone (used for ROS 2 communication) |
| `info` | dict | Status information including connection state, arming state, offboard mode |
| `position` | list\[float\] | Current position \[x, y, z\] in meters |
| `orientation` | list\[float\] | Current orientation \[roll, pitch, yaw\] in radians |
| `speed` | list\[float\] | Current velocity \[vx, vy, vz\] in meters per second |

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py137-183](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py#L137-L183)

#### Basic Methods

| Method | Description |
| --- | --- |
| `arm()` | Arms the drone's motors |
| `disarm()` | Disarms the drone's motors |
| `offboard()` | Puts the drone in offboard mode (for external control) |
| `manual()` | Disables offboard mode |
| `load_module(pkg)` | Loads a module by name or package |
| `shutdown()` | Properly cleans up resources |

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py211-287](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py#L211-L287)

#### Safety Methods

The `DroneInterface` provides methods for handling emergency situations:

| Method | Description |
| --- | --- |
| `send_emergency_land()` | Forces the drone to land |
| `send_emergency_hover()` | Forces the drone to hover in place |
| `send_emergency_land_to_aircraft()` | Calls platform emergency landing |
| `send_emergency_hover_to_aircraft()` | Calls platform emergency hover |
| `send_emergency_killswitch_to_aircraft()` | Immediately stops motors (USE WITH CAUTION) |

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py253-287](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py#L253-L287)

### Behavior Modules

Behavior modules provide high-level functionality like takeoff, landing, and navigation. The `DroneInterface` comes with several pre-loaded modules, and additional modules can be loaded dynamically.

#### Pre-loaded Modules in DroneInterface

| Module | Description |
| --- | --- |
| `takeoff` | Takes off to a specified height |
| `go_to` | Navigates to a specified position |
| `follow_path` | Follows a sequence of waypoints |
| `land` | Lands the drone |

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface.py58-65](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface.py#L58-L65)

#### Additional Modules in DroneInterfaceGPS

| Module | Description |
| --- | --- |
| `gps` | Provides GPS functionality |
| `go_to` | Navigates to a specified GPS position |
| `follow_path` | Follows a path of GPS waypoints |

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_gps.py63-67](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_gps.py#L63-L67)

#### Loading Custom Modules

Modules can be loaded dynamically using `load_module()`:

```
# Load a module by name
drone.load_module('point_gimbal')

# Use the loaded module
drone.point_gimbal(roll=0.0, pitch=-45.0, yaw=0.0)
```

The system searches for modules in:

1.  Standard modules in `as2_python_api.modules`
2.  Custom modules in paths specified by the `AS2_MODULES_PATH` environment variable

Sources: [as2\_python\_api/as2\_python\_api/drone\_interface\_base.py125-135](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_base.py#L125-L135) [as2\_python\_api/as2\_python\_api/tools/utils.py81-123](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/tools/utils.py#L81-L123)

### Behavior Handler

The `BehaviorHandler` class provides a common interface for interacting with ROS 2 action servers that implement drone behaviors. It's used internally by modules to communicate with behavior implementations.

```
"ROS2 Action Server""BehaviorHandler""Behavior Module""ROS2 Action Server""BehaviorHandler""Behavior Module"loop[During Execution]start(goal_msg)send_goal_async(goal_msg)goal_handleTruefeedback_callback(feedback)store feedbackwait_to_result()get_result_async()resultSuccess/Failure
```

Sources: [as2\_python\_api/as2\_python\_api/behavior\_actions/behavior\_handler.py47-257](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/behavior_actions/behavior_handler.py#L47-L257)

The `BehaviorHandler` provides these key methods:

| Method | Description |
| --- | --- |
| `start(goal_msg, wait_result=True)` | Starts the behavior with the given goal |
| `pause()` | Pauses the current behavior execution |
| `resume(wait_result=True)` | Resumes a paused behavior |
| `stop()` | Stops the current behavior |
| `wait_to_result()` | Waits for the behavior to complete and returns the result |

Sources: [as2\_python\_api/as2\_python\_api/behavior\_actions/behavior\_handler.py142-247](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/behavior_actions/behavior_handler.py#L142-L247)

### Mission Interpreter

The `MissionInterpreter` enables defining and executing sequences of behaviors as missions. This provides a higher-level control framework for autonomous drone operations.

```
Mission Execution FlowExecutionload_mission()CreatesControlsUsesstart_mission()Mission DefinitionMission InterpreterMission StackDrone InterfaceBehavior Handlernext_item()pause_mission()resume_mission()stop_mission()
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py52-289](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L52-L289) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py92-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py#L92-L109)

#### Mission Definition

A mission is defined as a JSON structure that specifies a sequence of behaviors to execute:

```
{
    "target": "drone_0",
    "plan": [
        {
            "behavior": "takeoff",
            "method": "__call__",
            "args": {
                "height": 2.0,
                "speed": 1.0
            }
        },
        {
            "behavior": "go_to",
            "args": {
                "x": 5.0,
                "y": 5.0,
                "z": 2.0,
                "speed": 2.0
            }
        },
        {
            "behavior": "land",
            "args": {
                "speed": 1.0
            }
        }
    ]
}
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py92-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py#L92-L109) [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py50-78](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L50-L78)

#### Using the Mission Interpreter

```
import rclpy
from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter

# Initialize ROS 2
rclpy.init()

# Create a mission interpreter
interpreter = MissionInterpreter(verbose=True)

# Load a mission from a JSON string
mission_json = """
{
    "target": "drone_0",
    "plan": [
        {
            "behavior": "takeoff",
            "args": {
                "height": 2.0,
                "speed": 1.0
            }
        },
        {
            "behavior": "land",
            "args": {
                "speed": 1.0
            }
        }
    ]
}
"""
mission = Mission.parse_raw(mission_json)
interpreter.load_mission(0, mission)

# Start the mission
interpreter.start_mission(0)

# Control mission execution
interpreter.pause_mission(0)  # Pause the current behavior
interpreter.resume_mission(0)  # Resume execution
interpreter.next_item(0)      # Skip to the next behavior
interpreter.stop_mission(0)   # Stop the mission

# Clean up
interpreter.shutdown()
rclpy.shutdown()
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py170-228](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L170-L228) [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py90-120](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L90-L120)

### ROS 2 Integration

The Python API allows both direct use within Python applications and integration with ROS 2 systems through an adapter node. This enables controlling drones from external ROS 2 nodes.

```
Python APIROS2 EnvironmentMissionUpdate messagesString status messagesBehavior ActionsUsesControlsMission Interpreter AdapterExternal NodesDrone NodesMission InterpreterDrone Interface
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py50-140](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py#L50-L140)

The adapter provides a ROS 2 interface with:

-   `mission_update` topic: Receives mission updates (load, start, pause, resume, stop)
-   `mission_status` topic: Publishes the current mission status

Mission updates use the `MissionUpdate` message type, which includes:

| Field | Type | Description |
| --- | --- | --- |
| `drone_id` | string | ID of the target drone |
| `mission_id` | int32 | ID of the mission |
| `action` | uint8 | Action to perform (EXECUTE, LOAD, START, etc.) |
| `mission` | string | JSON-formatted mission definition |

Sources: [as2\_msgs/msg/MissionUpdate.msg1-23](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/MissionUpdate.msg#L1-L23) [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py93-122](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py#L93-L122)

## Example Usage Patterns

Here are some common usage patterns for the Python API:

### Basic Single Drone Control

```
import rclpy
from as2_python_api.drone_interface import DroneInterface

rclpy.init()
drone = DroneInterface(drone_id='drone0', verbose=True)

# Basic flight operation
drone.arm()
drone.offboard()
drone.takeoff(height=2.0, speed=1.0)
drone.go_to(x=5.0, y=0.0, z=2.0, speed=2.0)
drone.land(speed=1.0)

drone.shutdown()
rclpy.shutdown()
```

### Multi-Drone Control

```
import rclpy
from as2_python_api.drone_interface import DroneInterface

rclpy.init()

# Create multiple drone interfaces
drone1 = DroneInterface(drone_id='drone_0', verbose=True)
drone2 = DroneInterface(drone_id='drone_1', verbose=True)

# Control drones individually
drone1.arm()
drone1.offboard()
drone1.takeoff(height=2.0, speed=1.0)

drone2.arm()
drone2.offboard()
drone2.takeoff(height=2.0, speed=1.0)

# Coordinate movements
drone1.go_to(x=5.0, y=0.0, z=2.0, speed=2.0)
drone2.go_to(x=0.0, y=5.0, z=2.0, speed=2.0)

# Land both drones
drone1.land(speed=1.0)
drone2.land(speed=1.0)

# Clean up
drone1.shutdown()
drone2.shutdown()
rclpy.shutdown()
```

Sources: [as2\_python\_api/as2\_python\_api/test/try\_multiple\_drone.py41-54](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/try_multiple_drone.py#L41-L54)

### Mission-Based Execution

```
import rclpy
from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter

rclpy.init()
interpreter = MissionInterpreter(verbose=True)

# Define a mission
mission_json = """
{
    "target": "drone_0",
    "plan": [
        {
            "behavior": "takeoff",
            "args": {
                "height": 2.0,
                "speed": 1.0
            }
        },
        {
            "behavior": "go_to",
            "args": {
                "x": 5.0,
                "y": 0.0,
                "z": 2.0,
                "speed": 2.0
            }
        },
        {
            "behavior": "go_to",
            "args": {
                "x": 0.0,
                "y": 0.0,
                "z": 2.0,
                "speed": 2.0
            }
        },
        {
            "behavior": "land",
            "args": {
                "speed": 1.0
            }
        }
    ]
}
"""

# Load and execute the mission
mission = Mission.parse_raw(mission_json)
interpreter.load_mission(0, mission)
interpreter.start_mission(0)

# Clean up
interpreter.shutdown()
rclpy.shutdown()
```

Sources: [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py133-184](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L133-L184)

## Advanced Topics

### Creating Custom Modules

Custom modules can extend the capabilities of the `DroneInterface`. A module is a class that inherits from `ModuleBase` and implements drone functionality:

```
from as2_python_api.modules.module_base import ModuleBase

class CustomModule(ModuleBase):
    """Custom module for specific functionality."""
    
    __alias__ = 'custom'  # Name used to access module: drone.custom
    __deps__ = []         # List of module dependencies
    
    def __init__(self, drone):
        super().__init__(drone, self.__alias__)
        # Initialize module
        
    def __call__(self, param1, param2=0.0):
        """Main functionality of the module."""
        # Implement behavior
        
    def destroy(self):
        """Clean up resources."""
        # Clean up
```

To use a custom module, place it in a location specified by the `AS2_MODULES_PATH` environment variable.

Sources: [as2\_python\_api/as2\_python\_api/modules/module\_base.py44-89](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/module_base.py#L44-L89) [as2\_python\_api/as2\_python\_api/modules/dummy\_module.py49-78](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/dummy_module.py#L49-L78)

### Mission Stack Management

The mission interpreter uses a mission stack to manage the execution of mission items. The stack provides methods for manipulating the sequence of behaviors:

```
# Add behaviors to the end of a mission
interpreter.append_mission(mission_id, additional_mission)

# Insert behaviors at the front of a mission
interpreter.insert_mission(mission_id, priority_mission)
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_stack.py46-116](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_stack.py#L46-L116) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py235-247](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L235-L247)

## Summary

The Aerostack2 Python API provides a comprehensive interface for drone control and autonomous mission execution. It abstracts the complexities of ROS 2 communication, enabling developers to focus on high-level application logic. The API supports direct drone control through the `DroneInterface` class and automated mission execution through the `MissionInterpreter`.

Key features include:

-   Direct control of drone operations (arm, takeoff, navigate, land)
-   Access to drone state information (position, orientation, speed)
-   Modular behavior system with pre-loaded and custom modules
-   Mission definition and execution capabilities
-   Integration with ROS 2 for external control

This API serves as the foundation for building complex autonomous drone applications using Aerostack2.