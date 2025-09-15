Relevant source files

-   [as2\_msgs/msg/MissionUpdate.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/MissionUpdate.msg)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_stack.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_stack.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py)
-   [as2\_python\_api/as2\_python\_api/modules/dummy\_module.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/dummy_module.py)
-   [as2\_python\_api/as2\_python\_api/modules/module\_base.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/module_base.py)
-   [as2\_python\_api/as2\_python\_api/test/modules/modules\_as\_properties.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/modules/modules_as_properties.py)
-   [as2\_python\_api/as2\_python\_api/test/profiling.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/profiling.py)
-   [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py)
-   [as2\_python\_api/as2\_python\_api/test/test\_mission\_stack.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_mission_stack.py)
-   [as2\_python\_api/as2\_python\_api/tools/utils.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/tools/utils.py)

The Mission Interpreter is a core component of the Aerostack2 framework that manages the execution of autonomous drone missions. It provides a way to define, load, and execute sequences of behaviors for aerial robots. This document covers the architecture, functionality, and usage of the Mission Interpreter component.

For information about the Python API that works with the Mission Interpreter, see [Python API](https://deepwiki.com/aerostack2/aerostack2/5.1-python-api).

## 1\. Overview

The Mission Interpreter translates high-level mission definitions into executable behavior sequences. It manages the lifecycle of mission execution including starting, pausing, resuming, and stopping missions. The system is designed to work with the behavior system and drone interface components of Aerostack2.

```
Drone InterfaceMission Interpreter SystemUser LayerMission Definition (JSON)Mission Control CommandsMissionInterpreterMission ModelMission StackROS2 AdapterDroneInterfaceBehavior Modules
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py) [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py)

## 2\. Key Components

The Mission Interpreter system consists of several key components that work together to manage mission execution:

### 2.1 MissionInterpreter

The `MissionInterpreter` class is the central component that manages the execution of missions. It loads mission definitions, tracks execution status, and orchestrates behavior calls through the drone interface.

```
managescontainsusescontrolsMissionInterpreter+missions: dict+current_mid: int+mission_stack: MissionStack+drone: DroneInterfaceBase+current_behavior: BehaviorHandler+load_mission(mid, mission)+start_mission(mid)+stop_mission(mid)+pause_mission(mid)+resume_mission(mid)+next_item(mid)+append_mission(mid, mission)+insert_mission(mid, mission)Mission+target: str+plan: List[MissionItem]+stack() : : MissionStackMissionItem+behavior: str+method: str+args: dictMissionStack+pending: List[MissionItem]+done: List[MissionItem]+current: MissionItem+next_item()+previous_item()+add(item)+insert(item)DroneInterfaceBase+modules: dict+load_module(module_name)
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py52-303](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L52-L303)

### 2.2 Mission and MissionItem

The `Mission` class represents a mission definition that contains a target drone identifier and a plan consisting of a sequence of `MissionItem` objects. Each `MissionItem` represents a single behavior call with its associated method and arguments.

```
MissionItem 3MissionItem 2MissionItem 1MissionTarget Drone IDPlan (List of Items)Behavior: takeoffMethod: callArgs: height, speedBehavior: go_toMethod: callArgs: x, y, z, speedBehavior: landMethod: callArgs: speedMissionItem1MissionItem2MissionItem3
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py60-69](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py#L60-L69) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py92-109](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py#L92-L109)

### 2.3 MissionStack

The `MissionStack` class manages the execution stack of mission items. It maintains three collections:

-   Pending items: Items that are yet to be executed
-   Current item: The item currently being executed
-   Done items: Items that have been executed

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_stack.py46-116](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_stack.py#L46-L116)

### 2.4 ROS2 Adapter

The `Adapter` class provides a ROS 2 interface for the Mission Interpreter. It handles mission updates through ROS 2 topics and publishes mission status.

```
ROS2 AdapterROS 2 Topics/mission_update/mission_statusmission_update_callback()status_timer_callback()MissionInterpreter
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py50-141](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py#L50-L141)

## 3\. Mission Definition Format

Missions are defined in a JSON format that specifies the target drone and a plan of behaviors to execute. Each behavior in the plan includes the behavior name, an optional method name (defaults to `__call__`), and a dictionary of arguments.

### Example Mission Definition

```
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
                "y": 5.0,
                "z": 2.0,
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
```

Sources: [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py50-78](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L50-L78) [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py133-168](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L133-L168)

## 4\. Mission Execution Flow

The mission execution follows a well-defined flow from loading to execution completion:

```
BehaviorDroneInterfaceMissionStackMissionInterpreterUserBehaviorDroneInterfaceMissionStackMissionInterpreterUserloop[Until mission complete or stopped]load_mission(mid, mission)Create MissionStackLoad required modulesstart_mission(mid)Create execution threadnext_item()Return MissionItemGet behavior moduleExecute method with argsReturn result/statusMission complete
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py169-179](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L169-L179) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py180-188](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L180-L188) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py249-274](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L249-L274)

## 5\. Mission Control Operations

The Mission Interpreter provides several operations to control mission execution:

| Operation | Description | Method |
| --- | --- | --- |
| Load | Load a mission definition | `load_mission(mid, mission)` |
| Start | Start executing a mission | `start_mission(mid)` |
| Stop | Stop a running mission | `stop_mission(mid)` |
| Pause | Pause a running mission | `pause_mission(mid)` |
| Resume | Resume a paused mission | `resume_mission(mid)` |
| Next Item | Skip to the next item in the mission | `next_item(mid)` |
| Append | Add items to the end of the mission | `append_mission(mid, mission)` |
| Insert | Insert items at the beginning of the mission | `insert_mission(mid, mission)` |
| Reset | Reset the interpreter with a new mission | `reset(mid, mission)` |

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py169-288](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L169-L288)

## 6\. Dynamic Mission Modification

The Mission Interpreter allows for dynamic modification of missions during execution:

### 6.1 Appending Items

New items can be appended to the end of the mission stack using the `append_mission` method. This is useful for extending missions on-the-fly.

```
# Example of appending items
interpreter.append_mission(mid, new_mission)
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py235-241](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L235-L241)

### 6.2 Inserting Items

Items can also be inserted at the beginning of the mission stack using the `insert_mission` method. This allows for priority behaviors to be executed next.

```
# Example of inserting items
interpreter.insert_mission(mid, priority_mission)
```

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py242-248](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L242-L248)

## 7\. ROS 2 Integration

The Mission Interpreter integrates with ROS 2 through the `Adapter` class, which provides a ROS 2 node interface.

### 7.1 Published Topics

| Topic | Message Type | Description |
| --- | --- | --- |
| `/mission_status` | `std_msgs/String` | JSON-encoded mission status information |

### 7.2 Subscribed Topics

| Topic | Message Type | Description |
| --- | --- | --- |
| `/mission_update` | `as2_msgs/MissionUpdate` | Commands for mission loading and control |

### 7.3 MissionUpdate Message Actions

The `MissionUpdate` message includes an action field that specifies the operation to perform:

| Action | Value | Description |
| --- | --- | --- |
| EXECUTE | 0 | Execute a mission in the interpreter |
| LOAD | 1 | Load a mission to the interpreter |
| START | 2 | Start the execution of a mission |
| PAUSE | 3 | Pause the execution of a mission |
| RESUME | 4 | Resume the execution of a mission |
| STOP | 5 | Stop the execution of a mission |
| NEXT\_ITEM | 6 | Execute the next item in the mission |
| REPEAT | 7 | Repeat the execution of a mission |
| INSERT | 8 | Insert an item in the mission |
| MODIFY | 9 | Modify an item in the mission |
| REMOVE | 10 | Remove an item in the mission |
| RESET | 11 | Reset the interpreter |

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py70-91](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py#L70-L91) [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py93-122](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py#L93-L122) [as2\_msgs/msg/MissionUpdate.msg7-17](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/MissionUpdate.msg#L7-L17)

## 8\. Status Tracking

The Mission Interpreter provides status tracking through the `InterpreterStatus` class:

```
InterpreterStatus+state: InterpreterState+pending_items: int+done_items: int+current_item: MissionItem+feedback_current: Any+total_items() : : int«enumeration»InterpreterStateIDLERUNNINGPAUSED
```

The status includes:

-   Current state (IDLE, RUNNING, PAUSED)
-   Number of pending items
-   Number of completed items
-   Current mission item being executed
-   Feedback from the current behavior

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py112-142](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py#L112-L142) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py52-58](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py#L52-L58)

## 9\. Module Loading

The Mission Interpreter automatically loads required behavior modules based on the mission definition:

```
Module LoadingMission Loading Processload_mission()load_modules()get_class_from_module()Create module instanceRegister in drone.modules
```

The interpreter analyzes the mission plan, identifies all required behavior modules, and ensures they are loaded in the drone interface before execution begins.

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py120-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L120-L124) [as2\_python\_api/as2\_python\_api/tools/utils.py81-105](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/tools/utils.py#L81-L105)

## 10\. Usage Example

Here's an example of how to use the Mission Interpreter:

```
# Create a mission definition
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
                "y": 5.0,
                "z": 2.0,
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

# Parse the mission
mission = Mission.parse_raw(mission_json)

# Create an interpreter
interpreter = MissionInterpreter(verbose=True)

# Load the mission
interpreter.load_mission(0, mission)

# Start the mission
interpreter.start_mission(0)

# Wait for completion or control the mission
# interpreter.pause_mission(0)
# interpreter.resume_mission(0)
# interpreter.stop_mission(0)

# Clean up
interpreter.shutdown()
```

Sources: [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py50-78](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L50-L78) [as2\_python\_api/as2\_python\_api/test/test\_interpreter.py81-83](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/test/test_interpreter.py#L81-L83) [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py305-345](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L305-L345)

## 11\. Integration with Behavior System

The Mission Interpreter integrates with the Aerostack2 behavior system by using the behavior modules registered in the drone interface:

```
Behavior ModulesDrone InterfaceMission Interpreterbehavior: 'takeoff'behavior: 'go_to'behavior: 'land'MissionInterpreterMissionMissionItemDroneInterfaceBasemodules: dictTakeoffModuleGoToModuleLandModuleOther Modules...
```

When executing a mission item, the interpreter:

1.  Looks up the behavior module in the drone interface's module registry
2.  Retrieves the specified method from the behavior module
3.  Calls the method with the provided arguments
4.  Monitors the behavior's execution status

Sources: [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py249-269](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L249-L269) [as2\_python\_api/as2\_python\_api/modules/module\_base.py44-89](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/module_base.py#L44-L89)