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
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/config\_values.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/config_values.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/localization\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/localization_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/settings\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/settings_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/config/teleop\_values\_config.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/config/teleop_values_config.yaml)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/launch/as2\_keyboard\_teleoperation\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/launch/as2_keyboard_teleoperation_launch.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/setup.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/setup.py)

This page documents the user interfaces available in Aerostack2 for controlling and interacting with drones. Aerostack2 provides several interface options to accommodate different use cases, from simple keyboard-based control to advanced programmatic mission execution.

For information about the core system and behaviors, see [Core System](https://deepwiki.com/aerostack2/aerostack2/2-core-system) and [Behavior System](https://deepwiki.com/aerostack2/aerostack2/3-behavior-system).

## Overview of User Interfaces

Aerostack2 offers several user interface options to control and interact with drones:

```
System LayerDrone Control LayerUser InterfacesPython APIMission InterpreterKeyboard TeleoperationCommand Line InterfaceDroneInterfaceBehavior ActionsMotion ReferencesBehavior ServersAerial Platforms
```

Sources:

-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py)

## Python API

The Python API provides a programmatic interface for controlling drones. It serves as the foundation for other user interfaces and can be used directly for creating custom applications.

```
DroneInterfaceBase+String drone_id+info+position+orientation+modules+arm()+disarm()+offboard()+takeoff()+land()+load_module(module_name)DroneInterface+takeoff+land+go_to+follow_pathDroneInterfaceTeleop+position+orientation+send_speed_command()+send_pose_command()
```

The Python API is centered around the `DroneInterface` class, which provides methods for:

-   Basic drone operations (arm, disarm, takeoff, land)
-   Motion control (position, speed commands)
-   Loading behavior modules (takeoff, land, go\_to, etc.)
-   Accessing drone state (position, orientation)

```
# Example of using the Python API
from as2_python_api.drone_interface import DroneInterface

# Create a drone interface
drone = DroneInterface(drone_id='drone1', verbose=True)

# Basic operations
drone.arm()
drone.offboard()
drone.takeoff(height=2.0, speed=1.0)
drone.go_to(x=5.0, y=5.0, z=2.0, speed=2.0)
drone.land(speed=1.0)
```

Sources:

-   [as2\_python\_api/as2\_python\_api/drone\_interface\_teleop.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/drone_interface_teleop.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py104-118](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py#L104-L118)

## Mission Interpreter

The Mission Interpreter provides a way to define and execute complex drone missions using a JSON format. It parses mission definitions, loads required modules, and executes behaviors sequentially.

```
ExecutionInterpreter SystemMission Componentsloadscreatesexecutescontrolsprovidesexecuted byMission(target, plan)MissionItem(behavior, method, args)MissionStack(pending, done, current)MissionInterpreterROS2 AdapterDroneInterfaceBehaviorHandler
```

### Mission Format

Missions are defined in JSON format:

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

### Mission Execution Flow

```
"Behavior Handler""DroneInterface""MissionInterpreter""ROS2 Adapter""Client Application""Behavior Handler""DroneInterface""MissionInterpreter""ROS2 Adapter""Client Application"loop[For each mission item]MissionUpdate(LOAD)load_mission()Parse mission JSONCreate drone interfaceLoad required modulesMissionUpdate(START)start_mission()arm() & offboard()Get next mission itemGet behavior handlerExecute method with argsBehavior resultUpdate statusMissionUpdate(PAUSE/RESUME/STOP)pause/resume/stop missionSend corresponding command
```

### Using the Mission Interpreter

The Mission Interpreter can be used both programmatically and through ROS 2 messages:

**Programmatic usage:**

```
from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter

# Create interpreter
interpreter = MissionInterpreter(verbose=True)

# Load mission
mission_json = """{"target": "drone_0", "plan": [...]}"""
mission = Mission.parse_raw(mission_json)
interpreter.load_mission(mission_id=0, mission=mission)

# Execute mission
interpreter.start_mission(0)
```

**ROS 2 usage:**

```
# Using ROS 2 CLI to send a mission
ros2 topic pub /mission_update as2_msgs/msg/MissionUpdate \
  "{drone_id: 'drone_0', mission_id: 0, action: 1, mission: '{\"target\": \"drone_0\", \"plan\": [...]}'}"
```

The Mission Interpreter supports the following actions:

-   LOAD: Load a mission
-   EXECUTE: Load and start a mission
-   START: Start a loaded mission
-   PAUSE: Pause a running mission
-   RESUME: Resume a paused mission
-   STOP: Stop a running mission
-   NEXT\_ITEM: Skip to the next mission item

Sources:

-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_stack.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_stack.py)
-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/ros2\_adapter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/ros2_adapter.py)
-   [as2\_msgs/msg/MissionUpdate.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/MissionUpdate.msg)

## Keyboard Teleoperation

The Keyboard Teleoperation interface provides a graphical interface for controlling drones using keyboard inputs. It's built using PySimpleGUI and offers multiple control modes.

```
Drone ControlKeyboard Teleoperation Componentscreatescreatescreatescreatessend eventsupdate settingsdisplay positionsend commandscontrol dronesend motion referencesKeyboardTeleoperation(Main class)MainWindow(UI interface)SettingsWindow(Configuration)LocalizationWindow(Position display)DroneManager(Command handler)DroneInterfaceMotion References
```

### Main Features

The Keyboard Teleoperation interface provides:

1.  **Multiple Control Modes**:
    
    -   Speed Control: Send velocity commands
    -   Pose Control: Send position commands in earth frame
    -   Body Pose Control: Send position commands in body frame
2.  **Basic Drone Operations**:
    
    -   Takeoff (key: t)
    -   Land (key: l)
    -   Hover (key: space)
    -   Emergency Stop (key: Delete)
3.  **Movement Controls**:
    
    -   Forward/Backward (arrow keys: Up/Down)
    -   Left/Right (arrow keys: Left/Right)
    -   Up/Down (keys: w/s)
    -   Rotate (keys: a/d)
4.  **Configuration Options**:
    
    -   Speed values
    -   Position increments
    -   Altitude increments
    -   Turning values
5.  **Multi-Drone Support**:
    
    -   Select multiple drones to control simultaneously
6.  **Behavior Control**:
    
    -   View active behaviors
    -   Pause/resume behaviors

### Interface Layout

!\[Keyboard Teleoperation Interface\]

The interface consists of:

-   Main window with control mode selection and drone status
-   Settings window for adjusting control parameters
-   Localization window for viewing drone position and orientation

### Starting the Keyboard Teleoperation

```
# Launch using default configuration
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py namespace:=drone0

# Launch with multiple drones
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py namespace:=drone0,drone1

# Launch with custom configuration
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
  config_file:=/path/to/custom_config.yaml
```

### Configuration

The keyboard teleoperation can be configured through a YAML file:

| Parameter | Description | Default |
| --- | --- | --- |
| `namespace` | Drone namespace(s) | drone0 |
| `use_sim_time` | Use simulation time | True |
| `speed_value` | Speed value (m/s) | 0.5 |
| `altitude_speed_value` | Vertical speed (m/s) | 0.5 |
| `turn_speed_value` | Turn speed (rad/s) | 0.5 |
| `position_value` | Position increment (m) | 1.0 |
| `altitude_value` | Altitude increment (m) | 1.0 |
| `turn_angle_value` | Turn angle (rad) | 0.78 |
| `speed_frame_id` | Frame ID for speed mode | earth |
| `pose_frame_id` | Frame ID for pose mode | earth |
| `initial_mode` | Initial control mode | pose |
| `modules` | Additional modules to load | "" |

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/config\_values.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/config_values.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/config/teleop\_values\_config.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/config/teleop_values_config.yaml)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/launch/as2\_keyboard\_teleoperation\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/launch/as2_keyboard_teleoperation_launch.py)

## Command Line Interface

The Command Line Interface (CLI) provides a text-based interface for controlling drones and executing simple commands. It's particularly useful for scripting and integrating with other tools.

While the details of the CLI implementation aren't available in the provided files, it follows the same principles as the other interfaces by leveraging the Python API to control drones.

## Integration Between User Interfaces

The user interfaces in Aerostack2 are designed to work together and complement each other:

```
Motion ControlBehavior SystemCore APIUser-facing InterfacesKeyboard TeleoperationMission InterpreterCommand Line InterfacePython APIBehavior ActionsBehavior ServersMotion ReferencesMotion Controllers
```

All user interfaces share the same underlying architecture:

1.  They use the Python API to interact with drones
2.  The Python API provides access to behavior actions and motion references
3.  These interfaces translate user intentions into concrete drone actions

This unified approach ensures consistent behavior across different interfaces and allows users to choose the most appropriate interface for their specific needs.

Sources:

-   [as2\_python\_api/as2\_python\_api/mission\_interpreter/mission\_interpreter.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/mission_interpreter/mission_interpreter.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py)

## Summary

Aerostack2 provides a comprehensive set of user interfaces that cater to different use cases:

1.  **Python API**: For programmatic control and building custom applications
2.  **Mission Interpreter**: For defining and executing complex missions
3.  **Keyboard Teleoperation**: For manual control through a graphical interface
4.  **Command Line Interface**: For text-based control and scripting

These interfaces are built on a common architecture, ensuring consistent behavior and making it easy to switch between different control methods as needed.