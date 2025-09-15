Relevant source files

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/config\_values.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/config_values.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/localization\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/localization_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/settings\_window.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/settings_window.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/config/teleop\_values\_config.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/config/teleop_values_config.yaml)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/launch/as2\_keyboard\_teleoperation\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/launch/as2_keyboard_teleoperation_launch.py)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/setup.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/setup.py)

## Purpose and Scope

The Keyboard Teleoperation module in Aerostack2 provides a graphical user interface for manual control of drones using keyboard inputs. It allows users to operate drones in different control modes, visualize their positions and orientations, and manage drone behaviors. This module serves as an interactive testing and debugging tool for drone operation.

For programmatic control of drones using Python, see [Python API](https://deepwiki.com/aerostack2/aerostack2/5.1-python-api). For mission definition and execution, see [Mission Interpreter](https://deepwiki.com/aerostack2/aerostack2/5.2-mission-interpreter).

## Architecture Overview

The Keyboard Teleoperation system consists of several components that work together to provide manual control of drones.

```
Drone LayerControl LayerUser Interface LayerMainWindowSettingsWindowLocalizationWindowKeyboardTeleoperationDroneManagerControlValues / KeyMappingsDroneInterfaceExtendedSwarmBehaviorManagerUser
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py147-161](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L147-L161)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py45-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L45-L74)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py46-62](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py#L46-L62)

## Components

### 1\. Main Classes

```
KeyboardTeleoperation-uav_list: List[DroneInterface]-drone_manager: DroneManager-main_window: MainWindow-settings_window: SettingsWindow-localization_window: LocalizationWindow+execute_main_window()+get_behavior_status()DroneManager-uav_list: List[DroneInterface]-drone_id_list: List-pose_frame_id: String-twist_frame_id: String-use_body_frame: Boolean+manage_common_behaviors()+manage_speed_behaviors()+manage_pose_behaviors()MainWindow-settings_window: SettingsWindow-localization_window: LocalizationWindow-control_mode: String+make_main_window()+event_handler()+update_window_to_speed()+update_window_to_pose()+update_window_to_body_pose()SettingsWindow-value_list: List[float]+make_settings_window()+event_handler()LocalizationWindow-uav_list: List[DroneInterface]+make_localization_window()+execute_localization_window()
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py154-337](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L154-L337)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py45-512](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L45-L512)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py46-395](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py#L46-L395)

### 2\. Control Flow

```
DroneInterfaceDroneManagerKeyboardTeleoperationMainWindowUserDroneInterfaceDroneManagerKeyboardTeleoperationMainWindowUseralt[Speed Control Mode][Pose Control Mode][Body Pose Control Mode]Keyboard Inputevent_handler()manage_common_behaviors(key)manage_speed_behaviors(key, values)manage_pose_behaviors(key, values)manage_pose_behaviors(key, values)execute_function(action)
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py260-313](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L260-L313)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py297-411](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L297-L411)

## Control Modes

The Keyboard Teleoperation system supports three different control modes:

### 1\. Speed Control Mode

In Speed Control mode, keyboard inputs generate velocity commands that control the drone's speed in different directions.

-   Forward/backward: Controls speed along the X-axis
-   Left/right: Controls speed along the Y-axis
-   Up/down: Controls speed along the Z-axis
-   Rotate left/right: Controls yaw rate

### 2\. Pose Control Mode

In Pose Control mode, keyboard inputs generate position increments in the earth frame.

-   Forward/backward: Increments position along the X-axis
-   Left/right: Increments position along the Y-axis
-   Up/down: Increments position along the Z-axis
-   Rotate left/right: Increments yaw angle

### 3\. Body Pose Control Mode

Similar to Pose Control, but movements are relative to the drone's current orientation (body frame) rather than the earth frame.

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py414-479](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L414-L479)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py284-293](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L284-L293)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py73-331](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py#L73-L331)

## User Interface

### Main Window

The main window provides the primary interface for drone control. It displays:

1.  Control mode selection buttons (Speed, Pose, Body Pose)
2.  Basic motion commands display
3.  Control value displays
4.  Drone selection controls
5.  Behavior control panel (can be toggled)

!\[Main Window Layout\]

```
Main WindowMain Control AreaBasic Motions(Takeoff, Land, Hover, Emergency)Control Panel(Mode-specific values)Movement Controls(Direction arrows)Drone Selection PanelBehavior Control Panel(Active/Paused Behaviors)Status Bar (Last key pressed)
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py75-295](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L75-L295)

### Settings Window

The Settings window allows users to adjust control values for different modes:

-   Speed values (m/s)
-   Vertical speed values (m/s)
-   Turn speed values (rad/s)
-   Position increment values (m)
-   Altitude increment values (m)
-   Turn angle values (rad)

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/settings\_window.py53-103](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/settings_window.py#L53-L103)

### Localization Window

The Localization window displays real-time position and orientation information for each drone:

-   Position (x, y, z) in meters
-   Orientation (roll, pitch, yaw) in radians

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/localization\_window.py57-113](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/localization_window.py#L57-L113)

### Behavior Control Panel

The Behavior Control panel allows users to manage active and paused behaviors:

-   List of active behaviors
-   List of paused behaviors
-   Controls to pause/resume selected behaviors or all behaviors

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py234-272](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L234-L272)

## Key Mappings

The system uses the following key mappings for drone control:

| Key | Function |
| --- | --- |
| t | Take off |
| l | Land |
| Space | Hover |
| Delete | Emergency Stop |
| Up Arrow | Move forward |
| Down Arrow | Move backward |
| Left Arrow | Move left |
| Right Arrow | Move right |
| w | Increase altitude |
| s | Decrease altitude |
| a | Rotate left |
| d | Rotate right |
| r | Reset orientation |

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/config\_values.py45-57](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/config_values.py#L45-L57)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py77-173](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L77-L173)

## Configuration

### Default Values

The system uses default control values that can be changed through the Settings window or configured at launch:

| Parameter | Default Value | Description |
| --- | --- | --- |
| speed\_value | 0.5 m/s | Linear speed in speed control mode |
| altitude\_speed\_value | 0.5 m/s | Vertical speed in speed control mode |
| turn\_speed\_value | 0.5 rad/s | Angular speed in speed control mode |
| position\_value | 1.0 m | Position increment in position control mode |
| altitude\_value | 1.0 m | Altitude increment in position control mode |
| turn\_angle\_value | 0.78 rad | Angle increment in position control mode |

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/config\_values.py60-68](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/config_values.py#L60-L68)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/config/teleop\_values\_config.yaml1-16](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/config/teleop_values_config.yaml#L1-L16)

### Configuration File

The configuration file (`teleop_values_config.yaml`) allows customization of default values and behavior:

```
namespace: "drone0"  # Comma-separated list for multiple drones
use_sim_time: True   # Use simulation time
verbose: False       # Verbose mode
speed_value: 0.5     # Speed value (m/s)
altitude_speed_value: 0.5  # Vertical speed value (m/s)
turn_speed_value: 0.5      # Turn speed value (rad/s)
position_value: 1.00       # Position increment (m)
altitude_value: 1.00       # Altitude increment (m)
turn_angle_value: 0.78     # Turn angle increment (rad)
drone_frequency: 100.0     # Drone interface update frequency
speed_frame_id: "earth"    # Frame ID for speed mode
pose_frame_id: "earth"     # Frame ID for pose mode
initial_mode: "pose"       # Initial control mode
modules: ""                # Additional modules to load
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/config/teleop\_values\_config.yaml1-16](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/config/teleop_values_config.yaml#L1-L16)

## Usage

### Launching the Keyboard Teleoperation

The keyboard teleoperation can be launched using the ROS 2 launch system:

```
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py namespace:=drone0
```

For multiple drones:

```
ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py namespace:="drone0,drone1"
```

### Launch Parameters

| Parameter | Description |
| --- | --- |
| namespace | Comma-separated list of drone namespaces |
| use\_sim\_time | Use simulation time (true/false) |
| verbose | Verbose mode (true/false) |
| config\_file | Path to configuration file |

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/launch/as2\_keyboard\_teleoperation\_launch.py132-153](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/launch/as2_keyboard_teleoperation_launch.py#L132-L153)

## Multi-Drone Control

### Drone Selection

The Keyboard Teleoperation interface includes a drone selection panel that allows users to:

1.  Select individual drones for control
2.  Select/deselect all drones at once
3.  Send the same command to multiple selected drones

Commands are only sent to drones that are currently selected in the interface.

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py210-232](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L210-L232)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/drone\_manager.py332-341](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/drone_manager.py#L332-L341)

### Behavior Management

The Behavior Control panel allows managing behaviors across multiple drones:

1.  View active behaviors for all drones
2.  Pause/resume specific behaviors
3.  Pause/resume all behaviors

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py295-311](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L295-L311)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/main\_window.py234-272](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/main_window.py#L234-L272)

## Code Structure

```
Package Structurekeyboard_teleoperation.py(Main Class)main_window.py(GUI Main Window)drone_manager.py(Drone Control)localization_window.py(Position Display)settings_window.py(Settings Panel)config_values.py(Configuration)as2_keyboard_teleoperation_launch.py(Launch File)teleop_values_config.yaml(Configuration)
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/setup.py1-41](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/setup.py#L1-L41)

## Integration with Aerostack2

The Keyboard Teleoperation module integrates with Aerostack2 through the `DroneInterfaceTeleop` class from the `as2_python_api` package. This interface provides access to all drone functionalities:

-   Motion control (speed, position)
-   Takeoff and landing
-   Behavior management
-   State monitoring (position, orientation)

```
Aerostack2 IntegrationKeyboardTeleoperationDroneInterfaceTeleopSwarmBehaviorManagerMotion Control SystemBehavior SystemPlatform Layer
```

Sources:

-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py54-70](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L54-L70)
-   [as2\_user\_interfaces/as2\_keyboard\_teleoperation/as2\_keyboard\_teleoperation/keyboard\_teleoperation.py133-146](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_user_interfaces/as2_keyboard_teleoperation/as2_keyboard_teleoperation/keyboard_teleoperation.py#L133-L146)