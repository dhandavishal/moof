Relevant source files

-   [as2\_behaviors/as2\_behaviors\_motion/follow\_path\_behavior/plugins/follow\_path\_plugin\_trajectory.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/follow_path_behavior/plugins/follow_path_plugin_trajectory.cpp)
-   [as2\_behaviors/as2\_behaviors\_motion/go\_to\_behavior/plugins/go\_to\_plugin\_trajectory.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_trajectory.cpp)
-   [as2\_behaviors/as2\_behaviors\_motion/land\_behavior/plugins/land\_plugin\_trajectory.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/land_behavior/plugins/land_plugin_trajectory.cpp)
-   [as2\_behaviors/as2\_behaviors\_motion/takeoff\_behavior/plugins/takeoff\_plugin\_trajectory.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_trajectory.cpp)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/config/config\_default.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/config/config_default.yaml)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/include/generate\_polynomial\_trajectory\_behavior/generate\_polynomial\_trajectory\_behavior.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/include/generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_behavior.hpp)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/launch/composable\_generate\_polynomial\_trajectory\_behavior.launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/launch/composable_generate_polynomial_trajectory_behavior.launch.py)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/launch/generate\_polynomial\_trajectory\_behavior\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/launch/generate_polynomial_trajectory_behavior_launch.py)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior\_node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior_node.cpp)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/tests/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/tests/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/tests/behavior\_test.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/tests/behavior_test.py)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/tests/generate\_polynomial\_trajectory\_behavior\_gtest.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/tests/generate_polynomial_trajectory_behavior_gtest.cpp)
-   [as2\_msgs/action/GeneratePolynomialTrajectory.action](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/action/GeneratePolynomialTrajectory.action)
-   [as2\_msgs/msg/YawMode.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/YawMode.msg)
-   [as2\_python\_api/as2\_python\_api/behavior\_actions/trajectory\_generation\_behavior.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/behavior_actions/trajectory_generation_behavior.py)
-   [as2\_python\_api/as2\_python\_api/modules/trajectory\_generation\_module.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/trajectory_generation_module.py)

This page documents the trajectory generation system in Aerostack2, which creates smooth polynomial trajectories for aerial robots. The trajectory generator takes waypoints as input and produces continuous-time trajectories with velocity and acceleration profiles that ensure smooth drone movement.

For information about specific motion behaviors that use trajectory generation (like takeoff, land, go\_to), see [Motion Behaviors](https://deepwiki.com/aerostack2/aerostack2/3.2-perception-behaviors).

## System Overview

The trajectory generation component in Aerostack2 creates mathematically continuous paths for drones to follow, ensuring smooth transitions between waypoints while respecting speed limits and other constraints. It plays a critical role in translating high-level movement goals into executable flight trajectories.

```
Motion SystemTrajectory GenerationClient BehaviorsAction ClientAction ClientAction ClientAction ClientUsesSends TrajectorySetpointsMotionReferencesTakeoff BehaviorLand BehaviorGo To BehaviorFollow Path BehaviorDynamicPolynomialTrajectoryGeneratordynamic_trajectory_generatorLibraryTrajectoryMotionHandlerMotionController
```

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp41-58](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L41-L58)
-   [as2\_behaviors/as2\_behaviors\_motion/takeoff\_behavior/plugins/takeoff\_plugin\_trajectory.cpp52-67](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_trajectory.cpp#L52-L67)
-   [as2\_behaviors/as2\_behaviors\_motion/land\_behavior/plugins/land\_plugin\_trajectory.cpp54-85](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/land_behavior/plugins/land_plugin_trajectory.cpp#L54-L85)
-   [as2\_behaviors/as2\_behaviors\_motion/go\_to\_behavior/plugins/go\_to\_plugin\_trajectory.cpp54-78](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_trajectory.cpp#L54-L78)
-   [as2\_behaviors/as2\_behaviors\_motion/follow\_path\_behavior/plugins/follow\_path\_plugin\_trajectory.cpp52-79](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/follow_path_behavior/plugins/follow_path_plugin_trajectory.cpp#L52-L79)

## Architecture

The trajectory generation system is implemented as a behavior in Aerostack2, following the behavior server pattern. It exposes an action interface that other behaviors can interact with to generate and execute trajectories.

```
BehaviorServer+on_activate()+on_modify()+on_deactivate()+on_pause()+on_resume()+on_run()DynamicPolynomialTrajectoryGenerator-trajectory_generator_-trajectory_motion_handler_-hover_motion_handler_-tf_handler_+evaluateTrajectory()+evaluateSetpoint()+goalToDynamicWaypoint()+computeYawAnglePathFacing()+computeYawFaceReference()DynamicTrajectory+setWaypoints()+modifyWaypoint()+appendWaypoint()+evaluateTrajectory()+getMinTime()+getMaxTime()TrajectoryMotionHandler+sendTrajectorySetpoints()
```

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/include/generate\_polynomial\_trajectory\_behavior/generate\_polynomial\_trajectory\_behavior.hpp80-251](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/include/generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_behavior.hpp#L80-L251)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp40-133](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L40-L133)

## Key Components

### DynamicPolynomialTrajectoryGenerator

The `DynamicPolynomialTrajectoryGenerator` is the main component that implements the trajectory generation behavior. It inherits from `as2_behavior::BehaviorServer` and manages the creation, evaluation, and execution of polynomial trajectories.

Key responsibilities:

-   Processing waypoint inputs from client behaviors
-   Managing trajectory generation via the `dynamic_trajectory_generator` library
-   Evaluating the trajectory over time
-   Handling different yaw control modes
-   Publishing trajectory setpoints to the motion system

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/include/generate\_polynomial\_trajectory\_behavior/generate\_polynomial\_trajectory\_behavior.hpp80-257](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/include/generate_polynomial_trajectory_behavior/generate_polynomial_trajectory_behavior.hpp#L80-L257)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp42-134](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L42-L134)

### dynamic\_trajectory\_generator Library

The trajectory generator uses an external library called `dynamic_trajectory_generator` for the mathematical computation of polynomial trajectories. This library is automatically fetched from GitHub during the build process if not found on the system.

Key classes from this library:

-   `DynamicTrajectory`: Main class for trajectory generation
-   `DynamicWaypoint`: Represents waypoints in the trajectory

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/CMakeLists.txt3-17](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/CMakeLists.txt#L3-L17)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp54-56](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L54-L56)

## Action Interface

The trajectory generation behavior exposes a ROS2 action interface using the `as2_msgs::action::GeneratePolynomialTrajectory` action definition.

### Action Goal

The goal definition includes:

-   A list of waypoints with unique IDs
-   Maximum speed
-   Yaw control mode
-   Timestamp

```
# Request
builtin_interfaces/Time stamp      # Request timestamp
as2_msgs/YawMode yaw               # Yaw goal mode
as2_msgs/PoseStampedWithID[] path  # List of poses with ID in path, with each frame id and time stamp
float32 max_speed                  # Maximum speed desired in path (m/s)
```

### Action Feedback

The feedback provides information about execution progress:

```
# Feedback
string next_waypoint_id            # Next waypoint id in path to follow
uint16 remaining_waypoints         # Number of remaining waypoints to follow
```

### Action Result

The result indicates success or failure:

```
# Result
bool trajectory_generator_success  # False if failed to follow the generated trajectory
```

Sources:

-   [as2\_msgs/action/GeneratePolynomialTrajectory.action1-17](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/action/GeneratePolynomialTrajectory.action#L1-L17)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp180-250](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L180-L250)

## Yaw Control Modes

The trajectory generator supports multiple yaw control modes, allowing flexible control of the vehicle's heading during trajectory execution.

| Mode ID | Name | Description |
| --- | --- | --- |
| 0 | KEEP\_YAW | Maintains the current yaw angle throughout the trajectory |
| 1 | PATH\_FACING | Aligns yaw with the direction of motion along the path |
| 2 | FIXED\_YAW | Maintains a specified fixed yaw angle |
| 3 | YAW\_FROM\_TOPIC | Sets yaw based on values received from a topic |
| 6 | FACE\_REFERENCE | Yaw angle faces the next waypoint in the trajectory |

The behavior handles these different modes in the `evaluateSetpoint` method, which computes the appropriate yaw angle for each trajectory point based on the selected mode.

Sources:

-   [as2\_msgs/msg/YawMode.msg1-13](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/YawMode.msg#L1-L13)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp631-664](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L631-L664)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp719-759](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L719-L759)

## Trajectory Execution Flow

This diagram illustrates the execution flow of the trajectory generation behavior:

```
"TrajectoryMotionHandler""dynamic_trajectory_generator""DynamicPolynomialTrajectoryGenerator""Client Behavior""TrajectoryMotionHandler""dynamic_trajectory_generator""DynamicPolynomialTrajectoryGenerator""Client Behavior"loop[until trajectory completion]Send goal (waypoints, speed, yaw mode)on_activate()setWaypoints()on_run()evaluateTrajectory(time)return trajectory pointcompute yaw based on yaw_modesendTrajectorySetpoints()Feedback (next_waypoint_id, remaining_waypoints)on_execution_end()Result (trajectory_generator_success)
```

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp235-505](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L235-L505)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp507-592](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L507-L592)

## Trajectory Generation Process

The trajectory generation process involves the following steps:

1.  **Goal Processing**: Converting the received waypoints into the format expected by the `dynamic_trajectory_generator` library.
2.  **Trajectory Setup**: Setting up the trajectory parameters and initializing the trajectory generator.
3.  **Trajectory Evaluation**: Evaluating the trajectory at specific time points to generate setpoints.
4.  **Command Publishing**: Publishing trajectory setpoints to the motion system.

```
Command PublishingTrajectory EvaluationGoal ProcessingNot CompleteCompleteReceive Action GoalConvert to DynamicWaypointsSet Waypoints in GeneratorCalculate Current TimeEvaluate Trajectory at TimeCompute Yaw AngleCreate Trajectory SetpointsSend Trajectory to Motion HandlerUpdate FeedbackCheck CompletionReturn Success
```

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp180-233](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L180-L233)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp507-592](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L507-L592)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp594-678](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L594-L678)

## Configuration Parameters

The trajectory generator behavior can be configured with various parameters to control its operation. These parameters are defined in `config_default.yaml`:

| Parameter | Description | Default Value |
| --- | --- | --- |
| sampling\_n | Number of trajectory points to sample | 1 |
| sampling\_dt | Time interval between samples (seconds) | 0.01 |
| path\_lenght | Number of waypoints to process at once (0 for all) | 0 |
| yaw\_threshold | Threshold for yaw angle calculations (meters) | 0.1 |
| yaw\_speed\_threshold | Maximum yaw speed (rad/s) | 6.0 |
| frequency\_update\_frame | Frequency to update frame transforms (Hz) | 0.0 |
| transform\_threshold | Threshold for frame transform updates (meters) | 1.0 |
| wp\_close\_threshold | Threshold for waypoint updates (seconds) | 0.0 |

Sources:

-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/config/config\_default.yaml1-16](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/config/config_default.yaml#L1-L16)
-   [as2\_behaviors/as2\_behaviors\_trajectory\_generation/generate\_polynomial\_trajectory\_behavior/src/generate\_polynomial\_trajectory\_behavior.cpp80-91](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_trajectory_generation/generate_polynomial_trajectory_behavior/src/generate_polynomial_trajectory_behavior.cpp#L80-L91)

## Usage in Motion Behaviors

The trajectory generation system is used by several motion behaviors to provide smooth movement for drones:

### Takeoff Behavior

The takeoff behavior uses trajectory generation to create a smooth vertical ascent to the desired takeoff height. It sends a single waypoint at the current X-Y position with the target Z height.

Sources:

-   [as2\_behaviors/as2\_behaviors\_motion/takeoff\_behavior/plugins/takeoff\_plugin\_trajectory.cpp52-228](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/takeoff_behavior/plugins/takeoff_plugin_trajectory.cpp#L52-L228)

### Land Behavior

The land behavior uses trajectory generation to create a controlled descent to the ground. It sends a waypoint at the current X-Y position with a negative Z value, and monitors the drone's state to detect when landing is complete.

Sources:

-   [as2\_behaviors/as2\_behaviors\_motion/land\_behavior/plugins/land\_plugin\_trajectory.cpp54-303](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/land_behavior/plugins/land_plugin_trajectory.cpp#L54-L303)

### Go To Behavior

The go-to behavior uses trajectory generation to create a path from the current position to a target position. It handles different yaw modes and provides feedback during motion.

Sources:

-   [as2\_behaviors/as2\_behaviors\_motion/go\_to\_behavior/plugins/go\_to\_plugin\_trajectory.cpp54-267](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_trajectory.cpp#L54-L267)

### Follow Path Behavior

The follow-path behavior uses trajectory generation to follow a sequence of waypoints. It provides feedback about the current and next waypoints during execution.

Sources:

-   [as2\_behaviors/as2\_behaviors\_motion/follow\_path\_behavior/plugins/follow\_path\_plugin\_trajectory.cpp52-304](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_motion/follow_path_behavior/plugins/follow_path_plugin_trajectory.cpp#L52-L304)

## Python API Interface

Aerostack2 provides a Python API for interacting with the trajectory generation system. The `TrajectoryGenerationModule` class offers a simplified interface for creating and executing trajectories.

Example methods:

-   `traj_generation_with_keep_yaw`: Generate trajectory maintaining current yaw
-   `traj_generation_with_yaw`: Generate trajectory with a fixed yaw angle
-   `traj_generation_with_path_facing`: Generate trajectory with path-facing yaw
-   `traj_generation_with_face_reference`: Generate trajectory facing the next reference point

Sources:

-   [as2\_python\_api/as2\_python\_api/modules/trajectory\_generation\_module.py1-171](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/modules/trajectory_generation_module.py#L1-L171)
-   [as2\_python\_api/as2\_python\_api/behavior\_actions/trajectory\_generation\_behavior.py1-163](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_python_api/as2_python_api/behavior_actions/trajectory_generation_behavior.py#L1-L163)