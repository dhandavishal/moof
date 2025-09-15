Relevant source files

-   [as2\_behaviors/as2\_behaviors\_perception/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/include/detect\_aruco\_markers\_behavior/detect\_aruco\_markers\_behavior.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/include/detect_aruco_markers_behavior/detect_aruco_markers_behavior.hpp)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/launch/detect\_aruco\_markers\_behavior\_real\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/launch/detect_aruco_markers_behavior_real_launch.py)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/launch/detect\_aruco\_markers\_behavior\_sim\_launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/launch/detect_aruco_markers_behavior_sim_launch.py)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/src/detect\_aruco\_markers\_behavior.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/src/detect_aruco_markers_behavior.cpp)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/src/detect\_aruco\_markers\_behavior\_node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/src/detect_aruco_markers_behavior_node.cpp)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/config/config\_default.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/config/config_default.yaml)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/include/point\_gimbal\_behavior/point\_gimbal\_behavior.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/include/point_gimbal_behavior/point_gimbal_behavior.hpp)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/launch/point\_gimbal\_behavior.launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/launch/point_gimbal_behavior.launch.py)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/src/point\_gimbal\_behavior.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/src/point_gimbal_behavior.cpp)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/src/point\_gimbal\_behavior\_node.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/src/point_gimbal_behavior_node.cpp)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/tests/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/tests/CMakeLists.txt)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/tests/point\_gimbal\_gtest.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/tests/point_gimbal_gtest.cpp)
-   [as2\_msgs/action/PointGimbal.action](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/action/PointGimbal.action)
-   [as2\_msgs/msg/GimbalControl.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/GimbalControl.msg)
-   [as2\_msgs/msg/PoseStampedWithIDArray.msg](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/msg/PoseStampedWithIDArray.msg)

## Overview

Perception Behaviors in Aerostack2 provide functionality related to sensors and perception capabilities of aerial robots. These behaviors handle tasks such as controlling onboard cameras/gimbals and processing sensor data to detect objects or features in the environment. They are implemented as ROS 2 Action Servers, following the Aerostack2 behavior architecture.

This document covers the perception behaviors included in the Aerostack2 framework, their architecture, configuration, and usage. For information about the general Behavior System architecture, see [Behavior System](https://deepwiki.com/aerostack2/aerostack2/3-behavior-system).

Sources: [as2\_behaviors/as2\_behaviors\_perception/CMakeLists.txt40-42](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/CMakeLists.txt#L40-L42)

## Architecture Overview

Perception behaviors in Aerostack2 inherit from the `BehaviorServer` template class in the `as2_behavior` package. They follow a standard interface with methods for activating, deactivating, running, pausing, and resuming the behavior.

```
BehaviorServer<T>+on_activate(goal)+on_modify(goal)+on_deactivate(message)+on_pause(message)+on_resume(message)+on_run(goal, feedback, result)+on_execution_end(status)PointGimbalBehavior-tf_handler_: as2::tf::TfHandler-gimbal_control_pub_: Publisher-gimbal_name_: string-gimbal_threshold_: double-check_gimbal_limits()-update_gimbal_state()-check_finished()DetectArucoMarkersBehavior-aruco_pose_pub_: Publisher-aruco_img_transport_: Camera-cam_image_sub_: Subscription-cam_info_sub_: Subscription-setCameraParameters()-imageCallback()-camerainfoCallback()-checkIdIsTarget()
```

Perception behaviors are instantiated as ROS 2 nodes that can be launched individually or as part of a larger system. They interact with the rest of the Aerostack2 system through ROS 2 topics and services.

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/include/point\_gimbal\_behavior/point\_gimbal\_behavior.hpp63-124](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/include/point_gimbal_behavior/point_gimbal_behavior.hpp#L63-L124)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/include/detect\_aruco\_markers\_behavior/detect\_aruco\_markers\_behavior.hpp63-123](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/include/detect_aruco_markers_behavior/detect_aruco_markers_behavior.hpp#L63-L123)

## Point Gimbal Behavior

The Point Gimbal Behavior controls a gimbal to point at a specific target location in 3D space. This behavior is useful for tracking objects, pointing cameras in specific directions, or maintaining visual focus on areas of interest during flight.

### Diagram: Point Gimbal Behavior Data Flow

```
TF SystemROS2 Topic InterfacePointGimbalBehaviorROS2 Action InterfaceAction GoalAction FeedbackAction Resulton_activate()on_run()check_gimbal_limits()update_gimbal_state()check_finished()tf_handler_gimbal_command(as2_msgs/GimbalControl)Transform Tree
```

### Configuration

The Point Gimbal Behavior can be configured with the following parameters:

| Parameter | Type | Description |
| --- | --- | --- |
| `gimbal_name` | string | Name of the gimbal device |
| `gimbal_frame_id` | string | Name of the gimbal frame in TF |
| `gimbal_base_frame_id` | string | Name of the gimbal base frame in TF |
| `gimbal_threshold` | double | Threshold angle for considering the gimbal pointed at target |
| `behavior_timeout` | double | Timeout in seconds for the behavior |
| `roll_range.min/max` | double | Min/max roll angle limits (radians) |
| `pitch_range.min/max` | double | Min/max pitch angle limits (radians) |
| `yaw_range.min/max` | double | Min/max yaw angle limits (radians) |

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/config/config\_default.yaml1-17](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/config/config_default.yaml#L1-L17)

### Usage

The Point Gimbal Behavior is controlled through a ROS 2 Action API defined by the `as2_msgs/action/PointGimbal` message type:

-   **Goal**:
    -   `control`: Defines the target to point at (position or orientation)
    -   `follow_mode`: Whether to keep following after reaching the target
-   **Feedback**:
    -   `gimbal_attitude`: Current gimbal orientation
    -   `gimbal_speed`: Current gimbal rotation speed
-   **Result**:
    -   `success`: Whether the gimbal successfully reached the target

#### Implementation Details

The behavior performs the following steps:

1.  Receives a target point to look at in any frame
2.  Transforms the target to the gimbal's base frame
3.  Calculates the required gimbal orientation (roll, pitch, yaw) to point at the target
4.  Checks if the required orientation is within limits
5.  Commands the gimbal to the desired orientation
6.  Monitors gimbal's orientation to determine when the target is reached
7.  Returns success when the gimbal is pointing at the target (within threshold)

The behavior publishes gimbal commands on `platform/<gimbal_name>/gimbal_command` topic and uses the TF system to track the current gimbal orientation.

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/src/point\_gimbal\_behavior.cpp44-204](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/src/point_gimbal_behavior.cpp#L44-L204)
-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/src/point\_gimbal\_behavior.cpp231-260](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/src/point_gimbal_behavior.cpp#L231-L260)
-   [as2\_msgs/action/PointGimbal.action1-12](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_msgs/action/PointGimbal.action#L1-L12)

## Detect ArUco Markers Behavior

The Detect ArUco Markers Behavior processes camera images to detect [ArUco markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html), which are square fiducial markers used for camera pose estimation and object tracking. The behavior estimates the 3D pose of detected markers relative to the camera.

### Diagram: ArUco Marker Detection Data Flow

```
ROS2 Action InterfaceROS2 PublicationsDetectArucoMarkersBehaviorROS2 SubscriptionsFilter markerscamera_image_topic(sensor_msgs/Image)camera_info_topic(sensor_msgs/CameraInfo)imageCallback()camerainfoCallback()setCameraParameters()ArUco Detection(OpenCV)Pose EstimationcheckIdIsTarget()aruco_pose(as2_msgs/PoseStampedWithIDArray)aruco_img_topic(Image with visualized markers)Action Goal(target_ids)Action Result
```

### Configuration

The Detect ArUco Markers Behavior can be configured with the following parameters:

| Parameter | Type | Description |
| --- | --- | --- |
| `aruco_size` | float | Size of the ArUco marker in meters |
| `camera_qos_reliable` | bool | Whether to use reliable QoS for camera subscription |
| `camera_image_topic` | string | Topic for camera images |
| `camera_info_topic` | string | Topic for camera calibration information |
| `camera_model` | string | Camera model type ("pinhole" or "fisheye") |

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/src/detect\_aruco\_markers\_behavior.cpp70-90](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/src/detect_aruco_markers_behavior.cpp#L70-L90)

### Usage

The Detect ArUco Markers Behavior is controlled through a ROS 2 Action API defined by the `as2_msgs/action/DetectArucoMarkers` message type:

-   **Goal**:
    -   `target_ids`: List of ArUco marker IDs to detect (empty list means detect all)
-   **Result**:
    -   The behavior runs indefinitely and publishes detected markers

The behavior publishes detected markers on the `aruco_pose` topic using the `as2_msgs/msg/PoseStampedWithIDArray` message type, which includes the marker ID and 3D pose (position and orientation) of each detected marker.

Additionally, it publishes a visualization image showing detected markers on the `aruco_img_topic` topic.

#### Implementation Details

The behavior performs the following steps:

1.  Receives camera images and camera calibration information
2.  Processes images to detect ArUco markers using OpenCV
3.  Estimates the 3D pose of detected markers relative to the camera
4.  Filters markers based on the target IDs specified in the goal
5.  Publishes the pose of detected markers
6.  Creates and publishes a visualization image showing detected markers

The behavior uses the OpenCV ArUco module for marker detection and pose estimation.

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/src/detect\_aruco\_markers\_behavior.cpp144-245](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/src/detect_aruco_markers_behavior.cpp#L144-L245)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/src/detect\_aruco\_markers\_behavior.cpp260-303](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/src/detect_aruco_markers_behavior.cpp#L260-L303)

## Launching Perception Behaviors

Perception behaviors can be launched using the provided launch files:

### Point Gimbal Behavior

```
ros2 launch as2_behaviors_perception point_gimbal_behavior.launch.py namespace:=drone_1
```

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/point\_gimbal\_behavior/launch/point\_gimbal\_behavior.launch.py47-73](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/point_gimbal_behavior/launch/point_gimbal_behavior.launch.py#L47-L73)

### Detect ArUco Markers Behavior

For simulation:

```
ros2 launch as2_behaviors_perception detect_aruco_markers_behavior_sim_launch.py namespace:=drone_1
```

For real hardware:

```
ros2 launch as2_behaviors_perception detect_aruco_markers_behavior_real_launch.py namespace:=drone_1
```

Sources:

-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/launch/detect\_aruco\_markers\_behavior\_sim\_launch.py45-66](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/launch/detect_aruco_markers_behavior_sim_launch.py#L45-L66)
-   [as2\_behaviors/as2\_behaviors\_perception/detect\_aruco\_markers\_behavior/launch/detect\_aruco\_markers\_behavior\_real\_launch.py45-66](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_behaviors/as2_behaviors_perception/detect_aruco_markers_behavior/launch/detect_aruco_markers_behavior_real_launch.py#L45-L66)

## Integration with Aerostack2

Perception behaviors are part of the Aerostack2 behavior system and can be used alongside other behaviors for complex missions. They can be triggered using the Python API, Mission Interpreter, or directly using ROS 2 action clients.

### Example: Using Point Gimbal Behavior with Python API

```
from as2_python_api.drone_interface import DroneInterface
import rclpy

rclpy.init()
drone = DroneInterface("drone_0")

# Point gimbal at a specific point
result = drone.point_gimbal(
    point=[1.0, 0.0, 0.0],  # Point 1 meter in front of the drone
    frame_id="drone_0_base_link"  # Reference frame
)

# Check result
if result.success:
    print("Gimbal pointed successfully")
else:
    print("Failed to point gimbal")
```