Relevant source files

-   [as2\_state\_estimator/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/CMakeLists.txt)
-   [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp)
-   [as2\_state\_estimator/launch/ground\_truth\_odometry\_fuse\_state\_estimator.launch.py](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/launch/ground_truth_odometry_fuse_state_estimator.launch.py)
-   [as2\_state\_estimator/package.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/package.xml)
-   [as2\_state\_estimator/plugins.xml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins.xml)
-   [as2\_state\_estimator/plugins/ground\_truth/config/plugin\_default.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth/config/plugin_default.yaml)
-   [as2\_state\_estimator/plugins/ground\_truth/include/ground\_truth.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth/include/ground_truth.hpp)
-   [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/CMakeLists.txt)
-   [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/config/plugin\_default.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/config/plugin_default.yaml)
-   [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/include/ground\_truth\_odometry\_fuse.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/include/ground_truth_odometry_fuse.hpp)
-   [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/src/ground\_truth\_odometry\_fuse.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/src/ground_truth_odometry_fuse.cpp)
-   [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/tests/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/tests/CMakeLists.txt)
-   [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/tests/ground\_truth\_odometry\_fuse\_gtest.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/tests/ground_truth_odometry_fuse_gtest.cpp)
-   [as2\_state\_estimator/plugins/mocap\_pose/include/mocap\_pose.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/mocap_pose/include/mocap_pose.hpp)
-   [as2\_state\_estimator/plugins/mocap\_pose/src/mocap\_pose.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/mocap_pose/src/mocap_pose.cpp)
-   [as2\_state\_estimator/plugins/raw\_odometry/CMakeLists.txt](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/CMakeLists.txt)
-   [as2\_state\_estimator/plugins/raw\_odometry/config/plugin\_default.yaml](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/config/plugin_default.yaml)
-   [as2\_state\_estimator/plugins/raw\_odometry/include/raw\_odometry.hpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/include/raw_odometry.hpp)
-   [as2\_state\_estimator/plugins/raw\_odometry/src/raw\_odometry.cpp](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/src/raw_odometry.cpp)

The State Estimation system in Aerostack2 provides a modular framework for estimating the position, orientation, and velocity of aerial robots. This page documents the architecture and usage of the state estimation system, including the plugin-based design and available implementations. For information on motion control that uses the state estimates, see [Motion Control](https://deepwiki.com/aerostack2/aerostack2/2.3-motion-control).

## Purpose and Scope

The state estimator's primary responsibility is to:

-   Process sensor data to determine the drone's state (position, orientation, velocity)
-   Maintain coordinate frame transformations between different reference frames
-   Publish pose and twist information for use by other components
-   Provide a consistent interface regardless of the underlying estimation technique

## Architecture Overview

The state estimation system uses a plugin-based architecture that allows for different estimation methods to be used interchangeably without changing the rest of the system.

```
StateEstimatorBase+setup()+on_setup()+get_earth_to_map_transform()+publish_transform()+publish_static_transform()+publish_pose()+publish_twist()RawOdometryPlugin+on_setup()-odom_callback()GroundTruthPlugin+on_setup()-pose_callback()-twist_callback()MocapPosePlugin+on_setup()-rigid_bodies_callback()GroundTruthOdometryFusePlugin+on_setup()-odomCallback()-groundTruthCallback()
```

**State Estimator Plugin Architecture**

Sources: [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp62-208](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp#L62-L208) [as2\_state\_estimator/plugins.xml1-22](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins.xml#L1-L22)

## Coordinate Frames

The state estimator manages transformations between several coordinate frames:

```
Coordinate Framesearth_to_mapmap_to_odomodom_to_baseearth_framemap_frameodom_framebase_frame
```

**Coordinate Frame Relationships**

-   **earth\_frame**: Global reference frame (world frame)
-   **map\_frame**: Local map frame, fixed relative to earth
-   **odom\_frame**: Odometry frame for continuous localization
-   **base\_frame**: Body frame of the drone

Sources: [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp68-74](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp#L68-L74) [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp161-169](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp#L161-L169)

## Plugin Base Implementation

The `StateEstimatorBase` class provides common functionality for all state estimator plugins:

```
StateEstimatorBase-string earth_frame_id_-string map_frame_id_-string odom_frame_id_-string base_frame_id_-tf2::Transform earth_to_map_-tf2::Transform map_to_odom_-tf2::Transform odom_to_base_-tf2::Transform earth_to_baselink+setup(node, tf_handler, tf_broadcaster, static_tf_broadcaster)+on_setup()+get_earth_to_map_transform()+publish_transform()+publish_static_transform()+publish_pose()+publish_twist()+get_earth_frame()+get_map_frame()+get_odom_frame()+get_base_frame()
```

**StateEstimatorBase Class**

Key responsibilities:

-   Maintains references to coordinate frames
-   Handles transform operations between frames
-   Provides methods for publishing transforms, poses, and twists
-   Defines the plugin interface through virtual methods

Sources: [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp62-208](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp#L62-L208)

## Data Flow

The state estimator processes sensor data and produces state estimates that are used by other components:

```
OutputsState EstimatorSensor InputsOdometry(nav_msgs/Odometry)Ground Truth(geometry_msgs/PoseStamped)Motion Capture(mocap4r2_msgs/RigidBodies)GPS(sensor_msgs/NavSatFix)Selected Plugin(raw_odometry, ground_truth, etc.)Pose(geometry_msgs/PoseStamped)Twist(geometry_msgs/TwistStamped)Transform Broadcasts(tf2)
```

**State Estimator Data Flow**

Sources: [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp89-92](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp#L89-L92) [as2\_state\_estimator/plugins/raw\_odometry/include/raw\_odometry.hpp89-91](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/include/raw_odometry.hpp#L89-L91) [as2\_state\_estimator/plugins/ground\_truth/include/ground\_truth.hpp90-95](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth/include/ground_truth.hpp#L90-L95)

## Available Plugins

Aerostack2 provides several state estimation plugins for different scenarios:

### Raw Odometry Plugin

Uses external odometry data, optionally with GPS for global reference:

```
OutputsRaw Odometry PluginInputs/sensor_measurements/odom/sensor_measurements/gpsodom_callback()gps_callback()setOriginCallback()getOriginCallback()/self_localization/pose/self_localization/twisttf broadcasts
```

**Raw Odometry Plugin Flow**

Sources: [as2\_state\_estimator/plugins/raw\_odometry/include/raw\_odometry.hpp61-330](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/include/raw_odometry.hpp#L61-L330)

### Ground Truth Plugin

Uses ground truth data, typically from a simulation environment:

```
OutputsGround Truth PluginInputs/ground_truth/pose/ground_truth/twist/sensor_measurements/gpspose_callback()twist_callback()gps_callback()/self_localization/pose/self_localization/twisttf broadcasts
```

**Ground Truth Plugin Flow**

Sources: [as2\_state\_estimator/plugins/ground\_truth/include/ground\_truth.hpp60-300](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth/include/ground_truth.hpp#L60-L300)

### Motion Capture Pose Plugin

Designed for use with motion capture systems:

```
OutputsMocap Pose PluginInputsmocap_topic(configured parameter)rigid_bodies_callback()process_mocap_pose()twist_from_pose()/self_localization/pose/self_localization/twisttf broadcasts
```

**Mocap Pose Plugin Flow**

Sources: [as2\_state\_estimator/plugins/mocap\_pose/include/mocap\_pose.hpp56-254](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/mocap_pose/include/mocap_pose.hpp#L56-L254)

### Ground Truth Odometry Fusion Plugin

Fuses ground truth and odometry data:

```
OutputsGround Truth Odometry Fusion PluginInputsodom_topic(configured parameter)ground_truth_topic(configured parameter)mocap_topic(configured parameter)odomCallback()groundTruthCallback()rigidBodiesCallback()getTransform()/self_localization/pose/self_localization/twisttf broadcasts
```

**Ground Truth Odometry Fusion Plugin Flow**

Sources: [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/include/ground\_truth\_odometry\_fuse.hpp63-339](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/include/ground_truth_odometry_fuse.hpp#L63-L339)

## Configuration

Each plugin can be configured using YAML parameter files. Below are common configuration parameters:

### Raw Odometry Plugin Configuration

```
/**:
  ros__parameters:
    use_gps: false  # Use GPS data
    set_origin_on_start: false  # Set origin with first GPS data
    earth_to_map:  # Manual map frame offset
      x: 0.0
      y: 0.0
      z: 0.0
    set_map_to_odom: true  # Set map to odom transform
```

Sources: [as2\_state\_estimator/plugins/raw\_odometry/config/plugin\_default.yaml1-15](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/raw_odometry/config/plugin_default.yaml#L1-L15)

### Ground Truth Plugin Configuration

```
/**:
  ros__parameters:
    use_gps: false  # Use GPS data
    set_origin_on_start: false  # Set origin with first GPS data
    use_gazebo_tf: false  # Use the gazebo tf tree
```

Sources: [as2\_state\_estimator/plugins/ground\_truth/config/plugin\_default.yaml1-9](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth/config/plugin_default.yaml#L1-L9)

### Ground Truth Odometry Fusion Plugin Configuration

```
/**:
  ros__parameters:
    odom_topic: "sensor_measurements/odom"  # Odometry topic
    ground_truth_topic: "ground_truth/pose"  # Ground truth topic
    mocap_topic: "/mocap/rigid_bodies"  # Motion capture topic
    rigid_body_name: ""  # Rigid body name (if using mocap)
```

Sources: [as2\_state\_estimator/plugins/ground\_truth\_odometry\_fuse/config/plugin\_default.yaml1-7](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins/ground_truth_odometry_fuse/config/plugin_default.yaml#L1-L7)

## Usage

To use the state estimator, you need to:

1.  Choose an appropriate plugin for your use case
2.  Configure the plugin parameters
3.  Launch the state estimator node with the selected plugin

Aerostack2 provides launch files for commonly used configurations:

```
# Example: Launch state estimator with ground truth odometry fusion
ros2 launch as2_state_estimator ground_truth_odometry_fuse_state_estimator.launch.py
```

Sources: [as2\_state\_estimator/launch/ground\_truth\_odometry\_fuse\_state\_estimator.launch.py29-47](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/launch/ground_truth_odometry_fuse_state_estimator.launch.py#L29-L47)

## Implementation Details

### State Estimator Base Class

The `StateEstimatorBase` class provides methods for:

-   Frame management (`get_earth_frame()`, `get_map_frame()`, etc.)
-   Transform broadcasting (`publish_transform()`, `publish_static_transform()`)
-   State publishing (`publish_pose()`, `publish_twist()`)
-   Transform conversion between different frames

Sources: [as2\_state\_estimator/include/as2\_state\_estimator/plugin\_base.hpp62-208](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/include/as2_state_estimator/plugin_base.hpp#L62-L208)

### Plugin Registration

Plugins are registered using the ROS 2 pluginlib mechanism:

```
<class_libraries>
  <library path="raw_odometry">
    <class type="raw_odometry::Plugin" base_class_type="as2_state_estimator_plugin_base::StateEstimatorBase">
      <description>State estimator plugin for external odom estimation.</description>
    </class>
  </library>
  <!-- Other plugins -->
</class_libraries>
```

Sources: [as2\_state\_estimator/plugins.xml1-22](https://github.com/aerostack2/aerostack2/blob/10200cd9/as2_state_estimator/plugins.xml#L1-L22)

## Summary

The state estimation system in Aerostack2 provides a flexible, plugin-based approach to estimating the drone's position, orientation, and velocity. Multiple plugins are available for different scenarios, such as using external odometry, ground truth data, or motion capture systems. The system maintains a consistent interface regardless of the underlying estimation method, making it easy to swap between different approaches as needed.