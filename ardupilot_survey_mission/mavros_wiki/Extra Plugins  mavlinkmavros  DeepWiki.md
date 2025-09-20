Relevant source files

-   [mavros\_extras/CMakeLists.txt](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt)
-   [mavros\_extras/mavros\_plugins.xml](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml)
-   [mavros\_extras/src/plugins/onboard\_computer\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/onboard_computer_status.cpp)
-   [mavros\_msgs/CMakeLists.txt](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/CMakeLists.txt)

## Overview

The MAVROS Extra Plugins provide additional functionality beyond the core capabilities of MAVROS. While [Core Plugins](https://deepwiki.com/mavlink/mavros/3.1-core-plugins) offer essential features for flight control, parameter management, and basic telemetry, Extra Plugins extend MAVROS with specialized capabilities for sensor integration, external positioning systems, vision-based estimation, companion computer integration, and more.

This page provides an overview of the MAVROS Extra Plugins system, its architecture, and the available plugins. For detailed information about specific plugin categories, see [Sensor Plugins](https://deepwiki.com/mavlink/mavros/3.2.1-sensor-plugins), [Vision and Motion Capture Plugins](https://deepwiki.com/mavlink/mavros/3.2.2-vision-and-motion-capture-plugins), and [Companion Computer Integration](https://deepwiki.com/mavlink/mavros/3.2.3-companion-computer-integration).

Sources: [mavros\_extras/CMakeLists.txt91-132](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L91-L132) [mavros\_extras/mavros\_plugins.xml1-10](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L1-L10)

## Plugin Architecture

MAVROS Extra Plugins exist as a separate package (`mavros_extras`) but follow the same plugin architecture as core plugins. Each plugin inherits from the base `Plugin` class and optionally from specialized mixins like `TF2ListenerMixin` for transformation capabilities.

```
MAVLink EnvironmentMAVROS SystemROS EnvironmentPlugin PackagesBase FrameworkROS NodeROS Topics/ServicesPlugin Base ClassUAS (Plugin Container)RouterCore Plugins PackagemavrosFlight Control Unit
```

Sources: [mavros\_extras/CMakeLists.txt23-32](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L23-L32) [mavros\_extras/mavros\_plugins.xml1-10](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L1-L10)

### Plugin Registration

Extra plugins are registered using the `MAVROS_PLUGIN_REGISTER` macro, which makes them discoverable through ROS pluginlib. The registration is defined in the `mavros_plugins.xml` file, which specifies each plugin's name, type, and description.

Sources: [mavros\_extras/mavros\_plugins.xml7-242](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L7-L242) [mavros\_extras/src/plugins/onboard\_computer\_status.cpp121-122](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/onboard_computer_status.cpp#L121-L122)

The mavros\_extras package provides over 30 specialized plugins. These can be categorized by their functionality:

```
Sensor IntegrationPluginsExternal PositioningPluginsCommunication & StatusPluginsCamera & VisionPluginsGPS & NavigationPluginsControlPluginsMiscellaneousPluginsdistance_sensoresc_statusesc_telemetryvibrationoptical_flowpx4flowvision_posevision_speedmocap_pose_estimatefake_gpswheel_odometryodom3dr_radioonboard_computer_statuscompanion_process_statusdebug_valuecameracam_imu_synclanding_targetgps_rtkgps_statusobstacle_distancetrajectorymount_controlgimbal_controlguided_targetadsbhilplay_tune
```

Sources: [mavros\_extras/CMakeLists.txt91-132](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L91-L132) [mavros\_extras/mavros\_plugins.xml8-241](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L8-L241)

### Plugin Categories

| Category | Plugins | Description |
| --- | --- | --- |
| Sensor Integration | distance\_sensor, esc\_status, esc\_telemetry, rangefinder, vibration, optical\_flow, px4flow | Connect external sensors to the flight controller |
| External Positioning | vision\_pose, vision\_speed, mocap\_pose\_estimate, fake\_gps, wheel\_odometry, odom | Provide external position and velocity estimates |
| Communication & Status | 3dr\_radio, onboard\_computer\_status, companion\_process\_status, debug\_value, tunnel | Report status information and handle communications |
| Camera & Vision | camera, cam\_imu\_sync, landing\_target | Control cameras and process vision data |
| GPS & Navigation | gps\_rtk, gps\_status, gps\_input, terrain, obstacle\_distance, trajectory | Enhance GPS and navigation capabilities |
| Control | mount\_control, gimbal\_control, guided\_target | Control external actuators and guidance |
| Miscellaneous | adsb, cellular\_status, mag\_calibration\_status, hil, play\_tune, log\_transfer, vfr\_hud | Various specialized functionalities |

Sources: [mavros\_extras/CMakeLists.txt91-132](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/CMakeLists.txt#L91-L132) [mavros\_extras/mavros\_plugins.xml8-241](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L8-L241)

## Example Plugin: Onboard Computer Status

The Onboard Computer Status plugin demonstrates a typical plugin implementation pattern. It allows reporting status information about the companion computer to the flight controller.

```
"Flight Controller""UAS""OnboardComputerStatusPlugin""ROS Application""Flight Controller""UAS""OnboardComputerStatusPlugin""ROS Application"Plugin registered in constructorPublish OnboardComputerStatus messagestatus_cb() converts to MAVLink formatsend_message()ONBOARD_COMPUTER_STATUS MAVLink message
```

The plugin subscribes to a ROS topic (`~/status`), processes incoming messages, converts them to MAVLink format, and sends them to the flight controller.

Sources: [mavros\_extras/src/plugins/onboard\_computer\_status.cpp16-123](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/onboard_computer_status.cpp#L16-L123)

Unlike core plugins which are loaded by default, extra plugins must be explicitly enabled in your launch configuration. To use an extra plugin:

1.  Include the extra plugin in your MAVROS launch file
2.  Configure plugin-specific parameters as needed

### Example: Loading the vision\_pose plugin

```
<launch>
  <arg name="fcu_url" default="..." />
  <arg name="gcs_url" default="..." />

  <node pkg="mavros" type="mavros_node" name="mavros" output="screen">
    <param name="fcu_url" value="$(arg fcu_url)" />
    <param name="gcs_url" value="$(arg gcs_url)" />
    
    <!-- Load extra plugins -->
    <param name="plugin_lists">
      <yaml>
        - name: vision_pose
          type: mavros::extra_plugins::VisionPoseEstimatePlugin
      </yaml>
    </param>
    
    <!-- Plugin specific parameters -->
    <param name="vision_pose/tf/listen" value="true" />
    <param name="vision_pose/tf/frame_id" value="map" />
    <param name="vision_pose/tf/child_frame_id" value="camera_link" />
  </node>
</launch>
```

Sources: [mavros\_extras/mavros\_plugins.xml221-227](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/mavros_plugins.xml#L221-L227)

## Plugin Implementation Structure

All extra plugins follow a common implementation pattern:

1.  Include necessary headers (mavros, mavros\_msgs, etc.)
2.  Implement a plugin class that inherits from `mavros::plugin::Plugin`
3.  Define constructors, subscriptions, and message handlers
4.  Register the plugin using the `MAVROS_PLUGIN_REGISTER` macro

The implementation typically includes:

-   Constructor that sets up ROS publishers and subscribers
-   `get_subscriptions()` method to define MAVLink messages to subscribe to
-   Callback methods for handling ROS and MAVLink messages
-   Conversion functions between ROS and MAVLink message formats

Sources: [mavros\_extras/src/plugins/onboard\_computer\_status.cpp16-123](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/onboard_computer_status.cpp#L16-L123)

## Summary

MAVROS Extra Plugins extend the core MAVROS functionality with additional features for specialized use cases. These plugins follow the same architecture as core plugins but are packaged separately and must be explicitly enabled.

For more details on specific plugin categories, refer to:

-   [Sensor Plugins](https://deepwiki.com/mavlink/mavros/3.2.1-sensor-plugins)
-   [Vision and Motion Capture Plugins](https://deepwiki.com/mavlink/mavros/3.2.2-vision-and-motion-capture-plugins)
-   [Companion Computer Integration](https://deepwiki.com/mavlink/mavros/3.2.3-companion-computer-integration)