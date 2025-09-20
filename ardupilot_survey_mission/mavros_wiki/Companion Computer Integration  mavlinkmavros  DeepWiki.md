Relevant source files

-   [mavros/src/lib/enum\_to\_string.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/enum_to_string.cpp)
-   [mavros\_extras/src/plugins/companion\_process\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/companion_process_status.cpp)
-   [mavros\_extras/src/plugins/mocap\_pose\_estimate.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mocap_pose_estimate.cpp)
-   [mavros\_extras/src/plugins/px4flow.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/px4flow.cpp)
-   [mavros\_extras/src/plugins/vibration.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vibration.cpp)
-   [mavros\_extras/src/plugins/vision\_pose\_estimate.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_pose_estimate.cpp)
-   [mavros\_extras/src/plugins/vision\_speed\_estimate.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_speed_estimate.cpp)
-   [mavros\_msgs/msg/CompanionProcessStatus.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/CompanionProcessStatus.msg)

This document describes how to integrate a companion computer with MAVROS and how to report the status of companion computer processes to the flight controller unit (FCU). Companion computers are secondary computing devices attached to unmanned vehicles to handle tasks that require more processing power than what the flight controller provides, such as computer vision, obstacle avoidance, or advanced navigation.

For implementing companion computer applications using ROS, see [Plugin System](https://deepwiki.com/mavlink/mavros/3-plugin-system). For information on how companion computers connect to the flight controller, see [Connection System](https://deepwiki.com/mavlink/mavros/2.3-connection-system).

## Overview of Companion Computer Integration

Companion computers typically run higher-level applications that interact with the flight controller through MAVROS. These applications can send various types of data to the flight controller, such as vision pose estimates, motion capture data, or obstacle detection information. Additionally, they can report their operational status to the flight controller.

```
Flight ControllerCompanion ComputerROS MessagesROS MessagesProcesses DataMAVLink MessagesMAVLink MessagesROS MessagesROS MessagesROS MessagesStatus ReportingStatus RequestsROS SystemMAVROS NodeMAVROS PluginsCompanion Processes(Vision, Avoidance, etc.)Flight Controller UnitPosition/Attitude EstimatorFlight Controller
```

Sources: [mavros\_extras/src/plugins/companion\_process\_status.cpp1-94](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/companion_process_status.cpp#L1-L94)

## Companion Process Status Reporting

The Companion Process Status plugin allows reporting the status of processes running on the companion computer to the flight controller. This is important for the flight controller to know if critical processes like obstacle avoidance or visual inertial odometry are functioning properly.

### How Status Reporting Works

The plugin sends status information using MAVLink HEARTBEAT messages to inform the flight controller about the state of companion processes. The process can be in different states such as UNINIT, BOOT, CALIBRATING, STANDBY, ACTIVE, CRITICAL, EMERGENCY, etc.

```
Flight ControllerMAVROS CoreCompanionProcessStatusPluginCompanion ProcessFlight ControllerMAVROS CoreCompanionProcessStatusPluginCompanion ProcessConvert to MAVLink HEARTBEATUpdate process statusPublish CompanionProcessStatus messageSend HEARTBEAT messageTransmit MAVLink message
```

Sources: [mavros\_extras/src/plugins/companion\_process\_status.cpp45-87](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/companion_process_status.cpp#L45-L87)

### ROS Message Interface

To report status, companion processes publish messages of type `CompanionProcessStatus` to the `/mavros/companion_process/status` topic.

The message structure includes:

-   `header`: Standard ROS message header
-   `state`: The state of the companion process (using MAV\_STATE enum values)
-   `component`: The MAVLink component ID of the process

#### CompanionProcessStatus Message Constants

| State Constant | Value | Description |
| --- | --- | --- |
| MAV\_STATE\_UNINIT | 0 | Component is uninitialized |
| MAV\_STATE\_BOOT | 1 | Component is booting up |
| MAV\_STATE\_CALIBRATING | 2 | Component is calibrating |
| MAV\_STATE\_STANDBY | 3 | Component is ready but inactive |
| MAV\_STATE\_ACTIVE | 4 | Component is active |
| MAV\_STATE\_CRITICAL | 5 | Component has experienced a critical error |
| MAV\_STATE\_EMERGENCY | 6 | Component is in emergency mode |
| MAV\_STATE\_POWEROFF | 7 | Component is shutting down |
| MAV\_STATE\_FLIGHT\_TERMINATION | 8 | Component has terminated |

| Component ID Constant | Value | Description |
| --- | --- | --- |
| MAV\_COMP\_ID\_OBSTACLE\_AVOIDANCE | 196 | Obstacle avoidance component |
| MAV\_COMP\_ID\_VISUAL\_INERTIAL\_ODOMETRY | 197 | Visual inertial odometry component |

Sources: [mavros\_msgs/msg/CompanionProcessStatus.msg1-20](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/CompanionProcessStatus.msg#L1-L20)

### Example Usage

To report the status of an obstacle avoidance process:

```
# ROS Python example
from mavros_msgs.msg import CompanionProcessStatus
from std_msgs.msg import Header

# Create publisher
status_pub = rospy.Publisher('/mavros/companion_process/status', CompanionProcessStatus, queue_size=10)

# Create and publish status message
status_msg = CompanionProcessStatus()
status_msg.header = Header()
status_msg.header.stamp = rospy.Time.now()
status_msg.state = CompanionProcessStatus.MAV_STATE_ACTIVE  # Process is active
status_msg.component = CompanionProcessStatus.MAV_COMP_ID_OBSTACLE_AVOIDANCE  # Obstacle avoidance component

status_pub.publish(status_msg)
```

## Common Plugins for Companion Computers

MAVROS provides several plugins that are particularly useful for companion computers. These plugins allow companion computers to send various types of data to the flight controller.

### Vision Pose Estimate Plugin

The Vision Pose Estimate plugin allows sending position and orientation estimates from external vision systems to the flight controller's estimator.

```
Flight ControllerCompanion ComputerPoseStamped orPoseWithCovarianceStampedVISION_POSITION_ESTIMATEMAVLink messageVision Process(SLAM, Visual Odometry)VisionPoseEstimatePluginPosition Estimator
```

#### Key Features:

-   Accepts pose data as `geometry_msgs/PoseStamped` or `geometry_msgs/PoseWithCovarianceStamped`
-   Transforms ENU frame data to NED frame used by MAVLink
-   Can listen for TF transforms instead of direct pose messages
-   Publishes to topics:
    -   `/mavros/vision_pose/pose`
    -   `/mavros/vision_pose/pose_cov`

Sources: [mavros\_extras/src/plugins/vision\_pose\_estimate.cpp1-191](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_pose_estimate.cpp#L1-L191)

### Motion Capture Pose Estimate Plugin

The MoCap Pose Estimate plugin allows sending motion capture data to the flight controller.

```
Flight ControllerCompanion ComputerPoseStamped orTransformStampedATT_POS_MOCAPMAVLink messageMotion Capture SystemMocapPoseEstimatePluginPosition Estimator
```

#### Key Features:

-   Accepts data as `geometry_msgs/PoseStamped` or `geometry_msgs/TransformStamped`
-   Compatible with Vicon and OptiTrack motion capture systems
-   Transforms data from ROS conventions to MAVLink conventions
-   Publishes to topics:
    -   `/mavros/mocap/pose`
    -   `/mavros/mocap/tf`

Sources: [mavros\_extras/src/plugins/mocap\_pose\_estimate.cpp1-129](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mocap_pose_estimate.cpp#L1-L129)

### Vision Speed Estimate Plugin

The Vision Speed Estimate plugin allows sending velocity estimates from vision systems to the flight controller.

```
Flight ControllerCompanion ComputerTwistStamped,TwistWithCovarianceStamped,or Vector3StampedVISION_SPEED_ESTIMATEMAVLink messageVision SystemVisionSpeedEstimatePluginVelocity Estimator
```

#### Key Features:

-   Accepts velocity data in multiple formats
-   Transforms ENU frame data to NED frame
-   Can include covariance information
-   Publishes to topics:
    -   `/mavros/vision_speed/speed_twist`
    -   `/mavros/vision_speed/speed_twist_cov`
    -   `/mavros/vision_speed/speed_vector`

Sources: [mavros\_extras/src/plugins/vision\_speed\_estimate.cpp1-164](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_speed_estimate.cpp#L1-L164)

### PX4Flow Plugin

The PX4Flow plugin interfaces with the PX4Flow optical flow sensor, which can be connected to the companion computer.

```
Flight ControllerCompanion ComputerOPTICAL_FLOW_RADMAVLink messageOpticalFlowRadROS messageGround distanceand temperaturePX4Flow SensorPX4FlowPluginPosition Estimator
```

#### Key Features:

-   Receives optical flow data from PX4Flow sensor
-   Publishes optical flow, ground distance, and temperature data
-   Transforms coordinate frames between MAVLink and ROS conventions
-   Publishes to topics:
    -   `/mavros/px4flow/raw/optical_flow_rad`
    -   `/mavros/px4flow/ground_distance`
    -   `/mavros/px4flow/temperature`

Sources: [mavros\_extras/src/plugins/px4flow.cpp1-217](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/px4flow.cpp#L1-L217)

## Setting Up Companion Computer Integration

To integrate a companion computer with a flight controller using MAVROS, follow these general steps:

1.  **Install ROS and MAVROS on the companion computer**
2.  **Configure the communication link** between companion computer and flight controller (typically via UART, USB, or Ethernet)
3.  **Run MAVROS** with appropriate configuration for your setup
4.  **Launch your companion computer applications** that use MAVROS plugins

### Example Launch Configuration

A typical launch file for MAVROS on a companion computer might look like:

```
<launch>
    <!-- MAVROS node -->
    <node pkg="mavros" type="mavros_node" name="mavros" required="true" clear_params="true" output="screen">
        <!-- Connection URL -->
        <param name="fcu_url" value="/dev/ttyACM0:921600" />
        <param name="gcs_url" value="" />
        <param name="target_system_id" value="1" />
        <param name="target_component_id" value="1" />
        
        <!-- Load plugins -->
        <rosparam command="load" file="$(find mavros)/launch/px4_pluginlists.yaml" />
        <rosparam command="load" file="$(find mavros)/launch/px4_config.yaml" />
    </node>
</launch>
```

## Status Reporting Recommendations

When implementing status reporting for companion processes, consider the following recommendations:

1.  **Report status regularly**: Update the status at a reasonable frequency (1-5 Hz)
2.  **Handle error conditions**: Transition to appropriate states (CRITICAL, EMERGENCY) when errors occur
3.  **Report startup correctly**: Use BOOT during initialization, then STANDBY or ACTIVE once ready
4.  **Use correct component IDs**: Use appropriate MAV\_COMP\_ID values for your processes

```
Start initializationBegin calibrationCalibration completeProcess activatedProcess deactivatedError detectedUnrecoverable errorError resolvedShutdown requestedShutdown requestedEmergency action requiredUNINITBOOTCALIBRATINGSTANDBYACTIVECRITICALEMERGENCYPOWEROFFFLIGHT_TERMINATION
```

Sources: [mavros\_extras/src/plugins/companion\_process\_status.cpp45-87](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/companion_process_status.cpp#L45-L87) [mavros\_msgs/msg/CompanionProcessStatus.msg1-20](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/CompanionProcessStatus.msg#L1-L20)

## Conclusion

Companion computers extend the capabilities of flight controllers by providing additional computing power for complex tasks. MAVROS facilitates the integration of companion computers with flight controllers by providing a ROS interface to MAVLink.

The Companion Process Status plugin allows reporting the status of companion processes to the flight controller, which is crucial for ensuring safe operation. Additionally, plugins like Vision Pose Estimate, MoCap Pose Estimate, Vision Speed Estimate, and PX4Flow enable companion computers to send various types of data to the flight controller's estimators.

By properly integrating a companion computer with MAVROS and implementing appropriate status reporting, you can create advanced autonomous systems that leverage both the real-time control capabilities of flight controllers and the computational power of companion computers.