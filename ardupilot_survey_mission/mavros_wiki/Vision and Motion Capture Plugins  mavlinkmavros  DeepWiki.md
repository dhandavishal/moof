Relevant source files

-   [mavros/src/lib/ftf\_frame\_conversions.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp)
-   [mavros/src/lib/ftf\_quaternion\_utils.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_quaternion_utils.cpp)
-   [mavros\_extras/src/plugins/fake\_gps.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp)
-   [mavros\_extras/src/plugins/mocap\_pose\_estimate.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mocap_pose_estimate.cpp)
-   [mavros\_extras/src/plugins/px4flow.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/px4flow.cpp)
-   [mavros\_extras/src/plugins/tunnel.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/tunnel.cpp)
-   [mavros\_extras/src/plugins/vibration.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vibration.cpp)
-   [mavros\_extras/src/plugins/vision\_pose\_estimate.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_pose_estimate.cpp)
-   [mavros\_extras/src/plugins/vision\_speed\_estimate.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_speed_estimate.cpp)

This page documents the MAVROS plugins that handle vision-based position/velocity estimation and motion capture systems integration. These plugins enable external localization sources to provide pose and velocity estimates to the flight controller unit (FCU). For information about other sensor plugins, see [Sensor Plugins](https://deepwiki.com/mavlink/mavros/3.2.1-sensor-plugins).

## Plugin Overview

MAVROS provides several plugins to integrate external localization systems with flight controllers, enabling accurate position and velocity estimation in GPS-denied environments. These plugins transform data between ROS coordinate frames (typically ENU - East, North, Up) and MAVLink coordinate frames (typically NED - North, East, Down).

```
Flight ControllerMAVROS PluginsROS EcosystemPoseStampedPoseWithCovarianceStampedTF TransformPoseStampedTransformStampedTwistStampedTwistWithCovarianceStampedVector3StampedPoseStampedTransformStampedPoseWithCovarianceStampedOPTICAL_FLOW_RADVISION_POSITION_ESTIMATEATT_POS_MOCAPVISION_SPEED_ESTIMATEHIL_GPS or GPS_INPUTProcessed optical flowVision SystemsMotion Capture SystemsOptical Flow CameraLocal Position SourcesVisionPoseEstimatePluginvision_poseMocapPoseEstimatePluginmocapVisionSpeedEstimatePluginvision_speedFakeGPSPluginfake_gpsPX4FlowPluginpx4flowExtended Kalman FilterPosition EstimatorVelocity Estimator
```

Sources: [mavros\_extras/src/plugins/vision\_pose\_estimate.cpp37-49](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_pose_estimate.cpp#L37-L49) [mavros\_extras/src/plugins/mocap\_pose\_estimate.cpp33-39](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mocap_pose_estimate.cpp#L33-L39) [mavros\_extras/src/plugins/vision\_speed\_estimate.cpp33-40](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_speed_estimate.cpp#L33-L40) [mavros\_extras/src/plugins/fake\_gps.cpp52-61](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L52-L61) [mavros\_extras/src/plugins/px4flow.cpp37-42](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/px4flow.cpp#L37-L42)

## Frame Transformation System

Vision and motion capture plugins rely heavily on coordinate frame transformations to convert between different reference frames. MAVROS provides utilities to perform these transformations, which are crucial for correctly integrating external localization data with the flight controller.

```
MAVLink Standard FramesROS Standard Framestransform_frame_enu_ned()transform_orientation_baselink_aircraft()transform_frame_enu_ecef()GeographicLib conversionsFrame Conversion Functionstransform_frame_enu_ned()transform_frame_ned_enu()transform_orientation_baselink_aircraft()transform_orientation_aircraft_baselink()transform_frame_enu_ecef()ENU FrameEast-North-UpBaselink FrameForward-Left-UpTF2 TransformsNED FrameNorth-East-DownAircraft FrameForward-Right-DownECEF FrameEarth-Centered Earth-FixedLLALatitude-Longitude-Altitude
```

Sources: [mavros/src/lib/ftf\_frame\_conversions.cpp26-315](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L26-L315) [mavros/src/lib/ftf\_quaternion\_utils.cpp23-57](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_quaternion_utils.cpp#L23-L57)

## Vision Pose Estimate Plugin

The Vision Pose Estimate plugin (`vision_pose`) sends position and attitude estimates from vision systems to the flight controller. It transforms the pose data from the ROS ENU frame to the MAVLink NED frame.

### Interfaces

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/pose` | `geometry_msgs/PoseStamped` | Vision position and orientation estimate |
| `~/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | Vision position and orientation with covariance estimate |

### Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `tf/listen` | `false` | Whether to listen for TF transforms instead of ROS topics |
| `tf/frame_id` | `map` | TF frame id for vision source |
| `tf/child_frame_id` | `vision_estimate` | TF child frame id for vision estimate |
| `tf/rate_limit` | `10.0` | Rate limit for TF transform publishing (Hz) |

### Flow of Data

```
Flight ControllerVisionPoseEstimatePluginVision SystemFlight ControllerVisionPoseEstimatePluginVision SystemUsing ROS messagesUsing TF transformsUsing PoseWithCovariancegeometry_msgs/PoseStamped (ENU)Convert to NED frameVISION_POSITION_ESTIMATE MAVLink messageTF transform (ENU)Convert to NED frameVISION_POSITION_ESTIMATE MAVLink messagegeometry_msgs/PoseWithCovarianceStamped (ENU)Convert to NED frame with covarianceVISION_POSITION_ESTIMATE MAVLink message with covariance data
```

Sources: [mavros\_extras/src/plugins/vision\_pose\_estimate.cpp47-186](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_pose_estimate.cpp#L47-L186)

## Motion Capture Pose Estimate Plugin

The Motion Capture Pose Estimate plugin (`mocap`) sends position and attitude data from motion capture systems to the flight controller. It supports both VICON and OptiTrack motion capture systems.

### Interfaces

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/tf` | `geometry_msgs/TransformStamped` | VICON motion capture pose as transform (preferred for VICON) |
| `~/pose` | `geometry_msgs/PoseStamped` | Motion capture pose (preferred for OptiTrack) |

### Flow of Data

The plugin transforms the incoming pose from ENU to NED frame and forwards it to the flight controller using the `ATT_POS_MOCAP` MAVLink message.

Sources: [mavros\_extras/src/plugins/mocap\_pose\_estimate.cpp39-124](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mocap_pose_estimate.cpp#L39-L124)

## Vision Speed Estimate Plugin

The Vision Speed Estimate plugin (`vision_speed`) sends velocity estimations from vision systems to the flight controller. It supports various input formats and transforms the velocity data to the correct reference frame.

### Interfaces

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/speed_twist` | `geometry_msgs/TwistStamped` | Vision velocity estimate |
| `~/speed_twist_cov` | `geometry_msgs/TwistWithCovarianceStamped` | Vision velocity with covariance |
| `~/speed_vector` | `geometry_msgs/Vector3Stamped` | Vision velocity as a 3D vector |

### Flow of Data

The plugin transforms the incoming velocity data from ENU to NED frame and forwards it to the flight controller using the `VISION_SPEED_ESTIMATE` MAVLink message.

Sources: [mavros\_extras/src/plugins/vision\_speed\_estimate.cpp40-159](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/vision_speed_estimate.cpp#L40-L159)

## Fake GPS Plugin

The Fake GPS plugin (`fake_gps`) creates simulated GPS data based on local position information from motion capture or vision systems. It converts local position to GPS coordinates using geodetic transformations.

### Interfaces

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/mocap/pose` | `geometry_msgs/PoseStamped` | Motion capture pose |
| `~/mocap/tf` | `geometry_msgs/TransformStamped` | Motion capture pose as transform |
| `~/mocap/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | Motion capture pose with covariance |
| `~/vision` | `geometry_msgs/PoseStamped` | Vision system pose |

### Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `use_mocap` | `true` | Use motion capture source |
| `mocap_transform` | `true` | Use TransformStamped for motion capture if true; PoseStamped if false |
| `mocap_withcovariance` | `false` | Whether motion capture provides covariance data |
| `use_vision` | `false` | Use vision source instead of motion capture |
| `use_hil_gps` | `true` | Send HIL\_GPS MAVLink messages if true, GPS\_INPUT if false |
| `geo_origin.lat` | `47.3667` | Origin latitude for local to global conversion |
| `geo_origin.lon` | `8.5500` | Origin longitude for local to global conversion |
| `geo_origin.alt` | `408.0` | Origin altitude for local to global conversion |
| `tf/listen` | `false` | Whether to listen for TF transforms |
| `tf/frame_id` | `map` | TF frame id for position source |
| `tf/child_frame_id` | `base_link` | TF child frame id for position estimate |
| `gps_rate` | `5.0` | Rate at which to send GPS messages (Hz) |
| `eph` | `2.0` | GPS horizontal position uncertainty (m) |
| `epv` | `2.0` | GPS vertical position uncertainty (m) |

### Workflow

```
Output OptionsFakeGPSPlugin ProcessingInput SourcesPoseStampedTransformStampedPoseWithCovarianceStampedPoseStampedTransformuse_hil_gps=trueuse_hil_gps=falseMotion CaptureVision SystemTF TransformRead Position DataTransform to ECEF FrameConvert to Geodetic CoordinatesLat/Lon/AltCalculate VelocityHIL_GPS MessageGPS_INPUT MessageFlight Controller
```

Sources: [mavros\_extras/src/plugins/fake\_gps.cpp61-448](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L61-L448)

## PX4Flow Plugin

The PX4Flow plugin (`px4flow`) handles data from the PX4Flow optical flow sensor. It processes incoming MAVLink `OPTICAL_FLOW_RAD` messages and publishes the data to ROS topics.

### Interfaces

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/raw/optical_flow_rad` | `mavros_msgs/OpticalFlowRad` | Raw optical flow data |
| `~/ground_distance` | `sensor_msgs/Range` | Distance to ground from flow sensor |
| `~/temperature` | `sensor_msgs/Temperature` | Temperature data from flow sensor |
| `~/raw/send` | `mavros_msgs/OpticalFlowRad` | Topic to send raw optical flow data to FCU |

### Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `frame_id` | `px4flow` | Frame ID for messages |
| `ranger_fov` | `0.118` | Field of view of rangefinder (rad) |
| `ranger_min_range` | `0.3` | Minimum range (m) |
| `ranger_max_range` | `5.0` | Maximum range (m) |

### Data Flow

```
ROS ApplicationPX4FlowPluginFlight ControllerROS ApplicationPX4FlowPluginFlight ControllerOPTICAL_FLOW_RAD MAVLink messageTransform coordinate framesPublish OpticalFlowRad messagePublish Range message (ground distance)Publish Temperature messageSend OpticalFlowRad messageTransform coordinate framesOPTICAL_FLOW_RAD MAVLink message
```

Sources: [mavros\_extras/src/plugins/px4flow.cpp42-212](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/px4flow.cpp#L42-L212)

## Installation and Configuration

All vision and motion capture plugins are part of the `mavros_extras` package. They need to be explicitly loaded in the node configuration.

### Plugin Loading

To load these plugins, add them to the `plugin_allowlist` parameter in your MAVROS node configuration:

```
mavros:
  ros__parameters:
    plugin_allowlist:
      - vision_pose
      - vision_speed
      - mocap
      - fake_gps
      - px4flow
```

### Flight Controller Configuration

For PX4 Firmware, you'll need to set parameters to accept external position data:

| Parameter | Value | Description |
| --- | --- | --- |
| `EKF2_AID_MASK` | Include bit 3 (vision position fusion) | Enable vision position fusion |
| `EKF2_HGT_MODE` | `3` (Vision) | Use vision as height source |
| `MAV_USEHILGPS` | `1` | Use HIL GPS message (for fake\_gps plugin) |

For ArduPilot, configure:

| Parameter | Value | Description |
| --- | --- | --- |
| `EK3_SRC1_POSXY` | `6` (ExternalNav) | Use external navigation for position |
| `EK3_SRC1_VELXY` | `6` (ExternalNav) | Use external navigation for velocity |
| `EK3_SRC1_POSZ` | `6` (ExternalNav) | Use external navigation for altitude |

## Common Issues and Troubleshooting

1.  **Coordinate frame mismatches**: Ensure that your vision/mocap system uses the correct coordinate frame. MAVROS expects ENU frame data from ROS topics.
    
2.  **Timestamp synchronization**: For proper fusion, timestamps between ROS and FCU need to be synchronized. Use the `/mavros/time_sync_status` topic to check synchronization.
    
3.  **Transform rates**: Set appropriate transform rates to avoid overwhelming the flight controller with too many updates.
    
4.  **Position jumps**: Large jumps in position estimates can destabilize the EKF. Make sure your vision/mocap system is stable and properly calibrated.
    
5.  **Covariance handling**: When sending data with covariance, ensure the covariance values are properly scaled and meaningful for the flight controller.