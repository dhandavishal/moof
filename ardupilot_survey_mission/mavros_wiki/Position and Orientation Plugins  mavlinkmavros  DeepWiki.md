Relevant source files

-   [mavros/src/plugins/global\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp)
-   [mavros/src/plugins/imu.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp)
-   [mavros/src/plugins/local\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp)

This page documents the MAVROS plugins responsible for handling position, orientation, and inertial measurement data from MAVLink-enabled flight controllers. These plugins convert various MAVLink messages into ROS topics that provide position, velocity, orientation, and inertial measurements in formats suitable for ROS applications.

For information about controlling the vehicle using setpoints, see [Setpoint Plugins](https://deepwiki.com/mavlink/mavros/3.1.3-setpoint-plugins).

## Overview

Position and orientation data is critical for autonomous vehicle operation. MAVROS provides three core plugins to handle different aspects of this data:

1.  **Global Position Plugin** - Handles GPS and global position data
2.  **Local Position Plugin** - Processes local position and velocity information
3.  **IMU Plugin** - Manages orientation, acceleration, and angular velocity data

Together, these plugins provide a complete picture of the vehicle's state in space, performing necessary coordinate transformations and publishing data to standardized ROS topics.

```
ROS TopicsMAVROS PluginsMAVLink MessagesGLOBAL_POSITION_INTGPS_RAW_INTLOCAL_POSITION_NEDLOCAL_POSITION_NED_COVATTITUDEATTITUDE_QUATERNIONRAW_IMUSCALED_IMUHIGHRES_IMUGlobalPositionPluginLocalPositionPluginIMUPlugin/mavros/global_position/global/mavros/global_position/local/mavros/local_position/pose/mavros/local_position/velocity_local/mavros/imu/data/mavros/imu/data_raw/mavros/imu/mag
```

Sources: [mavros/src/plugins/global\_position.cpp1-561](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L1-L561) [mavros/src/plugins/local\_position.cpp1-298](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L1-L298) [mavros/src/plugins/imu.cpp1-647](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L1-L647)

## Coordinate Frame Transformations

A key function of these plugins is transforming data between different coordinate frames. MAVROS handles these common coordinate systems:

-   **NED** (North-East-Down) - Used by MAVLink/flight controllers
-   **ENU** (East-North-Up) - ROS standard frame
-   **ECEF** (Earth-Centered Earth-Fixed) - Global reference frame
-   **LLA** (Latitude-Longitude-Altitude) - Geographic coordinates
-   **Aircraft Frame** (Forward-Right-Down) - Aircraft body frame
-   **Base Link Frame** (Forward-Left-Up) - ROS standard body frame

The plugins automatically perform the necessary transformations to ensure consistent coordinate representations.

```
Global FramesROS FramesMAVLink Framestransform_frame_ned_enu()transform_frame_aircraft_baselink()transform_frame_enu_ecef()GeographicLibNED(North-East-Down)Aircraft(Forward-Right-Down)ENU(East-North-Up)Base Link(Forward-Left-Up)LLA(Lat-Long-Alt)ECEF(Earth-Centered)
```

Sources: [mavros/src/plugins/global\_position.cpp354-396](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L354-L396) [mavros/src/plugins/local\_position.cpp145-161](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L145-L161) [mavros/src/plugins/imu.cpp344-355](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L344-L355)

## Global Position Plugin

The Global Position Plugin (`GlobalPositionPlugin`) handles global positioning data from the flight controller, including GPS coordinates, altitude, and global velocity.

### Handled MAVLink Messages

| Message | Description |
| --- | --- |
| `GPS_RAW_INT` | Raw GPS data (coordinates, altitude, velocity, status) |
| `GLOBAL_POSITION_INT` | Fused global position with altitude and velocity |
| `GPS_GLOBAL_ORIGIN` | Global origin for local coordinate system |
| `LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET` | Offset between local and global frames |

### Published ROS Topics

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/raw/fix` | `sensor_msgs/NavSatFix` | Raw GPS data |
| `~/raw/gps_vel` | `geometry_msgs/TwistStamped` | Raw GPS velocity |
| `~/raw/satellites` | `std_msgs/UInt32` | Satellite count |
| `~/global` | `sensor_msgs/NavSatFix` | Fused global position |
| `~/local` | `nav_msgs/Odometry` | Global position as local odometry |
| `~/rel_alt` | `std_msgs/Float64` | Relative altitude |
| `~/compass_hdg` | `std_msgs/Float64` | Compass heading |
| `~/gp_origin` | `geographic_msgs/GeoPointStamped` | Global origin for local coordinates |
| `~/gp_lp_offset` | `geometry_msgs/PoseStamped` | Offset between local and global frames |

### Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `frame_id` | `"map"` | Frame ID for topic headers |
| `child_frame_id` | `"base_link"` | Child frame ID for topic headers |
| `rot_covariance` | `99999.0` | Rotation covariance |
| `gps_uere` | `1.0` | GPS user equivalent range error |
| `use_relative_alt` | `true` | Use relative altitude instead of WGS-84 altitude |
| `tf.send` | `false` | Send transform from map to base\_link |
| `tf.frame_id` | `"map"` | Frame ID for TF |
| `tf.global_frame_id` | `"earth"` | Global frame ID for TF (ECEF) |
| `tf.child_frame_id` | `"base_link"` | Child frame ID for TF |

The plugin performs coordinate transformations from GPS coordinates (Lat/Long/Alt) to ECEF (Earth-Centered, Earth-Fixed) and then to a local ENU (East-North-Up) frame, which is standard for ROS.

```
ROS TopicsGlobalPositionPluginMAVLink MessagesGLOBAL_POSITION_INTGPS_RAW_INTGPS_GLOBAL_ORIGINhandle_global_position_int()handle_gps_raw_int()handle_gps_global_origin()LLA to ECEF conversionECEF to ENU conversionNavSatFix~/globalOdometry~/localFloat64~/rel_altFloat64~/compass_hdg
```

Sources: [mavros/src/plugins/global\_position.cpp54-555](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L54-L555)

## Local Position Plugin

The Local Position Plugin (`LocalPositionPlugin`) processes local position data from the flight controller, including pose, velocity, and acceleration in the local coordinate frame.

### Handled MAVLink Messages

| Message | Description |
| --- | --- |
| `LOCAL_POSITION_NED` | Local position and velocity in NED frame |
| `LOCAL_POSITION_NED_COV` | Local position, velocity, and acceleration with covariance |

### Published ROS Topics

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/pose` | `geometry_msgs/PoseStamped` | Local position |
| `~/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | Local position with covariance |
| `~/velocity_local` | `geometry_msgs/TwistStamped` | Velocity in local frame |
| `~/velocity_body` | `geometry_msgs/TwistStamped` | Velocity in body frame |
| `~/velocity_body_cov` | `geometry_msgs/TwistWithCovarianceStamped` | Velocity with covariance |
| `~/accel` | `geometry_msgs/AccelWithCovarianceStamped` | Local acceleration |
| `~/odom` | `nav_msgs/Odometry` | Complete odometry |

### Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `frame_id` | `"map"` | Frame ID for topics (world-fixed) |
| `tf.send` | `false` | Send transform from map to base\_link |
| `tf.frame_id` | `"map"` | Frame ID for TF |
| `tf.child_frame_id` | `"base_link"` | Child frame ID for TF |

The plugin transforms local position data from the NED (North-East-Down) frame used by the flight controller to the ENU (East-North-Up) frame used by ROS.

```
ROS TopicsLocalPositionPluginMAVLink MessagesLOCAL_POSITION_NEDLOCAL_POSITION_NED_COVhandle_local_position_ned()handle_local_position_ned_cov()NED to ENU transformPublish to ROS topicsPoseStamped~/posePoseWithCovarianceStamped~/pose_covTwistStamped~/velocity_localTwistStamped~/velocity_bodyOdometry~/odom
```

Sources: [mavros/src/plugins/local\_position.cpp48-292](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L48-L292)

## IMU Plugin

The IMU Plugin (`IMUPlugin`) processes inertial measurement data from the flight controller, including orientation, angular velocity, linear acceleration, and magnetic field measurements.

### Handled MAVLink Messages

| Message | Description |
| --- | --- |
| `ATTITUDE` | Roll/pitch/yaw attitude |
| `ATTITUDE_QUATERNION` | Quaternion representation of attitude |
| `HIGHRES_IMU` | High-resolution IMU data |
| `RAW_IMU` | Raw IMU sensor values |
| `SCALED_IMU` | Scaled IMU sensor values |
| `SCALED_PRESSURE` | Pressure sensor readings |

### Published ROS Topics

| Topic | Message Type | Description |
| --- | --- | --- |
| `~/data` | `sensor_msgs/Imu` | Processed IMU data (orientation, velocity, acceleration) |
| `~/data_raw` | `sensor_msgs/Imu` | Raw IMU data (without orientation) |
| `~/mag` | `sensor_msgs/MagneticField` | Magnetic field data |
| `~/temperature_imu` | `sensor_msgs/Temperature` | IMU temperature |
| `~/temperature_baro` | `sensor_msgs/Temperature` | Barometer temperature |
| `~/static_pressure` | `sensor_msgs/FluidPressure` | Static pressure measurement |
| `~/diff_pressure` | `sensor_msgs/FluidPressure` | Differential pressure measurement |

### Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `frame_id` | `"base_link"` | Frame ID for IMU (body frame) |
| `linear_acceleration_stdev` | `0.0003` | Standard deviation for linear acceleration |
| `angular_velocity_stdev` | `0.02 * (PI/180.0)` | Standard deviation for angular velocity |
| `orientation_stdev` | `1.0` | Standard deviation for orientation |
| `magnetic_stdev` | `0.0` | Standard deviation for magnetic field |

The IMU plugin handles multiple coordinate transformations:

1.  Transforms attitude from NED frame to ENU frame
2.  Transforms angular velocities from aircraft frame to base\_link frame
3.  Transforms accelerations from aircraft frame to base\_link frame

```
ROS TopicsIMUPluginMAVLink MessagesATTITUDEATTITUDE_QUATERNIONHIGHRES_IMURAW_IMUSCALED_IMUSCALED_PRESSUREhandle_attitude()handle_attitude_quaternion()handle_highres_imu()handle_raw_imu()handle_scaled_imu()handle_scaled_pressure()transform_orientation_ned_enu()transform_frame_aircraft_baselink()Imu~/dataImu~/data_rawMagneticField~/magTemperature~/temperature_imuFluidPressure~/static_pressure
```

Sources: [mavros/src/plugins/imu.cpp58-641](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L58-L641)

## Integration Example

These plugins work together to provide a complete picture of the vehicle's state:

```
ApplicationsVehicle StatePosition & Orientation PluginsGlobalPositionPluginLocalPositionPluginIMUPluginGlobal Position(Lat/Long/Alt)Local Position(x/y/z)Orientation(Roll/Pitch/Yaw)Velocity(Linear/Angular)AccelerationMagnetic FieldNavigationControlVisualizationLocalization
```

Sources: [mavros/src/plugins/global\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp) [mavros/src/plugins/local\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp) [mavros/src/plugins/imu.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp)

## Practical Considerations

When using these plugins, consider the following:

1.  **Coordinate Frames**: Understand the difference between NED (flight controller) and ENU (ROS) frames.
2.  **TF Integration**: Enable the `tf.send` parameter to publish transforms to the TF tree.
3.  **Data Priority**: The plugins prioritize higher-quality data sources:
    -   `HIGHRES_IMU` > `SCALED_IMU` > `RAW_IMU`
    -   `ATTITUDE_QUATERNION` > `ATTITUDE`
    -   `LOCAL_POSITION_NED_COV` > `LOCAL_POSITION_NED`
4.  **Covariance**: Covariance settings affect fusion algorithms and state estimation.
5.  **Frame IDs**: Ensure consistent frame\_id values across all plugins.

Sources: [mavros/src/plugins/global\_position.cpp59-107](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L59-L107) [mavros/src/plugins/local\_position.cpp51-79](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L51-L79) [mavros/src/plugins/imu.cpp63-107](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L63-L107)