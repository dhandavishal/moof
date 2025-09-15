Relevant source files

-   [mavros/src/lib/ftf\_frame\_conversions.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp)
-   [mavros/src/lib/ftf\_quaternion\_utils.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_quaternion_utils.cpp)
-   [mavros/src/plugins/global\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp)
-   [mavros/src/plugins/imu.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp)
-   [mavros/src/plugins/local\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp)
-   [mavros\_extras/src/plugins/fake\_gps.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp)
-   [mavros\_extras/src/plugins/tunnel.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/tunnel.cpp)

This page documents the coordinate frame transformation system in MAVROS. It explains how MAVROS converts between different coordinate frames used by MAVLink flight controllers and ROS, including both orientation and position transformations. For information about configuring transformations in specific plugins, see [Plugin System](https://deepwiki.com/mavlink/mavros/3-plugin-system).

## 1\. Overview of Coordinate Frames

MAVROS must bridge between different coordinate systems used by flight controllers (typically NED-based) and ROS (typically ENU-based). This section describes the key coordinate frames and their relationships.

### 1.1 Primary Coordinate Frames

| Frame | Description | Origin | Primary Use |
| --- | --- | --- | --- |
| **NED** | North-East-Down | Local origin | MAVLink/Flight controllers |
| **ENU** | East-North-Up | Local origin | ROS standard (REP-105) |
| **ECEF** | Earth-Centered Earth-Fixed | Earth's center | Global coordinate transformations |
| **LLA** | Latitude-Longitude-Altitude | \- | Geographic coordinates |
| **Aircraft** | Forward-Right-Down | Vehicle center | Flight controller body frame |
| **Baselink** | Forward-Left-Up | Vehicle center | ROS standard body frame |

```
Global FramesLocal Framestransform_frame_ned_enu()transform_frame_enu_ecef()GeographicLibBody Framestransform_frame_aircraft_baselink()Aircraft Frame(Forward-Right-Down)Baselink Frame(Forward-Left-Up)NED(North-East-Down)ENU(East-North-Up)ECEF(Earth-Centered Earth-Fixed)LLA(Latitude-Longitude-Altitude)
```

Sources: [mavros/src/lib/ftf\_frame\_conversions.cpp26-118](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L26-L118) [mavros/src/plugins/global\_position.cpp355-396](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L355-L396)

### 1.2 Frame Orientations

```
Baselink FrameX (Forward)Y (Left)Z (Up)Aircraft FrameX (Forward)Y (Right)Z (Down)ENU FrameX (East)Y (North)Z (Up)NED FrameX (North)Y (East)Z (Down)
```

Sources: [mavros/src/lib/ftf\_frame\_conversions.cpp19-118](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L19-L118) [mavros/src/plugins/imu.cpp77-86](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L77-L86)

## 2\. Frame Transformation Functions

MAVROS provides several transformation functions in its frame transformation framework (`ftf` namespace).

### 2.1 Static Frame Transformations

These transformations apply fixed rotations between frames with aligned origins.

```
Internal ImplementationTransformation Functionstransform_frame_ned_enu()transform_frame_aircraft_baselink()NED_ENU_Q - Static quaternion+PI/2 around Z, +PI around XAIRCRAFT_BASELINK_Q+PI rotation around X
```

Sources: [mavros/src/lib/ftf\_frame\_conversions.cpp26-118](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L26-L118)

#### Position/Vector Transformations

```
// NED to ENU conversion
Eigen::Vector3d enu_position = ftf::transform_frame_ned_enu(ned_position);

// Aircraft to Baselink conversion
Eigen::Vector3d baselink_vector = ftf::transform_frame_aircraft_baselink(aircraft_vector);
```

Sources: [mavros/src/plugins/local\_position.cpp146-151](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L146-L151) [mavros/src/plugins/imu.cpp434-438](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L434-L438)

#### Orientation Transformations

```
// NED to ENU orientation conversion
Eigen::Quaterniond enu_orientation = ftf::transform_orientation_ned_enu(ned_orientation);

// Aircraft to Baselink orientation conversion
Eigen::Quaterniond baselink_orientation = ftf::transform_orientation_aircraft_baselink(aircraft_orientation);
```

Sources: [mavros/src/plugins/imu.cpp344-347](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L344-L347) [mavros/src/plugins/imu.cpp396-397](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L396-L397)

### 2.2 Geographic Frame Transformations

MAVROS uses GeographicLib for transformations involving geographic coordinates.

```
GeographicLib ImplementationMAVROS Geographic TransformationsUses map originUses map origintransform_frame_enu_ecef()GeographicLib conversionsearth.Forward()LLA → ECEFearth.Reverse()ECEF → LLA
```

Sources: [mavros/src/plugins/global\_position.cpp355-396](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L355-L396) [mavros\_extras/src/plugins/fake\_gps.cpp147-156](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L147-L156) [mavros\_extras/src/plugins/fake\_gps.cpp301-307](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L301-L307)

#### ENU to ECEF Transformation

```
// Initialize GeographicLib converter
GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                               GeographicLib::Constants::WGS84_f());

// Convert from geodetic origin to ECEF origin
earth.Forward(
    map_origin.x(), map_origin.y(), map_origin.z(),
    ecef_origin.x(), ecef_origin.y(), ecef_origin.z());

// Transform local ENU coordinate to ECEF
Eigen::Vector3d ecef_point = ftf::transform_frame_enu_ecef(enu_point, map_origin);
```

Sources: [mavros/src/plugins/global\_position.cpp362-384](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L362-L384) [mavros\_extras/src/plugins/fake\_gps.cpp147-156](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L147-L156)

## 3\. MAVROS Plugin Implementation

Several MAVROS plugins implement frame transformations to convert between MAVLink and ROS data representations.

### 3.1 Global Position Plugin

This plugin converts global position data between different coordinate frames.

```
TransformationsROS TopicsMAVLink Messageshandle_gps_raw_int()handle_global_position_int()fill_lla()handle_gps_global_origin()GPS_RAW_INTGLOBAL_POSITION_INTGPS_GLOBAL_ORIGIN~/global (NavSatFix)~/local (Odometry)~/gp_origin (GeoPointStamped)LLA → ECEFGeographicLib::GeocentricECEF → ENUtransform_frame_ecef_enu()
```

Sources: [mavros/src/plugins/global\_position.cpp279-293](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L279-L293) [mavros/src/plugins/global\_position.cpp297-442](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L297-L442)

#### Key Global Position Transformations

1.  **LLA to ENU Conversion**:
    
    -   Converts GPS coordinates to local ENU frame using ECEF as intermediate frame
    -   Uses map origin (either from GPS\_GLOBAL\_ORIGIN message or first valid GPS fix)
2.  **Velocity Transformations**:
    
    -   Converts NED velocities from MAVLink to ENU for ROS
3.  **TF Publishing**:
    
    -   Optionally publishes transform between map frame and body frame

Sources: [mavros/src/plugins/global\_position.cpp362-396](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L362-L396)

### 3.2 Local Position Plugin

This plugin handles local position data from the flight controller and transforms it to ROS standard frames.

```
TransformationsROS TopicsMAVLink Messageshandle_local_position_ned()handle_local_position_ned_cov()LOCAL_POSITION_NEDLOCAL_POSITION_NED_COV~/pose (PoseStamped)~/odom (Odometry)~/velocity_local (TwistStamped)~/velocity_body (TwistStamped)NED → ENUtransform_frame_ned_enu()ENU → Baselinktransform_frame_enu_baselink()
```

Sources: [mavros/src/plugins/local\_position.cpp138-203](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L138-L203) [mavros/src/plugins/local\_position.cpp205-291](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L205-L291)

#### Key Local Position Transformations

1.  **NED to ENU Position Conversion**:
    
    ```
    auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.x, pos_ned.y, pos_ned.z));
    ```
    
2.  **NED to ENU Velocity Conversion**:
    
    ```
    auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(pos_ned.vx, pos_ned.vy, pos_ned.vz));
    ```
    
3.  **ENU to Body-Fixed Frame Conversion**:
    
    ```
    auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());
    ```
    

Sources: [mavros/src/plugins/local\_position.cpp146-159](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L146-L159)

### 3.3 IMU Plugin

This plugin handles IMU data transformations between aircraft frame and ROS standard frames.

```
TransformationsROS TopicsMAVLink Messageshandle_attitude()handle_attitude_quaternion()handle_highres_imu()handle_raw_imu()handle_scaled_imu()ATTITUDEATTITUDE_QUATERNIONHIGHRES_IMURAW_IMUSCALED_IMU~/data (Imu)~/data_raw (Imu)~/mag (MagneticField)NED → ENU orientationtransform_orientation_ned_enu()Aircraft → Baselink orientationtransform_orientation_aircraft_baselink()Aircraft → Baselink vectortransform_frame_aircraft_baselink()
```

Sources: [mavros/src/plugins/imu.cpp310-360](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L310-L360) [mavros/src/plugins/imu.cpp368-408](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L368-L408) [mavros/src/plugins/imu.cpp416-500](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L416-L500)

#### Key IMU Transformations

1.  **Aircraft to Baselink Frame Rotation**:
    
    ```
    auto gyro_flu = ftf::transform_frame_aircraft_baselink(gyro_frd);
    auto accel_flu = ftf::transform_frame_aircraft_baselink(accel_frd);
    ```
    
2.  **NED to ENU Orientation**:
    
    ```
    auto enu_baselink_orientation = ftf::transform_orientation_aircraft_baselink(
      ftf::transform_orientation_ned_enu(ned_aircraft_orientation));
    ```
    

Sources: [mavros/src/plugins/imu.cpp344-347](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L344-L347) [mavros/src/plugins/imu.cpp354-355](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/imu.cpp#L354-L355)

### 3.4 Fake GPS Plugin

The Fake GPS plugin uses frame transformations to convert local position data into GPS coordinates.

```
TransformationsMAVLink Output MessagesROS Input Topicsmocap_tf_cb()mocap_pose_cb()vision_cb()transform_cb()send_fake_gps()Unsupported markdown: del/mocap/tf~/visionTF LookupsHIL_GPSGPS_INPUTENU → ECEFtransform_frame_enu_ecef()ECEF → LLAearth.Reverse()
```

Sources: [mavros\_extras/src/plugins/fake\_gps.cpp286-398](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L286-L398) [mavros\_extras/src/plugins/fake\_gps.cpp402-447](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L402-L447)

#### Key Fake GPS Transformations

1.  **ENU to ECEF Transformation**:
    
    ```
    Eigen::Vector3d ecef_offset = ftf::transform_frame_enu_ecef(enu_position, map_origin);
    ```
    
2.  **ECEF to LLA (Geodetic) Transformation**:
    
    ```
    earth.Reverse(
      current_ecef.x(), current_ecef.y(), current_ecef.z(),
      geodetic.x(), geodetic.y(), geodetic.z());
    ```
    

Sources: [mavros\_extras/src/plugins/fake\_gps.cpp301-307](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L301-L307) [mavros\_extras/src/plugins/fake\_gps.cpp408-409](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L408-L409) [mavros\_extras/src/plugins/fake\_gps.cpp417-420](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L417-L420) [mavros\_extras/src/plugins/fake\_gps.cpp426-429](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L426-L429) [mavros\_extras/src/plugins/fake\_gps.cpp435-447](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/fake_gps.cpp#L435-L447)

## 4\. Implementation Details

### 4.1 Core Static Transformations

The core rotation matrices and quaternions for static transformations are defined in the `ftf` namespace:

```
Helper FunctionsTransformation TypesStatic QuaternionsNED_ENU_QRotation: +PI/2 around Z, +PI around XAIRCRAFT_BASELINK_QRotation: +PI around XStaticTF EnumNED_TO_ENU, ENU_TO_NEDAIRCRAFT_TO_BASELINK, BASELINK_TO_AIRCRAFTStaticEcefTF EnumECEF_TO_ENU, ENU_TO_ECEFtransform_orientation()transform_static_frame()transform_frame()
```

Sources: [mavros/src/lib/ftf\_frame\_conversions.cpp33-40](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L33-L40) [mavros/src/lib/ftf\_frame\_conversions.cpp78-118](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L78-L118) [mavros/src/lib/ftf\_frame\_conversions.cpp222-268](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L222-L268)

### 4.2 Quaternion Utilities

MAVROS provides utilities for working with quaternions and Euler angles:

```
// Convert Euler angles (roll, pitch, yaw) to quaternion
Eigen::Quaterniond q = ftf::quaternion_from_rpy(roll, pitch, yaw);

// Extract Euler angles from quaternion
Eigen::Vector3d rpy = ftf::quaternion_to_rpy(q);

// Get only yaw angle from quaternion
double yaw = ftf::quaternion_get_yaw(q);
```

Sources: [mavros/src/lib/ftf\_quaternion\_utils.cpp30-56](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_quaternion_utils.cpp#L30-L56)

## 5\. Common Usage Patterns

### 5.1 Setting Up a Map Origin

For conversion between local and global coordinates, a map origin must be established:

```
// Option 1: Initialize from a GPS message
void handle_gps_global_origin(GPS_GLOBAL_ORIGIN & glob_orig) {
    map_origin.x() = glob_orig.latitude / 1E7;
    map_origin.y() = glob_orig.longitude / 1E7;
    map_origin.z() = glob_orig.altitude / 1E3;
    
    // Convert to ECEF
    earth.Forward(
        map_origin.x(), map_origin.y(), map_origin.z(),
        ecef_origin.x(), ecef_origin.y(), ecef_origin.z());
}

// Option 2: Initialize from first valid GPS fix
if (!is_map_init && fix.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    map_origin.x() = fix.latitude;
    map_origin.y() = fix.longitude;
    map_origin.z() = fix.altitude;
    
    ecef_origin = map_point;    // Local position is zero
    is_map_init = true;
}
```

Sources: [mavros/src/plugins/global\_position.cpp279-293](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L279-L293) [mavros/src/plugins/global\_position.cpp378-385](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L378-L385)

### 5.2 Converting Between Body and World Frames

```
// Convert NED velocity to body frame
Eigen::Quaterniond enu_orientation; tf2::fromMsg(enu_orientation_msg, enu_orientation);
auto baselink_linear = ftf::transform_frame_enu_baselink(enu_velocity, enu_orientation.inverse());

// Convert body angular velocity to ENU frame
tf2::toMsg(
    ftf::transform_frame_baselink_enu(ftf::to_eigen(baselink_angular_msg), enu_orientation),
    twist_local.twist.angular);
```

Sources: [mavros/src/plugins/local\_position.cpp157-159](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L157-L159) [mavros/src/plugins/local\_position.cpp195-197](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/local_position.cpp#L195-L197)

## 6\. Common Pitfalls and Recommendations

1.  **Frame Consistency**: Always be aware of which frame your data is in and document it clearly.
    
2.  **Quaternion Normalization**: Always normalize quaternions before using them in transformations:
    
    ```
    R.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    ```
    
3.  **Map Origin Initialization**: Ensure your map origin is properly initialized before attempting transformations between local and global frames.
    
4.  **Coordinate Order**: Remember that LLA coordinates are in (latitude, longitude, altitude) order, while ENU coordinates are (east, north, up).
    
5.  **Units Conversion**: Pay attention to unit conversions:
    
    -   MAVLink often uses scaled integers (e.g., lat/lon as int32 1e7)
    -   Ensure proper scaling between MAVLink and ROS units

Sources: [mavros/src/plugins/global\_position.cpp362-384](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/global_position.cpp#L362-L384) [mavros/src/lib/ftf\_frame\_conversions.cpp294-296](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/lib/ftf_frame_conversions.cpp#L294-L296)