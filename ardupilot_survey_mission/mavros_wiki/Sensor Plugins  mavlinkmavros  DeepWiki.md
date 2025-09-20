Relevant source files

-   [mavros\_extras/src/plugins/cellular\_status.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/cellular_status.cpp)
-   [mavros\_extras/src/plugins/distance\_sensor.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/distance_sensor.cpp)
-   [mavros\_extras/src/plugins/landing\_target.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/landing_target.cpp)
-   [mavros\_extras/src/plugins/mount\_control.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mount_control.cpp)
-   [mavros\_extras/src/plugins/obstacle\_distance.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/obstacle_distance.cpp)
-   [mavros\_extras/src/plugins/odom.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/odom.cpp)
-   [mavros\_extras/src/plugins/wheel\_odometry.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/wheel_odometry.cpp)
-   [mavros\_msgs/msg/CellularStatus.msg](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/CellularStatus.msg)

This page documents the sensor plugins included in the MAVROS extras package. These plugins handle various sensor data types, providing bidirectional communication between ROS and MAVLink-compatible flight controllers (FCUs). For information about core position and orientation plugins, see [Position and Orientation Plugins](https://deepwiki.com/mavlink/mavros/3.1.2-position-and-orientation-plugins). For vision-specific plugins, see [Vision and Motion Capture Plugins](https://deepwiki.com/mavlink/mavros/3.2.2-vision-and-motion-capture-plugins).

## Sensor Plugin Architecture

MAVROS sensor plugins extend the base Plugin class and follow a common architectural pattern for handling the conversion of data between ROS and MAVLink message formats. Each plugin typically provides functionality for either sending sensor data to the FCU, receiving sensor data from the FCU, or both.

```
Plugin+UASPtr uas+std::string name+NodePtr node+virtual Subscriptions get_subscriptions()+node_declare_and_watch_parameter()TF2ListenerMixin+tf2_start()+tf2_stop()DistanceSensorPlugin-std::map<uint8_t, ItemPtr> sensor_map-std::string base_frame_id+handle_distance_sensor()+distance_sensor()ObstacleDistancePlugin-mavlink::common::MAV_FRAME frame-obstacle_cb()LandingTargetPlugin-rclcpp::Publisher land_target_pub-handle_landing_target()-send_landing_target()OdometryPlugin-Matrix6d cov_pose-Matrix6d cov_vel-handle_odom()-odom_cb()WheelOdometryPlugin-std::vector wheel_offset-std::vector wheel_radius-update_odometry()-handle_rpm()-handle_wheel_distance()MountControlPlugin-handle_mount_orientation()-handle_mount_status()-command_cb()CellularStatusPlugin-status_cb()
```

Sources: [mavros\_extras/src/plugins/distance\_sensor.cpp139-348](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/distance_sensor.cpp#L139-L348) [mavros\_extras/src/plugins/obstacle\_distance.cpp45-140](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/obstacle_distance.cpp#L45-L140) [mavros\_extras/src/plugins/landing\_target.cpp47-516](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/landing_target.cpp#L47-L516) [mavros\_extras/src/plugins/odom.cpp53-335](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/odom.cpp#L53-L335) [mavros\_extras/src/plugins/wheel\_odometry.cpp44-664](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/wheel_odometry.cpp#L44-L664) [mavros\_extras/src/plugins/mount\_control.cpp188-391](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mount_control.cpp#L188-L391) [mavros\_extras/src/plugins/cellular\_status.cpp36-76](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/cellular_status.cpp#L36-L76)

## Distance Sensor Plugin

The Distance Sensor plugin handles distance measurements between the vehicle and obstacles. It supports bidirectional communication, allowing both publishing distance sensor data from ROS to the FCU and receiving distance sensor data from the FCU to ROS.

### Features

-   Supports multiple distance sensors with different IDs
-   Configurable through YAML
-   Supports different sensor types (laser, radar, ultrasound)
-   Dynamic TF frame publishing
-   Field-of-view and orientation configuration

### Data Flow

```
"Flight Controller""Distance Sensor Plugin""ROS Application""Flight Controller""Distance Sensor Plugin""ROS Application"Sensor Data from ROS to FCUSensor Data from FCU to ROSopt[TF Publishing Enabled]sensor_msgs/Rangecalculate_variance()MAVLINK_DISTANCE_SENSORMAVLINK_DISTANCE_SENSORhandle_distance_sensor()sensor_msgs/RangeTransformStamped (TF2)
```

### Configuration

The plugin is configured using YAML format in the ROS parameter system. Each distance sensor requires a dedicated configuration block:

```
plugin_config:
  sensor_name:
    id: 0                     # Sensor ID (0-255)
    orientation: "PITCH_270"  # Sensor orientation
    frame_id: "lidar"         # Frame ID for ROS messages
    field_of_view: 0.7        # FOV in radians
    send_tf: true             # Whether to publish TF frame
    sensor_position:          # Position relative to base_link
      x: 0.0
      y: 0.0
      z: -0.1
    subscriber: false         # true if ROS->FCU, false if FCU->ROS
```

For subscribers (ROS to FCU), you can also configure:

-   `covariance`: Covariance value in centimeters
-   `horizontal_fov_ratio` and `vertical_fov_ratio`: FOV ratios for ROS messages
-   `custom_orientation`: Custom orientation in roll, pitch, yaw (degrees)

Sources: [mavros\_extras/src/plugins/distance\_sensor.cpp46-501](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/distance_sensor.cpp#L46-L501)

## Obstacle Distance Plugin

This plugin converts sensor data from a planar laser scan into an obstacle distance array used by the FCU for obstacle avoidance.

### Features

-   Converts `sensor_msgs/LaserScan` to MAVLink `OBSTACLE_DISTANCE` messages
-   Handles adaptive scaling of sensor data to fit MAVLink message constraints
-   Configurable reference frame

### Usage

The plugin subscribes to the `~/send` topic for `sensor_msgs/LaserScan` messages and forwards the data to the FCU as `OBSTACLE_DISTANCE` messages. If the number of range readings exceeds the capacity of the MAVLink message, it will intelligently combine adjacent readings by taking the minimum distance.

```
MAVLinkObstacleDistancePluginROS~/sendsensor_msgs/LaserScanobstacle_cb()MAV_FRAME settingOBSTACLE_DISTANCE message
```

Sources: [mavros\_extras/src/plugins/obstacle\_distance.cpp8-146](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/obstacle_distance.cpp#L8-L146)

## Landing Target Plugin

The Landing Target plugin facilitates precision landing by handling landing target information between ROS and the FCU.

### Features

-   Sends landing target information to the FCU
-   Receives landing target updates from the FCU
-   Configurable target size, camera parameters, and frame settings
-   TF frame publishing and listening capabilities
-   Support for different landing target types

### Data Flow

```
MAVLinkLandingTargetPluginROS~/poselanding_target_1 → camera_center~/rawIf tf_send=truegeometry_msgs/PoseStampedTF Treemavros_msgs/LandingTargetgeometry_msgs/PoseStampedgeometry_msgs/Vector3Stampedsend_landing_target()handle_landing_target()transform_cb()landtarget_cb()pose_cb()LANDING_TARGET message
```

Sources: [mavros\_extras/src/plugins/landing\_target.cpp41-516](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/landing_target.cpp#L41-L516)

## Odometry Plugins

MAVROS provides two plugins for handling odometry data: the standard Odometry plugin and the Wheel Odometry plugin.

### Standard Odometry Plugin

The standard Odometry plugin transfers vehicle position and velocity estimates between ROS and the FCU.

#### Features

-   Bidirectional communication of odometry data
-   Proper coordinate frame transformations
-   Covariance handling

#### Configuration

The plugin can be configured with:

-   `fcu.odom_parent_id_des`: Desired parent frame ID (default: "map")
-   `fcu.odom_child_id_des`: Desired child frame ID (default: "base\_link")

```
"Flight Controller""Odometry Plugin""ROS""Flight Controller""Odometry Plugin""ROS"Odometry from ROS to FCUOdometry from FCU to ROSnav_msgs/Odometrytransform framesMAVLINK_ODOMETRYMAVLINK_ODOMETRYhandle_odom()nav_msgs/Odometry
```

Sources: [mavros\_extras/src/plugins/odom.cpp53-335](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/odom.cpp#L53-L335)

### Wheel Odometry Plugin

The Wheel Odometry plugin computes odometry based on wheel encoder data from the FCU or sends wheel odometry to the FCU.

#### Features

-   Processes wheel RPM or cumulative distance measurements
-   Computes vehicle pose and twist based on wheel measurements
-   Supports primarily differential drive configurations
-   Integrates with vehicle orientation from IMU data

#### Configuration

Key parameters include:

-   `count`: Number of wheels (default: 2)
-   `use_rpm`: Whether to use RPM measurements instead of wheel distances
-   `send_raw`: Whether to publish raw wheel data
-   `frame_id` and `child_frame_id`: Frame IDs for published messages
-   Configuration for each wheel including position and radius

```
ROS MessagesWheelOdometryPluginMAVLink MessagesRPM messageWHEEL_DISTANCE messagehandle_rpm()handle_wheel_distance()process_measurement()update_odometry()publish_odometry()WheelOdomStampednav_msgs/OdometryTwistWithCovarianceStamped
```

Sources: [mavros\_extras/src/plugins/wheel\_odometry.cpp44-665](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/wheel_odometry.cpp#L44-L665)

## Mount Control Plugin

The Mount Control plugin manages camera or antenna mounts on the vehicle, allowing control of their orientation and monitoring their status.

### Features

-   Control of camera/antenna mount orientation
-   Reception of mount orientation updates from the FCU
-   Configuration of mount parameters
-   Diagnostic capabilities to detect mount errors

### Data Flow

```
MAVLinkMountControlPluginROS~/command~/configuremavros_msgs/MountControlgeometry_msgs/Quaterniongeometry_msgs/Vector3Stampedmavros_msgs/srv/MountConfigurecommand_cb()handle_mount_orientation()handle_mount_status()mount_configure_cb()MountStatusDiagCOMMAND_LONG (DO_MOUNT_CONTROL)MOUNT_ORIENTATIONMOUNT_STATUSCOMMAND_LONG (DO_MOUNT_CONFIGURE)
```

Sources: [mavros\_extras/src/plugins/mount\_control.cpp188-391](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/mount_control.cpp#L188-L391)

## Cellular Status Plugin

The Cellular Status plugin is a simple plugin that relays cellular connectivity information to the FCU.

### Features

-   Forwards cellular status data from ROS to the FCU
-   Includes information about signal quality, network identification, and connectivity status

### Usage

To use this plugin, publish a `mavros_msgs/CellularStatus` message to the `~/status` topic, which will be forwarded to the FCU.

```
MAVLinkCellularStatusPluginROS~/statusmavros_msgs/CellularStatusstatus_cb()CELLULAR_STATUS
```

The `CellularStatus` message includes fields for:

-   Network status and quality
-   Mobile Country Code (MCC) and Mobile Network Code (MNC)
-   Location Area Code (LAC)
-   Failure reason (if applicable)
-   Connection type

Sources: [mavros\_extras/src/plugins/cellular\_status.cpp36-81](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_extras/src/plugins/cellular_status.cpp#L36-L81) [mavros\_msgs/msg/CellularStatus.msg1-9](https://github.com/mavlink/mavros/blob/44fa2f90/mavros_msgs/msg/CellularStatus.msg#L1-L9)