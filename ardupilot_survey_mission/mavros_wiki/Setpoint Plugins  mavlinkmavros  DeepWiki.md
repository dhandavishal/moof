Relevant source files

-   [mavros/src/plugins/setpoint\_accel.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_accel.cpp)
-   [mavros/src/plugins/setpoint\_attitude.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp)
-   [mavros/src/plugins/setpoint\_position.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp)
-   [mavros/src/plugins/setpoint\_raw.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_raw.cpp)
-   [mavros/src/plugins/setpoint\_trajectory.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_trajectory.cpp)
-   [mavros/src/plugins/setpoint\_velocity.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp)
-   [mavros/src/plugins/wind\_estimation.cpp](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/wind_estimation.cpp)

## Purpose and Scope

Setpoint plugins provide mechanisms to control flight controllers (FCUs) by translating ROS messages into MAVLink commands. These plugins handle different control modes such as position, velocity, attitude, and acceleration setpoints. For information about coordinating vehicle movement using waypoints, see [WaypointPlugin](https://deepwiki.com/mavlink/mavros/4.2-ardupilot-configuration#3.1.8). For more advanced control through offboard mode, these setpoint plugins are the primary interface.

## Plugin Architecture

The setpoint plugins in MAVROS follow a common architecture pattern. Each plugin inherits from the base `Plugin` class and uses specialized mixins to handle the creation of specific MAVLink messages.

```
ImplementsImplementsImplementsExtendsExtendsExtendsExtendsExtendsExtendsUsed byUsed byUsed byUsed byUsed byUsed byUsed byUsed byUsed byPlugin Base ClassSetpoint MixinsSetAttitudeTargetMixinSetPositionTargetLocalNEDMixinSetPositionTargetGlobalIntMixinSetpointAttitudePluginSetpointPositionPluginSetpointVelocityPluginSetpointAccelerationPluginSetpointRawPluginSetpointTrajectoryPlugin
```

Sources: [mavros/src/plugins/setpoint\_attitude.cpp60-62](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp#L60-L62) [mavros/src/plugins/setpoint\_position.cpp43-45](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp#L43-L45) [mavros/src/plugins/setpoint\_velocity.cpp42-44](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp#L42-L44) [mavros/src/plugins/setpoint\_accel.cpp40-42](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_accel.cpp#L40-L42) [mavros/src/plugins/setpoint\_raw.cpp43-46](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_raw.cpp#L43-L46) [mavros/src/plugins/setpoint\_trajectory.cpp48-50](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_trajectory.cpp#L48-L50)

## Common Data Flow

All setpoint plugins share a similar data flow pattern:

```
"Flight Controller""UAS""Setpoint Mixin""Setpoint Plugin""ROS Application""Flight Controller""UAS""Setpoint Mixin""Setpoint Plugin""ROS Application""Transform coordinates (ENU to NED)""ROS Setpoint Message""Call set_xxx_target()""Create MAVLink message""Send MAVLink message"
```

Sources: [mavros/src/plugins/setpoint\_attitude.cpp164-183](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp#L164-L183) [mavros/src/plugins/setpoint\_position.cpp132-171](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp#L132-L171) [mavros/src/plugins/setpoint\_velocity.cpp98-131](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp#L98-L131)

## Available Setpoint Plugins

MAVROS provides the following setpoint plugins:

| Plugin | Purpose | Control Aspect |
| --- | --- | --- |
| SetpointAttitudePlugin | Controls vehicle orientation and thrust | Attitude (Orientation) |
| SetpointPositionPlugin | Controls vehicle position | Position (Local/Global) |
| SetpointVelocityPlugin | Controls vehicle velocity | Velocity (Linear and Angular) |
| SetpointAccelerationPlugin | Controls vehicle acceleration or force | Acceleration/Force |
| SetpointRawPlugin | Low-level control with fine-grained type\_mask flags | All of the above |
| SetpointTrajectoryPlugin | Execute multi-point trajectories | Position over time |

### SetpointAttitudePlugin

Controls the vehicle's attitude (orientation) and thrust.

**ROS Topics:**

-   `~/attitude` (geometry\_msgs/PoseStamped) - Attitude setpoint (quaternion)
-   `~/cmd_vel` (geometry\_msgs/TwistStamped) - Angular velocity setpoint
-   `~/thrust` (mavros\_msgs/Thrust) - Thrust setpoint (normalized 0.0-1.0)

**Parameters:**

-   `~use_quaternion` (bool, default: false) - Use quaternion or angular velocity control
-   `~reverse_thrust` (bool, default: false) - Allow negative thrust values

**MAVLink Message:**

-   `SET_ATTITUDE_TARGET` - Contains quaternion orientation, body rates, and thrust

```
truefalseUse quaternionparameterattitude topic(PoseStamped)cmd_vel topic(TwistStamped)thrust topic(Thrust)Message Syncsend_attitude_quaternion()send_attitude_ang_velocity()set_attitude_target()MAVLink Message
```

Sources: [mavros/src/plugins/setpoint\_attitude.cpp60-232](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp#L60-L232)

### SetpointPositionPlugin

Controls the vehicle's position in either local or global coordinates.

**ROS Topics:**

-   `~/local` (geometry\_msgs/PoseStamped) - Local position setpoint
-   `~/global` (geographic\_msgs/GeoPoseStamped) - Global position setpoint
-   `~/global_to_local` (geographic\_msgs/GeoPoseStamped) - Global position converted to local

**Parameters:**

-   `~mav_frame` (string, default: "LOCAL\_NED") - MAVLink frame for setpoints

**MAVLink Messages:**

-   `SET_POSITION_TARGET_LOCAL_NED` - For local position control
-   `SET_POSITION_TARGET_GLOBAL_INT` - For global position control

```
local topic(PoseStamped)send_position_target()global topic(GeoPoseStamped)set_position_target_global_int()global_to_local topic(GeoPoseStamped)Convert GPS to local ENUCurrent GPS positionset_position_target_local_ned()MAVLink MessageSET_POSITION_TARGET_GLOBAL_INTMAVLink Message
```

Sources: [mavros/src/plugins/setpoint\_position.cpp43-285](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp#L43-L285)

### SetpointVelocityPlugin

Controls the vehicle's velocity.

**ROS Topics:**

-   `~/cmd_vel` (geometry\_msgs/TwistStamped) - Velocity setpoint
-   `~/cmd_vel_unstamped` (geometry\_msgs/Twist) - Unstamped velocity setpoint

**Parameters:**

-   `~mav_frame` (string, default: "LOCAL\_NED") - MAVLink frame for setpoints

**MAVLink Message:**

-   `SET_POSITION_TARGET_LOCAL_NED` - With appropriate flags to control only velocity

```
cmd_vel topic(TwistStamped)send_setpoint_velocity()cmd_vel_unstamped topic(Twist)Transform ENU to NEDset_position_target_local_ned()MAVLink Message
```

Sources: [mavros/src/plugins/setpoint\_velocity.cpp42-154](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp#L42-L154)

### SetpointAccelerationPlugin

Controls the vehicle's acceleration or force.

**ROS Topics:**

-   `~/accel` (geometry\_msgs/Vector3Stamped) - Acceleration setpoint

**Parameters:**

-   `~send_force` (bool, default: false) - Send as force rather than acceleration

**MAVLink Message:**

-   `SET_POSITION_TARGET_LOCAL_NED` - With appropriate flags to control only acceleration/force

```
accel topic(Vector3Stamped)send_setpoint_acceleration()send_force parameterTransform ENU to NEDset_position_target_local_ned()MAVLink Message
```

Sources: [mavros/src/plugins/setpoint\_accel.cpp40-111](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_accel.cpp#L40-L111)

### SetpointRawPlugin

Provides direct control over position, velocity, acceleration, and attitude setpoints with fine-grained control through `type_mask` flags.

**ROS Topics:**

-   `~/local` (mavros\_msgs/PositionTarget) - Local position/velocity/acceleration setpoint
-   `~/global` (mavros\_msgs/GlobalPositionTarget) - Global position/velocity/acceleration setpoint
-   `~/attitude` (mavros\_msgs/AttitudeTarget) - Attitude setpoint

**Published Topics:**

-   `~/target_local` (mavros\_msgs/PositionTarget) - Current local target
-   `~/target_global` (mavros\_msgs/GlobalPositionTarget) - Current global target
-   `~/target_attitude` (mavros\_msgs/AttitudeTarget) - Current attitude target

**Parameters:**

-   `~thrust_scaling` (double, default: 1.0) - Scale factor for thrust values

**MAVLink Messages:**

-   `SET_POSITION_TARGET_LOCAL_NED`
-   `SET_POSITION_TARGET_GLOBAL_INT`
-   `SET_ATTITUDE_TARGET`

```
local topic(PositionTarget)local_cb()global topic(GlobalPositionTarget)global_cb()attitude topic(AttitudeTarget)attitude_cb()set_position_target_local_ned()set_position_target_global_int()thrust_scaling parameterset_attitude_target()POSITION_TARGET_LOCAL_NEDMAVLink Messagetarget_local topic(PositionTarget)POSITION_TARGET_GLOBAL_INTMAVLink Messagetarget_global topic(GlobalPositionTarget)ATTITUDE_TARGETMAVLink Messagetarget_attitude topic(AttitudeTarget)
```

Sources: [mavros/src/plugins/setpoint\_raw.cpp43-326](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_raw.cpp#L43-L326)

### SetpointTrajectoryPlugin

Executes multi-point trajectories by sending individual setpoints along a path.

**ROS Topics:**

-   `~/local` (trajectory\_msgs/MultiDOFJointTrajectory) - Trajectory setpoint
-   `~/desired` (nav\_msgs/Path) - Published desired path

**Services:**

-   `~/reset` (std\_srvs/Trigger) - Reset trajectory

**Parameters:**

-   `~frame_id` (string, default: "map") - Frame ID for path message
-   `~mav_frame` (string, default: "LOCAL\_NED") - MAVLink frame for setpoints

**MAVLink Message:**

-   `SET_POSITION_TARGET_LOCAL_NED` - With appropriate values for each trajectory point

```
local topic(MultiDOFJointTrajectory)local_cb()Store trajectorystart timer for first pointpublish_path()desired topic(Path)Timer callbackProcess current pointset_position_target_local_ned()Advance to next pointReset timerreset service(Trigger)reset_cb()Clear trajectory
```

Sources: [mavros/src/plugins/setpoint\_trajectory.cpp48-267](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_trajectory.cpp#L48-L267)

## Coordinate Transformations

All setpoint plugins perform coordinate transformations between different reference frames:

1.  **ROS to MAVLink Frame Transformation**:
    
    -   ROS uses ENU (East-North-Up) coordinates
    -   MAVLink uses NED (North-East-Down) coordinates
2.  **Body Frame Transformations**:
    
    -   ROS body frame: baselink (Forward-Left-Up)
    -   MAVLink body frame: aircraft (Forward-Right-Down)

```
transform_frame_enu_ned()transform_frame_baselink_aircraft()transform_orientation_enu_ned()ENU Frame(East-North-Up)NED Frame(North-East-Down)Baselink Frame(Forward-Left-Up)Aircraft Frame(Forward-Right-Down)Quaternion ENUQuaternion NED
```

Sources: [mavros/src/plugins/setpoint\_position.cpp144-151](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp#L144-L151) [mavros/src/plugins/setpoint\_velocity.cpp107-114](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp#L107-L114) [mavros/src/plugins/setpoint\_attitude.cpp172-177](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp#L172-L177)

## Using Type Masks

Many setpoint plugins use type masks to specify which components of the setpoint to use or ignore. These masks are defined in the MAVLink protocol and determine which fields in the message are considered valid.

For example, in the `SetpointVelocityPlugin`:

```
// Ignore position and accel vectors, yaw
uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);
```

This creates a mask that tells the FCU to ignore position, acceleration vectors, and yaw, focusing only on velocity and yaw rate.

Sources: [mavros/src/plugins/setpoint\_velocity.cpp106-107](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp#L106-L107) [mavros/src/plugins/setpoint\_attitude.cpp171](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp#L171-L171) [mavros/src/plugins/setpoint\_position.cpp142](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp#L142-L142)

## Practical Usage

When using setpoint plugins, keep in mind:

1.  **Coordinate System Awareness**: Always remember that ROS messages use ENU frame, while the FCU uses NED frame. The plugins handle the transformation internally.
    
2.  **MAV\_FRAME Selection**: Select the appropriate MAV\_FRAME parameter based on whether you want to control in local coordinates, global coordinates, or body-relative coordinates.
    
3.  **Offboard Mode**: To control a vehicle using setpoints, the FCU must be in OFFBOARD mode (for PX4) or GUIDED mode (for ArduPilot).
    
4.  **Control Hierarchy**: Only one control method should be actively sending setpoints at a time (position, velocity, or attitude).
    
5.  **Message Rate**: Setpoints should be sent at a consistent rate (typically 10-50 Hz) to maintain control.
    

Sources: [mavros/src/plugins/setpoint\_attitude.cpp82-108](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_attitude.cpp#L82-L108) [mavros/src/plugins/setpoint\_position.cpp51-67](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_position.cpp#L51-L67) [mavros/src/plugins/setpoint\_velocity.cpp51-63](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/src/plugins/setpoint_velocity.cpp#L51-L63)