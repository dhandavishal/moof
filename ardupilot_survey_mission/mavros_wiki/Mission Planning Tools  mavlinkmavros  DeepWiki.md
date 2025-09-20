Relevant source files

-   [mavros/mavros/cmd/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/ftp.py)
-   [mavros/mavros/cmd/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py)
-   [mavros/mavros/cmd/param.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py)
-   [mavros/mavros/cmd/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/system.py)
-   [mavros/mavros/command.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/command.py)
-   [mavros/mavros/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/ftp.py)
-   [mavros/mavros/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py)
-   [mavros/mavros/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/system.py)

This document covers the mission planning tools provided by MAVROS for managing waypoints, geofences, and rally points on MAVLink-enabled flight controllers. These tools enable efficient mission creation, modification, and execution through both command-line interfaces and programmatic APIs.

For related functionality on parameter management, see [Parameter Management](https://deepwiki.com/mavlink/mavros/5.2-parameter-management), and for file transfer operations, see [FTP and File Management](https://deepwiki.com/mavlink/mavros/5.3-ftp-and-file-management).

## Overview

MAVROS mission planning provides essential functionality for:

-   Loading and saving mission files in different formats
-   Transferring mission data between the companion computer and flight controller
-   Managing different mission components (waypoints, geofences, rally points)
-   Setting and querying the current mission state

### Mission Components Architecture

```
External SystemsCLI ToolMission Planning ComponentsMissionPluginBaseWaypointPluginGeofencePluginRallypointPluginPlanFile (Base)QGroundControlWPLQGroundControlPlanmavwp commandpullshowloaddumpclearsetcurFlight ControllerMission Files (.txt, .plan)
```

Sources: [mavros/mavros/mission.py69-243](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L69-L243) [mavros/mavros/cmd/mission.py40-453](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py#L40-L453)

## Plugin Architecture

The mission planning system is built around three primary plugins, all inheriting from the `MissionPluginBase` class:

1.  `WaypointPlugin` - Manages standard waypoints for mission paths
2.  `GeofencePlugin` - Handles geofence boundaries (inclusion/exclusion zones)
3.  `RallypointPlugin` - Manages rally points for emergency landing locations

Each plugin provides similar functionality through a consistent interface, including operations for:

-   Pulling points from the flight controller
-   Pushing points to the flight controller
-   Clearing points
-   Subscribing to point updates

The `WaypointPlugin` additionally supports setting the current waypoint in the mission sequence.

### Mission Plugin Classes

```
MissionPluginBase+cli_pull+cli_push+cli_clear+points+subscribe_points()WaypointPlugin+cli_set_currentGeofencePluginRallypointPluginPlanFile+mission+fence+rally+load()+save()QGroundControlWPL+file_header+known_versions+CSVDialect+load()+save()QGroundControlPlan
```

Sources: [mavros/mavros/mission.py169-243](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L169-L243)

## Waypoint Structure

All mission points (waypoints, geofence vertices, rally points) use the common `Waypoint` message format:

```
Common Command TypesCommon Frame TypesWaypoint Message StructureWaypointis_current: boolframe: intcommand: intparam1: floatparam2: floatparam3: floatparam4: floatx_lat: floaty_long: floatz_alt: floatautocontinue: boolFRAME_GLOBAL (GAA)FRAME_GLOBAL_REL_ALT (GRA)FRAME_LOCAL_ENUFRAME_LOCAL_NEDFRAME_MISSIONNAV_WAYPOINTNAV_RETURN_TO_LAUNCHNAV_LANDNAV_TAKEOFFNAV_LOITER_TIME/TURNS/UNLIMNAV_FENCE_* commandsNAV_RALLY_POINT
```

Sources: [mavros/mavros/mission.py34-68](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L34-L68)

## Mission File Formats

MAVROS supports multiple mission file formats:

### QGroundControl WPL Format

A tab-delimited CSV format for waypoints, identified by the header `QGC WPL 120`. This is the most widely supported format but only handles waypoints (not geofences or rally points).

Example format:

```
QGC WPL 120
0    1    0    16    0.000000    0.000000    0.000000    0.000000    47.397731    8.545594    488.000000    1
1    0    0    16    0.000000    0.000000    0.000000    0.000000    47.398884    8.543753    488.000000    1
```

### QGroundControl Plan Format

A JSON-based format that can support waypoints, geofences, and rally points. The implementation in MAVROS is marked as incomplete (class definition is empty with a TODO comment).

The `mavwp` command-line utility provides the following operations:

| Command | Description | Key Options |
| --- | --- | --- |
| `pull` | Download mission from FCU | `--mission/--no-mission`, `--fence/--no-fence`, `--rally/--no-rally` |
| `show` | Display mission points | `--mission`, `--fence`, `--rally`, `--pull`, `--follow` |
| `load` | Load mission from file to FCU | `--qgc-wpl`, `--qgc-plan`, `--preserve-home`, `--start-index`, `--end-index` |
| `dump` | Save mission from FCU to file | `--qgc-wpl`, `--qgc-plan`, `--no-mission`, `--no-fence`, `--no-rally` |
| `clear` | Clear mission from FCU | `--mission/--no-mission`, `--fence/--no-fence`, `--rally/--no-rally` |
| `setcur` | Set current mission item | Requires sequence number argument |

### Mission Data Flow

```
"Mission Files""Flight Controller""Mission Plugins""mavwp Tool"User"Mission Files""Flight Controller""Mission Plugins""mavwp Tool"UserLoad MissionSave MissionSet Current Waypointmavwp load mission.txtload()PlanFile with pointscli_push.call(WaypointPush.Request)Send via MAVLinkACKResponseSuccess/failure messagemavwp dump mission.txtPull pointsRequest via MAVLinkMission dataPoints collectionsave()SuccessSuccess messagemavwp setcur 2cli_set_current.call(WaypointSetCurrent.Request)MISSION_SET_CURRENTMISSION_CURRENTResponse"Set current done."
```

Sources: [mavros/mavros/cmd/mission.py85-453](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py#L85-L453)

## Usage Examples

### Pull and Show a Mission

To download the current mission from the flight controller and display it:

```
# Pull the mission from FCU
mavwp pull

# Show the mission
mavwp show
```

### Load a Mission from File

To load a mission from a QGroundControl WPL file:

```
mavwp load mission.txt
```

With options to preserve home position and load only certain waypoints:

```
mavwp load --preserve-home --start-index 1 --end-index 5 mission.txt
```

### Save a Mission to File

To save the current mission to a file:

```
mavwp dump mission.txt
```

To specify the file format explicitly:

```
mavwp dump --qgc-wpl mission.txt
```

### Clear the Mission

To clear all mission items from the flight controller:

```
mavwp clear
```

To clear only specific mission components:

```
mavwp clear --mission --no-fence --no-rally
```

### Set the Current Waypoint

To set waypoint #3 as the current active waypoint:

```
mavwp setcur 3
```

## Programmatic API

For developers integrating with MAVROS, the mission planning system provides a Python API:

```
# Example: Pull waypoints from FCU
waypoint_plugin = mavros_node.waypoint
req = WaypointPull.Request()
response = waypoint_plugin.cli_pull.call(req)
if response.success:
    print(f"Received {response.wp_received} waypoints")

# Example: Access waypoints
waypoints = waypoint_plugin.points
for wp in waypoints:
    print(f"Waypoint at lat: {wp.x_lat}, lon: {wp.y_long}, alt: {wp.z_alt}")
```

Similarly, the GeofencePlugin and RallypointPlugin provide the same interface for their respective mission components.

Sources: [mavros/mavros/mission.py169-243](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L169-L243)

## Limitations

1.  QGroundControl Plan JSON format support is incomplete (marked with TODO in the code)
2.  Some options are specific to certain autopilots (e.g., `--preserve-home` is APM-only)
3.  WPL format only supports waypoints, not geofences or rally points

Sources: [mavros/mavros/mission.py87-166](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L87-L166) [mavros/mavros/cmd/mission.py40-70](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py#L40-L70)