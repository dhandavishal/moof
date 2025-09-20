Relevant source files

-   [mavros/mavros/base.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/base.py)
-   [mavros/mavros/cmd/\_\_init\_\_.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/__init__.py)
-   [mavros/mavros/cmd/checkid.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/checkid.py)
-   [mavros/mavros/cmd/cmd.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/cmd.py)
-   [mavros/mavros/cmd/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/ftp.py)
-   [mavros/mavros/cmd/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py)
-   [mavros/mavros/cmd/param.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py)
-   [mavros/mavros/cmd/safety.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/safety.py)
-   [mavros/mavros/cmd/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/system.py)
-   [mavros/mavros/cmd/utils.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/utils.py)
-   [mavros/mavros/command.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/command.py)
-   [mavros/mavros/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/ftp.py)
-   [mavros/mavros/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py)
-   [mavros/mavros/param.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/param.py)
-   [mavros/mavros/setpoint.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/setpoint.py)
-   [mavros/mavros/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/system.py)

This page documents the command line utilities provided by MAVROS, which offer direct access to various MAVLink device functionalities without requiring custom ROS 2 applications. These tools are designed for quick operations, debugging, and testing with MAVLink-enabled flight controllers.

For information about the underlying plugin infrastructure these tools are built upon, see [Plugin System](https://deepwiki.com/mavlink/mavros/3-plugin-system).

MAVROS provides several command line tools that serve different purposes in working with MAVLink-enabled flight controllers:

1.  `mavcmd` - Send commands to the flight controller
2.  `mavparam` - Read, write, and manage flight controller parameters
3.  `mavwp` - Manage mission waypoints, geofence, and rally points
4.  `mavftp` - Transfer files between companion computer and flight controller
5.  `mavsys` - Control system settings like mode and stream rates
6.  `mavcheckid` - Diagnostic tool to verify proper system ID setup

All tools share a common interface and connection mechanism to interact with the MAVROS node.

Sources: [mavros/mavros/cmd/\_\_init\_\_.py72-132](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/__init__.py#L72-L132)

## CLI Architecture

The MAVROS command line tools follow a unified architecture based on the Click Python framework. All tools connect to an active MAVROS node and provide access to specific functions through a consistent command structure.

```
MAVROS NodeMAVROS Python ClientCommand Line Toolsmavcmd(Command Interface)mavparam(Parameter Management)mavwp(Mission Management)mavftp(File Transfer)mavsys(System Control)mavcheckid(ID Checking)CLI Entry Point(cmd/init.py)CliClient(ROS Interface)UAS(Unmanned Aircraft System)Plugin System
```

### Common CLI Framework

All tools use a common framework that handles:

-   Connecting to the MAVROS node
-   Starting a ROS spinner thread
-   Waiting for FCU connection
-   Handling command-line arguments
-   Providing verbose output options

Each command follows a similar pattern where they define command groups and subcommands with Click decorators, then implement the actual functionality using the MAVROS Python client.

Sources: [mavros/mavros/cmd/\_\_init\_\_.py21-132](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/__init__.py#L21-L132)

### Mission Planning Tool (mavwp)

The `mavwp` tool manages waypoint missions, geofences, and rally points on the flight controller.

```
Mission TypesFile Formatsmavwp CommandspullDownload mission from FCUloadUpload mission to FCUshowDisplay current missiondumpSave mission to fileclearClear mission on FCUsetcurSet current waypointQGroundControl WPL(waypoint list format)QGroundControl Plan(mission plan format)Waypoints(mission path)Geofence(boundary)Rally Points(emergency landing)
```

Key capabilities:

-   Download missions from the flight controller
-   Upload missions to the flight controller
-   Display missions in a human-readable format
-   Clear missions from the flight controller
-   Set the current mission waypoint
-   Support for QGroundControl formats

Example usage:

```
# Download the mission from the flight controller
mavwp pull

# Show the current mission
mavwp show

# Upload a mission from a file
mavwp load mission.txt

# Clear the mission on the flight controller
mavwp clear
```

Sources: [mavros/mavros/cmd/mission.py40-454](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py#L40-L454) [mavros/mavros/mission.py71-244](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L71-L244)

### Parameter Management Tool (mavparam)

The `mavparam` tool manages the parameters of the flight controller.

```
Parameter SystemFile Formatsmavparam CommandsgetRead parameter valuesetWrite parameter valuedumpSave parameters to fileloadLoad parameters from fileMissionPlanner formatQGroundControl formatMAVProxy formatParamDict(Parameter Cache)ParamPlugin(ROS Interface)
```

Key capabilities:

-   Read individual parameter values
-   Write parameter values
-   Save all parameters to files in various formats
-   Load parameters from files
-   Support for different parameter file formats (MissionPlanner, QGroundControl, MAVProxy)

Example usage:

```
# Get a parameter value
mavparam get SYSID_THISMAV

# Set a parameter value
mavparam set SYSID_MYGCS 1

# Save all parameters to a file
mavparam dump params.txt

# Load parameters from a file
mavparam load params.txt
```

Sources: [mavros/mavros/cmd/param.py19-145](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L19-L145) [mavros/mavros/param.py31-350](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/param.py#L31-L350)

### FTP and File Management Tool (mavftp)

The `mavftp` tool enables file operations on the flight controller's filesystem.

```
FTP Systemmavftp CommandslsList directory contentscdChange directorymkdirCreate directoryrmdirRemove directorycatDisplay file contentsrmRemove filedownloadDownload fileuploadUpload fileverifyCheck file integrityresetReset FTP serverFTPPlugin(ROS Interface)FTPFile(File Object)
```

Key capabilities:

-   List files and directories on the flight controller
-   Transfer files between companion computer and flight controller
-   Manage directories on the flight controller
-   View file contents directly
-   Verify file integrity with CRC32 checksums

Example usage:

```
# List files in the current directory
mavftp ls

# Change directory
mavftp cd /fs/microsd

# Download a file
mavftp download /fs/microsd/params.txt local_params.txt

# Upload a file
mavftp upload local_file.txt /fs/microsd/remote_file.txt
```

Sources: [mavros/mavros/cmd/ftp.py29-299](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/ftp.py#L29-L299) [mavros/mavros/ftp.py38-233](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/ftp.py#L38-L233)

### Command Tool (mavcmd)

The `mavcmd` tool sends direct commands to the flight controller.

```
Command SystemSafety Commandsmavcmd CommandslongSend COMMAND_LONGintSend COMMAND_INTset_homeSet home positiontakeoffRequest takeofftakeoff_curTakeoff from current positionlandRequest landingland_curLand at current positiontrigger_controlCamera trigger controltrigger_intervalCamera trigger intervalarmArm motorsdisarmDisarm motorskillEmergency kill motorsCommandPlugin(ROS Interface)CommandLong(Service)CommandInt(Service)CommandTOL(Service)CommandHome(Service)CommandBool(Service)CommandTriggerControl(Service)CommandTriggerInterval(Service)
```

Key capabilities:

-   Send generic MAVLink commands via COMMAND\_LONG and COMMAND\_INT
-   Set home position
-   Request takeoff and landing
-   Control camera triggering
-   Safety commands (arm, disarm, kill)

Example usage:

```
# Arm the motors
mavcmd safety arm

# Disarm the motors
mavcmd safety disarm

# Take off to 10m altitude
mavcmd takeoff_cur 15 0 10

# Land at current position
mavcmd land_cur 0 0
```

Sources: [mavros/mavros/cmd/cmd.py33-349](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/cmd.py#L33-L349) [mavros/mavros/cmd/safety.py22-61](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/safety.py#L22-L61) [mavros/mavros/command.py24-57](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/command.py#L24-L57)

### System Tool (mavsys)

The `mavsys` tool controls system settings and state.

```
System Pluginmavsys CommandsmodeSet flight moderateSet stream ratesmessage_intervalSet message intervalsSystemPlugin(ROS Interface)SetMode(Service)StreamRate(Service)MessageInterval(Service)
```

Key capabilities:

-   Change flight mode
-   Set MAVLink stream rates for different message types
-   Control message intervals for individual message IDs

Example usage:

```
# Change to GUIDED mode
mavsys mode -c GUIDED

# Set all stream rates to 10Hz
mavsys rate --all 10

# Set message interval for a specific message
mavsys message_interval --id 33 --rate 50.0
```

Sources: [mavros/mavros/cmd/system.py21-156](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/system.py#L21-L156) [mavros/mavros/system.py27-93](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/system.py#L27-L93)

### Check ID Diagnostic Tool (mavcheckid)

The `mavcheckid` tool verifies the correct configuration of system IDs.

```
Mavlink Message Flowmavcheckid CommandscheckidCheck system and component IDsMavlink Source(Router Topic)Message Sources(Dict of Sets)Report(Display IDs and Messages)
```

Key capabilities:

-   Listen to MAVLink messages on the router
-   Verify correct system and component IDs
-   Display information about received message types and sources

Example usage:

```
# Check system and component IDs
mavcheckid

# Watch continuously for ID changes
mavcheckid --follow
```

Sources: [mavros/mavros/cmd/checkid.py33-131](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/checkid.py#L33-L131)

## Common Usage Patterns

All MAVROS command line tools support several common options:

| Option | Description |
| --- | --- |
| `--node-name` | Set the ROS node name for the client (default: random) |
| `--mavros-ns` | Set the namespace of the MAVROS UAS node (default: mavros) |
| `--verbose` | Enable verbose output for debugging |
| `--wait-fcu` | Wait for establishing FCU connection before executing commands |
| `--version` | Show program version and exit |

Basic usage pattern:

```
# Basic format
<tool> [options] <command> [command options]

# Examples
mavparam --verbose get SYSID_THISMAV
mavwp --wait-fcu pull
mavftp ls /
```

Sources: [mavros/mavros/cmd/\_\_init\_\_.py72-132](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/__init__.py#L72-L132)

## Integration with ROS 2

The command line tools rely on ROS 2 Python libraries to communicate with the MAVROS node. They create a ROS 2 node with a spinner thread for each command execution.

```
"Flight Controller""MAVROS Node""ROS 2 Node""CliClient""CLI Tool"User"Flight Controller""MAVROS Node""ROS 2 Node""CliClient""CLI Tool"Useralt[Wait FCU specified]Invoke commandCreate clientInitialize nodeStart spinner threadWait for FCU connectionConnection checkConnected statusConnection statusExecute subcommandCall service/topicForward requestMAVLink command/messageResponseService responseProcess responseDisplay resultExitShutdown
```

This architecture allows the command line tools to leverage the full capabilities of the MAVROS ecosystem without duplicating functionality.

Sources: [mavros/mavros/cmd/\_\_init\_\_.py112-131](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/__init__.py#L112-L131) [mavros/mavros/base.py78-137](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/base.py#L78-L137)

## Advanced Features

### File Format Support

The parameter and mission management tools support multiple file formats:

| Tool | Supported Formats |
| --- | --- |
| mavparam | MissionPlanner (.param), QGroundControl (.txt), MAVProxy |
| mavwp | QGroundControl WPL (.waypoints), QGroundControl Plan (.plan) |

This makes it easy to exchange data with popular ground control stations.

Sources: [mavros/mavros/param.py31-162](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/param.py#L31-L162) [mavros/mavros/mission.py71-166](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py#L71-L166)

### Command and Parameter Completion

Some commands, particularly `mavcmd` and `mavparam`, can work with a large number of possible commands and parameters. The tools provide detailed help information when used with the `--help` flag to assist users.

Sources: [mavros/mavros/cmd/cmd.py38-349](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/cmd.py#L38-L349) [mavros/mavros/cmd/param.py82-144](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L82-L144)