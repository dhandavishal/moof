Relevant source files

-   [mavros/mavros/cmd/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/ftp.py)
-   [mavros/mavros/cmd/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py)
-   [mavros/mavros/cmd/param.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py)
-   [mavros/mavros/cmd/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/system.py)
-   [mavros/mavros/command.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/command.py)
-   [mavros/mavros/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/ftp.py)
-   [mavros/mavros/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py)
-   [mavros/mavros/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/system.py)

This page documents how MAVROS manages Flight Controller (FCU) parameters, which are configurable settings that control behavior such as PID gains, communication settings, and hardware configurations. MAVROS provides both command-line tools and programmatic interfaces for interacting with FCU parameters.

For mission planning and waypoint management, see [Mission Planning Tools](https://deepwiki.com/mavlink/mavros/5.1-mission-planning-tools). For file transfer capabilities, see [FTP and File Management](https://deepwiki.com/mavlink/mavros/5.3-ftp-and-file-management).

## Parameter Management Architecture

MAVROS implements parameter management through a dedicated plugin system that provides a bridge between ROS applications and the MAVLink parameter protocol.

```
MAVROS Parameter ManagementExternal SystemsMAVLink PARAM_* messagesFlight Controller UnitROS ApplicationsParameter Files(QGC, Mission Planner, MAVProxy)Parameter PluginParameter Dictionary(in-memory cache)ROS Services/mavros/param/*mavparam CLI ToolParameter File Handlers(ParamFile classes)
```

Sources: [mavros/mavros/cmd/param.py10-144](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L10-L144)

## Parameter Access Methods

MAVROS provides two primary methods for accessing parameters:

1.  **Command Line Interface**: The `mavparam` tool for parameter operations from the terminal
2.  **ROS Services**: Programmatic access through ROS service calls

### Parameter Flow Diagram

This diagram shows how parameter operations flow through the MAVROS system:

```
"ROS Application""Flight Controller""Parameter Plugin""mavparam CLI"User"ROS Application""Flight Controller""Parameter Plugin""mavparam CLI"UserParameter Load/Set FlowParameter Dump/Get Flowalt[Force pull]ROS Services Flowmavparam load file.paramUpdate parametersPARAM_SET messagesmavparam dump file.paramRequest pullPARAM_REQUEST_LISTPARAM_VALUE messagesReturn parameter valuesSave to fileCall get_param servicePARAM_REQUEST_READPARAM_VALUEReturn parameter valueCall set_param servicePARAM_SETPARAM_VALUE (confirmation)Return success/failure
```

Sources: [mavros/mavros/cmd/param.py83-144](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L83-L144)

## Parameter File Formats

MAVROS supports multiple parameter file formats through handler classes:

| Format | Class | Typical Extension | Description |
| --- | --- | --- | --- |
| Mission Planner | `MissionPlannerParam` | `.param` | Used by Mission Planner GCS |
| QGroundControl | `QGroundControlParam` | `.txt` | Used by QGroundControl GCS |
| MAVProxy | `MavProxyParam` | various | Used by MAVProxy command-line GCS |

The format is automatically detected based on file extension, but can be explicitly specified using command-line options.

Sources: [mavros/mavros/cmd/param.py25-79](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L25-L79)

## Command Line Interface

The `mavparam` command provides a comprehensive interface for parameter management:

### Loading Parameters

```
mavparam load [options] <file>
```

Options:

-   `-mp`, `--mission-planner`: Use Mission Planner format
-   `-qgc`, `--qgroundcontrol`: Use QGroundControl format
-   `-mpx`, `--mavproxy`: Use MAVProxy format

### Dumping Parameters

```
mavparam dump [options] <file>
```

Options:

-   `-mp`, `--mission-planner`: Use Mission Planner format
-   `-qgc`, `--qgroundcontrol`: Use QGroundControl format
-   `-mpx`, `--mavproxy`: Use MAVProxy format
-   `-f`, `--force`: Force pull parameters from FCU, update cache

### Getting Parameters

```
mavparam get <param_id>
```

Retrieves and displays a single parameter value.

### Setting Parameters

```
mavparam set <param_id> <value>
```

Sets a parameter to the specified value. The value type (int or float) is automatically determined.

Sources: [mavros/mavros/cmd/param.py82-144](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L82-L144)

## Parameter Implementation Details

### Parameter Value Storage

Parameters are stored in an in-memory dictionary (`ParamDict`) that serves as a cache between the FCU and ROS interfaces. This dictionary maps parameter names to their values and metadata.

### Parameter Type Detection

When setting parameters, MAVROS automatically detects the parameter type:

-   If the value contains a decimal point (`.`), it's treated as a float
-   Otherwise, it's treated as an integer

```
# From mavros/mavros/cmd/param.py
if "." in value:
    val = float(value)
else:
    val = int(value)
```

Sources: [mavros/mavros/cmd/param.py132-140](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L132-L140)

## ROS Service Interface

MAVROS exposes the following ROS services for parameter management:

| Service | Description |
| --- | --- |
| `/mavros/param/get` | Get a parameter value by ID |
| `/mavros/param/set` | Set a parameter value by ID |
| `/mavros/param/pull` | Pull all parameters from the FCU |
| `/mavros/param/push` | Push all parameters to the FCU |

### Example ROS Client Usage

```
# Get a parameter
from mavros_msgs.srv import ParamGet
param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
response = param_get(param_id='SYSID_MYGCS')
value = response.value.real  # or integer value

# Set a parameter
from mavros_msgs.srv import ParamSet
param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
response = param_set(param_id='SYSID_MYGCS', value=1)
```

## Common Use Cases

### Backing Up FCU Configuration

```
# Pull all parameters from the FCU and save to a file
mavparam dump --force backup.param
```

### Restoring FCU Configuration

```
# Load parameters from a backup file
mavparam load backup.param
```

### Modifying Multiple Parameters

```
# Pull current parameters
mavparam dump --force current.param

# Edit the file manually

# Load the modified parameters
mavparam load current.param
```

### Transferring Configuration Between FCUs

```
# Dump parameters from one FCU
mavparam dump --force config.param

# Connect to the second FCU and load parameters
mavparam load config.param
```

## Troubleshooting

### Parameters Not Taking Effect

If parameters are not taking effect on the FCU:

1.  Verify the FCU is properly connected to MAVROS
2.  Try forcing a parameter pull and push:
    
    ```
    mavparam dump --force temp.param
    mavparam load temp.param
    ```
    

### File Format Issues

If encountering issues with parameter files:

1.  Explicitly specify the file format:
    
    ```
    mavparam load --mission-planner my_params.txt
    ```
    
2.  Check for syntax errors in the parameter file
3.  Verify the file encoding (should be UTF-8)

Sources: [mavros/mavros/cmd/param.py82-144](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py#L82-L144)