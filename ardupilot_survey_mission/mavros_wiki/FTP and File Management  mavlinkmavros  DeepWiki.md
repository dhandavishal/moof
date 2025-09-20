Relevant source files

-   [mavros/mavros/cmd/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/ftp.py)
-   [mavros/mavros/cmd/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/mission.py)
-   [mavros/mavros/cmd/param.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/param.py)
-   [mavros/mavros/cmd/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/cmd/system.py)
-   [mavros/mavros/command.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/command.py)
-   [mavros/mavros/ftp.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/ftp.py)
-   [mavros/mavros/mission.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/mission.py)
-   [mavros/mavros/system.py](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/mavros/system.py)

## Overview

This document describes the File Transfer Protocol (FTP) capabilities in MAVROS, which enable file operations between a companion computer and an autopilot's filesystem. MAVROS implements the [MAVLink FTP Protocol](https://mavlink.io/en/services/ftp.html) to provide seamless file management functions between ROS-based systems and flight controllers.

The FTP system in MAVROS includes both a Python API and command-line utilities for performing operations such as:

-   Browsing the autopilot's filesystem
-   Uploading and downloading files
-   Creating and removing directories
-   Verifying file integrity

For information about mission planning and waypoint management, see [Mission Planning Tools](https://deepwiki.com/mavlink/mavros/5.1-mission-planning-tools). For information about parameter management, see [Parameter Management](https://deepwiki.com/mavlink/mavros/5.2-parameter-management).

Sources: [mavros/ftp.py1-233](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/ftp.py#L1-L233)

## Architecture

The FTP system in MAVROS is built around the following components:

```
Companion ComputerMAVLink FCUMAVROS FTP SystemROS ServicesMAVLink FTP ProtocolFTPPluginFTPFilemavftp Command Line ToolFCU FilesystemLocal FilesystemROS Applications
```

### Key Components

1.  **FTPPlugin**: Main class that interfaces with the MAVLink FTP protocol, providing high-level file operations
2.  **FTPFile**: File handle class for interacting with files on the flight controller
3.  **mavftp**: Command-line utility for performing FTP operations

Sources: [mavros/ftp.py142-233](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/ftp.py#L142-L233) [mavros/cmd/ftp.py1-299](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L1-L299)

## Python API

The MAVROS Python API provides two main classes for FTP operations:

### FTPPlugin

This class provides methods for file system operations and serves as the main interface to the MAVLink FTP protocol.

Key methods:

| Method | Description |
| --- | --- |
| `open(path, mode)` | Opens a file on the FCU |
| `listdir(dir_path)` | Lists files in a directory |
| `unlink(path)` | Removes a file |
| `mkdir(path)` | Creates a directory |
| `rmdir(path)` | Removes a directory |
| `rename(old_path, new_path)` | Renames a file |
| `checksum(path)` | Calculates CRC32 checksum of a file |
| `reset_server()` | Resets the FTP server on the FCU |

### FTPFile

This class represents a file handle for files on the flight controller and provides file I/O operations.

Key methods:

| Method | Description |
| --- | --- |
| `open(path, mode)` | Opens a file with specific mode |
| `close()` | Closes the file |
| `read(size)` | Reads data from the file |
| `write(bin_data)` | Writes data to the file |
| `tell()` | Returns current file position |
| `seek(offset, whence)` | Sets the file position |
| `truncate(size)` | Truncates the file to specified size |

```
FTPPlugin+open(path, mode) : : FTPFile+listdir(dir_path) : : List[FileEntry]+unlink(path)+mkdir(path)+rmdir(path)+rename(old_path, new_path)+checksum(path) : : int+reset_server()FTPFile-_fm: FTPPlugin-name: str-mode: str-size: int-offset: int+closed: bool+open(path, mode)+close()+read(size) : : bytearray+write(bin_data)+tell() : : int+seek(offset, whence)+truncate(size)
```

The supported file modes are:

-   `"w"` or `"wb"`: Write binary
-   `"r"` or `"rb"`: Read binary
-   `"cw"`: Create exclusive & write

Sources: [mavros/ftp.py38-140](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/ftp.py#L38-L140) [mavros/ftp.py142-233](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/ftp.py#L142-L233)

## File Transfer Optimizations

MAVROS uses optimized buffer sizes for file transfers to maximize throughput:

```
# Optimized transfer size for FTP message payload
FTP_PAGE_SIZE = 239 * 18 - 1
```

Note that the buffer size is slightly smaller than the theoretical maximum due to a bug in the FTP implementation that causes a doubling request of the last package.

Sources: [mavros/cmd/ftp.py25-26](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L25-L26)

MAVROS provides a comprehensive command-line tool called `mavftp` for file operations. This tool is part of the MAVROS command-line utilities.

```
File Transfer ProcessOpen file requestFile handleRead/Write requestsData/AcknowledgmentClose requestFTP ClientFCU FTP Servermavftp Commandsls - List filescd - Change directorycat - Display file contentsrm - Remove filemkdir - Create directoryrmdir - Remove directorydownload - Download fileupload - Upload fileverify - Verify file integrityreset - Reset FTP server
```

### Command Reference

| Command | Description | Example |
| --- | --- | --- |
| `ls [path]` | List files in directory | `mavftp ls /fs/microsd` |
| `cd [path]` | Change current directory | `mavftp cd /fs/microsd/log` |
| `cat <path>` | Display file contents | `mavftp cat /fs/microsd/params.bak` |
| `rm <path>` | Remove file | `mavftp rm /fs/microsd/old_log.txt` |
| `mkdir <path>` | Create directory | `mavftp mkdir /fs/microsd/new_dir` |
| `rmdir <path>` | Remove directory | `mavftp rmdir /fs/microsd/old_dir` |
| `download <src> [dest]` | Download file from FCU | `mavftp download /fs/microsd/log.bin ./local_log.bin` |
| `upload <src> [dest]` | Upload file to FCU | `mavftp upload ./firmware.px4 /fs/microsd/firmware.px4` |
| `verify <local> [remote]` | Verify file integrity | `mavftp verify ./local_file.txt /fs/microsd/remote_file.txt` |
| `reset` | Reset FTP server | `mavftp reset` |

Sources: [mavros/cmd/ftp.py61-65](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L61-L65) [mavros/cmd/ftp.py86-299](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L86-L299)

## File Transfer Process

The file transfer process in MAVROS involves several steps to ensure reliable data transfer with verification:

```
"Flight Controller""FTPPlugin""mavftp Tool""User/Application""Flight Controller""FTPPlugin""mavftp Tool""User/Application"loop[Transfer Data]alt[Verify enabled]Initiate download/uploadOpen fileFileOpen requestFile handleRead/Write requestFileRead/FileWriteData/AcknowledgmentData/StatusUpdate progress barCalculate local CRC32Close fileFileClose requestRequest checksumFileChecksum requestCRC32 valueCRC32 valueCompare checksumsVerification result
```

### Progress Tracking and Verification

MAVROS includes built-in progress tracking and file verification capabilities:

1.  **Progress Bar**: The `ProgressBar` class provides a visual indicator of transfer progress for large files.
    
2.  **File Verification**: After transfers, files can be verified using CRC32 checksums to ensure data integrity.
    

Sources: [mavros/cmd/ftp.py29-59](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L29-L59) [mavros/cmd/ftp.py175-299](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L175-L299)

## Path Resolution

The `mavftp` tool maintains persistent working directory state between commands using a temporary file:

```
FTP_PWD_FILE = pathlib.Path("/tmp/.mavftp_pwd")
```

Path resolution follows these rules:

1.  If the path starts with "/", it's treated as an absolute path
2.  Otherwise, it's treated as relative to the current working directory
3.  If no path is specified, the current working directory is used

Sources: [mavros/cmd/ftp.py26](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L26-L26) [mavros/cmd/ftp.py67-84](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L67-L84)

## Usage Examples

### Basic File Operations

```
# List files in root directory
mavftp ls /

# Change to a specific directory
mavftp cd /fs/microsd

# List files in current directory
mavftp ls

# Create a new directory
mavftp mkdir /fs/microsd/logs

# Remove a file
mavftp rm /fs/microsd/old_log.txt

# Show file contents
mavftp cat /fs/microsd/params.txt
```

### File Transfer

```
# Download a log file
mavftp download /fs/microsd/log123.bin ./log123.bin

# Upload a parameter file
mavftp upload ./params.bak /fs/microsd/params.bak

# Upload a file with verification disabled (faster)
mavftp upload --no-verify ./large_file.dat /fs/microsd/large_file.dat

# Verify a previously transferred file
mavftp verify ./local_copy.txt /fs/microsd/remote_copy.txt
```

Sources: [mavros/cmd/ftp.py86-299](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L86-L299)

## Troubleshooting

Common issues and solutions:

| Issue | Solution |
| --- | --- |
| Connection timeouts | Try resetting the FTP server with `mavftp reset` |
| File verification failures | Check disk space on FCU, try disabling verification and check manually |
| Slow transfers | Use larger block sizes or consider compression for large files |
| "File not found" errors | Verify path exists using `ls` command and check permissions |

For persistent issues, the FTP server on the FCU can be reset using:

```
mavftp reset
```

Sources: [mavros/cmd/ftp.py144-148](https://github.com/mavlink/mavros/blob/44fa2f90/mavros/cmd/ftp.py#L144-L148)